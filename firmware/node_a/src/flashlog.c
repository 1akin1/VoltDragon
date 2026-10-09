/**
 * @file flashlog.c
 * @brief Flight data recorder on the MT25Q SPI flash.
 */
#include "flashlog.h"

#include <stddef.h>
#include <string.h>

#include "crc32.h"
#include "iwdg.h"
#include "log.h"
#include "lsm9ds1.h"
#include "mt25q.h"
#include "reset_info.h"
#include "systick.h"

#define RECORD_MAGIC        (0x5644U)
#define QUEUE_LEN           (8U)
/* An erased slot reads as all ones; checking the first word (magic, type, length) is enough. */
#define ERASED_WORD         (0xFFFFFFFFUL)
/* Longest wait in flashlog_dump() for a running operation; a 4 KiB erase takes up to 400 ms. */
#define DUMP_WAIT_MS        (450UL)

typedef struct
{
    uint16_t magic;
    uint8_t  type;
    uint8_t  length;
    uint32_t seq;           /**< Equal to the slot index. */
    uint32_t time_ms;
    uint8_t  payload[FLASHLOG_PAYLOAD_MAX];
    uint32_t crc;           /**< CRC-32 of all preceding bytes. */
} record_t;

_Static_assert(sizeof(record_t) == FLASHLOG_RECORD_SIZE, "record size");
_Static_assert((MT25Q_PAGE_SIZE % FLASHLOG_RECORD_SIZE) == 0U, "records must not cross pages");
_Static_assert(sizeof(lsm9ds1_sample_t) <= FLASHLOG_PAYLOAD_MAX, "IMU payload size");

typedef enum
{
    OP_IDLE = 0,
    OP_ERASING,
    OP_PROGRAMMING
} flash_op_t;

typedef struct
{
    bool       ready;
    bool       full_reported;
    flash_op_t op;
    uint32_t   slot_count;
    uint32_t   next_slot;       /**< First free slot; also the number of slots used. */
    uint32_t   erased_until;    /**< Address up to which the flash is known to be erased. */
    uint32_t   erase_addr;
    record_t   queue[QUEUE_LEN];
    uint32_t   queue_head;
    uint32_t   queue_count;
    uint32_t   written_since_report;
    uint32_t   dropped;
    uint32_t   errors;
    const char *last_error;
} flashlog_state_t;

static flashlog_state_t s_log;

static uint32_t slot_addr(uint32_t slot)
{
    return slot * FLASHLOG_RECORD_SIZE;
}

static uint32_t record_crc(const record_t *rec)
{
    return crc32((const uint8_t *)rec, offsetof(record_t, crc));
}

static bool record_valid(const record_t *rec, uint32_t slot)
{
    return (rec->magic == RECORD_MAGIC) && (rec->seq == slot) &&
           (rec->length <= FLASHLOG_PAYLOAD_MAX) && (rec->crc == record_crc(rec));
}

/** Binary search for the first erased slot; records are contiguous from slot 0. */
static bool find_end(uint32_t *end)
{
    uint32_t lo = 0U;
    uint32_t hi = s_log.slot_count;

    while (lo < hi)
    {
        const uint32_t mid = lo + ((hi - lo) / 2U);
        uint32_t word = 0U;

        if (mt25q_read(slot_addr(mid), (uint8_t *)&word, sizeof(word)) != MT25Q_OK)
        {
            return false;
        }
        if (word == ERASED_WORD)
        {
            hi = mid;
        }
        else
        {
            lo = mid + 1U;
        }
    }
    *end = lo;
    return true;
}

/* Errors are counted, not logged, so a failing flash cannot flood the console; see flashlog_report(). */
static void note_error(const char *what)
{
    s_log.errors++;
    s_log.last_error = what;
}

/** Completes the running operation if the flash has finished it. Returns true when idle. */
static bool complete_operation(void)
{
    if (s_log.op == OP_IDLE)
    {
        return true;
    }

    bool busy = true;
    if (mt25q_busy(&busy) != MT25Q_OK)
    {
        note_error("status read");
        return false;
    }
    if (busy)
    {
        return false;
    }

    if (s_log.op == OP_ERASING)
    {
        s_log.erased_until = s_log.erase_addr + MT25Q_SUBSECTOR_SIZE;
    }
    else
    {
        const record_t *rec = &s_log.queue[s_log.queue_head];
        record_t readback;

        /* Verify-after-write. On a mismatch the slot is spent and the record is retried in the next one. */
        if ((mt25q_read(slot_addr(s_log.next_slot), (uint8_t *)&readback, sizeof(readback)) ==
             MT25Q_OK) && (memcmp(&readback, rec, sizeof(readback)) == 0))
        {
            s_log.queue_head = (s_log.queue_head + 1U) % QUEUE_LEN;
            s_log.queue_count--;
            s_log.written_since_report++;
        }
        else
        {
            note_error("verify");
        }
        s_log.next_slot++;
    }
    s_log.op = OP_IDLE;
    return true;
}

static void start_next_operation(void)
{
    if (s_log.queue_count == 0U)
    {
        return;
    }
    if (s_log.next_slot >= s_log.slot_count)
    {
        s_log.dropped += s_log.queue_count;
        s_log.queue_count = 0U;
        if (!s_log.full_reported)
        {
            s_log.full_reported = true;
            LOG_WARN("flashlog: flash full after %lu records, recording stopped", s_log.next_slot);
        }
        return;
    }

    const uint32_t addr = slot_addr(s_log.next_slot);
    mt25q_status_t status;

    if (addr >= s_log.erased_until)
    {
        /* Erase each subsector just before its first record is written. */
        s_log.erase_addr = addr - (addr % MT25Q_SUBSECTOR_SIZE);
        status = mt25q_erase_subsector_start(s_log.erase_addr);
        if (status == MT25Q_OK)
        {
            s_log.op = OP_ERASING;
        }
        else
        {
            note_error("erase");
        }
        return;
    }

    record_t *rec = &s_log.queue[s_log.queue_head];
    rec->seq = s_log.next_slot;
    rec->crc = record_crc(rec);
    status = mt25q_program_start(addr, (const uint8_t *)rec, sizeof(*rec));
    if (status == MT25Q_OK)
    {
        s_log.op = OP_PROGRAMMING;
    }
    else
    {
        note_error("program");
    }
}

bool flashlog_init(void)
{
    uint32_t size_bytes = 0U;
    uint32_t end = 0U;

    s_log = (flashlog_state_t){ 0 };

    mt25q_status_t status = mt25q_init(&size_bytes);
    if (status == MT25Q_OK)
    {
        s_log.slot_count = size_bytes / FLASHLOG_RECORD_SIZE;
        if (!find_end(&end))
        {
            status = MT25Q_ERR_BUS;
        }
    }
    if (status != MT25Q_OK)
    {
        LOG_ERROR("flashlog: init failed (%s), running without recorder", mt25q_status_name(status));
        return false;
    }

    s_log.next_slot = end;
    /* Subsectors are erased whole before use, so the rest of a partly used one is still erased. */
    const uint32_t addr = slot_addr(end);
    const uint32_t offset = addr % MT25Q_SUBSECTOR_SIZE;
    s_log.erased_until = (offset == 0U) ? addr : (addr - offset + MT25Q_SUBSECTOR_SIZE);
    s_log.ready = true;

    LOG_INFO("flashlog: MT25Q %lu KiB, %lu records found, %lu free",
             size_bytes / 1024U, end, s_log.slot_count - end);
    return true;
}

bool flashlog_append(flashlog_type_t type, const void *payload, uint32_t len)
{
    if (!s_log.ready || (len > FLASHLOG_PAYLOAD_MAX) || (s_log.queue_count >= QUEUE_LEN) ||
        (s_log.next_slot >= s_log.slot_count))
    {
        s_log.dropped++;
        return false;
    }

    record_t *rec = &s_log.queue[(s_log.queue_head + s_log.queue_count) % QUEUE_LEN];
    (void)memset(rec, 0, sizeof(*rec));
    rec->magic = RECORD_MAGIC;
    rec->type = (uint8_t)type;
    rec->length = (uint8_t)len;
    rec->time_ms = systick_now_ms();
    (void)memcpy(rec->payload, payload, len);
    s_log.queue_count++;
    return true;
}

void flashlog_poll(void)
{
    if (s_log.ready && complete_operation())
    {
        start_next_operation();
    }
}

bool flashlog_ok(void)
{
    return s_log.ready && (s_log.next_slot < s_log.slot_count);
}

void flashlog_report(void)
{
    if (!s_log.ready)
    {
        return;
    }

    LOG_INFO("flashlog: %lu rec/s, total %lu, dropped %lu, errors %lu",
             s_log.written_since_report, s_log.next_slot, s_log.dropped, s_log.errors);
    if (s_log.last_error != 0)
    {
        LOG_WARN("flashlog: last error: %s", s_log.last_error);
        s_log.last_error = 0;
    }
    s_log.written_since_report = 0U;
}

static void print_record(const record_t *rec)
{
    const uint32_t seconds = rec->time_ms / 1000U;
    const uint32_t millis = rec->time_ms % 1000U;

    if ((rec->type == (uint8_t)FLASHLOG_TYPE_BOOT) && (rec->length == sizeof(flashlog_boot_t)))
    {
        flashlog_boot_t boot;
        (void)memcpy(&boot, rec->payload, sizeof(boot));
        LOG_INFO("log #%lu %lu.%03lu BOOT cause %s, count %lu", rec->seq, seconds, millis,
                 reset_info_cause_name((reset_cause_t)boot.reset_cause), boot.reset_count);
    }
    else if ((rec->type == (uint8_t)FLASHLOG_TYPE_IMU) && (rec->length == sizeof(lsm9ds1_sample_t)))
    {
        lsm9ds1_sample_t s;
        (void)memcpy(&s, rec->payload, sizeof(s));
        LOG_INFO("log #%lu %lu.%03lu IMU acc %ld %ld %ld mg, gyro %ld %ld %ld mdps, mag %ld %ld %ld mG",
                 rec->seq, seconds, millis, s.accel_mg[0], s.accel_mg[1], s.accel_mg[2],
                 s.gyro_mdps[0], s.gyro_mdps[1], s.gyro_mdps[2],
                 s.mag_mgauss[0], s.mag_mgauss[1], s.mag_mgauss[2]);
    }
    else
    {
        LOG_INFO("log #%lu %lu.%03lu type %u, %u bytes", rec->seq, seconds, millis,
                 (unsigned int)rec->type, (unsigned int)rec->length);
    }
}

void flashlog_dump(uint32_t count)
{
    if (!s_log.ready)
    {
        LOG_WARN("flashlog: not available");
        return;
    }

    /* The flash only answers status reads while an operation runs. */
    const uint32_t start_ms = systick_now_ms();
    while (!complete_operation())
    {
        /* Debug-only wait that can outlast the watchdog timeout on silicon. */
        iwdg_kick();
        if ((systick_now_ms() - start_ms) >= DUMP_WAIT_MS)
        {
            LOG_WARN("flashlog: flash busy, dump skipped");
            return;
        }
    }

    const uint32_t first = (s_log.next_slot > count) ? (s_log.next_slot - count) : 0U;
    LOG_INFO("flashlog: last %lu of %lu records", s_log.next_slot - first, s_log.next_slot);
    for (uint32_t slot = first; slot < s_log.next_slot; ++slot)
    {
        record_t rec;

        if (mt25q_read(slot_addr(slot), (uint8_t *)&rec, sizeof(rec)) != MT25Q_OK)
        {
            LOG_WARN("log #%lu read failed", slot);
        }
        else if (!record_valid(&rec, slot))
        {
            LOG_WARN("log #%lu corrupt", slot);
        }
        else
        {
            print_record(&rec);
        }
    }
}
