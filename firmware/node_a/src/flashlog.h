/**
 * @file flashlog.h
 * @brief Flight data recorder on the MT25Q SPI flash (HLR-018).
 *
 * The log is an append-only array of 64-byte records starting at address 0.
 * A record's sequence number equals its slot index, so the end of the log is
 * found at boot by a binary search for the first erased slot.
 *
 * flashlog_append() only queues a record in RAM. flashlog_poll(), called from
 * the main loop, erases each 4 KiB subsector just before its first use,
 * programs the queued records one at a time and reads each one back to verify
 * it. No call waits for the flash, so erase times never stall the loop.
 *
 * When the flash is full, further records are dropped and counted.
 *
 * Thread safety: ControlTask appends records and LogTask runs everything else;
 * all functions except flashlog_ok() take the recorder lock.
 */
#ifndef FLASHLOG_H
#define FLASHLOG_H

#include <stdbool.h>
#include <stdint.h>

#define FLASHLOG_RECORD_SIZE    (64U)
#define FLASHLOG_PAYLOAD_MAX    (48U)
#define FLASHLOG_QUEUE_LEN      (8U)    /**< Records waiting to be written. */

typedef enum
{
    FLASHLOG_TYPE_BOOT = 1,     /**< Payload: flashlog_boot_t */
    FLASHLOG_TYPE_IMU = 2,      /**< Payload: lsm9ds1_sample_t */
    FLASHLOG_TYPE_VIB_ALARM = 3 /**< Payload: flashlog_vib_alarm_t */
} flashlog_type_t;

/** Payload of a FLASHLOG_TYPE_BOOT record. */
typedef struct
{
    uint32_t reset_cause;       /**< reset_cause_t */
    uint32_t reset_count;
} flashlog_boot_t;

/** Payload of a FLASHLOG_TYPE_VIB_ALARM record: the vibration alarm changed (HLR-009). */
typedef struct
{
    uint8_t  alarm;             /**< vib_class_t: VIB_CLASS_NOMINAL when the alarm clears. */
    uint8_t  confidence_pct;    /**< Of the window that changed it. */
    uint16_t reserved;
    uint32_t windows;           /**< Windows classified so far. */
} flashlog_vib_alarm_t;

/** Brings up the flash and finds the end of the log. Returns false if the flash is unusable. */
bool flashlog_init(void);

/**
 * Queues a record stamped with the current time.
 * @return false if the record was dropped (no flash, queue full, log full or payload too big).
 */
bool flashlog_append(flashlog_type_t type, const void *payload, uint32_t len);

/** Advances the erase/program/verify state machine. Call from the main loop. */
void flashlog_poll(void);

/** True while the recorder is working: the flash was found and is not full. Lock-free. */
bool flashlog_ok(void);

/** Number of records queued but not yet written. */
uint32_t flashlog_pending(void);

/** Logs the records written since the previous report and the running totals. */
void flashlog_report(void);

/** Reads back and logs the last @p count records. */
void flashlog_dump(uint32_t count);

/**
 * Selects the recorder lock: a mutex with priority inheritance (true, the
 * default) or a binary semaphore without it (false). Only for demonstrating
 * priority inversion (docs/rtos.md).
 */
void flashlog_use_priority_inheritance(bool inherit);

/**
 * Takes the recorder lock, calls @p while_held (if not NULL), keeps the lock
 * busy for @p ms milliseconds, as a slow flash operation would, and releases it.
 * Only for demonstrating priority inversion.
 */
void flashlog_hold_lock(uint32_t ms, void (*while_held)(void));

#endif /* FLASHLOG_H */
