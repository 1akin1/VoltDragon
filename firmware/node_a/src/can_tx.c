/**
 * @file can_tx.c
 * @brief Node A -> Node B CAN messages at 50 Hz.
 */
#include "can_tx.h"

#include "can.h"
#include "can_msg.h"
#include "flashlog.h"
#include "imu.h"
#include "log.h"
#include "reset_info.h"

/* Outside Node B's acceptance range 0x100..0x10F. */
#define FOREIGN_ID          (0x300U)
#define RESET_COUNT_MAX     (255UL)

typedef enum
{
    SLOT_STATUS = 0,
    SLOT_ACCEL,
    SLOT_GYRO,
    SLOT_MAG,
    SLOT_COUNT
} slot_t;

static const uint16_t s_ids[SLOT_COUNT] = {
    CANMSG_ID_A_STATUS, CANMSG_ID_A_ACCEL, CANMSG_ID_A_GYRO, CANMSG_ID_A_MAG
};

typedef struct
{
    bool           ready;
    uint32_t       next_due_ms;
    slot_t         next_slot;
    uint8_t        seq[SLOT_COUNT];
    can_tx_fault_t fault;
    uint32_t       last_queued;
} can_tx_state_t;

static can_tx_state_t s_tx;

static void send(slot_t slot, const uint8_t *payload)
{
    can_frame_t frame;

    if (s_tx.fault == CAN_TX_FAULT_SKIP_SEQ)
    {
        s_tx.seq[slot]++;
    }
    canmsg_seal(&frame, s_ids[slot], payload, s_tx.seq[slot]);
    s_tx.seq[slot]++;

    if (s_tx.fault == CAN_TX_FAULT_BAD_CRC)
    {
        frame.data[CANMSG_CRC_BYTE] ^= 0xFFU;
    }
    if (s_tx.fault != CAN_TX_FAULT_NONE)
    {
        LOG_WARN("can: injected fault %u into frame 0x%03X", (unsigned int)s_tx.fault,
                 (unsigned int)frame.id);
        if (s_tx.fault == CAN_TX_FAULT_FOREIGN_ID)
        {
            can_frame_t foreign = frame;
            foreign.id = FOREIGN_ID;
            (void)can_send(&foreign);
        }
        s_tx.fault = CAN_TX_FAULT_NONE;
    }
    (void)can_send(&frame);
}

static void send_slot(slot_t slot, uint32_t now_ms)
{
    uint8_t payload[CANMSG_PAYLOAD_LEN];
    lsm9ds1_sample_t sample;
    const bool imu_valid = imu_latest(&sample);

    if (slot == SLOT_STATUS)
    {
        const uint32_t resets = reset_info_count();
        const canmsg_status_t status = {
            .flags = (uint8_t)((imu_valid ? CANMSG_STATUS_IMU_VALID : 0U) |
                               (flashlog_ok() ? CANMSG_STATUS_RECORDER_OK : 0U)),
            .reset_count = (uint8_t)((resets > RESET_COUNT_MAX) ? RESET_COUNT_MAX : resets),
            .uptime_ms = now_ms,
        };
        canmsg_encode_status(payload, &status);
    }
    else if (!imu_valid)
    {
        /* No valid sample: send nothing, so Node B sees the IMU data age. */
        return;
    }
    else if (slot == SLOT_ACCEL)
    {
        canmsg_encode_vec3(payload, sample.accel_mg, 1);
    }
    else if (slot == SLOT_GYRO)
    {
        canmsg_encode_vec3(payload, sample.gyro_mdps, CANMSG_GYRO_DIVISOR);
    }
    else
    {
        canmsg_encode_vec3(payload, sample.mag_mgauss, 1);
    }
    send(slot, payload);
}

bool can_tx_init(void)
{
    s_tx = (can_tx_state_t){ 0 };

    if (!can_init())
    {
        LOG_ERROR("can: controller did not start, running without CAN");
        return false;
    }
    s_tx.ready = true;
    LOG_INFO("can: 500 kbit/s, one frame every %lu ms, each message every %lu ms",
             CAN_TX_SLOT_MS, CAN_TX_PERIOD_MS);
    return true;
}

void can_tx_poll(uint32_t now_ms)
{
    if (!s_tx.ready)
    {
        return;
    }
    can_poll();

    /* Signed difference handles counter wrap-around. */
    if ((int32_t)(now_ms - s_tx.next_due_ms) < 0)
    {
        return;
    }
    s_tx.next_due_ms = now_ms + CAN_TX_SLOT_MS;
    send_slot(s_tx.next_slot, now_ms);
    s_tx.next_slot = (slot_t)(((uint32_t)s_tx.next_slot + 1U) % (uint32_t)SLOT_COUNT);
}

void can_tx_inject(can_tx_fault_t fault)
{
    s_tx.fault = fault;
}

void can_tx_report(void)
{
    if (!s_tx.ready)
    {
        return;
    }

    const can_stats_t stats = can_stats();
    LOG_INFO("can: tx %lu/s, dropped %lu, tec %u%s", stats.tx_queued - s_tx.last_queued,
             stats.tx_dropped, (unsigned int)stats.tx_errors, stats.bus_off ? ", BUS OFF" : "");
    s_tx.last_queued = stats.tx_queued;
}
