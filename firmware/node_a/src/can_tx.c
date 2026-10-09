/**
 * @file can_tx.c
 * @brief Node A -> Node B CAN messages at 50 Hz.
 */
#include "can_tx.h"

#include "can.h"
#include "can_msg.h"
#include "flashlog.h"
#include "gps.h"
#include "imu.h"
#include "log.h"
#include "nav.h"
#include "reset_info.h"

/* Outside Node B's acceptance range 0x100..0x10F. */
#define FOREIGN_ID          (0x300U)
#define RESET_COUNT_MAX     (255UL)
#define ALT_DM_MAX          (32767L)
#define SPEED_DMPS_MAX      (255UL)
#define CM_PER_DM           (10L)

typedef enum
{
    MSG_STATUS = 0,
    MSG_ACCEL,
    MSG_GYRO,
    MSG_MAG,
    MSG_GPS_LAT,
    MSG_GPS_LON,
    MSG_NAV,
    MSG_COUNT,
    MSG_IDLE = MSG_COUNT
} message_t;

static const uint16_t s_ids[MSG_COUNT] = {
    CANMSG_ID_A_STATUS, CANMSG_ID_A_ACCEL, CANMSG_ID_A_GYRO, CANMSG_ID_A_MAG,
    CANMSG_ID_A_GPS_LAT, CANMSG_ID_A_GPS_LON, CANMSG_ID_A_NAV
};

/* One slot every CAN_TX_SLOT_MS; the schedule repeats every CAN_TX_PERIOD_MS. */
static const message_t s_schedule[CAN_TX_SLOTS] = {
    MSG_STATUS, MSG_ACCEL, MSG_GYRO, MSG_MAG, MSG_GPS_LAT, MSG_GPS_LON, MSG_NAV,
    MSG_IDLE, MSG_IDLE, MSG_IDLE
};

typedef struct
{
    bool           ready;
    uint32_t       next_due_ms;
    uint32_t       next_slot;
    uint8_t        seq[MSG_COUNT];
    can_tx_fault_t fault;
    uint32_t       last_queued;
} can_tx_state_t;

static can_tx_state_t s_tx;

static void send(message_t msg, const uint8_t *payload)
{
    can_frame_t frame;

    if (s_tx.fault == CAN_TX_FAULT_SKIP_SEQ)
    {
        s_tx.seq[msg]++;
    }
    canmsg_seal(&frame, s_ids[msg], payload, s_tx.seq[msg]);
    s_tx.seq[msg]++;

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

static void encode_status(uint8_t *payload, bool imu_valid, uint32_t now_ms)
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

static canmsg_gps_t gps_message(const nmea_fix_t *fix)
{
    const int32_t alt_dm = fix->alt_msl_cm / CM_PER_DM;
    const canmsg_gps_t gps = {
        .lat_e7 = fix->lat_e7,
        .lon_e7 = fix->lon_e7,
        .alt_msl_dm = (int16_t)((alt_dm > ALT_DM_MAX) ? ALT_DM_MAX
                                                      : ((alt_dm < -ALT_DM_MAX) ? -ALT_DM_MAX : alt_dm)),
        .quality = fix->quality,
        .satellites = fix->satellites,
    };
    return gps;
}

static void encode_nav(uint8_t *payload, bool gps_fix, const nmea_fix_t *fix)
{
    const nav_state_t state = nav_state();
    const uint32_t speed_dmps = gps_fix ? (fix->speed_cmps / (uint32_t)CM_PER_DM) : 0U;
    const canmsg_nav_t nav = {
        .heading_cdeg = state.heading_cdeg,
        .field_mgauss = state.field_mgauss,
        .flags = (uint8_t)((state.mag_ok ? CANMSG_NAV_MAG_OK : 0U) |
                           (gps_fix ? CANMSG_NAV_GPS_FIX : 0U) |
                           (((uint32_t)state.source << CANMSG_NAV_SOURCE_SHIFT) &
                            CANMSG_NAV_SOURCE_MASK)),
        .speed_dmps = (uint8_t)((speed_dmps > SPEED_DMPS_MAX) ? SPEED_DMPS_MAX : speed_dmps),
    };
    canmsg_encode_nav(payload, &nav);
}

static void send_message(message_t msg, uint32_t now_ms)
{
    uint8_t payload[CANMSG_PAYLOAD_LEN];
    lsm9ds1_sample_t sample;
    nmea_fix_t fix;
    const bool imu_valid = imu_latest(&sample);
    const bool gps_fix = gps_latest(&fix, now_ms);
    const canmsg_gps_t gps = gps_message(&fix);

    switch (msg)
    {
        case MSG_STATUS:
            encode_status(payload, imu_valid, now_ms);
            break;
        case MSG_NAV:
            encode_nav(payload, gps_fix, &fix);
            break;
        case MSG_ACCEL:
        case MSG_GYRO:
        case MSG_MAG:
            if (!imu_valid)
            {
                /* No valid sample: send nothing, so Node B sees the IMU data age. */
                return;
            }
            if (msg == MSG_ACCEL)
            {
                canmsg_encode_vec3(payload, sample.accel_mg, 1);
            }
            else if (msg == MSG_GYRO)
            {
                canmsg_encode_vec3(payload, sample.gyro_mdps, CANMSG_GYRO_DIVISOR);
            }
            else
            {
                canmsg_encode_vec3(payload, sample.mag_mgauss, 1);
            }
            break;
        case MSG_GPS_LAT:
        case MSG_GPS_LON:
            if (!gps_fix)
            {
                /* No fresh fix: send nothing, so Node B sees the position age. */
                return;
            }
            if (msg == MSG_GPS_LAT)
            {
                canmsg_encode_gps_lat(payload, &gps);
            }
            else
            {
                canmsg_encode_gps_lon(payload, &gps);
            }
            break;
        default:
            return;
    }
    send(msg, payload);
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
    LOG_INFO("can: 500 kbit/s, one slot every %lu ms, each message every %lu ms",
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
    send_message(s_schedule[s_tx.next_slot], now_ms);
    s_tx.next_slot = (s_tx.next_slot + 1U) % CAN_TX_SLOTS;
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
