/**
 * @file can_rx.c
 * @brief Node B reception of Node A's CAN messages.
 */
#include "can_rx.h"

#include <string.h>

#include "can.h"
#include "log.h"

#define NODE_A_FILTER_BANK  (0U)
#define NODE_A_MESSAGES     (9U)
#define U16_MAX             (65535UL)

typedef struct
{
    bool                 ready;
    bool                 have_status;
    can_rx_stats_t       stats;
    uint32_t             reported_valid;
    canmsg_seq_tracker_t seq[NODE_A_MESSAGES];
    can_rx_node_a_t      node_a;
    bool                 ground_contact;
    uint32_t             ground_contact_ms;
    uint32_t             next_b_status_ms;
    uint8_t              b_status_seq;
    uint8_t              mode_req_seq;
    uint8_t              next_request_id;
    bool                 request_pending;   /* forwarded, not yet confirmed by Node A */
    uint8_t              pending_id;
    flight_mode_t        pending_mode;
    uint32_t             pending_ms;
} can_rx_state_t;

static can_rx_state_t s_rx;

/** Index of a Node A message in s_rx.seq, or NODE_A_MESSAGES if unknown. */
static uint32_t message_index(uint16_t id)
{
    switch (id)
    {
        case CANMSG_ID_A_STATUS:
            return 0U;
        case CANMSG_ID_A_ACCEL:
            return 1U;
        case CANMSG_ID_A_GYRO:
            return 2U;
        case CANMSG_ID_A_MAG:
            return 3U;
        case CANMSG_ID_A_GPS_LAT:
            return 4U;
        case CANMSG_ID_A_GPS_LON:
            return 5U;
        case CANMSG_ID_A_NAV:
            return 6U;
        case CANMSG_ID_A_SAFETY:
            return 7U;
        case CANMSG_ID_A_HEALTH:
            return 8U;
        default:
            return NODE_A_MESSAGES;
    }
}

/** Resynchronises all sequence trackers if this STATUS shows that Node A restarted. */
static void check_restart(const can_frame_t *frame)
{
    canmsg_status_t status;

    canmsg_decode_status(frame->data, &status);
    if (s_rx.have_status && ((status.uptime_ms < s_rx.node_a.status.uptime_ms) ||
                             (status.reset_count != s_rx.node_a.status.reset_count)))
    {
        (void)memset(s_rx.seq, 0, sizeof(s_rx.seq));
        s_rx.stats.restarts++;
        LOG_WARN("can: Node A restarted (reset count %u)", (unsigned int)status.reset_count);
    }
    s_rx.have_status = true;
}

/** Keeps Node A's vibration monitor state and logs each change of its alarm as it arrives. */
static void store_health(const can_frame_t *frame)
{
    canmsg_health_t health;

    canmsg_decode_health(frame->data, &health);
    const uint8_t previous = s_rx.node_a.any_health ? s_rx.node_a.health.alarm
                                                    : (uint8_t)CANMSG_VIB_NOMINAL;
    if (health.alarm != previous)
    {
        if (health.alarm == CANMSG_VIB_NOMINAL)
        {
            LOG_INFO("can: Node A vibration alarm cleared");
        }
        else
        {
            LOG_WARN("can: Node A VIBRATION FAULT: %s (fault score %u %%)",
                     canmsg_vib_class_name(health.alarm), (unsigned int)health.fault_score_pct);
        }
    }
    s_rx.node_a.health = health;
    s_rx.node_a.any_health = true;
}

static void store(const can_frame_t *frame, uint32_t now_ms)
{
    switch (frame->id)
    {
        case CANMSG_ID_A_STATUS:
            canmsg_decode_status(frame->data, &s_rx.node_a.status);
            break;
        case CANMSG_ID_A_ACCEL:
            canmsg_decode_vec3(frame->data, s_rx.node_a.accel_mg, 1);
            break;
        case CANMSG_ID_A_GYRO:
            canmsg_decode_vec3(frame->data, s_rx.node_a.gyro_mdps, CANMSG_GYRO_DIVISOR);
            break;
        case CANMSG_ID_A_MAG:
            canmsg_decode_vec3(frame->data, s_rx.node_a.mag_mgauss, 1);
            break;
        case CANMSG_ID_A_GPS_LAT:
            canmsg_decode_gps_lat(frame->data, &s_rx.node_a.gps);
            break;
        case CANMSG_ID_A_GPS_LON:
            /* GPS_LON follows GPS_LAT in the schedule, so it completes a position. */
            canmsg_decode_gps_lon(frame->data, &s_rx.node_a.gps);
            s_rx.node_a.gps_ms = now_ms;
            s_rx.node_a.any_gps = true;
            break;
        case CANMSG_ID_A_SAFETY:
            canmsg_decode_safety(frame->data, &s_rx.node_a.safety);
            s_rx.node_a.any_safety = true;
            break;
        case CANMSG_ID_A_HEALTH:
            store_health(frame);
            break;
        default:
            canmsg_decode_nav(frame->data, &s_rx.node_a.nav);
            break;
    }
}

void can_rx_ground_contact(uint32_t now_ms)
{
    s_rx.ground_contact = true;
    s_rx.ground_contact_ms = now_ms;
}

uint32_t can_rx_ground_link_age_ms(uint32_t now_ms)
{
    return s_rx.ground_contact ? (now_ms - s_rx.ground_contact_ms) : UINT32_MAX;
}

static void send_b_status(uint32_t now_ms)
{
    const uint32_t age = can_rx_ground_link_age_ms(now_ms);
    const canmsg_b_status_t status = {
        .link_age_ms = (uint16_t)((age > U16_MAX) ? U16_MAX : age),
        .flags = s_rx.ground_contact ? CANMSG_B_GROUND_CONTACT : 0U,
    };
    uint8_t payload[CANMSG_PAYLOAD_LEN];
    can_frame_t frame;

    canmsg_encode_b_status(payload, &status);
    canmsg_seal(&frame, CANMSG_ID_B_STATUS, payload, s_rx.b_status_seq);
    s_rx.b_status_seq++;
    (void)can_send(&frame);
}

uint8_t can_rx_request_mode(flight_mode_t mode, bool override, uint32_t now_ms)
{
    const canmsg_mode_req_t request = {
        .request_id = s_rx.next_request_id,
        .mode = (uint8_t)mode,
        .override = override ? 1U : 0U,
    };
    uint8_t payload[CANMSG_PAYLOAD_LEN];
    can_frame_t frame;

    /* 0 is never used: it is Node A's last request id before any request. */
    s_rx.next_request_id = (s_rx.next_request_id == UINT8_MAX) ? 1U
                                                               : (uint8_t)(s_rx.next_request_id + 1U);
    s_rx.request_pending = true;
    s_rx.pending_id = request.request_id;
    s_rx.pending_mode = mode;
    s_rx.pending_ms = now_ms;
    canmsg_encode_mode_req(payload, &request);
    canmsg_seal(&frame, CANMSG_ID_B_MODE_REQ, payload, s_rx.mode_req_seq);
    s_rx.mode_req_seq++;
    (void)can_send(&frame);
    return request.request_id;
}

bool can_rx_node_a_mode(uint32_t now_ms, flight_mode_t *mode)
{
    if (!s_rx.node_a.any_safety)
    {
        return false;
    }
    if (s_rx.request_pending && (s_rx.node_a.safety.last_request_id != s_rx.pending_id) &&
        ((now_ms - s_rx.pending_ms) < CAN_RX_REQUEST_TIMEOUT_MS))
    {
        *mode = s_rx.pending_mode;
        return true;
    }
    /* Node A has handled the request (or it was lost): its report is the truth again. */
    s_rx.request_pending = false;
    *mode = (flight_mode_t)s_rx.node_a.safety.mode;
    return true;
}

bool can_rx_init(void)
{
    s_rx = (can_rx_state_t){ 0 };
    s_rx.next_request_id = 1U;

    if (!can_init() ||
        !can_add_filter(NODE_A_FILTER_BANK, CANMSG_NODE_A_ID_BASE, CANMSG_NODE_A_ID_MASK))
    {
        LOG_ERROR("can: controller did not start, running without CAN");
        return false;
    }
    s_rx.ready = true;
    LOG_INFO("can: 500 kbit/s, accepting 0x%03X..0x%03X", CANMSG_NODE_A_ID_BASE,
             CANMSG_NODE_A_ID_BASE | (~CANMSG_NODE_A_ID_MASK & CAN_STD_ID_MAX));
    return true;
}

void can_rx_poll(uint32_t now_ms)
{
    can_frame_t frame;

    if (!s_rx.ready)
    {
        return;
    }

    can_poll();
    if ((int32_t)(now_ms - s_rx.next_b_status_ms) >= 0)
    {
        s_rx.next_b_status_ms = now_ms + CAN_RX_B_STATUS_PERIOD_MS;
        send_b_status(now_ms);
    }

    while (can_receive(&frame))
    {
        const uint32_t index = message_index(frame.id);

        if (index == NODE_A_MESSAGES)
        {
            s_rx.stats.unknown_id++;
        }
        else if (!canmsg_valid(&frame))
        {
            /* A corrupt frame's sequence number cannot be trusted, so it is not tracked. */
            s_rx.stats.rejected++;
        }
        else
        {
            if (frame.id == CANMSG_ID_A_STATUS)
            {
                check_restart(&frame);
            }
            s_rx.stats.lost += canmsg_track_seq(&s_rx.seq[index], canmsg_seq(&frame));
            s_rx.stats.valid++;
            store(&frame, now_ms);
            s_rx.node_a.last_valid_ms = now_ms;
            s_rx.node_a.any_valid = true;
        }
    }
}

can_rx_stats_t can_rx_stats(void)
{
    can_rx_stats_t stats = s_rx.stats;

    stats.overruns = can_stats().rx_overruns;
    return stats;
}

bool can_rx_node_a(can_rx_node_a_t *out)
{
    *out = s_rx.node_a;
    return s_rx.node_a.any_valid;
}

void can_rx_report(uint32_t now_ms)
{
    if (!s_rx.ready)
    {
        return;
    }

    const can_rx_stats_t stats = can_rx_stats();
    LOG_INFO("can: rx %lu/s, rejected %lu, lost %lu, unknown id %lu, overruns %lu",
             stats.valid - s_rx.reported_valid, stats.rejected, stats.lost, stats.unknown_id,
             stats.overruns);
    s_rx.reported_valid = stats.valid;

    const can_rx_node_a_t *a = &s_rx.node_a;
    if (!a->any_valid)
    {
        LOG_WARN("can: no data from Node A");
        return;
    }
    LOG_INFO("can: A age %lu ms, resets %u, imu %s, recorder %s, acc %ld %ld %ld mg, "
             "gyro %ld %ld %ld mdps, mag %ld %ld %ld mG",
             now_ms - a->last_valid_ms, (unsigned int)a->status.reset_count,
             ((a->status.flags & CANMSG_STATUS_IMU_VALID) != 0U) ? "ok" : "invalid",
             ((a->status.flags & CANMSG_STATUS_RECORDER_OK) != 0U) ? "ok" : "off",
             a->accel_mg[0], a->accel_mg[1], a->accel_mg[2],
             a->gyro_mdps[0], a->gyro_mdps[1], a->gyro_mdps[2],
             a->mag_mgauss[0], a->mag_mgauss[1], a->mag_mgauss[2]);

    static const char *const sources[] = { "none", "magnetometer", "GPS course", "?" };
    const uint32_t source = ((uint32_t)a->nav.flags & CANMSG_NAV_SOURCE_MASK) >>
                            CANMSG_NAV_SOURCE_SHIFT;
    LOG_INFO("can: A heading %u cdeg from %s, field %u mG, magnetometer %s, gps %s",
             (unsigned int)a->nav.heading_cdeg, sources[source],
             (unsigned int)a->nav.field_mgauss,
             ((a->nav.flags & CANMSG_NAV_MAG_OK) != 0U) ? "ok" : "DISTURBED",
             ((a->nav.flags & CANMSG_NAV_GPS_FIX) != 0U) ? "fix" : "no fix");

    if (a->any_safety)
    {
        const canmsg_safety_t *s = &a->safety;
        LOG_INFO("can: A mode %s, distance %u dm, battery %u %%, flags 0x%02X; "
                 "ground link age %lu ms",
                 flight_mode_name((flight_mode_t)s->mode), (unsigned int)s->distance_dm,
                 (unsigned int)s->battery_pct, (unsigned int)s->flags,
                 can_rx_ground_link_age_ms(now_ms));
    }
    if (a->any_health)
    {
        const canmsg_health_t *h = &a->health;
        LOG_INFO("can: A vibration monitor %s, alarm %s, last %s %u %%, fault score %u %%",
                 ((h->flags & CANMSG_HEALTH_ACTIVE) != 0U) ? "active" : "INACTIVE",
                 canmsg_vib_class_name(h->alarm), canmsg_vib_class_name(h->last_class),
                 (unsigned int)h->confidence_pct, (unsigned int)h->fault_score_pct);
    }
}
