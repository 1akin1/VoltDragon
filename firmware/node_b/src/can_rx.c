/**
 * @file can_rx.c
 * @brief Node B reception of Node A's CAN messages.
 */
#include "can_rx.h"

#include <string.h>

#include "can.h"
#include "log.h"

#define NODE_A_FILTER_BANK  (0U)
#define NODE_A_MESSAGES     (4U)

typedef struct
{
    bool                 ready;
    bool                 have_status;
    can_rx_stats_t       stats;
    uint32_t             reported_valid;
    canmsg_seq_tracker_t seq[NODE_A_MESSAGES];
    can_rx_node_a_t      node_a;
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

static void store(const can_frame_t *frame)
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
        default:
            canmsg_decode_vec3(frame->data, s_rx.node_a.mag_mgauss, 1);
            break;
    }
}

bool can_rx_init(void)
{
    s_rx = (can_rx_state_t){ 0 };

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
            store(&frame);
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
}
