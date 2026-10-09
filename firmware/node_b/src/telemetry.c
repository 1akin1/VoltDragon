/**
 * @file telemetry.c
 * @brief Periodic UDP telemetry to the ground station.
 */
#include "telemetry.h"

#include <stdbool.h>

#include "can_rx.h"
#include "commands.h"
#include "log.h"
#include "net.h"
#include "reset_info.h"
#include "tlm_msg.h"

#define MS_PER_SECOND   (1000UL)
#define U8_MAX          (255UL)
#define U16_MAX         (65535UL)

typedef struct
{
    bool     started;
    uint32_t next_due_ms;
    uint32_t seq;
    uint32_t sent_since_report;
    uint32_t failed;
} telemetry_state_t;

static telemetry_state_t s_tlm;

static uint32_t saturate(uint32_t value, uint32_t max)
{
    return (value > max) ? max : value;
}

static void build(tlm_packet_t *p, uint32_t now_ms)
{
    can_rx_node_a_t a;
    const bool have_a = can_rx_node_a(&a);
    const can_rx_stats_t can = can_rx_stats();

    *p = (tlm_packet_t){ 0 };
    p->seq = s_tlm.seq;
    p->node_b_uptime_ms = now_ms;
    p->node_b_resets = (uint8_t)saturate(reset_info_count(), U8_MAX);
    p->can_valid = can.valid;
    p->can_rejected = can.rejected;
    p->can_lost = can.lost;

    if (!have_a)
    {
        p->node_a_age_ms = (uint16_t)U16_MAX;
        return;
    }

    const uint32_t age = now_ms - a.last_valid_ms;
    p->node_a_age_ms = (uint16_t)saturate(age, U16_MAX);
    p->node_a_uptime_ms = a.status.uptime_ms;
    p->node_a_resets = a.status.reset_count;
    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        p->accel_mg[axis] = a.accel_mg[axis];
        p->gyro_mdps[axis] = a.gyro_mdps[axis];
        p->mag_mgauss[axis] = a.mag_mgauss[axis];
    }
    if (age < TLM_FRESH_MS)
    {
        p->flags |= TLM_FLAG_NODE_A_FRESH;
    }
    if ((a.status.flags & CANMSG_STATUS_IMU_VALID) != 0U)
    {
        p->flags |= TLM_FLAG_IMU_VALID;
    }
    if ((a.status.flags & CANMSG_STATUS_RECORDER_OK) != 0U)
    {
        p->flags |= TLM_FLAG_RECORDER_OK;
    }
}

void telemetry_poll(uint32_t now_ms)
{
    const uint32_t period_ms = MS_PER_SECOND / commands_telemetry_rate_hz();

    if (!s_tlm.started)
    {
        s_tlm.started = true;
        s_tlm.next_due_ms = now_ms;
    }
    /* Signed difference handles counter wrap-around. */
    if ((int32_t)(now_ms - s_tlm.next_due_ms) < 0)
    {
        return;
    }
    /* Keep a steady rate, but do not send a burst after a stall or a rate change. */
    s_tlm.next_due_ms = ((now_ms - s_tlm.next_due_ms) >= period_ms) ? (now_ms + period_ms)
                                                                     : (s_tlm.next_due_ms + period_ms);

    if (!net_link_up())
    {
        return;
    }

    tlm_packet_t packet;
    uint8_t bytes[TLM_PACKET_LEN];
    build(&packet, now_ms);
    const size_t len = tlm_encode(&packet, bytes, sizeof(bytes));

    if (net_send_telemetry(bytes, len))
    {
        s_tlm.seq++;
        s_tlm.sent_since_report++;
    }
    else
    {
        s_tlm.failed++;
    }
}

void telemetry_report(void)
{
    LOG_INFO("tlm: %lu packets/s at %lu Hz, next seq %lu, failed %lu", s_tlm.sent_since_report,
             commands_telemetry_rate_hz(), s_tlm.seq, s_tlm.failed);
    s_tlm.sent_since_report = 0U;
}
