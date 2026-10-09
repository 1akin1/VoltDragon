/**
 * @file safety.c
 * @brief Node A's flight-mode and safety logic.
 */
#include "safety.h"

#include <math.h>

#include "ap_link.h"
#include "gps.h"
#include "lock.h"
#include "log.h"
#include "route.h"

#define DM_PER_M            (10.0f)
#define REPEAT_WINDOW_MS    (500UL)     /* a repeated request id within this is a duplicate frame */

typedef struct
{
    /* Inputs from CanTxTask, under s_input_lock. */
    bool              ground_report;
    canmsg_b_status_t ground;
    uint32_t          ground_rx_ms;
    bool              request_pending;
    canmsg_mode_req_t request;
} safety_inputs_t;

typedef struct
{
    safety_state_t state;           /* ControlTask's working copy */
    bool           ever_contact;
    ap_command_t   last_sent;
    bool           sent_once;
    uint32_t       last_send_ms;
    bool           have_request_id;
    uint32_t       last_request_ms;
} safety_internal_t;

static safety_inputs_t s_inputs;
static lock_t s_input_lock;
static safety_internal_t s_safety;
static safety_state_t s_published;
static lock_t s_state_lock;

void safety_init(void)
{
    s_inputs = (safety_inputs_t){ 0 };
    s_safety = (safety_internal_t){ 0 };
    s_safety.state.mode = FLIGHT_MODE_MISSION;
    s_safety.state.battery_pct = CANMSG_UNKNOWN_U8;
    s_published = s_safety.state;
    lock_init(&s_input_lock);
    lock_init(&s_state_lock);
    LOG_INFO("safety: mode MISSION; warning below %u m, return-to-home after %lu ms without "
             "ground contact or below %u %% battery, land below %u %%",
             (unsigned int)SAFETY_WARNING_DISTANCE_M, SAFETY_LINK_TIMEOUT_MS,
             SAFETY_BATTERY_LOW_PCT, SAFETY_BATTERY_CRITICAL_PCT);
}

void safety_on_ground_status(const canmsg_b_status_t *status, uint32_t now_ms)
{
    lock_take(&s_input_lock);
    s_inputs.ground = *status;
    s_inputs.ground_rx_ms = now_ms;
    s_inputs.ground_report = true;
    lock_give(&s_input_lock);
}

void safety_on_mode_request(const canmsg_mode_req_t *request)
{
    lock_take(&s_input_lock);
    s_inputs.request = *request;
    s_inputs.request_pending = true;
    lock_give(&s_input_lock);
}

/** Applies a mode change through the transition table; logs the outcome. */
static void change_mode(flight_mode_t requested, flight_cause_t cause, bool override)
{
    const flight_mode_t current = s_safety.state.mode;
    const flight_change_t result = flight_mode_check(current, requested, cause, override);

    if (result == FLIGHT_CHANGE_OK)
    {
        s_safety.state.mode = requested;
        LOG_WARN("safety: mode %s -> %s (%s%s)", flight_mode_name(current),
                 flight_mode_name(requested), flight_cause_name(cause),
                 override ? ", override" : "");
    }
    else if (result == FLIGHT_CHANGE_UNCHANGED)
    {
        /* Already there. */
    }
    else
    {
        s_safety.state.rejected_requests++;
        LOG_WARN("safety: rejected %s -> %s (%s%s): %s", flight_mode_name(current),
                 flight_mode_name(requested), flight_cause_name(cause),
                 override ? ", override" : "", flight_change_name(result));
    }
}

/** An automatic trigger: only ever towards a safer mode, so nothing to reject. */
static void trigger(flight_mode_t target, flight_cause_t cause)
{
    const flight_mode_t current = s_safety.state.mode;

    if ((current == FLIGHT_MODE_LAND) || (current == target))
    {
        return;
    }
    change_mode(target, cause, false);
}

static void update_distance(uint32_t now_ms)
{
    nmea_fix_t fix;
    safety_state_t *s = &s_safety.state;

    if (!gps_latest(&fix, now_ms))
    {
        /* Without a position there is no distance; the last decision stands. */
        s->have_distance = false;
        return;
    }
    s->distance_m = route_distance_to_conductors_m(fix.lat_e7, fix.lon_e7, fix.alt_msl_cm);
    s->have_distance = true;

    if (!s->proximity && (s->distance_m < SAFETY_WARNING_DISTANCE_M))
    {
        s->proximity = true;
        /* The fix age shows how long after the new position the warning was raised (HLR-002). */
        LOG_WARN("safety: PROXIMITY_WARNING, %u dm from a conductor (fix age %lu ms), moving away",
                 (unsigned int)(s->distance_m * DM_PER_M), gps_fix_age_ms(now_ms));
    }
    else if (s->proximity && (s->distance_m > SAFETY_CLEAR_DISTANCE_M))
    {
        s->proximity = false;
        LOG_INFO("safety: proximity warning cleared at %u dm",
                 (unsigned int)(s->distance_m * DM_PER_M));
    }
    else
    {
        /* No change. */
    }
}

static void update_ground_link(uint32_t now_ms, const safety_inputs_t *in)
{
    safety_state_t *s = &s_safety.state;

    if (!in->ground_report)
    {
        return;
    }
    if ((in->ground.flags & CANMSG_B_GROUND_CONTACT) != 0U)
    {
        s_safety.ever_contact = true;
    }
    /* Node B's silence adds to the link age: without Node B there is no ground link. */
    const uint32_t age = (uint32_t)in->ground.link_age_ms + (now_ms - in->ground_rx_ms);
    const bool lost = s_safety.ever_contact && (age > SAFETY_LINK_TIMEOUT_MS);

    if (lost && !s->link_lost)
    {
        LOG_WARN("safety: ground link lost for %lu ms", age);
        trigger(FLIGHT_MODE_RETURN_TO_HOME, FLIGHT_CAUSE_LINK_LOSS);
    }
    else if (!lost && s->link_lost)
    {
        LOG_INFO("safety: ground link back");
    }
    else
    {
        /* No change. */
    }
    s->link_lost = lost;
}

static void update_battery(uint32_t now_ms)
{
    safety_state_t *s = &s_safety.state;
    ap_status_t status;

    s->autopilot_ok = ap_link_status(&status, now_ms);
    if (!s->autopilot_ok)
    {
        return;
    }
    s->battery_pct = status.battery_pct;

    const bool critical = status.battery_pct < SAFETY_BATTERY_CRITICAL_PCT;
    const bool low = status.battery_pct < SAFETY_BATTERY_LOW_PCT;
    if (critical && !s->battery_critical)
    {
        LOG_WARN("safety: battery critical, %u %%", (unsigned int)status.battery_pct);
        trigger(FLIGHT_MODE_LAND, FLIGHT_CAUSE_BATTERY_CRITICAL);
    }
    else if (low && !s->battery_low)
    {
        LOG_WARN("safety: battery low, %u %%", (unsigned int)status.battery_pct);
        trigger(FLIGHT_MODE_RETURN_TO_HOME, FLIGHT_CAUSE_BATTERY_LOW);
    }
    else
    {
        /* No change. */
    }
    s->battery_critical = critical;
    s->battery_low = low;
}

static void handle_request(const safety_inputs_t *in, uint32_t now_ms)
{
    if (!in->request_pending)
    {
        return;
    }
    /*
     * Node B sends each request once; the id only guards against a frame delivered
     * twice. Later, the same id is a new request (Node B restarted).
     */
    if (s_safety.have_request_id && (in->request.request_id == s_safety.state.last_request_id) &&
        ((now_ms - s_safety.last_request_ms) < REPEAT_WINDOW_MS))
    {
        return;
    }
    s_safety.have_request_id = true;
    s_safety.state.last_request_id = in->request.request_id;
    s_safety.last_request_ms = now_ms;

    if (in->request.mode >= (uint8_t)FLIGHT_MODE_COUNT)
    {
        s_safety.state.rejected_requests++;
        LOG_WARN("safety: rejected request %u for unknown mode %u",
                 (unsigned int)in->request.request_id, (unsigned int)in->request.mode);
        return;
    }
    change_mode((flight_mode_t)in->request.mode, FLIGHT_CAUSE_OPERATOR, in->request.override != 0U);
}

static void command_autopilot(uint32_t now_ms)
{
    const safety_state_t *s = &s_safety.state;
    const ap_command_t cmd = {
        .mode = s->mode,
        .avoid = s->proximity,
        .min_distance_dm = s->proximity ? (uint16_t)SAFETY_AVOID_DISTANCE_DM : 0U,
    };
    const bool changed = !s_safety.sent_once || (cmd.mode != s_safety.last_sent.mode) ||
                         (cmd.avoid != s_safety.last_sent.avoid);

    /* A change goes out in this cycle; otherwise the command repeats every 100 ms. */
    if (changed || ((now_ms - s_safety.last_send_ms) >= SAFETY_COMMAND_PERIOD_MS))
    {
        ap_link_send(&cmd);
        s_safety.last_sent = cmd;
        s_safety.last_send_ms = now_ms;
        s_safety.sent_once = true;
    }
}

void safety_poll(uint32_t now_ms)
{
    lock_take(&s_input_lock);
    const safety_inputs_t inputs = s_inputs;
    s_inputs.request_pending = false;
    lock_give(&s_input_lock);

    ap_link_poll(now_ms);
    update_distance(now_ms);
    update_ground_link(now_ms, &inputs);
    update_battery(now_ms);
    handle_request(&inputs, now_ms);
    command_autopilot(now_ms);

    lock_take(&s_state_lock);
    s_published = s_safety.state;
    lock_give(&s_state_lock);
}

safety_state_t safety_state(void)
{
    lock_take(&s_state_lock);
    const safety_state_t state = s_published;
    lock_give(&s_state_lock);
    return state;
}

void safety_to_message(const safety_state_t *state, canmsg_safety_t *msg)
{
    const float dm = state->distance_m * DM_PER_M;

    msg->distance_dm = state->have_distance ?
                       (uint16_t)((dm > 65534.0f) ? 65534.0f : dm) : (uint16_t)CANMSG_UNKNOWN_U16;
    msg->battery_pct = state->autopilot_ok ? state->battery_pct : (uint8_t)CANMSG_UNKNOWN_U8;
    msg->mode = (uint8_t)state->mode;
    msg->last_request_id = state->last_request_id;
    msg->flags = (uint8_t)((state->proximity ? CANMSG_SAFETY_PROXIMITY : 0U) |
                           (state->proximity ? CANMSG_SAFETY_AVOIDING : 0U) |
                           (state->link_lost ? CANMSG_SAFETY_LINK_LOST : 0U) |
                           (state->battery_low ? CANMSG_SAFETY_BATTERY_LOW : 0U) |
                           (state->battery_critical ? CANMSG_SAFETY_BATTERY_CRIT : 0U) |
                           (state->autopilot_ok ? CANMSG_SAFETY_AUTOPILOT_OK : 0U));
}

void safety_report(void)
{
    const safety_state_t s = safety_state();

    LOG_INFO("safety: mode %s, distance %s%u dm, battery %u %%, autopilot %s, ground link %s, "
             "rejected requests %lu",
             flight_mode_name(s.mode), s.have_distance ? "" : "unknown/",
             s.have_distance ? (unsigned int)(s.distance_m * DM_PER_M) : 0U,
             (unsigned int)s.battery_pct, s.autopilot_ok ? "ok" : "silent",
             s.link_lost ? "LOST" : "ok", s.rejected_requests);
}
