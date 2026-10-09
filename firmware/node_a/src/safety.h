/**
 * @file safety.h
 * @brief Node A's flight-mode and safety logic (HLR-001 to HLR-006).
 *
 * ControlTask runs safety_poll() every 20 ms:
 *
 * - Distance to the line: from the GPS fix and the route geometry (route.h).
 *   Below 12 m a PROXIMITY_WARNING is raised and the autopilot is ordered to
 *   keep 15 m from the conductors (HLR-001, HLR-002); the warning clears above
 *   15 m. The order goes out in the same control cycle as the new distance.
 * - Ground link: once a ground station has been heard, more than 3 s without
 *   contact triggers RETURN_TO_HOME (HLR-003). Node B reports the link age over
 *   CAN; if Node B itself falls silent, its silence counts as link age too.
 * - Battery (from the autopilot): below 20 % triggers RETURN_TO_HOME, below
 *   10 % LAND (HLR-004). Triggers fire when the condition starts.
 * - Operator mode requests (via Node B) are applied through the transition
 *   table in flight_mode.h; rejected requests are logged and counted (HLR-005,
 *   HLR-006).
 *
 * The resulting mode and the avoidance order are sent to the autopilot at
 * least every 100 ms, and at once when they change.
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "can_msg.h"
#include "flight_mode.h"

#define SAFETY_WARNING_DISTANCE_M   (12.0f)
#define SAFETY_CLEAR_DISTANCE_M     (15.0f)
#define SAFETY_AVOID_DISTANCE_DM    (150U)      /* keep 15 m while avoiding */
#define SAFETY_LINK_TIMEOUT_MS      (3000UL)
#define SAFETY_BATTERY_LOW_PCT      (20U)
#define SAFETY_BATTERY_CRITICAL_PCT (10U)
#define SAFETY_COMMAND_PERIOD_MS    (100UL)

typedef struct
{
    flight_mode_t mode;
    bool          proximity;            /* closer than 12 m (with hysteresis) */
    bool          have_distance;
    float         distance_m;
    bool          link_lost;
    bool          battery_low;
    bool          battery_critical;
    bool          autopilot_ok;
    uint8_t       battery_pct;
    uint8_t       last_request_id;
    uint32_t      rejected_requests;
} safety_state_t;

void safety_init(void);

/** Runs one cycle of the safety logic. ControlTask, every 20 ms. */
void safety_poll(uint32_t now_ms);

/** Records Node B's ground-link report. CanTxTask, on each B_STATUS frame. */
void safety_on_ground_status(const canmsg_b_status_t *status, uint32_t now_ms);

/** Queues an operator mode request. CanTxTask, on each B_MODE_REQ frame. */
void safety_on_mode_request(const canmsg_mode_req_t *request);

/** Snapshot of the current state; any task. */
safety_state_t safety_state(void);

/** The state as carried by the CAN SAFETY message. */
void safety_to_message(const safety_state_t *state, canmsg_safety_t *msg);

/** Logs the mode, distance, battery and link state. */
void safety_report(void);

#endif /* SAFETY_H */
