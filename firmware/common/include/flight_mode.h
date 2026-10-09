/**
 * @file flight_mode.h
 * @brief Flight-mode state machine: the allowed transitions (HLR-003 to HLR-006).
 *
 * One table decides every mode change, so Node A (which owns the mode) and
 * Node B (which checks operator requests before forwarding them) always agree.
 *
 *   from \ to        MISSION     HOLD        RTH         LAND
 *   MISSION          -           operator    any         any
 *   HOLD             operator    -           any         any
 *   RTH              override    override    -           any
 *   LAND             no          no          no          -
 *
 * "any": an operator request or an automatic trigger (link loss, low battery).
 * "override": an operator request with the explicit override flag (HLR-006).
 * Automatic triggers only ever move towards a safer mode.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include <stdbool.h>

typedef enum
{
    FLIGHT_MODE_MISSION = 0,        /**< Flying the inspection plan. */
    FLIGHT_MODE_HOLD,               /**< Hovering in place, on operator request. */
    FLIGHT_MODE_RETURN_TO_HOME,     /**< Safety mode: flying back to the take-off point. */
    FLIGHT_MODE_LAND,               /**< Safety mode: descending to land; final. */
    FLIGHT_MODE_COUNT
} flight_mode_t;

typedef enum
{
    FLIGHT_CAUSE_OPERATOR = 0,      /**< Request from the ground station. */
    FLIGHT_CAUSE_LINK_LOSS,         /**< Ground link lost for more than 3 s (HLR-003). */
    FLIGHT_CAUSE_BATTERY_LOW,       /**< Battery below 20 % (HLR-004). */
    FLIGHT_CAUSE_BATTERY_CRITICAL,  /**< Battery below 10 % (HLR-004). */
    FLIGHT_CAUSE_COUNT
} flight_cause_t;

typedef enum
{
    FLIGHT_CHANGE_OK = 0,           /**< Allowed: the mode changes. */
    FLIGHT_CHANGE_UNCHANGED,        /**< Already in the requested mode. */
    FLIGHT_CHANGE_NEEDS_OVERRIDE,   /**< Leaving a safety mode needs the operator override. */
    FLIGHT_CHANGE_UNDEFINED         /**< Not a defined transition. */
} flight_change_t;

/** Decides whether @p current may change to @p requested for @p cause. */
flight_change_t flight_mode_check(flight_mode_t current, flight_mode_t requested,
                                  flight_cause_t cause, bool override);

/** True for RETURN_TO_HOME and LAND. */
bool flight_mode_is_safety(flight_mode_t mode);

/** Upper-case name, e.g. "RETURN_TO_HOME". */
const char *flight_mode_name(flight_mode_t mode);

/** Parses a name as returned by flight_mode_name(), or its short form (MISSION, HOLD, RTH, LAND). */
bool flight_mode_from_name(const char *name, flight_mode_t *mode);

const char *flight_cause_name(flight_cause_t cause);
const char *flight_change_name(flight_change_t change);

#endif /* FLIGHT_MODE_H */
