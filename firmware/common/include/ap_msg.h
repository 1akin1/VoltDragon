/**
 * @file ap_msg.h
 * @brief Messages between Node A and the autopilot (docs/autopilot-link.md).
 *
 * Node A is the mission and safety computer; the autopilot flies the vehicle.
 * Both directions use the NMEA-style framing of the command interface:
 *
 *   Node A -> autopilot  $VDCMD,<mode>,<avoid>,<min_distance_dm>*hh   guidance command
 *   autopilot -> Node A  $VDAPS,<battery_pct>,<mode>*hh               autopilot status
 *
 * <mode> is MISSION, HOLD, RTH or LAND; <avoid> is 1 while the autopilot must
 * keep at least <min_distance_dm> (0.1 m) from the conductors.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef AP_MSG_H
#define AP_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flight_mode.h"

#define AP_MSG_MAX_LEN (48U)        /**< Longest message including "\r\n" and NUL. */

typedef struct
{
    flight_mode_t mode;
    bool          avoid;
    uint16_t      min_distance_dm;
} ap_command_t;

typedef struct
{
    uint8_t       battery_pct;
    flight_mode_t mode;
} ap_status_t;

/** Formats a guidance command with "\r\n". Returns its length, or 0 if @p size is too small. */
size_t ap_format_command(const ap_command_t *cmd, char *out, size_t size);

/** Parses a status line (without line ending). Returns false if it is not a valid VDAPS. */
bool ap_parse_status(const char *line, ap_status_t *status);

/** Parses a command line (without line ending); used by tests and tools. */
bool ap_parse_command(const char *line, ap_command_t *cmd);

/** Short mode name used on the link: MISSION, HOLD, RTH or LAND. */
const char *ap_mode_name(flight_mode_t mode);

#endif /* AP_MSG_H */
