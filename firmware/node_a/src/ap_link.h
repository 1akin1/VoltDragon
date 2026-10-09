/**
 * @file ap_link.h
 * @brief Node A's link to the autopilot on UART5 (docs/autopilot-link.md).
 *
 * Receives the autopilot's status (battery, current mode) and sends guidance
 * commands. Used by ControlTask only.
 */
#ifndef AP_LINK_H
#define AP_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "ap_msg.h"

#define AP_LINK_STALE_MS (1000UL)

/** Configures UART5 and starts interrupt-driven reception. */
void ap_link_init(void);

/** Parses every status line received so far. */
void ap_link_poll(uint32_t now_ms);

/** Latest autopilot status. Returns false if none arrived within AP_LINK_STALE_MS. */
bool ap_link_status(ap_status_t *out, uint32_t now_ms);

/** Sends a guidance command. */
void ap_link_send(const ap_command_t *cmd);

/** Status lines received and rejected since start-up. */
uint32_t ap_link_received(void);
uint32_t ap_link_rejected(void);

#endif /* AP_LINK_H */
