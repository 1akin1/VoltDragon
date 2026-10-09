/**
 * @file commands.h
 * @brief Node B operator command interface on USART3 (HLR-014).
 *
 * Commands (protocol in docs/command-interface.md):
 *
 *   PING              liveness check
 *   VERSION           firmware version
 *   STATUS            uptime_ms, reset_count, commands_accepted, commands_rejected
 *   TLM_RATE [hz]     query or set the telemetry rate, 10..50 Hz
 *   CAN               valid, rejected, lost, unknown_id, age_ms of Node A data ('-' if none)
 *   MODE <m> [OVERRIDE]  request flight mode MISSION, HOLD, RTH or LAND (HLR-005, HLR-006)
 *
 * Every intact command counts as ground station contact for the link-loss
 * check (HLR-003); the ground station sends PING once a second as a heartbeat.
 */
#ifndef COMMANDS_H
#define COMMANDS_H

#include <stddef.h>
#include <stdint.h>

#define COMMANDS_TLM_RATE_MIN_HZ        (10UL)
#define COMMANDS_TLM_RATE_MAX_HZ        (50UL)
#define COMMANDS_TLM_RATE_DEFAULT_HZ    (10UL)

/** Configures the command UART and starts interrupt-driven reception. */
void commands_init(void);

/** Handles every complete line received so far and sends the responses. Call from the main loop. */
void commands_poll(void);

/**
 * Handles one command received over UDP (one command per datagram) and writes
 * the response, without line ending, into @p reply. Returns its length.
 * Matches net_command_handler_t.
 */
size_t commands_handle_datagram(const uint8_t *data, size_t len, char *reply, size_t reply_size);

/** Logs receive errors (UART overruns, buffer overflows) if any occurred since the last report. */
void commands_report(void);

/** Telemetry rate selected by the operator. */
uint32_t commands_telemetry_rate_hz(void);

#endif /* COMMANDS_H */
