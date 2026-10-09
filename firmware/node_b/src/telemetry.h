/**
 * @file telemetry.h
 * @brief Periodic UDP telemetry to the ground station (HLR-012, HLR-013).
 *
 * Sends one packet (tlm_msg.h, docs/telemetry.md) at the rate set by the
 * TLM_RATE command, 10 Hz by default. The sequence number advances only for
 * packets actually handed to the network, so a gap seen by the ground
 * station means a packet was lost on the way.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/** Sends a packet when one is due. Call from the main loop. */
void telemetry_poll(uint32_t now_ms);

/** Logs the packets sent in the last second and the send failures. */
void telemetry_report(void);

#endif /* TELEMETRY_H */
