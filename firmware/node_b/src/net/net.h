/**
 * @file net.h
 * @brief Node B's IPv4 network: static addressing, telemetry and command sockets.
 *
 * Node B is 192.168.10.2/24 with MAC 02:00:00:56:44:02 (locally administered).
 * Telemetry goes to the subnet broadcast address on UDP port 5600, so any
 * ground station on the link receives it without configuration or ARP.
 * Operator commands arrive on UDP port 5601, one command per datagram, and
 * each reply is sent back to the sender.
 */
#ifndef NET_H
#define NET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NET_TELEMETRY_PORT  (5600U)
#define NET_COMMAND_PORT    (5601U)

/**
 * Handles one command datagram and writes the reply into @p reply (up to
 * @p reply_size bytes). Returns the reply length, 0 for no reply.
 */
typedef size_t (*net_command_handler_t)(const uint8_t *data, size_t len, char *reply,
                                        size_t reply_size);

/** Starts lwIP and the Ethernet interface. Returns false if the hardware is not usable. */
bool net_init(net_command_handler_t command_handler);

/** Handles received frames, the link state and lwIP timers. Call from the main loop. */
void net_poll(uint32_t now_ms);

/** True while the Ethernet link is up. */
bool net_link_up(void);

/** Broadcasts a telemetry datagram. Returns false if the link is down or no buffer is free. */
bool net_send_telemetry(const uint8_t *data, size_t len);

/** Logs the link state and frame counters. */
void net_report(void);

#endif /* NET_H */
