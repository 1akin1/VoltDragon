/**
 * @file node_boot.h
 * @brief Boot sequence and debug console shared by all VoltDragon nodes.
 */
#ifndef NODE_BOOT_H
#define NODE_BOOT_H

#include <stdint.h>

/**
 * Brings up the board, console and time base, reports the reset cause, the
 * reset counter and any fault from the previous run, then starts the watchdog.
 *
 * @param node_name    Name printed in the boot banner.
 * @param watchdog_ms  Nominal watchdog timeout in milliseconds.
 */
void node_boot(const char *node_name, uint32_t watchdog_ms);

/**
 * Polls the console for single-key debug commands. Call from the main loop.
 *
 *   f  trigger a HardFault (undefined instruction)
 *   z  trigger a divide-by-zero fault
 *   s  trigger an invalid-state fault (branch to ARM state)
 *   w  hang the main loop so the watchdog resets the node
 *   r  request a software reset
 *   ?  print help
 */
void node_debug_console_poll(void);

#endif /* NODE_BOOT_H */
