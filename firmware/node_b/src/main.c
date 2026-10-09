/**
 * @file main.c
 * @brief Node B - gateway (CAN -> UDP telemetry, UART command interface).
 *
 * Bare-metal super-loop: boots, receives Node A's CAN messages, serves the
 * operator command interface on USART3, kicks the watchdog and prints a
 * heartbeat with a CAN report once per second. Networking arrives later in
 * Phase 2.
 */
#include <stdint.h>

#include "can_rx.h"
#include "commands.h"
#include "iwdg.h"
#include "log.h"
#include "node_boot.h"
#include "systick.h"

#define NODE_B_WATCHDOG_MS  (500UL)
#define HEARTBEAT_PERIOD_MS (1000UL)

int main(void)
{
    node_boot("Node B", NODE_B_WATCHDOG_MS);
    commands_init();
    (void)can_rx_init();

    uint32_t last_heartbeat_ms = systick_now_ms();
    uint32_t heartbeat_count = 0U;

    for (;;)
    {
        iwdg_kick();
        (void)node_debug_console_poll();
        const uint32_t now_ms = systick_now_ms();
        can_rx_poll(now_ms);
        commands_poll();

        if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
        {
            last_heartbeat_ms += HEARTBEAT_PERIOD_MS;
            heartbeat_count++;
            LOG_INFO("heartbeat %lu", heartbeat_count);
            can_rx_report(now_ms);
            commands_report();
        }

        /* Sleep until the next interrupt: the 1 ms SysTick or a received command byte. */
        __asm volatile("wfi");
    }
}
