/**
 * @file main.c
 * @brief Node B - gateway (CAN -> UDP telemetry, UART and UDP command interface).
 *
 * Bare-metal super-loop: boots, receives Node A's CAN messages, sends UDP
 * telemetry to the ground station, serves operator commands on USART3 and
 * UDP, kicks the watchdog and prints a heartbeat with CAN, network and
 * telemetry reports once per second.
 */
#include <stdint.h>

#include "can_rx.h"
#include "commands.h"
#include "iwdg.h"
#include "log.h"
#include "net.h"
#include "node_boot.h"
#include "systick.h"
#include "telemetry.h"

#define NODE_B_WATCHDOG_MS  (500UL)
#define HEARTBEAT_PERIOD_MS (1000UL)

int main(void)
{
    node_boot("Node B", NODE_B_WATCHDOG_MS, BOARD_CONSOLE_PD5_PD6);
    commands_init();
    (void)can_rx_init();
    (void)net_init(commands_handle_datagram);

    uint32_t last_heartbeat_ms = systick_now_ms();
    uint32_t heartbeat_count = 0U;

    for (;;)
    {
        iwdg_kick();
        (void)node_debug_console_poll();

        const uint32_t now_ms = systick_now_ms();
        can_rx_poll(now_ms);
        commands_poll();
        net_poll(now_ms);
        telemetry_poll(now_ms);

        if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
        {
            last_heartbeat_ms += HEARTBEAT_PERIOD_MS;
            heartbeat_count++;
            LOG_INFO("heartbeat %lu", heartbeat_count);
            can_rx_report(now_ms);
            net_report();
            telemetry_report();
            commands_report();
        }

        /* Sleep until the next interrupt: the 1 ms SysTick or a received command byte. */
        __asm volatile("wfi");
    }
}
