/**
 * @file main.c
 * @brief Node A - sensor acquisition and flight control.
 *
 * Phase 1: bare-metal super-loop. Boots, reports the reset history, kicks the
 * watchdog and prints a heartbeat once per second. FreeRTOS tasks replace the
 * loop in Phase 4.
 */
#include <stdint.h>

#include "iwdg.h"
#include "log.h"
#include "node_boot.h"
#include "systick.h"

#define NODE_A_WATCHDOG_MS  (500UL)
#define HEARTBEAT_PERIOD_MS (1000UL)

int main(void)
{
    node_boot("Node A", NODE_A_WATCHDOG_MS);

    uint32_t last_heartbeat_ms = systick_now_ms();
    uint32_t heartbeat_count = 0U;

    for (;;)
    {
        iwdg_kick();
        node_debug_console_poll();

        const uint32_t now_ms = systick_now_ms();
        if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
        {
            last_heartbeat_ms += HEARTBEAT_PERIOD_MS;
            heartbeat_count++;
            LOG_INFO("heartbeat %lu", heartbeat_count);
        }

        /* Sleep until the next interrupt (at the latest the 1 ms SysTick). */
        __asm volatile("wfi");
    }
}
