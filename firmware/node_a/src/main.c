/**
 * @file main.c
 * @brief Node A - sensor acquisition and flight control.
 *
 * Bare-metal super-loop: boots, reports the reset history, samples the IMU at
 * 100 Hz, sends sensor and status messages to Node B over CAN at 50 Hz,
 * records flight data to SPI flash at 10 Hz, kicks the watchdog and prints a
 * heartbeat with IMU, CAN and recorder reports once per second. FreeRTOS tasks
 * replace the loop in Phase 4.
 *
 * Node-specific debug keys:
 *   d  dump the last flight-log records
 *   c  send the next CAN frame with a wrong CRC
 *   g  skip one CAN sequence number (simulated frame loss)
 *   u  send one CAN frame with an identifier Node B does not accept
 */
#include <stdint.h>

#include "can_tx.h"
#include "flashlog.h"
#include "imu.h"
#include "iwdg.h"
#include "log.h"
#include "node_boot.h"
#include "reset_info.h"
#include "systick.h"

#define NODE_A_WATCHDOG_MS  (500UL)
#define HEARTBEAT_PERIOD_MS (1000UL)
#define RECORD_PERIOD_MS    (100UL)
#define DUMP_RECORD_COUNT   (5UL)

static void record_boot(void)
{
    const flashlog_boot_t boot = {
        .reset_cause = (uint32_t)reset_info_cause(),
        .reset_count = reset_info_count(),
    };

    (void)flashlog_append(FLASHLOG_TYPE_BOOT, &boot, sizeof(boot));
}

static void record_flight_data(void)
{
    lsm9ds1_sample_t sample;

    if (imu_latest(&sample))
    {
        (void)flashlog_append(FLASHLOG_TYPE_IMU, &sample, sizeof(sample));
    }
}

static void handle_node_key(char key)
{
    switch (key)
    {
        case 'd':
            flashlog_dump(DUMP_RECORD_COUNT);
            break;
        case 'c':
            can_tx_inject(CAN_TX_FAULT_BAD_CRC);
            break;
        case 'g':
            can_tx_inject(CAN_TX_FAULT_SKIP_SEQ);
            break;
        case 'u':
            can_tx_inject(CAN_TX_FAULT_FOREIGN_ID);
            break;
        default:
            /* Unknown keys, line endings and NODE_KEY_NONE are ignored. */
            break;
    }
}

int main(void)
{
    node_boot("Node A", NODE_A_WATCHDOG_MS);
    (void)imu_init();
    if (flashlog_init())
    {
        record_boot();
    }
    (void)can_tx_init();
    LOG_INFO("node A keys: d=dump flight log c=CAN bad CRC g=CAN seq gap u=CAN foreign id");

    uint32_t last_heartbeat_ms = systick_now_ms();
    uint32_t last_record_ms = last_heartbeat_ms;
    uint32_t heartbeat_count = 0U;

    for (;;)
    {
        iwdg_kick();
        handle_node_key(node_debug_console_poll());

        const uint32_t now_ms = systick_now_ms();
        imu_poll(now_ms);
        can_tx_poll(now_ms);
        flashlog_poll();

        if ((now_ms - last_record_ms) >= RECORD_PERIOD_MS)
        {
            last_record_ms += RECORD_PERIOD_MS;
            record_flight_data();
        }

        if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
        {
            last_heartbeat_ms += HEARTBEAT_PERIOD_MS;
            heartbeat_count++;
            LOG_INFO("heartbeat %lu", heartbeat_count);
            imu_report();
            can_tx_report();
            flashlog_report();
        }

        /* Sleep until the next interrupt (at the latest the 1 ms SysTick). */
        __asm volatile("wfi");
    }
}
