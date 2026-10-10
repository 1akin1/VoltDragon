/**
 * @file main.c
 * @brief Node A - sensor acquisition and flight control.
 *
 * Boots, reports the reset history, brings up the sensors, the vibration
 * classifier, the GPS receiver, the flight-data recorder and the CAN link,
 * then hands over to the FreeRTOS tasks (tasks.h) and never returns.
 *
 * Node-specific debug keys:
 *   d  dump the last flight-log records
 *   c  send the next CAN frame with a wrong CRC
 *   g  skip one CAN sequence number (simulated frame loss)
 *   u  send one CAN frame with an identifier Node B does not accept
 *   i  priority-inversion demo with a binary semaphore (no priority inheritance)
 *   m  the same demo with a mutex (priority inheritance)
 *   k  suspend ControlTask, so the task supervision lets the watchdog reset the node
 */
#include <stdint.h>

#include "FreeRTOS.h"
#include "ap_link.h"
#include "can_tx.h"
#include "flashlog.h"
#include "gps.h"
#include "health.h"
#include "imu.h"
#include "log.h"
#include "nav.h"
#include "node_boot.h"
#include "reset_info.h"
#include "safety.h"
#include "task.h"
#include "tasks.h"

#define NODE_A_WATCHDOG_MS  (500UL)

void rtos_log_lock_init(void);

static void record_boot(void)
{
    const flashlog_boot_t boot = {
        .reset_cause = (uint32_t)reset_info_cause(),
        .reset_count = reset_info_count(),
    };

    (void)flashlog_append(FLASHLOG_TYPE_BOOT, &boot, sizeof(boot));
}

int main(void)
{
    node_boot("Node A", NODE_A_WATCHDOG_MS, BOARD_CONSOLE_PA2_PA3);
    rtos_log_lock_init();
    (void)imu_init();
    (void)health_init();
    gps_init();
    nav_init();
    ap_link_init();
    safety_init();
    if (flashlog_init())
    {
        record_boot();
    }
    (void)can_tx_init();
    LOG_INFO("node A keys: d=dump flight log c=CAN bad CRC g=CAN seq gap u=CAN foreign id "
             "i=inversion demo (semaphore) m=inversion demo (mutex) k=stall ControlTask");

    tasks_create();
    LOG_INFO("rtos: FreeRTOS %s, starting the scheduler", tskKERNEL_VERSION_NUMBER);
    vTaskStartScheduler();

    /* Only reached if the scheduler could not start: the watchdog resets the node. */
    LOG_ERROR("rtos: scheduler did not start");
    for (;;)
    {
    }
}
