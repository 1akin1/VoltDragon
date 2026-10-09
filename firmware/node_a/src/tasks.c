/**
 * @file tasks.c
 * @brief Node A's FreeRTOS tasks.
 */
#include "tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "can_tx.h"
#include "flashlog.h"
#include "gps.h"
#include "imu.h"
#include "iwdg.h"
#include "log.h"
#include "nav.h"
#include "node_boot.h"
#include "safety.h"
#include "systick.h"
#include "task.h"

#define PRIORITY_LOG        (1U)
#define PRIORITY_LOAD       (2U)
#define PRIORITY_CONTROL    (3U)
#define PRIORITY_IMU        (4U)
#define PRIORITY_CAN        (5U)

#define STACK_WORDS         (512U)

#define CAN_PERIOD_MS       (CAN_TX_SLOT_MS)
#define IMU_PERIOD_MS       (IMU_SAMPLE_PERIOD_MS)
#define CONTROL_PERIOD_MS   (20UL)
#define RECORD_EVERY        (5U)        /* control cycles: 10 Hz flight-data records */
#define LOG_PERIOD_MS       (10UL)
#define WATCHDOG_CHECK_MS   (100UL)
#define REPORT_PERIOD_MS    (1000UL)
#define STACK_REPORT_EVERY  (10U)       /* reports */
#define DUMP_RECORD_COUNT   (5UL)

/* Liveness: each periodic task sets its bit once per cycle. */
#define ALIVE_CAN           (1UL << 0)
#define ALIVE_IMU           (1UL << 1)
#define ALIVE_CONTROL       (1UL << 2)
#define ALIVE_ALL           (ALIVE_CAN | ALIVE_IMU | ALIVE_CONTROL)

typedef struct
{
    StaticTask_t tcb;
    StackType_t  stack[STACK_WORDS];
    TaskHandle_t handle;
} task_slot_t;

static task_slot_t s_can_task;
static task_slot_t s_imu_task;
static task_slot_t s_control_task;
static task_slot_t s_load_task;
static task_slot_t s_log_task;

static volatile uint32_t s_alive;

/* Priority-inversion demonstration (docs/rtos.md). */
#define DEMO_HOLD_MS        (5UL)       /* LogTask's "flash operation" under the recorder lock */
/*
 * LoadTask's CPU burst: five control periods. LogTask gets no CPU during it, so it
 * also delays the watchdog reload: the worst-case gap between reloads (two 100 ms
 * checks plus the burst) must stay well under the 500 ms timeout.
 */
#define DEMO_LOAD_MS        (100UL)
#define DEMO_SETTLE_MS      (300UL)     /* before the result is reported */
#define DEMO_DEADLINE_SLACK (5UL)       /* a control cycle this late counts as a missed deadline */

/* Written by ControlTask, read and reset by LogTask. */
static volatile uint32_t s_max_lock_wait_ms;
static volatile uint32_t s_deadline_misses;

/* LogTask only: a demo whose result is still to be reported. */
static bool s_demo_running;
static uint32_t s_demo_report_ms;

static void mark_alive(uint32_t bit)
{
    taskENTER_CRITICAL();
    s_alive |= bit;
    taskEXIT_CRITICAL();
}

static void note_lock_wait(uint32_t start_ms)
{
    const uint32_t waited = systick_now_ms() - start_ms;

    if (waited > s_max_lock_wait_ms)
    {
        s_max_lock_wait_ms = waited;
    }
}

static void record_flight_data(void)
{
    lsm9ds1_sample_t sample;

    if (imu_latest(&sample))
    {
        /* The append takes the recorder lock: measure how long that takes. */
        const uint32_t start = systick_now_ms();
        (void)flashlog_append(FLASHLOG_TYPE_IMU, &sample, sizeof(sample));
        note_lock_wait(start);
    }
}

static void check_recorder(void)
{
    /* Every control cycle: a recorder that cannot keep up would show a growing queue. */
    const uint32_t start = systick_now_ms();
    if (flashlog_pending() == FLASHLOG_QUEUE_LEN)
    {
        LOG_WARN("control: flight-data queue full, records will be dropped");
    }
    note_lock_wait(start);
}

static void can_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();

    (void)arg;
    for (;;)
    {
        can_tx_poll(systick_now_ms());
        mark_alive(ALIVE_CAN);
        (void)xTaskDelayUntil(&wake, pdMS_TO_TICKS(CAN_PERIOD_MS));
    }
}

static void imu_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();

    (void)arg;
    for (;;)
    {
        imu_sample();
        nav_poll(systick_now_ms());
        mark_alive(ALIVE_IMU);
        (void)xTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

static void control_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t cycle = 0U;

    (void)arg;
    for (;;)
    {
        /* Started late by more than the slack: this cycle missed its deadline. */
        if ((xTaskGetTickCount() - wake) > pdMS_TO_TICKS(DEMO_DEADLINE_SLACK))
        {
            s_deadline_misses = s_deadline_misses + 1U;
        }
        gps_poll(systick_now_ms());
        safety_poll(systick_now_ms());
        check_recorder();
        /* Every 100 ms, the first one 100 ms after start-up (after the boot record). */
        if ((cycle > 0U) && ((cycle % RECORD_EVERY) == 0U))
        {
            record_flight_data();
        }
        cycle++;
        mark_alive(ALIVE_CONTROL);
        (void)xTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void load_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        /* Idle until the priority-inversion demo asks for a CPU burst. */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint32_t start = systick_now_ms();
        while ((systick_now_ms() - start) < DEMO_LOAD_MS)
        {
            /* Busy: everything below priority 2 gets no CPU time. */
        }
    }
}

static void start_load(void)
{
    /* LoadTask outranks LogTask, so it starts at once, while LogTask holds the lock. */
    (void)xTaskNotifyGive(s_load_task.handle);
}

/**
 * Low-priority LogTask holds the recorder lock while medium-priority LoadTask
 * runs a CPU burst, and high-priority ControlTask needs the lock. Without
 * priority inheritance ControlTask waits for the whole burst; with it, LogTask
 * runs at ControlTask's priority until it releases the lock.
 */
static void priority_inversion_demo(bool inheritance)
{
    LOG_WARN("rtos: priority inversion demo, recorder lock is a %s",
             inheritance ? "mutex (priority inheritance)" : "binary semaphore (no inheritance)");
    flashlog_use_priority_inheritance(inheritance);
    s_max_lock_wait_ms = 0U;
    s_deadline_misses = 0U;

    flashlog_hold_lock(DEMO_HOLD_MS, start_load);

    /* LogTask goes on with its normal work (and the watchdog); the result follows later. */
    s_demo_running = true;
    s_demo_report_ms = systick_now_ms() + DEMO_SETTLE_MS;
}

static void finish_demo(uint32_t now_ms)
{
    if (!s_demo_running || ((int32_t)(now_ms - s_demo_report_ms) < 0))
    {
        return;
    }
    s_demo_running = false;
    flashlog_use_priority_inheritance(true);
    LOG_INFO("rtos: demo result: ControlTask waited up to %lu ms for the recorder lock, "
             "%lu control deadlines missed", s_max_lock_wait_ms, s_deadline_misses);
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
        case 'i':
        case 'm':
            if (!s_demo_running)
            {
                priority_inversion_demo(key == 'm');
            }
            break;
        case 'k':
            LOG_WARN("rtos: suspending ControlTask, expecting a watchdog reset");
            vTaskSuspend(s_control_task.handle);
            break;
        default:
            /* Unknown keys, line endings and NODE_KEY_NONE are ignored. */
            break;
    }
}

static void report_stacks(void)
{
    LOG_INFO("rtos: free stack words: can %lu, imu %lu, control %lu, log %lu",
             (uint32_t)uxTaskGetStackHighWaterMark(s_can_task.handle),
             (uint32_t)uxTaskGetStackHighWaterMark(s_imu_task.handle),
             (uint32_t)uxTaskGetStackHighWaterMark(s_control_task.handle),
             (uint32_t)uxTaskGetStackHighWaterMark(s_log_task.handle));
}

static void supervise_watchdog(void)
{
    taskENTER_CRITICAL();
    const uint32_t alive = s_alive;
    if (alive == ALIVE_ALL)
    {
        s_alive = 0U;
    }
    taskEXIT_CRITICAL();

    /* No reload unless every periodic task ran: a stalled task resets the node. */
    if (alive == ALIVE_ALL)
    {
        iwdg_kick();
    }
}

static void log_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t now_ms = systick_now_ms();
    uint32_t last_check_ms = now_ms;
    uint32_t last_report_ms = now_ms;
    uint32_t reports = 0U;

    (void)arg;
    for (;;)
    {
        handle_node_key(node_debug_console_poll());
        flashlog_poll();

        now_ms = systick_now_ms();
        finish_demo(now_ms);
        if ((now_ms - last_check_ms) >= WATCHDOG_CHECK_MS)
        {
            last_check_ms += WATCHDOG_CHECK_MS;
            supervise_watchdog();
        }
        if ((now_ms - last_report_ms) >= REPORT_PERIOD_MS)
        {
            last_report_ms += REPORT_PERIOD_MS;
            reports++;
            LOG_INFO("heartbeat %lu", reports);
            imu_report();
            gps_report(now_ms);
            nav_report();
            safety_report();
            can_tx_report();
            flashlog_report();
            if ((reports % STACK_REPORT_EVERY) == 0U)
            {
                report_stacks();
            }
        }
        (void)xTaskDelayUntil(&wake, pdMS_TO_TICKS(LOG_PERIOD_MS));
    }
}

static void create(task_slot_t *slot, TaskFunction_t fn, const char *name, UBaseType_t priority)
{
    slot->handle = xTaskCreateStatic(fn, name, STACK_WORDS, 0, priority, slot->stack, &slot->tcb);
}

void tasks_create(void)
{
    s_alive = 0U;
    create(&s_can_task, can_task, "CanTx", PRIORITY_CAN);
    create(&s_imu_task, imu_task, "Imu", PRIORITY_IMU);
    create(&s_control_task, control_task, "Control", PRIORITY_CONTROL);
    create(&s_load_task, load_task, "Load", PRIORITY_LOAD);
    create(&s_log_task, log_task, "Log", PRIORITY_LOG);
}

void vApplicationIdleHook(void)
{
    /* Sleep until the next interrupt; Renode skips the idle time this way. */
    __asm volatile("wfi");
}
