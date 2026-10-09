/**
 * @file rtos_port.c
 * @brief Glue between FreeRTOS and the rest of Node A's firmware.
 *
 * - The kernel tick is driven from our SysTick interrupt (systick_hook()).
 * - Kernel assertions and stack overflows are logged and then reset the node,
 *   so a broken invariant always ends in a known state.
 * - Log lines from different tasks are serialised with a mutex.
 */
#include "FreeRTOS.h"
#include "log.h"
#include "reset_info.h"
#include "semphr.h"
#include "task.h"

void xPortSysTickHandler(void);
void vPortSetupTimerInterrupt(void);
void rtos_log_lock_init(void);

static StaticSemaphore_t s_log_mutex_storage;
static SemaphoreHandle_t s_log_mutex;

void systick_hook(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

void vPortSetupTimerInterrupt(void)
{
    /* Nothing to do: systick_init() already runs SysTick at configTICK_RATE_HZ. */
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, configSTACK_DEPTH_TYPE *size)
{
    static StaticTask_t s_idle_tcb;
    static StackType_t s_idle_stack[configMINIMAL_STACK_SIZE];

    *tcb = &s_idle_tcb;
    *stack = s_idle_stack;
    *size = configMINIMAL_STACK_SIZE;
}

void rtos_assert_failed(const char *file, int line)
{
    taskDISABLE_INTERRUPTS();
    LOG_ERROR("rtos: assertion failed at %s:%d", file, line);
    log_flush();
    reset_info_system_reset();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    taskDISABLE_INTERRUPTS();
    LOG_ERROR("rtos: stack overflow in task %s", name);
    log_flush();
    reset_info_system_reset();
}

static bool lock_usable(void)
{
    /* Not before the scheduler runs, and never from an interrupt or fault handler. */
    return (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && (xPortIsInsideInterrupt() == pdFALSE);
}

static void log_lock(void)
{
    if (lock_usable())
    {
        (void)xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    }
}

static void log_unlock(void)
{
    if (lock_usable())
    {
        (void)xSemaphoreGive(s_log_mutex);
    }
}

void rtos_log_lock_init(void)
{
    s_log_mutex = xSemaphoreCreateMutexStatic(&s_log_mutex_storage);
    log_set_lock(log_lock, log_unlock);
}
