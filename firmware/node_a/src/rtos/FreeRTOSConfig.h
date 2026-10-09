/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS configuration for Node A (Cortex-M4F at 16 MHz).
 *
 * Static allocation only: there is no heap, so every task, stack and mutex is
 * reserved at link time and memory use is known before the first run.
 * Software timers, recursive mutexes and queue sets are not used.
 *
 * Interrupt priorities: the kernel runs at the lowest priority (15). Interrupts
 * at priority 5 or lower in urgency may call "FromISR" functions; the UART
 * receive interrupts stay at priority 0, never call the kernel, and so are
 * never delayed by it.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

void rtos_assert_failed(const char *file, int line);

/* Scheduler */
#define configUSE_PREEMPTION                        1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1
#define configUSE_TIME_SLICING                      1
#define configCPU_CLOCK_HZ                          (16000000UL)
#define configTICK_RATE_HZ                          (1000U)
#define configTICK_TYPE_WIDTH_IN_BITS               TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                        (6)
#define configMINIMAL_STACK_SIZE                    (128U)
#define configMAX_TASK_NAME_LEN                     (12)
#define configIDLE_SHOULD_YIELD                     1

/* The tick comes from our SysTick driver (systick.c), which is already set up. */
#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION   1

/* Memory: static allocation only. */
#define configSUPPORT_STATIC_ALLOCATION             1
#define configSUPPORT_DYNAMIC_ALLOCATION            0
/* The idle task's memory comes from rtos_port.c (the kernel's version would also
 * reserve a timer task stack, although software timers are not used). */
#define configKERNEL_PROVIDED_STATIC_MEMORY         0

/* Synchronisation */
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 0
#define configUSE_COUNTING_SEMAPHORES               0
#define configUSE_TASK_NOTIFICATIONS                1
#define configUSE_QUEUE_SETS                        0
#define configQUEUE_REGISTRY_SIZE                   0

/* Features not used */
#define configUSE_TIMERS                            0
#define configUSE_CO_ROUTINES                       0
#define configUSE_IDLE_HOOK                         1   /* sleeps with WFI */
#define configUSE_TICK_HOOK                         0
#define configUSE_NEWLIB_REENTRANT                  0
#define configUSE_MALLOC_FAILED_HOOK                0

/* Diagnostics */
#define configCHECK_FOR_STACK_OVERFLOW              2
#define configUSE_TRACE_FACILITY                    0
#define configASSERT(x)                             if ((x) == 0) { rtos_assert_failed(__FILE__, __LINE__); }

/* API functions included */
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskDelayUntil                     1
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1

/* Cortex-M interrupt priorities: 4 priority bits on the STM32F4. */
#define configPRIO_BITS                             4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY             (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* The port's handlers take the names used in the vector table (startup_stm32f4.c). */
#define vPortSVCHandler                             SVC_Handler
#define xPortPendSVHandler                          PendSV_Handler

#endif /* FREERTOS_CONFIG_H */
