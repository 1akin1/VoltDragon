/**
 * @file lock.h
 * @brief Mutual exclusion for data shared between Node A's tasks.
 *
 * A lock is a FreeRTOS mutex (with priority inheritance) in static storage.
 * Before the scheduler starts there is only one thread of execution, so
 * taking and giving a lock then does nothing; start-up code can call the same
 * functions as the tasks.
 */
#ifndef LOCK_H
#define LOCK_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

typedef struct
{
    StaticSemaphore_t storage;
    SemaphoreHandle_t handle;
} lock_t;

static inline void lock_init(lock_t *lock)
{
    lock->handle = xSemaphoreCreateMutexStatic(&lock->storage);
}

/**
 * A binary semaphore used as a lock: mutual exclusion WITHOUT priority
 * inheritance. Only for demonstrating priority inversion; use lock_init().
 */
static inline void lock_init_without_inheritance(lock_t *lock)
{
    lock->handle = xSemaphoreCreateBinaryStatic(&lock->storage);
    (void)xSemaphoreGive(lock->handle);
}

static inline bool lock_active(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

static inline void lock_take(lock_t *lock)
{
    if (lock_active())
    {
        (void)xSemaphoreTake(lock->handle, portMAX_DELAY);
    }
}

static inline void lock_give(lock_t *lock)
{
    if (lock_active())
    {
        (void)xSemaphoreGive(lock->handle);
    }
}

#endif /* LOCK_H */
