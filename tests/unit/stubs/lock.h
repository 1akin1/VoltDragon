/**
 * @file lock.h
 * @brief Host test double for Node A's FreeRTOS lock (firmware/node_a/src/rtos/lock.h).
 *
 * The host tests run on one thread, so taking and giving a lock does nothing,
 * as on the target before the scheduler starts.
 */
#ifndef LOCK_H
#define LOCK_H

#include <stdbool.h>

typedef struct
{
    int unused;
} lock_t;

static inline void lock_init(lock_t *lock)
{
    lock->unused = 0;
}

static inline void lock_take(lock_t *lock)
{
    (void)lock;
}

static inline void lock_give(lock_t *lock)
{
    (void)lock;
}

#endif /* LOCK_H */
