/**
 * @file systick.h
 * @brief Millisecond time base using the Cortex-M SysTick timer.
 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * True once @p now_ms has reached @p due_ms, allowing for wrap-around of the
 * millisecond count: a due time up to about 24.8 days ahead is in the future.
 */
static inline bool systick_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (now_ms - due_ms) < 0x80000000UL;
}

/** Starts a 1 kHz SysTick interrupt from the core clock. */
void systick_init(uint32_t core_hz);

/** Milliseconds since systick_init(); wraps after about 49.7 days. */
uint32_t systick_now_ms(void);

/**
 * Microseconds since systick_init(), from the millisecond count and the
 * SysTick counter; wraps after about 71.6 minutes, so use it for intervals.
 */
uint32_t systick_now_us(void);

/** Busy-waits for at least @p ms milliseconds. */
void systick_delay_ms(uint32_t ms);

/**
 * Called from the SysTick interrupt after the millisecond count is updated.
 * The default does nothing; a node running an RTOS overrides it to drive the
 * RTOS tick from the same timer.
 */
void systick_hook(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H */
