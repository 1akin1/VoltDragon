/**
 * @file systick.h
 * @brief Millisecond time base using the Cortex-M SysTick timer.
 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/** Starts a 1 kHz SysTick interrupt from the core clock. */
void systick_init(uint32_t core_hz);

/** Milliseconds since systick_init(); wraps after about 49.7 days. */
uint32_t systick_now_ms(void);

/** Busy-waits for at least @p ms milliseconds. */
void systick_delay_ms(uint32_t ms);

#endif /* SYSTICK_H */
