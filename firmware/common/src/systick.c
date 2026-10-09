/**
 * @file systick.c
 * @brief Millisecond time base using the Cortex-M SysTick timer.
 */
#include "systick.h"

#include "stm32f4_regs.h"

#define TICK_RATE_HZ (1000UL)

static volatile uint32_t s_ticks_ms;

void SysTick_Handler(void);

void SysTick_Handler(void)
{
    s_ticks_ms = s_ticks_ms + 1U;
}

void systick_init(uint32_t core_hz)
{
    s_ticks_ms = 0U;
    SYSTICK->CTRL = 0U;
    SYSTICK->LOAD = (core_hz / TICK_RATE_HZ) - 1U;
    SYSTICK->VAL = 0U;
    SYSTICK->CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_ENABLE;
}

uint32_t systick_now_ms(void)
{
    return s_ticks_ms;
}

void systick_delay_ms(uint32_t ms)
{
    const uint32_t start = systick_now_ms();

    /* Unsigned subtraction handles counter wrap-around. */
    while ((systick_now_ms() - start) < ms)
    {
    }
}
