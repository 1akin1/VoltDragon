/**
 * @file systick.c
 * @brief Millisecond time base using the Cortex-M SysTick timer.
 */
#include "systick.h"

#include "stm32f4_regs.h"

#define TICK_RATE_HZ (1000UL)

static volatile uint32_t s_ticks_ms;

void SysTick_Handler(void);

/* Default: nothing else runs on the tick. Node A overrides this to drive the RTOS tick. */
__attribute__((weak)) void systick_hook(void)
{
}

void SysTick_Handler(void)
{
    s_ticks_ms = s_ticks_ms + 1U;
    systick_hook();
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

uint32_t systick_now_us(void)
{
    uint32_t ms;
    uint32_t val;

    /* Re-read if a tick interrupt came between the two reads. */
    do
    {
        ms = s_ticks_ms;
        val = SYSTICK->VAL;
    } while (ms != s_ticks_ms);

    /* The counter runs down from LOAD to 0 once per millisecond. */
    const uint32_t period = SYSTICK->LOAD + 1U;
    return (ms * 1000U) + (((period - 1U - val) * 1000U) / period);
}

void systick_delay_ms(uint32_t ms)
{
    const uint32_t start = systick_now_ms();

    /* Unsigned subtraction handles counter wrap-around. */
    while ((systick_now_ms() - start) < ms)
    {
    }
}
