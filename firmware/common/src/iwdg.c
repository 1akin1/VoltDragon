/**
 * @file iwdg.c
 * @brief Independent watchdog driver (HLR-016).
 */
#include "iwdg.h"

#include "stm32f4_regs.h"

/* Prescaler /32 on a nominal 32 kHz LSI gives one count per millisecond. */
#define IWDG_PR_DIV32       (3UL)
#define IWDG_SR_BUSY_MASK   (0x3UL)
#define IWDG_SR_POLL_LIMIT  (100000UL)

void iwdg_start(uint32_t timeout_ms)
{
    uint32_t reload = timeout_ms;

    if (reload > IWDG_MAX_TIMEOUT_MS)
    {
        reload = IWDG_MAX_TIMEOUT_MS;
    }
    if (reload == 0U)
    {
        reload = 1U;
    }

    IWDG->KR = IWDG_KEY_START;
    IWDG->KR = IWDG_KEY_UNLOCK;
    IWDG->PR = IWDG_PR_DIV32;
    IWDG->RLR = reload;

    /* Wait (bounded) for the prescaler and reload updates to take effect. */
    for (uint32_t i = 0U; (i < IWDG_SR_POLL_LIMIT) && ((IWDG->SR & IWDG_SR_BUSY_MASK) != 0U); ++i)
    {
    }

    IWDG->KR = IWDG_KEY_RELOAD;
}

void iwdg_kick(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}
