/**
 * @file iwdg.h
 * @brief Independent watchdog driver (HLR-016).
 *
 * Once started, the IWDG cannot be stopped by software. It runs from the
 * 32 kHz LSI oscillator, whose real frequency varies between parts (17-47 kHz
 * on the STM32F407), so the timeout is only nominal.
 */
#ifndef IWDG_H
#define IWDG_H

#include <stdint.h>

/** Longest supported nominal timeout (prescaler /32, reload 4095). */
#define IWDG_MAX_TIMEOUT_MS (4095UL)

/** Starts the watchdog with a nominal timeout of @p timeout_ms (1..4095 ms). */
void iwdg_start(uint32_t timeout_ms);

/** Reloads the watchdog counter. */
void iwdg_kick(void);

#endif /* IWDG_H */
