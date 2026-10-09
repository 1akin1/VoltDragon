/**
 * @file board.h
 * @brief Board-level clock and pin configuration shared by both nodes.
 *
 * Both nodes run from the 16 MHz internal HSI oscillator (reset default, no
 * PLL). The debug console is USART2 on PA2 (TX) / PA3 (RX), which is the
 * ST-LINK virtual COM port on Nucleo boards.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "stm32f4_regs.h"

#define BOARD_SYSCLK_HZ     (16000000UL)
#define BOARD_PCLK1_HZ      (16000000UL)
#define BOARD_CONSOLE_UART  (USART2)
#define BOARD_CONSOLE_BAUD  (115200UL)

/** Enables peripheral clocks and configures pins for the console UART. */
void board_init(void);

#endif /* BOARD_H */
