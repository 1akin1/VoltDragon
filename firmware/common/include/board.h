/**
 * @file board.h
 * @brief Board-level clock and pin configuration shared by both nodes.
 *
 * Both nodes run from the 16 MHz internal HSI oscillator (reset default, no
 * PLL). The debug console is USART2 on PA2 (TX) / PA3 (RX), which is the
 * ST-LINK virtual COM port on Nucleo boards.
 *
 * The sensor bus is I2C1 on PB6 (SCL) / PB7 (SDA), open-drain. The internal
 * pull-ups are enabled, but real hardware needs external pull-ups as well.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "stm32f4_regs.h"

#define BOARD_SYSCLK_HZ     (16000000UL)
#define BOARD_PCLK1_HZ      (16000000UL)
#define BOARD_CONSOLE_UART  (USART2)
#define BOARD_CONSOLE_BAUD  (115200UL)
#define BOARD_SENSOR_I2C    (I2C1)
#define BOARD_SENSOR_I2C_HZ (100000UL)

/** Enables peripheral clocks and configures pins for the console UART. */
void board_init(void);

/** Enables the clock and configures the pins of the sensor I2C bus. */
void board_sensor_bus_init(void);

#endif /* BOARD_H */
