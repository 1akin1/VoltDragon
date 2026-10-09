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
 *
 * Node B's operator command port is USART3 on PB10 (TX) / PB11 (RX), separate
 * from the debug console so the two cannot interfere.
 *
 * The inter-node bus is CAN1 on PB8 (RX) / PB9 (TX) at 500 kbit/s. Real hardware
 * needs a CAN transceiver and 120 ohm termination at both ends of the bus.
 *
 * The flight-data flash is on SPI1: PA5 (SCK), PA6 (MISO), PA7 (MOSI), with a
 * software-driven chip select on PA4 (active low). APB2 runs at 16 MHz, so the
 * SPI prescaler /2 gives an 8 MHz clock.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4_regs.h"

#define BOARD_SYSCLK_HZ     (16000000UL)
#define BOARD_PCLK1_HZ      (16000000UL)
#define BOARD_PCLK2_HZ      (16000000UL)
#define BOARD_CONSOLE_UART  (USART2)
#define BOARD_CONSOLE_BAUD  (115200UL)
#define BOARD_COMMAND_UART  (USART3)
#define BOARD_COMMAND_IRQN  (USART3_IRQN)
#define BOARD_COMMAND_BAUD  (115200UL)
#define BOARD_SENSOR_I2C    (I2C1)
#define BOARD_SENSOR_I2C_HZ (100000UL)
#define BOARD_CAN           (CAN1)
/* 16 MHz / 2 = 8 MHz time quantum; 1 + 13 + 2 = 16 tq per bit = 500 kbit/s, sample point 87.5 %. */
#define BOARD_CAN_PRESCALER (2U)
#define BOARD_CAN_BS1_TQ    (13U)
#define BOARD_CAN_BS2_TQ    (2U)
#define BOARD_FLASH_SPI     (SPI1)
/** SPI1 baud-rate field: 0 selects PCLK2 / 2. */
#define BOARD_FLASH_SPI_BR  (0U)

/** Enables peripheral clocks and configures pins for the console UART. */
void board_init(void);

/** Enables the clock and configures the pins of the operator command UART. */
void board_command_uart_init(void);

/** Enables the clock and configures the pins of the inter-node CAN bus. */
void board_can_init(void);

/** Enables the clock and configures the pins of the sensor I2C bus. */
void board_sensor_bus_init(void);

/** Enables the clock and configures the pins of the flash SPI bus; leaves the flash deselected. */
void board_flash_bus_init(void);

/** Drives the flash chip select: true pulls it low (selected), false releases it. */
void board_flash_select(bool selected);

#endif /* BOARD_H */
