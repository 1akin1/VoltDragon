/**
 * @file spi.h
 * @brief Polled, register-level SPI master driver (mode 0, 8-bit, MSB first).
 *
 * Chip select is not handled here: the caller drives it, so one bus can serve
 * several devices and a command can span several transfer calls.
 */
#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4_regs.h"

/** Configures master mode 0 with software slave management and enables the peripheral. */
void spi_init(spi_regs_t *spi, uint32_t baud_rate_field);

/**
 * Exchanges @p len bytes. Sends 0xFF when @p tx is null and discards the
 * received bytes when @p rx is null. Returns once the bus is idle.
 * @return false if a status wait timed out; the transfer is then incomplete.
 */
bool spi_transfer(spi_regs_t *spi, const uint8_t *tx, uint8_t *rx, size_t len);

#endif /* SPI_H */
