/**
 * @file i2c.h
 * @brief Polled, register-level I2C master driver (7-bit addressing).
 *
 * Every wait is bounded, so a stuck bus or a missing device returns an error
 * instead of hanging the caller. Register reads follow the 1-, 2- and N-byte
 * receive sequences of RM0090 section 27.3.3.
 */
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#include "stm32f4_regs.h"

typedef enum
{
    I2C_OK = 0,
    I2C_ERR_NACK,       /**< The device did not acknowledge its address or data. */
    I2C_ERR_BUS,        /**< Bus error, lost arbitration or bus stuck busy. */
    I2C_ERR_TIMEOUT,    /**< An expected status flag never appeared. */
    I2C_ERR_ARG         /**< Invalid length or null buffer. */
} i2c_status_t;

/** Resets the peripheral and configures standard-mode timing (up to 100 kHz). */
void i2c_init(i2c_regs_t *i2c, uint32_t pclk_hz, uint32_t bus_hz);

/** Writes one byte to register @p reg of the device at 7-bit address @p addr. */
i2c_status_t i2c_write_reg(i2c_regs_t *i2c, uint8_t addr, uint8_t reg, uint8_t value);

/**
 * Reads @p len bytes starting at register @p reg. The register address is sent
 * as given; devices that need a flag for auto-increment must have it set by
 * the caller.
 */
i2c_status_t i2c_read_regs(i2c_regs_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint32_t len);

/** Short name of a status code, for logging. */
const char *i2c_status_name(i2c_status_t status);

#endif /* I2C_H */
