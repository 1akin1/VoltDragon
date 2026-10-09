/**
 * @file i2c.c
 * @brief Polled, register-level I2C master driver.
 */
#include "i2c.h"

#include <stdbool.h>

/* Upper bound on status polls; well over 10 ms at 16 MHz, about 100 byte times at 100 kHz. */
#define I2C_POLL_LIMIT      (20000UL)
#define I2C_STANDARD_MAX_HZ (100000UL)
#define I2C_ERROR_FLAGS     (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF)
#define HZ_PER_MHZ          (1000000UL)

static i2c_status_t wait_sr1(i2c_regs_t *i2c, uint32_t flag)
{
    for (uint32_t i = 0U; i < I2C_POLL_LIMIT; ++i)
    {
        const uint32_t sr1 = i2c->SR1;

        if ((sr1 & I2C_ERROR_FLAGS) != 0U)
        {
            /* Error flags are rc_w0: writing 0 clears them, writing 1 has no effect. */
            i2c->SR1 = ~(sr1 & I2C_ERROR_FLAGS);
            return ((sr1 & I2C_SR1_AF) != 0U) ? I2C_ERR_NACK : I2C_ERR_BUS;
        }
        if ((sr1 & flag) != 0U)
        {
            return I2C_OK;
        }
    }
    return I2C_ERR_TIMEOUT;
}

static i2c_status_t wait_bus_idle(const i2c_regs_t *i2c)
{
    for (uint32_t i = 0U; i < I2C_POLL_LIMIT; ++i)
    {
        if ((i2c->SR2 & I2C_SR2_BUSY) == 0U)
        {
            return I2C_OK;
        }
    }
    return I2C_ERR_BUS;
}

/** Generates a (repeated) START and sends the address byte; returns once ADDR is set. */
static i2c_status_t send_address(i2c_regs_t *i2c, uint8_t addr, bool read)
{
    i2c->CR1 |= I2C_CR1_START;
    i2c_status_t status = wait_sr1(i2c, I2C_SR1_SB);
    if (status == I2C_OK)
    {
        /* Reading SR1 (in wait_sr1) then writing DR clears SB. */
        i2c->DR = ((uint32_t)addr << 1) | (read ? 1UL : 0UL);
        status = wait_sr1(i2c, I2C_SR1_ADDR);
    }
    return status;
}

static void clear_addr(const i2c_regs_t *i2c)
{
    (void)i2c->SR1;
    (void)i2c->SR2;
}

/** Releases the bus after a failed transfer and restores the default ACK settings. */
static i2c_status_t abort_transfer(i2c_regs_t *i2c, i2c_status_t status)
{
    i2c->CR1 = (i2c->CR1 & ~(I2C_CR1_POS | I2C_CR1_ACK)) | I2C_CR1_STOP;
    return status;
}

static i2c_status_t send_register(i2c_regs_t *i2c, uint8_t addr, uint8_t reg)
{
    i2c_status_t status = wait_bus_idle(i2c);
    if (status == I2C_OK)
    {
        status = send_address(i2c, addr, false);
    }
    if (status == I2C_OK)
    {
        clear_addr(i2c);
        i2c->DR = reg;
        status = wait_sr1(i2c, I2C_SR1_TXE);
    }
    return status;
}

void i2c_init(i2c_regs_t *i2c, uint32_t pclk_hz, uint32_t bus_hz)
{
    const uint32_t freq_mhz = pclk_hz / HZ_PER_MHZ;
    const uint32_t hz = (bus_hz < I2C_STANDARD_MAX_HZ) ? bus_hz : I2C_STANDARD_MAX_HZ;

    /* A software reset releases a bus left busy by an interrupted transfer. */
    i2c->CR1 = I2C_CR1_SWRST;
    i2c->CR1 = 0U;

    i2c->CR2 = freq_mhz;
    /* Standard mode, 50 % duty cycle: T_high = T_low = CCR * T_pclk. */
    i2c->CCR = pclk_hz / (2U * hz);
    /* Maximum SCL rise time in standard mode is 1000 ns, i.e. FREQ + 1. */
    i2c->TRISE = freq_mhz + 1U;
    i2c->CR1 = I2C_CR1_PE;
}

i2c_status_t i2c_write_reg(i2c_regs_t *i2c, uint8_t addr, uint8_t reg, uint8_t value)
{
    i2c_status_t status = send_register(i2c, addr, reg);
    if (status == I2C_OK)
    {
        i2c->DR = value;
        status = wait_sr1(i2c, I2C_SR1_BTF);
    }
    if (status != I2C_OK)
    {
        return abort_transfer(i2c, status);
    }
    i2c->CR1 |= I2C_CR1_STOP;
    return I2C_OK;
}

/* Single byte: NACK it before ADDR is cleared, then STOP. */
static i2c_status_t receive_one(i2c_regs_t *i2c, uint8_t *buf)
{
    i2c->CR1 &= ~I2C_CR1_ACK;
    clear_addr(i2c);
    i2c->CR1 |= I2C_CR1_STOP;

    const i2c_status_t status = wait_sr1(i2c, I2C_SR1_RXNE);
    if (status == I2C_OK)
    {
        buf[0] = (uint8_t)i2c->DR;
    }
    return status;
}

/* Two bytes: POS makes the NACK apply to the second byte. POS and ACK were set before ADDR. */
static i2c_status_t receive_two(i2c_regs_t *i2c, uint8_t *buf)
{
    clear_addr(i2c);
    i2c->CR1 &= ~I2C_CR1_ACK;

    const i2c_status_t status = wait_sr1(i2c, I2C_SR1_BTF);
    if (status == I2C_OK)
    {
        i2c->CR1 |= I2C_CR1_STOP;
        buf[0] = (uint8_t)i2c->DR;
        buf[1] = (uint8_t)i2c->DR;
    }
    i2c->CR1 &= ~I2C_CR1_POS;
    return status;
}

/* Three or more bytes: ACK all but the last, NACK the last using BTF to keep the timing safe. */
static i2c_status_t receive_many(i2c_regs_t *i2c, uint8_t *buf, uint32_t len)
{
    uint32_t i = 0U;

    clear_addr(i2c);
    while ((len - i) > 3U)
    {
        const i2c_status_t status = wait_sr1(i2c, I2C_SR1_RXNE);
        if (status != I2C_OK)
        {
            return status;
        }
        buf[i] = (uint8_t)i2c->DR;
        ++i;
    }

    /* Byte N-2 is in DR and N-1 in the shift register. */
    i2c_status_t status = wait_sr1(i2c, I2C_SR1_BTF);
    if (status == I2C_OK)
    {
        i2c->CR1 &= ~I2C_CR1_ACK;
        buf[i] = (uint8_t)i2c->DR;
        ++i;
        /* Byte N-1 is in DR and N (NACKed) in the shift register. */
        status = wait_sr1(i2c, I2C_SR1_BTF);
    }
    if (status == I2C_OK)
    {
        i2c->CR1 |= I2C_CR1_STOP;
        buf[i] = (uint8_t)i2c->DR;
        buf[i + 1U] = (uint8_t)i2c->DR;
    }
    return status;
}

i2c_status_t i2c_read_regs(i2c_regs_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint32_t len)
{
    if ((buf == 0) || (len == 0U))
    {
        return I2C_ERR_ARG;
    }

    i2c_status_t status = send_register(i2c, addr, reg);
    if (status == I2C_OK)
    {
        /* The register byte must be fully shifted out before the repeated START. */
        status = wait_sr1(i2c, I2C_SR1_BTF);
    }
    if (status == I2C_OK)
    {
        i2c->CR1 |= I2C_CR1_ACK;
        if (len == 2U)
        {
            i2c->CR1 |= I2C_CR1_POS;
        }
        status = send_address(i2c, addr, true);
    }
    if (status == I2C_OK)
    {
        if (len == 1U)
        {
            status = receive_one(i2c, buf);
        }
        else if (len == 2U)
        {
            status = receive_two(i2c, buf);
        }
        else
        {
            status = receive_many(i2c, buf, len);
        }
    }

    if (status != I2C_OK)
    {
        return abort_transfer(i2c, status);
    }
    i2c->CR1 &= ~I2C_CR1_ACK;
    return I2C_OK;
}

const char *i2c_status_name(i2c_status_t status)
{
    switch (status)
    {
        case I2C_OK:
            return "OK";
        case I2C_ERR_NACK:
            return "NACK";
        case I2C_ERR_BUS:
            return "BUS";
        case I2C_ERR_TIMEOUT:
            return "TIMEOUT";
        case I2C_ERR_ARG:
            return "ARG";
        default:
            return "?";
    }
}
