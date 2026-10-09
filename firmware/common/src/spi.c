/**
 * @file spi.c
 * @brief Polled, register-level SPI master driver.
 */
#include "spi.h"

/* Upper bound on status polls; one byte takes 16 core cycles at PCLK/2, so this is very generous. */
#define SPI_POLL_LIMIT  (20000UL)
#define SPI_FILL_BYTE   (0xFFU)

static bool wait_flag(const spi_regs_t *spi, uint32_t flag, bool set)
{
    for (uint32_t i = 0U; i < SPI_POLL_LIMIT; ++i)
    {
        if (((spi->SR & flag) != 0U) == set)
        {
            return true;
        }
    }
    return false;
}

void spi_init(spi_regs_t *spi, uint32_t baud_rate_field)
{
    spi->CR1 = 0U;
    spi->CR2 = 0U;
    /* SSM + SSI keep the internal NSS high, so the master never sees a mode fault. */
    spi->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
               ((baud_rate_field << SPI_CR1_BR_SHIFT) & SPI_CR1_BR_MASK);
    spi->CR1 |= SPI_CR1_SPE;
}

bool spi_transfer(spi_regs_t *spi, const uint8_t *tx, uint8_t *rx, size_t len)
{
    /* Drop a byte left over from an earlier, interrupted transfer. */
    if ((spi->SR & (SPI_SR_RXNE | SPI_SR_OVR)) != 0U)
    {
        (void)spi->DR;
        (void)spi->SR;
    }

    for (size_t i = 0U; i < len; ++i)
    {
        if (!wait_flag(spi, SPI_SR_TXE, true))
        {
            return false;
        }
        spi->DR = (tx != 0) ? tx[i] : SPI_FILL_BYTE;

        /* One byte in flight at a time: wait for its reply before sending the next. */
        if (!wait_flag(spi, SPI_SR_RXNE, true))
        {
            return false;
        }
        const uint8_t byte = (uint8_t)spi->DR;
        if (rx != 0)
        {
            rx[i] = byte;
        }
    }

    /* The caller may release chip select next, so the last bit must have left the shifter. */
    return wait_flag(spi, SPI_SR_BSY, false);
}
