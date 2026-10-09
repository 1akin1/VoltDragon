/**
 * @file uart.c
 * @brief Polled, register-level USART driver.
 */
#include "uart.h"

/* Upper bound on status polls; about 1 ms at 16 MHz, longer than one byte at 9600 baud. */
#define UART_POLL_LIMIT (20000UL)

static bool wait_for_flag(const usart_regs_t *uart, uint32_t flag)
{
    for (uint32_t i = 0U; i < UART_POLL_LIMIT; ++i)
    {
        if ((uart->SR & flag) != 0U)
        {
            return true;
        }
    }
    return false;
}

void uart_init(usart_regs_t *uart, uint32_t pclk_hz, uint32_t baud)
{
    uart->CR1 = 0U;
    uart->CR2 = 0U;     /* 1 stop bit */
    uart->CR3 = 0U;     /* no flow control */
    /* OVER8 = 0: BRR = pclk / baud, rounded to nearest. */
    uart->BRR = (pclk_hz + (baud / 2U)) / baud;
    uart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void uart_putc(usart_regs_t *uart, char c)
{
    /* On timeout the byte is still written: losing log output beats hanging. */
    (void)wait_for_flag(uart, USART_SR_TXE);
    uart->DR = (uint32_t)(uint8_t)c;
}

void uart_write(usart_regs_t *uart, const char *str)
{
    for (const char *p = str; *p != '\0'; ++p)
    {
        uart_putc(uart, *p);
    }
}

void uart_flush(usart_regs_t *uart)
{
    (void)wait_for_flag(uart, USART_SR_TC);
}

bool uart_try_getc(usart_regs_t *uart, char *out)
{
    const uint32_t sr = uart->SR;

    if ((sr & USART_SR_RXNE) != 0U)
    {
        *out = (char)(uint8_t)(uart->DR & 0xFFU);
        return true;
    }
    if ((sr & USART_SR_ORE) != 0U)
    {
        /* Clear overrun: SR read followed by DR read. */
        (void)uart->DR;
    }
    return false;
}

_Static_assert((UART_RX_BUFFER_SIZE & (UART_RX_BUFFER_SIZE - 1U)) == 0U,
               "UART_RX_BUFFER_SIZE must be a power of two");

void uart_rx_irq_start(usart_regs_t *uart, uart_rx_buffer_t *rx, uint32_t irq_number)
{
    rx->head = 0U;
    rx->tail = 0U;
    rx->overruns = 0U;
    rx->dropped = 0U;

    /* Discard anything received before the buffer existed. */
    (void)uart->SR;
    (void)uart->DR;

    uart->CR1 |= USART_CR1_RXNEIE;
    NVIC_ISER[irq_number / 32U] = 1UL << (irq_number % 32U);
}

void uart_rx_irq_handler(usart_regs_t *uart, uart_rx_buffer_t *rx)
{
    const uint32_t sr = uart->SR;

    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) == 0U)
    {
        return;
    }

    /* Reading DR after SR clears both RXNE and ORE. */
    const uint8_t byte = (uint8_t)(uart->DR & 0xFFU);
    if ((sr & USART_SR_ORE) != 0U)
    {
        rx->overruns = rx->overruns + 1U;
    }
    if ((sr & USART_SR_RXNE) == 0U)
    {
        return;
    }

    /* Indices run freely and wrap; their difference is the fill level. */
    const uint32_t head = rx->head;
    if ((head - rx->tail) >= UART_RX_BUFFER_SIZE)
    {
        rx->dropped = rx->dropped + 1U;
        return;
    }
    rx->data[head & (UART_RX_BUFFER_SIZE - 1U)] = byte;
    /* Publish the byte before the index that makes it visible. */
    __asm volatile("dmb" ::: "memory");
    rx->head = head + 1U;
}

bool uart_rx_pop(uart_rx_buffer_t *rx, char *out)
{
    const uint32_t tail = rx->tail;

    if (rx->head == tail)
    {
        return false;
    }
    *out = (char)rx->data[tail & (UART_RX_BUFFER_SIZE - 1U)];
    __asm volatile("dmb" ::: "memory");
    rx->tail = tail + 1U;
    return true;
}
