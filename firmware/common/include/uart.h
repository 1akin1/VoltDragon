/**
 * @file uart.h
 * @brief Polled, register-level USART driver.
 *
 * Transmission is blocking with a bounded wait so the driver can be used from
 * fault handlers, where interrupts are not available.
 */
#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4_regs.h"

/** Configures 8N1 framing at the given baud rate and enables TX and RX. */
void uart_init(usart_regs_t *uart, uint32_t pclk_hz, uint32_t baud);

/** Sends one byte, waiting (bounded) for the transmit register to be free. */
void uart_putc(usart_regs_t *uart, char c);

/** Sends a NUL-terminated string. */
void uart_write(usart_regs_t *uart, const char *str);

/** Waits (bounded) until the last byte has left the shift register. */
void uart_flush(usart_regs_t *uart);

/**
 * Reads one byte if available.
 * @return true and stores the byte in @p out if one was received.
 */
bool uart_try_getc(usart_regs_t *uart, char *out);

/** Size of the interrupt-driven receive buffer; a power of two. */
#define UART_RX_BUFFER_SIZE (128U)

/**
 * Receive ring buffer filled by the UART interrupt. Single producer (the
 * interrupt) and single consumer (the main loop), so no locking is needed.
 */
typedef struct
{
    volatile uint8_t  data[UART_RX_BUFFER_SIZE];
    volatile uint32_t head;         /**< Written only by the interrupt. */
    volatile uint32_t tail;         /**< Written only by the consumer. */
    volatile uint32_t overruns;     /**< Bytes lost in the UART itself (ORE). */
    volatile uint32_t dropped;      /**< Bytes lost because the buffer was full. */
} uart_rx_buffer_t;

/**
 * Starts interrupt-driven reception into @p rx. The UART must already be
 * initialised, and the node must route the UART's interrupt handler to
 * uart_rx_irq_handler().
 */
void uart_rx_irq_start(usart_regs_t *uart, uart_rx_buffer_t *rx, uint32_t irq_number);

/** Moves a received byte into the buffer. Call from the UART interrupt handler. */
void uart_rx_irq_handler(usart_regs_t *uart, uart_rx_buffer_t *rx);

/** Takes one byte from the buffer. Returns false if it is empty. */
bool uart_rx_pop(uart_rx_buffer_t *rx, char *out);

#endif /* UART_H */
