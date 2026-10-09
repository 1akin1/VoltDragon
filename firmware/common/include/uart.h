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

#endif /* UART_H */
