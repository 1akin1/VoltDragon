/**
 * @file log.c
 * @brief Timestamped console logging with a small printf subset.
 */
#include "log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "systick.h"
#include "uart.h"

/* Enough digits for a 32-bit value in base 10 (10 digits) plus a sign. */
#define NUM_BUF_SIZE (12U)

static usart_regs_t *s_uart;

static void out_char(char c)
{
    if (s_uart != 0)
    {
        uart_putc(s_uart, c);
    }
}

static void out_padded(const char *digits, uint32_t len, uint32_t width, char pad)
{
    for (uint32_t i = len; i < width; ++i)
    {
        out_char(pad);
    }
    for (uint32_t i = 0U; i < len; ++i)
    {
        out_char(digits[i]);
    }
}

static void out_unsigned(uint32_t value, uint32_t base, bool upper, uint32_t width, char pad)
{
    static const char lower_digits[] = "0123456789abcdef";
    static const char upper_digits[] = "0123456789ABCDEF";
    const char *digits = upper ? upper_digits : lower_digits;
    char buf[NUM_BUF_SIZE];
    uint32_t len = 0U;
    uint32_t v = value;

    do
    {
        buf[NUM_BUF_SIZE - 1U - len] = digits[v % base];
        v /= base;
        ++len;
    } while ((v != 0U) && (len < NUM_BUF_SIZE));

    out_padded(&buf[NUM_BUF_SIZE - len], len, width, pad);
}

static void out_signed(int32_t value, uint32_t width, char pad)
{
    if (value < 0)
    {
        /* Negate in unsigned arithmetic so INT32_MIN is handled. */
        const uint32_t magnitude = 0U - (uint32_t)value;
        out_char('-');
        out_unsigned(magnitude, 10U, false, (width > 0U) ? (width - 1U) : 0U, pad);
    }
    else
    {
        out_unsigned((uint32_t)value, 10U, false, width, pad);
    }
}

static void log_vprintf(const char *fmt, va_list args)
{
    const char *p = fmt;

    while (*p != '\0')
    {
        if (*p != '%')
        {
            out_char(*p);
            ++p;
            continue;
        }
        ++p;

        char pad = ' ';
        uint32_t width = 0U;

        if (*p == '0')
        {
            pad = '0';
            ++p;
        }
        while ((*p >= '0') && (*p <= '9'))
        {
            width = (width * 10U) + (uint32_t)(*p - '0');
            ++p;
        }
        if (*p == 'l')
        {
            ++p;
        }

        switch (*p)
        {
            case 'd':
                out_signed((int32_t)va_arg(args, long), width, pad);
                break;
            case 'u':
                out_unsigned((uint32_t)va_arg(args, unsigned long), 10U, false, width, pad);
                break;
            case 'x':
                out_unsigned((uint32_t)va_arg(args, unsigned long), 16U, false, width, pad);
                break;
            case 'X':
                out_unsigned((uint32_t)va_arg(args, unsigned long), 16U, true, width, pad);
                break;
            case 'c':
                out_char((char)va_arg(args, int));
                break;
            case 's':
            {
                const char *s = va_arg(args, const char *);
                for (const char *c = (s != 0) ? s : "(null)"; *c != '\0'; ++c)
                {
                    out_char(*c);
                }
                break;
            }
            case '%':
                out_char('%');
                break;
            case '\0':
                /* Format string ends with '%': stop without reading past it. */
                return;
            default:
                out_char('%');
                out_char(*p);
                break;
        }
        ++p;
    }
}

void log_init(usart_regs_t *uart)
{
    s_uart = uart;
}

void log_line(char level, const char *fmt, ...)
{
    const uint32_t now = systick_now_ms();
    va_list args;

    out_char('[');
    out_unsigned(now / 1000U, 10U, false, 6U, ' ');
    out_char('.');
    out_unsigned(now % 1000U, 10U, false, 3U, '0');
    out_char(']');
    out_char(' ');
    out_char(level);
    out_char(' ');

    va_start(args, fmt);
    log_vprintf(fmt, args);
    va_end(args);

    out_char('\r');
    out_char('\n');
}

void log_printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    log_vprintf(fmt, args);
    va_end(args);
}

void log_flush(void)
{
    if (s_uart != 0)
    {
        uart_flush(s_uart);
    }
}
