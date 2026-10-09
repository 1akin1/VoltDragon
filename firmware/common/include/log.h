/**
 * @file log.h
 * @brief Timestamped console logging with a small printf subset.
 *
 * Supported conversions: %s %c %d %u %x %X %%, with optional '0' flag,
 * field width and 'l' length modifier (long is 32-bit on this target).
 *
 * Output is polled, so logging is usable from fault handlers but must not be
 * called from time-critical code paths.
 *
 * MISRA C:2012 deviation D-001 (Rule 17.1): <stdarg.h> is used to provide a
 * printf-style interface. See docs/misra-deviations.md.
 */
#ifndef LOG_H
#define LOG_H

#include "stm32f4_regs.h"

/** Selects the UART used for log output. Must be called before logging. */
void log_init(usart_regs_t *uart);

typedef void (*log_lock_fn_t)(void);

/**
 * Installs functions that serialise log_line() between tasks. Without them
 * (bare metal) lines are written unlocked. The functions must do nothing when
 * called from an interrupt or exception handler, so fault reports never block.
 */
void log_set_lock(log_lock_fn_t lock, log_lock_fn_t unlock);

/** Writes a formatted line: "[ssssss.mmm] L <message>\r\n". */
void log_line(char level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/** Writes formatted text without a timestamp or line ending. */
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Waits until all queued log output has been transmitted. */
void log_flush(void);

#define LOG_INFO(...)   log_line('I', __VA_ARGS__)
#define LOG_WARN(...)   log_line('W', __VA_ARGS__)
#define LOG_ERROR(...)  log_line('E', __VA_ARGS__)

#endif /* LOG_H */
