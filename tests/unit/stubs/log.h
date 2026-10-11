/**
 * @file log.h
 * @brief Host test double for the debug log (firmware/common/include/log.h).
 *
 * Same macros, without the printf format attribute: the firmware's formats assume
 * 32-bit long, as on the target, and are checked by the firmware build. The test
 * that links this records each line so it can check what was logged.
 */
#ifndef LOG_H
#define LOG_H

void log_line(char level, const char *fmt, ...);

#define LOG_INFO(...)   log_line('I', __VA_ARGS__)
#define LOG_WARN(...)   log_line('W', __VA_ARGS__)
#define LOG_ERROR(...)  log_line('E', __VA_ARGS__)

#endif /* LOG_H */
