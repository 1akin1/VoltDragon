/**
 * @file cc.h
 * @brief lwIP architecture definitions for the Cortex-M4 with newlib.
 *
 * Types and byte order come from lwIP's defaults (stdint.h, little-endian).
 * A failed lwIP assertion is logged and then stops the main loop, so the
 * watchdog resets the node into a known state.
 */
#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

void lwip_port_diag(const char *message);
void lwip_port_assert(const char *message, const char *file, int line) __attribute__((noreturn));

#define LWIP_PLATFORM_DIAG(x)               do { lwip_port_diag("lwip diagnostic"); } while (0)
#define LWIP_PLATFORM_ASSERT(x)             lwip_port_assert((x), __FILE__, __LINE__)

#endif /* LWIP_ARCH_CC_H */
