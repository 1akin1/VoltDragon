/**
 * @file lwip_port.c
 * @brief lwIP system hooks: time base, diagnostics and assertions.
 */
#include "arch/cc.h"
#include "lwip/arch.h"
#include "lwip/sys.h"

#include "log.h"
#include "systick.h"

u32_t sys_now(void)
{
    return systick_now_ms();
}

void lwip_port_diag(const char *message)
{
    LOG_INFO("lwip: %s", message);
}

void lwip_port_assert(const char *message, const char *file, int line)
{
    LOG_ERROR("lwip assertion failed: %s (%s:%d)", message, file, line);
    log_flush();
    /* Stop here: the watchdog resets the node into a known state. */
    for (;;)
    {
    }
}
