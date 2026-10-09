/**
 * @file node_boot.c
 * @brief Boot sequence and debug console shared by all VoltDragon nodes.
 */
#include "node_boot.h"

#include "board.h"
#include "fault.h"
#include "iwdg.h"
#include "log.h"
#include "reset_info.h"
#include "voltdragon_version.h"
#include "systick.h"
#include "uart.h"

static void print_help(void)
{
    LOG_INFO("debug keys: f=HardFault z=div-by-zero s=invalid-state w=watchdog-hang r=reset ?=help");
}

void node_boot(const char *node_name, uint32_t watchdog_ms, board_console_pins_t console_pins)
{
    /* Read the reset cause first, before anything else can disturb RCC_CSR. */
    reset_info_init();

    board_init(console_pins);
    uart_init(BOARD_CONSOLE_UART, BOARD_PCLK1_HZ, BOARD_CONSOLE_BAUD);
    log_init(BOARD_CONSOLE_UART);
    systick_init(BOARD_SYSCLK_HZ);

    LOG_INFO("VoltDragon %s v%u.%u.%u", node_name,
             VOLTDRAGON_VERSION_MAJOR, VOLTDRAGON_VERSION_MINOR, VOLTDRAGON_VERSION_PATCH);
    LOG_INFO("reset cause: %s, reset count: %lu",
             reset_info_cause_name(reset_info_cause()), reset_info_count());
    if (!reset_info_flags_reliable())
    {
        LOG_WARN("RCC reset flags not clearable (emulator?): watchdog and pin resets report UNKNOWN");
    }

    fault_record_t last_fault;
    if (reset_info_last_fault(&last_fault))
    {
        LOG_WARN("previous run ended with a fault:");
        fault_print(&last_fault);
    }

    iwdg_start(watchdog_ms);
    LOG_INFO("watchdog started, timeout %lu ms", watchdog_ms);
    print_help();
}

char node_debug_console_poll(void)
{
    char key;

    if (!uart_try_getc(BOARD_CONSOLE_UART, &key))
    {
        return NODE_KEY_NONE;
    }

    switch (key)
    {
        case 'f':
            LOG_WARN("triggering HardFault (undefined instruction)");
            fault_trigger(FAULT_TEST_UNDEFINED_INSTRUCTION);
            break;
        case 'z':
            LOG_WARN("triggering divide-by-zero fault");
            fault_trigger(FAULT_TEST_DIVIDE_BY_ZERO);
            break;
        case 's':
            LOG_WARN("triggering invalid-state fault");
            fault_trigger(FAULT_TEST_INVALID_STATE);
            break;
        case 'w':
            LOG_WARN("hanging main loop, expecting watchdog reset");
            for (;;)
            {
            }
        case 'r':
            LOG_WARN("software reset requested");
            log_flush();
            reset_info_system_reset();
        case '?':
            print_help();
            break;
        default:
            /* Not a common key: let the node handle it. */
            return key;
    }
    return NODE_KEY_NONE;
}
