/**
 * @file ap_link.c
 * @brief Node A's link to the autopilot on UART5.
 */
#include "ap_link.h"

#include "board.h"
#include "cmd_protocol.h"
#include "log.h"
#include "uart.h"

typedef struct
{
    uart_rx_buffer_t rx;
    cmd_line_t       line;
    ap_status_t      status;
    bool             have_status;
    uint32_t         status_ms;
    uint32_t         received;
    uint32_t         rejected;
} ap_link_state_t;

static ap_link_state_t s_ap;

void UART5_IRQHandler(void);

void UART5_IRQHandler(void)
{
    uart_rx_irq_handler(BOARD_AP_UART, &s_ap.rx);
}

void ap_link_init(void)
{
    s_ap = (ap_link_state_t){ 0 };
    cmd_line_reset(&s_ap.line);
    board_autopilot_uart_init();
    uart_init(BOARD_AP_UART, BOARD_PCLK1_HZ, BOARD_AP_BAUD);
    uart_rx_irq_start(BOARD_AP_UART, &s_ap.rx, BOARD_AP_IRQN);
    LOG_INFO("autopilot: link on UART5 at %lu baud", BOARD_AP_BAUD);
}

void ap_link_poll(uint32_t now_ms)
{
    char c;

    while (uart_rx_pop(&s_ap.rx, &c))
    {
        const cmd_line_event_t event = cmd_line_feed(&s_ap.line, c);
        ap_status_t status;

        if (event == CMD_LINE_TOO_LONG)
        {
            s_ap.rejected++;
        }
        else if (event != CMD_LINE_READY)
        {
            /* Line not complete yet. */
        }
        else if (ap_parse_status(s_ap.line.buf, &status))
        {
            s_ap.status = status;
            s_ap.have_status = true;
            s_ap.status_ms = now_ms;
            s_ap.received++;
        }
        else
        {
            s_ap.rejected++;
        }
    }
}

bool ap_link_status(ap_status_t *out, uint32_t now_ms)
{
    *out = s_ap.status;
    return s_ap.have_status && ((now_ms - s_ap.status_ms) < AP_LINK_STALE_MS);
}

void ap_link_send(const ap_command_t *cmd)
{
    char line[AP_MSG_MAX_LEN];

    if (ap_format_command(cmd, line, sizeof(line)) != 0U)
    {
        /* About 2 ms at 115200 baud, in ControlTask. */
        uart_write(BOARD_AP_UART, line);
    }
}

uint32_t ap_link_received(void)
{
    return s_ap.received;
}

uint32_t ap_link_rejected(void)
{
    return s_ap.rejected;
}
