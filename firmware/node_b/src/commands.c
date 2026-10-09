/**
 * @file commands.c
 * @brief Node B operator command interface on USART3.
 */
#include "commands.h"

#include <stdbool.h>

#include "board.h"
#include "can_rx.h"
#include "cmd_dispatch.h"
#include "log.h"
#include "reset_info.h"
#include "systick.h"
#include "uart.h"
#include "voltdragon_version.h"

typedef struct
{
    uart_rx_buffer_t rx;
    cmd_line_t       line;
    cmd_dispatcher_t dispatcher;
    cmd_response_t   response;
    uint32_t         accepted;
    uint32_t         rejected;
    uint32_t         reported_rx_errors;
    uint32_t         telemetry_rate_hz;
} commands_state_t;

static commands_state_t s_cmd;

void USART3_IRQHandler(void);

void USART3_IRQHandler(void)
{
    uart_rx_irq_handler(BOARD_COMMAND_UART, &s_cmd.rx);
}

static bool handle_ping(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    (void)rsp;
    return true;
}

static bool handle_version(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    cmd_response_add(rsp, VOLTDRAGON_VERSION_STRING);
    return true;
}

static bool handle_status(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    cmd_response_add_u32(rsp, systick_now_ms());
    cmd_response_add_u32(rsp, reset_info_count());
    cmd_response_add_u32(rsp, s_cmd.accepted);
    cmd_response_add_u32(rsp, s_cmd.rejected);
    return true;
}

static bool handle_tlm_rate(const cmd_request_t *req, cmd_response_t *rsp)
{
    if (req->argc == 1U)
    {
        uint32_t hz = 0U;

        if (!cmd_parse_u32(req->args[0], &hz) || (hz < COMMANDS_TLM_RATE_MIN_HZ) ||
            (hz > COMMANDS_TLM_RATE_MAX_HZ))
        {
            return false;
        }
        s_cmd.telemetry_rate_hz = hz;
    }
    cmd_response_add_u32(rsp, s_cmd.telemetry_rate_hz);
    return true;
}

static bool handle_can(const cmd_request_t *req, cmd_response_t *rsp)
{
    const can_rx_stats_t stats = can_rx_stats();
    can_rx_node_a_t node_a;

    (void)req;
    cmd_response_add_u32(rsp, stats.valid);
    cmd_response_add_u32(rsp, stats.rejected);
    cmd_response_add_u32(rsp, stats.lost);
    cmd_response_add_u32(rsp, stats.unknown_id);
    if (can_rx_node_a(&node_a))
    {
        cmd_response_add_u32(rsp, systick_now_ms() - node_a.last_valid_ms);
    }
    else
    {
        cmd_response_add(rsp, "-");
    }
    return true;
}

static const cmd_entry_t s_table[] = {
    { "PING", 0U, 0U, handle_ping },
    { "VERSION", 0U, 0U, handle_version },
    { "STATUS", 0U, 0U, handle_status },
    { "TLM_RATE", 0U, 1U, handle_tlm_rate },
    { "CAN", 0U, 0U, handle_can },
};

#define TABLE_SIZE (sizeof(s_table) / sizeof(s_table[0]))

static void send_response(const char *request, cmd_outcome_t outcome)
{
    /* A replay re-sends an earlier reply; the command was not processed again. */
    if (outcome.replayed)
    {
    }
    else if (outcome.error == CMD_OK)
    {
        s_cmd.accepted++;
    }
    else
    {
        s_cmd.rejected++;
    }

    uart_write(BOARD_COMMAND_UART, s_cmd.response.buf);
    LOG_INFO("cmd: %s -> %s%s", request,
             (outcome.error == CMD_OK) ? "ACK" : cmd_error_name(outcome.error),
             outcome.replayed ? " (replayed)" : "");
}

void commands_init(void)
{
    s_cmd = (commands_state_t){ 0 };
    s_cmd.telemetry_rate_hz = COMMANDS_TLM_RATE_DEFAULT_HZ;
    cmd_line_reset(&s_cmd.line);
    cmd_dispatcher_init(&s_cmd.dispatcher, s_table, TABLE_SIZE);

    board_command_uart_init();
    uart_init(BOARD_COMMAND_UART, BOARD_PCLK1_HZ, BOARD_COMMAND_BAUD);
    uart_rx_irq_start(BOARD_COMMAND_UART, &s_cmd.rx, BOARD_COMMAND_IRQN);
    LOG_INFO("cmd: listening on USART3 at %lu baud", BOARD_COMMAND_BAUD);
}

void commands_poll(void)
{
    char c;

    while (uart_rx_pop(&s_cmd.rx, &c))
    {
        const cmd_line_event_t event = cmd_line_feed(&s_cmd.line, c);

        if (event == CMD_LINE_READY)
        {
            send_response(s_cmd.line.buf,
                          cmd_dispatch_line(&s_cmd.dispatcher, s_cmd.line.buf, &s_cmd.response));
        }
        else if (event == CMD_LINE_TOO_LONG)
        {
            send_response("(line too long)", cmd_dispatch_too_long(&s_cmd.response));
        }
        else
        {
            /* Line not complete yet. */
        }
    }
}

void commands_report(void)
{
    const uint32_t errors = s_cmd.rx.overruns + s_cmd.rx.dropped;

    if (errors != s_cmd.reported_rx_errors)
    {
        LOG_WARN("cmd: receive errors: %lu overruns, %lu dropped", s_cmd.rx.overruns,
                 s_cmd.rx.dropped);
        s_cmd.reported_rx_errors = errors;
    }
}

uint32_t commands_telemetry_rate_hz(void)
{
    return s_cmd.telemetry_rate_hz;
}
