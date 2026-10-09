/**
 * @file commands.c
 * @brief Node B operator command interface on USART3.
 */
#include "commands.h"

#include <stdbool.h>
#include <string.h>

#include "board.h"
#include "can_rx.h"
#include "cmd_dispatch.h"
#include "flight_mode.h"
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
    cmd_dispatcher_t udp_dispatcher;    /* Own retransmission cache: UDP is a separate session. */
    cmd_response_t   udp_response;
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

static cmd_error_t handle_ping(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    (void)rsp;
    return CMD_OK;
}

static cmd_error_t handle_version(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    cmd_response_add(rsp, VOLTDRAGON_VERSION_STRING);
    return CMD_OK;
}

static cmd_error_t handle_status(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    cmd_response_add_u32(rsp, systick_now_ms());
    cmd_response_add_u32(rsp, reset_info_count());
    cmd_response_add_u32(rsp, s_cmd.accepted);
    cmd_response_add_u32(rsp, s_cmd.rejected);
    return CMD_OK;
}

static cmd_error_t handle_tlm_rate(const cmd_request_t *req, cmd_response_t *rsp)
{
    if (req->argc == 1U)
    {
        uint32_t hz = 0U;

        if (!cmd_parse_u32(req->args[0], &hz) || (hz < COMMANDS_TLM_RATE_MIN_HZ) ||
            (hz > COMMANDS_TLM_RATE_MAX_HZ))
        {
            return CMD_ERR_ARGS;
        }
        s_cmd.telemetry_rate_hz = hz;
    }
    cmd_response_add_u32(rsp, s_cmd.telemetry_rate_hz);
    return CMD_OK;
}

static cmd_error_t handle_can(const cmd_request_t *req, cmd_response_t *rsp)
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
    return CMD_OK;
}

/*
 * MODE <name> [OVERRIDE]: checked here against Node A's mode with the same transition
 * table Node A uses, then forwarded over CAN; Node A decides finally. The ACK means
 * the request was forwarded (or the mode is already the one requested).
 */
static cmd_error_t handle_mode(const cmd_request_t *req, cmd_response_t *rsp)
{
    flight_mode_t requested = FLIGHT_MODE_MISSION;
    flight_mode_t current = FLIGHT_MODE_MISSION;
    const uint32_t now_ms = systick_now_ms();

    if (!flight_mode_from_name(req->args[0], &requested))
    {
        return CMD_ERR_ARGS;
    }
    const bool override = req->argc == 2U;
    if (override && (strcmp(req->args[1], "OVERRIDE") != 0))
    {
        return CMD_ERR_ARGS;
    }
    /* Without Node A's current mode the request cannot be checked. */
    if (!can_rx_node_a_mode(now_ms, &current))
    {
        return CMD_ERR_REFUSED;
    }

    const flight_change_t change = flight_mode_check(current, requested, FLIGHT_CAUSE_OPERATOR,
                                                     override);
    if ((change != FLIGHT_CHANGE_OK) && (change != FLIGHT_CHANGE_UNCHANGED))
    {
        LOG_WARN("cmd: mode %s -> %s refused: %s", flight_mode_name(current),
                 flight_mode_name(requested), flight_change_name(change));
        return CMD_ERR_REFUSED;
    }
    if (change == FLIGHT_CHANGE_OK)
    {
        (void)can_rx_request_mode(requested, override, now_ms);
    }
    cmd_response_add(rsp, flight_mode_name(requested));
    return CMD_OK;
}

static const cmd_entry_t s_table[] = {
    { "PING", 0U, 0U, handle_ping },
    { "VERSION", 0U, 0U, handle_version },
    { "STATUS", 0U, 0U, handle_status },
    { "TLM_RATE", 0U, 1U, handle_tlm_rate },
    { "CAN", 0U, 0U, handle_can },
    { "MODE", 1U, 2U, handle_mode },
};

#define TABLE_SIZE (sizeof(s_table) / sizeof(s_table[0]))

static void record_outcome(const char *transport, const char *request, cmd_outcome_t outcome)
{
    /*
     * Any intact frame shows the ground station is there (HLR-003), whether or not the
     * command was accepted. A damaged frame does not: it may be noise.
     */
    if ((outcome.error != CMD_ERR_FRAME) && (outcome.error != CMD_ERR_CHECKSUM) &&
        (outcome.error != CMD_ERR_SEQ) && (outcome.error != CMD_ERR_LENGTH))
    {
        can_rx_ground_contact(systick_now_ms());
    }

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

    LOG_INFO("cmd%s: %s -> %s%s", transport, request,
             (outcome.error == CMD_OK) ? "ACK" : cmd_error_name(outcome.error),
             outcome.replayed ? " (replayed)" : "");
}

static void send_response(const char *request, cmd_outcome_t outcome)
{
    uart_write(BOARD_COMMAND_UART, s_cmd.response.buf);
    record_outcome("", request, outcome);
}

void commands_init(void)
{
    s_cmd = (commands_state_t){ 0 };
    s_cmd.telemetry_rate_hz = COMMANDS_TLM_RATE_DEFAULT_HZ;
    cmd_line_reset(&s_cmd.line);
    cmd_dispatcher_init(&s_cmd.dispatcher, s_table, TABLE_SIZE);
    cmd_dispatcher_init(&s_cmd.udp_dispatcher, s_table, TABLE_SIZE);

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

size_t commands_handle_datagram(const uint8_t *data, size_t len, char *reply, size_t reply_size)
{
    char line[CMD_LINE_MAX + 1U];
    size_t text_len = len;
    cmd_outcome_t outcome;

    /* One command per datagram; a trailing CR/LF is allowed, as on the UART. */
    while ((text_len > 0U) && ((data[text_len - 1U] == (uint8_t)'\r') ||
                               (data[text_len - 1U] == (uint8_t)'\n')))
    {
        text_len--;
    }

    if (text_len > CMD_LINE_MAX)
    {
        outcome = cmd_dispatch_too_long(&s_cmd.udp_response);
        record_outcome(" udp", "(datagram too long)", outcome);
    }
    else
    {
        (void)memcpy(line, data, text_len);
        line[text_len] = '\0';
        outcome = cmd_dispatch_line(&s_cmd.udp_dispatcher, line, &s_cmd.udp_response);
        record_outcome(" udp", line, outcome);
    }

    /* Replies are sent without the line ending, which a datagram does not need. */
    const size_t reply_len = s_cmd.udp_response.len - 2U;
    if (reply_len > reply_size)
    {
        return 0U;
    }
    (void)memcpy(reply, s_cmd.udp_response.buf, reply_len);
    return reply_len;
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
