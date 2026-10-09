/**
 * @file cmd_dispatch.h
 * @brief Command table lookup, argument-count checks and retransmission handling.
 *
 * A request whose sequence number equals that of the previous request is a
 * retransmission (the reply was lost): the previous response is sent again and
 * the command is not executed a second time.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef CMD_DISPATCH_H
#define CMD_DISPATCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cmd_protocol.h"

/**
 * Executes a command. The response has already been started as an ACK for the
 * verb; the handler appends any result fields. Returning false turns the reply
 * into a NAK with reason ARGS (an argument value was rejected).
 */
typedef bool (*cmd_handler_t)(const cmd_request_t *req, cmd_response_t *rsp);

typedef struct
{
    const char   *verb;
    uint8_t       min_args;
    uint8_t       max_args;
    cmd_handler_t handler;
} cmd_entry_t;

typedef struct
{
    const cmd_entry_t *table;
    size_t             count;
    bool               has_last;
    uint16_t           last_seq;
    cmd_error_t        last_error;
    cmd_response_t     last_response;
} cmd_dispatcher_t;

/** Outcome of one line, for logging. */
typedef struct
{
    cmd_error_t error;      /**< CMD_OK if the command was acknowledged. */
    uint16_t    seq;
    bool        replayed;   /**< The response was resent for a retransmitted request. */
} cmd_outcome_t;

void cmd_dispatcher_init(cmd_dispatcher_t *dispatcher, const cmd_entry_t *table, size_t count);

/** Parses and executes one complete line and builds the response. */
cmd_outcome_t cmd_dispatch_line(cmd_dispatcher_t *dispatcher, const char *line, cmd_response_t *rsp);

/** Builds the NAK for a line that exceeded CMD_LINE_MAX. */
cmd_outcome_t cmd_dispatch_too_long(cmd_response_t *rsp);

#endif /* CMD_DISPATCH_H */
