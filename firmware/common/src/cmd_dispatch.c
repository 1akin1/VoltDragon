/**
 * @file cmd_dispatch.c
 * @brief Command table lookup, argument-count checks and retransmission handling.
 */
#include "cmd_dispatch.h"

#include <string.h>

/* Verb shown in a NAK when the request could not be parsed far enough to know it. */
#define UNKNOWN_VERB "-"

static void build_nak(cmd_response_t *rsp, uint16_t seq, const char *verb, cmd_error_t error)
{
    cmd_response_begin(rsp, seq, false, verb);
    cmd_response_add(rsp, cmd_error_name(error));
    cmd_response_end(rsp);
}

static const cmd_entry_t *find_entry(const cmd_dispatcher_t *dispatcher, const char *verb)
{
    for (size_t i = 0U; i < dispatcher->count; ++i)
    {
        if (strcmp(dispatcher->table[i].verb, verb) == 0)
        {
            return &dispatcher->table[i];
        }
    }
    return 0;
}

static cmd_error_t execute(const cmd_dispatcher_t *dispatcher, const cmd_request_t *req,
                           cmd_response_t *rsp)
{
    const cmd_entry_t *entry = find_entry(dispatcher, req->verb);

    if (entry == 0)
    {
        return CMD_ERR_UNKNOWN;
    }
    if ((req->argc < entry->min_args) || (req->argc > entry->max_args))
    {
        return CMD_ERR_ARGS;
    }

    cmd_response_begin(rsp, req->seq, true, req->verb);
    if (!entry->handler(req, rsp))
    {
        return CMD_ERR_ARGS;
    }
    cmd_response_end(rsp);
    return CMD_OK;
}

void cmd_dispatcher_init(cmd_dispatcher_t *dispatcher, const cmd_entry_t *table, size_t count)
{
    (void)memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->table = table;
    dispatcher->count = count;
}

cmd_outcome_t cmd_dispatch_line(cmd_dispatcher_t *dispatcher, const char *line, cmd_response_t *rsp)
{
    cmd_request_t req;
    cmd_outcome_t outcome = { CMD_OK, CMD_SEQ_UNKNOWN, false };

    outcome.error = cmd_parse(line, &req);
    outcome.seq = req.seq;

    if ((outcome.error == CMD_ERR_FRAME) || (outcome.error == CMD_ERR_CHECKSUM) ||
        (outcome.error == CMD_ERR_SEQ) || (outcome.error == CMD_ERR_LENGTH))
    {
        /* The content cannot be trusted, so neither the sequence number nor the verb is echoed. */
        build_nak(rsp, CMD_SEQ_UNKNOWN, UNKNOWN_VERB, outcome.error);
        return outcome;
    }

    if (dispatcher->has_last && (req.seq == dispatcher->last_seq))
    {
        *rsp = dispatcher->last_response;
        outcome.error = dispatcher->last_error;
        outcome.replayed = true;
        return outcome;
    }

    if (outcome.error == CMD_OK)
    {
        outcome.error = execute(dispatcher, &req, rsp);
    }
    if (outcome.error != CMD_OK)
    {
        /* A verb too long to store is left empty by the parser. */
        build_nak(rsp, req.seq, (req.verb[0] != '\0') ? req.verb : UNKNOWN_VERB, outcome.error);
    }

    dispatcher->has_last = true;
    dispatcher->last_seq = req.seq;
    dispatcher->last_error = outcome.error;
    dispatcher->last_response = *rsp;
    return outcome;
}

cmd_outcome_t cmd_dispatch_too_long(cmd_response_t *rsp)
{
    const cmd_outcome_t outcome = { CMD_ERR_LENGTH, CMD_SEQ_UNKNOWN, false };

    build_nak(rsp, CMD_SEQ_UNKNOWN, UNKNOWN_VERB, CMD_ERR_LENGTH);
    return outcome;
}
