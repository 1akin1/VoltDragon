/**
 * @file test_cmd_dispatch.c
 * @brief Host unit tests for cmd_dispatch.c: lookup, argument checks, NAK reasons, replays.
 */
#include <stdio.h>
#include <string.h>

#include "cmd_dispatch.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

static int s_ping_calls;

static cmd_error_t handle_ping(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    (void)rsp;
    s_ping_calls++;
    return CMD_OK;
}

/* SET <value>: accepts 1..9 and echoes the value. */
static cmd_error_t handle_set(const cmd_request_t *req, cmd_response_t *rsp)
{
    uint32_t value = 0U;

    if (!cmd_parse_u32(req->args[0], &value) || (value < 1U) || (value > 9U))
    {
        return CMD_ERR_ARGS;
    }
    cmd_response_add_u32(rsp, value);
    return CMD_OK;
}

/* ARM: never allowed in this test, to exercise CMD_ERR_REFUSED. */
static cmd_error_t handle_arm(const cmd_request_t *req, cmd_response_t *rsp)
{
    (void)req;
    cmd_response_add(rsp, "ignored");
    return CMD_ERR_REFUSED;
}

static const cmd_entry_t s_table[] = {
    { "PING", 0U, 0U, handle_ping },
    { "SET", 1U, 1U, handle_set },
    { "ARM", 0U, 0U, handle_arm },
};

static const char *frame(const char *body)
{
    static char buf[256];
    const unsigned int sum = cmd_checksum(body, strlen(body));

    (void)snprintf(buf, sizeof(buf), "$%s*%02X", body, sum);
    return buf;
}

/** Response without the "*CK\r\n" tail, for readable comparisons. */
static const char *body_of(const cmd_response_t *rsp)
{
    static char buf[CMD_RESPONSE_MAX];

    (void)memcpy(buf, rsp->buf, rsp->len - 5U);
    buf[rsp->len - 5U] = '\0';
    return buf;
}

static cmd_dispatcher_t s_dispatcher;
static cmd_response_t s_rsp;

static cmd_outcome_t dispatch(const char *body)
{
    return cmd_dispatch_line(&s_dispatcher, frame(body), &s_rsp);
}

static void setup(void)
{
    cmd_dispatcher_init(&s_dispatcher, s_table, sizeof(s_table) / sizeof(s_table[0]));
    s_ping_calls = 0;
}

static void acknowledges_known_command(void)
{
    setup();
    const cmd_outcome_t out = dispatch("1,PING");
    CHECK_EQ(out.error, CMD_OK);
    CHECK_EQ(out.seq, 1);
    CHECK(!out.replayed);
    CHECK_STR(body_of(&s_rsp), "$1,ACK,PING");
    CHECK_EQ(s_ping_calls, 1);
}

static void handler_adds_result_fields(void)
{
    setup();
    CHECK_EQ(dispatch("2,SET,7").error, CMD_OK);
    CHECK_STR(body_of(&s_rsp), "$2,ACK,SET,7");
}

static void rejects_unknown_verb(void)
{
    setup();
    CHECK_EQ(dispatch("3,FLY").error, CMD_ERR_UNKNOWN);
    CHECK_STR(body_of(&s_rsp), "$3,NAK,FLY,UNKNOWN");
    /* Verbs are case-sensitive. */
    CHECK_EQ(dispatch("4,ping").error, CMD_ERR_UNKNOWN);
}

static void rejects_overlong_verb_without_echoing_it(void)
{
    setup();
    CHECK_EQ(dispatch("5,ABCDEFGHIJKLM").error, CMD_ERR_UNKNOWN);
    CHECK_STR(body_of(&s_rsp), "$5,NAK,-,UNKNOWN");
}

static void rejects_wrong_argument_count(void)
{
    setup();
    CHECK_EQ(dispatch("6,SET").error, CMD_ERR_ARGS);
    CHECK_STR(body_of(&s_rsp), "$6,NAK,SET,ARGS");
    CHECK_EQ(dispatch("7,PING,x").error, CMD_ERR_ARGS);
    CHECK_EQ(s_ping_calls, 0);
}

static void rejects_argument_refused_by_handler(void)
{
    setup();
    CHECK_EQ(dispatch("8,SET,10").error, CMD_ERR_ARGS);
    CHECK_STR(body_of(&s_rsp), "$8,NAK,SET,ARGS");
}

static void refused_request_gets_a_clean_nak(void)
{
    setup();
    CHECK_EQ(dispatch("20,ARM").error, CMD_ERR_REFUSED);
    /* Fields the handler added before refusing are not sent. */
    CHECK_STR(body_of(&s_rsp), "$20,NAK,ARM,REFUSED");
}

static void rejects_corrupt_frames_with_unknown_sequence(void)
{
    setup();
    CHECK_EQ(cmd_dispatch_line(&s_dispatcher, "$9,PING*00", &s_rsp).error, CMD_ERR_CHECKSUM);
    CHECK_STR(body_of(&s_rsp), "$0,NAK,-,CHECKSUM");
    CHECK_EQ(cmd_dispatch_line(&s_dispatcher, "hello", &s_rsp).error, CMD_ERR_FRAME);
    CHECK_STR(body_of(&s_rsp), "$0,NAK,-,FRAME");
    CHECK_EQ(dispatch("0,PING").error, CMD_ERR_SEQ);
    CHECK_STR(body_of(&s_rsp), "$0,NAK,-,SEQ");
    CHECK_EQ(cmd_dispatch_too_long(&s_rsp).error, CMD_ERR_LENGTH);
    CHECK_STR(body_of(&s_rsp), "$0,NAK,-,LENGTH");
}

static void replays_response_for_retransmitted_request(void)
{
    setup();
    CHECK_EQ(dispatch("10,PING").error, CMD_OK);
    const cmd_outcome_t again = dispatch("10,PING");
    CHECK(again.replayed);
    CHECK_EQ(again.error, CMD_OK);
    CHECK_STR(body_of(&s_rsp), "$10,ACK,PING");
    /* Executed only once. */
    CHECK_EQ(s_ping_calls, 1);
}

static void replays_nak_with_original_reason(void)
{
    setup();
    CHECK_EQ(dispatch("11,FLY").error, CMD_ERR_UNKNOWN);
    const cmd_outcome_t again = dispatch("11,FLY");
    CHECK(again.replayed);
    CHECK_EQ(again.error, CMD_ERR_UNKNOWN);
    CHECK_STR(body_of(&s_rsp), "$11,NAK,FLY,UNKNOWN");
}

static void corrupt_frame_does_not_disturb_replay_cache(void)
{
    setup();
    CHECK_EQ(dispatch("12,PING").error, CMD_OK);
    CHECK_EQ(cmd_dispatch_line(&s_dispatcher, "$12,PING*00", &s_rsp).error, CMD_ERR_CHECKSUM);
    CHECK(dispatch("12,PING").replayed);
    CHECK_EQ(s_ping_calls, 1);
}

static void different_request_with_same_sequence_is_executed(void)
{
    setup();
    CHECK_EQ(dispatch("15,PING").error, CMD_OK);
    /* Another sender happened to pick the same number: not a retransmission. */
    const cmd_outcome_t out = dispatch("15,SET,4");
    CHECK(!out.replayed);
    CHECK_EQ(out.error, CMD_OK);
    CHECK_STR(body_of(&s_rsp), "$15,ACK,SET,4");
}

static void new_sequence_number_executes_again(void)
{
    setup();
    CHECK_EQ(dispatch("13,PING").error, CMD_OK);
    CHECK_EQ(dispatch("14,PING").error, CMD_OK);
    CHECK_EQ(dispatch("13,PING").error, CMD_OK);
    CHECK_EQ(s_ping_calls, 3);
}

int main(void)
{
    RUN(acknowledges_known_command);
    RUN(handler_adds_result_fields);
    RUN(rejects_unknown_verb);
    RUN(rejects_overlong_verb_without_echoing_it);
    RUN(rejects_wrong_argument_count);
    RUN(rejects_argument_refused_by_handler);
    RUN(refused_request_gets_a_clean_nak);
    RUN(rejects_corrupt_frames_with_unknown_sequence);
    RUN(replays_response_for_retransmitted_request);
    RUN(replays_nak_with_original_reason);
    RUN(corrupt_frame_does_not_disturb_replay_cache);
    RUN(different_request_with_same_sequence_is_executed);
    RUN(new_sequence_number_executes_again);
    return (unit_failures == 0) ? 0 : 1;
}
