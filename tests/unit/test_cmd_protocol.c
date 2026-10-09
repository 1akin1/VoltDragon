/**
 * @file test_cmd_protocol.c
 * @brief Host unit tests for cmd_protocol.c: checksum, line assembly, parsing, responses.
 */
#include <stdio.h>
#include <string.h>

#include "cmd_protocol.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

/** Builds "$<body>*<CK>" with the correct checksum. */
static const char *frame(const char *body)
{
    static char buf[256];
    const unsigned int sum = cmd_checksum(body, strlen(body));

    (void)snprintf(buf, sizeof(buf), "$%s*%02X", body, sum);
    return buf;
}

/** Feeds a string and returns the last event. */
static cmd_line_event_t feed(cmd_line_t *line, const char *text)
{
    cmd_line_event_t event = CMD_LINE_PENDING;

    for (const char *p = text; *p != '\0'; ++p)
    {
        event = cmd_line_feed(line, *p);
    }
    return event;
}

/* ---- checksum ---------------------------------------------------------- */

static void checksum_is_xor_of_bytes(void)
{
    CHECK_EQ(cmd_checksum("", 0U), 0x00);
    CHECK_EQ(cmd_checksum("A", 1U), 0x41);
    /* NMEA reference sentence: $GPGLL,5300.97914,N,00259.98174,E,125926,A*28 */
    const char *nmea = "GPGLL,5300.97914,N,00259.98174,E,125926,A";
    CHECK_EQ(cmd_checksum(nmea, strlen(nmea)), 0x28);
}

/* ---- line assembly ----------------------------------------------------- */

static void line_ends_on_cr_lf_or_both(void)
{
    cmd_line_t line;

    cmd_line_reset(&line);
    CHECK_EQ(feed(&line, "$1,PING*00\r"), CMD_LINE_READY);
    CHECK_STR(line.buf, "$1,PING*00");
    /* The LF of the CR LF pair is an empty line and is ignored. */
    CHECK_EQ(cmd_line_feed(&line, '\n'), CMD_LINE_PENDING);
    CHECK_EQ(feed(&line, "$2,PING*00\n"), CMD_LINE_READY);
    CHECK_STR(line.buf, "$2,PING*00");
}

static void line_ignores_empty_lines(void)
{
    cmd_line_t line;

    cmd_line_reset(&line);
    CHECK_EQ(feed(&line, "\r\n\r\n"), CMD_LINE_PENDING);
}

static void line_resynchronises_on_frame_start(void)
{
    cmd_line_t line;

    cmd_line_reset(&line);
    CHECK_EQ(feed(&line, "noise$$1,PING*00\n"), CMD_LINE_READY);
    CHECK_STR(line.buf, "$1,PING*00");
}

static void line_reports_overflow_once_at_line_end(void)
{
    cmd_line_t line;
    char long_line[CMD_LINE_MAX + 2U];

    cmd_line_reset(&line);
    (void)memset(long_line, 'X', CMD_LINE_MAX + 1U);
    long_line[CMD_LINE_MAX + 1U] = '\0';
    CHECK_EQ(feed(&line, long_line), CMD_LINE_PENDING);
    CHECK_EQ(cmd_line_feed(&line, '\n'), CMD_LINE_TOO_LONG);
    /* The next line is assembled normally. */
    CHECK_EQ(feed(&line, "$3,PING*00\n"), CMD_LINE_READY);
    CHECK_STR(line.buf, "$3,PING*00");
}

static void line_accepts_exactly_max_length(void)
{
    cmd_line_t line;
    char max_line[CMD_LINE_MAX + 1U];

    cmd_line_reset(&line);
    (void)memset(max_line, 'X', CMD_LINE_MAX);
    max_line[CMD_LINE_MAX] = '\0';
    CHECK_EQ(feed(&line, max_line), CMD_LINE_PENDING);
    CHECK_EQ(cmd_line_feed(&line, '\r'), CMD_LINE_READY);
    CHECK_EQ(strlen(line.buf), CMD_LINE_MAX);
}

/* ---- parsing ----------------------------------------------------------- */

static void parse_accepts_command_without_arguments(void)
{
    cmd_request_t req;

    CHECK_EQ(cmd_parse(frame("42,PING"), &req), CMD_OK);
    CHECK_EQ(req.seq, 42);
    CHECK_STR(req.verb, "PING");
    CHECK_EQ(req.argc, 0);
}

static void parse_accepts_arguments(void)
{
    cmd_request_t req;

    CHECK_EQ(cmd_parse(frame("65535,SET,a,bb,ccc,dddd"), &req), CMD_OK);
    CHECK_EQ(req.seq, 65535);
    CHECK_EQ(req.argc, 4);
    CHECK_STR(req.args[0], "a");
    CHECK_STR(req.args[3], "dddd");
}

static void parse_accepts_lower_case_checksum(void)
{
    cmd_request_t req;

    /* "1,X": 0x31 ^ 0x2C ^ 0x58 = 0x45 */
    CHECK_EQ(cmd_parse("$1,X*45", &req), CMD_OK);
    /* "1,PING" = 0x31^0x2C^0x50^0x49^0x4E^0x47 = 0x0D */
    CHECK_EQ(cmd_parse("$1,PING*0d", &req), CMD_OK);
}

static void parse_rejects_bad_frames(void)
{
    cmd_request_t req;

    CHECK_EQ(cmd_parse("", &req), CMD_ERR_FRAME);
    CHECK_EQ(cmd_parse("1,PING*0D", &req), CMD_ERR_FRAME);      /* no '$' */
    CHECK_EQ(cmd_parse("$1,PING", &req), CMD_ERR_FRAME);        /* no checksum */
    CHECK_EQ(cmd_parse("$1,PING*0", &req), CMD_ERR_FRAME);      /* one checksum digit */
    CHECK_EQ(cmd_parse("$1,PING*0G", &req), CMD_ERR_FRAME);     /* not hex */
    CHECK_EQ(cmd_parse("$1,PI*NG*0D", &req), CMD_ERR_FRAME);    /* '*' in the body */
    CHECK_EQ(cmd_parse("$1,PI\tNG*00", &req), CMD_ERR_FRAME);   /* control character */
    CHECK_EQ(cmd_parse(frame("1"), &req), CMD_ERR_FRAME);       /* no verb */
    CHECK_EQ(cmd_parse(frame("1,"), &req), CMD_ERR_FRAME);      /* empty verb */
    CHECK_EQ(req.seq, CMD_SEQ_UNKNOWN);
}

static void parse_rejects_wrong_checksum(void)
{
    cmd_request_t req;

    CHECK_EQ(cmd_parse("$1,PING*0E", &req), CMD_ERR_CHECKSUM);
    CHECK_EQ(req.seq, CMD_SEQ_UNKNOWN);
}

static void parse_rejects_invalid_sequence_numbers(void)
{
    cmd_request_t req;

    CHECK_EQ(cmd_parse(frame("0,PING"), &req), CMD_ERR_SEQ);
    CHECK_EQ(cmd_parse(frame("65536,PING"), &req), CMD_ERR_SEQ);
    CHECK_EQ(cmd_parse(frame("123456,PING"), &req), CMD_ERR_SEQ);
    CHECK_EQ(cmd_parse(frame("-1,PING"), &req), CMD_ERR_SEQ);
    CHECK_EQ(cmd_parse(frame("1a,PING"), &req), CMD_ERR_SEQ);
    CHECK_EQ(cmd_parse(frame(",PING"), &req), CMD_ERR_SEQ);
    CHECK_EQ(req.seq, CMD_SEQ_UNKNOWN);
}

static void parse_keeps_sequence_for_verb_and_argument_errors(void)
{
    cmd_request_t req;

    CHECK_EQ(cmd_parse(frame("7,ABCDEFGHIJKLM"), &req), CMD_ERR_UNKNOWN);   /* 13-char verb */
    CHECK_EQ(req.seq, 7);
    CHECK_EQ(cmd_parse(frame("8,SET,1,2,3,4,5"), &req), CMD_ERR_ARGS);     /* five arguments */
    CHECK_EQ(req.seq, 8);
    CHECK_EQ(cmd_parse(frame("9,SET,"), &req), CMD_ERR_ARGS);              /* empty argument */
    CHECK_EQ(cmd_parse(frame("9,SET,a,,b"), &req), CMD_ERR_ARGS);
    CHECK_EQ(cmd_parse(frame("9,SET,12345678901234567"), &req), CMD_ERR_ARGS);  /* 17 chars */
    CHECK_EQ(req.seq, 9);
}

static void parse_rejects_too_long_line(void)
{
    cmd_request_t req;
    char long_line[CMD_LINE_MAX + 2U];

    (void)memset(long_line, 'X', CMD_LINE_MAX + 1U);
    long_line[CMD_LINE_MAX + 1U] = '\0';
    CHECK_EQ(cmd_parse(long_line, &req), CMD_ERR_LENGTH);
}

static void parse_u32_handles_limits(void)
{
    uint32_t value = 0U;

    CHECK(cmd_parse_u32("0", &value));
    CHECK_EQ(value, 0);
    CHECK(cmd_parse_u32("4294967295", &value));
    CHECK_EQ(value, 4294967295LL);
    CHECK(!cmd_parse_u32("4294967296", &value));
    CHECK(!cmd_parse_u32("", &value));
    CHECK(!cmd_parse_u32("12x", &value));
    CHECK(!cmd_parse_u32("00000000001", &value));
}

/* ---- responses --------------------------------------------------------- */

static void response_has_valid_checksum(void)
{
    cmd_response_t rsp;
    cmd_request_t req;

    cmd_response_begin(&rsp, 12, true, "STATUS");
    cmd_response_add_u32(&rsp, 0);
    cmd_response_add_u32(&rsp, 4294967295U);
    cmd_response_add(&rsp, "x");
    cmd_response_end(&rsp);

    CHECK_STR(rsp.buf, "$12,ACK,STATUS,0,4294967295,x*37\r\n");
    CHECK(!rsp.truncated);
    /* A response is itself a well-formed frame. */
    rsp.buf[rsp.len - 2U] = '\0';
    CHECK_EQ(cmd_parse(rsp.buf, &req), CMD_OK);
}

static void response_is_truncated_safely(void)
{
    cmd_response_t rsp;

    cmd_response_begin(&rsp, 1, true, "X");
    for (int i = 0; i < 20; ++i)
    {
        cmd_response_add(&rsp, "0123456789");
    }
    cmd_response_end(&rsp);

    CHECK(rsp.truncated);
    CHECK(rsp.len < CMD_RESPONSE_MAX);
    CHECK_EQ(strlen(rsp.buf), rsp.len);
    CHECK_STR(&rsp.buf[rsp.len - 2U], "\r\n");
}

int main(void)
{
    RUN(checksum_is_xor_of_bytes);
    RUN(line_ends_on_cr_lf_or_both);
    RUN(line_ignores_empty_lines);
    RUN(line_resynchronises_on_frame_start);
    RUN(line_reports_overflow_once_at_line_end);
    RUN(line_accepts_exactly_max_length);
    RUN(parse_accepts_command_without_arguments);
    RUN(parse_accepts_arguments);
    RUN(parse_accepts_lower_case_checksum);
    RUN(parse_rejects_bad_frames);
    RUN(parse_rejects_wrong_checksum);
    RUN(parse_rejects_invalid_sequence_numbers);
    RUN(parse_keeps_sequence_for_verb_and_argument_errors);
    RUN(parse_rejects_too_long_line);
    RUN(parse_u32_handles_limits);
    RUN(response_has_valid_checksum);
    RUN(response_is_truncated_safely);
    return (unit_failures == 0) ? 0 : 1;
}
