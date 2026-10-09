/**
 * @file cmd_protocol.c
 * @brief Operator command framing: line assembly, request parsing and response building.
 */
#include "cmd_protocol.h"

#include <string.h>

#define FRAME_START     ('$')
#define FIELD_SEP       (',')
#define CHECKSUM_SEP    ('*')
#define SEQ_MAX         (65535UL)
#define SEQ_DIGITS_MAX  (5U)
#define U32_DIGITS_MAX  (10U)
/* Room kept at the end of a response for "*CK\r\n" and the NUL. */
#define RESPONSE_TAIL   (6U)
/* Smallest frame: "$" + "*" + two checksum digits. */
#define FRAME_MIN_LEN   (4U)

static bool is_printable(char c)
{
    return (c >= ' ') && (c <= '~');
}

static bool is_digit(char c)
{
    return (c >= '0') && (c <= '9');
}

static bool hex_value(char c, uint8_t *value)
{
    if (is_digit(c))
    {
        *value = (uint8_t)(c - '0');
    }
    else if ((c >= 'A') && (c <= 'F'))
    {
        *value = (uint8_t)((c - 'A') + 10);
    }
    else if ((c >= 'a') && (c <= 'f'))
    {
        *value = (uint8_t)((c - 'a') + 10);
    }
    else
    {
        return false;
    }
    return true;
}

static size_t bounded_length(const char *text, size_t max)
{
    size_t len = 0U;

    while ((len <= max) && (text[len] != '\0'))
    {
        ++len;
    }
    return len;
}

uint8_t cmd_checksum(const char *data, size_t len)
{
    uint8_t sum = 0U;

    for (size_t i = 0U; i < len; ++i)
    {
        sum ^= (uint8_t)data[i];
    }
    return sum;
}

void cmd_line_reset(cmd_line_t *line)
{
    line->len = 0U;
    line->overflow = false;
    line->complete = false;
    line->buf[0] = '\0';
}

cmd_line_event_t cmd_line_feed(cmd_line_t *line, char c)
{
    if (line->complete)
    {
        cmd_line_reset(line);
    }

    if ((c == '\r') || (c == '\n'))
    {
        cmd_line_event_t event = CMD_LINE_PENDING;

        if (line->overflow)
        {
            event = CMD_LINE_TOO_LONG;
        }
        else if (line->len > 0U)
        {
            event = CMD_LINE_READY;
        }
        else
        {
            /* Empty line, e.g. the LF of a CR LF pair. */
        }
        line->buf[line->len] = '\0';
        line->complete = (event != CMD_LINE_PENDING);
        if (!line->complete)
        {
            cmd_line_reset(line);
        }
        return event;
    }

    if (c == FRAME_START)
    {
        /* Resynchronise: whatever came before this frame start is noise. */
        cmd_line_reset(line);
    }
    if (line->overflow)
    {
        return CMD_LINE_PENDING;
    }
    if (line->len >= CMD_LINE_MAX)
    {
        line->overflow = true;
        return CMD_LINE_PENDING;
    }
    line->buf[line->len] = c;
    line->len++;
    return CMD_LINE_PENDING;
}

bool cmd_parse_u32(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    size_t i = 0U;

    for (; text[i] != '\0'; ++i)
    {
        if ((i >= U32_DIGITS_MAX) || !is_digit(text[i]))
        {
            return false;
        }
        const uint32_t digit = (uint32_t)(text[i] - '0');
        if (result > ((UINT32_MAX - digit) / 10U))
        {
            return false;
        }
        result = (result * 10U) + digit;
    }
    if (i == 0U)
    {
        return false;
    }
    *value = result;
    return true;
}

/** Copies a field of @p len bytes into @p dest if it fits in @p max characters. */
static bool copy_field(char *dest, size_t max, const char *src, size_t len)
{
    if ((len == 0U) || (len > max))
    {
        return false;
    }
    (void)memcpy(dest, src, len);
    dest[len] = '\0';
    return true;
}

static bool parse_seq(const char *field, size_t len, uint16_t *seq)
{
    char digits[SEQ_DIGITS_MAX + 1U];
    uint32_t value = 0U;

    if (!copy_field(digits, SEQ_DIGITS_MAX, field, len) || !cmd_parse_u32(digits, &value) ||
        (value == 0U) || (value > SEQ_MAX))
    {
        return false;
    }
    *seq = (uint16_t)value;
    return true;
}

/** Checks the frame structure and checksum. On success returns the body between '$' and '*'. */
static cmd_error_t check_frame(const char *line, const char **body, size_t *body_len)
{
    const size_t len = bounded_length(line, CMD_LINE_MAX);
    uint8_t hi = 0U;
    uint8_t lo = 0U;

    if (len > CMD_LINE_MAX)
    {
        return CMD_ERR_LENGTH;
    }
    if ((len < FRAME_MIN_LEN) || (line[0] != FRAME_START) || (line[len - 3U] != CHECKSUM_SEP) ||
        !hex_value(line[len - 2U], &hi) || !hex_value(line[len - 1U], &lo))
    {
        return CMD_ERR_FRAME;
    }

    *body = &line[1];
    *body_len = len - 4U;
    for (size_t i = 0U; i < *body_len; ++i)
    {
        const char c = (*body)[i];
        if (!is_printable(c) || (c == FRAME_START) || (c == CHECKSUM_SEP))
        {
            return CMD_ERR_FRAME;
        }
    }

    const uint8_t expected = (uint8_t)(((uint32_t)hi << 4) | lo);
    return (cmd_checksum(*body, *body_len) == expected) ? CMD_OK : CMD_ERR_CHECKSUM;
}

cmd_error_t cmd_parse(const char *line, cmd_request_t *req)
{
    const char *body = 0;
    size_t body_len = 0U;

    (void)memset(req, 0, sizeof(*req));

    cmd_error_t error = check_frame(line, &body, &body_len);
    if (error != CMD_OK)
    {
        return error;
    }

    /* Split the body into fields: seq, verb, then arguments. */
    size_t start = 0U;
    uint32_t field = 0U;

    for (size_t i = 0U; i <= body_len; ++i)
    {
        if ((i < body_len) && (body[i] != FIELD_SEP))
        {
            continue;
        }

        const char *text = &body[start];
        const size_t text_len = i - start;

        if (field == 0U)
        {
            if (!parse_seq(text, text_len, &req->seq))
            {
                return CMD_ERR_SEQ;
            }
        }
        else if (field == 1U)
        {
            if (text_len == 0U)
            {
                req->seq = CMD_SEQ_UNKNOWN;
                return CMD_ERR_FRAME;
            }
            if (!copy_field(req->verb, CMD_VERB_MAX, text, text_len))
            {
                return CMD_ERR_UNKNOWN;
            }
        }
        else if (req->argc >= CMD_MAX_ARGS)
        {
            return CMD_ERR_ARGS;
        }
        else
        {
            if (!copy_field(req->args[req->argc], CMD_ARG_MAX, text, text_len))
            {
                return CMD_ERR_ARGS;
            }
            req->argc++;
        }
        field++;
        start = i + 1U;
    }

    if (field < 2U)
    {
        /* A frame with a sequence number but no verb. */
        req->seq = CMD_SEQ_UNKNOWN;
        return CMD_ERR_FRAME;
    }
    return CMD_OK;
}

const char *cmd_error_name(cmd_error_t error)
{
    switch (error)
    {
        case CMD_OK:
            return "OK";
        case CMD_ERR_FRAME:
            return "FRAME";
        case CMD_ERR_CHECKSUM:
            return "CHECKSUM";
        case CMD_ERR_SEQ:
            return "SEQ";
        case CMD_ERR_LENGTH:
            return "LENGTH";
        case CMD_ERR_UNKNOWN:
            return "UNKNOWN";
        case CMD_ERR_ARGS:
            return "ARGS";
        default:
            return "?";
    }
}

static void append_char(cmd_response_t *rsp, char c)
{
    if (rsp->len < (CMD_RESPONSE_MAX - RESPONSE_TAIL))
    {
        rsp->buf[rsp->len] = c;
        rsp->len++;
    }
    else
    {
        rsp->truncated = true;
    }
}

static void append_text(cmd_response_t *rsp, const char *text)
{
    for (const char *p = text; *p != '\0'; ++p)
    {
        append_char(rsp, *p);
    }
}

static void append_u32(cmd_response_t *rsp, uint32_t value)
{
    char digits[U32_DIGITS_MAX];
    uint32_t count = 0U;
    uint32_t v = value;

    do
    {
        digits[count] = (char)('0' + (char)(v % 10U));
        v /= 10U;
        count++;
    } while ((v != 0U) && (count < U32_DIGITS_MAX));

    while (count > 0U)
    {
        count--;
        append_char(rsp, digits[count]);
    }
}

void cmd_response_begin(cmd_response_t *rsp, uint16_t seq, bool ack, const char *verb)
{
    rsp->len = 0U;
    rsp->truncated = false;
    append_char(rsp, FRAME_START);
    append_u32(rsp, seq);
    append_char(rsp, FIELD_SEP);
    append_text(rsp, ack ? "ACK" : "NAK");
    append_char(rsp, FIELD_SEP);
    append_text(rsp, verb);
}

void cmd_response_add(cmd_response_t *rsp, const char *field)
{
    append_char(rsp, FIELD_SEP);
    append_text(rsp, field);
}

void cmd_response_add_u32(cmd_response_t *rsp, uint32_t value)
{
    append_char(rsp, FIELD_SEP);
    append_u32(rsp, value);
}

void cmd_response_end(cmd_response_t *rsp)
{
    static const char hex[] = "0123456789ABCDEF";
    /* The checksum covers everything after the '$'. */
    const uint8_t sum = cmd_checksum(&rsp->buf[1], rsp->len - 1U);

    /* RESPONSE_TAIL keeps room for these five characters and the NUL. */
    rsp->buf[rsp->len] = CHECKSUM_SEP;
    rsp->buf[rsp->len + 1U] = hex[sum >> 4];
    rsp->buf[rsp->len + 2U] = hex[sum & 0x0FU];
    rsp->buf[rsp->len + 3U] = '\r';
    rsp->buf[rsp->len + 4U] = '\n';
    rsp->len += 5U;
    rsp->buf[rsp->len] = '\0';
}
