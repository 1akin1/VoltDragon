/**
 * @file cmd_protocol.h
 * @brief Operator command framing: line assembly, request parsing and response building.
 *
 * Frame format (see docs/command-interface.md):
 *
 *     request:   $<seq>,<VERB>[,<arg>...]*<CK>
 *     response:  $<seq>,ACK,<VERB>[,<field>...]*<CK>
 *                $<seq>,NAK,<VERB>,<REASON>*<CK>
 *
 * CK is the XOR of every byte between '$' and '*', as two hex digits. Lines
 * end with CR, LF or both. This module has no hardware dependencies, so it
 * is unit-tested on the host (tests/unit).
 */
#ifndef CMD_PROTOCOL_H
#define CMD_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CMD_LINE_MAX        (80U)   /**< Longest request, '$' to checksum inclusive. */
#define CMD_RESPONSE_MAX    (96U)   /**< Longest response including "\r\n" and NUL. */
#define CMD_VERB_MAX        (12U)
#define CMD_ARG_MAX         (16U)
#define CMD_MAX_ARGS        (4U)
#define CMD_SEQ_UNKNOWN     (0U)    /**< Used in replies when the sequence number cannot be trusted. */

typedef enum
{
    CMD_OK = 0,
    CMD_ERR_FRAME,      /**< Malformed frame: no '$', bad checksum field, illegal character. */
    CMD_ERR_CHECKSUM,   /**< Checksum does not match the content. */
    CMD_ERR_SEQ,        /**< Sequence number missing or outside 1..65535. */
    CMD_ERR_LENGTH,     /**< Line longer than CMD_LINE_MAX. */
    CMD_ERR_UNKNOWN,    /**< Unknown verb. */
    CMD_ERR_ARGS        /**< Wrong number of arguments or an invalid argument value. */
} cmd_error_t;

/** Line assembler state. */
typedef struct
{
    char     buf[CMD_LINE_MAX + 1U];
    uint32_t len;
    bool     overflow;
    bool     complete;
} cmd_line_t;

typedef enum
{
    CMD_LINE_PENDING = 0,   /**< No complete line yet. */
    CMD_LINE_READY,         /**< buf holds a complete, NUL-terminated line without its ending. */
    CMD_LINE_TOO_LONG       /**< A line ended after exceeding CMD_LINE_MAX; its content was dropped. */
} cmd_line_event_t;

typedef struct
{
    uint16_t seq;
    char     verb[CMD_VERB_MAX + 1U];
    uint32_t argc;
    char     args[CMD_MAX_ARGS][CMD_ARG_MAX + 1U];
} cmd_request_t;

typedef struct
{
    char   buf[CMD_RESPONSE_MAX];
    size_t len;
    bool   truncated;
} cmd_response_t;

/** XOR of @p len bytes. */
uint8_t cmd_checksum(const char *data, size_t len);

/** Clears the line assembler. */
void cmd_line_reset(cmd_line_t *line);

/**
 * Adds one received byte. A '$' always starts a new frame, so a line resynchronises
 * on the next frame start after noise. Empty lines are ignored. After an event other
 * than CMD_LINE_PENDING, the line stays in buf until the next byte is fed.
 */
cmd_line_event_t cmd_line_feed(cmd_line_t *line, char c);

/**
 * Parses and validates a complete line. Fields are filled as far as parsing got,
 * so the sequence number is available for UNKNOWN and ARGS errors; it is
 * CMD_SEQ_UNKNOWN for FRAME, CHECKSUM and SEQ errors.
 */
cmd_error_t cmd_parse(const char *line, cmd_request_t *req);

/** Upper-case name of an error, as used in NAK responses. */
const char *cmd_error_name(cmd_error_t error);

/** Starts "$<seq>,ACK,<verb>" or "$<seq>,NAK,<verb>". */
void cmd_response_begin(cmd_response_t *rsp, uint16_t seq, bool ack, const char *verb);

/** Appends ",<field>". */
void cmd_response_add(cmd_response_t *rsp, const char *field);

/** Appends "," and an unsigned decimal number. */
void cmd_response_add_u32(cmd_response_t *rsp, uint32_t value);

/** Appends "*<CK>\r\n" and terminates the string. */
void cmd_response_end(cmd_response_t *rsp);

/** Parses a decimal number of 1 to 10 digits. Returns false on any other input or overflow. */
bool cmd_parse_u32(const char *text, uint32_t *value);

#endif /* CMD_PROTOCOL_H */
