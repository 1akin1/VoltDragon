/**
 * @file ap_msg.c
 * @brief Messages between Node A and the autopilot.
 */
#include "ap_msg.h"

#include <string.h>

#include "cmd_protocol.h"

#define MAX_FIELDS      (4U)
#define FIELD_MAX       (12U)
#define BATTERY_MAX     (100UL)
#define DISTANCE_MAX    (65535UL)

static const char *const s_link_names[FLIGHT_MODE_COUNT] = { "MISSION", "HOLD", "RTH", "LAND" };

const char *ap_mode_name(flight_mode_t mode)
{
    return (mode < FLIGHT_MODE_COUNT) ? s_link_names[mode] : "?";
}

static bool mode_from_link_name(const char *name, flight_mode_t *mode)
{
    for (uint32_t i = 0U; i < (uint32_t)FLIGHT_MODE_COUNT; ++i)
    {
        if (strcmp(name, s_link_names[i]) == 0)
        {
            *mode = (flight_mode_t)i;
            return true;
        }
    }
    return false;
}

static void append(char *out, size_t size, size_t *len, const char *text)
{
    for (const char *p = text; (*p != '\0') && ((*len + 1U) < size); ++p)
    {
        out[*len] = *p;
        (*len)++;
    }
    out[*len] = '\0';
}

static void append_u32(char *out, size_t size, size_t *len, uint32_t value)
{
    char digits[11];
    uint32_t n = 0U;
    uint32_t v = value;

    do
    {
        digits[n] = (char)('0' + (char)(v % 10U));
        v /= 10U;
        n++;
    } while (v != 0U);

    char text[11];
    for (uint32_t i = 0U; i < n; ++i)
    {
        text[i] = digits[n - 1U - i];
    }
    text[n] = '\0';
    append(out, size, len, text);
}

size_t ap_format_command(const ap_command_t *cmd, char *out, size_t size)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t len = 0U;

    if (size < AP_MSG_MAX_LEN)
    {
        return 0U;
    }
    append(out, size, &len, "$VDCMD,");
    append(out, size, &len, ap_mode_name(cmd->mode));
    append(out, size, &len, cmd->avoid ? ",1," : ",0,");
    append_u32(out, size, &len, cmd->min_distance_dm);

    const uint8_t sum = cmd_checksum(&out[1], len - 1U);
    const char tail[] = { '*', hex[sum >> 4], hex[sum & 0x0FU], '\r', '\n', '\0' };
    append(out, size, &len, tail);
    return len;
}

/** Checks "$<body>*hh" and splits the body into fields. Returns the number of fields, 0 if invalid. */
static uint32_t split_frame(const char *line, char fields[MAX_FIELDS][FIELD_MAX + 1U])
{
    const size_t len = strlen(line);
    uint32_t value = 0U;

    if ((len < 4U) || (len >= AP_MSG_MAX_LEN) || (line[0] != '$') || (line[len - 3U] != '*'))
    {
        return 0U;
    }
    for (size_t i = len - 2U; i < len; ++i)
    {
        const char c = line[i];
        uint32_t digit;
        if ((c >= '0') && (c <= '9'))
        {
            digit = (uint32_t)(c - '0');
        }
        else if ((c >= 'A') && (c <= 'F'))
        {
            digit = (uint32_t)(c - 'A') + 10U;
        }
        else
        {
            return 0U;
        }
        value = (value << 4) | digit;
    }
    if (cmd_checksum(&line[1], len - 4U) != value)
    {
        return 0U;
    }

    uint32_t count = 0U;
    size_t field_len = 0U;
    for (size_t i = 1U; i <= (len - 3U); ++i)
    {
        const char c = line[i];
        if ((c == ',') || (c == '*'))
        {
            if (count >= MAX_FIELDS)
            {
                return 0U;
            }
            fields[count][field_len] = '\0';
            count++;
            field_len = 0U;
        }
        else if ((field_len < FIELD_MAX) && (count < MAX_FIELDS))
        {
            fields[count][field_len] = c;
            field_len++;
        }
        else
        {
            return 0U;
        }
    }
    return count;
}

bool ap_parse_status(const char *line, ap_status_t *status)
{
    char fields[MAX_FIELDS][FIELD_MAX + 1U];
    uint32_t battery = 0U;
    flight_mode_t mode = FLIGHT_MODE_MISSION;

    if ((split_frame(line, fields) != 3U) || (strcmp(fields[0], "VDAPS") != 0) ||
        !cmd_parse_u32(fields[1], &battery) || (battery > BATTERY_MAX) ||
        !mode_from_link_name(fields[2], &mode))
    {
        return false;
    }
    status->battery_pct = (uint8_t)battery;
    status->mode = mode;
    return true;
}

bool ap_parse_command(const char *line, ap_command_t *cmd)
{
    char fields[MAX_FIELDS][FIELD_MAX + 1U];
    uint32_t distance = 0U;
    flight_mode_t mode = FLIGHT_MODE_MISSION;

    if ((split_frame(line, fields) != 4U) || (strcmp(fields[0], "VDCMD") != 0) ||
        !mode_from_link_name(fields[1], &mode) ||
        ((strcmp(fields[2], "0") != 0) && (strcmp(fields[2], "1") != 0)) ||
        !cmd_parse_u32(fields[3], &distance) || (distance > DISTANCE_MAX))
    {
        return false;
    }
    cmd->mode = mode;
    cmd->avoid = fields[2][0] == '1';
    cmd->min_distance_dm = (uint16_t)distance;
    return true;
}
