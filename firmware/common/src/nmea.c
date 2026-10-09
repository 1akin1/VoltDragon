/**
 * @file nmea.c
 * @brief NMEA 0183 parser for GGA and RMC sentences, in integer fixed point.
 */
#include "nmea.h"

#include <stddef.h>
#include <string.h>

#include "cmd_protocol.h"

#define MAX_FIELDS          (20U)
#define MAX_LINE            (82U)
#define MAX_INT_DIGITS      (9U)
#define MINUTES_DECIMALS    (5U)
#define E7                  (10000000LL)
#define MINUTES_PER_DEGREE  (60LL)
/* U, not UL: unsigned long is 64 bits on the host where the unit tests run. */
#define MS_PER_HOUR         (3600000U)
#define MS_PER_MINUTE       (60000U)
#define MS_PER_SECOND       (1000U)
#define CDEG_FULL_CIRCLE    (36000LL)
/* 1 knot = 51.4444 cm/s, applied to knots x 1000 with 7 extra decimal places. */
#define CMPS_PER_KNOT_E4    (514444LL)
#define KNOT_SCALE          (10000000LL)

typedef struct
{
    const char *text;
    size_t      len;
} field_t;

static bool hex_digit(char c, uint8_t *value)
{
    if ((c >= '0') && (c <= '9'))
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

/** Checks "$<body>*hh" and the checksum. On success returns the body, else sets @p error. */
static bool check_frame(const char *line, const char **body, size_t *body_len,
                        nmea_result_t *error)
{
    size_t len = 0U;
    uint8_t hi = 0U;
    uint8_t lo = 0U;

    while ((len <= MAX_LINE) && (line[len] != '\0'))
    {
        ++len;
    }
    if ((len < 4U) || (len > MAX_LINE) || (line[0] != '$') || (line[len - 3U] != '*') ||
        !hex_digit(line[len - 2U], &hi) || !hex_digit(line[len - 1U], &lo))
    {
        *error = NMEA_ERR_FRAME;
        return false;
    }
    *body = &line[1];
    *body_len = len - 4U;
    const uint8_t expected = (uint8_t)(((uint32_t)hi << 4) | lo);
    if (cmd_checksum(*body, *body_len) != expected)
    {
        *error = NMEA_ERR_CHECKSUM;
        return false;
    }
    return true;
}

static uint32_t split(const char *body, size_t len, field_t *fields)
{
    uint32_t count = 0U;
    size_t start = 0U;

    for (size_t i = 0U; (i <= len) && (count < MAX_FIELDS); ++i)
    {
        if ((i == len) || (body[i] == ','))
        {
            fields[count].text = &body[start];
            fields[count].len = i - start;
            count++;
            start = i + 1U;
        }
    }
    return count;
}

/**
 * Parses "[-]digits[.digits]" into an integer scaled by 10^decimals. Extra
 * decimal digits are truncated; missing ones count as zero.
 */
static bool parse_fixed(field_t f, uint32_t decimals, int64_t *out)
{
    size_t i = 0U;
    bool negative = false;
    int64_t value = 0;
    uint32_t int_digits = 0U;
    uint32_t frac_digits = 0U;
    bool seen_point = false;

    if ((f.len > 0U) && (f.text[0] == '-'))
    {
        negative = true;
        i = 1U;
    }
    for (; i < f.len; ++i)
    {
        const char c = f.text[i];
        if ((c == '.') && !seen_point)
        {
            seen_point = true;
        }
        else if ((c >= '0') && (c <= '9'))
        {
            if (!seen_point)
            {
                if (++int_digits > MAX_INT_DIGITS)
                {
                    return false;
                }
                value = (value * 10) + (c - '0');
            }
            else if (frac_digits < decimals)
            {
                value = (value * 10) + (c - '0');
                frac_digits++;
            }
            else
            {
                /* Beyond the requested precision: truncated. */
            }
        }
        else
        {
            return false;
        }
    }
    if (int_digits == 0U)
    {
        return false;
    }
    for (; frac_digits < decimals; ++frac_digits)
    {
        value *= 10;
    }
    *out = negative ? -value : value;
    return true;
}

static bool parse_time(field_t f, uint32_t *ms)
{
    int64_t hhmmss_e3 = 0;

    if (!parse_fixed(f, 3U, &hhmmss_e3) || (hhmmss_e3 < 0))
    {
        return false;
    }
    const uint32_t whole = (uint32_t)(hhmmss_e3 / 1000);
    const uint32_t hours = whole / 10000U;
    const uint32_t minutes = (whole / 100U) % 100U;
    const uint32_t seconds = whole % 100U;
    if ((hours > 23U) || (minutes > 59U) || (seconds > 60U))
    {
        return false;
    }
    *ms = (hours * MS_PER_HOUR) + (minutes * MS_PER_MINUTE) + (seconds * MS_PER_SECOND) +
          (uint32_t)(hhmmss_e3 % 1000);
    return true;
}

/** Parses "(d)ddmm.mmmmm" plus a hemisphere letter into degrees x 1e7. */
static bool parse_coordinate(field_t value, field_t hemisphere, char positive, char negative,
                             int64_t max_degrees, int32_t *out)
{
    int64_t ddmm_e5 = 0;

    if (!parse_fixed(value, MINUTES_DECIMALS, &ddmm_e5) || (ddmm_e5 < 0) || (hemisphere.len != 1U))
    {
        return false;
    }
    /* ddmm.mmmmm x 1e5 = degrees x 1e7 + minutes x 1e5 */
    const int64_t degrees = ddmm_e5 / E7;
    const int64_t minutes_e5 = ddmm_e5 % E7;
    if ((degrees > max_degrees) || (minutes_e5 >= (MINUTES_PER_DEGREE * 100000LL)))
    {
        return false;
    }
    /* minutes / 60 in units of 1e-7 degrees = minutes_e5 x 100 / 60, rounded. */
    int64_t result = (degrees * E7) + (((minutes_e5 * 100LL) + (MINUTES_PER_DEGREE / 2)) /
                                       MINUTES_PER_DEGREE);
    if (hemisphere.text[0] == negative)
    {
        result = -result;
    }
    else if (hemisphere.text[0] != positive)
    {
        return false;
    }
    *out = (int32_t)result;
    return true;
}

static bool type_is(field_t f, const char *type)
{
    /* Accept any talker ID (GP, GN, GL, ...): compare the last three letters. */
    return (f.len == 5U) && (memcmp(&f.text[2], type, 3U) == 0);
}

static nmea_result_t parse_gga(const field_t *fields, uint32_t count, nmea_fix_t *fix)
{
    nmea_fix_t next = *fix;
    int64_t value = 0;

    if ((count < 10U) || !parse_time(fields[1], &next.utc_ms) ||
        !parse_fixed(fields[6], 0U, &value) || (value < 0) || (value > 9))
    {
        return NMEA_ERR_FIELD;
    }
    next.quality = (uint8_t)value;
    next.fix = next.quality > 0U;
    next.satellites = (parse_fixed(fields[7], 0U, &value) && (value >= 0) && (value < 100)) ?
                      (uint8_t)value : 0U;
    next.hdop_x10 = (parse_fixed(fields[8], 1U, &value) && (value >= 0) && (value < 10000)) ?
                    (uint16_t)value : 0U;

    if (next.fix)
    {
        if (!parse_coordinate(fields[2], fields[3], 'N', 'S', 90, &next.lat_e7) ||
            !parse_coordinate(fields[4], fields[5], 'E', 'W', 180, &next.lon_e7) ||
            !parse_fixed(fields[9], 2U, &value))
        {
            return NMEA_ERR_FIELD;
        }
        next.alt_msl_cm = (int32_t)value;
    }
    *fix = next;
    return NMEA_GGA;
}

static nmea_result_t parse_rmc(const field_t *fields, uint32_t count, nmea_fix_t *fix)
{
    nmea_fix_t next = *fix;
    int64_t value = 0;

    if ((count < 9U) || !parse_time(fields[1], &next.utc_ms) || (fields[2].len != 1U))
    {
        return NMEA_ERR_FIELD;
    }
    next.rmc_valid = fields[2].text[0] == 'A';
    if (next.rmc_valid)
    {
        if (!parse_fixed(fields[7], 3U, &value) || (value < 0))
        {
            return NMEA_ERR_FIELD;
        }
        next.speed_cmps = (uint32_t)((value * CMPS_PER_KNOT_E4) / KNOT_SCALE);
        /* Course is empty when the receiver is not moving: keep the last one. */
        if ((fields[8].len > 0U) && parse_fixed(fields[8], 2U, &value) && (value >= 0))
        {
            next.course_cdeg = (uint16_t)(value % CDEG_FULL_CIRCLE);
        }
    }
    *fix = next;
    return NMEA_RMC;
}

nmea_result_t nmea_parse(const char *line, nmea_fix_t *fix)
{
    const char *body = 0;
    size_t body_len = 0U;
    field_t fields[MAX_FIELDS];

    nmea_result_t error = NMEA_ERR_FRAME;
    if (!check_frame(line, &body, &body_len, &error))
    {
        return error;
    }

    const uint32_t count = split(body, body_len, fields);
    if (type_is(fields[0], "GGA"))
    {
        return parse_gga(fields, count, fix);
    }
    if (type_is(fields[0], "RMC"))
    {
        return parse_rmc(fields, count, fix);
    }
    return NMEA_IGNORED;
}

const char *nmea_result_name(nmea_result_t result)
{
    switch (result)
    {
        case NMEA_GGA:
            return "GGA";
        case NMEA_RMC:
            return "RMC";
        case NMEA_IGNORED:
            return "IGNORED";
        case NMEA_ERR_FRAME:
            return "FRAME";
        case NMEA_ERR_CHECKSUM:
            return "CHECKSUM";
        case NMEA_ERR_FIELD:
            return "FIELD";
        default:
            return "?";
    }
}
