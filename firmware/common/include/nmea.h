/**
 * @file nmea.h
 * @brief NMEA 0183 parser for GPS receivers: GGA (fix data) and RMC (minimum navigation data).
 *
 * Sentences use the same framing as operator commands ('$', fields separated by
 * commas, '*' and an XOR checksum), so lines are assembled with cmd_line_feed()
 * from cmd_protocol.h. All values are kept in integer fixed point; no floating
 * point is used.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef NMEA_H
#define NMEA_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    NMEA_GGA = 0,           /**< Fix data parsed. */
    NMEA_RMC,               /**< Navigation data parsed. */
    NMEA_IGNORED,           /**< Valid sentence of another type. */
    NMEA_ERR_FRAME,         /**< Not a '$...*hh' sentence. */
    NMEA_ERR_CHECKSUM,
    NMEA_ERR_FIELD          /**< A required field is missing or malformed. */
} nmea_result_t;

/** Latest navigation data, merged from GGA and RMC sentences. */
typedef struct
{
    bool     fix;                   /**< GGA fix quality is 1 or more. */
    bool     rmc_valid;             /**< RMC status is 'A'. */
    uint8_t  quality;               /**< GGA fix quality: 0 none, 1 GPS, 2 DGPS, ... */
    uint8_t  satellites;
    uint16_t hdop_x10;
    int32_t  lat_e7;                /**< Degrees x 1e7, north positive. */
    int32_t  lon_e7;                /**< Degrees x 1e7, east positive. */
    int32_t  alt_msl_cm;
    uint32_t utc_ms;                /**< Milliseconds since midnight UTC. */
    uint32_t speed_cmps;            /**< Ground speed, cm/s. */
    uint16_t course_cdeg;           /**< Course over ground, 0.01 deg from true north. */
} nmea_fix_t;

/**
 * Parses one complete sentence (without line ending) and updates @p fix with
 * the fields it carries. On any error @p fix is left unchanged.
 */
nmea_result_t nmea_parse(const char *line, nmea_fix_t *fix);

/** Short name of a result, for logging. */
const char *nmea_result_name(nmea_result_t result);

#endif /* NMEA_H */
