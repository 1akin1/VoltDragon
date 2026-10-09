/**
 * @file can_msg.h
 * @brief Inter-node CAN messages: identifiers, layout, protection and sequence tracking.
 *
 * Every message is an 8-byte frame (see docs/can-messages.md):
 *
 *     byte 0..5   payload
 *     byte 6      sequence counter, per identifier, wraps at 255
 *     byte 7      CRC-8/SAE-J1850 over the identifier (2 bytes, big-endian) and bytes 0..6
 *
 * Including the identifier in the CRC means a frame that arrives under the
 * wrong identifier is rejected, as in AUTOSAR E2E Profile 1.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef CAN_MSG_H
#define CAN_MSG_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

/* Node A -> Node B, 50 Hz. Node B accepts 0x100..0x10F. */
#define CANMSG_ID_A_STATUS      (0x100U)
#define CANMSG_ID_A_ACCEL       (0x101U)
#define CANMSG_ID_A_GYRO        (0x102U)
#define CANMSG_ID_A_MAG         (0x103U)
#define CANMSG_ID_A_GPS_LAT     (0x104U)
#define CANMSG_ID_A_GPS_LON     (0x105U)
#define CANMSG_ID_A_NAV         (0x106U)
#define CANMSG_NODE_A_ID_BASE   (0x100U)
#define CANMSG_NODE_A_ID_MASK   (0x7F0U)

#define CANMSG_DLC              (8U)
#define CANMSG_PAYLOAD_LEN      (6U)
#define CANMSG_SEQ_BYTE         (6U)
#define CANMSG_CRC_BYTE         (7U)

/** Gyroscope values travel in units of 10 mdps (0.01 dps) to fit 16 bits. */
#define CANMSG_GYRO_DIVISOR     (10)

/* STATUS flags. */
#define CANMSG_STATUS_IMU_VALID     (0x01U)
#define CANMSG_STATUS_RECORDER_OK   (0x02U)

/* NAV flags. */
#define CANMSG_NAV_MAG_OK           (0x01U)     /**< Magnetometer field within limits. */
#define CANMSG_NAV_GPS_FIX          (0x02U)     /**< Fresh GPS position fix. */
#define CANMSG_NAV_SOURCE_SHIFT     (2U)        /**< Heading source: 0 none, 1 magnetometer, 2 GPS. */
#define CANMSG_NAV_SOURCE_MASK      (0x0CU)

typedef struct
{
    uint8_t  flags;
    uint8_t  reset_count;       /**< Saturates at 255. */
    uint32_t uptime_ms;
} canmsg_status_t;

/** GPS position, carried by GPS_LAT and GPS_LON. */
typedef struct
{
    int32_t lat_e7;             /**< Degrees x 1e7. */
    int32_t lon_e7;             /**< Degrees x 1e7. */
    int16_t alt_msl_dm;         /**< Altitude above mean sea level, 0.1 m; saturates. */
    uint8_t quality;            /**< NMEA GGA fix quality. */
    uint8_t satellites;
} canmsg_gps_t;

typedef struct
{
    uint16_t heading_cdeg;      /**< True heading, 0.01 deg. */
    uint16_t field_mgauss;      /**< Measured magnetic field strength. */
    uint8_t  flags;             /**< CANMSG_NAV_* */
    uint8_t  speed_dmps;        /**< GPS ground speed, 0.1 m/s; saturates at 25.5 m/s. */
} canmsg_nav_t;

typedef struct
{
    bool    synced;
    uint8_t expected;
} canmsg_seq_tracker_t;

/** Fills a frame: identifier, DLC 8, payload, sequence counter and CRC. */
void canmsg_seal(can_frame_t *frame, uint16_t id, const uint8_t *payload, uint8_t seq);

/** True if the frame has DLC 8 and a matching CRC for its identifier. */
bool canmsg_valid(const can_frame_t *frame);

/** Sequence counter of a frame. */
uint8_t canmsg_seq(const can_frame_t *frame);

/** Encodes three values divided by @p divisor as saturated little-endian int16. */
void canmsg_encode_vec3(uint8_t *payload, const int32_t *values, int32_t divisor);

/** Decodes three little-endian int16 values multiplied by @p multiplier. */
void canmsg_decode_vec3(const uint8_t *payload, int32_t *values, int32_t multiplier);

void canmsg_encode_status(uint8_t *payload, const canmsg_status_t *status);
void canmsg_decode_status(const uint8_t *payload, canmsg_status_t *status);

/** GPS_LAT payload: latitude (int32), fix quality, satellites. */
void canmsg_encode_gps_lat(uint8_t *payload, const canmsg_gps_t *gps);
/** GPS_LON payload: longitude (int32), altitude (int16 dm). */
void canmsg_encode_gps_lon(uint8_t *payload, const canmsg_gps_t *gps);
/** Updates the fields of @p gps carried by a GPS_LAT payload. */
void canmsg_decode_gps_lat(const uint8_t *payload, canmsg_gps_t *gps);
/** Updates the fields of @p gps carried by a GPS_LON payload. */
void canmsg_decode_gps_lon(const uint8_t *payload, canmsg_gps_t *gps);

/** NAV payload: heading (uint16), field (uint16), flags, speed. */
void canmsg_encode_nav(uint8_t *payload, const canmsg_nav_t *nav);
void canmsg_decode_nav(const uint8_t *payload, canmsg_nav_t *nav);

/**
 * Updates the tracker with a received sequence counter and returns how many
 * frames were missed before it (0 when in order). The first frame, and a jump
 * backwards (a duplicate or a sender restart), resynchronise without counting
 * losses.
 */
uint32_t canmsg_track_seq(canmsg_seq_tracker_t *tracker, uint8_t seq);

#endif /* CAN_MSG_H */
