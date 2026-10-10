/**
 * @file tlm_msg.h
 * @brief Telemetry packet sent by Node B to the ground station over UDP (HLR-012, HLR-013).
 *
 * Fixed 88-byte little-endian packet, version 4 (see docs/telemetry.md):
 *
 *     0  "VDTM" magic          42  GPS satellites
 *     4  version (4)           43  GPS fix quality
 *     5  flags                 44  CAN frames valid
 *     6  length (80)           48  CAN frames rejected
 *     8  sequence number       52  CAN frames lost
 *    12  Node B uptime ms      56  latitude, deg x 1e7
 *    16  Node A uptime ms      60  longitude, deg x 1e7
 *    20  Node A reset count    64  altitude MSL, dm
 *    21  Node B reset count    66  heading, 0.01 deg
 *    22  Node A data age ms    68  magnetic field, mgauss
 *    24  accel X/Y/Z, mg       70  ground speed, dm/s
 *    30  gyro X/Y/Z, 10 mdps   72  GPS data age ms
 *    36  mag X/Y/Z, mgauss     74  distance to conductor, dm
 *                              76  battery, %
 *                              77  flight mode
 *                              78  safety flags (CAN SAFETY)
 *                              79  last mode request id
 *                              80  ground link age ms
 *                              82  vibration alarm (HLR-009)
 *                              83  vibration fault score, %
 *                              84  CRC-32 of bytes 0..83
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef TLM_MSG_H
#define TLM_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TLM_PACKET_LEN          (88U)
#define TLM_VERSION             (4U)
#define TLM_UNKNOWN_U8          (0xFFU)     /**< Battery or mode not known. */
#define TLM_UNKNOWN_U16         (0xFFFFU)   /**< Distance not known; ages saturate here. */

#define TLM_FLAG_NODE_A_FRESH   (0x01U)     /**< Node A data younger than TLM_FRESH_MS. */
#define TLM_FLAG_IMU_VALID      (0x02U)
#define TLM_FLAG_RECORDER_OK    (0x04U)
#define TLM_FLAG_GPS_FIX        (0x08U)     /**< Position younger than TLM_GPS_FRESH_MS. */
#define TLM_FLAG_MAG_OK         (0x10U)     /**< Magnetometer field within limits. */
#define TLM_HEADING_SOURCE_SHIFT (5U)       /**< 0 none, 1 magnetometer, 2 GPS course. */
#define TLM_HEADING_SOURCE_MASK (0x60U)
#define TLM_FLAG_VIB_ACTIVE     (0x80U)     /**< Node A's vibration monitor is classifying. */
#define TLM_FRESH_MS            (100UL)
#define TLM_GPS_FRESH_MS        (1000UL)

typedef struct
{
    uint32_t seq;
    uint8_t  flags;
    uint32_t node_b_uptime_ms;
    uint32_t node_a_uptime_ms;
    uint8_t  node_a_resets;
    uint8_t  node_b_resets;
    uint16_t node_a_age_ms;     /**< Saturates at 65535. */
    int32_t  accel_mg[3];
    int32_t  gyro_mdps[3];      /**< Sent in units of 10 mdps. */
    int32_t  mag_mgauss[3];
    uint8_t  gps_satellites;
    uint8_t  gps_quality;
    uint32_t can_valid;
    uint32_t can_rejected;
    uint32_t can_lost;
    int32_t  lat_e7;
    int32_t  lon_e7;
    int16_t  alt_msl_dm;
    uint16_t heading_cdeg;
    uint16_t field_mgauss;
    uint16_t speed_dmps;
    uint16_t gps_age_ms;        /**< Saturates at 65535. */
    uint16_t distance_dm;       /**< Onboard distance to the nearest conductor; TLM_UNKNOWN_U16. */
    uint8_t  battery_pct;       /**< TLM_UNKNOWN_U8 if the autopilot is silent. */
    uint8_t  flight_mode;       /**< flight_mode_t; TLM_UNKNOWN_U8 before Node A reports it. */
    uint8_t  safety_flags;      /**< CANMSG_SAFETY_* as Node A sent them. */
    uint8_t  last_request_id;   /**< Last operator mode request Node A processed. */
    uint16_t ground_link_age_ms; /**< Since the last ground station command; saturates. */
    uint8_t  vibration_alarm;   /**< CANMSG_VIB_* alarm; TLM_UNKNOWN_U8 before Node A reports it. */
    uint8_t  fault_score_pct;   /**< 100 minus P(nominal) of the latest window; TLM_UNKNOWN_U8. */
} tlm_packet_t;

/** Serialises a packet. Returns TLM_PACKET_LEN, or 0 if @p size is too small. */
size_t tlm_encode(const tlm_packet_t *packet, uint8_t *out, size_t size);

/** Parses and checks (length, magic, version, CRC) a packet. Returns false if it is not valid. */
bool tlm_decode(const uint8_t *in, size_t len, tlm_packet_t *packet);

#endif /* TLM_MSG_H */
