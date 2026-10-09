/**
 * @file tlm_msg.h
 * @brief Telemetry packet sent by Node B to the ground station over UDP (HLR-012, HLR-013).
 *
 * Fixed 60-byte little-endian packet (see docs/telemetry.md):
 *
 *     0  "VDTM" magic          24  accel X/Y/Z, int16 mg
 *     4  version (1)           30  gyro X/Y/Z, int16 10 mdps
 *     5  flags                 36  mag X/Y/Z, int16 mgauss
 *     6  length (60)           42  reserved (0)
 *     8  sequence number       44  CAN frames valid
 *    12  Node B uptime ms      48  CAN frames rejected
 *    16  Node A uptime ms      52  CAN frames lost
 *    20  Node A reset count    56  CRC-32 of bytes 0..55
 *    21  Node B reset count
 *    22  Node A data age ms
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef TLM_MSG_H
#define TLM_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TLM_PACKET_LEN          (60U)
#define TLM_VERSION             (1U)

#define TLM_FLAG_NODE_A_FRESH   (0x01U)     /**< Node A data younger than TLM_FRESH_MS. */
#define TLM_FLAG_IMU_VALID      (0x02U)
#define TLM_FLAG_RECORDER_OK    (0x04U)
#define TLM_FRESH_MS            (100UL)

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
    uint32_t can_valid;
    uint32_t can_rejected;
    uint32_t can_lost;
} tlm_packet_t;

/** Serialises a packet. Returns TLM_PACKET_LEN, or 0 if @p size is too small. */
size_t tlm_encode(const tlm_packet_t *packet, uint8_t *out, size_t size);

/** Parses and checks (length, magic, version, CRC) a packet. Returns false if it is not valid. */
bool tlm_decode(const uint8_t *in, size_t len, tlm_packet_t *packet);

#endif /* TLM_MSG_H */
