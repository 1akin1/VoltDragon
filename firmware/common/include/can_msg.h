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

typedef struct
{
    uint8_t  flags;
    uint8_t  reset_count;       /**< Saturates at 255. */
    uint32_t uptime_ms;
} canmsg_status_t;

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

/**
 * Updates the tracker with a received sequence counter and returns how many
 * frames were missed before it (0 when in order). The first frame, and a jump
 * backwards (a duplicate or a sender restart), resynchronise without counting
 * losses.
 */
uint32_t canmsg_track_seq(canmsg_seq_tracker_t *tracker, uint8_t seq);

#endif /* CAN_MSG_H */
