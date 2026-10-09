/**
 * @file can_msg.c
 * @brief Inter-node CAN messages: layout, protection and sequence tracking.
 */
#include "can_msg.h"

#include <string.h>

#include "crc8.h"

/* A forward jump of half the counter range or more is treated as going backwards. */
#define SEQ_HALF_RANGE  (128U)
#define AXES            (3U)

static uint8_t frame_crc(const can_frame_t *frame)
{
    const uint8_t id_bytes[2] = { (uint8_t)(frame->id >> 8), (uint8_t)frame->id };
    uint8_t crc = crc8_update(CRC8_INIT, id_bytes, sizeof(id_bytes));

    crc = crc8_update(crc, frame->data, CANMSG_CRC_BYTE);
    return crc8_final(crc);
}

static int16_t saturate16(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

void canmsg_seal(can_frame_t *frame, uint16_t id, const uint8_t *payload, uint8_t seq)
{
    frame->id = id;
    frame->dlc = CANMSG_DLC;
    (void)memcpy(frame->data, payload, CANMSG_PAYLOAD_LEN);
    frame->data[CANMSG_SEQ_BYTE] = seq;
    frame->data[CANMSG_CRC_BYTE] = frame_crc(frame);
}

bool canmsg_valid(const can_frame_t *frame)
{
    return (frame->dlc == CANMSG_DLC) && (frame->data[CANMSG_CRC_BYTE] == frame_crc(frame));
}

uint8_t canmsg_seq(const can_frame_t *frame)
{
    return frame->data[CANMSG_SEQ_BYTE];
}

void canmsg_encode_vec3(uint8_t *payload, const int32_t *values, int32_t divisor)
{
    for (uint32_t axis = 0U; axis < AXES; ++axis)
    {
        const uint16_t raw = (uint16_t)saturate16(values[axis] / divisor);

        payload[2U * axis] = (uint8_t)raw;
        payload[(2U * axis) + 1U] = (uint8_t)(raw >> 8);
    }
}

void canmsg_decode_vec3(const uint8_t *payload, int32_t *values, int32_t multiplier)
{
    for (uint32_t axis = 0U; axis < AXES; ++axis)
    {
        const uint16_t raw = (uint16_t)((uint32_t)payload[2U * axis] |
                                        ((uint32_t)payload[(2U * axis) + 1U] << 8));

        values[axis] = (int32_t)(int16_t)raw * multiplier;
    }
}

void canmsg_encode_status(uint8_t *payload, const canmsg_status_t *status)
{
    payload[0] = status->flags;
    payload[1] = status->reset_count;
    payload[2] = (uint8_t)status->uptime_ms;
    payload[3] = (uint8_t)(status->uptime_ms >> 8);
    payload[4] = (uint8_t)(status->uptime_ms >> 16);
    payload[5] = (uint8_t)(status->uptime_ms >> 24);
}

void canmsg_decode_status(const uint8_t *payload, canmsg_status_t *status)
{
    status->flags = payload[0];
    status->reset_count = payload[1];
    status->uptime_ms = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8) |
                        ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);
}

uint32_t canmsg_track_seq(canmsg_seq_tracker_t *tracker, uint8_t seq)
{
    uint32_t lost = 0U;

    if (tracker->synced)
    {
        const uint32_t gap = (uint32_t)(uint8_t)(seq - tracker->expected);

        lost = (gap < SEQ_HALF_RANGE) ? gap : 0U;
    }
    tracker->synced = true;
    tracker->expected = (uint8_t)(seq + 1U);
    return lost;
}
