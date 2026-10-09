/**
 * @file tlm_msg.c
 * @brief Telemetry packet encoding and decoding.
 */
#include "tlm_msg.h"

#include <string.h>

#include "crc32.h"

#define GYRO_DIVISOR    (10)
#define CRC_OFFSET      (56U)
#define AXES            (3U)

static const uint8_t MAGIC[4] = { 'V', 'D', 'T', 'M' };

static void put_u16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, v);
    put_u16(&p[2], v >> 16);
}

static uint32_t get_u16(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p)
{
    return get_u16(p) | (get_u16(&p[2]) << 16);
}

static uint32_t saturated_i16_bits(int32_t value)
{
    const int32_t clamped = (value > INT16_MAX) ? INT16_MAX : ((value < INT16_MIN) ? INT16_MIN : value);

    return (uint32_t)(uint16_t)(int16_t)clamped;
}

static void put_vec3(uint8_t *p, const int32_t *values, int32_t divisor)
{
    for (uint32_t axis = 0U; axis < AXES; ++axis)
    {
        put_u16(&p[2U * axis], saturated_i16_bits(values[axis] / divisor));
    }
}

static void get_vec3(const uint8_t *p, int32_t *values, int32_t multiplier)
{
    for (uint32_t axis = 0U; axis < AXES; ++axis)
    {
        values[axis] = (int32_t)(int16_t)(uint16_t)get_u16(&p[2U * axis]) * multiplier;
    }
}

size_t tlm_encode(const tlm_packet_t *packet, uint8_t *out, size_t size)
{
    if (size < TLM_PACKET_LEN)
    {
        return 0U;
    }

    (void)memset(out, 0, TLM_PACKET_LEN);
    (void)memcpy(out, MAGIC, sizeof(MAGIC));
    out[4] = TLM_VERSION;
    out[5] = packet->flags;
    put_u16(&out[6], TLM_PACKET_LEN);
    put_u32(&out[8], packet->seq);
    put_u32(&out[12], packet->node_b_uptime_ms);
    put_u32(&out[16], packet->node_a_uptime_ms);
    out[20] = packet->node_a_resets;
    out[21] = packet->node_b_resets;
    put_u16(&out[22], packet->node_a_age_ms);
    put_vec3(&out[24], packet->accel_mg, 1);
    put_vec3(&out[30], packet->gyro_mdps, GYRO_DIVISOR);
    put_vec3(&out[36], packet->mag_mgauss, 1);
    /* Bytes 42..43 are reserved and stay zero. */
    put_u32(&out[44], packet->can_valid);
    put_u32(&out[48], packet->can_rejected);
    put_u32(&out[52], packet->can_lost);
    put_u32(&out[CRC_OFFSET], crc32(out, CRC_OFFSET));
    return TLM_PACKET_LEN;
}

bool tlm_decode(const uint8_t *in, size_t len, tlm_packet_t *packet)
{
    if ((len != TLM_PACKET_LEN) || (memcmp(in, MAGIC, sizeof(MAGIC)) != 0) ||
        (in[4] != TLM_VERSION) || (get_u16(&in[6]) != TLM_PACKET_LEN) ||
        (get_u32(&in[CRC_OFFSET]) != crc32(in, CRC_OFFSET)))
    {
        return false;
    }

    packet->flags = in[5];
    packet->seq = get_u32(&in[8]);
    packet->node_b_uptime_ms = get_u32(&in[12]);
    packet->node_a_uptime_ms = get_u32(&in[16]);
    packet->node_a_resets = in[20];
    packet->node_b_resets = in[21];
    packet->node_a_age_ms = (uint16_t)get_u16(&in[22]);
    get_vec3(&in[24], packet->accel_mg, 1);
    get_vec3(&in[30], packet->gyro_mdps, GYRO_DIVISOR);
    get_vec3(&in[36], packet->mag_mgauss, 1);
    packet->can_valid = get_u32(&in[44]);
    packet->can_rejected = get_u32(&in[48]);
    packet->can_lost = get_u32(&in[52]);
    return true;
}
