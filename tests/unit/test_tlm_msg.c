/**
 * @file test_tlm_msg.c
 * @brief Host unit tests for tlm_msg.c against the reference packet in docs/telemetry.md.
 */
#include <stdio.h>
#include <string.h>

#include "tlm_msg.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

/* Built from the specification with Python's struct module; also used by tests/python. */
static const char REFERENCE_HEX[] =
    "5644544d033f58000700000040e20100c0d40100010003000cfefa00e7031a04"
    "fcd600001f018dff35fe0901e803000001000000020000009246c8172ef88c13"
    "22247c1708023c009600b7004c002003a4010000305d4ac4";

static const tlm_packet_t REFERENCE = {
    .seq = 7U,
    .flags = TLM_FLAG_NODE_A_FRESH | TLM_FLAG_IMU_VALID | TLM_FLAG_RECORDER_OK |
             TLM_FLAG_GPS_FIX | TLM_FLAG_MAG_OK | (1U << TLM_HEADING_SOURCE_SHIFT),
    .node_b_uptime_ms = 123456U,
    .node_a_uptime_ms = 120000U,
    .node_a_resets = 1U,
    .node_b_resets = 0U,
    .node_a_age_ms = 3U,
    .accel_mg = { -500, 250, 999 },
    .gyro_mdps = { 10500, -105000, 0 },
    .mag_mgauss = { 287, -115, -459 },
    .gps_satellites = 9U,
    .gps_quality = 1U,
    .can_valid = 1000U,
    .can_rejected = 1U,
    .can_lost = 2U,
    .lat_e7 = 399001234,
    .lon_e7 = 328005678,
    .alt_msl_dm = 9250,
    .heading_cdeg = 6012U,
    .field_mgauss = 520U,
    .speed_dmps = 60U,
    .gps_age_ms = 150U,
    .distance_dm = 183U,
    .battery_pct = 76U,
    .flight_mode = 0U,
    .safety_flags = 0x20U,
    .last_request_id = 3U,
    .ground_link_age_ms = 420U,
};

static void reference_bytes(uint8_t *out)
{
    for (size_t i = 0U; i < TLM_PACKET_LEN; ++i)
    {
        unsigned int byte = 0U;
        (void)sscanf(&REFERENCE_HEX[2U * i], "%2x", &byte);
        out[i] = (uint8_t)byte;
    }
}

static void encodes_reference_packet(void)
{
    uint8_t expected[TLM_PACKET_LEN];
    uint8_t actual[TLM_PACKET_LEN];

    reference_bytes(expected);
    CHECK_EQ(tlm_encode(&REFERENCE, actual, sizeof(actual)), TLM_PACKET_LEN);
    CHECK(memcmp(actual, expected, TLM_PACKET_LEN) == 0);
}

static void decodes_reference_packet(void)
{
    uint8_t bytes[TLM_PACKET_LEN];
    tlm_packet_t p;

    reference_bytes(bytes);
    CHECK(tlm_decode(bytes, sizeof(bytes), &p));
    CHECK_EQ(p.seq, 7);
    CHECK_EQ(p.flags, 0x3F);
    CHECK_EQ(p.node_b_uptime_ms, 123456);
    CHECK_EQ(p.node_a_age_ms, 3);
    CHECK_EQ(p.accel_mg[0], -500);
    CHECK_EQ(p.gyro_mdps[1], -105000);
    CHECK_EQ(p.mag_mgauss[2], -459);
    CHECK_EQ(p.gps_satellites, 9);
    CHECK_EQ(p.can_lost, 2);
    CHECK_EQ(p.lat_e7, 399001234);
    CHECK_EQ(p.lon_e7, 328005678);
    CHECK_EQ(p.alt_msl_dm, 9250);
    CHECK_EQ(p.heading_cdeg, 6012);
    CHECK_EQ(p.field_mgauss, 520);
    CHECK_EQ(p.speed_dmps, 60);
    CHECK_EQ(p.gps_age_ms, 150);
    CHECK_EQ(p.distance_dm, 183);
    CHECK_EQ(p.battery_pct, 76);
    CHECK_EQ(p.flight_mode, 0);
    CHECK_EQ(p.safety_flags, 0x20);
    CHECK_EQ(p.last_request_id, 3);
    CHECK_EQ(p.ground_link_age_ms, 420);
}

static void rejects_any_single_bit_error(void)
{
    uint8_t bytes[TLM_PACKET_LEN];
    tlm_packet_t p;

    reference_bytes(bytes);
    for (size_t i = 0U; i < TLM_PACKET_LEN; ++i)
    {
        for (uint32_t bit = 0U; bit < 8U; ++bit)
        {
            bytes[i] ^= (uint8_t)(1U << bit);
            CHECK(!tlm_decode(bytes, sizeof(bytes), &p));
            bytes[i] ^= (uint8_t)(1U << bit);
        }
    }
}

static void rejects_wrong_length(void)
{
    uint8_t bytes[TLM_PACKET_LEN + 1U];
    tlm_packet_t p;

    reference_bytes(bytes);
    CHECK(!tlm_decode(bytes, TLM_PACKET_LEN - 1U, &p));
    CHECK(!tlm_decode(bytes, TLM_PACKET_LEN + 1U, &p));
    CHECK_EQ(tlm_encode(&REFERENCE, bytes, TLM_PACKET_LEN - 1U), 0);
}

static void saturates_out_of_range_values(void)
{
    tlm_packet_t in = REFERENCE;
    tlm_packet_t out;
    uint8_t bytes[TLM_PACKET_LEN];

    in.accel_mg[0] = 40000;
    in.gyro_mdps[0] = -400000;
    CHECK_EQ(tlm_encode(&in, bytes, sizeof(bytes)), TLM_PACKET_LEN);
    CHECK(tlm_decode(bytes, sizeof(bytes), &out));
    CHECK_EQ(out.accel_mg[0], 32767);
    CHECK_EQ(out.gyro_mdps[0], -327680);
}

int main(void)
{
    RUN(encodes_reference_packet);
    RUN(decodes_reference_packet);
    RUN(rejects_any_single_bit_error);
    RUN(rejects_wrong_length);
    RUN(saturates_out_of_range_values);
    return (unit_failures == 0) ? 0 : 1;
}
