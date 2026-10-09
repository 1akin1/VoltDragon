/**
 * @file test_can_msg.c
 * @brief Host unit tests for crc8.c, crc32.c and can_msg.c.
 */
#include <string.h>

#include "can_msg.h"
#include "crc32.h"
#include "crc8.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

static const uint8_t CHECK_INPUT[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };

static void crc8_matches_sae_j1850_check_value(void)
{
    CHECK_EQ(crc8(CHECK_INPUT, sizeof(CHECK_INPUT)), 0x4B);
    /* Incremental computation gives the same result. */
    uint8_t crc = crc8_update(CRC8_INIT, CHECK_INPUT, 4U);
    crc = crc8_update(crc, &CHECK_INPUT[4], 5U);
    CHECK_EQ(crc8_final(crc), 0x4B);
}

static void crc32_matches_ieee_check_value(void)
{
    CHECK_EQ(crc32(CHECK_INPUT, sizeof(CHECK_INPUT)), 0xCBF43926LL);
}

static void sealed_frame_is_valid(void)
{
    const uint8_t payload[CANMSG_PAYLOAD_LEN] = { 1, 2, 3, 4, 5, 6 };
    can_frame_t frame;

    canmsg_seal(&frame, CANMSG_ID_A_ACCEL, payload, 42U);
    CHECK_EQ(frame.id, CANMSG_ID_A_ACCEL);
    CHECK_EQ(frame.dlc, 8);
    CHECK(memcmp(frame.data, payload, CANMSG_PAYLOAD_LEN) == 0);
    CHECK_EQ(canmsg_seq(&frame), 42);
    CHECK(canmsg_valid(&frame));
}

static void every_single_bit_error_is_detected(void)
{
    const uint8_t payload[CANMSG_PAYLOAD_LEN] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };
    can_frame_t frame;

    canmsg_seal(&frame, CANMSG_ID_A_GYRO, payload, 7U);
    for (uint32_t byte = 0U; byte < CANMSG_DLC; ++byte)
    {
        for (uint32_t bit = 0U; bit < 8U; ++bit)
        {
            can_frame_t corrupted = frame;
            corrupted.data[byte] ^= (uint8_t)(1U << bit);
            CHECK(!canmsg_valid(&corrupted));
        }
    }
}

static void frame_under_wrong_identifier_is_rejected(void)
{
    const uint8_t payload[CANMSG_PAYLOAD_LEN] = { 0 };
    can_frame_t frame;

    canmsg_seal(&frame, CANMSG_ID_A_ACCEL, payload, 0U);
    frame.id = CANMSG_ID_A_MAG;
    CHECK(!canmsg_valid(&frame));
}

static void short_frame_is_rejected(void)
{
    const uint8_t payload[CANMSG_PAYLOAD_LEN] = { 0 };
    can_frame_t frame;

    canmsg_seal(&frame, CANMSG_ID_A_STATUS, payload, 0U);
    frame.dlc = 7U;
    CHECK(!canmsg_valid(&frame));
}

static void vec3_round_trips_and_saturates(void)
{
    const int32_t in[3] = { -500, 32767, -40000 };
    int32_t out[3];
    uint8_t payload[CANMSG_PAYLOAD_LEN];

    canmsg_encode_vec3(payload, in, 1);
    canmsg_decode_vec3(payload, out, 1);
    CHECK_EQ(out[0], -500);
    CHECK_EQ(out[1], 32767);
    CHECK_EQ(out[2], -32768);
    /* Little-endian: -500 = 0xFE0C. */
    CHECK_EQ(payload[0], 0x0C);
    CHECK_EQ(payload[1], 0xFE);
}

static void gyro_scaling_keeps_ten_mdps_resolution(void)
{
    const int32_t in[3] = { 10500, -105000, 245000 };
    int32_t out[3];
    uint8_t payload[CANMSG_PAYLOAD_LEN];

    canmsg_encode_vec3(payload, in, CANMSG_GYRO_DIVISOR);
    canmsg_decode_vec3(payload, out, CANMSG_GYRO_DIVISOR);
    CHECK_EQ(out[0], 10500);
    CHECK_EQ(out[1], -105000);
    CHECK_EQ(out[2], 245000);
}

static void status_round_trips(void)
{
    const canmsg_status_t in = { CANMSG_STATUS_IMU_VALID, 3U, 0x12345678UL };
    canmsg_status_t out;
    uint8_t payload[CANMSG_PAYLOAD_LEN];

    canmsg_encode_status(payload, &in);
    canmsg_decode_status(payload, &out);
    CHECK_EQ(out.flags, CANMSG_STATUS_IMU_VALID);
    CHECK_EQ(out.reset_count, 3);
    CHECK_EQ(out.uptime_ms, 0x12345678LL);
}

static void gps_and_nav_round_trip(void)
{
    const canmsg_gps_t gps_in = { -15000000, 328000000, -123, 1U, 9U };
    const canmsg_nav_t nav_in = { 35999U, 650U, CANMSG_NAV_GPS_FIX | (2U << CANMSG_NAV_SOURCE_SHIFT),
                                  60U };
    canmsg_gps_t gps_out = { 0, 0, 0, 0U, 0U };
    canmsg_nav_t nav_out = { 0U, 0U, 0U, 0U };
    uint8_t payload[CANMSG_PAYLOAD_LEN];

    canmsg_encode_gps_lat(payload, &gps_in);
    canmsg_decode_gps_lat(payload, &gps_out);
    canmsg_encode_gps_lon(payload, &gps_in);
    canmsg_decode_gps_lon(payload, &gps_out);
    CHECK_EQ(gps_out.lat_e7, -15000000);
    CHECK_EQ(gps_out.lon_e7, 328000000);
    CHECK_EQ(gps_out.alt_msl_dm, -123);
    CHECK_EQ(gps_out.quality, 1);
    CHECK_EQ(gps_out.satellites, 9);

    canmsg_encode_nav(payload, &nav_in);
    canmsg_decode_nav(payload, &nav_out);
    CHECK_EQ(nav_out.heading_cdeg, 35999);
    CHECK_EQ(nav_out.field_mgauss, 650);
    CHECK_EQ(nav_out.flags, nav_in.flags);
    CHECK_EQ(nav_out.speed_dmps, 60);
}

static void safety_and_node_b_messages_round_trip(void)
{
    const canmsg_safety_t safety_in = { 115U, 18U, 2U, CANMSG_SAFETY_PROXIMITY | CANMSG_SAFETY_LINK_LOST,
                                        7U };
    const canmsg_b_status_t status_in = { 3200U, CANMSG_B_GROUND_CONTACT };
    const canmsg_mode_req_t req_in = { 9U, 2U, 1U };
    canmsg_safety_t safety_out;
    canmsg_b_status_t status_out;
    canmsg_mode_req_t req_out;
    uint8_t payload[CANMSG_PAYLOAD_LEN];

    canmsg_encode_safety(payload, &safety_in);
    canmsg_decode_safety(payload, &safety_out);
    CHECK_EQ(safety_out.distance_dm, 115);
    CHECK_EQ(safety_out.battery_pct, 18);
    CHECK_EQ(safety_out.mode, 2);
    CHECK_EQ(safety_out.flags, safety_in.flags);
    CHECK_EQ(safety_out.last_request_id, 7);

    canmsg_encode_b_status(payload, &status_in);
    canmsg_decode_b_status(payload, &status_out);
    CHECK_EQ(status_out.link_age_ms, 3200);
    CHECK_EQ(status_out.flags, CANMSG_B_GROUND_CONTACT);

    canmsg_encode_mode_req(payload, &req_in);
    canmsg_decode_mode_req(payload, &req_out);
    CHECK_EQ(req_out.request_id, 9);
    CHECK_EQ(req_out.mode, 2);
    CHECK_EQ(req_out.override, 1);
}

static void sequence_tracker_counts_gaps(void)
{
    canmsg_seq_tracker_t t = { false, 0U };

    CHECK_EQ(canmsg_track_seq(&t, 10U), 0);     /* first frame synchronises */
    CHECK_EQ(canmsg_track_seq(&t, 11U), 0);
    CHECK_EQ(canmsg_track_seq(&t, 14U), 2);     /* 12 and 13 lost */
    CHECK_EQ(canmsg_track_seq(&t, 15U), 0);
}

static void sequence_tracker_handles_wrap_around(void)
{
    canmsg_seq_tracker_t t = { false, 0U };

    CHECK_EQ(canmsg_track_seq(&t, 254U), 0);
    CHECK_EQ(canmsg_track_seq(&t, 255U), 0);
    CHECK_EQ(canmsg_track_seq(&t, 0U), 0);
    CHECK_EQ(canmsg_track_seq(&t, 2U), 1);
}

static void sequence_tracker_resynchronises_on_restart_or_duplicate(void)
{
    canmsg_seq_tracker_t t = { false, 0U };

    CHECK_EQ(canmsg_track_seq(&t, 100U), 0);
    CHECK_EQ(canmsg_track_seq(&t, 100U), 0);    /* duplicate */
    CHECK_EQ(canmsg_track_seq(&t, 0U), 0);      /* sender restarted */
    CHECK_EQ(canmsg_track_seq(&t, 1U), 0);
}

static void sequence_tracker_splits_loss_and_restart_at_half_range(void)
{
    canmsg_seq_tracker_t t = { false, 0U };

    CHECK_EQ(canmsg_track_seq(&t, 0U), 0);      /* expects 1 next */
    CHECK_EQ(canmsg_track_seq(&t, 128U), 127);  /* largest gap counted as loss */
    CHECK_EQ(canmsg_track_seq(&t, 1U), 0);      /* 128 behind the expected 129: a restart */
}

int main(void)
{
    RUN(crc8_matches_sae_j1850_check_value);
    RUN(crc32_matches_ieee_check_value);
    RUN(sealed_frame_is_valid);
    RUN(every_single_bit_error_is_detected);
    RUN(frame_under_wrong_identifier_is_rejected);
    RUN(short_frame_is_rejected);
    RUN(vec3_round_trips_and_saturates);
    RUN(gyro_scaling_keeps_ten_mdps_resolution);
    RUN(status_round_trips);
    RUN(gps_and_nav_round_trip);
    RUN(safety_and_node_b_messages_round_trip);
    RUN(sequence_tracker_counts_gaps);
    RUN(sequence_tracker_handles_wrap_around);
    RUN(sequence_tracker_resynchronises_on_restart_or_duplicate);
    RUN(sequence_tracker_splits_loss_and_restart_at_half_range);
    return (unit_failures == 0) ? 0 : 1;
}
