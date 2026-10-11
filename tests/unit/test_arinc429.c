/**
 * @file test_arinc429.c
 * @brief Host unit tests for arinc429.c: word layout, parity, BNR scaling and Node B's labels.
 *
 * The expected words are worked out by hand from the layout in arinc429.h; the
 * Renode bus monitor (renode/Arinc429Transmitter.cs) decodes the same words
 * independently in tests/robot/system_arinc.robot.
 */
#include <stdint.h>

#include "arinc429.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

static void heading_of_90_degrees(void)
{
    /*
     * 9000 cdeg x 2^15 / 18000 = 16384 counts; 15 significant bits, so shifted left by
     * 18 - 15 = 3: data 0x20000, at bit 11: 0x08000000. SSM normal (3 << 29):
     * 0x60000000. Label 314 octal = 0xCC = 11001100b, reversed 00110011b = 0x33.
     * 0x68000033 has 3 + 4 = 7 ones: odd already, so the parity bit stays clear.
     */
    CHECK_EQ(arinc429_true_heading(9000, ARINC429_SSM_NORMAL), 0x68000033UL);
}

static void heading_of_270_degrees_is_minus_90(void)
{
    /*
     * -16384 counts: two's complement 0x...FFFFC000 << 3, 19 bits: 0x60000 (sign set);
     * at bit 11: 0x18000000. With SSM and label: 0x78000033, 4 + 4 = 8 ones, so the
     * parity bit is set: 0xF8000033.
     */
    CHECK_EQ(arinc429_true_heading(27000, ARINC429_SSM_NORMAL), 0xF8000033UL);
    CHECK_EQ(arinc429_true_heading(-9000, ARINC429_SSM_NORMAL), 0xF8000033UL);
}

static void heading_wraps_to_plus_minus_180(void)
{
    const uint32_t south = arinc429_true_heading(18000, ARINC429_SSM_NORMAL);

    /* 180 deg is sent as -180 deg, the most negative value. */
    CHECK_EQ(arinc429_bnr_counts(south, ARINC429_HEADING_BITS), -32768);
    CHECK_EQ(arinc429_true_heading(36000 + 9000, ARINC429_SSM_NORMAL),
             arinc429_true_heading(9000, ARINC429_SSM_NORMAL));
    CHECK_EQ(arinc429_true_heading(-27000, ARINC429_SSM_NORMAL),
             arinc429_true_heading(9000, ARINC429_SSM_NORMAL));
    /* 359.99 deg = -0.01 deg = -1 cdeg: -1.82 counts, rounded to -2. */
    CHECK_EQ(arinc429_bnr_counts(arinc429_true_heading(35999, ARINC429_SSM_NORMAL),
                                 ARINC429_HEADING_BITS), -2);
}

static void altitude_in_feet(void)
{
    /*
     * 925.0 m = 9250 dm = 3034.78 ft, rounded to 3035 counts; 17 bits, shifted by 1:
     * 6070 = 0x17B6, at bit 11: 0x5ED800. SSM: 0x60000000. Label 203 octal = 0x83 =
     * 10000011b, reversed 11000001b = 0xC1. 0x605ED8C1 has 2 + 5 + 4 + 3 = 14 ones,
     * so the parity bit is set: 0xE05ED8C1.
     */
    CHECK_EQ(arinc429_altitude(9250, ARINC429_SSM_NORMAL), 0xE05ED8C1UL);
    CHECK_EQ(arinc429_bnr_counts(arinc429_altitude(-305, ARINC429_SSM_NORMAL),
                                 ARINC429_ALTITUDE_BITS), -100);
}

static void line_distance_in_eighths_of_a_metre(void)
{
    /*
     * 11.9 m = 119 dm = 95.2 eighths of a metre, rounded to 95; 16 bits, shifted by 2:
     * 380 = 0x17C, at bit 11: 0x5F000. SSM: 0x60000000. Label 251 octal = 0xA9 =
     * 10101001b, reversed 10010101b = 0x95. 0x6005F095 has 2 + 2 + 4 + 4 = 12 ones,
     * so the parity bit is set: 0xE005F095.
     */
    CHECK_EQ(arinc429_line_distance(119U, ARINC429_SSM_NORMAL), 0xE005F095UL);
    /* The largest distance Node A reports, 6553.4 m, still fits the 8192 m range. */
    CHECK_EQ(arinc429_bnr_counts(arinc429_line_distance(65534U, ARINC429_SSM_NORMAL),
                                 ARINC429_DISTANCE_BITS), 52427);
}

static void no_data_words_carry_zero_and_their_status(void)
{
    /* SSM 1 << 29 and the label: 1 + 4 = 5 ones, parity clear. */
    CHECK_EQ(arinc429_true_heading(9000, ARINC429_SSM_NO_COMPUTED_DATA), 0x20000033UL);
    const uint32_t failed = arinc429_altitude(9250, ARINC429_SSM_FAILURE_WARNING);
    CHECK_EQ(arinc429_ssm(failed), ARINC429_SSM_FAILURE_WARNING);
    CHECK_EQ(arinc429_bnr_counts(failed, ARINC429_ALTITUDE_BITS), 0);
    CHECK_EQ(arinc429_bnr_counts(arinc429_line_distance(119U, ARINC429_SSM_NO_COMPUTED_DATA),
                                 ARINC429_DISTANCE_BITS), 0);
}

static void fields_decode_back(void)
{
    const uint32_t word = arinc429_word(ARINC429_LABEL_ALTITUDE, 2U, 0x12345U,
                                        ARINC429_SSM_FUNCTIONAL_TEST);

    CHECK_EQ(arinc429_label(word), ARINC429_LABEL_ALTITUDE);
    CHECK_EQ(arinc429_sdi(word), 2);
    CHECK_EQ(arinc429_ssm(word), ARINC429_SSM_FUNCTIONAL_TEST);
    CHECK_EQ((word >> 10) & 0x7FFFFUL, 0x12345);
    CHECK(arinc429_parity_ok(word));
    /* Out-of-range fields are masked rather than spilling into their neighbours. */
    CHECK_EQ(arinc429_word(0U, 7U, 0xFFFFFFFFUL, 7U) & 0x7FFFFFFFUL, 0x7FFFFF00UL);
}

static void every_word_has_odd_parity(void)
{
    for (uint32_t data = 0U; data < 0x80000UL; data += 0x1111U)
    {
        for (uint32_t ssm = 0U; ssm < 4U; ++ssm)
        {
            const uint32_t word = arinc429_word((uint8_t)(data & 0xFFU), data & 3U, data, ssm);
            CHECK(arinc429_parity_ok(word));
            /* Flipping any one bit is detected. */
            CHECK(!arinc429_parity_ok(word ^ (1U << (data % 32U))));
        }
    }
}

static void bnr_round_trips_and_saturates(void)
{
    for (int32_t counts = -32768; counts <= 32767; counts += 7)
    {
        CHECK_EQ(arinc429_bnr_counts(arinc429_bnr(0x01U, ARINC429_SSM_NORMAL, counts, 15U), 15U),
                 counts);
    }
    CHECK_EQ(arinc429_bnr_counts(arinc429_bnr(0x01U, ARINC429_SSM_NORMAL, 100000, 15U), 15U),
             32767);
    CHECK_EQ(arinc429_bnr_counts(arinc429_bnr(0x01U, ARINC429_SSM_NORMAL, -100000, 15U), 15U),
             -32768);
    /* 18 significant bits use the whole field; out-of-range widths are clamped. */
    CHECK_EQ(arinc429_bnr_counts(arinc429_bnr(0x01U, ARINC429_SSM_NORMAL, -131072, 18U), 18U),
             -131072);
    CHECK_EQ(arinc429_bnr_counts(arinc429_bnr(0x01U, ARINC429_SSM_NORMAL, 5, 30U), 30U), 5);
    CHECK_EQ(arinc429_bnr_counts(arinc429_bnr(0x01U, ARINC429_SSM_NORMAL, -1, 0U), 0U), -1);
}

int main(void)
{
    RUN(heading_of_90_degrees);
    RUN(heading_of_270_degrees_is_minus_90);
    RUN(heading_wraps_to_plus_minus_180);
    RUN(altitude_in_feet);
    RUN(line_distance_in_eighths_of_a_metre);
    RUN(no_data_words_carry_zero_and_their_status);
    RUN(fields_decode_back);
    RUN(every_word_has_odd_parity);
    RUN(bnr_round_trips_and_saturates);
    return (unit_failures == 0) ? 0 : 1;
}
