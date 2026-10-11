/**
 * @file arinc429.c
 * @brief ARINC 429 word encoding and decoding (HLR-015).
 */
#include "arinc429.h"

#define LABEL_BITS          (8U)
#define SDI_SHIFT           (8U)
#define DATA_SHIFT          (10U)
#define SSM_SHIFT           (29U)
/* U, not UL: unsigned long is 64 bits on the host where the unit tests run. */
#define TWO_BITS            (3U)
#define DATA_MASK           (0x7FFFFU)
#define PARITY_BIT          (0x80000000U)   /* bit 32 */

#define CDEG_HALF_TURN      (18000)
#define CDEG_FULL_TURN      (36000)
/* 15 significant bits over 180 deg: 2^15 counts per half turn. */
#define HEADING_COUNTS      (32768LL)
/* 1 ft = 0.3048 m: feet = decimetres x 1000 / 3048. */
#define DM_X1000_PER_FT     (3048LL)
/* 0.125 m per count: counts = decimetres x 4 / 5. */
#define DISTANCE_NUM        (4U)
#define DISTANCE_DEN        (5U)

static uint32_t reverse_label(uint8_t label)
{
    uint32_t in = label;
    uint32_t out = 0U;

    for (uint32_t i = 0U; i < LABEL_BITS; ++i)
    {
        out = (out << 1) | (in & 1U);
        in >>= 1;
    }
    return out;
}

static uint32_t count_ones(uint32_t value)
{
    uint32_t count = 0U;

    for (uint32_t v = value; v != 0U; v &= (v - 1U))
    {
        count++;
    }
    return count;
}

static uint32_t clamp_bits(uint32_t sig_bits)
{
    uint32_t bits = sig_bits;

    if (bits < 1U)
    {
        bits = 1U;
    }
    else if (bits > ARINC429_BNR_MAX_BITS)
    {
        bits = ARINC429_BNR_MAX_BITS;
    }
    else
    {
        /* In range. */
    }
    return bits;
}

/** Rounds num / den to the nearest integer, halves away from zero (den > 0). */
static int64_t divide_rounded(int64_t num, int64_t den)
{
    const int64_t half = den / 2;
    int64_t result;

    if (num < 0)
    {
        result = (num - half) / den;
    }
    else
    {
        result = (num + half) / den;
    }
    return result;
}

uint32_t arinc429_word(uint8_t label, uint32_t sdi, uint32_t data, uint32_t ssm)
{
    uint32_t word = reverse_label(label) | ((sdi & TWO_BITS) << SDI_SHIFT) |
                    ((data & DATA_MASK) << DATA_SHIFT) | ((ssm & TWO_BITS) << SSM_SHIFT);

    if ((count_ones(word) % 2U) == 0U)
    {
        word |= PARITY_BIT;
    }
    return word;
}

uint32_t arinc429_bnr(uint8_t label, uint32_t ssm, int32_t counts, uint32_t sig_bits)
{
    const uint32_t bits = clamp_bits(sig_bits);
    const uint32_t limit = 1U << bits;
    const int32_t max = (int32_t)limit - 1;
    const int32_t min = -(int32_t)limit;
    int32_t value = counts;

    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }
    else
    {
        /* In range. */
    }
    /* Two's complement, left-aligned so the sign lands in bit 29. */
    const uint32_t pattern = (uint32_t)value;
    const uint32_t data = (pattern << (ARINC429_BNR_MAX_BITS - bits)) & DATA_MASK;
    return arinc429_word(label, 0U, data, ssm);
}

uint8_t arinc429_label(uint32_t word)
{
    const uint32_t label = reverse_label((uint8_t)(word & 0xFFU));
    return (uint8_t)label;
}

uint32_t arinc429_sdi(uint32_t word)
{
    return (word >> SDI_SHIFT) & TWO_BITS;
}

uint32_t arinc429_ssm(uint32_t word)
{
    return (word >> SSM_SHIFT) & TWO_BITS;
}

bool arinc429_parity_ok(uint32_t word)
{
    return (count_ones(word) % 2U) == 1U;
}

int32_t arinc429_bnr_counts(uint32_t word, uint32_t sig_bits)
{
    const uint32_t bits = clamp_bits(sig_bits);
    const uint32_t field = ((word >> DATA_SHIFT) & DATA_MASK) >> (ARINC429_BNR_MAX_BITS - bits);
    const uint32_t sign = 1U << bits;
    const uint32_t magnitude = field & (sign - 1U);
    int32_t counts = (int32_t)magnitude;

    if ((field & sign) != 0U)
    {
        counts -= (int32_t)sign;
    }
    return counts;
}

uint32_t arinc429_true_heading(int32_t heading_cdeg, uint32_t ssm)
{
    int32_t counts = 0;

    if (ssm == ARINC429_SSM_NORMAL)
    {
        /* Reduce to [-180, 180) deg; the C remainder keeps the sign of the dividend. */
        int32_t cdeg = heading_cdeg % CDEG_FULL_TURN;
        if (cdeg >= CDEG_HALF_TURN)
        {
            cdeg -= CDEG_FULL_TURN;
        }
        else if (cdeg < -CDEG_HALF_TURN)
        {
            cdeg += CDEG_FULL_TURN;
        }
        else
        {
            /* In range. */
        }
        const int64_t scaled = (int64_t)cdeg * HEADING_COUNTS;
        const int64_t rounded = divide_rounded(scaled, CDEG_HALF_TURN);
        counts = (int32_t)rounded;
    }
    return arinc429_bnr(ARINC429_LABEL_TRUE_HEADING, ssm, counts, ARINC429_HEADING_BITS);
}

uint32_t arinc429_altitude(int32_t alt_dm, uint32_t ssm)
{
    int32_t counts = 0;

    if (ssm == ARINC429_SSM_NORMAL)
    {
        const int64_t scaled = (int64_t)alt_dm * 1000LL;
        const int64_t feet = divide_rounded(scaled, DM_X1000_PER_FT);
        counts = (int32_t)feet;
    }
    return arinc429_bnr(ARINC429_LABEL_ALTITUDE, ssm, counts, ARINC429_ALTITUDE_BITS);
}

uint32_t arinc429_line_distance(uint32_t distance_dm, uint32_t ssm)
{
    int32_t counts = 0;

    if (ssm == ARINC429_SSM_NORMAL)
    {
        /* At most 65535 dm: the product and the result fit easily. */
        const uint32_t scaled = (distance_dm & 0xFFFFU) * DISTANCE_NUM;
        const uint32_t eighths = (scaled + (DISTANCE_DEN / 2U)) / DISTANCE_DEN;
        counts = (int32_t)eighths;
    }
    return arinc429_bnr(ARINC429_LABEL_LINE_DISTANCE, ssm, counts, ARINC429_DISTANCE_BITS);
}
