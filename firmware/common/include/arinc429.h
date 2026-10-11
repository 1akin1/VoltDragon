/**
 * @file arinc429.h
 * @brief ARINC 429 word encoding and decoding (HLR-015).
 *
 * A 32-bit word, numbered from bit 1 (sent first) to bit 32 (ARINC 429 Part 1):
 *
 *   bits 1-8    label, most significant bit first (so bit 1 is the label's MSB)
 *   bits 9-10   SDI, source/destination identifier
 *   bits 11-29  data; for BNR a two's complement fraction of the range, sign in bit 29
 *   bits 30-31  SSM, sign/status matrix
 *   bit 32      parity: odd over the whole word
 *
 * In the uint32_t used here, bit 1 is the least significant bit. Labels are octal by
 * convention; the constants are written in hex (MISRA C forbids octal constants)
 * with the octal label alongside. Word layout and labels: docs/arinc429.md.
 */
#ifndef ARINC429_H
#define ARINC429_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sign/status matrix for BNR data. */
#define ARINC429_SSM_FAILURE_WARNING    (0U)
#define ARINC429_SSM_NO_COMPUTED_DATA   (1U)
#define ARINC429_SSM_FUNCTIONAL_TEST    (2U)
#define ARINC429_SSM_NORMAL             (3U)

#define ARINC429_DATA_BITS      (19U)   /**< Bits 11-29, sign included. */
#define ARINC429_BNR_MAX_BITS   (18U)   /**< Significant bits a BNR value can use. */

/* The labels Node B sends. */
#define ARINC429_LABEL_ALTITUDE         (0x83U)     /**< Octal 203: altitude, BNR 17 bits, 1 ft. */
#define ARINC429_LABEL_TRUE_HEADING     (0xCCU)     /**< Octal 314: true heading, BNR 15 bits, 180 deg. */
#define ARINC429_LABEL_LINE_DISTANCE    (0xA9U)     /**< Octal 251: project-defined, BNR 16 bits, 0.125 m. */

#define ARINC429_ALTITUDE_BITS          (17U)
#define ARINC429_HEADING_BITS           (15U)
#define ARINC429_DISTANCE_BITS          (16U)

/** Assembles a word: label, SDI (0..3), the 19 data bits and SSM (0..3), and sets the parity bit. */
uint32_t arinc429_word(uint8_t label, uint32_t sdi, uint32_t data, uint32_t ssm);

/**
 * A BNR word: @p counts is the value in units of the least significant bit, with
 * @p sig_bits significant bits (1..18) below the sign; it saturates to the range.
 * SDI is 0.
 */
uint32_t arinc429_bnr(uint8_t label, uint32_t ssm, int32_t counts, uint32_t sig_bits);

uint8_t arinc429_label(uint32_t word);
uint32_t arinc429_sdi(uint32_t word);
uint32_t arinc429_ssm(uint32_t word);
bool arinc429_parity_ok(uint32_t word);

/** The value of a BNR word in units of its least significant bit. */
int32_t arinc429_bnr_counts(uint32_t word, uint32_t sig_bits);

/** True heading, label 314: @p heading_cdeg in 0.01 deg (any value; reduced to +-180 deg). */
uint32_t arinc429_true_heading(int32_t heading_cdeg, uint32_t ssm);

/** Altitude, label 203: @p alt_dm in 0.1 m, sent in feet. */
uint32_t arinc429_altitude(int32_t alt_dm, uint32_t ssm);

/** Distance to the nearest conductor, label 251: @p distance_dm in 0.1 m. */
uint32_t arinc429_line_distance(uint32_t distance_dm, uint32_t ssm);

#ifdef __cplusplus
}
#endif

#endif /* ARINC429_H */
