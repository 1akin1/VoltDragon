/**
 * @file crc8.h
 * @brief CRC-8/SAE-J1850 (polynomial 0x1D, init 0xFF, final XOR 0xFF), as used
 *        by AUTOSAR E2E Profile 1 for CAN messages.
 */
#ifndef CRC8_H
#define CRC8_H

#include <stddef.h>
#include <stdint.h>

/** Initial value to pass to the first crc8_update() call. */
#define CRC8_INIT (0xFFU)

/** Adds @p len bytes to a running CRC (not yet finalised). */
uint8_t crc8_update(uint8_t crc, const uint8_t *data, size_t len);

/** Applies the final XOR. */
uint8_t crc8_final(uint8_t crc);

/** One-shot CRC of @p len bytes. crc8("123456789") is 0x4B. */
uint8_t crc8(const uint8_t *data, size_t len);

#endif /* CRC8_H */
