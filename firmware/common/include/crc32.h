/**
 * @file crc32.h
 * @brief CRC-32 (IEEE 802.3, reflected, polynomial 0xEDB88320), as used by zlib and Ethernet.
 */
#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

/** Returns the CRC-32 of @p len bytes. crc32("123456789") is 0xCBF43926. */
uint32_t crc32(const uint8_t *data, size_t len);

#endif /* CRC32_H */
