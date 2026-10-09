/**
 * @file crc32.c
 * @brief CRC-32 (IEEE 802.3, reflected).
 */
#include "crc32.h"

/* Bitwise implementation: small, and fast enough for records of a few dozen bytes. */
uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}
