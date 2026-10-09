/**
 * @file crc8.c
 * @brief CRC-8/SAE-J1850.
 */
#include "crc8.h"

#define CRC8_POLY   (0x1DU)
#define CRC8_XOROUT (0xFFU)

uint8_t crc8_update(uint8_t crc, const uint8_t *data, size_t len)
{
    uint8_t c = crc;

    for (size_t i = 0U; i < len; ++i)
    {
        c ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit)
        {
            c = ((c & 0x80U) != 0U) ? (uint8_t)((uint32_t)(c << 1) ^ CRC8_POLY) : (uint8_t)(c << 1);
        }
    }
    return c;
}

uint8_t crc8_final(uint8_t crc)
{
    return (uint8_t)(crc ^ CRC8_XOROUT);
}

uint8_t crc8(const uint8_t *data, size_t len)
{
    return crc8_final(crc8_update(CRC8_INIT, data, len));
}
