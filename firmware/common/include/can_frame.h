/**
 * @file can_frame.h
 * @brief Classic CAN data frame with an 11-bit identifier.
 */
#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>

#define CAN_STD_ID_MAX  (0x7FFU)
#define CAN_DATA_MAX    (8U)

typedef struct
{
    uint16_t id;                    /**< 11-bit standard identifier. */
    uint8_t  dlc;                   /**< Data length, 0..8. */
    uint8_t  data[CAN_DATA_MAX];
} can_frame_t;

#endif /* CAN_FRAME_H */
