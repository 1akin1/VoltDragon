/**
 * @file can.h
 * @brief Polled, register-level bxCAN driver (standard identifiers, data frames).
 *
 * Transmission goes through a small software queue in front of the three
 * hardware mailboxes, drained in request order by can_poll(), so a burst of
 * frames never blocks. Reception uses FIFO 0 behind a hardware acceptance
 * filter. Automatic bus-off recovery is enabled.
 */
#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define CAN_TX_QUEUE_LEN (8U)

/** Counters for link monitoring. */
typedef struct
{
    uint32_t tx_queued;
    uint32_t tx_dropped;        /**< Software queue full. */
    uint32_t rx_frames;
    uint32_t rx_overruns;       /**< Frames lost because FIFO 0 was full. */
    uint8_t  tx_errors;         /**< Transmit error counter (TEC). */
    uint8_t  rx_errors;         /**< Receive error counter (REC). */
    bool     bus_off;
} can_stats_t;

/**
 * Configures the pins and bit timing, accepts nothing yet, and joins the bus.
 * @return false if the controller did not enter or leave initialisation mode.
 */
bool can_init(void);

/**
 * Accepts standard identifiers that match @p id in every bit set in @p mask,
 * into FIFO 0. Up to 14 filters (banks 0..13).
 */
bool can_add_filter(uint32_t bank, uint16_t id, uint16_t mask);

/** Queues a frame for transmission. Returns false (and counts a drop) if the queue is full. */
bool can_send(const can_frame_t *frame);

/** Moves queued frames into free mailboxes. Call from the main loop. */
void can_poll(void);

/** Takes one received frame from FIFO 0. Returns false if none is pending. */
bool can_receive(can_frame_t *frame);

/** Returns the counters, with the error state read from the controller. */
can_stats_t can_stats(void);

#endif /* CAN_H */
