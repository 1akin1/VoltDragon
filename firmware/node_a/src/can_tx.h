/**
 * @file can_tx.h
 * @brief Node A -> Node B CAN messages at 50 Hz (HLR-011, HLR-013).
 *
 * Node A uses a fixed schedule of ten 2 ms slots: STATUS, ACCEL, GYRO, MAG,
 * GPS_LAT, GPS_LON, NAV, SAFETY, HEALTH and one idle slot. Each message therefore repeats
 * every 20 ms (50 Hz) and only one frame is ever in flight; bursts of
 * back-to-back frames would overflow the receiver's three-deep FIFO. The IMU
 * and GPS messages are skipped while their data is not valid, so Node B sees
 * the data age grow. Message layout: docs/can-messages.md.
 *
 * The same task receives Node B's frames (ground-link status, operator mode
 * requests) and hands them to the safety logic (safety.h).
 */
#ifndef CAN_TX_H
#define CAN_TX_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_TX_PERIOD_MS (20UL)
#define CAN_TX_SLOT_MS   (2UL)
#define CAN_TX_SLOTS     (10U)

/** Faults that can be injected into the next transmitted frame, for testing. */
typedef enum
{
    CAN_TX_FAULT_NONE = 0,
    CAN_TX_FAULT_BAD_CRC,       /**< Next frame carries a wrong CRC. */
    CAN_TX_FAULT_SKIP_SEQ,      /**< Next frame skips one sequence number, as if a frame was lost. */
    CAN_TX_FAULT_FOREIGN_ID     /**< Send one extra frame with an identifier Node B does not accept. */
} can_tx_fault_t;

/** Joins the CAN bus. Returns false if the controller could not be started. */
bool can_tx_init(void);

/** Sends the periodic messages when due and feeds the transmit mailboxes. Call from the main loop. */
void can_tx_poll(uint32_t now_ms);

/** Arms a fault for the next cycle. */
void can_tx_inject(can_tx_fault_t fault);

/** Logs frames sent in the last second, drops and the controller error state. */
void can_tx_report(void);

#endif /* CAN_TX_H */
