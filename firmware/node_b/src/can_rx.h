/**
 * @file can_rx.h
 * @brief Node B reception of Node A's CAN messages (HLR-011, HLR-013).
 *
 * The hardware filter admits identifiers 0x100..0x10F only. Each frame is
 * checked (DLC and CRC), its sequence counter is tracked per identifier to
 * count lost frames, and the latest values are kept with their arrival time.
 *
 * A Node A restart (STATUS uptime going backwards or a new reset count)
 * restarts its sequence counters, so the trackers are resynchronised instead
 * of counting the jump as lost frames.
 */
#ifndef CAN_RX_H
#define CAN_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "can_msg.h"

typedef struct
{
    uint32_t valid;             /**< Frames accepted. */
    uint32_t rejected;          /**< Wrong DLC or CRC. */
    uint32_t lost;              /**< Frames missing from the sequence. */
    uint32_t unknown_id;        /**< Passed the filter but not a known message. */
    uint32_t overruns;          /**< Lost because the receive FIFO was full. */
    uint32_t restarts;          /**< Node A restarts detected. */
} can_rx_stats_t;

/** Latest data received from Node A. */
typedef struct
{
    canmsg_status_t status;
    int32_t         accel_mg[3];
    int32_t         gyro_mdps[3];
    int32_t         mag_mgauss[3];
    uint32_t        last_valid_ms;  /**< Arrival time of the last valid frame. */
    bool            any_valid;
} can_rx_node_a_t;

/** Joins the CAN bus and installs the acceptance filter. Returns false on failure. */
bool can_rx_init(void);

/** Processes every received frame. Call from the main loop. */
void can_rx_poll(uint32_t now_ms);

can_rx_stats_t can_rx_stats(void);

/** Latest Node A data. Returns false if no valid frame has been received yet. */
bool can_rx_node_a(can_rx_node_a_t *out);

/** Logs the frames received in the last second, the error counts and Node A's latest data. */
void can_rx_report(uint32_t now_ms);

#endif /* CAN_RX_H */
