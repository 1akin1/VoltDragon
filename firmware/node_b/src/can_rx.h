/**
 * @file can_rx.h
 * @brief Node B's side of the CAN link (HLR-003, HLR-011, HLR-013, HLR-014).
 *
 * Reception: the hardware filter admits identifiers 0x100..0x10F only. Each
 * frame is checked (DLC and CRC), its sequence counter is tracked per
 * identifier to count lost frames, and the latest values are kept with their
 * arrival time. A Node A restart (STATUS uptime going backwards or a new reset
 * count) restarts its sequence counters, so the trackers are resynchronised
 * instead of counting the jump as lost frames.
 *
 * Transmission: Node B reports the ground link (time since the last ground
 * station command) every 100 ms, and forwards operator mode requests.
 */
#ifndef CAN_RX_H
#define CAN_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "can_msg.h"
#include "flight_mode.h"

#define CAN_RX_B_STATUS_PERIOD_MS (100UL)
#define CAN_RX_REQUEST_TIMEOUT_MS (500UL)   /**< Wait this long for Node A to confirm a request. */

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
    canmsg_gps_t    gps;
    canmsg_nav_t    nav;
    canmsg_safety_t safety;
    canmsg_health_t health;
    uint32_t        last_valid_ms;  /**< Arrival time of the last valid frame. */
    uint32_t        gps_ms;         /**< Arrival time of the last GPS position frame. */
    bool            any_valid;
    bool            any_gps;
    bool            any_safety;
    bool            any_health;
} can_rx_node_a_t;

/** Joins the CAN bus and installs the acceptance filter. Returns false on failure. */
bool can_rx_init(void);

/** Processes every received frame and sends the periodic ground-link report. Main loop. */
void can_rx_poll(uint32_t now_ms);

can_rx_stats_t can_rx_stats(void);

/** Latest Node A data. Returns false if no valid frame has been received yet. */
bool can_rx_node_a(can_rx_node_a_t *out);

/** Records contact with the ground station (a valid command arrived). */
void can_rx_ground_contact(uint32_t now_ms);

/** Milliseconds since the last ground station contact; UINT32_MAX if none yet. */
uint32_t can_rx_ground_link_age_ms(uint32_t now_ms);

/** Forwards an operator mode request to Node A. Returns the request id used (1..255). */
uint8_t can_rx_request_mode(flight_mode_t mode, bool override, uint32_t now_ms);

/**
 * Node A's flight mode as far as Node B knows it: the mode of a request just
 * forwarded, until Node A's SAFETY report shows it handled that request (or
 * CAN_RX_REQUEST_TIMEOUT_MS passes), and Node A's reported mode otherwise.
 * Requests are checked against this, so a command sent straight after another
 * is not judged on a report older than the first. False if Node A has sent no
 * SAFETY report yet.
 */
bool can_rx_node_a_mode(uint32_t now_ms, flight_mode_t *mode);

/** Logs the frames received in the last second, the error counts and Node A's latest data. */
void can_rx_report(uint32_t now_ms);

#endif /* CAN_RX_H */
