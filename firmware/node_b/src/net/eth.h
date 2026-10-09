/**
 * @file eth.h
 * @brief Polled STM32F4 Ethernet MAC/DMA driver with an 802.3 PHY on RMII.
 *
 * The DMA uses chained descriptors: four for receive and four for transmit,
 * each with its own frame buffer. Frames are handed over without copying in
 * the driver: the caller fills a transmit buffer in place, and reads a
 * received frame in place before releasing it.
 *
 * PHY auto-negotiation takes about two seconds on real hardware, longer than
 * the watchdog timeout, so the link is brought up by eth_poll_link() from the
 * main loop instead of waiting in eth_init().
 */
#ifndef ETH_H
#define ETH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ETH_HWADDR_LEN  (6U)
#define ETH_FRAME_MAX   (1524U)   /**< Largest frame without CRC, including a VLAN tag. */

typedef enum
{
    ETH_LINK_DOWN = 0,
    ETH_LINK_10_HALF,
    ETH_LINK_10_FULL,
    ETH_LINK_100_HALF,
    ETH_LINK_100_FULL
} eth_link_t;

typedef struct
{
    uint32_t rx_frames;
    uint32_t rx_errors;     /**< Frames the MAC flagged as bad, or split across buffers. */
    uint32_t tx_frames;
    uint32_t tx_busy;       /**< No free transmit descriptor when a frame was offered. */
} eth_stats_t;

/**
 * Resets the MAC and DMA, checks for a PHY and starts auto-negotiation.
 * @return false if the DMA did not come out of reset or no PHY answered.
 */
bool eth_init(const uint8_t *hwaddr);

/**
 * Checks the PHY and applies the negotiated speed and duplex when the link
 * comes up. Call periodically (about every 100 ms). Returns the link state.
 */
eth_link_t eth_poll_link(void);

/** Next free transmit buffer of ETH_FRAME_MAX bytes, or NULL if all are in use. */
uint8_t *eth_tx_acquire(void);

/** Hands the buffer returned by eth_tx_acquire() to the DMA with @p len bytes. */
void eth_tx_commit(size_t len);

/**
 * Next good received frame, read in place. Returns its length without the CRC,
 * or 0 if none is pending. Call eth_rx_release() when done with it.
 */
size_t eth_rx_peek(const uint8_t **frame);

/** Returns the frame from eth_rx_peek() to the DMA. */
void eth_rx_release(void);

eth_stats_t eth_stats(void);

/** Short description of a link state, for logging. */
const char *eth_link_name(eth_link_t link);

#endif /* ETH_H */
