/**
 * @file ethernetif.h
 * @brief lwIP network interface on top of the Ethernet driver (eth.c).
 */
#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include <stdint.h>

#include "lwip/err.h"
#include "lwip/netif.h"

/** netif_add() init callback. netif->state must point to the 6-byte hardware address. */
err_t ethernetif_init(struct netif *netif);

/** Passes every received frame to lwIP. Call from the main loop. */
void ethernetif_input(struct netif *netif);

/** Frames dropped because no receive pbuf was available. */
uint32_t ethernetif_rx_dropped(void);

#endif /* ETHERNETIF_H */
