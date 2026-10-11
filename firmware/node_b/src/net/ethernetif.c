/**
 * @file ethernetif.c
 * @brief lwIP network interface on top of the Ethernet driver.
 */
#include "ethernetif.h"

#include <string.h>

#include "eth.h"
#include "lwip/etharp.h"
#include "lwip/pbuf.h"

#define ETHERNET_MTU (1500U)

/* Locally administered unicast address 02:00:00:56:44:02. */
static const uint8_t s_hwaddr[ETH_HWADDR_LEN] = { 0x02U, 0x00U, 0x00U, 0x56U, 0x44U, 0x02U };

static uint32_t s_rx_dropped;

static err_t link_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;

    if (p->tot_len > ETH_FRAME_MAX)
    {
        return ERR_BUF;
    }
    uint8_t *buffer = eth_tx_acquire();
    if (buffer == NULL)
    {
        /* All transmit descriptors are still owned by the DMA. */
        return ERR_MEM;
    }
    (void)pbuf_copy_partial(p, buffer, p->tot_len, 0U);
    eth_tx_commit(p->tot_len);
    return ERR_OK;
}

err_t ethernetif_init(struct netif *netif)
{
    netif->name[0] = 'e';
    netif->name[1] = '0';
    netif->output = etharp_output;
    netif->linkoutput = link_output;
    netif->mtu = ETHERNET_MTU;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    (void)memcpy(netif->hwaddr, s_hwaddr, ETH_HWADDR_LEN);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;

    return eth_init(s_hwaddr) ? ERR_OK : ERR_IF;
}

void ethernetif_input(struct netif *netif)
{
    const uint8_t *frame = NULL;
    size_t length = eth_rx_peek(&frame);

    while (length != 0U)
    {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)length, PBUF_POOL);

        if (p == NULL)
        {
            s_rx_dropped++;
        }
        else
        {
            (void)pbuf_take(p, frame, (u16_t)length);
            if (netif->input(p, netif) != (err_t)ERR_OK)
            {
                (void)pbuf_free(p);
            }
        }
        eth_rx_release();
        length = eth_rx_peek(&frame);
    }
}

uint32_t ethernetif_rx_dropped(void)
{
    return s_rx_dropped;
}
