/**
 * @file net.c
 * @brief Node B's IPv4 network: static addressing, telemetry and command sockets.
 */
#include "net.h"

#include "eth.h"
#include "ethernetif.h"
#include "log.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "netif/ethernet.h"

#define LINK_POLL_MS        (100UL)
#define COMMAND_REPLY_MAX   (128U)

static const uint8_t s_hwaddr[ETH_HWADDR_LEN] = { 0x02U, 0x00U, 0x00U, 0x56U, 0x44U, 0x02U };

typedef struct
{
    bool                  ready;
    struct netif          netif;
    struct udp_pcb       *telemetry_pcb;
    struct udp_pcb       *command_pcb;
    net_command_handler_t command_handler;
    eth_link_t            link;
    uint32_t              next_link_poll_ms;
} net_state_t;

static net_state_t s_net;

static void on_command(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                       u16_t port)
{
    uint8_t request[COMMAND_REPLY_MAX];
    char reply[COMMAND_REPLY_MAX];

    (void)arg;
    /* An oversized datagram is passed on truncated but with its real length, so it is rejected. */
    const u16_t copied = pbuf_copy_partial(p, request, sizeof(request), 0U);
    const size_t length = (p->tot_len > copied) ? p->tot_len : copied;
    const size_t reply_len = s_net.command_handler(request, length, reply, sizeof(reply));
    (void)pbuf_free(p);

    if (reply_len == 0U)
    {
        return;
    }
    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, (u16_t)reply_len, PBUF_RAM);
    if (out != 0)
    {
        (void)pbuf_take(out, reply, (u16_t)reply_len);
        (void)udp_sendto(pcb, out, addr, port);
        (void)pbuf_free(out);
    }
}

bool net_init(net_command_handler_t command_handler)
{
    ip4_addr_t address;
    ip4_addr_t netmask;
    ip4_addr_t gateway;

    s_net = (net_state_t){ 0 };
    s_net.command_handler = command_handler;
    lwip_init();

    IP4_ADDR(&address, 192, 168, 10, 2);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 192, 168, 10, 1);
    if (netif_add(&s_net.netif, &address, &netmask, &gateway, (void *)s_hwaddr, ethernetif_init,
                  ethernet_input) == 0)
    {
        LOG_ERROR("net: Ethernet did not start (no PHY?), running without network");
        return false;
    }
    netif_set_default(&s_net.netif);
    netif_set_up(&s_net.netif);

    s_net.telemetry_pcb = udp_new();
    s_net.command_pcb = udp_new();
    if ((s_net.telemetry_pcb == 0) || (s_net.command_pcb == 0) ||
        (udp_bind(s_net.command_pcb, IP_ADDR_ANY, NET_COMMAND_PORT) != ERR_OK))
    {
        LOG_ERROR("net: could not open UDP sockets");
        return false;
    }
    udp_recv(s_net.command_pcb, on_command, 0);

    s_net.ready = true;
    LOG_INFO("net: 192.168.10.2/24, telemetry to broadcast port %u, commands on port %u",
             NET_TELEMETRY_PORT, NET_COMMAND_PORT);
    return true;
}

static void poll_link(void)
{
    const eth_link_t link = eth_poll_link();

    if (link == s_net.link)
    {
        return;
    }
    s_net.link = link;
    if (link == ETH_LINK_DOWN)
    {
        netif_set_link_down(&s_net.netif);
        LOG_WARN("net: link down");
    }
    else
    {
        netif_set_link_up(&s_net.netif);
        LOG_INFO("net: link up, %s", eth_link_name(link));
    }
}

void net_poll(uint32_t now_ms)
{
    if (!s_net.ready)
    {
        return;
    }

    ethernetif_input(&s_net.netif);
    if ((int32_t)(now_ms - s_net.next_link_poll_ms) >= 0)
    {
        s_net.next_link_poll_ms = now_ms + LINK_POLL_MS;
        poll_link();
    }
    sys_check_timeouts();
}

bool net_link_up(void)
{
    return s_net.ready && (s_net.link != ETH_LINK_DOWN);
}

bool net_send_telemetry(const uint8_t *data, size_t len)
{
    if (!net_link_up())
    {
        return false;
    }

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (p == 0)
    {
        return false;
    }
    (void)pbuf_take(p, data, (u16_t)len);

    ip4_addr_t broadcast;
    ip4_addr_set_u32(&broadcast, ip4_addr_get_u32(netif_ip4_addr(&s_net.netif)) |
                                     ~ip4_addr_get_u32(netif_ip4_netmask(&s_net.netif)));
    const err_t err = udp_sendto(s_net.telemetry_pcb, p, &broadcast, NET_TELEMETRY_PORT);
    (void)pbuf_free(p);
    return err == ERR_OK;
}

void net_report(void)
{
    if (!s_net.ready)
    {
        return;
    }

    const eth_stats_t stats = eth_stats();
    LOG_INFO("net: link %s, rx %lu (errors %lu, dropped %lu), tx %lu (busy %lu)",
             eth_link_name(s_net.link), stats.rx_frames, stats.rx_errors, ethernetif_rx_dropped(),
             stats.tx_frames, stats.tx_busy);
}
