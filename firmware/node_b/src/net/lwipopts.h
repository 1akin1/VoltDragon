/**
 * @file lwipopts.h
 * @brief lwIP configuration for Node B: bare-metal (NO_SYS), IPv4, UDP, ARP and ICMP only.
 *
 * Everything runs from the main loop through lwIP's raw API, so no locking or
 * OS layer is needed. TCP, DHCP and DNS are not used: the node has a static
 * address and only exchanges UDP datagrams (telemetry and commands).
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* Platform */
#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0
#define LWIP_NETCONN                0
#define LWIP_SOCKET                 0
#define LWIP_SINGLE_NETIF           1
#define LWIP_PROVIDE_ERRNO          1

/* Memory: a small heap for outgoing datagrams and a pool of full-size receive buffers. */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (8 * 1024)
#define PBUF_POOL_SIZE              8
#define PBUF_POOL_BUFSIZE           1536
#define MEMP_NUM_UDP_PCB            4
#define MEMP_NUM_SYS_TIMEOUT        8
#define ARP_TABLE_SIZE              4
#define ARP_QUEUEING                1

/* Protocols */
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_RAW                    0
#define LWIP_DHCP                   0
#define LWIP_AUTOIP                 0
#define LWIP_ACD                    0
#define LWIP_IGMP                   0
#define LWIP_DNS                    0
#define IP_REASSEMBLY               0
#define IP_FRAG                     0

/* Checksums are computed in software. */
#define CHECKSUM_GEN_IP             1
#define CHECKSUM_GEN_UDP            1
#define CHECKSUM_GEN_ICMP           1
#define CHECKSUM_CHECK_IP           1
#define CHECKSUM_CHECK_UDP          1
#define CHECKSUM_CHECK_ICMP         1
#define LWIP_CHKSUM_ALGORITHM       3

/* Network interface */
#define LWIP_NETIF_STATUS_CALLBACK  0
#define LWIP_NETIF_LINK_CALLBACK    0

/* Diagnostics */
#define LWIP_STATS                  0
#define LWIP_DEBUG                  0

#endif /* LWIPOPTS_H */
