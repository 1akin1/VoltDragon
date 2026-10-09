/**
 * @file eth.c
 * @brief Polled STM32F4 Ethernet MAC/DMA driver with an 802.3 PHY on RMII.
 */
#include "eth.h"

#include "board.h"
#include "stm32f4_regs.h"

#define RX_DESCRIPTORS      (4U)
#define TX_DESCRIPTORS      (4U)
#define BUFFER_SIZE         (1528U)     /* ETH_FRAME_MAX rounded up to a multiple of 4 */
#define POLL_LIMIT          (100000UL)
#define CRC_LEN             (4U)

/* Descriptor bits (RM0090 section 33.6). */
#define DES0_OWN            (1UL << 31)
#define TDES0_LS            (1UL << 29)
#define TDES0_FS            (1UL << 28)
#define TDES0_TCH           (1UL << 20)
#define RDES0_FL_SHIFT      (16U)
#define RDES0_FL_MASK       (0x3FFFUL)
#define RDES0_ES            (1UL << 15)
#define RDES0_FS            (1UL << 9)
#define RDES0_LS            (1UL << 8)
#define RDES1_RCH           (1UL << 14)
#define DES1_SIZE_MASK      (0x1FFFUL)

/* 802.3 clause 22 PHY registers. */
#define PHY_ADDRESS         (0U)
#define PHY_BCR             (0U)
#define PHY_BSR             (1U)
#define PHY_ID1             (2U)
#define PHY_ANAR            (4U)
#define PHY_ANLPAR          (5U)
#define PHY_BCR_RESET       (1UL << 15)
#define PHY_BCR_ANEG_ENABLE (1UL << 12)
#define PHY_BCR_ANEG_RESTART (1UL << 9)
#define PHY_BSR_LINK        (1UL << 2)
#define PHY_BSR_ANEG_DONE   (1UL << 5)
#define PHY_ABILITY_100FD   (1UL << 8)
#define PHY_ABILITY_100HD   (1UL << 7)
#define PHY_ABILITY_10FD    (1UL << 6)
/* Advertise 10/100 half and full duplex, IEEE 802.3 selector. */
#define PHY_ANAR_ALL        (0x01E1UL)
/* One MDIO read takes about 64 us at a 1 MHz MDC, so this allows about 64 ms. */
#define PHY_RESET_POLLS     (1000U)

/* MDC = HCLK / 16 (CR = 2), within 2.5 MHz for any HCLK up to 35 MHz. */
#define MDIO_CLOCK_RANGE    (2UL)
/* Programmable burst length of 32 beats. */
#define DMA_BURST_LENGTH    (32UL)

typedef struct
{
    volatile uint32_t des0;
    volatile uint32_t des1;
    volatile uint32_t des2;     /* buffer address */
    volatile uint32_t des3;     /* next descriptor (chained mode) */
} dma_descriptor_t;

typedef struct
{
    dma_descriptor_t rx[RX_DESCRIPTORS];
    dma_descriptor_t tx[TX_DESCRIPTORS];
    uint32_t         rx_next;
    uint32_t         tx_next;
    eth_link_t       link;
    eth_stats_t      stats;
} eth_state_t;

static eth_state_t s_eth;
static uint32_t s_rx_buffers[RX_DESCRIPTORS][BUFFER_SIZE / 4U];
static uint32_t s_tx_buffers[TX_DESCRIPTORS][BUFFER_SIZE / 4U];

static void memory_barrier(void)
{
    /* Descriptor fields must be visible to the DMA before ownership changes hands. */
    __asm volatile("dmb" ::: "memory");
}

static bool mdio_wait(void)
{
    for (uint32_t i = 0U; i < POLL_LIMIT; ++i)
    {
        if ((ETH_MAC->MACMIIAR & ETH_MACMIIAR_MB) == 0U)
        {
            return true;
        }
    }
    return false;
}

static uint32_t mdio_command(uint32_t reg, bool write)
{
    return (PHY_ADDRESS << ETH_MACMIIAR_PA_SHIFT) | (reg << ETH_MACMIIAR_MR_SHIFT) |
           (MDIO_CLOCK_RANGE << ETH_MACMIIAR_CR_SHIFT) | (write ? ETH_MACMIIAR_MW : 0UL) |
           ETH_MACMIIAR_MB;
}

static bool phy_read(uint32_t reg, uint32_t *value)
{
    ETH_MAC->MACMIIAR = mdio_command(reg, false);
    if (!mdio_wait())
    {
        return false;
    }
    *value = ETH_MAC->MACMIIDR & 0xFFFFUL;
    return true;
}

static bool phy_write(uint32_t reg, uint32_t value)
{
    ETH_MAC->MACMIIDR = value & 0xFFFFUL;
    ETH_MAC->MACMIIAR = mdio_command(reg, true);
    return mdio_wait();
}

static void init_descriptors(void)
{
    for (uint32_t i = 0U; i < RX_DESCRIPTORS; ++i)
    {
        dma_descriptor_t *d = &s_eth.rx[i];
        d->des1 = RDES1_RCH | BUFFER_SIZE;
        d->des2 = (uint32_t)s_rx_buffers[i];
        d->des3 = (uint32_t)&s_eth.rx[(i + 1U) % RX_DESCRIPTORS];
        d->des0 = DES0_OWN;
    }
    for (uint32_t i = 0U; i < TX_DESCRIPTORS; ++i)
    {
        dma_descriptor_t *d = &s_eth.tx[i];
        d->des0 = TDES0_TCH;
        d->des1 = 0U;
        d->des2 = (uint32_t)s_tx_buffers[i];
        d->des3 = (uint32_t)&s_eth.tx[(i + 1U) % TX_DESCRIPTORS];
    }
    s_eth.rx_next = 0U;
    s_eth.tx_next = 0U;
    memory_barrier();
    ETH_DMA->DMARDLAR = (uint32_t)&s_eth.rx[0];
    ETH_DMA->DMATDLAR = (uint32_t)&s_eth.tx[0];
}

bool eth_init(const uint8_t *hwaddr)
{
    s_eth = (eth_state_t){ 0 };
    board_eth_init();

    /* The DMA reset completes only with the PHY's reference clock running. */
    ETH_DMA->DMABMR |= ETH_DMABMR_SR;
    bool ok = false;
    for (uint32_t i = 0U; (i < POLL_LIMIT) && !ok; ++i)
    {
        ok = (ETH_DMA->DMABMR & ETH_DMABMR_SR) == 0U;
    }

    uint32_t id = 0xFFFFU;
    if (!ok || !phy_read(PHY_ID1, &id) || (id == 0U) || (id == 0xFFFFU))
    {
        return false;
    }
    if (!phy_write(PHY_BCR, PHY_BCR_RESET))
    {
        return false;
    }
    /* The reset bit self-clears when the PHY is ready for register writes (well under 1 ms). */
    uint32_t bcr = PHY_BCR_RESET;
    for (uint32_t i = 0U; (i < PHY_RESET_POLLS) && ((bcr & PHY_BCR_RESET) != 0U); ++i)
    {
        if (!phy_read(PHY_BCR, &bcr))
        {
            return false;
        }
    }
    if (((bcr & PHY_BCR_RESET) != 0U) || !phy_write(PHY_ANAR, PHY_ANAR_ALL) ||
        !phy_write(PHY_BCR, PHY_BCR_ANEG_ENABLE | PHY_BCR_ANEG_RESTART))
    {
        return false;
    }

    ETH_MAC->MACA0HR = ((uint32_t)hwaddr[5] << 8) | hwaddr[4];
    ETH_MAC->MACA0LR = ((uint32_t)hwaddr[3] << 24) | ((uint32_t)hwaddr[2] << 16) |
                       ((uint32_t)hwaddr[1] << 8) | hwaddr[0];
    /* Default filter: own unicast address and broadcast. */
    ETH_MAC->MACFFR = 0U;

    ETH_DMA->DMABMR = DMA_BURST_LENGTH << ETH_DMABMR_PBL_SHIFT;
    init_descriptors();

    /* Store-and-forward in both directions; transmission and reception start when the link is up. */
    ETH_DMA->DMAOMR = ETH_DMAOMR_RSF | ETH_DMAOMR_TSF;
    s_eth.link = ETH_LINK_DOWN;
    return true;
}

static eth_link_t negotiated_link(void)
{
    uint32_t advertised = 0U;
    uint32_t partner = 0U;

    if (!phy_read(PHY_ANAR, &advertised) || !phy_read(PHY_ANLPAR, &partner))
    {
        return ETH_LINK_DOWN;
    }

    const uint32_t common = advertised & partner;
    if ((common & PHY_ABILITY_100FD) != 0U)
    {
        return ETH_LINK_100_FULL;
    }
    if ((common & PHY_ABILITY_100HD) != 0U)
    {
        return ETH_LINK_100_HALF;
    }
    if ((common & PHY_ABILITY_10FD) != 0U)
    {
        return ETH_LINK_10_FULL;
    }
    /* 10 Mbit/s half duplex is the lowest common denominator. */
    return ETH_LINK_10_HALF;
}

static void start_mac(eth_link_t link)
{
    uint32_t maccr = ETH_MACCR_TE | ETH_MACCR_RE;

    if ((link == ETH_LINK_100_FULL) || (link == ETH_LINK_100_HALF))
    {
        maccr |= ETH_MACCR_FES;
    }
    if ((link == ETH_LINK_100_FULL) || (link == ETH_LINK_10_FULL))
    {
        maccr |= ETH_MACCR_DM;
    }

    ETH_DMA->DMAOMR |= ETH_DMAOMR_FTF;
    for (uint32_t i = 0U; (i < POLL_LIMIT) && ((ETH_DMA->DMAOMR & ETH_DMAOMR_FTF) != 0U); ++i)
    {
    }
    ETH_MAC->MACCR = maccr;
    ETH_DMA->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;
}

static void stop_mac(void)
{
    ETH_DMA->DMAOMR &= ~(ETH_DMAOMR_ST | ETH_DMAOMR_SR);
    ETH_MAC->MACCR &= ~(ETH_MACCR_TE | ETH_MACCR_RE);
}

eth_link_t eth_poll_link(void)
{
    uint32_t bsr = 0U;

    /* BSR link status latches low: the second read gives the current state. */
    const bool read_ok = phy_read(PHY_BSR, &bsr) && phy_read(PHY_BSR, &bsr);
    const bool up = read_ok && ((bsr & PHY_BSR_LINK) != 0U) && ((bsr & PHY_BSR_ANEG_DONE) != 0U);

    if (up && (s_eth.link == ETH_LINK_DOWN))
    {
        s_eth.link = negotiated_link();
        if (s_eth.link != ETH_LINK_DOWN)
        {
            start_mac(s_eth.link);
        }
    }
    else if (!up && (s_eth.link != ETH_LINK_DOWN))
    {
        stop_mac();
        s_eth.link = ETH_LINK_DOWN;
    }
    else
    {
        /* No change. */
    }
    return s_eth.link;
}

uint8_t *eth_tx_acquire(void)
{
    const dma_descriptor_t *d = &s_eth.tx[s_eth.tx_next];

    if ((s_eth.link == ETH_LINK_DOWN) || ((d->des0 & DES0_OWN) != 0U))
    {
        s_eth.stats.tx_busy++;
        return 0;
    }
    return (uint8_t *)s_tx_buffers[s_eth.tx_next];
}

void eth_tx_commit(size_t len)
{
    dma_descriptor_t *d = &s_eth.tx[s_eth.tx_next];

    d->des1 = (uint32_t)len & DES1_SIZE_MASK;
    memory_barrier();
    d->des0 = DES0_OWN | TDES0_FS | TDES0_LS | TDES0_TCH;
    memory_barrier();

    /* Resume the DMA in case it suspended on an empty ring. */
    ETH_DMA->DMASR = ETH_DMASR_TBUS;
    ETH_DMA->DMATPDR = 0U;

    s_eth.tx_next = (s_eth.tx_next + 1U) % TX_DESCRIPTORS;
    s_eth.stats.tx_frames++;
}

static void give_back_rx(dma_descriptor_t *d)
{
    memory_barrier();
    d->des0 = DES0_OWN;
    s_eth.rx_next = (s_eth.rx_next + 1U) % RX_DESCRIPTORS;

    if ((ETH_DMA->DMASR & ETH_DMASR_RBUS) != 0U)
    {
        /* The DMA ran out of descriptors and suspended: resume it. */
        ETH_DMA->DMASR = ETH_DMASR_RBUS;
        ETH_DMA->DMARPDR = 0U;
    }
}

size_t eth_rx_peek(const uint8_t **frame)
{
    for (uint32_t i = 0U; i < RX_DESCRIPTORS; ++i)
    {
        dma_descriptor_t *d = &s_eth.rx[s_eth.rx_next];
        const uint32_t des0 = d->des0;

        if ((des0 & DES0_OWN) != 0U)
        {
            return 0U;
        }

        const uint32_t whole = RDES0_FS | RDES0_LS;
        const size_t length = (size_t)((des0 >> RDES0_FL_SHIFT) & RDES0_FL_MASK);
        if (((des0 & RDES0_ES) != 0U) || ((des0 & whole) != whole) || (length <= CRC_LEN))
        {
            /* Bad frame, or larger than one buffer: drop it and look at the next one. */
            s_eth.stats.rx_errors++;
            give_back_rx(d);
            continue;
        }

        *frame = (const uint8_t *)s_rx_buffers[s_eth.rx_next];
        return length - CRC_LEN;
    }
    return 0U;
}

void eth_rx_release(void)
{
    s_eth.stats.rx_frames++;
    give_back_rx(&s_eth.rx[s_eth.rx_next]);
}

eth_stats_t eth_stats(void)
{
    return s_eth.stats;
}

const char *eth_link_name(eth_link_t link)
{
    switch (link)
    {
        case ETH_LINK_10_HALF:
            return "10 Mbit/s half duplex";
        case ETH_LINK_10_FULL:
            return "10 Mbit/s full duplex";
        case ETH_LINK_100_HALF:
            return "100 Mbit/s half duplex";
        case ETH_LINK_100_FULL:
            return "100 Mbit/s full duplex";
        default:
            return "down";
    }
}
