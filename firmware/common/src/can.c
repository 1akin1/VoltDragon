/**
 * @file can.c
 * @brief Polled, register-level bxCAN driver.
 */
#include "can.h"

#include <string.h>

#include "board.h"

/* Upper bound on mode-change polls; leaving init needs 11 recessive bits (22 us at 500 kbit/s). */
#define CAN_POLL_LIMIT      (100000UL)
/* CAN1 owns filter banks 0..13 while CAN2SB keeps its reset value of 14. */
#define CAN1_FILTER_BANKS   (14U)
#define BTR_BRP_SHIFT       (0U)
#define BTR_TS1_SHIFT       (16U)
#define BTR_TS2_SHIFT       (20U)
#define BTR_SJW_SHIFT       (24U)

typedef struct
{
    can_frame_t queue[CAN_TX_QUEUE_LEN];
    uint32_t    head;
    uint32_t    count;
    can_stats_t stats;
} can_state_t;

static can_state_t s_can;

static bool wait_msr(uint32_t flag, bool set)
{
    for (uint32_t i = 0U; i < CAN_POLL_LIMIT; ++i)
    {
        if (((BOARD_CAN->MSR & flag) != 0U) == set)
        {
            return true;
        }
    }
    return false;
}

static uint32_t pack_data(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) |
           ((uint32_t)bytes[3] << 24);
}

static void unpack_data(uint32_t word, uint8_t *bytes)
{
    bytes[0] = (uint8_t)word;
    bytes[1] = (uint8_t)(word >> 8);
    bytes[2] = (uint8_t)(word >> 16);
    bytes[3] = (uint8_t)(word >> 24);
}

bool can_init(void)
{
    s_can = (can_state_t){ 0 };
    board_can_init();

    /* Leave sleep mode and request initialisation mode. */
    BOARD_CAN->MCR = (BOARD_CAN->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
    if (!wait_msr(CAN_MSR_INAK, true) || !wait_msr(CAN_MSR_SLAK, false))
    {
        return false;
    }

    /* Transmit in request order (TXFP); recover from bus-off automatically (ABOM). */
    BOARD_CAN->MCR = CAN_MCR_INRQ | CAN_MCR_TXFP | CAN_MCR_ABOM;
    BOARD_CAN->BTR = ((BOARD_CAN_PRESCALER - 1UL) << BTR_BRP_SHIFT) |
                     ((BOARD_CAN_BS1_TQ - 1UL) << BTR_TS1_SHIFT) |
                     ((BOARD_CAN_BS2_TQ - 1UL) << BTR_TS2_SHIFT) |
                     (0UL << BTR_SJW_SHIFT);

    /* No filter active yet: nothing is received until can_add_filter(). */
    BOARD_CAN->FMR |= CAN_FMR_FINIT;
    BOARD_CAN->FA1R = 0U;
    BOARD_CAN->FMR &= ~CAN_FMR_FINIT;

    /* Leave initialisation mode; the controller joins the bus after 11 recessive bits. */
    BOARD_CAN->MCR &= ~CAN_MCR_INRQ;
    return wait_msr(CAN_MSR_INAK, false);
}

bool can_add_filter(uint32_t bank, uint16_t id, uint16_t mask)
{
    if ((bank >= CAN1_FILTER_BANKS) || (id > CAN_STD_ID_MAX))
    {
        return false;
    }

    const uint32_t bit = 1UL << bank;

    BOARD_CAN->FMR |= CAN_FMR_FINIT;
    BOARD_CAN->FA1R &= ~bit;
    BOARD_CAN->FM1R &= ~bit;            /* identifier-mask mode */
    BOARD_CAN->FS1R |= bit;             /* one 32-bit filter */
    BOARD_CAN->FFA1R &= ~bit;           /* to FIFO 0 */
    /* Match standard data frames only: IDE and RTR must be 0. */
    BOARD_CAN->FILTER[bank].FR1 = (uint32_t)id << CAN_ID_STD_SHIFT;
    BOARD_CAN->FILTER[bank].FR2 = ((uint32_t)(mask & CAN_STD_ID_MAX) << CAN_ID_STD_SHIFT) |
                                  CAN_ID_IDE | CAN_ID_RTR;
    BOARD_CAN->FA1R |= bit;
    BOARD_CAN->FMR &= ~CAN_FMR_FINIT;
    return true;
}

bool can_send(const can_frame_t *frame)
{
    if ((s_can.count >= CAN_TX_QUEUE_LEN) || (frame->id > CAN_STD_ID_MAX) ||
        (frame->dlc > CAN_DATA_MAX))
    {
        s_can.stats.tx_dropped++;
        return false;
    }
    s_can.queue[(s_can.head + s_can.count) % CAN_TX_QUEUE_LEN] = *frame;
    s_can.count++;
    s_can.stats.tx_queued++;
    can_poll();
    return true;
}

void can_poll(void)
{
    while (s_can.count > 0U)
    {
        uint32_t mailbox = CAN_TX_MAILBOXES;

        for (uint32_t i = 0U; i < CAN_TX_MAILBOXES; ++i)
        {
            if ((BOARD_CAN->TSR & (CAN_TSR_TME0 << i)) != 0U)
            {
                mailbox = i;
                break;
            }
        }
        if (mailbox == CAN_TX_MAILBOXES)
        {
            return;
        }

        const can_frame_t *frame = &s_can.queue[s_can.head];
        uint8_t data[CAN_DATA_MAX] = { 0U };
        (void)memcpy(data, frame->data, frame->dlc);

        can_tx_mailbox_t *mb = &BOARD_CAN->TX[mailbox];
        mb->TDTR = frame->dlc;
        mb->TDLR = pack_data(&data[0]);
        mb->TDHR = pack_data(&data[4]);
        /* Writing TXRQ last hands the mailbox to the controller. */
        mb->TIR = ((uint32_t)frame->id << CAN_ID_STD_SHIFT) | CAN_TIR_TXRQ;

        s_can.head = (s_can.head + 1U) % CAN_TX_QUEUE_LEN;
        s_can.count--;
    }
}

bool can_receive(can_frame_t *frame)
{
    const uint32_t rf0r = BOARD_CAN->RF0R;

    if ((rf0r & CAN_RF0R_FOVR0) != 0U)
    {
        /* rc_w1: writing 1 clears the overrun flag. */
        BOARD_CAN->RF0R = CAN_RF0R_FOVR0;
        s_can.stats.rx_overruns++;
    }
    if ((rf0r & CAN_RF0R_FMP0_MASK) == 0U)
    {
        return false;
    }

    const can_fifo_mailbox_t *mb = &BOARD_CAN->RX[0];
    uint8_t data[CAN_DATA_MAX];

    frame->id = (uint16_t)((mb->RIR >> CAN_ID_STD_SHIFT) & CAN_STD_ID_MAX);
    frame->dlc = (uint8_t)(mb->RDTR & CAN_DLC_MASK);
    if (frame->dlc > CAN_DATA_MAX)
    {
        /* DLC 9..15 means 8 bytes in classic CAN. */
        frame->dlc = CAN_DATA_MAX;
    }
    unpack_data(mb->RDLR, &data[0]);
    unpack_data(mb->RDHR, &data[4]);
    (void)memcpy(frame->data, data, CAN_DATA_MAX);

    /* Release the output mailbox so the next frame moves up. */
    BOARD_CAN->RF0R = CAN_RF0R_RFOM0;
    s_can.stats.rx_frames++;
    return true;
}

can_stats_t can_stats(void)
{
    const uint32_t esr = BOARD_CAN->ESR;
    can_stats_t stats = s_can.stats;

    stats.tx_errors = (uint8_t)(esr >> CAN_ESR_TEC_SHIFT);
    stats.rx_errors = (uint8_t)(esr >> CAN_ESR_REC_SHIFT);
    stats.bus_off = (esr & CAN_ESR_BOFF) != 0U;
    return stats;
}
