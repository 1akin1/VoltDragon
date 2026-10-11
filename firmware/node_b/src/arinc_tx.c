/**
 * @file arinc_tx.c
 * @brief Node B's ARINC 429 output (HLR-015).
 */
#include "arinc_tx.h"

#include <stddef.h>

#include "arinc429.h"
#include "board.h"
#include "can_msg.h"
#include "can_rx.h"
#include "log.h"
#include "spi.h"
#include "systick.h"
#include "tlm_msg.h"

/* The transmitter's command set (docs/arinc429.md). */
#define OP_RESET        (0x01U)
#define OP_TRANSMIT     (0x02U)
#define OP_READ_ID      (0x03U)
#define TRANSMITTER_ID  (0xA4U)

#define WORDS           (3U)
#define WORD_ALTITUDE   (0U)
#define WORD_HEADING    (1U)
#define WORD_DISTANCE   (2U)

typedef struct
{
    bool     ready;
    bool     started;
    uint32_t next_due_ms;
    uint32_t sent_since_report;
    uint32_t errors;
    uint32_t ssm[WORDS];
} arinc_tx_state_t;

static arinc_tx_state_t s_tx;

static bool command(const uint8_t *tx, uint8_t *rx, size_t len)
{
    board_arinc_select(true);
    const bool ok = spi_transfer(BOARD_ARINC_SPI, tx, rx, len);
    board_arinc_select(false);
    return ok;
}

bool arinc_tx_init(void)
{
    static const uint8_t RESET[1] = { OP_RESET };
    static const uint8_t READ_ID[2] = { OP_READ_ID, 0x00U };
    uint8_t reply[2] = { 0 };

    s_tx = (arinc_tx_state_t){ 0 };
    board_arinc_bus_init();
    spi_init(BOARD_ARINC_SPI, BOARD_ARINC_SPI_BR);

    const bool answered = command(RESET, NULL, sizeof(RESET)) &&
                          command(READ_ID, reply, sizeof(READ_ID));
    if (!answered || (reply[1] != TRANSMITTER_ID))
    {
        LOG_ERROR("arinc: no transmitter on SPI2 (id 0x%02x), running without ARINC 429 output",
                  (unsigned int)reply[1]);
        return false;
    }
    s_tx.ready = true;
    LOG_INFO("arinc: transmitter ready; labels 203, 314 and 251 every %lu ms", ARINC_TX_PERIOD_MS);
    return true;
}

/** The status of a value Node A may or may not have (its data being fresh). */
static uint32_t status(bool available)
{
    uint32_t ssm = ARINC429_SSM_NO_COMPUTED_DATA;

    if (available)
    {
        ssm = ARINC429_SSM_NORMAL;
    }
    return ssm;
}

/** The three words from Node A's latest data, with the status of each. */
static void build(uint32_t now_ms, uint32_t words[WORDS])
{
    can_rx_node_a_t a;
    uint32_t ssm[WORDS] = { ARINC429_SSM_FAILURE_WARNING, ARINC429_SSM_FAILURE_WARNING,
                            ARINC429_SSM_FAILURE_WARNING };
    int32_t alt_dm = 0;
    int32_t heading_cdeg = 0;
    uint32_t distance_dm = 0U;

    if (can_rx_node_a(&a) && ((now_ms - a.last_valid_ms) < TLM_FRESH_MS))
    {
        const bool gps_fix = a.any_gps && ((now_ms - a.gps_ms) < TLM_GPS_FRESH_MS) &&
                             ((a.nav.flags & CANMSG_NAV_GPS_FIX) != 0U);
        const uint32_t source = ((uint32_t)a.nav.flags & CANMSG_NAV_SOURCE_MASK) >>
                                CANMSG_NAV_SOURCE_SHIFT;
        const bool distance = a.any_safety && (a.safety.distance_dm != CANMSG_UNKNOWN_U16);

        ssm[WORD_ALTITUDE] = status(gps_fix);
        ssm[WORD_HEADING] = status(source != 0U);
        ssm[WORD_DISTANCE] = status(distance);
        alt_dm = a.gps.alt_msl_dm;
        heading_cdeg = (int32_t)a.nav.heading_cdeg;
        distance_dm = a.safety.distance_dm;
    }
    words[WORD_ALTITUDE] = arinc429_altitude(alt_dm, ssm[WORD_ALTITUDE]);
    words[WORD_HEADING] = arinc429_true_heading(heading_cdeg, ssm[WORD_HEADING]);
    words[WORD_DISTANCE] = arinc429_line_distance(distance_dm, ssm[WORD_DISTANCE]);
    for (uint32_t i = 0U; i < WORDS; ++i)
    {
        s_tx.ssm[i] = ssm[i];
    }
}

static void send(uint32_t word)
{
    const uint8_t frame[5] = {
        OP_TRANSMIT,
        (uint8_t)(word >> 24),
        (uint8_t)((word >> 16) & 0xFFUL),
        (uint8_t)((word >> 8) & 0xFFUL),
        (uint8_t)(word & 0xFFUL),
    };

    if (command(frame, NULL, sizeof(frame)))
    {
        s_tx.sent_since_report++;
    }
    else
    {
        s_tx.errors++;
    }
}

void arinc_tx_poll(uint32_t now_ms)
{
    uint32_t words[WORDS];

    if (!s_tx.ready)
    {
        return;
    }
    if (!s_tx.started)
    {
        s_tx.started = true;
        s_tx.next_due_ms = now_ms;
    }
    if (!systick_reached(now_ms, s_tx.next_due_ms))
    {
        return;
    }
    /* A steady 20 Hz, without a burst after a stall. */
    s_tx.next_due_ms = (((now_ms - s_tx.next_due_ms) >= ARINC_TX_PERIOD_MS) ?
                        (now_ms + ARINC_TX_PERIOD_MS) : (s_tx.next_due_ms + ARINC_TX_PERIOD_MS));

    build(now_ms, words);
    for (uint32_t i = 0U; i < WORDS; ++i)
    {
        send(words[i]);
    }
}

static const char *ssm_name(uint32_t ssm)
{
    static const char *const NAMES[4] = { "FW", "NCD", "FT", "NO" };
    return NAMES[ssm & 3U];
}

void arinc_tx_report(void)
{
    if (!s_tx.ready)
    {
        return;
    }
    LOG_INFO("arinc: %lu words/s, SPI errors %lu; altitude %s, heading %s, line distance %s",
             s_tx.sent_since_report, s_tx.errors, ssm_name(s_tx.ssm[WORD_ALTITUDE]),
             ssm_name(s_tx.ssm[WORD_HEADING]), ssm_name(s_tx.ssm[WORD_DISTANCE]));
    s_tx.sent_since_report = 0U;
}
