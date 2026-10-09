/**
 * @file gps.c
 * @brief GPS receiver on UART4: NMEA reception and the latest position fix.
 */
#include "gps.h"

#include "board.h"
#include "cmd_protocol.h"
#include "log.h"
#include "uart.h"

#define DEGREE_E7   (10000000L)

typedef struct
{
    uart_rx_buffer_t rx;
    cmd_line_t       line;
    nmea_fix_t       fix;
    bool             have_fix;
    uint32_t         fix_ms;            /* Time of the last GGA that reported a fix. */
    uint32_t         sentences;
    uint32_t         reported_sentences;
    uint32_t         errors;
    nmea_result_t    last_error;
} gps_state_t;

static gps_state_t s_gps;

void UART4_IRQHandler(void);

void UART4_IRQHandler(void)
{
    uart_rx_irq_handler(BOARD_GPS_UART, &s_gps.rx);
}

void gps_init(void)
{
    s_gps = (gps_state_t){ 0 };
    cmd_line_reset(&s_gps.line);

    board_gps_uart_init();
    uart_init(BOARD_GPS_UART, BOARD_PCLK1_HZ, BOARD_GPS_BAUD);
    uart_rx_irq_start(BOARD_GPS_UART, &s_gps.rx, BOARD_GPS_IRQN);
    LOG_INFO("gps: listening on UART4 at %lu baud", BOARD_GPS_BAUD);
}

void gps_poll(uint32_t now_ms)
{
    char c;

    while (uart_rx_pop(&s_gps.rx, &c))
    {
        const cmd_line_event_t event = cmd_line_feed(&s_gps.line, c);

        if (event == CMD_LINE_TOO_LONG)
        {
            s_gps.errors++;
            s_gps.last_error = NMEA_ERR_FRAME;
            continue;
        }
        if (event != CMD_LINE_READY)
        {
            continue;
        }

        const nmea_result_t result = nmea_parse(s_gps.line.buf, &s_gps.fix);
        if ((result == NMEA_GGA) || (result == NMEA_RMC))
        {
            s_gps.sentences++;
            if ((result == NMEA_GGA) && s_gps.fix.fix)
            {
                s_gps.have_fix = true;
                s_gps.fix_ms = now_ms;
            }
        }
        else if (result != NMEA_IGNORED)
        {
            s_gps.errors++;
            s_gps.last_error = result;
        }
        else
        {
            /* Another sentence type: not used. */
        }
    }
}

bool gps_latest(nmea_fix_t *out, uint32_t now_ms)
{
    *out = s_gps.fix;
    return s_gps.have_fix && s_gps.fix.fix && ((now_ms - s_gps.fix_ms) < GPS_STALE_MS);
}

/** Prints degrees x 1e7 as a signed decimal with seven places. */
static void format_degrees(int32_t e7, char *sign, uint32_t *whole, uint32_t *fraction)
{
    const uint32_t magnitude = (e7 < 0) ? (0U - (uint32_t)e7) : (uint32_t)e7;

    *sign = (e7 < 0) ? '-' : ' ';
    *whole = magnitude / (uint32_t)DEGREE_E7;
    *fraction = magnitude % (uint32_t)DEGREE_E7;
}

void gps_report(uint32_t now_ms)
{
    nmea_fix_t fix;
    const bool valid = gps_latest(&fix, now_ms);
    const uint32_t rate = s_gps.sentences - s_gps.reported_sentences;

    s_gps.reported_sentences = s_gps.sentences;
    if (s_gps.errors != 0U)
    {
        LOG_WARN("gps: %lu bad sentences, last %s", s_gps.errors,
                 nmea_result_name(s_gps.last_error));
    }
    if ((s_gps.rx.overruns + s_gps.rx.dropped) != 0U)
    {
        LOG_WARN("gps: receive errors: %lu overruns, %lu dropped", s_gps.rx.overruns,
                 s_gps.rx.dropped);
    }
    if (!valid)
    {
        LOG_WARN("gps: no fix (%lu sentences/s)", rate);
        return;
    }

    char lat_sign;
    char lon_sign;
    uint32_t lat_whole;
    uint32_t lat_fraction;
    uint32_t lon_whole;
    uint32_t lon_fraction;
    format_degrees(fix.lat_e7, &lat_sign, &lat_whole, &lat_fraction);
    format_degrees(fix.lon_e7, &lon_sign, &lon_whole, &lon_fraction);
    LOG_INFO("gps: %lu sentences/s, fix q%u sats %u, lat%c%lu.%07lu lon%c%lu.%07lu alt %ld cm, "
             "%lu cm/s course %u cdeg",
             rate, (unsigned int)fix.quality, (unsigned int)fix.satellites, lat_sign, lat_whole,
             lat_fraction, lon_sign, lon_whole, lon_fraction, fix.alt_msl_cm, fix.speed_cmps,
             (unsigned int)fix.course_cdeg);
}
