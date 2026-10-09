/**
 * @file gps.h
 * @brief GPS receiver on UART4: NMEA reception and the latest position fix.
 *
 * Bytes arrive by interrupt into a ring buffer; gps_poll() assembles sentences
 * and parses GGA and RMC (nmea.h). A fix older than GPS_STALE_MS is treated as
 * lost (HLR-010).
 */
#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

#include "nmea.h"

#define GPS_STALE_MS (1000UL)

/** Configures UART4 at 9600 baud and starts interrupt-driven reception. */
void gps_init(void);

/** Parses every complete sentence received so far. Call from the main loop. */
void gps_poll(uint32_t now_ms);

/**
 * Copies the latest data. Returns true if it holds a position fix that is
 * younger than GPS_STALE_MS.
 */
bool gps_latest(nmea_fix_t *out, uint32_t now_ms);

/** Milliseconds since the last position fix was received (UINT32_MAX if none yet). */
uint32_t gps_fix_age_ms(uint32_t now_ms);

/** Logs the sentences received in the last second, error counts and the fix. */
void gps_report(uint32_t now_ms);

#endif /* GPS_H */
