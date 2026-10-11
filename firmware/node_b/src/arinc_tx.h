/**
 * @file arinc_tx.h
 * @brief Node B's ARINC 429 output (HLR-015).
 *
 * Every 50 ms Node B sends three BNR words to the ARINC 429 transmitter on SPI2:
 * altitude (label 203), true heading (314) and the distance to the nearest
 * conductor (251), from Node A's latest data. The status of each word (SSM):
 *
 * - failure warning: Node A's data is older than TLM_FRESH_MS (as in telemetry);
 * - no computed data: Node A has no value (no GPS fix, no heading source, no distance);
 * - normal operation otherwise.
 *
 * Without a transmitter on the bus Node B logs it once and runs without the output.
 * Words, labels and the transmitter's command set: docs/arinc429.md.
 */
#ifndef ARINC_TX_H
#define ARINC_TX_H

#include <stdbool.h>
#include <stdint.h>

#define ARINC_TX_PERIOD_MS  (50UL)

/** Starts SPI2 and checks the transmitter's identifier. False: running without ARINC output. */
bool arinc_tx_init(void);

/** Sends the three words when they are due. Main loop. */
void arinc_tx_poll(uint32_t now_ms);

/** Logs the words sent in the last second, transfer errors and each label's status. */
void arinc_tx_report(void);

#endif /* ARINC_TX_H */
