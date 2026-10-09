/**
 * @file imu.h
 * @brief Periodic IMU sampling for Node A (HLR-007, HLR-010).
 *
 * Samples the LSM9DS1 every 10 ms from the main loop and keeps the latest
 * sample together with its validity, the achieved rate and an error count.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "lsm9ds1.h"

#define IMU_SAMPLE_PERIOD_MS (10UL)

/** Brings up the sensor bus and the LSM9DS1. Returns false if the IMU is unusable. */
bool imu_init(void);

/** Takes a sample if one is due. Call from the main loop. */
void imu_poll(uint32_t now_ms);

/** Copies the latest sample. Returns false if there is no valid sample. */
bool imu_latest(lsm9ds1_sample_t *out);

/** Logs the sample rate since the previous report, the error count and the latest sample. */
void imu_report(void);

#endif /* IMU_H */
