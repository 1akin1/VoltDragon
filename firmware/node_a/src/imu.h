/**
 * @file imu.h
 * @brief IMU sampling for Node A (HLR-007, HLR-010).
 *
 * ImuTask calls imu_sample() every 10 ms; the latest sample is kept together
 * with its validity, the achieved rate and an error count. All functions are
 * safe to call from any task.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "lsm9ds1.h"

#define IMU_SAMPLE_PERIOD_MS (10UL)

/** Brings up the sensor bus and the LSM9DS1. Returns false if the IMU is unusable. */
bool imu_init(void);

/** Reads one sample from the LSM9DS1. Called by ImuTask every IMU_SAMPLE_PERIOD_MS. */
void imu_sample(void);

/** Copies the latest sample. Returns false if there is no valid sample. */
bool imu_latest(lsm9ds1_sample_t *out);

/** Number of valid samples taken since start-up; changes whenever a new sample is available. */
uint32_t imu_sample_count(void);

/** Logs the sample rate since the previous report, the error count and the latest sample. */
void imu_report(void);

#endif /* IMU_H */
