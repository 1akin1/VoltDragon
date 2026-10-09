/**
 * @file lsm9ds1.h
 * @brief ST LSM9DS1 9-axis IMU (accelerometer, gyroscope, magnetometer) over I2C.
 *
 * Configuration: accelerometer and gyroscope at 119 Hz output data rate,
 * +/-2 g and +/-245 dps full scale; magnetometer at 80 Hz, +/-4 gauss,
 * continuous conversion. Block data update is enabled so the low and high
 * bytes of an axis always come from the same sample.
 */
#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stdint.h>

#include "i2c.h"

/** 7-bit I2C addresses with SDO_AG and SDO_M tied high. */
#define LSM9DS1_AG_ADDR (0x6BU)
#define LSM9DS1_M_ADDR  (0x1EU)

typedef enum
{
    LSM9DS1_OK = 0,
    LSM9DS1_ERR_BUS,        /**< An I2C transfer failed; see the I2C status. */
    LSM9DS1_ERR_AG_ID,      /**< Accelerometer/gyroscope WHO_AM_I mismatch. */
    LSM9DS1_ERR_M_ID        /**< Magnetometer WHO_AM_I mismatch. */
} lsm9ds1_status_t;

/** One sample in engineering units, X/Y/Z order. */
typedef struct
{
    int32_t accel_mg[3];        /**< Acceleration in milli-g. */
    int32_t gyro_mdps[3];       /**< Angular rate in milli-degrees per second. */
    int32_t mag_mgauss[3];      /**< Magnetic field in milli-gauss. */
} lsm9ds1_sample_t;

/**
 * Checks both WHO_AM_I registers and configures the device.
 * @param[out] bus_status  I2C status of the first failed transfer, or I2C_OK.
 */
lsm9ds1_status_t lsm9ds1_init(i2c_regs_t *i2c, i2c_status_t *bus_status);

/**
 * Reads the latest accelerometer, gyroscope and magnetometer outputs.
 * @param[out] bus_status  I2C status of the first failed transfer, or I2C_OK.
 */
lsm9ds1_status_t lsm9ds1_read(i2c_regs_t *i2c, lsm9ds1_sample_t *out, i2c_status_t *bus_status);

/** Short name of a status code, for logging. */
const char *lsm9ds1_status_name(lsm9ds1_status_t status);

#endif /* LSM9DS1_H */
