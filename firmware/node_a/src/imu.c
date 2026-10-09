/**
 * @file imu.c
 * @brief IMU sampling for Node A. ImuTask calls imu_sample() every 10 ms.
 */
#include "imu.h"

#include "board.h"
#include "i2c.h"
#include "lock.h"
#include "log.h"

typedef struct
{
    bool present;
    bool valid;
    uint32_t samples_since_report;
    uint32_t sample_count;
    uint32_t errors_since_report;
    i2c_status_t last_error;
    lsm9ds1_sample_t latest;
} imu_state_t;

/* Written by ImuTask; read by the CAN, control and log tasks. */
static imu_state_t s_imu;
static lock_t s_lock;

bool imu_init(void)
{
    i2c_status_t bus_status = I2C_OK;

    s_imu = (imu_state_t){ 0 };
    lock_init(&s_lock);
    board_sensor_bus_init();
    i2c_init(BOARD_SENSOR_I2C, BOARD_PCLK1_HZ, BOARD_SENSOR_I2C_HZ);

    const lsm9ds1_status_t status = lsm9ds1_init(BOARD_SENSOR_I2C, &bus_status);
    if (status != LSM9DS1_OK)
    {
        LOG_ERROR("imu: init failed (%s, i2c %s), running without IMU",
                  lsm9ds1_status_name(status), i2c_status_name(bus_status));
        return false;
    }

    s_imu.present = true;
    LOG_INFO("imu: LSM9DS1 ready, sampling every %lu ms", IMU_SAMPLE_PERIOD_MS);
    return true;
}

void imu_sample(void)
{
    if (!s_imu.present)
    {
        return;
    }

    i2c_status_t bus_status = I2C_OK;
    lsm9ds1_sample_t sample;
    /* The I2C transfer runs outside the lock; only the result is published under it. */
    const bool ok = lsm9ds1_read(BOARD_SENSOR_I2C, &sample, &bus_status) == LSM9DS1_OK;

    lock_take(&s_lock);
    if (ok)
    {
        s_imu.latest = sample;
        s_imu.valid = true;
        s_imu.samples_since_report++;
        s_imu.sample_count++;
    }
    else
    {
        /* A failed read invalidates the sample so stale data is never used. */
        s_imu.valid = false;
        s_imu.errors_since_report++;
        s_imu.last_error = bus_status;
    }
    lock_give(&s_lock);
}

uint32_t imu_sample_count(void)
{
    lock_take(&s_lock);
    const uint32_t count = s_imu.sample_count;
    lock_give(&s_lock);
    return count;
}

bool imu_latest(lsm9ds1_sample_t *out)
{
    lock_take(&s_lock);
    const bool valid = s_imu.valid;
    if (valid)
    {
        *out = s_imu.latest;
    }
    lock_give(&s_lock);
    return valid;
}

void imu_report(void)
{
    if (!s_imu.present)
    {
        return;
    }

    /* Copy under the lock, log outside it: logging is slow. */
    lock_take(&s_lock);
    const imu_state_t snapshot = s_imu;
    s_imu.samples_since_report = 0U;
    s_imu.errors_since_report = 0U;
    lock_give(&s_lock);

    if (snapshot.errors_since_report != 0U)
    {
        LOG_WARN("imu: %lu read errors, last i2c %s", snapshot.errors_since_report,
                 i2c_status_name(snapshot.last_error));
    }
    if (!snapshot.valid)
    {
        LOG_WARN("imu: rate %lu Hz, no valid sample", snapshot.samples_since_report);
        return;
    }

    const lsm9ds1_sample_t *s = &snapshot.latest;
    LOG_INFO("imu: rate %lu Hz, acc %ld %ld %ld mg, gyro %ld %ld %ld mdps, mag %ld %ld %ld mG",
             snapshot.samples_since_report, s->accel_mg[0], s->accel_mg[1], s->accel_mg[2],
             s->gyro_mdps[0], s->gyro_mdps[1], s->gyro_mdps[2],
             s->mag_mgauss[0], s->mag_mgauss[1], s->mag_mgauss[2]);
}
