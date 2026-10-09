/**
 * @file imu.c
 * @brief Periodic IMU sampling for Node A.
 */
#include "imu.h"

#include "board.h"
#include "i2c.h"
#include "log.h"
#include "systick.h"

/* If the loop falls further behind than this, skip the missed samples instead of bursting. */
#define MAX_LAG_MS  (2UL * IMU_SAMPLE_PERIOD_MS)

typedef struct
{
    bool present;
    bool valid;
    uint32_t next_due_ms;
    uint32_t samples_since_report;
    uint32_t sample_count;
    uint32_t errors_since_report;
    i2c_status_t last_error;
    lsm9ds1_sample_t latest;
} imu_state_t;

static imu_state_t s_imu;

bool imu_init(void)
{
    i2c_status_t bus_status = I2C_OK;

    s_imu = (imu_state_t){ 0 };
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
    s_imu.next_due_ms = systick_now_ms();
    LOG_INFO("imu: LSM9DS1 ready, sampling every %lu ms", IMU_SAMPLE_PERIOD_MS);
    return true;
}

void imu_poll(uint32_t now_ms)
{
    if (!s_imu.present)
    {
        return;
    }
    /* Signed difference handles counter wrap-around. */
    if ((int32_t)(now_ms - s_imu.next_due_ms) < 0)
    {
        return;
    }
    if ((now_ms - s_imu.next_due_ms) >= MAX_LAG_MS)
    {
        s_imu.next_due_ms = now_ms;
    }
    s_imu.next_due_ms += IMU_SAMPLE_PERIOD_MS;

    i2c_status_t bus_status = I2C_OK;
    lsm9ds1_sample_t sample;

    if (lsm9ds1_read(BOARD_SENSOR_I2C, &sample, &bus_status) == LSM9DS1_OK)
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
}

uint32_t imu_sample_count(void)
{
    return s_imu.sample_count;
}

bool imu_latest(lsm9ds1_sample_t *out)
{
    if (s_imu.valid)
    {
        *out = s_imu.latest;
    }
    return s_imu.valid;
}

void imu_report(void)
{
    if (!s_imu.present)
    {
        return;
    }

    const uint32_t rate_hz = s_imu.samples_since_report;
    const uint32_t errors = s_imu.errors_since_report;
    s_imu.samples_since_report = 0U;
    s_imu.errors_since_report = 0U;

    if (errors != 0U)
    {
        LOG_WARN("imu: %lu read errors, last i2c %s", errors, i2c_status_name(s_imu.last_error));
    }
    if (!s_imu.valid)
    {
        LOG_WARN("imu: rate %lu Hz, no valid sample", rate_hz);
        return;
    }

    const lsm9ds1_sample_t *s = &s_imu.latest;
    LOG_INFO("imu: rate %lu Hz, acc %ld %ld %ld mg, gyro %ld %ld %ld mdps, mag %ld %ld %ld mG",
             rate_hz, s->accel_mg[0], s->accel_mg[1], s->accel_mg[2],
             s->gyro_mdps[0], s->gyro_mdps[1], s->gyro_mdps[2],
             s->mag_mgauss[0], s->mag_mgauss[1], s->mag_mgauss[2]);
}
