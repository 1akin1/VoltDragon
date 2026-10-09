/**
 * @file lsm9ds1.c
 * @brief ST LSM9DS1 9-axis IMU driver. Register map from the LSM9DS1 datasheet (DocID025715).
 */
#include "lsm9ds1.h"

#include <stdbool.h>

/* Accelerometer/gyroscope registers. */
#define AG_WHO_AM_I         (0x0FU)
#define AG_CTRL_REG1_G      (0x10U)
#define AG_OUT_X_L_G        (0x18U)
#define AG_CTRL_REG6_XL     (0x20U)
#define AG_CTRL_REG8        (0x22U)
#define AG_OUT_X_L_XL       (0x28U)
#define AG_WHO_AM_I_VALUE   (0x68U)

/* Magnetometer registers. */
#define M_WHO_AM_I          (0x0FU)
#define M_CTRL_REG1_M       (0x20U)
#define M_CTRL_REG2_M       (0x21U)
#define M_CTRL_REG3_M       (0x22U)
#define M_CTRL_REG5_M       (0x24U)
#define M_OUT_X_L_M         (0x28U)
#define M_WHO_AM_I_VALUE    (0x3DU)
/* The magnetometer only auto-increments over I2C if the sub-address MSB is set. */
#define M_AUTO_INCREMENT    (0x80U)

/* ODR_G = 119 Hz, FS_G = 245 dps. */
#define CTRL_REG1_G_VALUE   (0x60U)
/* ODR_XL = 119 Hz, FS_XL = 2 g. */
#define CTRL_REG6_XL_VALUE  (0x60U)
/* BDU = 1, IF_ADD_INC = 1 (reset default). */
#define CTRL_REG8_VALUE     (0x44U)
/* TEMP_COMP = 1, OM = ultra-high performance, DO = 80 Hz. */
#define CTRL_REG1_M_VALUE   (0xFCU)
/* FS = 4 gauss. */
#define CTRL_REG2_M_VALUE   (0x00U)
/* MD = continuous conversion. */
#define CTRL_REG3_M_VALUE   (0x00U)
/* BDU = 1. */
#define CTRL_REG5_M_VALUE   (0x40U)

/* Sensitivities from the datasheet, as num/den: 0.061 mg, 8.75 mdps, 0.14 mgauss per LSB. */
#define ACCEL_NUM   (61)
#define ACCEL_DEN   (1000)
#define GYRO_NUM    (875)
#define GYRO_DEN    (100)
#define MAG_NUM     (14)
#define MAG_DEN     (100)

#define AXES        (3U)
#define AXIS_BYTES  (6U)

typedef struct
{
    uint8_t addr;
    uint8_t reg;
    uint8_t value;
} reg_write_t;

static const reg_write_t s_config[] = {
    { LSM9DS1_AG_ADDR, AG_CTRL_REG8, CTRL_REG8_VALUE },
    { LSM9DS1_AG_ADDR, AG_CTRL_REG1_G, CTRL_REG1_G_VALUE },
    { LSM9DS1_AG_ADDR, AG_CTRL_REG6_XL, CTRL_REG6_XL_VALUE },
    { LSM9DS1_M_ADDR, M_CTRL_REG1_M, CTRL_REG1_M_VALUE },
    { LSM9DS1_M_ADDR, M_CTRL_REG2_M, CTRL_REG2_M_VALUE },
    { LSM9DS1_M_ADDR, M_CTRL_REG3_M, CTRL_REG3_M_VALUE },
    { LSM9DS1_M_ADDR, M_CTRL_REG5_M, CTRL_REG5_M_VALUE },
};

#define CONFIG_COUNT (sizeof(s_config) / sizeof(s_config[0]))

/** Scales a raw reading by num/den, rounding half away from zero. */
static int32_t scale(int16_t raw, int32_t num, int32_t den)
{
    const int32_t product = (int32_t)raw * num;
    const int32_t half = den / 2;

    return (product >= 0) ? ((product + half) / den) : ((product - half) / den);
}

/** Converts three little-endian 16-bit axes to engineering units. */
static void convert_axes(const uint8_t *bytes, int32_t *out, int32_t num, int32_t den)
{
    for (uint32_t axis = 0U; axis < AXES; ++axis)
    {
        const uint32_t lo = bytes[2U * axis];
        const uint32_t hi = bytes[(2U * axis) + 1U];
        const int16_t raw = (int16_t)(uint16_t)((hi << 8) | lo);

        out[axis] = scale(raw, num, den);
    }
}

static bool check_id(i2c_regs_t *i2c, uint8_t addr, uint8_t expected, i2c_status_t *bus_status)
{
    uint8_t id = 0U;

    *bus_status = i2c_read_regs(i2c, addr, AG_WHO_AM_I, &id, 1U);
    return (*bus_status == I2C_OK) && (id == expected);
}

lsm9ds1_status_t lsm9ds1_init(i2c_regs_t *i2c, i2c_status_t *bus_status)
{
    /* AG_WHO_AM_I and M_WHO_AM_I share the address 0x0F. */
    if (!check_id(i2c, LSM9DS1_AG_ADDR, AG_WHO_AM_I_VALUE, bus_status))
    {
        return (*bus_status == I2C_OK) ? LSM9DS1_ERR_AG_ID : LSM9DS1_ERR_BUS;
    }
    if (!check_id(i2c, LSM9DS1_M_ADDR, M_WHO_AM_I_VALUE, bus_status))
    {
        return (*bus_status == I2C_OK) ? LSM9DS1_ERR_M_ID : LSM9DS1_ERR_BUS;
    }

    for (uint32_t i = 0U; i < CONFIG_COUNT; ++i)
    {
        *bus_status = i2c_write_reg(i2c, s_config[i].addr, s_config[i].reg, s_config[i].value);
        if (*bus_status != I2C_OK)
        {
            return LSM9DS1_ERR_BUS;
        }
    }
    return LSM9DS1_OK;
}

lsm9ds1_status_t lsm9ds1_read(i2c_regs_t *i2c, lsm9ds1_sample_t *out, i2c_status_t *bus_status)
{
    uint8_t accel[AXIS_BYTES];
    uint8_t gyro[AXIS_BYTES];
    uint8_t mag[AXIS_BYTES];

    *bus_status = i2c_read_regs(i2c, LSM9DS1_AG_ADDR, AG_OUT_X_L_XL, accel, AXIS_BYTES);
    if (*bus_status == I2C_OK)
    {
        *bus_status = i2c_read_regs(i2c, LSM9DS1_AG_ADDR, AG_OUT_X_L_G, gyro, AXIS_BYTES);
    }
    if (*bus_status == I2C_OK)
    {
        *bus_status = i2c_read_regs(i2c, LSM9DS1_M_ADDR, M_OUT_X_L_M | M_AUTO_INCREMENT, mag,
                                    AXIS_BYTES);
    }
    if (*bus_status != I2C_OK)
    {
        return LSM9DS1_ERR_BUS;
    }

    convert_axes(accel, out->accel_mg, ACCEL_NUM, ACCEL_DEN);
    convert_axes(gyro, out->gyro_mdps, GYRO_NUM, GYRO_DEN);
    convert_axes(mag, out->mag_mgauss, MAG_NUM, MAG_DEN);
    return LSM9DS1_OK;
}

const char *lsm9ds1_status_name(lsm9ds1_status_t status)
{
    switch (status)
    {
        case LSM9DS1_OK:
            return "OK";
        case LSM9DS1_ERR_BUS:
            return "BUS";
        case LSM9DS1_ERR_AG_ID:
            return "AG_ID";
        case LSM9DS1_ERR_M_ID:
            return "M_ID";
        default:
            return "?";
    }
}
