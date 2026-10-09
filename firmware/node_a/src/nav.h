/**
 * @file nav.h
 * @brief Heading and magnetometer integrity for Node A (HLR-008, HLR-010).
 *
 * The heading is the tilt-compensated magnetometer heading, corrected by the
 * local declination. The gravity direction used for the tilt compensation and
 * the dip check comes from a complementary filter (gyroscope propagation,
 * accelerometer correction), so it stays right while the vehicle accelerates. Near a loaded power line the conductors' field distorts
 * the measured field, so every sample is checked against the expected local
 * field: its strength and its dip angle below the horizontal. While the field
 * is disturbed, the magnetometer heading is not used; the GPS course over
 * ground replaces it when the vehicle is moving fast enough for it to mean
 * something.
 *
 * Axes: the accelerometer and magnetometer axes are assumed aligned (x forward,
 * y left, z up). On a real LSM9DS1 the magnetometer axes need remapping.
 */
#ifndef NAV_H
#define NAV_H

#include <stdbool.h>
#include <stdint.h>

/* Expected local geomagnetic field (from a world magnetic model for the mission area). */
#define NAV_FIELD_MGAUSS            (500.0f)
#define NAV_DIP_DEG                 (56.0f)
#define NAV_DECLINATION_DEG         (5.0f)

/* Disturbance limits and hysteresis. */
#define NAV_FIELD_TOLERANCE         (0.15f)     /* relative */
#define NAV_DIP_TOLERANCE_DEG       (8.0f)
#define NAV_DISTURBED_SAMPLES       (5U)        /* 50 ms at 100 Hz */
#define NAV_RECOVERY_SAMPLES        (100U)      /* 1 s at 100 Hz */
#define NAV_GPS_COURSE_MIN_CMPS     (100U)      /* below 1 m/s the GPS course is noise */

typedef enum
{
    NAV_HEADING_NONE = 0,
    NAV_HEADING_MAG,
    NAV_HEADING_GPS
} nav_heading_source_t;

typedef struct
{
    nav_heading_source_t source;
    uint16_t             heading_cdeg;      /**< True heading, 0.01 deg from north, clockwise. */
    bool                 mag_ok;            /**< Magnetometer field within limits. */
    uint16_t             field_mgauss;      /**< Measured field strength. */
    int16_t              dip_cdeg;          /**< Measured dip angle below the horizontal. */
    uint32_t             disturbances;      /**< Times the magnetometer was declared disturbed. */
} nav_state_t;

void nav_init(void);

/** Processes the newest IMU sample, if any, and the GPS data. Call from the main loop. */
void nav_poll(uint32_t now_ms);

nav_state_t nav_state(void);

/** Logs the heading, its source and the measured field. */
void nav_report(void);

#endif /* NAV_H */
