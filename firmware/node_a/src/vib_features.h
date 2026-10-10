/**
 * @file vib_features.h
 * @brief Vibration analysis windows and spectral features for the fault classifier (HLR-009).
 *
 * The IMU samples go into a window of VIB_WINDOW samples (0.64 s at 100 Hz);
 * a new window is complete every VIB_HOP samples (0.32 s). For each of the six
 * axes (accelerometer in g, gyroscope in deg/s) the features are:
 *
 *   1. the window's mean removed,
 *   2. a periodic Hann window applied,
 *   3. the power of DFT bins 1..32 (1.5625 Hz each), summed in seven bands,
 *   4. log10(band power + VIB_POWER_FLOOR).
 *
 * This is the C version of ml/features.py, which the classifier was trained
 * with; tests/unit/test_vib_features.c checks the two agree.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef VIB_FEATURES_H
#define VIB_FEATURES_H

#include <stdbool.h>
#include <stdint.h>

#define VIB_WINDOW          (64U)
#define VIB_HOP             (32U)
#define VIB_AXES            (6U)
#define VIB_BANDS           (7U)
#define VIB_FEATURES        (VIB_AXES * VIB_BANDS)
#define VIB_POWER_FLOOR     (1e-6f)

/** One window of samples, oldest first: accel x, y, z in g, then gyro x, y, z in deg/s. */
typedef float vib_samples_t[VIB_WINDOW][VIB_AXES];

/** Sliding window over the IMU samples. */
typedef struct
{
    vib_samples_t ring;
    uint32_t      head;         /**< Where the next sample goes. */
    uint32_t      filled;       /**< Samples stored, up to VIB_WINDOW. */
    uint32_t      since_window; /**< Samples since the last complete window. */
} vib_window_t;

#ifdef __cplusplus
extern "C" {
#endif

void vib_window_init(vib_window_t *w);

/**
 * Adds one IMU sample (the driver's units: mg and mdps). Returns true when a
 * new window is complete: first after VIB_WINDOW samples, then every VIB_HOP.
 */
bool vib_window_add(vib_window_t *w, const int32_t accel_mg[3], const int32_t gyro_mdps[3]);

/** Copies the latest VIB_WINDOW samples, oldest first. */
void vib_window_copy(const vib_window_t *w, vib_samples_t out);

/** Computes the VIB_FEATURES features of a window, axis by axis, band by band. */
void vib_features(const vib_samples_t window, float features[VIB_FEATURES]);

#ifdef __cplusplus
}
#endif

#endif /* VIB_FEATURES_H */
