/**
 * @file vib_features.c
 * @brief Vibration analysis windows and spectral features.
 */
#include "vib_features.h"

#include <math.h>
#include <string.h>

#define UNITS_PER_KILO  (1000.0f)
#define FIRST_BIN       (1U)
#define LAST_BIN        (VIB_WINDOW / 2U)
#define QUARTER_TURN    (VIB_WINDOW / 4U)

/* cos(2 pi i / 64): the DFT twiddle factors and the Hann window. */
static const float COS64[VIB_WINDOW] = {
     1.000000000f,  0.995184727f,  0.980785280f,  0.956940336f,
     0.923879533f,  0.881921264f,  0.831469612f,  0.773010453f,
     0.707106781f,  0.634393284f,  0.555570233f,  0.471396737f,
     0.382683432f,  0.290284677f,  0.195090322f,  0.098017140f,
     0.000000000f, -0.098017140f, -0.195090322f, -0.290284677f,
    -0.382683432f, -0.471396737f, -0.555570233f, -0.634393284f,
    -0.707106781f, -0.773010453f, -0.831469612f, -0.881921264f,
    -0.923879533f, -0.956940336f, -0.980785280f, -0.995184727f,
    -1.000000000f, -0.995184727f, -0.980785280f, -0.956940336f,
    -0.923879533f, -0.881921264f, -0.831469612f, -0.773010453f,
    -0.707106781f, -0.634393284f, -0.555570233f, -0.471396737f,
    -0.382683432f, -0.290284677f, -0.195090322f, -0.098017140f,
     0.000000000f,  0.098017140f,  0.195090322f,  0.290284677f,
     0.382683432f,  0.471396737f,  0.555570233f,  0.634393284f,
     0.707106781f,  0.773010453f,  0.831469612f,  0.881921264f,
     0.923879533f,  0.956940336f,  0.980785280f,  0.995184727f,
};

/* Inclusive DFT bin ranges of the bands (ml/features.py: BANDS). */
static const uint8_t BAND_FIRST[VIB_BANDS] = { 1U, 3U, 6U, 10U, 15U, 21U, 27U };
static const uint8_t BAND_LAST[VIB_BANDS]  = { 2U, 5U, 9U, 14U, 20U, 26U, 32U };

void vib_window_init(vib_window_t *w)
{
    (void)memset(w, 0, sizeof(*w));
}

bool vib_window_add(vib_window_t *w, const int32_t accel_mg[3], const int32_t gyro_mdps[3])
{
    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        w->ring[w->head][axis] = (float)accel_mg[axis] / UNITS_PER_KILO;
        w->ring[w->head][axis + 3U] = (float)gyro_mdps[axis] / UNITS_PER_KILO;
    }
    w->head = (w->head + 1U) % VIB_WINDOW;
    if (w->filled < VIB_WINDOW)
    {
        w->filled++;
        if (w->filled == VIB_WINDOW)
        {
            w->since_window = 0U;
            return true;
        }
        return false;
    }
    w->since_window++;
    if (w->since_window == VIB_HOP)
    {
        w->since_window = 0U;
        return true;
    }
    return false;
}

void vib_window_copy(const vib_window_t *w, vib_samples_t out)
{
    /* Once full, head is the oldest sample. */
    for (uint32_t i = 0U; i < VIB_WINDOW; ++i)
    {
        (void)memcpy(out[i], w->ring[(w->head + i) % VIB_WINDOW], sizeof(out[i]));
    }
}

/** Power of DFT bins FIRST_BIN..LAST_BIN of a windowed signal, divided by VIB_WINDOW. */
static void power_spectrum(const float x[VIB_WINDOW], float power[LAST_BIN + 1U])
{
    for (uint32_t k = FIRST_BIN; k <= LAST_BIN; ++k)
    {
        float re = 0.0f;
        float im = 0.0f;
        for (uint32_t n = 0U; n < VIB_WINDOW; ++n)
        {
            const uint32_t m = (k * n) % VIB_WINDOW;
            /* sin(a) = cos(a - pi/2): three quarters of a turn on. */
            re += x[n] * COS64[m];
            im -= x[n] * COS64[(m + (3U * QUARTER_TURN)) % VIB_WINDOW];
        }
        power[k] = ((re * re) + (im * im)) / (float)VIB_WINDOW;
    }
}

void vib_features(const vib_samples_t window, float features[VIB_FEATURES])
{
    float x[VIB_WINDOW];
    float power[LAST_BIN + 1U];

    for (uint32_t axis = 0U; axis < VIB_AXES; ++axis)
    {
        float mean = 0.0f;
        for (uint32_t n = 0U; n < VIB_WINDOW; ++n)
        {
            mean += window[n][axis];
        }
        mean /= (float)VIB_WINDOW;
        for (uint32_t n = 0U; n < VIB_WINDOW; ++n)
        {
            const float hann = 0.5f - (0.5f * COS64[n]);
            x[n] = (window[n][axis] - mean) * hann;
        }
        power_spectrum(x, power);
        for (uint32_t band = 0U; band < VIB_BANDS; ++band)
        {
            float sum = 0.0f;
            for (uint32_t k = BAND_FIRST[band]; k <= BAND_LAST[band]; ++k)
            {
                sum += power[k];
            }
            features[(axis * VIB_BANDS) + band] = log10f(sum + VIB_POWER_FLOOR);
        }
    }
}
