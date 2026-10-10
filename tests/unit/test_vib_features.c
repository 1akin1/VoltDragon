/**
 * @file test_vib_features.c
 * @brief Host unit test for Node A's vibration window, features and alarm debouncing.
 *
 * The features are checked against vectors that ml/train.py computed with the
 * Python reference (ml/features.py) from test-set windows (vib_vectors.h).
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "unit.h"
#include "vib_alarm.h"
#include "vib_features.h"
#include "vib_vectors.h"

UNIT_MAIN_DEFINITIONS;

/* The Python reference works in double precision; single precision differs slightly. */
#define FEATURE_TOLERANCE (2e-3f)

static vib_window_t s_window;
static vib_samples_t s_samples;

static void feed(const vib_vector_t *v, uint32_t count, uint32_t *completed)
{
    for (uint32_t n = 0U; n < count; ++n)
    {
        const uint32_t i = (n % VIB_WINDOW) * 3U;
        if (vib_window_add(&s_window, &v->accel_mg[i], &v->gyro_mdps[i]))
        {
            (*completed)++;
        }
    }
}

static void features_match_the_python_reference(void)
{
    for (uint32_t v = 0U; v < VIB_VECTOR_COUNT; ++v)
    {
        const vib_vector_t *vector = &VIB_VECTORS[v];
        float features[VIB_FEATURES];
        uint32_t completed = 0U;

        vib_window_init(&s_window);
        feed(vector, VIB_WINDOW, &completed);
        CHECK_EQ(completed, 1U);
        vib_window_copy(&s_window, s_samples);
        vib_features(s_samples, features);
        for (uint32_t f = 0U; f < VIB_FEATURES; ++f)
        {
            if (fabsf(features[f] - vector->features[f]) > FEATURE_TOLERANCE)
            {
                printf("  vector %u feature %u: %f, expected %f\n", (unsigned int)v,
                       (unsigned int)f, (double)features[f], (double)vector->features[f]);
            }
            CHECK(fabsf(features[f] - vector->features[f]) <= FEATURE_TOLERANCE);
        }
    }
}

static void windows_complete_every_hop_after_the_first(void)
{
    uint32_t completed = 0U;

    vib_window_init(&s_window);
    feed(&VIB_VECTORS[0], VIB_WINDOW - 1U, &completed);
    CHECK_EQ(completed, 0U);
    feed(&VIB_VECTORS[0], 1U, &completed);
    CHECK_EQ(completed, 1U);
    feed(&VIB_VECTORS[0], VIB_HOP - 1U, &completed);
    CHECK_EQ(completed, 1U);
    feed(&VIB_VECTORS[0], 1U, &completed);
    CHECK_EQ(completed, 2U);
    feed(&VIB_VECTORS[0], 10U * VIB_HOP, &completed);
    CHECK_EQ(completed, 12U);
}

static void window_copy_is_oldest_first(void)
{
    int32_t accel[3] = { 0, 0, 0 };
    const int32_t gyro[3] = { 0, 0, 0 };

    vib_window_init(&s_window);
    for (int32_t n = 0; n < (int32_t)(VIB_WINDOW + 5U); ++n)
    {
        accel[0] = n * 1000;            /* 1 g per sample index */
        (void)vib_window_add(&s_window, accel, gyro);
    }
    vib_window_copy(&s_window, s_samples);
    CHECK(fabsf(s_samples[0][0] - 5.0f) < 1e-6f);
    CHECK(fabsf(s_samples[VIB_WINDOW - 1U][0] - (float)(VIB_WINDOW + 4U)) < 1e-6f);
}

static void a_constant_signal_gives_the_floor(void)
{
    float features[VIB_FEATURES];

    for (uint32_t n = 0U; n < VIB_WINDOW; ++n)
    {
        for (uint32_t axis = 0U; axis < VIB_AXES; ++axis)
        {
            s_samples[n][axis] = (float)axis - 2.5f;
        }
    }
    vib_features(s_samples, features);
    for (uint32_t f = 0U; f < VIB_FEATURES; ++f)
    {
        CHECK(fabsf(features[f] - log10f(VIB_POWER_FLOOR)) < 1e-3f);
    }
}

static vib_class_t run(vib_alarm_t *a, const vib_class_t *classes, uint32_t count)
{
    vib_class_t alarm = VIB_CLASS_NOMINAL;

    for (uint32_t i = 0U; i < count; ++i)
    {
        alarm = vib_alarm_update(a, classes[i]);
    }
    return alarm;
}

static void alarm_needs_three_of_four_fault_windows(void)
{
    static const vib_class_t sparse[] = { VIB_CLASS_IMBALANCE, VIB_CLASS_NOMINAL,
                                          VIB_CLASS_IMBALANCE, VIB_CLASS_NOMINAL,
                                          VIB_CLASS_IMBALANCE };
    static const vib_class_t dense[] = { VIB_CLASS_IMBALANCE, VIB_CLASS_NOMINAL,
                                         VIB_CLASS_IMBALANCE, VIB_CLASS_IMBALANCE };
    vib_alarm_t a;

    vib_alarm_init(&a);
    CHECK_EQ(run(&a, sparse, 5U), VIB_CLASS_NOMINAL);
    vib_alarm_init(&a);
    CHECK_EQ(run(&a, dense, 3U), VIB_CLASS_NOMINAL);
    CHECK_EQ(run(&a, &dense[3], 1U), VIB_CLASS_IMBALANCE);
}

static void alarm_takes_the_dominant_fault_and_clears_after_eight_nominal(void)
{
    static const vib_class_t mixed[] = { VIB_CLASS_BEARING, VIB_CLASS_IMBALANCE,
                                         VIB_CLASS_BEARING };
    static const vib_class_t tie[] = { VIB_CLASS_IMBALANCE, VIB_CLASS_BEARING,
                                       VIB_CLASS_IMBALANCE, VIB_CLASS_BEARING };
    vib_alarm_t a;

    vib_alarm_init(&a);
    CHECK_EQ(run(&a, mixed, 3U), VIB_CLASS_BEARING);
    for (uint32_t i = 0U; i < (VIB_ALARM_CLEAR_WINDOWS - 1U); ++i)
    {
        CHECK_EQ(vib_alarm_update(&a, VIB_CLASS_NOMINAL), VIB_CLASS_BEARING);
    }
    /* A fault window restarts the count; one alone does not change the alarm. */
    CHECK_EQ(vib_alarm_update(&a, VIB_CLASS_IMBALANCE), VIB_CLASS_BEARING);
    for (uint32_t i = 0U; i < (VIB_ALARM_CLEAR_WINDOWS - 1U); ++i)
    {
        CHECK_EQ(vib_alarm_update(&a, VIB_CLASS_NOMINAL), VIB_CLASS_BEARING);
    }
    CHECK_EQ(vib_alarm_update(&a, VIB_CLASS_NOMINAL), VIB_CLASS_NOMINAL);

    /* Two of each: the latest wins. */
    vib_alarm_init(&a);
    CHECK_EQ(run(&a, tie, 4U), VIB_CLASS_BEARING);
}

static void class_names(void)
{
    CHECK_STR(vib_class_name(VIB_CLASS_NOMINAL), "nominal");
    CHECK_STR(vib_class_name(VIB_CLASS_IMBALANCE), "imbalance");
    CHECK_STR(vib_class_name(VIB_CLASS_BEARING), "bearing");
    CHECK_STR(vib_class_name(VIB_CLASS_COUNT), "?");
}

int main(void)
{
    RUN(features_match_the_python_reference);
    RUN(windows_complete_every_hop_after_the_first);
    RUN(window_copy_is_oldest_first);
    RUN(a_constant_signal_gives_the_floor);
    RUN(alarm_needs_three_of_four_fault_windows);
    RUN(alarm_takes_the_dominant_fault_and_clears_after_eight_nominal);
    RUN(class_names);
    return (unit_failures == 0) ? 0 : 1;
}
