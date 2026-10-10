/**
 * @file test_vib_model.cc
 * @brief Host unit test: the INT8 classifier on TensorFlow Lite Micro against the Python reference.
 *
 * The vectors (vib_vectors.h) hold the quantised input and the INT8 output that
 * ml/train.py got from the TFLite interpreter's reference kernels for the same
 * model. TFLite Micro runs the same reference arithmetic, so the outputs must
 * match exactly; the input may differ by one step where single-precision
 * features fall on the other side of a rounding boundary.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "unit.h"
#include "vib_features.h"
#include "vib_model.h"
#include "vib_vectors.h"

UNIT_MAIN_DEFINITIONS;

/* The firmware's log goes to stdout here. */
extern "C" void log_line(char level, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    printf("  [%c] ", level);
    (void)vprintf(fmt, args);
    printf("\n");
    va_end(args);
}

static vib_class_t predicted(const int8_t output[VIB_CLASS_COUNT])
{
    uint32_t best = 0U;

    for (uint32_t c = 1U; c < (uint32_t)VIB_CLASS_COUNT; ++c)
    {
        if (output[c] > output[best])
        {
            best = c;
        }
    }
    return (vib_class_t)best;
}

static void model_loads_into_its_arena(void)
{
    CHECK(vib_model_init());
    CHECK(vib_model_arena_used() > 0U);
    printf("  arena used: %u bytes\n", (unsigned int)vib_model_arena_used());
}

static void quantised_input_matches(void)
{
    for (uint32_t v = 0U; v < VIB_VECTOR_COUNT; ++v)
    {
        int8_t input[VIB_FEATURES];

        vib_model_quantise(VIB_VECTORS[v].features, input);
        for (uint32_t f = 0U; f < VIB_FEATURES; ++f)
        {
            CHECK(abs((int)input[f] - (int)VIB_VECTORS[v].input[f]) <= 1);
        }
    }
}

static void output_matches_the_reference_exactly(void)
{
    for (uint32_t v = 0U; v < VIB_VECTOR_COUNT; ++v)
    {
        int8_t output[VIB_CLASS_COUNT];

        CHECK(vib_model_run(VIB_VECTORS[v].input, output));
        for (uint32_t c = 0U; c < (uint32_t)VIB_CLASS_COUNT; ++c)
        {
            CHECK_EQ(output[c], VIB_VECTORS[v].output[c]);
        }
    }
}

static void raw_samples_are_classified_correctly(void)
{
    static vib_window_t window;
    static vib_samples_t samples;

    for (uint32_t v = 0U; v < VIB_VECTOR_COUNT; ++v)
    {
        const vib_vector_t *vector = &VIB_VECTORS[v];
        float features[VIB_FEATURES];
        int8_t input[VIB_FEATURES];
        int8_t output[VIB_CLASS_COUNT];

        vib_window_init(&window);
        for (uint32_t n = 0U; n < VIB_WINDOW; ++n)
        {
            (void)vib_window_add(&window, &vector->accel_mg[3U * n], &vector->gyro_mdps[3U * n]);
        }
        vib_window_copy(&window, samples);
        vib_features(samples, features);
        vib_model_quantise(features, input);
        CHECK(vib_model_run(input, output));
        CHECK_EQ(predicted(output), vector->label);
        CHECK(vib_model_probability(output[vector->label]) > 0.9f);
    }
}

int main(void)
{
    RUN(model_loads_into_its_arena);
    RUN(quantised_input_matches);
    RUN(output_matches_the_reference_exactly);
    RUN(raw_samples_are_classified_correctly);
    return (unit_failures == 0) ? 0 : 1;
}
