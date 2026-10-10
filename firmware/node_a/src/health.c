/**
 * @file health.c
 * @brief Node A's vibration health monitor.
 */
#include "health.h"

#include <string.h>

#include "flashlog.h"
#include "lock.h"
#include "log.h"
#include "systick.h"
#include "vib_features.h"
#include "vib_model.h"

#define PERCENT (100.0f)

typedef struct
{
    bool        ready;
    vib_alarm_t debounce;
    vib_class_t alarm;
    vib_class_t last_class;
    uint8_t     confidence_pct;
    uint8_t     fault_score_pct;
    uint32_t    windows;
    uint32_t    fault_windows;
    uint32_t    last_window_ms;
    uint32_t    inference_us;
    uint32_t    max_inference_us;
    uint32_t    model_errors;
} health_status_t;

/* ImuTask only. */
static vib_window_t s_window;

/* Handed from ImuTask to AiTask under s_lock. */
static vib_samples_t s_pending;
static bool s_pending_ready;
static uint32_t s_overruns;     /* windows replaced before AiTask took them */

/* AiTask only. */
static vib_samples_t s_work;

/* Written by AiTask under s_lock; read by any task. */
static health_status_t s_status;
static lock_t s_lock;

bool health_init(void)
{
    (void)memset(&s_status, 0, sizeof(s_status));
    lock_init(&s_lock);
    vib_window_init(&s_window);
    vib_alarm_init(&s_status.debounce);
    s_pending_ready = false;
    s_overruns = 0U;

    if (!vib_model_init())
    {
        LOG_ERROR("ai: vibration monitor disabled");
        return false;
    }
    s_status.ready = true;
    LOG_INFO("ai: vibration classifier ready, tensor arena %lu bytes; a window every %lu ms",
             vib_model_arena_used(), (uint32_t)(VIB_HOP * 10U));
    return true;
}

bool health_on_sample(const lsm9ds1_sample_t *sample)
{
    if (!s_status.ready || !vib_window_add(&s_window, sample->accel_mg, sample->gyro_mdps))
    {
        return false;
    }
    lock_take(&s_lock);
    if (s_pending_ready)
    {
        s_overruns++;
    }
    vib_window_copy(&s_window, s_pending);
    s_pending_ready = true;
    lock_give(&s_lock);
    return true;
}

void health_on_sample_lost(void)
{
    vib_window_init(&s_window);
}

/** Records and logs a change of the alarm. */
static void alarm_changed(vib_class_t alarm, uint8_t confidence_pct, uint32_t windows)
{
    const flashlog_vib_alarm_t record = {
        .alarm = (uint8_t)alarm,
        .confidence_pct = confidence_pct,
        .reserved = 0U,
        .windows = windows,
    };

    (void)flashlog_append(FLASHLOG_TYPE_VIB_ALARM, &record, sizeof(record));
    if (alarm == VIB_CLASS_NOMINAL)
    {
        LOG_INFO("ai: vibration alarm cleared");
    }
    else
    {
        LOG_WARN("ai: VIBRATION FAULT: %s (%u %%)", vib_class_name(alarm),
                 (unsigned int)confidence_pct);
    }
}

void health_classify(uint32_t now_ms)
{
    float features[VIB_FEATURES];
    int8_t input[VIB_FEATURES];
    int8_t output[VIB_CLASS_COUNT];

    lock_take(&s_lock);
    const bool ready = s_pending_ready;
    if (ready)
    {
        (void)memcpy(s_work, s_pending, sizeof(s_work));
        s_pending_ready = false;
    }
    lock_give(&s_lock);
    if (!ready || !s_status.ready)
    {
        return;
    }

    const uint32_t start_us = systick_now_us();
    vib_features(s_work, features);
    vib_model_quantise(features, input);
    const bool ok = vib_model_run(input, output);
    const uint32_t elapsed_us = systick_now_us() - start_us;

    if (!ok)
    {
        lock_take(&s_lock);
        s_status.model_errors++;
        lock_give(&s_lock);
        return;
    }

    vib_class_t best = VIB_CLASS_NOMINAL;
    for (uint32_t c = 1U; c < (uint32_t)VIB_CLASS_COUNT; ++c)
    {
        if (output[c] > output[best])
        {
            best = (vib_class_t)c;
        }
    }
    const uint8_t confidence = (uint8_t)((vib_model_probability(output[best]) * PERCENT) + 0.5f);
    const float nominal = vib_model_probability(output[VIB_CLASS_NOMINAL]);
    const uint8_t fault_score = (uint8_t)(((1.0f - nominal) * PERCENT) + 0.5f);

    /* The debounce state is AiTask's own; only the published fields need the lock. */
    const vib_class_t previous = s_status.alarm;
    const vib_class_t alarm = vib_alarm_update(&s_status.debounce, best);

    lock_take(&s_lock);
    s_status.alarm = alarm;
    s_status.last_class = best;
    s_status.confidence_pct = confidence;
    s_status.fault_score_pct = fault_score;
    s_status.windows++;
    if (best != VIB_CLASS_NOMINAL)
    {
        s_status.fault_windows++;
    }
    s_status.last_window_ms = now_ms;
    s_status.inference_us = elapsed_us;
    if (elapsed_us > s_status.max_inference_us)
    {
        s_status.max_inference_us = elapsed_us;
    }
    const uint32_t windows = s_status.windows;
    lock_give(&s_lock);

    if (alarm != previous)
    {
        alarm_changed(alarm, confidence, windows);
    }
}

health_state_t health_state(uint32_t now_ms)
{
    lock_take(&s_lock);
    const health_status_t s = s_status;
    lock_give(&s_lock);

    const health_state_t state = {
        .active = s.ready && (s.windows > 0U) && ((now_ms - s.last_window_ms) < HEALTH_STALE_MS),
        .alarm = s.alarm,
        .last_class = s.last_class,
        .confidence_pct = s.confidence_pct,
        .fault_score_pct = s.fault_score_pct,
        .windows = s.windows,
        .fault_windows = s.fault_windows,
        .inference_us = s.inference_us,
        .max_inference_us = s.max_inference_us,
    };
    return state;
}

void health_report(uint32_t now_ms)
{
    if (!s_status.ready)
    {
        return;
    }
    const health_state_t s = health_state(now_ms);

    lock_take(&s_lock);
    const uint32_t overruns = s_overruns;
    const uint32_t errors = s_status.model_errors;
    lock_give(&s_lock);

    LOG_INFO("ai: %s, alarm %s, last %s %u %%, windows %lu (fault %lu), inference %lu us "
             "(max %lu), overruns %lu, errors %lu",
             s.active ? "active" : "INACTIVE", vib_class_name(s.alarm),
             vib_class_name(s.last_class), (unsigned int)s.confidence_pct, s.windows,
             s.fault_windows, s.inference_us, s.max_inference_us, overruns, errors);
}
