/**
 * @file health.h
 * @brief Node A's vibration health monitor: on-board fault classification (HLR-009).
 *
 * ImuTask hands every IMU sample to health_on_sample(), which keeps a sliding
 * window (vib_features.h). Every 0.32 s a window is complete: it is copied for
 * AiTask, which ImuTask then wakes. AiTask runs health_classify(): the
 * features, the INT8 classifier on TensorFlow Lite Micro (vib_model.h) and the
 * alarm debouncing (vib_alarm.h). Alarm changes are logged and recorded in
 * the flight-data recorder; the state goes to Node B in the CAN HEALTH message
 * and the alarm as a SAFETY flag.
 *
 * The classifier is advisory: it raises an alarm for the operator and takes
 * no flight action, and it runs at the lowest priority of the periodic work.
 * A monitor that stops classifying (the model failed to load, AiTask starved)
 * is reported as inactive rather than resetting the node through the watchdog.
 */
#ifndef HEALTH_H
#define HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#include "lsm9ds1.h"
#include "vib_alarm.h"

/** The monitor is reported inactive if no window was classified for this long. */
#define HEALTH_STALE_MS (1000UL)

typedef struct
{
    bool        active;             /**< The model is loaded and classified a window recently. */
    vib_class_t alarm;              /**< Debounced alarm; VIB_CLASS_NOMINAL when none. */
    vib_class_t last_class;         /**< The latest window's class. */
    uint8_t     confidence_pct;     /**< The latest window's probability of last_class. */
    uint8_t     fault_score_pct;    /**< 100 minus the latest window's probability of nominal. */
    uint32_t    windows;            /**< Windows classified since start-up. */
    uint32_t    fault_windows;      /**< ... of which classified as a fault. */
    uint32_t    inference_us;       /**< Latest features + inference time. */
    uint32_t    max_inference_us;
} health_state_t;

/** Loads the model. Returns false if the monitor cannot run; samples are then ignored. */
bool health_init(void);

/**
 * Adds one valid IMU sample. ImuTask, every 10 ms. Returns true when a window
 * is complete and waiting: the caller wakes AiTask.
 */
bool health_on_sample(const lsm9ds1_sample_t *sample);

/** A sample is missing (failed read): the window restarts, so it never spans a gap. */
void health_on_sample_lost(void);

/** Classifies the waiting window, if any. AiTask, when woken. */
void health_classify(uint32_t now_ms);

/** Snapshot of the state; any task. */
health_state_t health_state(uint32_t now_ms);

/** Logs the alarm, the latest class and the inference time. */
void health_report(uint32_t now_ms);

#endif /* HEALTH_H */
