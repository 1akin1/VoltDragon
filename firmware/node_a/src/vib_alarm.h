/**
 * @file vib_alarm.h
 * @brief Debounces the vibration classifier's per-window output into an alarm (HLR-009).
 *
 * A fault alarm is raised when VIB_ALARM_FAULT_WINDOWS of the last
 * VIB_ALARM_HISTORY windows were classified as a fault; its class is the most
 * frequent fault class among them (ties go to the latest). It clears after
 * VIB_ALARM_CLEAR_WINDOWS nominal windows in a row. With a window every
 * 0.32 s, a fault that every window shows raises the alarm 0.64 s after the
 * first window that shows it.
 *
 * The C version of ml/features.py: AlarmFilter. No hardware dependencies;
 * unit-tested on the host (tests/unit).
 */
#ifndef VIB_ALARM_H
#define VIB_ALARM_H

#include <stdint.h>

#define VIB_ALARM_HISTORY       (4U)
#define VIB_ALARM_FAULT_WINDOWS (3U)
#define VIB_ALARM_CLEAR_WINDOWS (8U)

/** Classifier outputs, in the model's output order. */
typedef enum
{
    VIB_CLASS_NOMINAL = 0,
    VIB_CLASS_IMBALANCE,        /**< Damaged propeller. */
    VIB_CLASS_BEARING,          /**< Worn motor bearing. */
    VIB_CLASS_COUNT
} vib_class_t;

typedef struct
{
    vib_class_t history[VIB_ALARM_HISTORY];
    uint32_t    next;           /**< Where the next class goes in history. */
    uint32_t    stored;         /**< Classes in history, up to VIB_ALARM_HISTORY. */
    uint32_t    nominal_run;
    vib_class_t alarm;          /**< VIB_CLASS_NOMINAL: no alarm. */
} vib_alarm_t;

#ifdef __cplusplus
extern "C" {
#endif

void vib_alarm_init(vib_alarm_t *a);

/** Adds one window's class and returns the alarm (VIB_CLASS_NOMINAL when none). */
vib_class_t vib_alarm_update(vib_alarm_t *a, vib_class_t predicted);

/** Short name of a class: "nominal", "imbalance", "bearing". */
const char *vib_class_name(vib_class_t c);

#ifdef __cplusplus
}
#endif

#endif /* VIB_ALARM_H */
