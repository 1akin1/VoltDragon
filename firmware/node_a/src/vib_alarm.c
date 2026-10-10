/**
 * @file vib_alarm.c
 * @brief Debouncing of the vibration classifier's output.
 */
#include "vib_alarm.h"

#include <string.h>

void vib_alarm_init(vib_alarm_t *a)
{
    (void)memset(a, 0, sizeof(*a));
    a->alarm = VIB_CLASS_NOMINAL;
}

/** The most frequent fault class in the history; ties go to the latest. */
static vib_class_t dominant_fault(const vib_alarm_t *a)
{
    uint32_t counts[VIB_CLASS_COUNT] = { 0U };
    vib_class_t best = VIB_CLASS_NOMINAL;
    uint32_t best_count = 0U;

    for (uint32_t i = 0U; i < a->stored; ++i)
    {
        counts[a->history[i]]++;
    }
    /* Newest first, so that on a tie the latest class is kept. */
    for (uint32_t age = 0U; age < a->stored; ++age)
    {
        const uint32_t index = (a->next + VIB_ALARM_HISTORY - 1U - age) % VIB_ALARM_HISTORY;
        const vib_class_t c = a->history[index];
        if ((c != VIB_CLASS_NOMINAL) && (counts[c] > best_count))
        {
            best = c;
            best_count = counts[c];
        }
    }
    return best;
}

vib_class_t vib_alarm_update(vib_alarm_t *a, vib_class_t predicted)
{
    a->history[a->next] = predicted;
    a->next = (a->next + 1U) % VIB_ALARM_HISTORY;
    if (a->stored < VIB_ALARM_HISTORY)
    {
        a->stored++;
    }

    if (predicted == VIB_CLASS_NOMINAL)
    {
        a->nominal_run++;
        if (a->nominal_run >= VIB_ALARM_CLEAR_WINDOWS)
        {
            a->alarm = VIB_CLASS_NOMINAL;
        }
        return a->alarm;
    }

    a->nominal_run = 0U;
    uint32_t faults = 0U;
    for (uint32_t i = 0U; i < a->stored; ++i)
    {
        if (a->history[i] != VIB_CLASS_NOMINAL)
        {
            faults++;
        }
    }
    if (faults >= VIB_ALARM_FAULT_WINDOWS)
    {
        a->alarm = dominant_fault(a);
    }
    return a->alarm;
}

const char *vib_class_name(vib_class_t c)
{
    static const char *const names[VIB_CLASS_COUNT] = { "nominal", "imbalance", "bearing" };

    return ((uint32_t)c < (uint32_t)VIB_CLASS_COUNT) ? names[c] : "?";
}
