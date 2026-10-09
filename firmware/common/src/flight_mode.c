/**
 * @file flight_mode.c
 * @brief Flight-mode state machine: the allowed transitions.
 */
#include "flight_mode.h"

#include <stdint.h>
#include <string.h>

typedef enum
{
    NO = 0,         /* never */
    OPERATOR,       /* operator request only */
    ANY,            /* operator request or automatic trigger */
    OVERRIDE        /* operator request with the override flag */
} rule_t;

static const rule_t s_rules[FLIGHT_MODE_COUNT][FLIGHT_MODE_COUNT] = {
    /*                     to MISSION  HOLD      RTH  LAND */
    /* from MISSION */     { NO,       OPERATOR, ANY, ANY },
    /* from HOLD */        { OPERATOR, NO,       ANY, ANY },
    /* from RTH */         { OVERRIDE, OVERRIDE, NO,  ANY },
    /* from LAND */        { NO,       NO,       NO,  NO  },
};

static const char *const s_mode_names[FLIGHT_MODE_COUNT] = {
    "MISSION", "HOLD", "RETURN_TO_HOME", "LAND"
};

flight_change_t flight_mode_check(flight_mode_t current, flight_mode_t requested,
                                  flight_cause_t cause, bool override)
{
    if ((current >= FLIGHT_MODE_COUNT) || (requested >= FLIGHT_MODE_COUNT) ||
        (cause >= FLIGHT_CAUSE_COUNT))
    {
        return FLIGHT_CHANGE_UNDEFINED;
    }
    if (current == requested)
    {
        return FLIGHT_CHANGE_UNCHANGED;
    }

    const bool by_operator = cause == FLIGHT_CAUSE_OPERATOR;
    switch (s_rules[current][requested])
    {
        case ANY:
            return FLIGHT_CHANGE_OK;
        case OPERATOR:
            return by_operator ? FLIGHT_CHANGE_OK : FLIGHT_CHANGE_UNDEFINED;
        case OVERRIDE:
            if (!by_operator)
            {
                return FLIGHT_CHANGE_UNDEFINED;
            }
            return override ? FLIGHT_CHANGE_OK : FLIGHT_CHANGE_NEEDS_OVERRIDE;
        default:
            return FLIGHT_CHANGE_UNDEFINED;
    }
}

bool flight_mode_is_safety(flight_mode_t mode)
{
    return (mode == FLIGHT_MODE_RETURN_TO_HOME) || (mode == FLIGHT_MODE_LAND);
}

const char *flight_mode_name(flight_mode_t mode)
{
    return (mode < FLIGHT_MODE_COUNT) ? s_mode_names[mode] : "?";
}

bool flight_mode_from_name(const char *name, flight_mode_t *mode)
{
    for (uint32_t i = 0U; i < (uint32_t)FLIGHT_MODE_COUNT; ++i)
    {
        if (strcmp(name, s_mode_names[i]) == 0)
        {
            *mode = (flight_mode_t)i;
            return true;
        }
    }
    if (strcmp(name, "RTH") == 0)
    {
        *mode = FLIGHT_MODE_RETURN_TO_HOME;
        return true;
    }
    return false;
}

const char *flight_cause_name(flight_cause_t cause)
{
    switch (cause)
    {
        case FLIGHT_CAUSE_OPERATOR:
            return "operator";
        case FLIGHT_CAUSE_LINK_LOSS:
            return "ground link lost";
        case FLIGHT_CAUSE_BATTERY_LOW:
            return "battery low";
        case FLIGHT_CAUSE_BATTERY_CRITICAL:
            return "battery critical";
        default:
            return "?";
    }
}

const char *flight_change_name(flight_change_t change)
{
    switch (change)
    {
        case FLIGHT_CHANGE_OK:
            return "OK";
        case FLIGHT_CHANGE_UNCHANGED:
            return "UNCHANGED";
        case FLIGHT_CHANGE_NEEDS_OVERRIDE:
            return "NEEDS_OVERRIDE";
        case FLIGHT_CHANGE_UNDEFINED:
            return "UNDEFINED";
        default:
            return "?";
    }
}
