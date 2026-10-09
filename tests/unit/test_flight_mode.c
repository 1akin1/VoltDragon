/**
 * @file test_flight_mode.c
 * @brief Host unit tests for flight_mode.c: every transition, cause and override combination.
 */
#include <stdint.h>
#include <string.h>

#include "flight_mode.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

#define M FLIGHT_MODE_MISSION
#define H FLIGHT_MODE_HOLD
#define R FLIGHT_MODE_RETURN_TO_HOME
#define L FLIGHT_MODE_LAND

/* The rules of HLR-003..006, written out independently of the implementation's table. */
static flight_change_t expected(flight_mode_t from, flight_mode_t to, flight_cause_t cause,
                                bool override)
{
    const bool operator_request = cause == FLIGHT_CAUSE_OPERATOR;

    if (from == to)
    {
        return FLIGHT_CHANGE_UNCHANGED;
    }
    if (from == L)
    {
        return FLIGHT_CHANGE_UNDEFINED;                 /* landing is final */
    }
    if ((to == R) || (to == L))
    {
        return FLIGHT_CHANGE_OK;                        /* towards safety: always allowed */
    }
    if (!operator_request)
    {
        return FLIGHT_CHANGE_UNDEFINED;                 /* automation never leaves safety */
    }
    if (from == R)
    {
        return override ? FLIGHT_CHANGE_OK : FLIGHT_CHANGE_NEEDS_OVERRIDE;  /* HLR-006 */
    }
    return FLIGHT_CHANGE_OK;                            /* MISSION <-> HOLD by the operator */
}

static void every_combination_follows_the_rules(void)
{
    uint32_t checked = 0U;

    for (int from = 0; from < FLIGHT_MODE_COUNT; ++from)
    {
        for (int to = 0; to < FLIGHT_MODE_COUNT; ++to)
        {
            for (int cause = 0; cause < FLIGHT_CAUSE_COUNT; ++cause)
            {
                for (int override = 0; override < 2; ++override)
                {
                    const flight_change_t want = expected((flight_mode_t)from, (flight_mode_t)to,
                                                          (flight_cause_t)cause, override != 0);
                    const flight_change_t got = flight_mode_check(
                        (flight_mode_t)from, (flight_mode_t)to, (flight_cause_t)cause,
                        override != 0);
                    if (got != want)
                    {
                        printf("  %s -> %s by %s%s: got %s, expected %s\n",
                               flight_mode_name((flight_mode_t)from),
                               flight_mode_name((flight_mode_t)to),
                               flight_cause_name((flight_cause_t)cause),
                               override ? " (override)" : "", flight_change_name(got),
                               flight_change_name(want));
                    }
                    CHECK_EQ(got, want);
                    checked++;
                }
            }
        }
    }
    CHECK_EQ(checked, 4 * 4 * 4 * 2);
}

static void named_examples(void)
{
    /* HLR-003: link loss sends a mission to return-to-home. */
    CHECK_EQ(flight_mode_check(M, R, FLIGHT_CAUSE_LINK_LOSS, false), FLIGHT_CHANGE_OK);
    /* HLR-004: a critical battery lands, even from return-to-home. */
    CHECK_EQ(flight_mode_check(R, L, FLIGHT_CAUSE_BATTERY_CRITICAL, false), FLIGHT_CHANGE_OK);
    /* HLR-006: the operator cannot resume the mission without the override. */
    CHECK_EQ(flight_mode_check(R, M, FLIGHT_CAUSE_OPERATOR, false), FLIGHT_CHANGE_NEEDS_OVERRIDE);
    CHECK_EQ(flight_mode_check(R, M, FLIGHT_CAUSE_OPERATOR, true), FLIGHT_CHANGE_OK);
    /* Automation never resumes a mission, override or not. */
    CHECK_EQ(flight_mode_check(R, M, FLIGHT_CAUSE_BATTERY_LOW, true), FLIGHT_CHANGE_UNDEFINED);
    /* Nothing leaves LAND. */
    CHECK_EQ(flight_mode_check(L, H, FLIGHT_CAUSE_OPERATOR, true), FLIGHT_CHANGE_UNDEFINED);
}

static void rejects_out_of_range_values(void)
{
    CHECK_EQ(flight_mode_check(FLIGHT_MODE_COUNT, M, FLIGHT_CAUSE_OPERATOR, false),
             FLIGHT_CHANGE_UNDEFINED);
    CHECK_EQ(flight_mode_check(M, FLIGHT_MODE_COUNT, FLIGHT_CAUSE_OPERATOR, false),
             FLIGHT_CHANGE_UNDEFINED);
    CHECK_EQ(flight_mode_check(M, R, FLIGHT_CAUSE_COUNT, false), FLIGHT_CHANGE_UNDEFINED);
}

static void names_round_trip(void)
{
    flight_mode_t mode = M;

    for (int i = 0; i < FLIGHT_MODE_COUNT; ++i)
    {
        CHECK(flight_mode_from_name(flight_mode_name((flight_mode_t)i), &mode));
        CHECK_EQ(mode, i);
    }
    CHECK(flight_mode_from_name("RTH", &mode));
    CHECK_EQ(mode, R);
    CHECK(!flight_mode_from_name("rth", &mode));
    CHECK(!flight_mode_from_name("FLY", &mode));
    CHECK(flight_mode_is_safety(R) && flight_mode_is_safety(L));
    CHECK(!flight_mode_is_safety(M) && !flight_mode_is_safety(H));
}

int main(void)
{
    RUN(every_combination_follows_the_rules);
    RUN(named_examples);
    RUN(rejects_out_of_range_values);
    RUN(names_round_trip);
    return (unit_failures == 0) ? 0 : 1;
}
