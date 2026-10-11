/**
 * @file test_safety.c
 * @brief Host unit tests for safety.c, Node A's flight-mode and safety logic (HLR-001 to HLR-006).
 *
 * safety.c runs with the real transition table (flight_mode.c) and test doubles for
 * its inputs and outputs: the GPS fix, the route geometry, the autopilot link, the
 * FreeRTOS lock (stubs/lock.h) and the debug log (stubs/log.h). Each test scripts one
 * scenario cycle by cycle. Together they are the MC/DC test set for safety.c
 * (tools/coverage/mcdc.py, docs/mcdc-report.md); the comments name the decisions
 * and the condition each step shows to act independently.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ap_link.h"
#include "gps.h"
#include "log.h"
#include "route.h"
#include "safety.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

#define M FLIGHT_MODE_MISSION
#define H FLIGHT_MODE_HOLD
#define R FLIGHT_MODE_RETURN_TO_HOME
#define L FLIGHT_MODE_LAND

/* ------------------------------------------------------------------------- */
/* Test doubles                                                              */
/* ------------------------------------------------------------------------- */

#define LOG_LINES   (64U)
#define LOG_WIDTH   (160U)

static struct
{
    bool         gps_ok;
    float        distance_m;
    bool         autopilot_ok;
    uint8_t      battery_pct;
    uint32_t     commands_sent;
    ap_command_t last_command;
    char         log[LOG_LINES][LOG_WIDTH];
    uint32_t     log_count;
} s_world;

bool gps_latest(nmea_fix_t *out, uint32_t now_ms)
{
    (void)now_ms;
    memset(out, 0, sizeof(*out));
    return s_world.gps_ok;
}

uint32_t gps_fix_age_ms(uint32_t now_ms)
{
    (void)now_ms;
    return 40U;
}

float route_distance_to_conductors_m(int32_t lat_e7, int32_t lon_e7, int32_t alt_msl_cm)
{
    (void)lat_e7;
    (void)lon_e7;
    (void)alt_msl_cm;
    return s_world.distance_m;
}

void ap_link_poll(uint32_t now_ms)
{
    (void)now_ms;
}

bool ap_link_status(ap_status_t *out, uint32_t now_ms)
{
    (void)now_ms;
    out->battery_pct = s_world.battery_pct;
    out->mode = M;
    return s_world.autopilot_ok;
}

void ap_link_send(const ap_command_t *cmd)
{
    s_world.last_command = *cmd;
    s_world.commands_sent++;
}

void log_line(char level, const char *fmt, ...)
{
    va_list args;
    char *line = s_world.log[s_world.log_count % LOG_LINES];

    line[0] = level;
    line[1] = ' ';
    va_start(args, fmt);
    (void)vsnprintf(&line[2], LOG_WIDTH - 2U, fmt, args);
    va_end(args);
    s_world.log_count++;
}

/** True if a line containing @p text was logged since the test started. */
static bool logged(const char *text)
{
    const uint32_t first = (s_world.log_count > LOG_LINES) ? (s_world.log_count - LOG_LINES) : 0U;

    for (uint32_t i = first; i < s_world.log_count; ++i)
    {
        if (strstr(s_world.log[i % LOG_LINES], text) != NULL)
        {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------- */
/* Scenario helpers                                                          */
/* ------------------------------------------------------------------------- */

/** A healthy vehicle far from the line, with no ground station heard yet. */
static void start(void)
{
    memset(&s_world, 0, sizeof(s_world));
    s_world.gps_ok = true;
    s_world.distance_m = 100.0f;
    s_world.autopilot_ok = true;
    s_world.battery_pct = 80U;
    safety_init();
}

static void ground(uint16_t link_age_ms, bool contact, uint32_t now_ms)
{
    const canmsg_b_status_t status = {
        .link_age_ms = link_age_ms,
        .flags = contact ? CANMSG_B_GROUND_CONTACT : 0U,
    };
    safety_on_ground_status(&status, now_ms);
}

static void request(uint8_t id, uint8_t mode, bool override)
{
    const canmsg_mode_req_t req = { .request_id = id, .mode = mode, .override = override ? 1U : 0U };
    safety_on_mode_request(&req);
}

/* ------------------------------------------------------------------------- */
/* Autopilot commands (command_autopilot)                                    */
/* ------------------------------------------------------------------------- */

static void commands_the_autopilot_at_once_and_then_every_100_ms(void)
{
    start();
    safety_poll(0U);                    /* first command: never sent before */
    CHECK_EQ(s_world.commands_sent, 1);
    CHECK_EQ(s_world.last_command.mode, M);
    CHECK(!s_world.last_command.avoid);
    CHECK_EQ(s_world.last_command.min_distance_dm, 0);

    safety_poll(20U);                   /* nothing changed, 20 ms since: no command */
    CHECK_EQ(s_world.commands_sent, 1);
    safety_poll(100U);                  /* nothing changed, 100 ms since: repeated */
    CHECK_EQ(s_world.commands_sent, 2);
}

static void sends_a_mode_change_in_the_same_cycle(void)
{
    start();
    safety_poll(0U);
    request(1U, (uint8_t)H, false);
    safety_poll(20U);                   /* mode changed, 20 ms since: sent at once */
    CHECK_EQ(s_world.commands_sent, 2);
    CHECK_EQ(s_world.last_command.mode, H);
}

/* ------------------------------------------------------------------------- */
/* Distance to the line (update_distance), HLR-001 / HLR-002                 */
/* ------------------------------------------------------------------------- */

static void warns_below_12_m_and_clears_above_15_m(void)
{
    start();
    s_world.distance_m = 20.0f;
    safety_poll(0U);                    /* no warning, far: stays clear */
    CHECK(!safety_state().proximity);

    s_world.distance_m = 11.9f;
    safety_poll(20U);                   /* no warning, below 12 m: raised */
    CHECK(safety_state().proximity);
    CHECK(logged("PROXIMITY_WARNING, 119 dm"));
    /* The avoidance order goes out in the same cycle (HLR-002). */
    CHECK_EQ(s_world.commands_sent, 2);
    CHECK(s_world.last_command.avoid);
    CHECK_EQ(s_world.last_command.min_distance_dm, SAFETY_AVOID_DISTANCE_DM);

    s_world.distance_m = 11.0f;
    safety_poll(40U);                   /* warning already raised, still below 12 m */
    s_world.distance_m = 14.0f;
    safety_poll(60U);                   /* warning raised, between 12 and 15 m: kept */
    CHECK(safety_state().proximity);

    s_world.distance_m = 15.1f;
    safety_poll(80U);                   /* warning raised, above 15 m: cleared */
    CHECK(!safety_state().proximity);
    CHECK(logged("proximity warning cleared at 151 dm"));
    CHECK(!s_world.last_command.avoid);
}

static void keeps_the_last_decision_without_a_fix(void)
{
    start();
    s_world.distance_m = 5.0f;
    safety_poll(0U);
    s_world.gps_ok = false;
    s_world.distance_m = 50.0f;
    safety_poll(20U);
    const safety_state_t s = safety_state();
    CHECK(!s.have_distance);
    CHECK(s.proximity);
}

/* ------------------------------------------------------------------------- */
/* Ground link (update_ground_link), HLR-003                                 */
/* ------------------------------------------------------------------------- */

static void does_not_lose_a_link_never_established(void)
{
    start();
    ground(5000U, false, 0U);           /* old link age, but never in contact */
    safety_poll(0U);
    CHECK(!safety_state().link_lost);
    CHECK_EQ(safety_state().mode, M);
}

static void returns_home_3_s_after_the_last_contact_and_reports_recovery(void)
{
    start();
    ground(100U, true, 0U);             /* in contact, fresh: link fine */
    safety_poll(0U);
    CHECK(!safety_state().link_lost);

    ground(3001U, true, 20U);           /* in contact before, over 3 s: lost */
    safety_poll(20U);
    CHECK(safety_state().link_lost);
    CHECK_EQ(safety_state().mode, R);
    CHECK(logged("ground link lost for 3001 ms"));

    safety_poll(40U);                   /* still lost: no second trigger */
    CHECK_EQ(safety_state().rejected_requests, 0);

    ground(100U, true, 60U);            /* contact again: link back, mode stays */
    safety_poll(60U);
    CHECK(!safety_state().link_lost);
    CHECK(logged("ground link back"));
    CHECK_EQ(safety_state().mode, R);
}

static void counts_node_b_silence_as_link_age(void)
{
    start();
    ground(0U, true, 0U);
    safety_poll(0U);
    safety_poll(3000U);                 /* 3000 ms: not more than 3 s yet */
    CHECK(!safety_state().link_lost);
    safety_poll(3001U);
    CHECK(safety_state().link_lost);
    CHECK_EQ(safety_state().mode, R);
}

/* ------------------------------------------------------------------------- */
/* Battery (update_battery), HLR-004                                         */
/* ------------------------------------------------------------------------- */

static void returns_home_below_20_percent_and_lands_below_10(void)
{
    start();
    s_world.battery_pct = 20U;
    safety_poll(0U);                    /* at 20 %: not low */
    CHECK_EQ(safety_state().mode, M);

    s_world.battery_pct = 19U;
    safety_poll(20U);                   /* low starts: return home */
    CHECK_EQ(safety_state().mode, R);
    CHECK(logged("battery low, 19 %"));
    CHECK(safety_state().battery_low);
    safety_poll(40U);                   /* still low: no new trigger */

    s_world.battery_pct = 9U;
    safety_poll(60U);                   /* critical starts: land */
    CHECK_EQ(safety_state().mode, L);
    CHECK(logged("battery critical, 9 %"));
    safety_poll(80U);                   /* still critical: no new trigger */
    CHECK_EQ(safety_state().mode, L);
    CHECK_EQ(safety_state().rejected_requests, 0);
}

static void lands_at_once_on_a_sudden_critical_battery(void)
{
    start();
    s_world.battery_pct = 5U;
    safety_poll(0U);
    const safety_state_t s = safety_state();
    CHECK_EQ(s.mode, L);
    CHECK(s.battery_low && s.battery_critical);
}

static void ignores_the_battery_while_the_autopilot_is_silent(void)
{
    start();
    s_world.autopilot_ok = false;
    s_world.battery_pct = 5U;
    safety_poll(0U);
    const safety_state_t s = safety_state();
    CHECK(!s.autopilot_ok);
    CHECK_EQ(s.battery_pct, CANMSG_UNKNOWN_U8);
    CHECK_EQ(s.mode, M);
}

/* ------------------------------------------------------------------------- */
/* Automatic triggers (trigger)                                              */
/* ------------------------------------------------------------------------- */

static void a_trigger_towards_the_current_mode_does_nothing(void)
{
    start();
    s_world.battery_pct = 15U;
    safety_poll(0U);                    /* battery low: return home */
    CHECK_EQ(safety_state().mode, R);
    ground(3500U, true, 20U);           /* link loss would also return home: already there */
    safety_poll(20U);
    CHECK_EQ(safety_state().mode, R);
    CHECK(!logged("safety: mode RETURN_TO_HOME -> RETURN_TO_HOME"));
}

static void nothing_triggers_out_of_land(void)
{
    start();
    s_world.battery_pct = 5U;
    safety_poll(0U);                    /* land */
    ground(3500U, true, 20U);           /* link loss after landing: stays landed */
    safety_poll(20U);
    CHECK_EQ(safety_state().mode, L);
    CHECK(safety_state().link_lost);
    CHECK_EQ(safety_state().rejected_requests, 0);

    request(1U, (uint8_t)M, true);      /* not even the operator's override leaves LAND */
    safety_poll(40U);
    CHECK_EQ(safety_state().mode, L);
    CHECK_EQ(safety_state().rejected_requests, 1);
    CHECK(logged("rejected LAND -> MISSION (operator, override): UNDEFINED"));
}

/* ------------------------------------------------------------------------- */
/* Operator requests (handle_request, change_mode), HLR-005 / HLR-006        */
/* ------------------------------------------------------------------------- */

static void applies_operator_requests_through_the_transition_table(void)
{
    start();
    request(1U, (uint8_t)H, false);     /* first request: applied */
    safety_poll(0U);
    CHECK_EQ(safety_state().mode, H);
    CHECK_EQ(safety_state().last_request_id, 1);
    CHECK(logged("safety: mode MISSION -> HOLD (operator)"));

    request(2U, (uint8_t)H, false);     /* already there: neither applied nor rejected */
    safety_poll(20U);
    CHECK_EQ(safety_state().rejected_requests, 0);

    request(3U, (uint8_t)R, false);
    safety_poll(40U);
    CHECK_EQ(safety_state().mode, R);
    request(4U, (uint8_t)M, false);     /* leaving return-to-home needs the override */
    safety_poll(60U);
    CHECK_EQ(safety_state().mode, R);
    CHECK_EQ(safety_state().rejected_requests, 1);
    CHECK(logged("rejected RETURN_TO_HOME -> MISSION (operator): NEEDS_OVERRIDE"));
    request(5U, (uint8_t)M, true);
    safety_poll(80U);
    CHECK_EQ(safety_state().mode, M);
    CHECK(logged("safety: mode RETURN_TO_HOME -> MISSION (operator, override)"));
}

static void rejects_an_unknown_mode(void)
{
    start();
    request(1U, (uint8_t)FLIGHT_MODE_COUNT, false);
    safety_poll(0U);
    CHECK_EQ(safety_state().mode, M);
    CHECK_EQ(safety_state().rejected_requests, 1);
    CHECK(logged("rejected request 1 for unknown mode 4"));
}

static void ignores_a_repeated_request_id_within_500_ms(void)
{
    start();
    request(7U, (uint8_t)H, false);
    safety_poll(0U);
    CHECK_EQ(safety_state().mode, H);

    request(7U, (uint8_t)M, false);     /* same id, 20 ms later: a duplicate frame */
    safety_poll(20U);
    CHECK_EQ(safety_state().mode, H);

    request(8U, (uint8_t)M, false);     /* new id within the window: applied */
    safety_poll(40U);
    CHECK_EQ(safety_state().mode, M);

    request(8U, (uint8_t)H, false);     /* same id 500 ms later: a new request */
    safety_poll(540U);
    CHECK_EQ(safety_state().mode, H);
}

/* ------------------------------------------------------------------------- */
/* Reporting (safety_to_message, safety_report)                              */
/* ------------------------------------------------------------------------- */

static void encodes_the_state_for_the_can_safety_message(void)
{
    safety_state_t s = { 0 };
    canmsg_safety_t msg;

    /* Nothing known. */
    safety_to_message(&s, &msg);
    CHECK_EQ(msg.distance_dm, CANMSG_UNKNOWN_U16);
    CHECK_EQ(msg.battery_pct, CANMSG_UNKNOWN_U8);
    CHECK_EQ(msg.flags, 0);

    /* Everything raised. */
    s.mode = R;
    s.have_distance = true;
    s.distance_m = 11.5f;
    s.proximity = true;
    s.link_lost = true;
    s.battery_low = true;
    s.battery_critical = true;
    s.autopilot_ok = true;
    s.battery_pct = 9U;
    s.last_request_id = 42U;
    safety_to_message(&s, &msg);
    CHECK_EQ(msg.distance_dm, 115);
    CHECK_EQ(msg.battery_pct, 9);
    CHECK_EQ(msg.mode, R);
    CHECK_EQ(msg.last_request_id, 42);
    CHECK_EQ(msg.flags, CANMSG_SAFETY_PROXIMITY | CANMSG_SAFETY_AVOIDING |
                        CANMSG_SAFETY_LINK_LOST | CANMSG_SAFETY_BATTERY_LOW |
                        CANMSG_SAFETY_BATTERY_CRIT | CANMSG_SAFETY_AUTOPILOT_OK);

    /* Far away: the distance saturates below the "unknown" value. */
    s.distance_m = 9000.0f;
    safety_to_message(&s, &msg);
    CHECK_EQ(msg.distance_dm, 65534);
}

static void reports_the_state_on_the_console(void)
{
    start();
    safety_report();                    /* before any cycle: nothing known */
    CHECK(logged("distance unknown/0 dm, battery 255 %, autopilot silent, ground link ok"));

    s_world.distance_m = 42.0f;
    s_world.autopilot_ok = false;
    ground(3500U, true, 0U);
    safety_poll(0U);
    safety_report();
    CHECK(logged("distance 420 dm, battery 255 %, autopilot silent, ground link LOST"));

    s_world.autopilot_ok = true;
    safety_poll(20U);
    safety_report();
    CHECK(logged("battery 80 %, autopilot ok, ground link LOST"));
}

int main(void)
{
    RUN(commands_the_autopilot_at_once_and_then_every_100_ms);
    RUN(sends_a_mode_change_in_the_same_cycle);
    RUN(warns_below_12_m_and_clears_above_15_m);
    RUN(keeps_the_last_decision_without_a_fix);
    RUN(does_not_lose_a_link_never_established);
    RUN(returns_home_3_s_after_the_last_contact_and_reports_recovery);
    RUN(counts_node_b_silence_as_link_age);
    RUN(returns_home_below_20_percent_and_lands_below_10);
    RUN(lands_at_once_on_a_sudden_critical_battery);
    RUN(ignores_the_battery_while_the_autopilot_is_silent);
    RUN(a_trigger_towards_the_current_mode_does_nothing);
    RUN(nothing_triggers_out_of_land);
    RUN(applies_operator_requests_through_the_transition_table);
    RUN(rejects_an_unknown_mode);
    RUN(ignores_a_repeated_request_id_within_500_ms);
    RUN(encodes_the_state_for_the_can_safety_message);
    RUN(reports_the_state_on_the_console);
    return (unit_failures == 0) ? 0 : 1;
}
