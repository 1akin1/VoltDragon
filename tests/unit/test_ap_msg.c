/**
 * @file test_ap_msg.c
 * @brief Host unit tests for ap_msg.c (autopilot link messages).
 */
#include <string.h>

#include "ap_msg.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

static void formats_commands(void)
{
    char buf[AP_MSG_MAX_LEN];
    const ap_command_t avoid = { FLIGHT_MODE_MISSION, true, 150U };
    const ap_command_t rth = { FLIGHT_MODE_RETURN_TO_HOME, false, 0U };

    /* "VDCMD,MISSION,1,150": XOR = 0x3D (also in tests/python/test_autopilot.py). */
    CHECK_EQ(ap_format_command(&avoid, buf, sizeof(buf)), strlen("$VDCMD,MISSION,1,150*3D\r\n"));
    CHECK_STR(buf, "$VDCMD,MISSION,1,150*3D\r\n");
    (void)ap_format_command(&rth, buf, sizeof(buf));
    CHECK(strncmp(buf, "$VDCMD,RTH,0,0*", 15) == 0);
    CHECK_EQ(ap_format_command(&rth, buf, 10U), 0);
}

static void command_round_trip(void)
{
    char buf[AP_MSG_MAX_LEN];
    ap_command_t out = { FLIGHT_MODE_MISSION, false, 0U };

    for (int mode = 0; mode < FLIGHT_MODE_COUNT; ++mode)
    {
        const ap_command_t in = { (flight_mode_t)mode, (mode % 2) == 0, (uint16_t)(65535 - mode) };
        const size_t len = ap_format_command(&in, buf, sizeof(buf));
        buf[len - 2U] = '\0';      /* the parser takes lines without their ending */
        CHECK(ap_parse_command(buf, &out));
        CHECK_EQ(out.mode, in.mode);
        CHECK_EQ(out.avoid, in.avoid);
        CHECK_EQ(out.min_distance_dm, in.min_distance_dm);
    }
}

static void parses_status(void)
{
    ap_status_t status = { 0U, FLIGHT_MODE_MISSION };

    /* "VDAPS,18,RTH": XOR = 0x17. */
    CHECK(ap_parse_status("$VDAPS,18,RTH*17", &status));
    CHECK_EQ(status.battery_pct, 18);
    CHECK_EQ(status.mode, FLIGHT_MODE_RETURN_TO_HOME);
}

static void rejects_bad_status(void)
{
    ap_status_t status = { 55U, FLIGHT_MODE_HOLD };

    /* Each line has a correct checksum unless that is what is being tested. */
    CHECK(!ap_parse_status("$VDAPS,18,RTH*18", &status));      /* wrong checksum */
    CHECK(!ap_parse_status("VDAPS,18,RTH*17", &status));       /* no '$' */
    CHECK(!ap_parse_status("$VDAPS,18*75", &status));          /* missing field */
    CHECK(!ap_parse_status("$VDAPS,101,RTH*2E", &status));     /* over 100 % */
    CHECK(!ap_parse_status("$VDAPS,18,FLY*0A", &status));      /* unknown mode */
    CHECK(!ap_parse_status("$VDCMD,MISSION,1,150*3D", &status));   /* a command, not a status */
    CHECK_EQ(status.battery_pct, 55);                           /* unchanged */
}

int main(void)
{
    RUN(formats_commands);
    RUN(command_round_trip);
    RUN(parses_status);
    RUN(rejects_bad_status);
    return (unit_failures == 0) ? 0 : 1;
}
