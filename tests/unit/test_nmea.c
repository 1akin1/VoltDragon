/**
 * @file test_nmea.c
 * @brief Host unit tests for nmea.c. Sentences come from the plant's GPS model (sim/plant/nmea.py).
 */
#include <string.h>

#include "nmea.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

static void parses_gga_fix(void)
{
    nmea_fix_t fix = { 0 };

    CHECK_EQ(nmea_parse("$GPGGA,100001.50,3954.00000,N,03248.00000,E,1,09,0.9,925.0,M,36.0,M,,*64",
                        &fix), NMEA_GGA);
    CHECK(fix.fix);
    CHECK_EQ(fix.quality, 1);
    CHECK_EQ(fix.satellites, 9);
    CHECK_EQ(fix.hdop_x10, 9);
    CHECK_EQ(fix.lat_e7, 399000000);
    CHECK_EQ(fix.lon_e7, 328000000);
    CHECK_EQ(fix.alt_msl_cm, 92500);
    CHECK_EQ(fix.utc_ms, ((10 * 3600) + 1) * 1000 + 500);
}

static void parses_rmc_speed_and_course(void)
{
    nmea_fix_t fix = { 0 };

    CHECK_EQ(nmea_parse("$GPRMC,100001.50,A,3954.00000,N,03248.00000,E,10.00,90.0,091026,,,A*69",
                        &fix), NMEA_RMC);
    CHECK(fix.rmc_valid);
    CHECK_EQ(fix.speed_cmps, 514);      /* 10 knots = 5.144 m/s */
    CHECK_EQ(fix.course_cdeg, 9000);
}

static void parses_southern_and_western_hemispheres(void)
{
    nmea_fix_t fix = { 0 };

    CHECK_EQ(nmea_parse("$GPGGA,100001.50,0130.00000,S,00015.00000,W,1,12,1.2,-12.3,M,36.0,M,,*78",
                        &fix), NMEA_GGA);
    CHECK_EQ(fix.lat_e7, -15000000);
    CHECK_EQ(fix.lon_e7, -2500000);
    CHECK_EQ(fix.alt_msl_cm, -1230);
}

static void keeps_seven_decimal_places(void)
{
    nmea_fix_t fix = { 0 };

    /* 39 deg 07.40736 min = 39.1234560 deg; 32 deg 39.25926 min = 32.6543210 deg. */
    CHECK_EQ(nmea_parse("$GPRMC,100001.50,A,3907.40736,N,03239.25926,E,0.00,0.0,091026,,,A*6D",
                        &fix), NMEA_RMC);
    CHECK_EQ(fix.speed_cmps, 0);
    CHECK_EQ(fix.course_cdeg, 0);
}

static void reports_loss_of_fix_and_keeps_last_position(void)
{
    nmea_fix_t fix = { 0 };

    (void)nmea_parse("$GPGGA,100001.50,3954.00000,N,03248.00000,E,1,09,0.9,925.0,M,36.0,M,,*64",
                     &fix);
    CHECK_EQ(nmea_parse("$GPGGA,100002.00,,,,,0,00,99.9,,M,,M,,*5C", &fix), NMEA_GGA);
    CHECK(!fix.fix);
    CHECK_EQ(fix.satellites, 0);
    CHECK_EQ(fix.lat_e7, 399000000);
    CHECK_EQ(nmea_parse("$GPRMC,100003.00,V,,,,,,,091026,,,N*73", &fix), NMEA_RMC);
    CHECK(!fix.rmc_valid);
}

static void accepts_other_talkers_and_ignores_other_sentences(void)
{
    nmea_fix_t fix = { 0 };

    CHECK_EQ(nmea_parse("$GNGGA,235959.99,3954.00000,N,03248.00000,E,2,15,0.6,1000.5,M,36.0,M,,*45",
                        &fix), NMEA_GGA);
    CHECK_EQ(fix.quality, 2);
    CHECK_EQ(fix.utc_ms, 86399990);
    CHECK_EQ(nmea_parse("$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74",
                        &fix), NMEA_IGNORED);
}

static void rejects_bad_sentences_without_changing_the_fix(void)
{
    nmea_fix_t fix = { 0 };
    nmea_fix_t before;

    (void)nmea_parse("$GPGGA,100001.50,3954.00000,N,03248.00000,E,1,09,0.9,925.0,M,36.0,M,,*64",
                     &fix);
    before = fix;
    CHECK_EQ(nmea_parse("$GPGGA,100001.50,3954.00000,N,03248.00000,E,1,09,0.9,925.0,M,36.0,M,,*65",
                        &fix), NMEA_ERR_CHECKSUM);
    CHECK_EQ(nmea_parse("GPGGA,100001.50*64", &fix), NMEA_ERR_FRAME);
    CHECK_EQ(nmea_parse("$GPGGA,100001.50", &fix), NMEA_ERR_FRAME);
    /* 60 minutes is not a valid minute value. */
    CHECK_EQ(nmea_parse("$GPGGA,100004.00,3960.00000,N,03248.00000,E,1,09,0.9,925.0,M,36.0,M,,*63",
                        &fix), NMEA_ERR_FIELD);
    CHECK(memcmp(&fix, &before, sizeof(fix)) == 0);
}

int main(void)
{
    RUN(parses_gga_fix);
    RUN(parses_rmc_speed_and_course);
    RUN(parses_southern_and_western_hemispheres);
    RUN(keeps_seven_decimal_places);
    RUN(reports_loss_of_fix_and_keeps_last_position);
    RUN(accepts_other_talkers_and_ignores_other_sentences);
    RUN(rejects_bad_sentences_without_changing_the_fix);
    return (unit_failures == 0) ? 0 : 1;
}
