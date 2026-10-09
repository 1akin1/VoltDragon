/**
 * @file nav.c
 * @brief Heading and magnetometer integrity for Node A.
 */
#include "nav.h"

#include <math.h>

#include "gps.h"
#include "imu.h"
#include "log.h"

#define DEG_PER_RAD     (57.29577951f)
#define RAD_PER_MDPS    (1.745329252e-5f)
#define CDEG_PER_DEG    (100.0f)
#define FULL_CIRCLE_DEG (360.0f)
#define SAMPLE_PERIOD_S (0.01f)     /* IMU_SAMPLE_PERIOD_MS */

/*
 * "Down" (the gravity direction in body axes) comes from a complementary filter.
 * An accelerometer measures specific force, which tilts away from gravity while
 * the vehicle accelerates, with little change in magnitude: a 10 degree error
 * adds only 1.5 % to it. So the estimate is propagated with the gyroscope every
 * sample and pulled slowly towards the accelerometer, with a 2 s time constant,
 * and only while the accelerometer magnitude is within 3 % of 1 g.
 */
#define DOWN_TAU_S      (2.0f)
#define MIN_GRAVITY_MG  (970.0f)
#define MAX_GRAVITY_MG  (1030.0f)

typedef struct
{
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct
{
    nav_state_t state;
    uint32_t    last_sample;
    uint32_t    disturbed_run;
    uint32_t    clean_run;
    bool        have_down;
    vec3f_t     down;           /* Unit vector, body axes. */
} nav_internal_t;

static nav_internal_t s_nav;

static vec3f_t to_vec(const int32_t *v)
{
    const vec3f_t r = { (float)v[0], (float)v[1], (float)v[2] };
    return r;
}

static float dot(vec3f_t a, vec3f_t b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static vec3f_t cross(vec3f_t a, vec3f_t b)
{
    const vec3f_t r = { (a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z),
                        (a.x * b.y) - (a.y * b.x) };
    return r;
}

static float wrap_degrees(float deg)
{
    float d = fmodf(deg, FULL_CIRCLE_DEG);
    return (d < 0.0f) ? (d + FULL_CIRCLE_DEG) : d;
}

static uint16_t to_cdeg(float deg)
{
    const uint32_t cdeg = (uint32_t)lroundf(wrap_degrees(deg) * CDEG_PER_DEG);
    return (uint16_t)(cdeg % 36000U);
}

/** Updates the disturbance state machine with one sample's verdict. */
static void update_integrity(bool disturbed)
{
    if (disturbed)
    {
        s_nav.clean_run = 0U;
        s_nav.disturbed_run++;
        if (s_nav.state.mag_ok && (s_nav.disturbed_run >= NAV_DISTURBED_SAMPLES))
        {
            s_nav.state.mag_ok = false;
            s_nav.state.disturbances++;
            LOG_WARN("nav: magnetometer disturbed (field %u mG, dip %d cdeg), not using its heading",
                     (unsigned int)s_nav.state.field_mgauss, (int)s_nav.state.dip_cdeg);
        }
    }
    else
    {
        s_nav.disturbed_run = 0U;
        s_nav.clean_run++;
        if (!s_nav.state.mag_ok && (s_nav.clean_run >= NAV_RECOVERY_SAMPLES))
        {
            s_nav.state.mag_ok = true;
            LOG_INFO("nav: magnetometer field back within limits");
        }
    }
}

static vec3f_t normalised(vec3f_t v)
{
    const float n = sqrtf(dot(v, v));
    const vec3f_t r = { v.x / n, v.y / n, v.z / n };
    return r;
}

/** Updates the "down" estimate; returns false until it has been initialised. */
static bool update_down(const lsm9ds1_sample_t *s)
{
    const vec3f_t accel = to_vec(s->accel_mg);
    const float gravity = sqrtf(dot(accel, accel));
    const bool gravity_ok = (gravity > MIN_GRAVITY_MG) && (gravity < MAX_GRAVITY_MG);

    if (!s_nav.have_down)
    {
        if (!gravity_ok)
        {
            return false;
        }
        /* The accelerometer measures the reaction to gravity, so "down" is opposite to it. */
        const vec3f_t initial = { -accel.x / gravity, -accel.y / gravity, -accel.z / gravity };
        s_nav.down = initial;
        s_nav.have_down = true;
        return true;
    }

    /* A body rotation w turns a fixed world direction by -w x d in body axes. */
    const vec3f_t w = { (float)s->gyro_mdps[0] * RAD_PER_MDPS,
                        (float)s->gyro_mdps[1] * RAD_PER_MDPS,
                        (float)s->gyro_mdps[2] * RAD_PER_MDPS };
    const vec3f_t turn = cross(w, s_nav.down);
    vec3f_t d = { s_nav.down.x - (turn.x * SAMPLE_PERIOD_S),
                  s_nav.down.y - (turn.y * SAMPLE_PERIOD_S),
                  s_nav.down.z - (turn.z * SAMPLE_PERIOD_S) };

    if (gravity_ok)
    {
        const float k = SAMPLE_PERIOD_S / DOWN_TAU_S;
        d.x += k * ((-accel.x / gravity) - d.x);
        d.y += k * ((-accel.y / gravity) - d.y);
        d.z += k * ((-accel.z / gravity) - d.z);
    }
    s_nav.down = normalised(d);
    return true;
}

static void process_sample(const lsm9ds1_sample_t *s, uint32_t now_ms)
{
    const vec3f_t mag = to_vec(s->mag_mgauss);
    const float field = sqrtf(dot(mag, mag));
    const bool have_down = update_down(s);
    const vec3f_t down = s_nav.down;

    s_nav.state.field_mgauss = (uint16_t)((field > 65535.0f) ? 65535.0f : field);

    bool disturbed = fabsf(field - NAV_FIELD_MGAUSS) > (NAV_FIELD_TOLERANCE * NAV_FIELD_MGAUSS);
    if (have_down && (field > 0.0f))
    {
        const float sine = fmaxf(-1.0f, fminf(1.0f, dot(mag, down) / field));
        const float dip = asinf(sine) * DEG_PER_RAD;
        s_nav.state.dip_cdeg = (int16_t)lroundf(dip * CDEG_PER_DEG);
        disturbed = disturbed || (fabsf(dip - NAV_DIP_DEG) > NAV_DIP_TOLERANCE_DEG);
    }
    update_integrity(disturbed);

    nmea_fix_t fix;
    const bool gps_ok = gps_latest(&fix, now_ms) && fix.rmc_valid &&
                        (fix.speed_cmps >= NAV_GPS_COURSE_MIN_CMPS);

    if (s_nav.state.mag_ok && have_down)
    {
        /* Horizontal magnetic east and north in body axes; heading of the body x axis. */
        const vec3f_t east = cross(down, mag);
        const vec3f_t north = cross(east, down);
        const float heading = atan2f(east.x, north.x) * DEG_PER_RAD + NAV_DECLINATION_DEG;
        s_nav.state.heading_cdeg = to_cdeg(heading);
        s_nav.state.source = NAV_HEADING_MAG;
    }
    else if (gps_ok)
    {
        /* Course over ground: the direction of travel, close to the heading in forward flight. */
        s_nav.state.heading_cdeg = fix.course_cdeg;
        s_nav.state.source = NAV_HEADING_GPS;
    }
    else
    {
        s_nav.state.source = NAV_HEADING_NONE;
    }
}

void nav_init(void)
{
    s_nav = (nav_internal_t){ 0 };
    s_nav.state.mag_ok = true;
    LOG_INFO("nav: expecting %u mG, dip %u deg; disturbed beyond %u%% or %u deg",
             (unsigned int)NAV_FIELD_MGAUSS, (unsigned int)NAV_DIP_DEG,
             (unsigned int)(NAV_FIELD_TOLERANCE * 100.0f), (unsigned int)NAV_DIP_TOLERANCE_DEG);
}

void nav_poll(uint32_t now_ms)
{
    lsm9ds1_sample_t sample;
    const uint32_t count = imu_sample_count();

    if ((count == s_nav.last_sample) || !imu_latest(&sample))
    {
        return;
    }
    s_nav.last_sample = count;
    process_sample(&sample, now_ms);
}

nav_state_t nav_state(void)
{
    return s_nav.state;
}

void nav_report(void)
{
    static const char *const sources[] = { "none", "magnetometer", "GPS course" };
    const nav_state_t *n = &s_nav.state;

    LOG_INFO("nav: heading %u cdeg from %s, field %u mG, dip %d cdeg, magnetometer %s",
             (unsigned int)n->heading_cdeg, sources[n->source], (unsigned int)n->field_mgauss,
             (int)n->dip_cdeg, n->mag_ok ? "ok" : "DISTURBED");
}
