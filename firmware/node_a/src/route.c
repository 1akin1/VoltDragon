/**
 * @file route.c
 * @brief The inspected power line: distance to the conductors.
 */
#include "route.h"

#include <math.h>
#include <stdbool.h>

/* sim/plant/routes/line_a.json */
#define ORIGIN_LAT_E7       (399000000L)
#define ORIGIN_LON_E7       (328000000L)
#define ORIGIN_ALT_CM       (90000L)
#define CONDUCTOR_HEIGHT_M  (25.0f)
#define PHASE_SPACING_M     (6.0f)
#define PYLON_COUNT         (6U)
#define PHASES              (3U)

#define EARTH_RADIUS_M      (6378137.0f)
#define RAD_PER_E7_DEG      (1.745329252e-9f)   /* pi / 180 / 1e7 */
/* cos(39.9 deg): the east-west scale of a degree of longitude at the origin. */
#define COS_ORIGIN_LAT      (0.7671694f)

typedef struct
{
    float east;
    float north;
} pylon_t;

static const pylon_t s_pylons[PYLON_COUNT] = {
    { 0.0f, 0.0f }, { 260.0f, 150.0f }, { 520.0f, 300.0f },
    { 800.0f, 380.0f }, { 1080.0f, 460.0f }, { 1340.0f, 610.0f },
};

typedef struct
{
    float x;
    float y;
    float z;
} point_t;

static float segment_distance(point_t p, point_t a, point_t b)
{
    const point_t d = { b.x - a.x, b.y - a.y, b.z - a.z };
    const point_t ap = { p.x - a.x, p.y - a.y, p.z - a.z };
    const float length2 = (d.x * d.x) + (d.y * d.y) + (d.z * d.z);
    float t = ((ap.x * d.x) + (ap.y * d.y) + (ap.z * d.z)) / length2;

    t = fmaxf(0.0f, fminf(1.0f, t));
    const point_t r = { ap.x - (d.x * t), ap.y - (d.y * t), ap.z - (d.z * t) };
    return sqrtf((r.x * r.x) + (r.y * r.y) + (r.z * r.z));
}

float route_distance_to_conductors_m(int32_t lat_e7, int32_t lon_e7, int32_t alt_msl_cm)
{
    /* Local East-North-Up from the origin, flat-Earth (centimetre accuracy over a few km). */
    const point_t p = {
        (float)(lon_e7 - ORIGIN_LON_E7) * RAD_PER_E7_DEG * EARTH_RADIUS_M * COS_ORIGIN_LAT,
        (float)(lat_e7 - ORIGIN_LAT_E7) * RAD_PER_E7_DEG * EARTH_RADIUS_M,
        (float)(alt_msl_cm - ORIGIN_ALT_CM) / 100.0f,
    };
    float best = INFINITY;

    for (uint32_t span = 0U; (span + 1U) < PYLON_COUNT; ++span)
    {
        const pylon_t a = s_pylons[span];
        const pylon_t b = s_pylons[span + 1U];
        const float dx = b.east - a.east;
        const float dy = b.north - a.north;
        const float length = sqrtf((dx * dx) + (dy * dy));
        /* Unit vector to the right of the span, looking from pylon a to pylon b. */
        const float right_x = dy / length;
        const float right_y = -dx / length;

        for (uint32_t phase = 0U; phase < PHASES; ++phase)
        {
            const float offset = ((float)phase - 1.0f) * PHASE_SPACING_M;
            const point_t ca = { a.east + (right_x * offset), a.north + (right_y * offset),
                                 CONDUCTOR_HEIGHT_M };
            const point_t cb = { b.east + (right_x * offset), b.north + (right_y * offset),
                                 CONDUCTOR_HEIGHT_M };
            best = fminf(best, segment_distance(p, ca, cb));
        }
    }
    return best;
}
