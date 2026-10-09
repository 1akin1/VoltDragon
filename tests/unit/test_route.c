/**
 * @file test_route.c
 * @brief Host unit test for Node A's route.c against the plant model's geometry.
 *
 * The reference distances were computed with sim/plant/route.py
 * (Route.load("line_a").distance_to_conductors) at the same positions, so the
 * firmware's copy of the route must match sim/plant/routes/line_a.json.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "route.h"
#include "unit.h"

UNIT_MAIN_DEFINITIONS;

typedef struct
{
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_cm;
    float   distance_m;
    const char *where;
} reference_t;

static const reference_t REFERENCES[] = {
    { 399005177, 328016384, 92500, 14.003f, "cruise: 20 m right of the centre line" },
    { 399005956, 328015799, 92500, 3.995f, "close pass: 10 m right of the centre line" },
    { 399006734, 328015214, 94000, 15.000f, "15 m above the centre conductor" },
    { 399030705, 328071629, 92500, 9.002f, "left of the line, after the second bend" },
    { 399047993, 328142706, 92000, 5.099f, "near the last pylon" },
    { 399000000, 328000000, 92500, 0.000f, "on the centre conductor at the first pylon" },
};

static void matches_the_plant_model(void)
{
    for (size_t i = 0U; i < sizeof(REFERENCES) / sizeof(REFERENCES[0]); ++i)
    {
        const reference_t *r = &REFERENCES[i];
        const float d = route_distance_to_conductors_m(r->lat_e7, r->lon_e7, r->alt_cm);
        if (fabsf(d - r->distance_m) > 0.05f)
        {
            printf("  %s: %.3f m, expected %.3f m\n", r->where, (double)d, (double)r->distance_m);
        }
        CHECK(fabsf(d - r->distance_m) <= 0.05f);
    }
}

int main(void)
{
    RUN(matches_the_plant_model);
    return (unit_failures == 0) ? 0 : 1;
}
