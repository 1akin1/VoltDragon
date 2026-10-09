/**
 * @file route.h
 * @brief The inspected power line, as Node A knows it: distance to the conductors.
 *
 * A copy of sim/plant/routes/line_a.json (pylons, conductor height and phase
 * spacing, local origin). The host unit test test_route compares this code's
 * distances with the plant model's at reference points, so the two cannot
 * drift apart unnoticed.
 *
 * No hardware dependencies; unit-tested on the host (tests/unit).
 */
#ifndef ROUTE_H
#define ROUTE_H

#include <stdint.h>

/**
 * Shortest distance in metres from a position to any phase conductor.
 * @param lat_e7, lon_e7  degrees x 1e7
 * @param alt_msl_cm      altitude above mean sea level, cm
 */
float route_distance_to_conductors_m(int32_t lat_e7, int32_t lon_e7, int32_t alt_msl_cm);

#endif /* ROUTE_H */
