"""Sensor models: IMU and magnetometer (LSM9DS1) at 100 Hz, GPS at 5 Hz.

Outputs are in the units the firmware reports (g, deg/s, gauss) and in the
sensor's body axes (FLU, aligned with the vehicle). Noise is white Gaussian
plus a constant bias per axis; GPS position error is a slowly wandering
Gauss-Markov process, as for a real receiver without corrections.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from datetime import datetime, timedelta

from sim.plant import magnetics, nmea
from sim.plant.route import Route
from sim.plant.vec import Vec3
from sim.plant.vehicle import GRAVITY, VehicleState


@dataclass(frozen=True)
class ImuSample:
    t: float
    accel_g: Vec3
    gyro_dps: Vec3
    mag_gauss: Vec3
    line_field_gauss: float     # magnitude of the line's contribution, for analysis


def _gauss3(rng: random.Random, sigma: float) -> Vec3:
    return Vec3(rng.gauss(0.0, sigma), rng.gauss(0.0, sigma), rng.gauss(0.0, sigma))


class Imu:
    ACCEL_NOISE_G = 0.004
    GYRO_NOISE_DPS = 0.05
    MAG_NOISE_GAUSS = 0.002

    def __init__(self, route: Route, earth: magnetics.EarthField, seed: int = 2) -> None:
        self.route = route
        self.earth_enu = earth.enu()
        self.rng = random.Random(seed)
        self.accel_bias = _gauss3(self.rng, 0.01)
        self.gyro_bias = _gauss3(self.rng, 0.2)
        self.mag_bias = _gauss3(self.rng, 0.003)

    def sample(self, state: VehicleState) -> ImuSample:
        line = magnetics.line_field_gauss(self.route, state.position, state.t)
        field_body = state.attitude.apply_transpose(self.earth_enu + line)
        rate_dps = state.rate_body * math.degrees(1.0)
        return ImuSample(
            t=state.t,
            accel_g=state.specific_force_body * (1.0 / GRAVITY)
            + self.accel_bias
            + _gauss3(self.rng, self.ACCEL_NOISE_G),
            gyro_dps=rate_dps + self.gyro_bias + _gauss3(self.rng, self.GYRO_NOISE_DPS),
            mag_gauss=field_body + self.mag_bias + _gauss3(self.rng, self.MAG_NOISE_GAUSS),
            line_field_gauss=line.norm(),
        )


@dataclass(frozen=True)
class GpsFix:
    t: float
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    sentences: str              # NMEA text as the receiver sends it


class Gps:
    RATE_HZ = 5.0
    WANDER_SIGMA_M = 1.0
    WANDER_TAU_S = 20.0
    VERTICAL_FACTOR = 2.0       # vertical error is larger than horizontal
    SATELLITES = 9
    HDOP = 0.9

    def __init__(self, route: Route, start_utc: datetime, seed: int = 3) -> None:
        self.route = route
        self.start_utc = start_utc
        self.rng = random.Random(seed)
        self.error = Vec3(0.0, 0.0, 0.0)
        self.next_fix_t = 0.0

    def due(self, t: float) -> bool:
        return t + 1e-9 >= self.next_fix_t

    def fix(self, state: VehicleState) -> GpsFix:
        period = 1.0 / self.RATE_HZ
        self.next_fix_t += period
        a = math.exp(-period / self.WANDER_TAU_S)
        b = self.WANDER_SIGMA_M * math.sqrt(1.0 - a * a)
        self.error = Vec3(
            a * self.error.x + b * self.rng.gauss(0.0, 1.0),
            a * self.error.y + b * self.rng.gauss(0.0, 1.0),
            a * self.error.z + b * self.VERTICAL_FACTOR * self.rng.gauss(0.0, 1.0),
        )
        lat, lon, alt = self.route.enu_to_geodetic(state.position + self.error)
        utc = self.start_utc + timedelta(seconds=state.t)
        speed = math.hypot(state.velocity.x, state.velocity.y)
        # Course over ground: from north, clockwise.
        course = math.degrees(math.atan2(state.velocity.x, state.velocity.y))
        text = nmea.gga(utc, lat, lon, alt, self.SATELLITES, self.HDOP) + nmea.rmc(
            utc, lat, lon, speed, course
        )
        return GpsFix(t=state.t, lat_deg=lat, lon_deg=lon, alt_msl_m=alt, sentences=text)
