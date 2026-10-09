"""Kinematic multirotor flying an inspection pass along the power line.

The vehicle follows a desired point that moves along the route at the plan's
speed, at a lateral offset from the line's centre and a set altitude. A simple
position/velocity controller tracks it; wind pushes the vehicle through drag,
so gusts cause realistic tracking errors.

A multirotor tilts its thrust to accelerate, so the body z axis points along
the specific force (acceleration minus gravity) and the nose follows the route
direction with a lag. Body axes are x forward, y left, z up (FLU); world axes
are East, North, Up (ENU).

The autopilot (sim/plant/autopilot.py) decides what the desired point does:
MISSION moves it along the route, HOLD stops it, RTH moves it back to the start
at the plan's normal offset, and LAND lowers it to the ground at 1 m/s where the
vehicle is. An active keep-out pushes it away from the line until it is at least
that far from every conductor.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from sim.plant.autopilot import Autopilot
from sim.plant.route import Route
from sim.plant.vec import ZERO, Mat3, Vec3, body_rates

GRAVITY = 9.80665


@dataclass(frozen=True)
class FlightPlan:
    speed_mps: float = 6.0
    speed_ramp_s: float = 8.0             # from hover to cruise speed
    altitude_m: float = 25.0              # level with the conductors
    offset_m: float = 20.0                # to the right of the line's centre, looking along it
    start_s_m: float = 0.0                # along-track position at t = 0
    # Close inspection pass: the offset ramps to close_offset_m over ramp_m
    # before close_from_m and back after close_to_m.
    close_from_m: float = 240.0
    close_to_m: float = 330.0
    close_offset_m: float = 10.0
    ramp_m: float = 30.0

    def offset_at(self, s: float) -> float:
        lo, hi, ramp = self.close_from_m, self.close_to_m, self.ramp_m
        if s <= lo - ramp or s >= hi + ramp:
            return self.offset_m
        if lo <= s <= hi:
            return self.close_offset_m
        fraction = (lo - s) / ramp if s < lo else (s - hi) / ramp
        return self.close_offset_m + (self.offset_m - self.close_offset_m) * fraction


@dataclass
class Wind:
    """Mean wind plus first-order Gauss-Markov gusts on each horizontal axis."""

    mean_enu: Vec3 = Vec3(-2.0, 1.0, 0.0)
    gust_sigma_mps: float = 1.5
    gust_tau_s: float = 2.0
    seed: int = 1
    _gust: Vec3 = field(default=ZERO, init=False)
    _rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def step(self, dt: float) -> Vec3:
        a = math.exp(-dt / self.gust_tau_s)
        b = self.gust_sigma_mps * math.sqrt(1.0 - a * a)
        self._gust = Vec3(
            a * self._gust.x + b * self._rng.gauss(0.0, 1.0),
            a * self._gust.y + b * self._rng.gauss(0.0, 1.0),
            0.0,
        )
        return self.mean_enu + self._gust


@dataclass(frozen=True)
class VehicleState:
    t: float
    position: Vec3              # ENU, m
    velocity: Vec3              # ENU, m/s
    attitude: Mat3              # body (FLU) to world (ENU)
    rate_body: Vec3             # rad/s
    specific_force_body: Vec3   # m/s^2, what an accelerometer measures
    s_m: float                  # along-track position of the desired point
    distance_to_line_m: float   # to the nearest conductor
    wind: Vec3
    mode: str = "MISSION"       # what the vehicle is flying (the autopilot's mode)
    airborne: bool = True


class Vehicle:
    # Controller and drag gains: about a 2 s position response, moderate wind sensitivity.
    K_POSITION = 0.6
    K_VELOCITY = 2.0
    K_DRAG = 0.25
    YAW_TAU_S = 1.5
    ATTITUDE_TAU_S = 0.2
    MAX_ACCEL_MPS2 = 3.0          # about 17 degrees of tilt
    LAND_SPEED_MPS = 1.0
    MAX_EXTRA_OFFSET_M = 60.0     # search range for the keep-out

    def __init__(
        self, route: Route, plan: FlightPlan, wind: Wind, autopilot: Autopilot | None = None
    ) -> None:
        self.route = route
        self.plan = plan
        self.wind = wind
        self.autopilot = autopilot
        self.t = 0.0
        self.s = plan.start_s_m
        self.mode = "MISSION"
        self.airborne = True
        self.offset = plan.offset_at(self.s)
        self.land_point = ZERO
        self.land_t = 0.0
        self.position = self._desired(self.s)
        self.velocity = ZERO
        _, direction = route.point_at(self.s)
        self.yaw = math.atan2(direction.y, direction.x)
        self.thrust_axis = Vec3(0.0, 0.0, GRAVITY)
        self.attitude = self._attitude(self.thrust_axis)

    def _desired(self, s: float) -> Vec3:
        return self._point(s, self.plan.offset_at(s))

    def _point(self, s: float, offset: float) -> Vec3:
        ground, direction = self.route.point_at(s)
        right = Vec3(direction.y, -direction.x, 0.0)
        return ground + right * offset + Vec3(0.0, 0.0, self.plan.altitude_m)

    def _attitude(self, specific_force: Vec3) -> Mat3:
        z_b = specific_force.unit()
        heading = Vec3(math.cos(self.yaw), math.sin(self.yaw), 0.0)
        y_b = z_b.cross(heading).unit()
        x_b = y_b.cross(z_b)
        return Mat3.from_columns(x_b, y_b, z_b)

    def _respect_keep_out(self, s: float, offset: float) -> float:
        """The offset, moved away from the line until the point respects the keep-out."""
        keep_out = self.autopilot.keep_out_m if self.autopilot is not None else None
        if keep_out is None:
            return offset
        if self.route.distance_to_conductors(self._point(s, offset)) >= keep_out:
            return offset
        lo, hi = offset, offset + self.MAX_EXTRA_OFFSET_M
        for _ in range(30):
            mid = (lo + hi) / 2.0
            if self.route.distance_to_conductors(self._point(s, mid)) < keep_out:
                lo = mid
            else:
                hi = mid
        return hi

    def _change_mode(self, mode: str) -> None:
        if mode == "LAND":
            self.land_point = self.position
            self.land_t = self.t
        self.mode = mode

    def _guidance(self, dt: float) -> tuple[Vec3, Vec3, Vec3]:
        """Moves the desired point for the current mode.

        Returns the desired point, the feed-forward velocity and the direction of travel.
        """
        if self.mode == "LAND":
            elapsed = self.t + dt - self.land_t
            altitude = max(0.0, self.land_point.z - self.LAND_SPEED_MPS * elapsed)
            target = Vec3(self.land_point.x, self.land_point.y, altitude)
            sink = Vec3(0.0, 0.0, -self.LAND_SPEED_MPS if altitude > 0.0 else 0.0)
            return target, sink, ZERO

        if self.mode == "MISSION":
            # The vehicle starts in a hover and builds up to cruise speed gradually.
            speed = self.plan.speed_mps * min(1.0, (self.t + dt) / self.plan.speed_ramp_s)
            self.s = min(self.s + speed * dt, self.route.length())
            plan_offset = self.plan.offset_at(self.s)
            along = 1.0
        elif self.mode == "RTH":
            # Back along the route to the start, at the plan's normal offset.
            speed = self.plan.speed_mps if self.s > self.plan.start_s_m else 0.0
            self.s = max(self.s - speed * dt, self.plan.start_s_m)
            plan_offset = self.plan.offset_m
            along = -1.0
        else:
            # HOLD: the desired point stays where it is.
            speed = 0.0
            plan_offset = self.offset
            along = 1.0

        if self.autopilot is not None:
            self.autopilot.release_keep_out(
                self.route.distance_to_conductors(self._point(self.s, plan_offset))
            )
        self.offset = self._respect_keep_out(self.s, plan_offset)
        _, direction = self.route.point_at(self.s)
        travel = direction * along
        return self._point(self.s, self.offset), travel * speed, travel

    def step(self, dt: float) -> VehicleState:
        wind = self.wind.step(dt)
        mode = self.autopilot.mode if self.autopilot is not None else "MISSION"
        # LAND is final for the vehicle too: once down, or going down, it stays.
        if mode != self.mode and self.mode != "LAND":
            self._change_mode(mode)

        if not self.airborne:
            return self._state(dt, ZERO, wind, self.attitude)

        target, feed_forward, travel = self._guidance(dt)
        velocity_cmd = feed_forward + (target - self.position) * self.K_POSITION
        accel = (velocity_cmd - self.velocity) * self.K_VELOCITY
        # The controller limits its tilt, so the commanded horizontal acceleration is capped.
        horizontal = math.hypot(accel.x, accel.y)
        if horizontal > self.MAX_ACCEL_MPS2:
            accel = Vec3(accel.x, accel.y, 0.0) * (self.MAX_ACCEL_MPS2 / horizontal) + Vec3(
                0.0, 0.0, accel.z
            )
        accel = accel + (wind - self.velocity) * self.K_DRAG
        self.velocity = self.velocity + accel * dt
        self.position = self.position + self.velocity * dt

        if self.mode == "LAND" and target.z == 0.0 and self.position.z <= 0.05:
            # Touchdown: the motors stop and the vehicle rests on the ground.
            self.position = Vec3(self.position.x, self.position.y, 0.0)
            self.velocity = ZERO
            self.airborne = False
            self.thrust_axis = Vec3(0.0, 0.0, GRAVITY)
            previous = self.attitude
            self.attitude = self._attitude(self.thrust_axis)
            return self._state(dt, ZERO, wind, previous)

        # The nose follows the direction of travel with a first-order lag (shortest way
        # round); hovering and landing keep the current heading.
        if travel.norm() > 0.0:
            target_yaw = math.atan2(travel.y, travel.x)
            error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            self.yaw += error * (1.0 - math.exp(-dt / self.YAW_TAU_S))

        specific_force = accel + Vec3(0.0, 0.0, GRAVITY)
        # The attitude loop cannot tilt instantly: the thrust axis follows the
        # demanded direction with a lag, which keeps body rates realistic.
        blend = 1.0 - math.exp(-dt / self.ATTITUDE_TAU_S)
        self.thrust_axis = (self.thrust_axis + (specific_force - self.thrust_axis) * blend)
        previous = self.attitude
        self.attitude = self._attitude(self.thrust_axis)
        return self._state(dt, accel, wind, previous)

    def _state(self, dt: float, accel: Vec3, wind: Vec3, previous: Mat3) -> VehicleState:
        """Advances time and reports the state; at rest the accelerometer reads gravity only."""
        self.t += dt
        specific_force = accel + Vec3(0.0, 0.0, GRAVITY)
        return VehicleState(
            t=self.t,
            position=self.position,
            velocity=self.velocity,
            attitude=self.attitude,
            rate_body=body_rates(previous, self.attitude, dt),
            specific_force_body=self.attitude.apply_transpose(specific_force),
            s_m=self.s,
            distance_to_line_m=self.route.distance_to_conductors(self.position),
            wind=wind,
            mode=self.mode,
            airborne=self.airborne,
        )
