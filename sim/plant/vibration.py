"""Motor and propeller vibration as the IMU sees it, with injectable faults.

Each of the four rotors spins at a speed set by the thrust it delivers
(rotor speed grows with the square root of thrust) and carries a small
residual imbalance, so a healthy airframe already vibrates a little. The
imbalance force grows with the square of the rotor speed and rotates in the
rotor plane (body x/y); it also rocks the airframe, which the gyroscope sees
as roll and pitch rates at the rotor frequency.

Two faults can be injected on one rotor:

- ``imbalance``: a damaged (chipped or bent) propeller. The rotating
  imbalance of that rotor grows sharply: a strong tone at the rotor frequency
  on the radial axes, with a weaker twice-per-revolution axial component.
- ``bearing``: a worn motor bearing. Each revolution the damaged race causes
  impacts: broadband, impulsive vibration, strongest along the motor axis
  (body z), plus a tone at the ball-pass frequency.

The rotors turn at about 80 Hz in a hover, well above the 50 Hz Nyquist limit
of Node A's 100 Hz IMU sampling, so the vibration reaches the firmware
aliased. That is modelled exactly here, because the vibration is evaluated at
the sample instants; the classifier learns the aliased signature. A real
implementation would read the sensor's FIFO at its 952 Hz output rate.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from sim.plant.vec import ZERO, Vec3

GRAVITY = 9.80665

KINDS = ("imbalance", "bearing")
CLASSES = ("nominal", *KINDS)       # classifier labels, in output order


@dataclass(frozen=True)
class VibrationFault:
    kind: str                   # "imbalance" or "bearing"
    onset_s: float              # the fault appears suddenly at this time
    severity: float = 1.0       # 0..1 (and somewhat beyond): scales the fault's vibration
    rotor: int = 0

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f"unknown vibration fault {self.kind!r}; expected one of {KINDS}")


@dataclass(frozen=True)
class VibrationSample:
    accel_g: Vec3
    gyro_dps: Vec3
    label: str                  # what the vibration shows: one of CLASSES


class Vibration:
    ROTORS = 4
    HOVER_ROTOR_HZ = 80.0
    SPEED_SPREAD = 0.03         # rotors differ by a few percent (differential thrust)
    SPEED_WANDER = 0.01         # and their speeds wander slowly (control activity)
    SPEED_WANDER_TAU_S = 0.5
    # Healthy residual imbalance per rotor, at hover speed.
    RESIDUAL_G = 0.010
    RESIDUAL_DPS = 0.15
    # Faults at severity 1, at hover speed.
    IMBALANCE_G = 0.25
    IMBALANCE_DPS = 4.0
    BEARING_G = 0.12            # RMS of the impacts, along z; 40 % of it radially
    BEARING_DPS = 1.5
    BALL_PASS_RATIO = 3.57      # outer-race ball-pass frequency / rotor frequency
    BALL_PASS_G = 0.05

    def __init__(
        self,
        fault: VibrationFault | None = None,
        seed: int = 4,
        baseline_scale: float = 1.0,
    ) -> None:
        self.fault = fault
        self.rng = random.Random(seed)
        self.baseline_scale = baseline_scale
        self.t = 0.0
        self.spread = [1.0 + self.rng.uniform(-1.0, 1.0) * self.SPEED_SPREAD
                       for _ in range(self.ROTORS)]
        self.wander = [0.0] * self.ROTORS
        self.phase = [self.rng.uniform(0.0, 2.0 * math.pi) for _ in range(self.ROTORS)]
        self.ball_phase = 0.0
        # Each rotor's imbalance points in its own direction relative to the blade.
        self.residual = [self.baseline_scale * self.rng.uniform(0.5, 1.5)
                         for _ in range(self.ROTORS)]

    def active_fault(self, t: float) -> VibrationFault | None:
        if self.fault is not None and t >= self.fault.onset_s:
            return self.fault
        return None

    def step(self, dt: float, specific_force_body: Vec3, airborne: bool) -> VibrationSample:
        """Advances the rotors by @dt and returns the vibration at the new sample instant."""
        self.t += dt
        if not airborne:
            # Motors stopped: no vibration and nothing to classify.
            return VibrationSample(ZERO, ZERO, "nominal")

        thrust_ratio = max(0.2, specific_force_body.norm() / GRAVITY)
        hover_speed = math.sqrt(thrust_ratio)
        a = math.exp(-dt / self.SPEED_WANDER_TAU_S)
        b = self.SPEED_WANDER * math.sqrt(1.0 - a * a)
        fault = self.active_fault(self.t)

        ax = ay = az = 0.0
        gx = gy = 0.0
        for i in range(self.ROTORS):
            self.wander[i] = a * self.wander[i] + b * self.rng.gauss(0.0, 1.0)
            speed = hover_speed * (self.spread[i] + self.wander[i])  # relative to hover
            self.phase[i] = (self.phase[i] + 2.0 * math.pi * self.HOVER_ROTOR_HZ * speed * dt) % (
                2.0 * math.pi
            )
            force = speed * speed           # imbalance force grows with speed squared
            accel = self.RESIDUAL_G * self.residual[i]
            rate = self.RESIDUAL_DPS * self.residual[i]
            if fault is not None and fault.kind == "imbalance" and fault.rotor == i:
                accel += self.IMBALANCE_G * fault.severity
                rate += self.IMBALANCE_DPS * fault.severity
            c, s = math.cos(self.phase[i]), math.sin(self.phase[i])
            # Rotors alternate their direction of spin.
            spin = 1.0 if i % 2 == 0 else -1.0
            ax += force * accel * c
            ay += force * accel * s * spin
            az += force * accel * 0.2 * math.cos(2.0 * self.phase[i])
            gx += force * rate * s * spin
            gy += force * rate * c

        gz = 0.0
        if fault is not None and fault.kind == "bearing":
            i = fault.rotor
            speed = hover_speed * (self.spread[i] + self.wander[i])
            # An impact once per revolution: noise bursts gated by the rotor's phase.
            gate = max(0.0, math.cos(self.phase[i])) ** 4 * 2.6   # unit RMS on average
            impact = self.BEARING_G * fault.severity * speed * speed
            self.ball_phase = (
                self.ball_phase
                + 2.0 * math.pi * self.HOVER_ROTOR_HZ * speed * self.BALL_PASS_RATIO * dt
            ) % (2.0 * math.pi)
            tone = self.BALL_PASS_G * fault.severity * speed * speed * math.sin(self.ball_phase)
            az += impact * gate * self.rng.gauss(0.0, 1.0) + tone
            ax += 0.4 * impact * gate * self.rng.gauss(0.0, 1.0)
            ay += 0.4 * impact * gate * self.rng.gauss(0.0, 1.0)
            rate = self.BEARING_DPS * fault.severity * speed * speed
            gx += rate * gate * self.rng.gauss(0.0, 1.0)
            gy += rate * gate * self.rng.gauss(0.0, 1.0)
            gz += 0.5 * rate * gate * self.rng.gauss(0.0, 1.0)

        return VibrationSample(
            accel_g=Vec3(ax, ay, az),
            gyro_dps=Vec3(gx, gy, gz),
            label=fault.kind if fault is not None else "nominal",
        )
