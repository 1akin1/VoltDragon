"""Magnetic fields: the Earth's field and the field of the three-phase line current.

Units: gauss (1 G = 100 uT), the unit the LSM9DS1 magnetometer reports.

Near a loaded high-voltage line the conductors' field is a sizeable fraction of
the Earth's field, so a magnetometer there no longer gives a usable heading
(HLR-008). The field of each conductor segment follows from the Biot-Savart law
for a finite straight wire; the three phase currents are 120 degrees apart, so
far from the line their fields largely cancel, but close to one conductor its
own field dominates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from sim.plant.route import Route, Segment
from sim.plant.vec import ZERO, Vec3

MU0_OVER_4PI = 1e-7         # T m / A
TESLA_TO_GAUSS = 1e4


@dataclass(frozen=True)
class EarthField:
    """A uniform geomagnetic field, given the way magnetic models report it."""

    total_gauss: float = 0.500       # about 50 uT, typical for central Anatolia
    inclination_deg: float = 56.0    # dip below the horizontal
    declination_deg: float = 5.0     # east of true north

    def enu(self) -> Vec3:
        horizontal = self.total_gauss * math.cos(math.radians(self.inclination_deg))
        down = self.total_gauss * math.sin(math.radians(self.inclination_deg))
        dec = math.radians(self.declination_deg)
        return Vec3(horizontal * math.sin(dec), horizontal * math.cos(dec), -down)


def segment_field_tesla(segment: Segment, current_a: float, p: Vec3) -> Vec3:
    """Field at p of a straight wire carrying current_a from segment.a to segment.b."""
    d = segment.b - segment.a
    r = p - _foot_on_line(segment, p)
    dist = r.norm()
    if dist < 1e-6:
        return ZERO
    u = d.unit()
    # Angles from each end, measured along the wire direction.
    cos1 = (p - segment.a).unit().dot(u)
    cos2 = (p - segment.b).unit().dot(u)
    magnitude = MU0_OVER_4PI * current_a / dist * (cos1 - cos2)
    return u.cross(r.unit()) * magnitude


def _foot_on_line(segment: Segment, p: Vec3) -> Vec3:
    """Foot of the perpendicular from p onto the (infinite) line through the segment."""
    d = segment.b - segment.a
    t = (p - segment.a).dot(d) / d.dot(d)
    return segment.a + d * t


def line_field_gauss(route: Route, p: Vec3, t: float) -> Vec3:
    """Instantaneous field of the three phase currents at position p and time t."""
    total = ZERO
    omega = 2.0 * math.pi * route.frequency_hz
    peak = route.current_a * math.sqrt(2.0)
    for k, phase in enumerate(route.conductors):
        current = peak * math.sin(omega * t - k * 2.0 * math.pi / 3.0)
        for segment in phase:
            total = total + segment_field_tesla(segment, current, p)
    return total * TESLA_TO_GAUSS


def line_field_peak_gauss(route: Route, p: Vec3, samples: int = 24) -> float:
    """Largest field magnitude over one mains cycle at p."""
    period = 1.0 / route.frequency_hz
    return max(line_field_gauss(route, p, period * k / samples).norm() for k in range(samples))
