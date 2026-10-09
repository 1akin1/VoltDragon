"""Power-line route: pylons, phase conductors and along-track geometry.

Coordinates are local East-North-Up (ENU) metres, origin at the first pylon's
foot. The conductors are modelled as straight segments between attachment
points (no sag), three phases side by side at the same height.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from sim.plant.vec import Vec3

ROUTES_DIR = Path(__file__).with_name("routes")
EARTH_RADIUS_M = 6378137.0


@dataclass(frozen=True)
class Segment:
    a: Vec3
    b: Vec3

    def closest_point(self, p: Vec3) -> Vec3:
        d = self.b - self.a
        length2 = d.dot(d)
        t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, (p - self.a).dot(d) / length2))
        return self.a + d * t

    def distance(self, p: Vec3) -> float:
        return (p - self.closest_point(p)).norm()


@dataclass(frozen=True)
class Route:
    """A line of pylons with three phase conductors."""

    name: str
    pylons: tuple[tuple[float, float], ...]   # (east, north) of each pylon foot
    conductor_height_m: float
    phase_spacing_m: float                     # horizontal distance between adjacent phases
    current_a: float                           # RMS current per phase
    frequency_hz: float = 50.0
    origin_lat_deg: float = 39.9000
    origin_lon_deg: float = 32.8000
    origin_alt_m: float = 900.0                # ground altitude above mean sea level
    conductors: tuple[tuple[Segment, ...], ...] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        phases: list[list[Segment]] = [[], [], []]
        for (e0, n0), (e1, n1) in zip(self.pylons, self.pylons[1:], strict=False):
            # Offset each phase sideways from the span's centre line.
            span = Vec3(e1 - e0, n1 - n0, 0.0).unit()
            side = Vec3(span.y, -span.x, 0.0)  # to the right of the direction of travel
            for k, offset in enumerate((-self.phase_spacing_m, 0.0, self.phase_spacing_m)):
                shift = side * offset + Vec3(0.0, 0.0, self.conductor_height_m)
                phases[k].append(Segment(Vec3(e0, n0, 0.0) + shift, Vec3(e1, n1, 0.0) + shift))
        object.__setattr__(self, "conductors", tuple(tuple(p) for p in phases))

    @staticmethod
    def load(name: str) -> Route:
        data = json.loads((ROUTES_DIR / f"{name}.json").read_text(encoding="utf-8"))
        data["pylons"] = tuple(tuple(p) for p in data["pylons"])
        return Route(**data)

    def spans(self) -> list[Segment]:
        """Centre lines of the spans at ground level."""
        return [
            Segment(Vec3(e0, n0, 0.0), Vec3(e1, n1, 0.0))
            for (e0, n0), (e1, n1) in zip(self.pylons, self.pylons[1:], strict=False)
        ]

    def length(self) -> float:
        return sum((s.b - s.a).norm() for s in self.spans())

    def point_at(self, s: float) -> tuple[Vec3, Vec3]:
        """Ground point and unit direction at along-track distance s (clamped to the route)."""
        remaining = max(0.0, s)
        spans = self.spans()
        for span in spans:
            d = span.b - span.a
            length = d.norm()
            if remaining <= length:
                return span.a + d * (remaining / length), d.unit()
            remaining -= length
        last = spans[-1]
        return last.b, (last.b - last.a).unit()

    def distance_to_conductors(self, p: Vec3) -> float:
        """Shortest distance from p to any phase conductor."""
        return min(seg.distance(p) for phase in self.conductors for seg in phase)

    def enu_to_geodetic(self, p: Vec3) -> tuple[float, float, float]:
        """Latitude and longitude in degrees, altitude above mean sea level in metres.

        Flat-Earth approximation, accurate to centimetres over a few kilometres.
        """
        lat = self.origin_lat_deg + math.degrees(p.y / EARTH_RADIUS_M)
        lon = self.origin_lon_deg + math.degrees(
            p.x / (EARTH_RADIUS_M * math.cos(math.radians(self.origin_lat_deg)))
        )
        return lat, lon, self.origin_alt_m + p.z

    def geodetic_to_enu(self, lat_deg: float, lon_deg: float, alt_msl_m: float) -> Vec3:
        """Inverse of enu_to_geodetic."""
        north = math.radians(lat_deg - self.origin_lat_deg) * EARTH_RADIUS_M
        east = (
            math.radians(lon_deg - self.origin_lon_deg)
            * EARTH_RADIUS_M
            * math.cos(math.radians(self.origin_lat_deg))
        )
        return Vec3(east, north, alt_msl_m - self.origin_alt_m)
