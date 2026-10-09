"""Minimal 3-D vector and rotation helpers (pure Python, no dependencies)."""

from __future__ import annotations

import math
from typing import NamedTuple


class Vec3(NamedTuple):
    x: float
    y: float
    z: float

    def __add__(self, o: Vec3) -> Vec3:  # type: ignore[override]
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o: Vec3) -> Vec3:
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, k: float) -> Vec3:  # type: ignore[override]
        return Vec3(self.x * k, self.y * k, self.z * k)

    __rmul__ = __mul__

    def __neg__(self) -> Vec3:
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, o: Vec3) -> float:
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, o: Vec3) -> Vec3:
        return Vec3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def unit(self) -> Vec3:
        n = self.norm()
        return self * (1.0 / n) if n > 0.0 else Vec3(0.0, 0.0, 0.0)


ZERO = Vec3(0.0, 0.0, 0.0)


class Mat3(NamedTuple):
    """3x3 matrix stored by columns: c0, c1, c2."""

    c0: Vec3
    c1: Vec3
    c2: Vec3

    @staticmethod
    def from_columns(c0: Vec3, c1: Vec3, c2: Vec3) -> Mat3:
        return Mat3(c0, c1, c2)

    def apply(self, v: Vec3) -> Vec3:
        """Returns M v."""
        return self.c0 * v.x + self.c1 * v.y + self.c2 * v.z

    def apply_transpose(self, v: Vec3) -> Vec3:
        """Returns M^T v."""
        return Vec3(self.c0.dot(v), self.c1.dot(v), self.c2.dot(v))

    def transpose_times(self, o: Mat3) -> Mat3:
        """Returns M^T O."""
        return Mat3(
            self.apply_transpose(o.c0), self.apply_transpose(o.c1), self.apply_transpose(o.c2)
        )


def body_rates(r_prev: Mat3, r_now: Mat3, dt: float) -> Vec3:
    """Angular rate in body axes from two body-to-world rotations dt apart.

    Uses the skew-symmetric part of R_prev^T R_now, which is exact to first order.
    """
    d = r_prev.transpose_times(r_now)
    # d - d^T = 2 [w]x dt for a small rotation; columns are d.c0 .. d.c2.
    wx = (d.c1.z - d.c2.y) / 2.0
    wy = (d.c2.x - d.c0.z) / 2.0
    wz = (d.c0.y - d.c1.x) / 2.0
    return Vec3(wx, wy, wz) * (1.0 / dt)
