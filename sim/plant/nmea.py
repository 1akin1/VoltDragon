"""NMEA 0183 sentence generation for the simulated GPS receiver (GGA and RMC)."""

from __future__ import annotations

import functools
from datetime import datetime

KNOTS_PER_MPS = 1.0 / 0.514444


def checksum(body: str) -> int:
    """XOR of every character between '$' and '*'."""
    return functools.reduce(lambda acc, ch: acc ^ ord(ch), body, 0)


def sentence(body: str) -> str:
    return f"${body}*{checksum(body):02X}\r\n"


def _lat(deg: float) -> tuple[str, str]:
    hemisphere = "N" if deg >= 0.0 else "S"
    deg = abs(deg)
    whole = int(deg)
    return f"{whole:02d}{(deg - whole) * 60.0:08.5f}", hemisphere


def _lon(deg: float) -> tuple[str, str]:
    hemisphere = "E" if deg >= 0.0 else "W"
    deg = abs(deg)
    whole = int(deg)
    return f"{whole:03d}{(deg - whole) * 60.0:08.5f}", hemisphere


def _time(utc: datetime) -> str:
    return utc.strftime("%H%M%S.") + f"{utc.microsecond // 10000:02d}"


def gga(utc: datetime, lat: float, lon: float, alt_msl: float, satellites: int, hdop: float) -> str:
    """Fix data: time, position, quality 1 (GPS fix), satellites, HDOP, altitude."""
    la, ns = _lat(lat)
    lo, ew = _lon(lon)
    body = (
        f"GPGGA,{_time(utc)},{la},{ns},{lo},{ew},1,{satellites:02d},{hdop:.1f},"
        f"{alt_msl:.1f},M,36.0,M,,"
    )
    return sentence(body)


def rmc(utc: datetime, lat: float, lon: float, speed_mps: float, course_deg: float) -> str:
    """Recommended minimum data: time, status A (valid), position, speed, course, date."""
    la, ns = _lat(lat)
    lo, ew = _lon(lon)
    # Round before wrapping, so 359.99 becomes 0.0 rather than an invalid 360.0.
    course = round(course_deg % 360.0, 1) % 360.0
    body = (
        f"GPRMC,{_time(utc)},A,{la},{ns},{lo},{ew},{speed_mps * KNOTS_PER_MPS:.2f},"
        f"{course:.1f},{utc.strftime('%d%m%y')},,,A"
    )
    return sentence(body)
