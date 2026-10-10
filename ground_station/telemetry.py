"""Decoder for Node B's 88-byte UDP telemetry packet, version 4 (docs/telemetry.md).

Written independently from the firmware encoder (firmware/common/src/tlm_msg.c);
both are checked against the same reference packet, so a layout change on one
side that is not mirrored on the other fails the tests.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass

PACKET_LEN = 88
VERSION = 4
MAGIC = b"VDTM"
DEFAULT_PORT = 5600

FLAG_NODE_A_FRESH = 0x01
FLAG_IMU_VALID = 0x02
FLAG_RECORDER_OK = 0x04
FLAG_GPS_FIX = 0x08
FLAG_MAG_OK = 0x10
HEADING_SOURCE_SHIFT = 5
HEADING_SOURCE_MASK = 0x60
HEADING_SOURCES = ("none", "magnetometer", "gps")
FLAG_VIB_ACTIVE = 0x80

# Node A's safety flags, forwarded unchanged from the CAN SAFETY message.
SAFETY_PROXIMITY = 0x01
SAFETY_AVOIDING = 0x02
SAFETY_LINK_LOST = 0x04
SAFETY_BATTERY_LOW = 0x08
SAFETY_BATTERY_CRITICAL = 0x10
SAFETY_AUTOPILOT_OK = 0x20
SAFETY_VIBRATION = 0x40

# Node A's vibration classes (HLR-009), in the classifier's output order.
VIBRATION_CLASSES = ("nominal", "imbalance", "bearing")

FLIGHT_MODES = ("MISSION", "HOLD", "RETURN_TO_HOME", "LAND")
UNKNOWN_U8 = 0xFF
UNKNOWN_U16 = 0xFFFF

# magic, version, flags, length, seq, B uptime, A uptime, A resets, B resets, A age,
# accel[3], gyro[3], mag[3], GPS satellites, GPS quality, CAN valid, rejected, lost,
# latitude, longitude, altitude, heading, field, speed, GPS age, distance, battery,
# flight mode, safety flags, last request id, ground link age, vibration alarm,
# vibration fault score, CRC-32
_LAYOUT = struct.Struct("<4sBBHIIIBBH3h3h3hBBIIIiihHHHHHBBBBHBBI")
assert _LAYOUT.size == PACKET_LEN

_GYRO_UNIT_MDPS = 10


class TelemetryError(ValueError):
    """Raised for a datagram that is not a valid telemetry packet."""


@dataclass(frozen=True)
class Telemetry:
    seq: int
    flags: int
    node_b_uptime_ms: int
    node_a_uptime_ms: int
    node_a_resets: int
    node_b_resets: int
    node_a_age_ms: int
    accel_mg: tuple[int, int, int]
    gyro_mdps: tuple[int, int, int]
    mag_mgauss: tuple[int, int, int]
    gps_satellites: int
    gps_quality: int
    can_valid: int
    can_rejected: int
    can_lost: int
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    heading_deg: float
    field_mgauss: int
    speed_mps: float
    gps_age_ms: int
    distance_m: float | None
    battery_pct: int | None
    flight_mode: str | None
    safety_flags: int
    last_request_id: int
    ground_link_age_ms: int
    vibration_alarm: str | None     # VIBRATION_CLASSES; None before Node A reports it
    fault_score_pct: int | None     # 100 minus P(nominal) of the latest window

    @property
    def node_a_fresh(self) -> bool:
        return bool(self.flags & FLAG_NODE_A_FRESH)

    @property
    def imu_valid(self) -> bool:
        return bool(self.flags & FLAG_IMU_VALID)

    @property
    def recorder_ok(self) -> bool:
        return bool(self.flags & FLAG_RECORDER_OK)

    @property
    def gps_fix(self) -> bool:
        return bool(self.flags & FLAG_GPS_FIX)

    @property
    def mag_ok(self) -> bool:
        return bool(self.flags & FLAG_MAG_OK)

    @property
    def heading_source(self) -> str:
        index = (self.flags & HEADING_SOURCE_MASK) >> HEADING_SOURCE_SHIFT
        return HEADING_SOURCES[index] if index < len(HEADING_SOURCES) else "unknown"

    @property
    def proximity(self) -> bool:
        return bool(self.safety_flags & SAFETY_PROXIMITY)

    @property
    def link_lost(self) -> bool:
        return bool(self.safety_flags & SAFETY_LINK_LOST)

    @property
    def battery_low(self) -> bool:
        return bool(self.safety_flags & SAFETY_BATTERY_LOW)

    @property
    def battery_critical(self) -> bool:
        return bool(self.safety_flags & SAFETY_BATTERY_CRITICAL)

    @property
    def autopilot_ok(self) -> bool:
        return bool(self.safety_flags & SAFETY_AUTOPILOT_OK)

    @property
    def vibration_monitor_active(self) -> bool:
        return bool(self.flags & FLAG_VIB_ACTIVE)

    @property
    def vibration_fault(self) -> bool:
        return bool(self.safety_flags & SAFETY_VIBRATION)


def decode(datagram: bytes) -> Telemetry:
    """Parses and checks a telemetry datagram."""
    if len(datagram) != PACKET_LEN:
        raise TelemetryError(f"length {len(datagram)}, expected {PACKET_LEN}")

    f = _LAYOUT.unpack(datagram)
    magic, version, flags, length = f[0:4]
    if magic != MAGIC:
        raise TelemetryError(f"bad magic {magic!r}")
    if version != VERSION:
        raise TelemetryError(f"unsupported version {version}")
    if length != PACKET_LEN:
        raise TelemetryError(f"length field {length}, expected {PACKET_LEN}")
    if zlib.crc32(datagram[:-4]) != f[-1]:
        raise TelemetryError("CRC mismatch")

    seq, b_up, a_up, a_rst, b_rst, a_age = f[4:10]
    accel, gyro, mag = f[10:13], f[13:16], f[16:19]
    sats, quality, can_valid, can_rejected, can_lost = f[19:24]
    lat, lon, alt_dm, heading, field, speed_dm, gps_age = f[24:31]
    distance_dm, battery, mode, safety_flags, request_id, link_age = f[31:37]
    vib_alarm, fault_score = f[37:39]

    return Telemetry(
        seq=seq,
        flags=flags,
        node_b_uptime_ms=b_up,
        node_a_uptime_ms=a_up,
        node_a_resets=a_rst,
        node_b_resets=b_rst,
        node_a_age_ms=a_age,
        accel_mg=accel,
        gyro_mdps=tuple(v * _GYRO_UNIT_MDPS for v in gyro),
        mag_mgauss=mag,
        gps_satellites=sats,
        gps_quality=quality,
        can_valid=can_valid,
        can_rejected=can_rejected,
        can_lost=can_lost,
        lat_deg=lat / 1e7,
        lon_deg=lon / 1e7,
        alt_msl_m=alt_dm / 10.0,
        heading_deg=heading / 100.0,
        field_mgauss=field,
        speed_mps=speed_dm / 10.0,
        gps_age_ms=gps_age,
        distance_m=None if distance_dm == UNKNOWN_U16 else distance_dm / 10.0,
        battery_pct=None if battery == UNKNOWN_U8 else battery,
        flight_mode=FLIGHT_MODES[mode] if mode < len(FLIGHT_MODES) else None,
        safety_flags=safety_flags,
        last_request_id=request_id,
        ground_link_age_ms=link_age,
        vibration_alarm=VIBRATION_CLASSES[vib_alarm] if vib_alarm < len(VIBRATION_CLASSES)
        else None,
        fault_score_pct=None if fault_score == UNKNOWN_U8 else fault_score,
    )


def encode(t: Telemetry) -> bytes:
    """Builds a packet; used by tests and tools that stand in for Node B."""
    body = _LAYOUT.pack(
        MAGIC, VERSION, t.flags, PACKET_LEN, t.seq, t.node_b_uptime_ms, t.node_a_uptime_ms,
        t.node_a_resets, t.node_b_resets, t.node_a_age_ms,
        *t.accel_mg, *(v // _GYRO_UNIT_MDPS for v in t.gyro_mdps), *t.mag_mgauss,
        t.gps_satellites, t.gps_quality, t.can_valid, t.can_rejected, t.can_lost,
        round(t.lat_deg * 1e7), round(t.lon_deg * 1e7), round(t.alt_msl_m * 10.0),
        round(t.heading_deg * 100.0), t.field_mgauss, round(t.speed_mps * 10.0), t.gps_age_ms,
        UNKNOWN_U16 if t.distance_m is None else round(t.distance_m * 10.0),
        UNKNOWN_U8 if t.battery_pct is None else t.battery_pct,
        UNKNOWN_U8 if t.flight_mode is None else FLIGHT_MODES.index(t.flight_mode),
        t.safety_flags, t.last_request_id, t.ground_link_age_ms,
        UNKNOWN_U8 if t.vibration_alarm is None else VIBRATION_CLASSES.index(t.vibration_alarm),
        UNKNOWN_U8 if t.fault_score_pct is None else t.fault_score_pct,
        0,
    )[:-4]
    return body + struct.pack("<I", zlib.crc32(body))


class SequenceMonitor:
    """Counts telemetry packets lost between received ones, from the sequence number."""

    def __init__(self) -> None:
        self.expected: int | None = None
        self.received = 0
        self.lost = 0

    def update(self, seq: int) -> int:
        """Records a packet and returns how many were lost just before it."""
        missed = 0
        if self.expected is not None and seq > self.expected:
            missed = seq - self.expected
        # A sequence number going backwards means Node B restarted: resynchronise.
        self.expected = seq + 1
        self.received += 1
        self.lost += missed
        return missed
