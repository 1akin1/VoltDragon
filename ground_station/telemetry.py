"""Decoder for Node B's 60-byte UDP telemetry packet (docs/telemetry.md).

Written independently from the firmware encoder (firmware/common/src/tlm_msg.c);
both are checked against the same reference packet, so a layout change on one
side that is not mirrored on the other fails the tests.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass

PACKET_LEN = 60
VERSION = 1
MAGIC = b"VDTM"
DEFAULT_PORT = 5600

FLAG_NODE_A_FRESH = 0x01
FLAG_IMU_VALID = 0x02
FLAG_RECORDER_OK = 0x04

# magic, version, flags, length, seq, B uptime, A uptime, A resets, B resets, A age,
# accel[3], gyro[3], mag[3], reserved, CAN valid, rejected, lost, CRC-32
_LAYOUT = struct.Struct("<4sBBHIIIBBH3h3h3hHIIII")
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
    can_valid: int
    can_rejected: int
    can_lost: int

    @property
    def node_a_fresh(self) -> bool:
        return bool(self.flags & FLAG_NODE_A_FRESH)

    @property
    def imu_valid(self) -> bool:
        return bool(self.flags & FLAG_IMU_VALID)

    @property
    def recorder_ok(self) -> bool:
        return bool(self.flags & FLAG_RECORDER_OK)


def decode(datagram: bytes) -> Telemetry:
    """Parses and checks a telemetry datagram."""
    if len(datagram) != PACKET_LEN:
        raise TelemetryError(f"length {len(datagram)}, expected {PACKET_LEN}")

    fields = _LAYOUT.unpack(datagram)
    magic, version, flags, length, seq, b_up, a_up, a_rst, b_rst, a_age = fields[:10]
    accel, gyro, mag = fields[10:13], fields[13:16], fields[16:19]
    can_valid, can_rejected, can_lost, crc = fields[20:24]

    if magic != MAGIC:
        raise TelemetryError(f"bad magic {magic!r}")
    if version != VERSION:
        raise TelemetryError(f"unsupported version {version}")
    if length != PACKET_LEN:
        raise TelemetryError(f"length field {length}, expected {PACKET_LEN}")
    if zlib.crc32(datagram[:-4]) != crc:
        raise TelemetryError("CRC mismatch")

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
        can_valid=can_valid,
        can_rejected=can_rejected,
        can_lost=can_lost,
    )


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
