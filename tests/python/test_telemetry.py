"""Tests for the ground station's telemetry decoder against the reference packet."""

import pytest

from ground_station.telemetry import (
    PACKET_LEN,
    SequenceMonitor,
    TelemetryError,
    decode,
)

# Reference packet from docs/telemetry.md; the firmware encoder is tested against it too.
REFERENCE = bytes.fromhex(
    "5644544d01073c000700000040e20100c0d40100010003000cfefa00e7031a04"
    "fcd600001f018dff35fe0000e80300000100000002000000bc7cc438"
)


def test_decodes_reference_packet() -> None:
    t = decode(REFERENCE)
    assert t.seq == 7
    assert t.node_a_fresh and t.imu_valid and t.recorder_ok
    assert t.node_b_uptime_ms == 123456
    assert t.node_a_uptime_ms == 120000
    assert (t.node_a_resets, t.node_b_resets, t.node_a_age_ms) == (1, 0, 3)
    assert t.accel_mg == (-500, 250, 999)
    assert t.gyro_mdps == (10500, -105000, 0)
    assert t.mag_mgauss == (287, -115, -459)
    assert (t.can_valid, t.can_rejected, t.can_lost) == (1000, 1, 2)


@pytest.mark.parametrize("index", range(PACKET_LEN))
def test_rejects_corrupted_byte(index: int) -> None:
    corrupted = bytearray(REFERENCE)
    corrupted[index] ^= 0x01
    with pytest.raises(TelemetryError):
        decode(bytes(corrupted))


@pytest.mark.parametrize("length", [0, PACKET_LEN - 1, PACKET_LEN + 1])
def test_rejects_wrong_length(length: int) -> None:
    with pytest.raises(TelemetryError):
        decode((REFERENCE * 2)[:length])


def test_sequence_monitor_counts_gaps_and_resynchronises() -> None:
    m = SequenceMonitor()
    assert m.update(10) == 0
    assert m.update(11) == 0
    assert m.update(14) == 2
    assert m.update(0) == 0  # Node B restarted
    assert m.update(1) == 0
    assert (m.received, m.lost) == (5, 2)
