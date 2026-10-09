"""Tests for the ground station's telemetry decoder against the reference packet."""

from dataclasses import replace

import pytest
from packets import REFERENCE

from ground_station.telemetry import (
    PACKET_LEN,
    SequenceMonitor,
    TelemetryError,
    decode,
    encode,
)


def test_decodes_reference_packet() -> None:
    t = decode(REFERENCE)
    assert t.seq == 7
    assert t.node_a_fresh and t.imu_valid and t.recorder_ok and t.gps_fix and t.mag_ok
    assert t.heading_source == "magnetometer"
    assert t.node_b_uptime_ms == 123456
    assert t.node_a_uptime_ms == 120000
    assert (t.node_a_resets, t.node_b_resets, t.node_a_age_ms) == (1, 0, 3)
    assert t.accel_mg == (-500, 250, 999)
    assert t.gyro_mdps == (10500, -105000, 0)
    assert t.mag_mgauss == (287, -115, -459)
    assert (t.gps_satellites, t.gps_quality) == (9, 1)
    assert (t.can_valid, t.can_rejected, t.can_lost) == (1000, 1, 2)
    assert t.lat_deg == pytest.approx(39.9001234)
    assert t.lon_deg == pytest.approx(32.8005678)
    assert t.alt_msl_m == pytest.approx(925.0)
    assert t.heading_deg == pytest.approx(60.12)
    assert t.field_mgauss == 520
    assert t.speed_mps == pytest.approx(6.0)
    assert t.gps_age_ms == 150
    assert t.distance_m == pytest.approx(18.3)
    assert t.battery_pct == 76
    assert t.flight_mode == "MISSION"
    assert t.autopilot_ok and not (t.proximity or t.link_lost or t.battery_low)
    assert (t.last_request_id, t.ground_link_age_ms) == (3, 420)


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


def test_unknown_safety_values_decode_as_none() -> None:
    t = replace(decode(REFERENCE), distance_m=None, battery_pct=None, flight_mode=None)
    again = decode(encode(t))
    assert (again.distance_m, again.battery_pct, again.flight_mode) == (None, None, None)


def test_sequence_monitor_counts_gaps_and_resynchronises() -> None:
    m = SequenceMonitor()
    assert m.update(10) == 0
    assert m.update(11) == 0
    assert m.update(14) == 2
    assert m.update(0) == 0  # Node B restarted
    assert m.update(1) == 0
    assert (m.received, m.lost) == (5, 2)
