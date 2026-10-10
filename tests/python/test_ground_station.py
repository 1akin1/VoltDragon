"""Tests for the ground station's state, alarms and display."""

import dataclasses
import struct

import pytest
from packets import REFERENCE

from ground_station.state import GroundState
from ground_station.telemetry import Telemetry, decode, encode
from sim.plant.route import Route
from sim.plant.vec import Vec3

ROUTE = Route.load("line_a")
ALL_OK = 0x01 | 0x02 | 0x04 | 0x08 | 0x10 | (1 << 5) | 0x80


def packet(seq: int, offset_m: float = 20.0, flags: int = ALL_OK, **changes) -> Telemetry:
    """A telemetry packet for a vehicle 150 m along the route, offset_m right of its centre."""
    ground, direction = ROUTE.point_at(150.0)
    right = Vec3(direction.y, -direction.x, 0.0)
    p = ground + right * offset_m + Vec3(0.0, 0.0, ROUTE.conductor_height_m)
    lat, lon, alt = ROUTE.enu_to_geodetic(p)
    base = dataclasses.replace(
        decode(REFERENCE), seq=seq, flags=flags, lat_deg=lat, lon_deg=lon, alt_msl_m=alt
    )
    return dataclasses.replace(base, **changes)


def test_encode_matches_reference() -> None:
    assert encode(decode(REFERENCE)) == REFERENCE


def test_geodetic_round_trip() -> None:
    p = Vec3(812.5, -97.25, 31.0)
    q = ROUTE.geodetic_to_enu(*ROUTE.enu_to_geodetic(p))
    assert q.x == pytest.approx(p.x, abs=1e-6)
    assert q.y == pytest.approx(p.y, abs=1e-6)
    assert q.z == pytest.approx(p.z, abs=1e-9)


def test_quiet_flight_has_no_alarms() -> None:
    state = GroundState(ROUTE)
    state.update(packet(1), now=10.0)
    assert state.distance_m() == pytest.approx(14.0, abs=0.01)
    assert state.alarms(now=10.1) == []


def test_proximity_alarms_follow_distance_to_conductor() -> None:
    state = GroundState(ROUTE)
    state.update(packet(1, offset_m=17.0), now=1.0)        # 11 m from the outer phase
    assert [a.name for a in state.alarms(1.0)] == ["PROXIMITY"]
    state.update(packet(2, offset_m=12.0), now=1.1)        # 6 m
    assert [a.name for a in state.alarms(1.1)] == ["TOO CLOSE TO LINE"]


def test_link_loss_and_packet_loss() -> None:
    state = GroundState(ROUTE)
    state.update(packet(1), now=1.0)
    state.update(packet(5), now=1.1)
    names = [a.name for a in state.alarms(1.2)]
    assert "PACKET LOSS" in names and state.lost == 3
    assert [a.name for a in state.alarms(3.0)][0] == "LINK LOST"
    # The loss alarm clears after a quiet period.
    state.update(packet(6), now=10.0)
    assert state.alarms(10.0) == []


def test_status_flags_raise_alarms() -> None:
    state = GroundState(ROUTE)
    disturbed = ALL_OK & ~0x10 & ~0x60 | (2 << 5)          # magnetometer disturbed, GPS course
    state.update(packet(1, flags=disturbed), now=1.0)
    alarms = {a.name: a for a in state.alarms(1.0)}
    assert "MAGNETOMETER DISTURBED" in alarms
    assert "gps" in alarms["MAGNETOMETER DISTURBED"].detail
    state.update(packet(2, flags=ALL_OK & ~0x01 & ~0x08, gps_age_ms=1500), now=1.1)
    names = [a.name for a in state.alarms(1.1)]
    assert names[0] == "NODE A DATA STALE"          # critical first
    assert "GPS NO FIX" in names


def test_onboard_safety_state_raises_alarms() -> None:
    state = GroundState(ROUTE)
    state.update(
        packet(1, flight_mode="RETURN_TO_HOME", battery_pct=18, safety_flags=0x20 | 0x08 | 0x04),
        now=1.0,
    )
    alarms = {a.name: a.severity for a in state.alarms(1.0)}
    assert alarms == {
        "RETURN TO HOME": "critical",
        "VEHICLE LOST COMMAND LINK": "critical",
        "BATTERY LOW": "warning",
    }
    state.update(packet(2, battery_pct=7, safety_flags=0x08 | 0x10 | 0x01 | 0x02), now=1.1)
    names = [a.name for a in state.alarms(1.1)]
    assert "BATTERY CRITICAL" in names and "BATTERY LOW" not in names
    assert "AUTOPILOT SILENT" in names and "AVOIDING" in names


def test_vibration_fault_and_inactive_monitor_raise_alarms() -> None:
    state = GroundState(ROUTE)
    state.update(
        packet(1, safety_flags=0x20 | 0x40, vibration_alarm="imbalance", fault_score_pct=98),
        now=1.0,
    )
    alarms = {a.name: a for a in state.alarms(1.0)}
    assert alarms["VIBRATION FAULT"].severity == "critical"
    assert alarms["VIBRATION FAULT"].detail == "damaged propeller, fault score 98 %"
    assert state.samples[-1].fault_score_pct == 98
    state.update(packet(2, flags=ALL_OK & ~0x80), now=1.1)
    assert [a.name for a in state.alarms(1.1)] == ["VIBRATION MONITOR INACTIVE"]


def test_missing_safety_report_is_flagged() -> None:
    state = GroundState(ROUTE)
    state.update(packet(1, flight_mode=None), now=1.0)
    assert [a.name for a in state.alarms(1.0)] == ["SAFETY STATE UNKNOWN"]


def test_can_errors_alarm_when_counters_grow() -> None:
    state = GroundState(ROUTE)
    state.update(packet(1, can_lost=2), now=1.0)
    state.update(packet(2, can_lost=3), now=1.1)
    assert "CAN ERRORS" in [a.name for a in state.alarms(1.1)]


def test_display_renders_a_replay(tmp_path) -> None:
    pytest.importorskip("matplotlib")
    from ground_station import display

    recording = tmp_path / "flight.tlm"
    with recording.open("wb") as f:
        for k in range(50):
            datagram = encode(packet(k, offset_m=20.0 - 0.2 * k))
            f.write(struct.pack("<dH", 0.1 * k, len(datagram)) + datagram)
    image = tmp_path / "view.png"
    assert display.main(["--replay", str(recording), "--snapshot", str(image)]) == 0
    assert image.stat().st_size > 10_000
