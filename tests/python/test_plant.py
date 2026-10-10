"""Tests for the plant model: geometry, magnetic fields, flight, sensors and vibration."""

import math
import re

import pytest

from sim.plant import nmea
from sim.plant.magnetics import EarthField, line_field_peak_gauss, segment_field_tesla
from sim.plant.plant import STEP_S, Plant, Scenario
from sim.plant.route import Route, Segment
from sim.plant.vec import Mat3, Vec3, body_rates
from sim.plant.vibration import Vibration, VibrationFault

ROUTE = Route.load("line_a")


def test_route_geometry() -> None:
    assert len(ROUTE.pylons) == 6
    assert ROUTE.length() == pytest.approx(sum(
        math.dist(a, b) for a, b in zip(ROUTE.pylons, ROUTE.pylons[1:], strict=False)
    ))
    ground, direction = ROUTE.point_at(0.0)
    assert ground == Vec3(0.0, 0.0, 0.0)
    assert direction.norm() == pytest.approx(1.0)


def test_distance_to_conductors() -> None:
    # Mid-span, at conductor height, 10 m to the right of the centre line: 4 m from the outer phase.
    ground, direction = ROUTE.point_at(150.0)
    right = Vec3(direction.y, -direction.x, 0.0)
    p = ground + right * 10.0 + Vec3(0.0, 0.0, ROUTE.conductor_height_m)
    assert ROUTE.distance_to_conductors(p) == pytest.approx(4.0, abs=1e-6)


def test_long_wire_field_matches_infinite_wire() -> None:
    wire = Segment(Vec3(-1e5, 0.0, 0.0), Vec3(1e5, 0.0, 0.0))     # current along +x
    field = segment_field_tesla(wire, 100.0, Vec3(0.0, 0.0, 2.0))  # 2 m above
    assert field.norm() == pytest.approx(2e-7 * 100.0 / 2.0, rel=1e-6)
    # Right-hand rule: above a wire carrying current along +x, the field points along -y.
    assert field.unit().y == pytest.approx(-1.0)


def test_earth_field_components() -> None:
    earth = EarthField(total_gauss=0.5, inclination_deg=56.0, declination_deg=0.0)
    b = earth.enu()
    assert b.norm() == pytest.approx(0.5)
    assert math.degrees(math.atan2(-b.z, b.y)) == pytest.approx(56.0)
    assert b.x == pytest.approx(0.0, abs=1e-12)


def test_line_field_falls_off_with_distance() -> None:
    ground, direction = ROUTE.point_at(150.0)
    right = Vec3(direction.y, -direction.x, 0.0)
    up = Vec3(0.0, 0.0, ROUTE.conductor_height_m)
    near = line_field_peak_gauss(ROUTE, ground + right * 10.0 + up)
    far = line_field_peak_gauss(ROUTE, ground + right * 20.0 + up)
    assert near > 3.0 * far
    assert 0.1 < near < 0.3  # 4 m from a 400 A conductor


def test_body_rates_from_rotation() -> None:
    def yaw(angle: float) -> Mat3:
        c, s = math.cos(angle), math.sin(angle)
        return Mat3(Vec3(c, s, 0.0), Vec3(-s, c, 0.0), Vec3(0.0, 0.0, 1.0))

    rate = body_rates(yaw(0.1), yaw(0.1 + 0.002), 0.01)
    assert rate.z == pytest.approx(0.2, rel=1e-3)
    assert abs(rate.x) < 1e-9 and abs(rate.y) < 1e-9


def test_nmea_sentences() -> None:
    from datetime import UTC, datetime

    utc = datetime(2026, 10, 9, 10, 0, 1, 500000, tzinfo=UTC)
    gga = nmea.gga(utc, 39.9, -32.8, 925.0, 9, 0.9)
    assert gga.startswith("$GPGGA,100001.50,3954.00000,N,03248.00000,W,1,09,0.9,925.0,M,")
    match = re.fullmatch(r"\$(.*)\*([0-9A-F]{2})\r\n", gga)
    assert match and int(match.group(2), 16) == nmea.checksum(match.group(1))
    rmc = nmea.rmc(utc, -1.5, 0.25, 5.144444, 450.0)
    assert ",A,0130.00000,S,00015.00000,E,10.00,90.0,091026,,,A*" in rmc


@pytest.fixture(scope="module")
def flight() -> list:
    plant = Plant()
    return [plant.step() for _ in range(int(30.0 / STEP_S))]


def test_vehicle_tracks_the_flight_plan(flight: list) -> None:
    cruise = [s.state for s in flight[1000:]]  # after the first 10 s
    distances = [s.distance_to_line_m for s in cruise]
    assert min(distances) > 10.0 and max(distances) < 18.0
    speeds = [math.hypot(s.velocity.x, s.velocity.y) for s in cruise]
    assert sum(speeds) / len(speeds) == pytest.approx(6.0, abs=0.5)


def test_imu_sees_gravity_and_small_rates(flight: list) -> None:
    samples = [s.imu for s in flight[1000:]]
    mean_z = sum(s.accel_g.z for s in samples) / len(samples)
    assert mean_z == pytest.approx(1.0, abs=0.03)
    assert max(s.gyro_dps.norm() for s in samples) < 30.0


def test_magnetometer_far_from_line_reads_earth_field(flight: list) -> None:
    magnitudes = [s.imu.mag_gauss.norm() for s in flight[1000:]]
    assert sum(magnitudes) / len(magnitudes) == pytest.approx(0.5, abs=0.03)


def test_gps_runs_at_5_hz(flight: list) -> None:
    fixes = [s.gps for s in flight if s.gps is not None]
    assert len(fixes) == pytest.approx(30.0 * 5.0, abs=1)
    assert fixes[0].sentences.count("$GP") == 2


HOVER_FORCE = Vec3(0.0, 0.0, 9.80665)


def _vibration_rms(fault: VibrationFault | None, seconds: float = 5.0) -> tuple[Vec3, set]:
    vibration = Vibration(fault, seed=7)
    samples = [vibration.step(STEP_S, HOVER_FORCE, True) for _ in range(int(seconds / STEP_S))]
    later = samples[len(samples) // 2:]
    rms = Vec3(*(math.sqrt(sum(s.accel_g[i] ** 2 for s in later) / len(later)) for i in range(3)))
    return rms, {s.label for s in later}


def test_healthy_rotors_vibrate_a_little() -> None:
    rms, labels = _vibration_rms(None)
    assert 0.005 < rms.x < 0.03 and 0.005 < rms.y < 0.03
    assert labels == {"nominal"}


def test_damaged_propeller_shakes_the_radial_axes() -> None:
    healthy, _ = _vibration_rms(None)
    rms, labels = _vibration_rms(VibrationFault("imbalance", onset_s=0.0))
    assert rms.x > 5.0 * healthy.x and rms.y > 5.0 * healthy.y
    assert rms.x > 2.0 * rms.z
    assert labels == {"imbalance"}


def test_worn_bearing_shakes_mostly_along_the_motor_axis() -> None:
    healthy, _ = _vibration_rms(None)
    rms, labels = _vibration_rms(VibrationFault("bearing", onset_s=0.0))
    assert rms.z > 10.0 * healthy.z
    assert rms.z > 1.5 * rms.x
    assert labels == {"bearing"}


def test_fault_starts_at_its_onset_and_stops_with_the_motors() -> None:
    vibration = Vibration(VibrationFault("imbalance", onset_s=1.0))
    labels = [vibration.step(STEP_S, HOVER_FORCE, True).label for _ in range(200)]
    assert labels[:99] == ["nominal"] * 99 and labels[100:] == ["imbalance"] * 100
    landed = vibration.step(STEP_S, HOVER_FORCE, False)
    assert landed.accel_g == Vec3(0.0, 0.0, 0.0) and landed.label == "nominal"


def test_unknown_fault_kind_is_rejected() -> None:
    with pytest.raises(ValueError):
        VibrationFault("loose screw", onset_s=0.0)


def test_plant_adds_the_vibration_to_the_imu() -> None:
    plant = Plant(Scenario(vibration_fault=VibrationFault("imbalance", onset_s=0.0)))
    steps = [plant.step() for _ in range(300)]
    assert all(s.vibration.label == "imbalance" for s in steps)
    radial = [s.imu.accel_g.x for s in steps[100:]]
    mean = sum(radial) / len(radial)
    assert math.sqrt(sum((a - mean) ** 2 for a in radial) / len(radial)) > 0.1
