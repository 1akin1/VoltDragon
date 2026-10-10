"""End-to-end inspection pass: the plant model flies along the line with both nodes in Renode.

Two flights from 150 m along the route, through the close part of the plan
(within 4 m of a conductor as written):

- nominal: the autopilot obeys Node A's avoidance order, so the vehicle stays
  clear of the line (HLR-001, HLR-002); the rest of the system runs as usual
  (HLR-008, HLR-010 to HLR-012);
- no-avoidance: an autopilot that ignores the order (fault injection) flies the
  plan as written, which exercises the magnetometer's disturbance detection
  near the line (HLR-010) and shows Node A still warning.

Needs Renode and the debug firmware build; takes about 6 minutes (the
co-simulation runs at about 0.35x real time).

Run from the repository root (Linux/WSL):
    python -m pytest tests/integration
"""

from __future__ import annotations

import re

import pytest
from flights import Flight, fly, needs_renode

DURATION_S = 45.0
START_M = 150.0

pytestmark = needs_renode


@pytest.fixture(scope="module")
def nominal(tmp_path_factory) -> Flight:
    return fly(tmp_path_factory.mktemp("nominal"), "nominal", DURATION_S, 4570, START_M)


@pytest.fixture(scope="module")
def faulty(tmp_path_factory) -> Flight:
    return fly(tmp_path_factory.mktemp("no_avoidance"), "no-avoidance", DURATION_S, 4571, START_M)


# --- Safety: the close pass (HLR-001, HLR-002)

def test_close_pass_keeps_10_m_from_the_line(nominal: Flight) -> None:
    closest = min(nominal.truth, key=lambda row: row["distance_to_line_m"])
    assert closest["distance_to_line_m"] >= 10.0, closest
    # The plan itself would have come within 4 m: the avoidance did the work.
    assert max(row["keep_out_m"] for row in nominal.truth) == pytest.approx(16.0)


def test_proximity_warning_and_avoidance_order(nominal: Flight) -> None:
    warnings = nominal.messages("a", "safety: PROXIMITY_WARNING")
    assert len(warnings) == 1, warnings
    t, message = warnings[0]
    # Raised in the control cycle that saw the position: well within 100 ms.
    age = int(re.search(r"fix age (\d+) ms", message).group(1))
    assert age < 100, message
    # Close to 12 m by the plant's truth (the GPS wanders by a metre or two).
    assert 9.5 < nominal.at(t)["distance_to_line_m"] < 14.0
    # The order reaches the autopilot, which keeps out from the next step or two.
    assert "$VDCMD,MISSION,1,150*3D" in nominal.autopilot_rx
    assert nominal.at(t + 0.3)["keep_out_m"] > 0.0


def test_mission_continues_without_safety_mode_changes(nominal: Flight) -> None:
    assert not nominal.messages("a", "safety: mode MISSION ->")
    assert not nominal.messages("a", "safety: ground link lost")
    assert nominal.truth[-1]["s"] > START_M + 200.0


def test_faulty_autopilot_flies_the_plan_while_node_a_warns(faulty: Flight) -> None:
    assert min(row["distance_to_line_m"] for row in faulty.truth) < 6.0
    assert faulty.messages("a", "safety: PROXIMITY_WARNING")
    assert all(row["keep_out_m"] == 0.0 for row in faulty.truth)


# --- Navigation (HLR-008, HLR-010)

def test_magnetometer_is_flagged_only_near_the_line(faulty: Flight) -> None:
    disturbed = faulty.messages("a", "nav: magnetometer disturbed")
    recovered = faulty.messages("a", "nav: magnetometer field back")
    assert len(disturbed) == 1, disturbed
    assert len(recovered) == 1, recovered
    # The pass brings the vehicle to about 4 m from a conductor; the 15 % field
    # limit is crossed at about 6.5 m.
    assert faulty.at(disturbed[0][0])["distance_to_line_m"] < 8.0
    assert faulty.at(recovered[0][0])["distance_to_line_m"] > 6.0
    assert recovered[0][0] > disturbed[0][0] + 5.0


def test_magnetometer_stays_usable_when_the_line_is_avoided(nominal: Flight) -> None:
    assert not nominal.messages("a", "nav: magnetometer disturbed")


def test_heading_falls_back_to_gps_course_while_disturbed(faulty: Flight) -> None:
    disturbed_at = faulty.first("a", "nav: magnetometer disturbed")
    recovered_at = faulty.first("a", "nav: magnetometer field back")
    during = [m for t, m in faulty.messages("a", "nav: heading")
              if disturbed_at + 1 < t < recovered_at]
    assert during and all("from GPS course" in m for m in during)


def test_magnetometer_heading_matches_truth(nominal: Flight) -> None:
    errors = []
    for t, message in nominal.messages("a", "nav: heading"):
        match = re.match(r"nav: heading (\d+) cdeg from magnetometer", message)
        if match and t > 12.0:
            measured = int(match.group(1)) / 100.0
            true = nominal.at(t)["heading_deg"]
            errors.append(abs((measured - true + 180.0) % 360.0 - 180.0))
    assert len(errors) > 25
    assert max(errors) < 6.0


# --- Links (HLR-011, HLR-012)

def test_gps_runs_at_5_fixes_per_second(nominal: Flight) -> None:
    rates = [int(m.split()[1]) for t, m in nominal.messages("a", "gps: ")
             if "sentences/s" in m and t > 2.0]
    assert rates and all(r == 10 for r in rates), rates
    assert not any("bad sentences" in m or "receive errors" in m for _, m in nominal.a)


def test_can_and_telemetry_run_without_losses(nominal: Flight) -> None:
    can = [m for t, m in nominal.messages("b", "can: rx") if t > 2.0]
    assert can and all(m.startswith("can: rx 450/s, rejected 0, lost 0, unknown id 0, overruns 0")
                       for m in can)
    tlm = [m for t, m in nominal.messages("b", "tlm:") if t > 2.0]
    assert tlm and all(m.startswith("tlm: 10 packets/s") and m.endswith("failed 0") for m in tlm)


def test_position_and_safety_state_reach_node_b(nominal: Flight) -> None:
    gps = [m for t, m in nominal.messages("b", "can: A heading") if t > 2.0]
    assert gps and all(m.endswith("gps fix") for m in gps)
    safety = [m for t, m in nominal.messages("b", "can: A mode") if t > 2.0]
    assert safety and all(m.startswith("can: A mode MISSION") for m in safety)
    # Heartbeats once a second: Node B never sees the ground link older than about 1 s.
    ages = [int(re.search(r"ground link age (\d+) ms", m).group(1)) for m in safety]
    assert max(ages) < 1200, ages


# --- Vibration monitor (HLR-009): no false alarm on a healthy airframe

def test_no_vibration_alarm_on_a_healthy_airframe(nominal: Flight, faulty: Flight) -> None:
    for flight in (nominal, faulty):
        assert not any("VIBRATION FAULT" in m for _, m in flight.a + flight.b)
    monitor = [m for t, m in nominal.messages("b", "can: A vibration monitor") if t > 2.0]
    assert monitor and all(m.startswith("can: A vibration monitor active, alarm nominal")
                           for m in monitor)
