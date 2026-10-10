"""Vibration faults: the plant's rotors shake the IMU and Node A must classify it on board.

- prop-damage: a propeller is damaged 15 s into the flight; Node A's INT8
  classifier must raise a VIBRATION_FAULT alarm for an imbalance, and the alarm
  must reach the ground station within 2 s of the onset (HLR-009).
- bearing-wear: the same for a worn motor bearing.

Before the onset the airframe is healthy and no alarm may be raised. The
time to the ground station is taken at Node B, which logs the alarm as soon as
Node A's HEALTH message carries it; the next telemetry packet (at most 100 ms
later) takes it to the ground station.

Open-loop checks with a synthetic signature are in tests/robot/node_a_ai.robot
and system_telemetry.robot; the classifier's accuracy is in
docs/vibration-model-report.md.

Run from the repository root (Linux/WSL):
    python -m pytest tests/integration
"""

from __future__ import annotations

import pytest
from flights import Flight, fly, needs_renode

ONSET_S = 15.0          # sim/mission.py: the prop-damage and bearing-wear scenarios
DURATION_S = 20.0
TELEMETRY_PERIOD_S = 0.1

pytestmark = needs_renode


@pytest.fixture(scope="module")
def prop_damage(tmp_path_factory) -> Flight:
    return fly(tmp_path_factory.mktemp("prop_damage"), "prop-damage", DURATION_S, 4574)


@pytest.fixture(scope="module")
def bearing_wear(tmp_path_factory) -> Flight:
    return fly(tmp_path_factory.mktemp("bearing_wear"), "bearing-wear", DURATION_S, 4575)


@pytest.mark.parametrize(("scenario", "kind"), [("prop_damage", "imbalance"),
                                                ("bearing_wear", "bearing")])
def test_fault_reaches_the_ground_station_within_2_s(request, scenario: str, kind: str) -> None:
    flight: Flight = request.getfixturevalue(scenario)
    on_node_a = flight.first("a", "ai: VIBRATION FAULT")
    on_node_b = flight.first("b", "can: Node A VIBRATION FAULT")
    assert flight.messages("a", "ai: VIBRATION FAULT")[0][1].startswith(
        f"ai: VIBRATION FAULT: {kind}")
    assert flight.messages("b", "can: Node A VIBRATION FAULT")[0][1].startswith(
        f"can: Node A VIBRATION FAULT: {kind}")
    assert ONSET_S < on_node_a <= on_node_b
    assert on_node_b + TELEMETRY_PERIOD_S - ONSET_S < 2.0


@pytest.mark.parametrize("scenario", ["prop_damage", "bearing_wear"])
def test_no_alarm_before_the_fault(request, scenario: str) -> None:
    flight: Flight = request.getfixturevalue(scenario)
    early = [m for t, m in flight.a + flight.b if t < ONSET_S and "VIBRATION" in m]
    assert early == []
    # The monitor ran throughout: three windows a second, all nominal before the onset.
    reports = [m for t, m in flight.messages("a", "ai: active") if 2.0 < t < ONSET_S]
    assert len(reports) >= 12 and all("alarm nominal" in m for m in reports)


def test_alarm_stays_raised_and_is_recorded(prop_damage: Flight) -> None:
    after = [m for t, m in prop_damage.messages("a", "ai: ") if t > ONSET_S + 2.0]
    assert after and all("alarm imbalance" in m for m in after if m.startswith("ai: active"))
    assert not any("vibration alarm cleared" in m for _, m in prop_damage.a)
    # The ground station's view: Node B's report carries the alarm and a high fault score.
    reports = [m for t, m in prop_damage.messages("b", "can: A vibration monitor")
               if t > ONSET_S + 2.0]
    assert reports and all("alarm imbalance" in m for m in reports)
