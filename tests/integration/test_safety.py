"""Closed-loop failsafes: the plant flies, both nodes run in Renode, and the vehicle must react.

- link-loss: the ground station's heartbeat stops after 12 s; Node A must order
  RETURN_TO_HOME just over 3 s after the last contact, and the vehicle must turn
  back along the route (HLR-003).
- battery: the flight starts at 22 % and drains at 0.5 %/s; below 20 % the
  vehicle must return home and below 10 % land (HLR-004).

Open-loop checks of the same logic, with exact inputs, are in
tests/robot/system_safety.robot.

Run from the repository root (Linux/WSL):
    python -m pytest tests/integration
"""

from __future__ import annotations

import re

import pytest
from flights import Flight, fly, mode_index, needs_renode

pytestmark = needs_renode


@pytest.fixture(scope="module")
def link_loss(tmp_path_factory) -> Flight:
    return fly(tmp_path_factory.mktemp("link_loss"), "link-loss", 25.0, 4572)


@pytest.fixture(scope="module")
def battery(tmp_path_factory) -> Flight:
    return fly(tmp_path_factory.mktemp("battery"), "battery", 36.0, 4573)


def test_link_loss_returns_home_after_3_s(link_loss: Flight) -> None:
    last_ping = link_loss.messages("b", "cmd: $")[-1][0]
    lost = link_loss.first("a", "safety: ground link lost")
    assert 3.0 < lost - last_ping < 3.3
    switched = link_loss.first("a", "safety: mode MISSION -> RETURN_TO_HOME (ground link lost)")
    assert switched - lost < 0.05
    assert "$VDCMD,RTH,0,0*3A" in link_loss.autopilot_rx


def test_vehicle_turns_back_on_link_loss(link_loss: Flight) -> None:
    switched = link_loss.first("a", "safety: mode MISSION -> RETURN_TO_HOME")
    assert link_loss.at(switched + 0.3)["mode"] == mode_index("RTH")
    furthest = max(row["s"] for row in link_loss.truth)
    assert link_loss.truth[-1]["s"] < furthest - 20.0
    # Node B and the ground station see it too.
    assert any("can: A mode RETURN_TO_HOME" in m and "flags 0x24" in m for _, m in link_loss.b)


def test_low_battery_returns_home_then_critical_battery_lands(battery: Flight) -> None:
    low = battery.first("a", "safety: battery low, 19 %")
    critical = battery.first("a", "safety: battery critical, 9 %")
    # 22 % draining at 0.5 %/s: below 20 % (reported as 19) at 4 s, below 10 % at 24 s;
    # the autopilot reports at 5 Hz.
    assert 4.0 < low < 4.5
    assert 24.0 < critical < 24.5
    assert battery.first("a", "safety: mode MISSION -> RETURN_TO_HOME (battery low)") - low < 0.05
    assert battery.first("a", "safety: mode RETURN_TO_HOME -> LAND (battery critical)") \
        - critical < 0.05
    assert battery.at(low + 0.3)["mode"] == mode_index("RTH")
    assert battery.at(critical + 0.3)["mode"] == mode_index("LAND")


def test_vehicle_descends_at_1_mps_when_landing(battery: Flight) -> None:
    critical = battery.first("a", "safety: battery critical")
    start, end = battery.at(critical + 2.0), battery.truth[-1]
    rate = (start["up"] - end["up"]) / (end["t"] - start["t"])
    assert rate == pytest.approx(1.0, abs=0.15)
    report = [m for _, m in battery.messages("b", "can: A mode LAND")]
    assert report and re.search(r"battery [0-9] %, flags 0x38", report[-1])
