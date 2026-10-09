"""End-to-end mission test: the plant model flies the inspection pass with both nodes in Renode.

Checks the firmware's behaviour against the plant's ground truth (HLR-008,
HLR-010, HLR-011, HLR-012). Needs Renode and the debug firmware build; takes
about 3.5 minutes (the co-simulation runs at about 0.35x real time).

Run from the repository root (Linux/WSL):
    python -m pytest tests/integration
"""

from __future__ import annotations

import csv
import re
import shutil
from bisect import bisect_left
from pathlib import Path

import pytest

from sim import mission

REPO = Path(__file__).resolve().parents[2]
DURATION_S = 70.0
LINE = re.compile(r"^\[\s*(\d+\.\d+)\] (\w) (.*)$")

pytestmark = pytest.mark.skipif(
    shutil.which("renode") is None
    or not (REPO / "build/debug/firmware/node_a/node_a.elf").exists(),
    reason="needs Renode and the debug firmware build",
)


def read_log(path: Path) -> list[tuple[float, str]]:
    entries = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = LINE.match(raw.strip())
        if match:
            entries.append((float(match.group(1)), match.group(3)))
    return entries


@pytest.fixture(scope="module")
def flight(tmp_path_factory) -> dict:
    out = tmp_path_factory.mktemp("mission")
    assert mission.main(["--duration", str(DURATION_S), "--out", str(out), "--port", "4570"]) == 0
    with (out / "truth.csv").open(encoding="utf-8") as f:
        truth = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
    return {
        "a": read_log(out / "node_a.log"),
        "b": read_log(out / "node_b.log"),
        "truth": truth,
        "truth_t": [row["t"] for row in truth],
    }


def truth_at(flight: dict, t: float) -> dict:
    i = min(bisect_left(flight["truth_t"], t), len(flight["truth"]) - 1)
    return flight["truth"][i]


def test_magnetometer_is_flagged_only_near_the_line(flight: dict) -> None:
    disturbed = [t for t, msg in flight["a"] if msg.startswith("nav: magnetometer disturbed")]
    recovered = [t for t, msg in flight["a"] if msg.startswith("nav: magnetometer field back")]
    assert len(disturbed) == 1, disturbed
    assert len(recovered) == 1, recovered
    # The pass brings the vehicle to about 4 m from a conductor; the 15 % field
    # limit is crossed at about 6.5 m.
    assert truth_at(flight, disturbed[0])["distance_to_line_m"] < 8.0
    assert truth_at(flight, recovered[0])["distance_to_line_m"] > 6.0
    assert recovered[0] > disturbed[0] + 5.0


def test_heading_falls_back_to_gps_course_while_disturbed(flight: dict) -> None:
    disturbed_at = next(t for t, m in flight["a"] if m.startswith("nav: magnetometer disturbed"))
    recovered_at = next(t for t, m in flight["a"] if m.startswith("nav: magnetometer field back"))
    during = [m for t, m in flight["a"]
              if m.startswith("nav: heading") and disturbed_at + 1 < t < recovered_at]
    assert during and all("from GPS course" in m for m in during)


def test_magnetometer_heading_matches_truth_in_cruise(flight: dict) -> None:
    errors = []
    for t, msg in flight["a"]:
        match = re.match(r"nav: heading (\d+) cdeg from magnetometer", msg)
        if match and t > 12.0:
            measured = int(match.group(1)) / 100.0
            true = truth_at(flight, t)["heading_deg"]
            errors.append(abs((measured - true + 180.0) % 360.0 - 180.0))
    assert len(errors) > 30
    assert max(errors) < 6.0


def test_gps_runs_at_5_fixes_per_second(flight: dict) -> None:
    rates = [int(m.split()[1]) for t, m in flight["a"] if m.startswith("gps: ") and
             "sentences/s" in m and t > 2.0]
    assert rates and all(r == 10 for r in rates), rates
    assert not any("bad sentences" in m or "receive errors" in m for _, m in flight["a"])


def test_can_and_telemetry_run_without_losses(flight: dict) -> None:
    can = [m for t, m in flight["b"] if m.startswith("can: rx") and t > 2.0]
    assert can and all(m.startswith("can: rx 350/s, rejected 0, lost 0, unknown id 0, overruns 0")
                       for m in can)
    tlm = [m for t, m in flight["b"] if m.startswith("tlm:") and t > 2.0]
    assert tlm and all(m.startswith("tlm: 10 packets/s") and m.endswith("failed 0") for m in tlm)


def test_position_reaches_node_b(flight: dict) -> None:
    gps = [m for t, m in flight["b"] if m.startswith("can: A heading") and t > 2.0]
    assert gps and all(m.endswith("gps fix") for m in gps)
