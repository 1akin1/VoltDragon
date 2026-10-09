"""Shared helpers for the integration tests: fly a scenario, read the logs and the ground truth."""

from __future__ import annotations

import csv
import re
import shutil
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

import pytest

from sim import mission
from sim.plant.autopilot import MODES

REPO = Path(__file__).resolve().parents[2]
LINE = re.compile(r"^\[\s*(\d+\.\d+)\] (\w) (.*)$")

needs_renode = pytest.mark.skipif(
    shutil.which("renode") is None
    or not (REPO / "build/debug/firmware/node_a/node_a.elf").exists(),
    reason="needs Renode and the debug firmware build",
)


def read_log(path: Path) -> list[tuple[float, str]]:
    """(time, message) for each line of a node's console log."""
    entries = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = LINE.match(raw.strip())
        if match:
            entries.append((float(match.group(1)), match.group(3)))
    return entries


@dataclass
class Flight:
    a: list[tuple[float, str]]          # Node A console
    b: list[tuple[float, str]]          # Node B console
    truth: list[dict[str, float]]       # plant ground truth every 50 ms
    autopilot_rx: list[str]             # what Node A sent to the autopilot

    def __post_init__(self) -> None:
        self._t = [row["t"] for row in self.truth]

    def at(self, t: float) -> dict[str, float]:
        """Ground truth at (just after) time t."""
        return self.truth[min(bisect_left(self._t, t), len(self.truth) - 1)]

    def first(self, node: str, prefix: str) -> float:
        """Time of the first message on a node starting with prefix."""
        log = self.a if node == "a" else self.b
        return next(t for t, m in log if m.startswith(prefix))

    def messages(self, node: str, prefix: str) -> list[tuple[float, str]]:
        log = self.a if node == "a" else self.b
        return [(t, m) for t, m in log if m.startswith(prefix)]


def mode_index(name: str) -> float:
    return float(MODES.index(name))


def fly(out: Path, scenario: str, duration_s: float, port: int, start_m: float = 0.0) -> Flight:
    argv = ["--scenario", scenario, "--duration", str(duration_s), "--start", str(start_m),
            "--out", str(out), "--port", str(port)]
    assert mission.main(argv) == 0
    with (out / "truth.csv").open(encoding="utf-8") as f:
        truth = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
    rx = (out / "autopilot_rx.log").read_text(encoding="ascii", errors="replace").splitlines()
    return Flight(read_log(out / "node_a.log"), read_log(out / "node_b.log"), truth, rx)
