"""Smoke tests ensuring the Python packages import cleanly."""

import ground_station
import sim.plant


def test_packages_import() -> None:
    assert sim.plant.__version__ == "0.1.0"
    assert ground_station.__version__ == "0.1.0"
