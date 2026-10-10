"""Tests for the vibration dataset collector (needs numpy, from the 'ml' extra)."""

import pytest

np = pytest.importorskip("numpy")

from ml.dataset import flight_scenario, fly, to_firmware_units  # noqa: E402
from sim.plant.vibration import CLASSES  # noqa: E402


def test_firmware_units_quantise_truncate_and_clip() -> None:
    values = np.array([1000.03, -0.1, 5000.0, -5000.0])
    assert to_firmware_units(values, 0.061, 2000.0).tolist() == [1000, 0, 1997, -1997]


def test_flight_conditions_are_reproducible() -> None:
    first = flight_scenario(3, seed=1, duration_s=40.0)
    assert first == flight_scenario(3, seed=1, duration_s=40.0)
    assert first != flight_scenario(4, seed=1, duration_s=40.0)


def test_a_flight_is_labelled_from_the_fault_onset() -> None:
    index = next(i for i in range(50) if flight_scenario(i, 1, 16.0)[1]["fault"] is not None)
    meta, data = fly(index, seed=1, duration_s=16.0)
    assert len(data["t"]) == 1600
    assert data["accel_mg"].dtype == np.int32 and data["accel_mg"].shape == (1600, 3)
    assert abs(float(np.mean(data["accel_mg"][:, 2])) - 1000.0) < 50.0
    onset = meta["fault"]["onset_s"]
    kind = CLASSES.index(meta["fault"]["kind"])
    assert set(data["label"][data["t"] < onset - 0.01].tolist()) == {0}
    assert set(data["label"][data["t"] > onset + 0.01].tolist()) == {kind}
