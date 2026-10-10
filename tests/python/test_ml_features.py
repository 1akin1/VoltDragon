"""Tests for the vibration features and the alarm debouncing (needs numpy, from the 'ml' extra)."""

import math

import pytest

np = pytest.importorskip("numpy")

from ml import features as F  # noqa: E402


def _tone(axis: int, frequency_hz: float, amplitude: float) -> np.ndarray:
    t = np.arange(F.WINDOW) / F.SAMPLE_RATE_HZ
    window = np.zeros((F.WINDOW, F.AXES))
    window[:, axis] = amplitude * np.sin(2.0 * math.pi * frequency_hz * t)
    return window


def _band_of(frequency_hz: float) -> int:
    k = round(frequency_hz / (F.SAMPLE_RATE_HZ / F.WINDOW))
    return next(i for i, (lo, hi) in enumerate(F.BANDS) if lo <= k <= hi)


def test_constant_input_gives_the_floor_everywhere() -> None:
    window = np.tile([0.1, -0.2, 1.0, 5.0, 0.0, -3.0], (F.WINDOW, 1))
    assert np.allclose(F.window_features(window), math.log10(F.POWER_FLOOR))


@pytest.mark.parametrize("frequency_hz", [3.125, 12.5, 20.3125, 37.5])
def test_a_tone_lands_in_its_band_on_its_axis(frequency_hz: float) -> None:
    features = F.window_features(_tone(4, frequency_hz, 2.0)).reshape(F.AXES, len(F.BANDS))
    band = _band_of(frequency_hz)
    assert np.unravel_index(features.argmax(), features.shape) == (4, band)
    others = np.delete(features, 4, axis=0)
    assert np.allclose(others, math.log10(F.POWER_FLOOR))


def test_power_follows_amplitude_squared() -> None:
    small = F.window_features(_tone(0, 12.5, 0.01)).reshape(F.AXES, -1)[0, _band_of(12.5)]
    large = F.window_features(_tone(0, 12.5, 0.1)).reshape(F.AXES, -1)[0, _band_of(12.5)]
    assert large - small == pytest.approx(2.0, abs=0.01)


def test_window_shape_is_checked() -> None:
    with pytest.raises(ValueError):
        F.window_features(np.zeros((F.WINDOW - 1, F.AXES)))


def test_windows_hop_by_half_a_window() -> None:
    assert F.windows(np.zeros(160)).tolist() == [0, 32, 64, 96]


def test_alarm_needs_three_of_four_fault_windows() -> None:
    alarm = F.AlarmFilter()
    assert [alarm.update(c) for c in (1, 0, 1, 0, 1)] == [0, 0, 0, 0, 0]
    alarm = F.AlarmFilter()
    assert [alarm.update(c) for c in (1, 0, 1, 1)] == [0, 0, 0, 1]


def test_alarm_takes_the_most_frequent_fault_and_clears_after_eight_nominal() -> None:
    alarm = F.AlarmFilter()
    assert [alarm.update(c) for c in (2, 1, 2)] == [0, 0, 2]
    assert [alarm.update(0) for _ in range(F.CLEAR_WINDOWS - 1)] == [2] * (F.CLEAR_WINDOWS - 1)
    assert alarm.update(1) == 2          # a fault window resets the nominal run
    assert [alarm.update(0) for _ in range(F.CLEAR_WINDOWS)][-1] == 0
