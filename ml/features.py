"""Vibration features and alarm logic: the reference for Node A's C implementation.

Node A classifies a window of WINDOW consecutive IMU samples (0.64 s at
100 Hz) every HOP samples (0.32 s). For each of the six axes (accelerometer
x, y, z in g and gyroscope x, y, z in deg/s):

1. the window's mean is removed (gravity, the flight's slow manoeuvres);
2. the samples are weighted by a periodic Hann window;
3. the power spectrum of bins 1..32 is computed with a direct DFT;
4. the power is summed in BANDS and the log10 of each band (plus a floor)
   is a feature.

That gives 6 x 7 = 42 features. Spectral bands suit this problem: a damaged
propeller is a narrow tone on the radial axes, a worn bearing is broadband
energy along the motor axis, and both differ from the airframe's healthy
vibration and from manoeuvres, which stay in the lowest band. Bins are
1.5625 Hz wide; the rotor tones arrive aliased (sim/plant/vibration.py).

The firmware (firmware/node_a/src/vib_features.c) computes the same in single
precision; tests/unit/test_vib_features.c checks it against vectors that
ml/train.py produces with this module.

The alarm (AlarmFilter) debounces the per-window classification: a fault is
raised when FAULT_WINDOWS of the last HISTORY windows show a fault, and
cleared after CLEAR_WINDOWS consecutive nominal windows.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

SAMPLE_RATE_HZ = 100.0
WINDOW = 64
HOP = 32
AXES = 6
# Inclusive DFT bin ranges (bin k is k x 1.5625 Hz).
BANDS = ((1, 2), (3, 5), (6, 9), (10, 14), (15, 20), (21, 26), (27, 32))
FEATURES = AXES * len(BANDS)
POWER_FLOOR = 1e-6

# Alarm debouncing, in windows (one every 0.32 s).
HISTORY = 4
FAULT_WINDOWS = 3
CLEAR_WINDOWS = 8

AXIS_NAMES = ("ax", "ay", "az", "gx", "gy", "gz")
FEATURE_NAMES = tuple(f"{axis}_b{band}" for axis in AXIS_NAMES for band in range(len(BANDS)))


def hann() -> np.ndarray:
    n = np.arange(WINDOW)
    return 0.5 - 0.5 * np.cos(2.0 * np.pi * n / WINDOW)


def to_physical(accel_mg: np.ndarray, gyro_mdps: np.ndarray) -> np.ndarray:
    """(N, 3) accel in mg and gyro in mdps -> (N, 6) in g and deg/s."""
    return np.concatenate([accel_mg / 1000.0, gyro_mdps / 1000.0], axis=-1)


def window_features(window: np.ndarray) -> np.ndarray:
    """Features of one window: (WINDOW, 6) samples in g and deg/s -> (FEATURES,)."""
    if window.shape != (WINDOW, AXES):
        raise ValueError(f"expected a ({WINDOW}, {AXES}) window, got {window.shape}")
    x = (window - window.mean(axis=0)) * hann()[:, None]
    spectrum = np.fft.rfft(x, axis=0)                   # bins 0..32
    power = (spectrum.real ** 2 + spectrum.imag ** 2) / WINDOW
    bands = np.stack([power[lo:hi + 1].sum(axis=0) for lo, hi in BANDS])   # (bands, axes)
    return np.log10(bands.T + POWER_FLOOR).reshape(FEATURES).astype(np.float32)


def windows(samples: np.ndarray) -> np.ndarray:
    """Start indices of the windows Node A classifies in a run of samples."""
    return np.arange(0, len(samples) - WINDOW + 1, HOP)


@dataclass
class AlarmFilter:
    """Debounces per-window classes into an alarm (0 = none, else the fault class)."""

    history: deque | None = None
    nominal_run: int = 0
    alarm: int = 0

    def __post_init__(self) -> None:
        self.history = deque(maxlen=HISTORY)

    def update(self, predicted: int) -> int:
        self.history.append(predicted)
        if predicted == 0:
            self.nominal_run += 1
            if self.nominal_run >= CLEAR_WINDOWS:
                self.alarm = 0
            return self.alarm
        self.nominal_run = 0
        faults = [c for c in self.history if c != 0]
        if len(faults) >= FAULT_WINDOWS:
            # The most frequent fault class in the history; ties go to the latest.
            counts = {c: faults.count(c) for c in faults}
            best = max(counts.values())
            self.alarm = next(c for c in reversed(faults) if counts[c] == best)
        return self.alarm
