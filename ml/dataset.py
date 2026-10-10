"""Collects labelled IMU data for the vibration classifier from the plant model.

Each flight draws its own conditions: flight plan (hover or cruise, speed,
altitude, offset, start along the line), wind and gusts, the airframe's
healthy vibration level, and either no fault or a damaged propeller or worn
motor bearing on a random rotor, with a random onset time and severity.

The plant runs on its own here, without Renode: the IMU samples are converted
to what Node A's LSM9DS1 driver reports (whole mg and mdps after the sensor's
LSB quantisation, clipped at the configured full-scale ranges), so the
features are computed from the same numbers as on the MCU.

Usage (needs numpy):
    python -m ml.dataset [--flights 240] [--duration 40] [--seed 1] [--out build/ml/dataset.npz]

The file holds, per 100 Hz sample: flight index, time, accel (mg) and gyro
(mdps) as int32, the label (index into sim.plant.vibration.CLASSES) and
whether the vehicle was airborne; plus each flight's parameters as JSON.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import random
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from sim.plant.plant import STEP_S, Plant, Scenario
from sim.plant.vec import Vec3
from sim.plant.vehicle import FlightPlan, Wind
from sim.plant.vibration import CLASSES, KINDS, VibrationFault

REPO = Path(__file__).resolve().parent.parent

# LSM9DS1 sensitivities and ranges as configured by the firmware (lsm9ds1.c).
ACCEL_LSB_MG = 0.061
GYRO_LSB_MDPS = 8.75
ACCEL_RANGE_MG = 2000.0
GYRO_RANGE_MDPS = 245000.0


def to_firmware_units(values: np.ndarray, lsb: float, full_scale: float) -> np.ndarray:
    """Clips like the co-simulation (sim/cosim.py), quantises to the sensor's LSB, then
    truncates to whole units like the driver."""
    margin = full_scale * 0.999
    counts = np.round(np.clip(values, -margin, margin) / lsb)
    return np.trunc(counts * lsb).astype(np.int32)


def flight_scenario(index: int, seed: int, duration_s: float) -> tuple[Scenario, dict]:
    rng = random.Random(seed * 100_003 + index)
    hover = rng.random() < 0.15
    plan = FlightPlan(
        speed_mps=0.0 if hover else rng.uniform(3.0, 9.0),
        altitude_m=rng.uniform(15.0, 35.0),
        offset_m=rng.uniform(14.0, 30.0),
        start_s_m=rng.uniform(0.0, 300.0),
    )
    kind = rng.choice((None, *KINDS))
    fault = None
    if kind is not None:
        fault = VibrationFault(
            kind=kind,
            onset_s=rng.uniform(5.0, duration_s - 10.0),
            severity=rng.uniform(0.2, 1.2),
            rotor=rng.randrange(4),
        )
    wind = {
        "mean_east": rng.uniform(-6.0, 6.0),
        "mean_north": rng.uniform(-6.0, 6.0),
        "gust_sigma": rng.uniform(0.3, 3.0),
    }
    scenario = Scenario(
        plan=plan,
        wind_seed=rng.randrange(1 << 30),
        sensor_seed=rng.randrange(1 << 30),
        vibration_fault=fault,
        vibration_seed=rng.randrange(1 << 30),
        vibration_baseline=rng.uniform(0.6, 1.6),
    )
    meta = {
        "index": index,
        "hover": hover,
        "plan": dataclasses.asdict(plan),
        "wind": wind,
        "fault": dataclasses.asdict(fault) if fault is not None else None,
        "baseline": scenario.vibration_baseline,
    }
    return scenario, meta


def fly(index: int, seed: int, duration_s: float) -> tuple[dict, dict[str, np.ndarray]]:
    scenario, meta = flight_scenario(index, seed, duration_s)
    plant = Plant(scenario)
    w = meta["wind"]
    plant.vehicle.wind = Wind(
        mean_enu=Vec3(w["mean_east"], w["mean_north"], 0.0),
        gust_sigma_mps=w["gust_sigma"],
        seed=scenario.wind_seed,
    )
    steps = [plant.step() for _ in range(round(duration_s / STEP_S))]
    accel_g = np.array([s.imu.accel_g for s in steps])
    gyro_dps = np.array([s.imu.gyro_dps for s in steps])
    arrays = {
        "t": np.array([s.state.t for s in steps], dtype=np.float32),
        "accel_mg": to_firmware_units(accel_g * 1000.0, ACCEL_LSB_MG, ACCEL_RANGE_MG),
        "gyro_mdps": to_firmware_units(gyro_dps * 1000.0, GYRO_LSB_MDPS, GYRO_RANGE_MDPS),
        "label": np.array([CLASSES.index(s.vibration.label) for s in steps], dtype=np.int8),
        "airborne": np.array([s.state.airborne for s in steps], dtype=bool),
    }
    return meta, arrays


def collect(flights: int, duration_s: float, seed: int, workers: int | None = None) -> dict:
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(fly, range(flights), [seed] * flights, [duration_s] * flights))
    metas = [m for m, _ in results]
    data = {
        key: np.concatenate([a[key] for _, a in results]) for key in results[0][1]
    }
    data["flight"] = np.concatenate(
        [np.full(len(a["t"]), m["index"], dtype=np.int32) for m, a in results]
    )
    data["meta"] = np.array(json.dumps(metas))
    data["classes"] = np.array(CLASSES)
    return data


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flights", type=int, default=240)
    parser.add_argument("--duration", type=float, default=40.0, help="seconds per flight")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--out", type=Path, default=REPO / "build" / "ml" / "dataset.npz")
    args = parser.parse_args(argv)

    data = collect(args.flights, args.duration, args.seed, args.workers)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, **data)
    counts = np.bincount(data["label"], minlength=len(CLASSES))
    summary = ", ".join(f"{c} {n / 100.0:.0f} s" for c, n in zip(CLASSES, counts, strict=True))
    print(f"{args.flights} flights, {len(data['t']) / 100.0:.0f} s of data ({summary}) "
          f"-> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
