"""The plant: vehicle, environment, sensors and autopilot advanced together in fixed 10 ms steps."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import UTC, datetime

from sim.plant.autopilot import Autopilot, Battery
from sim.plant.magnetics import EarthField
from sim.plant.route import Route
from sim.plant.sensors import Gps, GpsFix, Imu, ImuSample
from sim.plant.vehicle import FlightPlan, Vehicle, VehicleState, Wind
from sim.plant.vibration import Vibration, VibrationFault, VibrationSample

STEP_S = 0.01      # matches Node A's 100 Hz IMU sampling: one sample per firmware read


@dataclass(frozen=True)
class Scenario:
    route_name: str = "line_a"
    plan: FlightPlan = field(default_factory=FlightPlan)
    earth: EarthField = field(default_factory=EarthField)
    wind_seed: int = 1
    sensor_seed: int = 2
    start_utc: datetime = datetime(2026, 10, 9, 10, 0, 0, tzinfo=UTC)
    battery_pct: float = 95.0
    battery_drain_pct_per_s: float = 0.05
    # Fault injection: an autopilot that ignores avoidance orders flies the plan as
    # written, however close it comes to the line.
    obey_avoidance: bool = True
    # Fault injection: a damaged propeller or a worn motor bearing (sim/plant/vibration.py).
    vibration_fault: VibrationFault | None = None
    vibration_seed: int = 4
    vibration_baseline: float = 1.0     # scales the healthy airframe's residual vibration


@dataclass(frozen=True)
class PlantStep:
    state: VehicleState
    imu: ImuSample
    gps: GpsFix | None
    autopilot_tx: str | None    # $VDAPS sentence the autopilot sends in this step
    vibration: VibrationSample  # the rotors' share of the IMU sample, and its label


class Plant:
    def __init__(self, scenario: Scenario | None = None) -> None:
        self.scenario = scenario or Scenario()
        self.route = Route.load(self.scenario.route_name)
        self.battery = Battery(self.scenario.battery_pct, self.scenario.battery_drain_pct_per_s)
        self.autopilot = Autopilot(self.battery, obey_avoidance=self.scenario.obey_avoidance)
        self.vehicle = Vehicle(
            self.route, self.scenario.plan, Wind(seed=self.scenario.wind_seed), self.autopilot
        )
        self.imu = Imu(self.route, self.scenario.earth, seed=self.scenario.sensor_seed)
        self.vibration = Vibration(
            self.scenario.vibration_fault,
            seed=self.scenario.vibration_seed,
            baseline_scale=self.scenario.vibration_baseline,
        )
        self.gps = Gps(self.route, self.scenario.start_utc, seed=self.scenario.sensor_seed + 1)

    def receive(self, text: str) -> None:
        """Bytes Node A sent to the autopilot (its UART5)."""
        self.autopilot.receive(text)

    def step(self) -> PlantStep:
        state = self.vehicle.step(STEP_S)
        self.battery.step(STEP_S, state.airborne)
        fix = self.gps.fix(state) if self.gps.due(state.t) else None
        vibration = self.vibration.step(STEP_S, state.specific_force_body, state.airborne)
        return PlantStep(
            state=state,
            imu=self.imu.sample(state, vibration),
            gps=fix,
            autopilot_tx=self.autopilot.status(state.t),
            vibration=vibration,
        )
