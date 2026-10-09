"""The plant: vehicle, environment and sensors advanced together in fixed 10 ms steps."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import UTC, datetime

from sim.plant.magnetics import EarthField
from sim.plant.route import Route
from sim.plant.sensors import Gps, GpsFix, Imu, ImuSample
from sim.plant.vehicle import FlightPlan, Vehicle, VehicleState, Wind

STEP_S = 0.01      # matches Node A's 100 Hz IMU sampling: one sample per firmware read


@dataclass(frozen=True)
class Scenario:
    route_name: str = "line_a"
    plan: FlightPlan = field(default_factory=FlightPlan)
    earth: EarthField = field(default_factory=EarthField)
    wind_seed: int = 1
    sensor_seed: int = 2
    start_utc: datetime = datetime(2026, 10, 9, 10, 0, 0, tzinfo=UTC)


@dataclass(frozen=True)
class PlantStep:
    state: VehicleState
    imu: ImuSample
    gps: GpsFix | None


class Plant:
    def __init__(self, scenario: Scenario | None = None) -> None:
        self.scenario = scenario or Scenario()
        self.route = Route.load(self.scenario.route_name)
        self.vehicle = Vehicle(self.route, self.scenario.plan, Wind(seed=self.scenario.wind_seed))
        self.imu = Imu(self.route, self.scenario.earth, seed=self.scenario.sensor_seed)
        self.gps = Gps(self.route, self.scenario.start_utc, seed=self.scenario.sensor_seed + 1)

    def step(self) -> PlantStep:
        state = self.vehicle.step(STEP_S)
        fix = self.gps.fix(state) if self.gps.due(state.t) else None
        return PlantStep(state=state, imu=self.imu.sample(state), gps=fix)
