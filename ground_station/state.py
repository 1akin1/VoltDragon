"""Ground-station state: position on the route, history for the plots, and alarms (HLR-019).

Pure logic, independent of any display, so it is unit-tested directly.

The proximity alarms use the GPS position from telemetry and the route's
conductor geometry: the ground station's own check of HLR-001/HLR-002. The
vehicle's on-board safety logic does not depend on it; its decisions (mode,
avoidance, link loss, battery) arrive in the safety fields of telemetry and are
shown as alarms of their own.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from ground_station.telemetry import Telemetry
from sim.plant.route import Route

LINK_TIMEOUT_S = 1.0
RECENT_S = 5.0
PROXIMITY_WARNING_M = 12.0      # HLR-002
MINIMUM_DISTANCE_M = 10.0       # HLR-001
FIELD_EXPECTED_MG = 500.0       # must match the firmware's NAV_FIELD_MGAUSS
FIELD_TOLERANCE = 0.15


@dataclass(frozen=True)
class Alarm:
    name: str
    severity: str               # "warning" or "critical"
    detail: str


@dataclass(frozen=True)
class Sample:
    """One telemetry packet as plotted."""

    t: float                    # receive time on the ground station's clock
    mission_t: float            # Node B uptime, s: the time axis of the plots
    east: float | None
    north: float | None
    distance_m: float | None
    heading_deg: float
    heading_source: str
    field_mgauss: int
    mag_ok: bool
    alt_m: float


@dataclass
class GroundState:
    route: Route
    history_s: float = 300.0
    samples: deque[Sample] = field(default_factory=deque)
    last: Telemetry | None = None
    last_rx: float | None = None
    received: int = 0
    lost: int = 0
    _expected_seq: int | None = None
    _loss_times: deque[float] = field(default_factory=deque)
    _can_bad: deque[tuple[float, int]] = field(default_factory=deque)

    def update(self, packet: Telemetry, now: float) -> None:
        """Records a received packet."""
        if self._expected_seq is not None and packet.seq > self._expected_seq:
            missed = packet.seq - self._expected_seq
            self.lost += missed
            self._loss_times.append(now)
        self._expected_seq = packet.seq + 1
        self.received += 1
        self.last = packet
        self.last_rx = now
        self._can_bad.append((now, packet.can_rejected + packet.can_lost))

        east = north = distance = None
        if packet.gps_fix:
            p = self.route.geodetic_to_enu(packet.lat_deg, packet.lon_deg, packet.alt_msl_m)
            east, north = p.x, p.y
            distance = self.route.distance_to_conductors(p)
        self.samples.append(
            Sample(
                t=now,
                mission_t=packet.node_b_uptime_ms / 1000.0,
                east=east,
                north=north,
                distance_m=distance,
                heading_deg=packet.heading_deg,
                heading_source=packet.heading_source,
                field_mgauss=packet.field_mgauss,
                mag_ok=packet.mag_ok,
                alt_m=packet.alt_msl_m - self.route.origin_alt_m,
            )
        )
        while self.samples and self.samples[0].t < now - self.history_s:
            self.samples.popleft()

    def distance_m(self) -> float | None:
        for sample in reversed(self.samples):
            if sample.distance_m is not None:
                return sample.distance_m
        return None

    def alarms(self, now: float) -> list[Alarm]:
        """Active alarms, most severe first."""
        found: list[Alarm] = []
        if self.last is None or self.last_rx is None:
            return [Alarm("NO TELEMETRY", "critical", "nothing received yet")]

        age = now - self.last_rx
        if age > LINK_TIMEOUT_S:
            found.append(Alarm("LINK LOST", "critical", f"no telemetry for {age:.1f} s"))

        t = self.last
        distance = self.distance_m()
        if t.gps_fix and distance is not None:
            if distance < MINIMUM_DISTANCE_M:
                found.append(
                    Alarm("TOO CLOSE TO LINE", "critical", f"{distance:.1f} m (minimum 10 m)")
                )
            elif distance < PROXIMITY_WARNING_M:
                found.append(Alarm("PROXIMITY", "warning", f"{distance:.1f} m from conductor"))
        if not t.node_a_fresh:
            found.append(Alarm("NODE A DATA STALE", "critical", f"age {t.node_a_age_ms} ms"))
        if not t.gps_fix:
            found.append(Alarm("GPS NO FIX", "warning", f"age {t.gps_age_ms} ms"))
        if not t.imu_valid:
            found.append(Alarm("IMU INVALID", "critical", "Node A reports no IMU data"))
        if not t.mag_ok:
            found.append(
                Alarm(
                    "MAGNETOMETER DISTURBED",
                    "warning",
                    f"field {t.field_mgauss} mG, heading from {t.heading_source}",
                )
            )
        found.extend(_onboard_alarms(t))
        if not t.recorder_ok:
            found.append(Alarm("RECORDER OFF", "warning", "flight data not being recorded"))

        while self._loss_times and self._loss_times[0] < now - RECENT_S:
            self._loss_times.popleft()
        if self._loss_times:
            found.append(Alarm("PACKET LOSS", "warning", f"{self.lost} packets lost in total"))

        while self._can_bad and self._can_bad[0][0] < now - RECENT_S:
            self._can_bad.popleft()
        if len(self._can_bad) >= 2 and self._can_bad[-1][1] > self._can_bad[0][1]:
            found.append(Alarm("CAN ERRORS", "warning", "frames rejected or lost on the CAN bus"))

        found.sort(key=lambda a: a.severity != "critical")
        return found


def _onboard_alarms(t: Telemetry) -> list[Alarm]:
    """Alarms raised from Node A's own safety state."""
    if t.flight_mode is None:
        if t.node_a_fresh:
            return [Alarm("SAFETY STATE UNKNOWN", "warning", "no safety report from Node A")]
        return []

    found: list[Alarm] = []
    if t.flight_mode in ("RETURN_TO_HOME", "LAND"):
        found.append(Alarm(t.flight_mode.replace("_", " "), "critical", "vehicle mode"))
    elif t.flight_mode == "HOLD":
        found.append(Alarm("HOLD", "warning", "vehicle holding position"))
    if t.link_lost:
        found.append(
            Alarm("VEHICLE LOST COMMAND LINK", "critical",
                  f"ground link age {t.ground_link_age_ms} ms at Node B")
        )
    if t.battery_critical:
        found.append(Alarm("BATTERY CRITICAL", "critical", f"{t.battery_pct} %"))
    elif t.battery_low:
        found.append(Alarm("BATTERY LOW", "warning", f"{t.battery_pct} %"))
    if not t.autopilot_ok:
        found.append(Alarm("AUTOPILOT SILENT", "critical", "no status from the autopilot"))
    if t.proximity:
        distance = "unknown" if t.distance_m is None else f"{t.distance_m:.1f} m"
        found.append(Alarm("AVOIDING", "warning", f"on-board distance {distance}"))
    return found
