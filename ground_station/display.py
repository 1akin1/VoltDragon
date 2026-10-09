"""Ground-station display: map, telemetry plots and alarms (HLR-019).

Usage:
    python -m ground_station.display                       live, from UDP port 5600
    python -m ground_station.display --record flight.tlm   live, and keep the packets
    python -m ground_station.display --replay flight.tlm   play a recording back
    ... --snapshot view.png [--duration 60]                 save an image instead of a window

With live data the display also sends Node B a PING heartbeat once a second
(--target, or --no-heartbeat to stay silent and let the vehicle see link loss).

The map shows the route's pylons and conductors, the vehicle's track (red
where the magnetometer was disturbed) and its current position and heading.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time
from pathlib import Path

from ground_station.command import DEFAULT_TARGET, Heartbeat
from ground_station.state import (
    FIELD_EXPECTED_MG,
    FIELD_TOLERANCE,
    MINIMUM_DISTANCE_M,
    PROXIMITY_WARNING_M,
    GroundState,
)
from ground_station.telemetry import DEFAULT_PORT, TelemetryError, decode
from sim.plant.route import Route

REFRESH_S = 0.2
PLOT_WINDOW_S = 300.0
_RECORD = struct.Struct("<dH")      # receive time, datagram length; then the datagram

SEVERITY_COLOURS = {"critical": "#c62828", "warning": "#ef6c00"}
SOURCE_COLOURS = {"magnetometer": "#1565c0", "gps": "#6a1b9a", "none": "#9e9e9e"}


class Display:
    def __init__(self, route: Route, headless: bool) -> None:
        import matplotlib

        if headless:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        self.plt = plt
        self.route = route
        self.state = GroundState(route)
        self.t0: float | None = None

        self.fig = plt.figure(figsize=(15, 8.5))
        grid = self.fig.add_gridspec(4, 2, width_ratios=(1.35, 1.0), height_ratios=(1, 1, 1, 0.8))
        self.ax_map = self.fig.add_subplot(grid[0:3, 0])
        self.ax_alarms = self.fig.add_subplot(grid[3, 0])
        self.ax_dist = self.fig.add_subplot(grid[0, 1])
        self.ax_field = self.fig.add_subplot(grid[1, 1], sharex=self.ax_dist)
        self.ax_heading = self.fig.add_subplot(grid[2, 1], sharex=self.ax_dist)
        self.ax_status = self.fig.add_subplot(grid[3, 1])
        self._draw_static()

    def _draw_static(self) -> None:
        ax = self.ax_map
        for phase in self.route.conductors:
            for seg in phase:
                ax.plot([seg.a.x, seg.b.x], [seg.a.y, seg.b.y], color="#424242", lw=0.8)
        xs = [p[0] for p in self.route.pylons]
        ys = [p[1] for p in self.route.pylons]
        ax.plot(xs, ys, "s", color="#212121", ms=7, label="pylons")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        ax.set_title(f"Route {self.route.name}")
        ax.grid(True, alpha=0.3)
        (self.track_ok,) = ax.plot([], [], ".", color="#1565c0", ms=2, label="track")
        (self.track_bad,) = ax.plot([], [], ".", color="#c62828", ms=3,
                                    label="track, magnetometer disturbed")
        (self.vehicle,) = ax.plot([], [], "o", color="#2e7d32", ms=9, label="vehicle")
        (self.nose,) = ax.plot([], [], "-", color="#2e7d32", lw=2)
        ax.legend(loc="lower right", fontsize=8)

        self.ax_dist.set_ylabel("distance to\nconductor (m)")
        self.ax_dist.axhline(PROXIMITY_WARNING_M, color="#ef6c00", ls="--", lw=1)
        self.ax_dist.axhline(MINIMUM_DISTANCE_M, color="#c62828", ls="--", lw=1)
        (self.dist_line,) = self.ax_dist.plot([], [], color="#2e7d32")

        lo = FIELD_EXPECTED_MG * (1 - FIELD_TOLERANCE)
        hi = FIELD_EXPECTED_MG * (1 + FIELD_TOLERANCE)
        self.ax_field.axhspan(lo, hi, color="#c8e6c9", alpha=0.6)
        self.ax_field.set_ylabel("magnetic\nfield (mG)")
        (self.field_line,) = self.ax_field.plot([], [], color="#1565c0")

        self.ax_heading.set_ylabel("heading (deg)")
        self.ax_heading.set_xlabel("mission time (s)")
        self.ax_heading.set_ylim(0, 360)
        self.heading_points = {
            src: self.ax_heading.plot([], [], ".", ms=3, color=col, label=src)[0]
            for src, col in SOURCE_COLOURS.items()
        }
        self.ax_heading.legend(loc="upper right", fontsize=8, ncol=3)

        for ax in (self.ax_alarms, self.ax_status):
            ax.axis("off")
        self.fig.tight_layout()

    def feed(self, datagram: bytes, now: float) -> None:
        try:
            packet = decode(datagram)
        except TelemetryError:
            return
        if self.t0 is None:
            self.t0 = now
        self.state.update(packet, now)

    def render(self, now: float) -> None:
        s = list(self.state.samples)
        ok =[(p.east, p.north) for p in s if p.east is not None and p.mag_ok]
        bad = [(p.east, p.north) for p in s if p.east is not None and not p.mag_ok]
        self.track_ok.set_data([p[0] for p in ok], [p[1] for p in ok])
        self.track_bad.set_data([p[0] for p in bad], [p[1] for p in bad])

        positioned = [p for p in s if p.east is not None]
        if positioned:
            last = positioned[-1]
            heading = math.radians(last.heading_deg)
            self.vehicle.set_data([last.east], [last.north])
            self.nose.set_data(
                [last.east, last.east + 40.0 * math.sin(heading)],
                [last.north, last.north + 40.0 * math.cos(heading)],
            )

        # Plots use mission time (Node B's uptime), which keeps its scale even when the
        # simulation runs slower than real time.
        times = [p.mission_t for p in s]
        dist = [(p.mission_t, p.distance_m) for p in s if p.distance_m is not None]
        self.dist_line.set_data([d[0] for d in dist], [d[1] for d in dist])
        self.field_line.set_data(times, [p.field_mgauss for p in s])
        for src, artist in self.heading_points.items():
            pts = [(p.mission_t, p.heading_deg) for p in s if p.heading_source == src]
            artist.set_data([q[0] for q in pts], [q[1] for q in pts])
        for ax in (self.ax_dist, self.ax_field):
            ax.relim()
            ax.autoscale_view()
        if times:
            self.ax_heading.set_xlim(max(0.0, times[-1] - PLOT_WINDOW_S), max(10.0, times[-1]))

        self._render_text(now)

    def _render_text(self, now: float) -> None:
        self.ax_alarms.clear()
        self.ax_alarms.axis("off")
        self.ax_alarms.set_title("Alarms", loc="left", fontsize=10)
        alarms = self.state.alarms(now)
        if not alarms:
            self.ax_alarms.text(0.0, 0.8, "none", color="#2e7d32", fontsize=11,
                                transform=self.ax_alarms.transAxes)
        for i, alarm in enumerate(alarms[:5]):
            self.ax_alarms.text(
                0.0, 0.8 - 0.18 * i, f"{alarm.name}: {alarm.detail}",
                color=SEVERITY_COLOURS[alarm.severity], fontsize=11, fontweight="bold",
                transform=self.ax_alarms.transAxes,
            )

        self.ax_status.clear()
        self.ax_status.axis("off")
        t = self.state.last
        if t is None:
            return
        lines = [
            f"packets {self.state.received}, lost {self.state.lost}   "
            f"telemetry seq {t.seq}",
            f"position {t.lat_deg:.6f}, {t.lon_deg:.6f}, {t.alt_msl_m:.1f} m MSL  "
            f"({t.gps_satellites} satellites)",
            f"heading {t.heading_deg:.1f} deg from {t.heading_source}, "
            f"speed {t.speed_mps:.1f} m/s",
            f"CAN valid/rejected/lost {t.can_valid}/{t.can_rejected}/{t.can_lost}   "
            f"Node A age {t.node_a_age_ms} ms",
            f"mode {t.flight_mode or '?'}   battery "
            f"{'?' if t.battery_pct is None else t.battery_pct} %   on-board distance "
            f"{'?' if t.distance_m is None else f'{t.distance_m:.1f}'} m   "
            f"request {t.last_request_id}",
        ]
        for i, line in enumerate(lines):
            self.ax_status.text(0.0, 0.88 - 0.19 * i, line, fontsize=9.5, family="monospace",
                                transform=self.ax_status.transAxes)

    def save(self, path: Path) -> None:
        self.fig.savefig(path, dpi=110)


def _replay(display: Display, path: Path) -> float:
    data = path.read_bytes()
    offset = 0
    now = 0.0
    while offset + _RECORD.size <= len(data):
        now, length = _RECORD.unpack_from(data, offset)
        offset += _RECORD.size
        display.feed(data[offset:offset + length], now)
        offset += length
    return now


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--route", default="line_a")
    parser.add_argument("--record", type=Path, help="append received packets to this file")
    parser.add_argument("--replay", type=Path, help="show a recording instead of live data")
    parser.add_argument("--snapshot", type=Path, help="save an image instead of opening a window")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="with --snapshot and live data: seconds to listen")
    parser.add_argument("--target", default=DEFAULT_TARGET, help="Node B address for the heartbeat")
    parser.add_argument("--no-heartbeat", action="store_true",
                        help="do not send the 1 Hz PING heartbeat")
    args = parser.parse_args(argv)

    display = Display(Route.load(args.route), headless=args.snapshot is not None)

    if args.replay:
        end = _replay(display, args.replay)
        display.render(end)
        if args.snapshot:
            display.save(args.snapshot)
        else:
            display.plt.show()
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.setblocking(False)
    record = args.record.open("ab") if args.record else None
    heartbeat = None if args.no_heartbeat else Heartbeat(args.target)

    def poll() -> None:
        if heartbeat is not None:
            heartbeat.poll(time.monotonic())
        while True:
            try:
                datagram = sock.recv(2048)
            except BlockingIOError:
                return
            now = time.monotonic()
            display.feed(datagram, now)
            if record is not None:
                record.write(_RECORD.pack(now, len(datagram)) + datagram)

    try:
        if args.snapshot:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                poll()
                time.sleep(0.05)
            display.render(time.monotonic())
            display.save(args.snapshot)
            state = display.state
            print(f"{state.received} packets, {state.lost} lost; saved {args.snapshot}")
        else:
            timer = display.fig.canvas.new_timer(interval=int(REFRESH_S * 1000))

            def tick() -> None:
                poll()
                display.render(time.monotonic())
                display.fig.canvas.draw_idle()

            timer.add_callback(tick)
            timer.start()
            display.plt.show()
    finally:
        sock.close()
        if heartbeat is not None:
            heartbeat.close()
        if record is not None:
            record.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
