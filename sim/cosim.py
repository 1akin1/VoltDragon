"""Co-simulation: the Python plant model drives Node A's sensors inside Renode.

Renode runs headless with its monitor on a TCP port. Each step of STEP_S
seconds, the plant's 100 Hz accelerometer and gyroscope samples for the next
step are queued in the sensor model (one sample per firmware read); the
magnetometer value and any GPS sentences for the step about to run are applied;
then the emulation runs for one step. Queuing one step ahead keeps the inertial
queue from ever running dry. The magnetometer model ignores queued samples, so
it is updated once per step (10 Hz), which is ample for a field that changes
over metres.

Renode's LSM9DS1 model scales its outputs by ideal counts per unit rather than
the datasheet sensitivities the firmware uses (see tests/robot/node_a_imu.robot),
so samples are pre-scaled here and the firmware reads the plant's true values.
"""

from __future__ import annotations

import csv
import math
import re
import socket
import subprocess
import time
from collections import deque
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

from ground_station.command import frame
from sim.plant.autopilot import MODES
from sim.plant.plant import STEP_S as PLANT_STEP_S
from sim.plant.plant import Plant, PlantStep

REPO = Path(__file__).resolve().parent.parent
STEP_S = 0.1                                    # co-simulation step (10 plant steps)
SAMPLES_PER_STEP = round(STEP_S / PLANT_STEP_S)

# Firmware reading per unit fed into Renode's LSM9DS1 model (measured in Phase 2):
#   accel 16384 LSB/g x 0.061 mg/LSB, gyro 120 LSB/dps x 8.75 mdps/LSB,
#   mag 8192 LSB/gauss x 0.14 mgauss/LSB.
ACCEL_GAIN = 16384 * 0.061 / 1000.0
GYRO_GAIN = 120 * 8.75 / 1000.0
MAG_GAIN = 8192 * 0.14 / 1000.0


class RenodeError(RuntimeError):
    pass


class RenodeMonitor:
    """Minimal client for Renode's monitor over TCP."""

    def __init__(self, port: int = 4567, log_path: Path | None = None) -> None:
        self.port = port
        self.log = log_path.open("w", encoding="utf-8") if log_path else subprocess.DEVNULL
        self.process = subprocess.Popen(
            ["renode", "--disable-gui", "--plain", "--port", str(port)],
            stdout=self.log,
            stderr=subprocess.STDOUT,
        )
        self.sock = self._connect()
        self.buffer = b""
        self.counter = 0
        self.execute("logLevel 3")

    def _connect(self) -> socket.socket:
        deadline = time.monotonic() + 30.0
        while True:
            try:
                sock = socket.create_connection(("127.0.0.1", self.port), timeout=120.0)
                return sock
            except OSError:
                if time.monotonic() > deadline or self.process.poll() is not None:
                    raise RenodeError("Renode monitor did not open its port") from None
                time.sleep(0.2)

    def execute(self, command: str) -> str:
        """Runs one monitor command and returns its output."""
        self.counter += 1
        marker = f"__done_{self.counter}__".encode()
        # The monitor echoes command lines, so the marker also appears inside the echoed
        # 'echo "..."' line; only a line consisting of the marker itself ends the output.
        # Lines are separated telnet-style, with "\n\r" before and "\r\r\n" after output.
        end = re.compile(rb"[\r\n]" + re.escape(marker) + rb"\r*\n")
        self.sock.sendall(command.encode() + b'\necho "' + marker + b'"\n')
        while (match := end.search(self.buffer)) is None:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RenodeError(f"Renode closed the monitor during: {command}")
            self.buffer += chunk
        output = self.buffer[: match.start()]
        self.buffer = self.buffer[match.end():]
        text = output.decode(errors="replace")
        if "There was an error executing command" in text:
            raise RenodeError(f"{command}\n{text}")
        return text

    def close(self) -> None:
        try:
            self.sock.sendall(b"quit\n")
            self.sock.close()
        except OSError:
            pass
        try:
            self.process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.process.kill()
        if self.log is not subprocess.DEVNULL:
            self.log.close()


@contextmanager
def renode_monitor(port: int = 4567, log_path: Path | None = None) -> Iterator[RenodeMonitor]:
    monitor = RenodeMonitor(port, log_path)
    try:
        yield monitor
    finally:
        monitor.close()


# Full-scale ranges configured by the firmware (lsm9ds1.h). A real sensor clips at its
# range; Renode's model aborts the emulation instead, so values are clipped here.
ACCEL_RANGE_G = 2.0
GYRO_RANGE_DPS = 245.0
MAG_RANGE_GAUSS = 4.0


def _clip(value: float, limit: float) -> float:
    margin = limit * 0.999
    return max(-margin, min(margin, value))


def _inertial_text(step: PlantStep) -> str:
    a, g = step.imu.accel_g, step.imu.gyro_dps
    values = (
        *(_clip(v, ACCEL_RANGE_G) / ACCEL_GAIN for v in a),
        *(_clip(v, GYRO_RANGE_DPS) / GYRO_GAIN for v in g),
    )
    return ",".join(f"{v:.6f}" for v in values)


def _mag_text(step: PlantStep) -> str:
    return ",".join(f"{_clip(v, MAG_RANGE_GAUSS) / MAG_GAIN:.6f}" for v in step.imu.mag_gauss)


@dataclass
class CoSimulation:
    """Runs the plant and the emulation in lockstep.

    Accelerometer and gyroscope samples are queued in the sensor model two steps
    ahead, because the model hands them out one per firmware read. The
    magnetometer value, the GPS sentences and the autopilot's status belong to the
    step about to run and are applied just before it.

    Node A's orders to the autopilot (its UART5 output) reach the plant through
    a file that Renode writes as they are sent (@autopilot_path); they are read
    after each step, so they act on the plant one to two steps later, like a
    real link with a little latency.

    The ground station's heartbeat, a PING once a second into Node B's command
    UART, is sent while the time is below @heartbeat_until_s.
    """

    monitor: RenodeMonitor
    plant: Plant
    truth_path: Path | None = None
    autopilot_path: Path | None = None
    heartbeat_until_s: float = math.inf

    def __post_init__(self) -> None:
        self.time_s = 0.0
        self._pending: deque[list[PlantStep]] = deque()
        self._autopilot_offset = 0
        self._next_heartbeat_s = 1.0
        self._heartbeat_seq = 0
        self._truth_file = None
        self._truth = None
        if self.truth_path is not None:
            self.truth_path.parent.mkdir(parents=True, exist_ok=True)
            self._truth_file = self.truth_path.open("w", newline="", encoding="utf-8")
            self._truth = csv.writer(self._truth_file)
            self._truth.writerow(
                ["t", "east", "north", "up", "lat", "lon", "alt_msl", "heading_deg",
                 "distance_to_line_m", "line_field_gauss", "field_gauss", "s",
                 "mode", "airborne", "battery_pct", "keep_out_m"]
            )

    def _plant_step_block(self) -> list[PlantStep]:
        return [self.plant.step() for _ in range(SAMPLES_PER_STEP)]

    def _feed(self, queue: list[PlantStep], current: list[PlantStep]) -> None:
        """Queues @queue's inertial samples and applies @current's magnetometer and GPS data."""
        samples = ";".join(_inertial_text(s) for s in queue)
        gps = "".join(s.gps.sentences for s in current if s.gps is not None)
        gps_hex = gps.encode("ascii").hex() if gps else "-"
        mag = _mag_text(current[len(current) // 2])
        status = "".join(s.autopilot_tx for s in current if s.autopilot_tx is not None)
        status_hex = status.encode("ascii").hex() if status else "-"
        self.monitor.execute(f'plant_feed "{samples}" "{gps_hex}" "{mag}" "{status_hex}"')

    def _read_autopilot_orders(self) -> None:
        """Passes what Node A sent to the autopilot since the last step to the plant."""
        if self.autopilot_path is None or not self.autopilot_path.exists():
            return
        with self.autopilot_path.open("rb") as f:
            f.seek(self._autopilot_offset)
            data = f.read()
        self._autopilot_offset += len(data)
        if data:
            self.plant.receive(data.decode("ascii", errors="replace"))

    def _heartbeat(self) -> None:
        if self.time_s + 1e-9 < self._next_heartbeat_s or self.time_s >= self.heartbeat_until_s:
            return
        self._next_heartbeat_s += 1.0
        self._heartbeat_seq = self._heartbeat_seq % 65535 + 1
        ping = (frame(self._heartbeat_seq, "PING") + "\r\n").encode("ascii").hex()
        self.monitor.execute('mach set "node_b"')
        self.monitor.execute(f'uart_write "sysbus.usart3" "{ping}"')
        self.monitor.execute('mach set "node_a"')

    def _record(self, block: list[PlantStep]) -> None:
        if self._truth is None:
            return
        for s in block[::SAMPLES_PER_STEP // 2]:   # every 50 ms is plenty for plots
            st = s.state
            lat, lon, alt = self.plant.route.enu_to_geodetic(st.position)
            x_axis = st.attitude.c0
            heading = math.degrees(math.atan2(x_axis.x, x_axis.y)) % 360.0
            keep_out = self.plant.autopilot.keep_out_m
            self._truth.writerow(
                [f"{st.t:.3f}", f"{st.position.x:.2f}", f"{st.position.y:.2f}",
                 f"{st.position.z:.2f}", f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}",
                 f"{heading:.2f}", f"{st.distance_to_line_m:.2f}",
                 f"{s.imu.line_field_gauss:.4f}", f"{s.imu.mag_gauss.norm():.4f}",
                 f"{st.s_m:.2f}", MODES.index(st.mode), int(st.airborne),
                 f"{self.plant.battery.pct:.2f}", f"{keep_out or 0.0:.1f}"]
            )

    def start(self) -> None:
        """Queues the first step of inertial data before the emulation starts."""
        self.monitor.execute('mach set "node_a"')
        first = self._plant_step_block()
        self._pending.append(first)
        self.monitor.execute(
            f'plant_feed "{";".join(_inertial_text(s) for s in first)}" "-" "{_mag_text(first[0])}"'
        )

    def step(self) -> None:
        """Runs one co-simulation step, keeping the inertial queue one step ahead."""
        current = self._pending.popleft()
        upcoming = self._plant_step_block()
        self._pending.append(upcoming)
        self._feed(upcoming, current)
        self._heartbeat()
        self.monitor.execute(f'emulation RunFor "{STEP_S}"')
        self.time_s += STEP_S
        self._read_autopilot_orders()
        self._record(current)

    def close(self) -> None:
        if self._truth_file is not None:
            self._truth_file.close()
