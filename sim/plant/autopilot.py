"""Stand-in for the flight controller: obeys Node A's orders and reports its battery.

Node A talks to it over UART5 (docs/autopilot-link.md):

    Node A -> autopilot   $VDCMD,<MISSION|HOLD|RTH|LAND>,<avoid 0/1>,<min_distance_dm>*hh
    autopilot -> Node A   $VDAPS,<battery_pct>,<MISSION|HOLD|RTH|LAND>*hh   (5 Hz)

The autopilot flies the mode it was last ordered (MISSION until the first
order). An avoidance order sets a keep-out distance from the conductors; the
autopilot keeps it until the order is withdrawn *and* its own flight plan no
longer comes closer than that, so a withdrawn order does not send it straight
back towards the line.

The battery drains at a fixed rate while airborne; it is a scenario parameter,
so tests can start a flight with a nearly empty battery.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass

MODES = ("MISSION", "HOLD", "RTH", "LAND")
STATUS_PERIOD_S = 0.2
KEEP_OUT_MARGIN_M = 1.0     # flies this much beyond the ordered minimum distance


def checksum(body: str) -> int:
    return functools.reduce(lambda acc, ch: acc ^ ord(ch), body, 0)


def sentence(body: str) -> str:
    return f"${body}*{checksum(body):02X}\r\n"


@dataclass(frozen=True)
class Command:
    mode: str
    avoid: bool
    min_distance_m: float


def parse_command(line: str) -> Command | None:
    """Parses one $VDCMD line; None if it is not a valid command."""
    line = line.strip()
    if not line.startswith("$") or len(line) < 4 or line[-3] != "*":
        return None
    body = line[1:-3]
    try:
        if int(line[-2:], 16) != checksum(body):
            return None
    except ValueError:
        return None
    fields = body.split(",")
    if len(fields) != 4 or fields[0] != "VDCMD" or fields[1] not in MODES:
        return None
    if fields[2] not in ("0", "1") or not fields[3].isdigit():
        return None
    return Command(fields[1], fields[2] == "1", int(fields[3]) / 10.0)


@dataclass
class Battery:
    pct: float = 95.0
    drain_pct_per_s: float = 0.05       # about 30 minutes of flight

    def step(self, dt: float, airborne: bool) -> None:
        if airborne:
            self.pct = max(0.0, self.pct - self.drain_pct_per_s * dt)


class Autopilot:
    """Receives Node A's orders, keeps the vehicle's mode and keep-out, sends status."""

    def __init__(self, battery: Battery, obey_avoidance: bool = True) -> None:
        self.battery = battery
        self.obey_avoidance = obey_avoidance
        self.mode = "MISSION"
        self.avoid = False
        self.keep_out_m: float | None = None
        self.commands = 0
        self.rejected = 0
        self._rx = ""
        self._next_status_t = 0.0

    def receive(self, text: str) -> list[Command]:
        """Feeds bytes from Node A's UART; applies and returns the complete commands."""
        self._rx += text
        *lines, self._rx = self._rx.split("\n")
        applied = []
        for line in lines:
            if not line.strip():
                continue
            command = parse_command(line)
            if command is None:
                self.rejected += 1
                continue
            self.commands += 1
            self.apply(command)
            applied.append(command)
        return applied

    def apply(self, command: Command) -> None:
        self.mode = command.mode
        self.avoid = command.avoid and self.obey_avoidance
        if self.avoid:
            wanted = command.min_distance_m + KEEP_OUT_MARGIN_M
            self.keep_out_m = max(self.keep_out_m or 0.0, wanted)

    def release_keep_out(self, plan_distance_m: float) -> None:
        """Drops the keep-out once avoidance is withdrawn and the plan respects it anyway."""
        if self.keep_out_m is not None and not self.avoid and plan_distance_m >= self.keep_out_m:
            self.keep_out_m = None

    def status(self, t: float) -> str | None:
        """The $VDAPS sentence if one is due at time @t."""
        if t + 1e-9 < self._next_status_t:
            return None
        self._next_status_t += STATUS_PERIOD_S
        return sentence(f"VDAPS,{int(self.battery.pct)},{self.mode}")
