"""Runs the inspection mission: the plant model flies along the line and drives Node A in Renode.

Usage (Linux/WSL, from the repository root, firmware built with the debug preset):
    python -m sim.mission [--scenario nominal] [--duration 90] [--start 0] [--out DIR] [--tap]

Scenarios:
    nominal        the inspection pass; the close part of the plan comes within 4 m
                   of a conductor, so Node A has the autopilot keep its distance
    no-avoidance   the same with an autopilot that ignores avoidance orders (fault
                   injection): the vehicle flies the plan as written
    link-loss      the ground station's heartbeat stops after 12 s
    battery        starts with 22 % battery, draining at 0.5 %/s
    prop-damage    a propeller is damaged at 15 s (vibration fault, HLR-009)
    bearing-wear   a motor bearing fails at 15 s (vibration fault, HLR-009)

Writes to the output directory:
    node_a.log, node_b.log   the two nodes' debug consoles
    autopilot_rx.log         what Node A sent to the autopilot
    truth.csv                the plant's ground truth every 50 ms
    renode.log               Renode's own log (warnings and errors)

The ground station's heartbeat (PING once a second) is simulated on Node B's
command UART. With --tap, Node B's Ethernet is bridged to tap0 instead, so a
ground station on the host receives the telemetry and sends the heartbeat
itself (Renode then needs root to create the TAP device):
    sudo ip addr add 192.168.10.1/24 dev tap0 && sudo ip link set tap0 up
    python -m ground_station.display
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import time
from pathlib import Path

from sim.cosim import REPO, STEP_S, CoSimulation, renode_monitor
from sim.plant.plant import Plant, Scenario
from sim.plant.vibration import VibrationFault

REPORT_EVERY_S = 10.0

# name: (plant scenario, simulated heartbeat stops at this time)
SCENARIOS = {
    "nominal": (Scenario(), math.inf),
    "no-avoidance": (Scenario(obey_avoidance=False), math.inf),
    "link-loss": (Scenario(), 12.0),
    "battery": (Scenario(battery_pct=22.0, battery_drain_pct_per_s=0.5), math.inf),
    "prop-damage": (
        Scenario(vibration_fault=VibrationFault("imbalance", onset_s=15.0, rotor=2)), math.inf
    ),
    "bearing-wear": (
        Scenario(vibration_fault=VibrationFault("bearing", onset_s=15.0, rotor=1)), math.inf
    ),
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="nominal")
    parser.add_argument("--start", type=float, default=0.0,
                        help="along-track position at the start, m (close pass: 210 m)")
    parser.add_argument("--duration", type=float, default=90.0, help="simulated seconds")
    parser.add_argument("--out", type=Path, default=REPO / "build" / "mission")
    parser.add_argument("--tap", action="store_true", help="bridge Node B to tap0")
    parser.add_argument("--port", type=int, default=4567, help="Renode monitor port")
    args = parser.parse_args(argv)

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    for name in ("node_a.log", "node_b.log", "autopilot_rx.log"):
        (out / name).unlink(missing_ok=True)

    scenario, heartbeat_until_s = SCENARIOS[args.scenario]
    scenario = dataclasses.replace(
        scenario, plan=dataclasses.replace(scenario.plan, start_s_m=args.start)
    )
    if args.tap:
        heartbeat_until_s = -1.0        # the ground station on the host sends it
    plant = Plant(scenario)
    with renode_monitor(args.port, out / "renode.log") as monitor:
        monitor.execute(f"include @{REPO / 'renode' / 'system.resc'}")
        if args.tap:
            monitor.execute(f"include @{REPO / 'renode' / 'tap_link.resc'}")
        monitor.execute(f"include @{REPO / 'renode' / 'plant_feed.py'}")
        for node in ("node_a", "node_b"):
            monitor.execute(f'mach set "{node}"')
            monitor.execute(f"sysbus.usart2 CreateFileBackend @{out / (node + '.log')} true")
        autopilot_path = out / "autopilot_rx.log"
        monitor.execute('mach set "node_a"')
        monitor.execute(f"sysbus.uart5 CreateFileBackend @{autopilot_path} true")

        cosim = CoSimulation(
            monitor,
            plant,
            truth_path=out / "truth.csv",
            autopilot_path=autopilot_path,
            heartbeat_until_s=heartbeat_until_s,
        )
        cosim.start()
        started = time.monotonic()
        next_report = REPORT_EVERY_S
        steps = round(args.duration / STEP_S)
        try:
            for _ in range(steps):
                cosim.step()
                if cosim.time_s + 1e-9 >= next_report:
                    next_report += REPORT_EVERY_S
                    vehicle = plant.vehicle
                    distance = plant.route.distance_to_conductors(vehicle.position)
                    wall = time.monotonic() - started
                    print(
                        f"t={cosim.time_s:6.1f} s  {vehicle.mode:7}  s {vehicle.s:6.1f} m  "
                        f"distance to line {distance:5.1f} m  battery {plant.battery.pct:4.1f} %  "
                        f"({cosim.time_s / wall:.2f}x real time)",
                        flush=True,
                    )
        finally:
            cosim.close()

    print(f"done: logs and ground truth in {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
