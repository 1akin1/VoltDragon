"""Runs the inspection mission: the plant model flies along the line and drives Node A in Renode.

Usage (Linux/WSL, from the repository root, firmware built with the debug preset):
    python -m sim.mission [--duration 90] [--out build/mission] [--tap]

Writes to the output directory:
    node_a.log, node_b.log   the two nodes' debug consoles
    truth.csv                the plant's ground truth every 50 ms
    renode.log               Renode's own log (warnings and errors)

With --tap, Node B's Ethernet is bridged to tap0 so a ground station on the
host receives the telemetry (Renode then needs root to create the TAP device):
    sudo ip addr add 192.168.10.1/24 dev tap0 && sudo ip link set tap0 up
    python -m ground_station.display
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from sim.cosim import REPO, STEP_S, CoSimulation, renode_monitor
from sim.plant.plant import Plant, Scenario

REPORT_EVERY_S = 10.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--duration", type=float, default=90.0, help="simulated seconds")
    parser.add_argument("--out", type=Path, default=REPO / "build" / "mission")
    parser.add_argument("--tap", action="store_true", help="bridge Node B to tap0")
    parser.add_argument("--port", type=int, default=4567, help="Renode monitor port")
    args = parser.parse_args(argv)

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    for name in ("node_a.log", "node_b.log"):
        (out / name).unlink(missing_ok=True)

    plant = Plant(Scenario())
    with renode_monitor(args.port, out / "renode.log") as monitor:
        monitor.execute(f"include @{REPO / 'renode' / 'system.resc'}")
        if args.tap:
            monitor.execute(f"include @{REPO / 'renode' / 'tap_link.resc'}")
        monitor.execute(f"include @{REPO / 'renode' / 'plant_feed.py'}")
        for node in ("node_a", "node_b"):
            monitor.execute(f'mach set "{node}"')
            monitor.execute(f"sysbus.usart2 CreateFileBackend @{out / (node + '.log')} true")

        cosim = CoSimulation(monitor, plant, truth_path=out / "truth.csv")
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
                        f"t={cosim.time_s:6.1f} s  along-track {vehicle.s:6.1f} m  "
                        f"distance to line {distance:5.1f} m  "
                        f"({cosim.time_s / wall:.2f}x real time)",
                        flush=True,
                    )
        finally:
            cosim.close()

    print(f"done: logs and ground truth in {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
