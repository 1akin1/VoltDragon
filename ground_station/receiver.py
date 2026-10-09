"""Prints Node B's UDP telemetry as it arrives.

Usage:
    python -m ground_station.receiver [--port 5600] [--count N]

Each valid packet is printed on one line; invalid datagrams and sequence gaps
are reported. Exits after --count packets, or on Ctrl+C.
"""

from __future__ import annotations

import argparse
import socket

from ground_station.telemetry import (
    DEFAULT_PORT,
    SequenceMonitor,
    Telemetry,
    TelemetryError,
    decode,
)


def format_packet(t: Telemetry) -> str:
    flags = "".join(
        mark if on else "-"
        for mark, on in (
            ("F", t.node_a_fresh),
            ("I", t.imu_valid),
            ("R", t.recorder_ok),
            ("G", t.gps_fix),
            ("M", t.mag_ok),
        )
    )
    return (
        f"#{t.seq:<6} B {t.node_b_uptime_ms / 1000:8.3f}s  "
        f"A age {t.node_a_age_ms:5} ms  [{flags}]  "
        f"pos {t.lat_deg:.6f} {t.lon_deg:.6f} {t.alt_msl_m:.1f} m  "
        f"hdg {t.heading_deg:6.2f} ({t.heading_source})  field {t.field_mgauss} mG  "
        f"can ok/bad/lost {t.can_valid}/{t.can_rejected}/{t.can_lost}"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--count", type=int, default=0, help="stop after N packets (0: forever)")
    args = parser.parse_args(argv)

    monitor = SequenceMonitor()
    invalid = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("", args.port))
        print(f"listening for telemetry on UDP port {args.port}")
        try:
            while args.count == 0 or monitor.received < args.count:
                datagram, sender = sock.recvfrom(2048)
                try:
                    packet = decode(datagram)
                except TelemetryError as error:
                    invalid += 1
                    print(f"invalid datagram from {sender[0]}: {error}")
                    continue
                missed = monitor.update(packet.seq)
                if missed:
                    print(f"  ({missed} packets lost)")
                print(format_packet(packet))
        except KeyboardInterrupt:
            pass
    print(f"received {monitor.received}, lost {monitor.lost}, invalid {invalid}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
