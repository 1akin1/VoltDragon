"""Sends one operator command to Node B over UDP and prints the reply.

Usage:
    python -m ground_station.command [--target 192.168.10.2] [--seq N] VERB [ARG ...]

Example:
    python -m ground_station.command TLM_RATE 50

The frame format is described in docs/command-interface.md.
"""

from __future__ import annotations

import argparse
import functools
import socket

DEFAULT_TARGET = "192.168.10.2"
COMMAND_PORT = 5601


def checksum(body: str) -> int:
    """XOR of all characters between '$' and '*'."""
    return functools.reduce(lambda acc, ch: acc ^ ord(ch), body, 0)


def frame(seq: int, verb: str, *args: str) -> str:
    """Builds "$<seq>,<VERB>[,<arg>...]*<CK>"."""
    body = ",".join([str(seq), verb, *args])
    return f"${body}*{checksum(body):02X}"


def reply_is_valid(reply: str) -> bool:
    """Checks the framing and checksum of a reply."""
    if not reply.startswith("$") or len(reply) < 4 or reply[-3] != "*":
        return False
    try:
        return int(reply[-2:], 16) == checksum(reply[1:-3])
    except ValueError:
        return False


class Heartbeat:
    """Sends PING to Node B once a second, so the vehicle knows the ground station is there.

    Node B counts every intact command as ground contact; if none arrives for
    3 s Node A returns home (HLR-003). Replies are read and counted, never waited for.
    """

    PERIOD_S = 1.0

    def __init__(self, target: str = DEFAULT_TARGET, port: int = COMMAND_PORT) -> None:
        self.address = (target, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))     # replies come back to this port; Windows needs it bound
        self.sock.setblocking(False)
        self.seq = 0
        self.sent = 0
        self.acknowledged = 0
        self._next = 0.0

    def poll(self, now: float) -> None:
        """Sends a PING if one is due and counts the replies received so far."""
        while True:
            try:
                reply = self.sock.recv(256).decode("ascii", errors="replace").strip()
            except OSError:     # nothing waiting, or an ICMP error from a closed port
                break
            if reply_is_valid(reply) and ",ACK,PING" in reply:
                self.acknowledged += 1
        if now < self._next:
            return
        self._next = now + self.PERIOD_S
        self.seq = self.seq % 65535 + 1
        try:
            self.sock.sendto(frame(self.seq, "PING").encode("ascii"), self.address)
            self.sent += 1
        except OSError:
            pass                    # no route yet (TAP down): the vehicle will see link loss

    def close(self) -> None:
        self.sock.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", default=DEFAULT_TARGET)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("verb")
    parser.add_argument("args", nargs="*")
    args = parser.parse_args(argv)

    request = frame(args.seq, args.verb, *args.args)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(args.timeout)
        sock.sendto(request.encode("ascii"), (args.target, COMMAND_PORT))
        print(f"-> {request}")
        try:
            reply = sock.recv(256).decode("ascii", errors="replace").strip()
        except TimeoutError:
            print("no reply")
            return 1
    print(f"<- {reply}{'' if reply_is_valid(reply) else '  (bad checksum)'}")
    return 0 if ",ACK," in reply else 2


if __name__ == "__main__":
    raise SystemExit(main())
