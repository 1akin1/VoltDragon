"""Tests for the ground station's command framing and heartbeat."""

import socket
import time

from ground_station.command import Heartbeat, checksum, frame, reply_is_valid


def test_frame_matches_firmware_examples() -> None:
    # Checksums also used by the firmware tests (tests/unit, tests/robot).
    assert frame(1, "PING") == "$1,PING*0D"
    assert frame(1, "TLM_RATE", "50") == "$1,TLM_RATE,50*3C"
    assert frame(1, "CAN") == "$1,CAN*51"


def test_checksum_is_xor_of_body() -> None:
    assert checksum("") == 0
    assert checksum("A") == 0x41


def test_reply_validation() -> None:
    assert reply_is_valid("$1,ACK,PING*68")
    assert not reply_is_valid("$1,ACK,PING*69")
    assert not reply_is_valid("1,ACK,PING*68")
    assert not reply_is_valid("$1,ACK,PING*ZZ")
    assert not reply_is_valid("")


def test_heartbeat_pings_once_a_second_and_counts_replies() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as node_b:
        node_b.bind(("127.0.0.1", 0))
        node_b.settimeout(1.0)
        beat = Heartbeat("127.0.0.1", node_b.getsockname()[1])
        try:
            beat.poll(100.0)
            beat.poll(100.5)                    # not due yet
            beat.poll(101.0)
            first, sender = node_b.recvfrom(256)
            second, _ = node_b.recvfrom(256)
            assert (first, second) == (b"$1,PING*0D", frame(2, "PING").encode())
            node_b.sendto(b"$1,ACK,PING*68", sender)
            node_b.sendto(b"$2,ACK,PING*00", sender)  # bad checksum: not counted
            node_b.settimeout(0.2)
            for _ in range(20):
                beat.poll(101.1)
                if beat.acknowledged:
                    break
                time.sleep(0.01)
            assert (beat.sent, beat.acknowledged) == (2, 1)
        finally:
            beat.close()
