"""Tests for the ground station's command framing."""

from ground_station.command import checksum, frame, reply_is_valid


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
