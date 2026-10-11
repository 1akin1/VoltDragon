# Records Node B's UDP telemetry inside the emulation, without a TAP device (no root).
#
# Load with:  include @renode/telemetry_capture.py
# Then, with Node B selected (mach set "node_b"):
#   tlm_capture @<path>
#
# Every frame Node B's Ethernet MAC sends to UDP port 5600 is appended to <path> in
# the ground station's recording format (ground_station/display.py): a little-endian
# double (time in seconds), an unsigned 16-bit length, then the datagram. The time is
# Node B's virtual time, so a recording replays at mission speed however slowly the
# simulation ran.
#
# Renode turns functions named mc_<name> into monitor commands; this runs in
# Renode's IronPython 2.7.

import struct

TELEMETRY_PORT = 5600
ETHERNET_HEADER = 14
UDP_HEADER = 8
PROTOCOL_UDP = 17

_captures = []


def _udp_payload(frame, port):
    """The payload of an IPv4 UDP datagram to <port> in an Ethernet frame, or None."""
    if len(frame) < ETHERNET_HEADER + 20 + UDP_HEADER:
        return None
    if frame[12] != 0x08 or frame[13] != 0x00 or frame[ETHERNET_HEADER + 9] != PROTOCOL_UDP:
        return None
    udp = ETHERNET_HEADER + (frame[ETHERNET_HEADER] & 0x0F) * 4
    if ((frame[udp + 2] << 8) | frame[udp + 3]) != port:
        return None
    length = ((frame[udp + 4] << 8) | frame[udp + 5]) - UDP_HEADER
    return frame[udp + UDP_HEADER:udp + UDP_HEADER + length]


def mc_tlm_capture(path):
    machine = monitor.Machine  # noqa: F821
    out = open(str(path), "wb")

    def on_frame(ethernet_frame):
        frame = [int(b) for b in ethernet_frame.Bytes]
        payload = _udp_payload(frame, TELEMETRY_PORT)
        if payload is None:
            return
        seconds = machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds
        out.write(struct.pack("<dH", seconds, len(payload)) + str(bytearray(payload)))
        out.flush()

    machine["sysbus.ethernet"].FrameReady += on_frame
    # Keep the handler and the file alive for as long as the emulation runs.
    _captures.append((on_frame, out))
