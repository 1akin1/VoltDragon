# Simulated ground station for tests: injects frames into the current machine's
# Ethernet MAC, as if they arrived from a ground station at 192.168.10.1.
#
# Load with:  include @renode/ground_station.py
# Then, with Node B selected (mach set "node_b"):
#   gs_arp                       ARP request from the ground station, so Node B learns its MAC
#   gs_udp <port> "<text>"       UDP datagram from the ground station's port 5700
#
# Renode turns functions named mc_<name> into monitor commands. Renode runs
# IronPython 2.7, so frames are built as lists of integers rather than bytes.

from Antmicro.Renode.Network import EthernetFrame
from System import Array, Byte

GS_MAC = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01]
GS_IP = [192, 168, 10, 1]
GS_PORT = 5700
NODE_B_MAC = [0x02, 0x00, 0x00, 0x56, 0x44, 0x02]
NODE_B_IP = [192, 168, 10, 2]
ETHERTYPE_IPV4 = [0x08, 0x00]
ETHERTYPE_ARP = [0x08, 0x06]


def _u16(value):
    return [(value >> 8) & 0xFF, value & 0xFF]


def _checksum(data):
    if len(data) % 2:
        data = data + [0]
    total = 0
    for i in range(0, len(data), 2):
        total += (data[i] << 8) | data[i + 1]
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return ~total & 0xFFFF


def _inject(frame):
    ok, ethernet_frame = EthernetFrame.TryCreateEthernetFrame(Array[Byte](frame), True)
    if not ok:
        raise Exception("could not build Ethernet frame")
    # `monitor` is provided by Renode's Python environment.
    monitor.Machine["sysbus.ethernet"].ReceiveFrame(ethernet_frame)  # noqa: F821


def mc_gs_arp():
    # Hardware type Ethernet, protocol IPv4, address lengths 6 and 4, operation request.
    arp = _u16(1) + _u16(0x0800) + [6, 4] + _u16(1) + GS_MAC + GS_IP + [0] * 6 + NODE_B_IP
    _inject(NODE_B_MAC + GS_MAC + ETHERTYPE_ARP + arp)


def mc_gs_udp(port, text):
    payload = [ord(c) for c in str(text)]
    # UDP checksum 0 means "not computed", which IPv4 allows.
    udp = _u16(GS_PORT) + _u16(int(port)) + _u16(8 + len(payload)) + _u16(0) + payload
    # IPv4: version 4, 20-byte header, no options, TTL 64, protocol UDP, checksum filled in below.
    header = [0x45, 0] + _u16(20 + len(udp)) + _u16(0) + _u16(0) + [64, 17] + _u16(0)
    header += GS_IP + NODE_B_IP
    header[10:12] = _u16(_checksum(header))
    _inject(NODE_B_MAC + GS_MAC + ETHERTYPE_IPV4 + header + udp)
