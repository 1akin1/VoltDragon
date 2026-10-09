# Telemetry and ground link

Interface control document for the UDP link between Node B and the ground
station (HLR-012, HLR-013, HLR-014).

Implementation: [`tlm_msg.c`](../firmware/common/src/tlm_msg.c) (packet),
[`telemetry.c`](../firmware/node_b/src/telemetry.c) (sender),
[`net/`](../firmware/node_b/src/net) (Ethernet driver, lwIP port, sockets),
[`ground_station/`](../ground_station) (Python receiver and command tool).

## Network

| Parameter | Value |
|-----------|-------|
| Node B | 192.168.10.2/24, MAC 02:00:00:56:44:02 (locally administered) |
| Ground station | Any address on 192.168.10.0/24, 192.168.10.1 by convention |
| Telemetry | UDP to the subnet broadcast address 192.168.10.255, port 5600 |
| Commands | UDP to 192.168.10.2 port 5601, one command per datagram; the reply goes back to the sender's address and port |
| Stack | lwIP 2.2.1, bare metal (NO_SYS), IPv4 with ARP, ICMP and UDP; no TCP or DHCP |
| PHY | 802.3 clause 22 over MDIO, RMII; auto-negotiated speed and duplex |

Broadcasting telemetry means a ground station needs no configuration on the
node and no ARP exchange before data flows. Any number of listeners can
receive it.

## Telemetry packet

One 60-byte packet per period, at the rate set with `TLM_RATE` (10 Hz by
default, 10 to 50 Hz). All fields are little-endian.

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 4 | Magic | ASCII `VDTM` |
| 4 | 1 | Version | 1 |
| 5 | 1 | Flags | bit 0: Node A data fresh (age < 100 ms); bit 1: Node A IMU valid; bit 2: Node A recorder OK |
| 6 | 2 | Length | 60 |
| 8 | 4 | Sequence number | Increments per packet sent; restarts at 0 when Node B restarts |
| 12 | 4 | Node B uptime | ms |
| 16 | 4 | Node A uptime | ms, from Node A's STATUS message |
| 20 | 1 | Node A reset count | Saturates at 255 |
| 21 | 1 | Node B reset count | Saturates at 255 |
| 22 | 2 | Node A data age | ms since the last valid CAN frame; 65535 if none yet |
| 24 | 6 | Acceleration X, Y, Z | int16, mg |
| 30 | 6 | Angular rate X, Y, Z | int16, units of 10 mdps |
| 36 | 6 | Magnetic field X, Y, Z | int16, mgauss |
| 42 | 2 | Reserved | 0 |
| 44 | 4 | CAN frames valid | Running totals on Node B (see [can-messages.md](can-messages.md)) |
| 48 | 4 | CAN frames rejected | |
| 52 | 4 | CAN frames lost | |
| 56 | 4 | CRC-32 | IEEE 802.3 (zlib) over bytes 0..55 |

UDP has its own checksum, but it is optional in IPv4 and only covers the
transport hop. The CRC-32 protects the packet end to end, from the encoder on
Node B to the decoder on the ground. A receiver detects lost packets from gaps
in the sequence number, and a Node B restart from the number going backwards.

### Reference packet

Both decoders (C and Python) and the C encoder are tested against this packet,
which was built from the table above with Python's `struct` module:

```
5644544d01073c000700000040e20100c0d40100010003000cfefa00e7031a04
fcd600001f018dff35fe0000e80300000100000002000000bc7cc438
```

It decodes to sequence 7, all flags set, Node B uptime 123456 ms, Node A
uptime 120000 ms, Node A reset count 1, data age 3 ms, acceleration
(-500, 250, 999) mg, angular rate (10500, -105000, 0) mdps, magnetic field
(287, -115, -459) mgauss and CAN counters 1000 valid, 1 rejected, 2 lost.

## Commands over UDP

The frames are the same as on the UART (see
[command-interface.md](command-interface.md)), one per datagram, with or
without a trailing CR/LF. Replies carry no line ending. UDP has its own
retransmission cache, separate from the UART's, because the two are
independent sessions.

## Running the link against a real ground station

`renode/ground_link.resc` bridges Node B's Ethernet to a TAP interface on a
Linux host (in WSL on Windows). Renode needs permission to create the TAP
device, so run it as root:

```sh
sudo renode renode/ground_link.resc        # starts both nodes
sudo ip addr add 192.168.10.1/24 dev tap0
sudo ip link set tap0 up

python -m ground_station.receiver          # prints each packet and counts losses
python -m ground_station.command TLM_RATE 20
python -m ground_station.command CAN
```

Wireshark or `tcpdump -i tap0` shows the telemetry broadcasts, ARP and the
command exchanges.

## Automated tests

`tests/robot/system_telemetry.robot` checks the frames leaving Node B byte by
byte with Renode's network interface tester, and injects ARP and UDP command
frames from a simulated ground station (`renode/ground_station.py`). The
interface tester only sees virtual time pass while the emulation runs, so that
suite starts the emulation once and does not pause it between checks.
