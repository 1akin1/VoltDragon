# Telemetry and ground link

Interface control document for the UDP link between Node B and the ground
station (HLR-003, HLR-012, HLR-013, HLR-014).

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

One 88-byte packet per period, at the rate set with `TLM_RATE` (10 Hz by
default, 10 to 50 Hz). All fields are little-endian. This is version 3, which
adds Node A's safety state; version 2 (Phase 3) was 80 bytes without it, and
version 1 (Phase 2) 60 bytes without the navigation fields.

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 4 | Magic | ASCII `VDTM` |
| 4 | 1 | Version | 3 |
| 5 | 1 | Flags | bit 0: Node A data fresh (age < 100 ms); bit 1: Node A IMU valid; bit 2: Node A recorder OK; bit 3: GPS fix (position younger than 1 s); bit 4: magnetometer OK; bits 5-6: heading source (0 none, 1 magnetometer, 2 GPS course) |
| 6 | 2 | Length | 88 |
| 8 | 4 | Sequence number | Increments per packet sent; restarts at 0 when Node B restarts |
| 12 | 4 | Node B uptime | ms |
| 16 | 4 | Node A uptime | ms, from Node A's STATUS message |
| 20 | 1 | Node A reset count | Saturates at 255 |
| 21 | 1 | Node B reset count | Saturates at 255 |
| 22 | 2 | Node A data age | ms since the last valid CAN frame; 65535 if none yet |
| 24 | 6 | Acceleration X, Y, Z | int16, mg |
| 30 | 6 | Angular rate X, Y, Z | int16, units of 10 mdps |
| 36 | 6 | Magnetic field X, Y, Z | int16, mgauss |
| 42 | 1 | GPS satellites | |
| 43 | 1 | GPS fix quality | NMEA GGA quality: 0 none, 1 GPS, 2 DGPS |
| 44 | 4 | CAN frames valid | Running totals on Node B (see [can-messages.md](can-messages.md)) |
| 48 | 4 | CAN frames rejected | |
| 52 | 4 | CAN frames lost | |
| 56 | 4 | Latitude | int32, degrees x 1e7 |
| 60 | 4 | Longitude | int32, degrees x 1e7 |
| 64 | 2 | Altitude | int16, above mean sea level, 0.1 m |
| 66 | 2 | Heading | uint16, true heading, 0.01 deg |
| 68 | 2 | Magnetic field strength | uint16, mgauss, as measured by Node A |
| 70 | 2 | Ground speed | uint16, 0.1 m/s |
| 72 | 2 | GPS data age | ms since the last position on CAN; 65535 if none yet |
| 74 | 2 | Distance to the line | uint16, 0.1 m, Node A's own estimate (SAFETY message); 65535 if unknown |
| 76 | 1 | Battery | %, from the autopilot via Node A; 255 if unknown |
| 77 | 1 | Flight mode | 0 MISSION, 1 HOLD, 2 RETURN_TO_HOME, 3 LAND; 255 before Node A has reported it |
| 78 | 1 | Safety flags | As in the SAFETY message: proximity, avoiding, ground link lost, battery low, battery critical, autopilot OK |
| 79 | 1 | Last mode request | Id of the last operator mode request Node A handled |
| 80 | 2 | Ground link age | ms since Node B last received an intact command; 65535 if none yet |
| 82 | 2 | Reserved | 0 |
| 84 | 4 | CRC-32 | IEEE 802.3 (zlib) over bytes 0..83 |

UDP has its own checksum, but it is optional in IPv4 and only covers the
transport hop. The CRC-32 protects the packet end to end, from the encoder on
Node B to the decoder on the ground. A receiver detects lost packets from gaps
in the sequence number, and a Node B restart from the number going backwards.

### Reference packet

Both decoders (C and Python) and the C encoder are tested against this packet,
which was built from the table above with Python's `struct` module:

```
5644544d033f58000700000040e20100c0d40100010003000cfefa00e7031a04
fcd600001f018dff35fe0901e803000001000000020000009246c8172ef88c13
22247c1708023c009600b7004c002003a4010000305d4ac4
```

It decodes to sequence 7, flags 0x3F (all status flags set, heading from the
magnetometer), Node B uptime 123456 ms, Node A uptime 120000 ms, Node A reset
count 1, data age 3 ms, acceleration (-500, 250, 999) mg, angular rate
(10500, -105000, 0) mdps, magnetic field (287, -115, -459) mgauss, 9 satellites
with fix quality 1, CAN counters 1000 valid, 1 rejected, 2 lost, position
39.9001234 N 32.8005678 E at 925.0 m, heading 60.12 deg, field 520 mG, ground
speed 6.0 m/s, GPS data age 150 ms, distance to the line 18.3 m, battery 76 %,
mode MISSION, safety flags 0x20 (autopilot OK), last mode request 3 and ground
link age 420 ms.

## Commands over UDP

The frames are the same as on the UART (see
[command-interface.md](command-interface.md)), one per datagram, with or
without a trailing CR/LF. Replies carry no line ending. UDP has its own
retransmission cache, separate from the UART's, because the two are
independent sessions.

### Heartbeat

The ground station display sends `PING` once a second (`--target` sets Node B's
address, `--no-heartbeat` turns it off). Node B counts every intact command,
on either interface, as contact with the ground station and reports the link
age to Node A, which returns home after 3 s without contact (HLR-003). A
ground station that only listens is therefore not enough to keep the mission
going once one has been heard.

## Running the link against a real ground station

`renode/ground_link.resc` bridges Node B's Ethernet to a TAP interface on a
Linux host (in WSL on Windows). Renode needs permission to create the TAP
device, so run it as root:

```sh
sudo renode renode/ground_link.resc        # starts both nodes
sudo ip addr add 192.168.10.1/24 dev tap0
sudo ip link set tap0 up

python -m ground_station.display           # map, plots and alarms (needs matplotlib)
python -m ground_station.receiver          # or: one text line per packet, with loss counts
python -m ground_station.command TLM_RATE 20
python -m ground_station.command CAN
python -m ground_station.command --seq 7 MODE HOLD
python -m ground_station.command --seq 8 MODE MISSION OVERRIDE
```

Wireshark or `tcpdump -i tap0` shows the telemetry broadcasts, ARP and the
command exchanges. For a flight with the plant model driving the sensors, use
`sudo python -m sim.mission --tap` instead of `ground_link.resc` (see
[simulation.md](simulation.md)).

## Ground-station display

`python -m ground_station.display` (HLR-019) shows:

- a map of the route (pylons and conductors) with the vehicle's track, red
  where the magnetometer was disturbed, and its position and heading;
- the distance to the nearest conductor, computed on the ground from the GPS
  position and the route geometry, against the 12 m warning and 10 m minimum
  (HLR-001, HLR-002);
- the measured field strength against the magnetometer's acceptance band, and
  the heading coloured by its source;
- the flight mode, battery and Node A's own distance estimate;
- the active alarms: link lost, too close to the line, proximity, Node A data
  stale, GPS no fix, IMU invalid, magnetometer disturbed, recorder off, packet
  loss and CAN errors, and from Node A's safety state: return to home or land
  (critical) and hold, vehicle lost the command link, battery low or critical,
  autopilot silent, avoiding, and safety state unknown.

`--record FILE` keeps the received packets, `--replay FILE` plays them back, and
`--snapshot IMAGE` saves an image instead of opening a window.

## Automated tests

`tests/robot/system_telemetry.robot` checks the frames leaving Node B byte by
byte with Renode's network interface tester, and injects ARP and UDP command
frames from a simulated ground station (`renode/ground_station.py`). The
interface tester only sees virtual time pass while the emulation runs, so that
suite starts the emulation once and does not pause it between checks.
