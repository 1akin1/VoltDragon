# CAN messages

Interface control document for the inter-node CAN bus (HLR-003, HLR-005, HLR-011, HLR-013).

Implementation: [`can.c`](../firmware/common/src/can.c) (bxCAN driver),
[`can_msg.c`](../firmware/common/src/can_msg.c) (layout and protection),
[`can_tx.c`](../firmware/node_a/src/can_tx.c) (Node A),
[`can_rx.c`](../firmware/node_b/src/can_rx.c) (Node B).

## Bus

| Parameter | Value |
|-----------|-------|
| Controller | STM32F407 CAN1, PB8 (RX) / PB9 (TX) |
| Bit rate | 500 kbit/s: prescaler 2 on the 16 MHz APB1 clock, 1 + 13 + 2 = 16 time quanta, sample point 87.5 % |
| Frames | Classic CAN, 11-bit identifiers, data frames only, always 8 data bytes |
| Error handling | Automatic retransmission; automatic bus-off recovery (ABOM) |
| Load | 400 frames/s from Node A and 10 from Node B, at about 130 bits each: about 11 % of the bus |

## Frame layout

Every message has the same 8-byte layout:

| Byte | Content |
|------|---------|
| 0..5 | Payload (message-specific) |
| 6 | Sequence counter, one per identifier, incremented per frame, wraps 255 → 0 |
| 7 | CRC-8/SAE-J1850 over the identifier (2 bytes, big-endian) followed by bytes 0..6 |

The CRC uses polynomial 0x1D, initial value 0xFF and final XOR 0xFF (check
value 0x4B for `"123456789"`). It protects the data end to end, on top of the
CAN controller's own CRC, which only covers a single hop and is checked in
hardware that could itself be faulty. Including the identifier in the CRC
means a frame that arrives under the wrong identifier is rejected. This is the
scheme of AUTOSAR E2E Profile 1.

## Messages from Node A

Node A uses a fixed schedule of ten 2 ms slots: STATUS, ACCEL, GYRO, MAG,
GPS_LAT, GPS_LON, NAV, SAFETY and two idle slots. Each message therefore repeats
every 20 ms (50 Hz), and the bus never carries a burst: a burst of frames would
overflow the receiver's three-deep hardware FIFO, which the first version of
this design did.

| ID | Name | Payload (little-endian) | Sent |
|----|------|-------------------------|------|
| 0x100 | STATUS | byte 0: flags (bit 0 IMU valid, bit 1 recorder OK); byte 1: reset count (saturates at 255); bytes 2..5: uptime in ms (uint32) | Always |
| 0x101 | ACCEL | X, Y, Z as int16, mg | While the IMU sample is valid |
| 0x102 | GYRO | X, Y, Z as int16, units of 10 mdps (0.01 dps) | While the IMU sample is valid |
| 0x103 | MAG | X, Y, Z as int16, mgauss | While the IMU sample is valid |
| 0x104 | GPS_LAT | bytes 0..3: latitude, int32, degrees x 1e7; byte 4: GGA fix quality; byte 5: satellites | While the GPS fix is fresh (under 1 s) |
| 0x105 | GPS_LON | bytes 0..3: longitude, int32, degrees x 1e7; bytes 4..5: altitude above mean sea level, int16, 0.1 m | While the GPS fix is fresh |
| 0x106 | NAV | bytes 0..1: true heading, uint16, 0.01 deg; bytes 2..3: measured field strength, uint16, mgauss; byte 4: flags (bit 0 magnetometer OK, bit 1 GPS fix, bits 2-3 heading source: 0 none, 1 magnetometer, 2 GPS course); byte 5: ground speed, 0.1 m/s (saturates at 25.5 m/s) | Always |
| 0x107 | SAFETY | bytes 0..1: distance to the nearest conductor, uint16, 0.1 m (0xFFFF: no fresh GPS fix); byte 2: battery, % (0xFF: autopilot silent); byte 3: flight mode (0 MISSION, 1 HOLD, 2 RETURN_TO_HOME, 3 LAND); byte 4: flags (see below); byte 5: id of the last operator mode request handled | Always |

SAFETY flags:

| Bit | Meaning |
|-----|---------|
| 0 (0x01) | Proximity warning: closer than 12 m to a conductor (until clear of 15 m) |
| 1 (0x02) | Avoiding: the autopilot is ordered to keep 15 m from the line |
| 2 (0x04) | Ground link lost (more than 3 s without ground station contact) |
| 3 (0x08) | Battery low (below 20 %) |
| 4 (0x10) | Battery critical (below 10 %) |
| 5 (0x20) | Autopilot OK: its status arrived in the last second |

Values outside their range saturate. Without a valid IMU sample the IMU
messages are not sent, and without a fresh GPS fix the position messages are
not sent; the receiver sees the flags cleared and the data age growing. With no
IMU and no GPS, only STATUS, NAV and SAFETY flow (150 frames/s).

## Messages from Node B

| ID | Name | Payload (little-endian) | Sent |
|----|------|-------------------------|------|
| 0x200 | B_STATUS | bytes 0..1: ground link age, uint16, ms since the last intact ground station command (saturates at 65535); byte 2: flags (bit 0: a ground station has been heard since start); bytes 3..5 zero | Every 100 ms |
| 0x201 | MODE_REQ | byte 0: request id (1..255, then wraps to 1); byte 1: requested flight mode; byte 2: 1 for an operator override (HLR-006); bytes 3..5 zero | Once per accepted `MODE` command |

Node A adds the time since the last B_STATUS to the reported link age, so a
silent Node B counts as a lost ground link too. Node A applies a MODE_REQ
through its own transition table and reports the outcome in SAFETY (mode and
last request id). A repeated request id within 500 ms is taken as the same
frame delivered twice and ignored. Node B checks requests before forwarding
them, against the mode it expects Node A to be in: the mode of a request it has
just forwarded, until SAFETY shows that request handled (or 500 ms pass), so a
command sent straight after another is not judged on an old report.

## Reception on Node A

A hardware filter admits 0x200..0x20F. Frames with the wrong length or CRC are
rejected and counted; Node A's console reports the frames received from Node B
and the rejected ones once per second. Node B's messages are not
sequence-tracked: B_STATUS is periodic and its age is what matters, and
MODE_REQ carries its own request id.

## Reception on Node B

- A hardware filter (bank 0, identifier/mask mode) admits 0x100..0x10F into
  FIFO 0. Frames with any other identifier never reach the software.
- A frame with the wrong length or CRC is **rejected**. Its sequence number
  cannot be trusted, so it is not tracked, and the gap it leaves is then
  counted as **lost** as well.
- For each identifier, a gap of 1..127 in the sequence counter counts as that
  many **lost** frames. A jump backwards or of 128 or more is treated as a
  restart or duplicate and resynchronises without counting losses.
- A Node A restart is recognised from STATUS (uptime going backwards or a new
  reset count). All trackers are then resynchronised, because the restarted
  counters would otherwise look like lost frames.
- The latest values and the arrival time of the last valid frame are kept.
  Node B reports them once per second on its console, and the `CAN` command
  returns the counters and the data age (see
  [command-interface.md](command-interface.md)).

## Fault injection

Node A's debug console injects faults into the next frame, to verify the
receiver's checks (see `tests/robot/system_can.robot`):

| Key | Fault | Expected on Node B |
|-----|-------|--------------------|
| `c` | Wrong CRC | rejected +1, lost +1 |
| `g` | Sequence number skipped | lost +1 |
| `u` | Extra frame with identifier 0x300 | Nothing: removed by the hardware filter |
