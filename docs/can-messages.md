# CAN messages

Interface control document for the inter-node CAN bus (HLR-011, HLR-013).

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
| Load | 200 frames/s at about 130 bits each: about 5 % of the bus |

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

Node A sends one frame every 5 ms in a fixed rotation, so each message repeats
every 20 ms (50 Hz) and the bus never carries a burst. A burst of four frames
would overflow the receiver's three-deep hardware FIFO, which the first
version of this design did.

| ID | Name | Payload (little-endian) | Sent |
|----|------|-------------------------|------|
| 0x100 | STATUS | byte 0: flags (bit 0 IMU valid, bit 1 recorder OK); byte 1: reset count (saturates at 255); bytes 2..5: uptime in ms (uint32) | Always |
| 0x101 | ACCEL | X, Y, Z as int16, mg | While the IMU sample is valid |
| 0x102 | GYRO | X, Y, Z as int16, units of 10 mdps (0.01 dps) | While the IMU sample is valid |
| 0x103 | MAG | X, Y, Z as int16, mgauss | While the IMU sample is valid |

Values outside the int16 range saturate. Without a valid IMU sample, only
STATUS is sent; the receiver sees the IMU flag cleared and the IMU data age
growing.

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
