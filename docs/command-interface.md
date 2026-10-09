# Operator command interface

Interface control document for the operator commands accepted by Node B
(HLR-014). The same frames arrive on two transports:

- **USART3** (PB10 TX / PB11 RX, 115200 baud, 8N1), one frame per line;
- **UDP port 5601** on 192.168.10.2, one frame per datagram, with the reply
  sent back to the sender (see [telemetry.md](telemetry.md)).

Each transport has its own retransmission cache. The debug console on USART2
is separate and unaffected.

Implementation: [`cmd_protocol.c`](../firmware/common/src/cmd_protocol.c),
[`cmd_dispatch.c`](../firmware/common/src/cmd_dispatch.c),
[`commands.c`](../firmware/node_b/src/commands.c).

## Framing

```
request:   $<seq>,<VERB>[,<arg>...]*<CK><EOL>
response:  $<seq>,ACK,<VERB>[,<field>...]*<CK>\r\n
           $<seq>,NAK,<VERB>,<REASON>*<CK>\r\n
```

| Element | Rule |
|---------|------|
| `$` | Starts a frame. Any `$` restarts frame assembly, so the receiver resynchronises after line noise. |
| `<seq>` | Decimal, 1 to 65535, chosen by the sender. `0` is reserved for replies to frames whose content cannot be trusted. |
| `<VERB>` | 1 to 12 printable characters, case-sensitive. |
| `<arg>` | Up to 4 arguments of 1 to 16 printable characters. Empty arguments are invalid. |
| `<CK>` | XOR of every byte between `$` and `*`, as two hex digits (either case accepted, upper case sent). The same checksum as NMEA 0183. |
| `<EOL>` | CR, LF or CR LF. Empty lines are ignored. |
| Length | At most 80 characters from `$` to the last checksum digit. |
| Characters | Printable ASCII (0x20 to 0x7E) only; `$` and `*` only as delimiters. |

Example: `$1,PING*0D` → `$1,ACK,PING*0D`

## Rejection reasons

Every complete line gets exactly one response.

| Reason | Meaning | `<seq>` and `<VERB>` in the NAK |
|--------|---------|------------------------------|
| `FRAME` | Not a valid frame: missing `$` or checksum, illegal character, missing verb | `0`, `-` |
| `CHECKSUM` | Checksum does not match the content | `0`, `-` |
| `SEQ` | Sequence number missing, not a number or outside 1..65535 | `0`, `-` |
| `LENGTH` | Line longer than 80 characters | `0`, `-` |
| `UNKNOWN` | Unknown verb | echoed (`-` if the verb is over 12 characters) |
| `ARGS` | Wrong number of arguments, or a value out of range | echoed |
| `REFUSED` | Valid request that is not allowed now (see `MODE`) | echoed |

When the frame is corrupt, its sequence number cannot be trusted, so the NAK
uses `0`. The sender should treat a `0` NAK as "my last frame was damaged" and
retransmit.

## Retransmission

If a request is identical to the previous request (same sequence number and
same content), the previous response is sent again and the command is **not
executed a second time**. A sender that loses a reply can therefore retransmit safely, which
matters for commands that change state. Rejected corrupt frames (`FRAME`,
`CHECKSUM`, `SEQ`, `LENGTH`) do not affect this. Senders should increment the
sequence number for each new command. A different request that reuses the
previous sequence number is executed: the ground station's heartbeat and an
operator command, sent independently, may collide on a number.

## Commands

| Command | Arguments | ACK fields | Notes |
|---------|-----------|------------|-------|
| `PING` | none | none | Liveness check |
| `VERSION` | none | `<major.minor.patch>` | Firmware version |
| `STATUS` | none | `<uptime_ms>,<reset_count>,<accepted>,<rejected>` | Counters exclude the STATUS request itself and replays |
| `CAN` | none | `<valid>,<rejected>,<lost>,<unknown_id>,<age_ms>` | CAN link counters (see [can-messages.md](can-messages.md)); `age_ms` is the time since the last valid frame from Node A, or `-` if none has arrived |
| `TLM_RATE` | `[<hz>]` | `<hz>` | Without an argument, returns the current rate. With one, sets it; valid range 10 to 50 Hz (HLR-012 requires at least 10 Hz). Default 10 Hz. |
| `MODE` | `<mode> [OVERRIDE]` | `<mode>` | Requests a flight mode: `MISSION`, `HOLD`, `RTH` (or `RETURN_TO_HOME`) or `LAND` (HLR-005, HLR-006). The ACK names the mode and means the request went to Node A. |

### MODE

Node B checks a request against the flight-mode transition table before
forwarding it to Node A over CAN (MODE_REQ, see [can-messages.md](can-messages.md)):

| From → to | MISSION | HOLD | RTH | LAND |
|-----------|---------|------|-----|------|
| MISSION | - | yes | yes | yes |
| HOLD | yes | - | yes | yes |
| RTH | OVERRIDE | OVERRIDE | - | yes |
| LAND | no | no | no | - |

- Moving to a safety mode (RTH, LAND) is always allowed.
- Leaving RTH needs the `OVERRIDE` argument (HLR-006): the vehicle may have
  been sent home by its own failsafe, and resuming is an explicit decision.
- LAND is final.
- A request for the current mode is acknowledged and not forwarded.
- An unknown mode, or a second argument other than `OVERRIDE`, gets `ARGS`;
  a transition that is not allowed, or a request before Node A has reported
  its mode, gets `REFUSED` and is logged on Node B's console.

Node A applies the same table again (it decides; Node B's check gives the
operator an immediate answer) and reports the result, with the request id, in
its SAFETY message and in telemetry. Examples:

```
$1,MODE,HOLD*3D                     ->  $1,ACK,MODE,HOLD*58
$2,MODE,RTH*7F                      ->  $2,ACK,MODE,RETURN_TO_HOME*4A
$3,MODE,MISSION*7C                  ->  $3,NAK,MODE,REFUSED*0E
$4,MODE,MISSION,OVERRIDE*43         ->  $4,ACK,MODE,MISSION*1E
```

### Ground contact

Every intact command (anything but a `FRAME`, `CHECKSUM`, `SEQ` or `LENGTH`
rejection), on the UART or over UDP, counts as contact with the ground station
for the link-loss check (HLR-003). The ground station display sends `PING`
once a second as a heartbeat.

## Timing and limits

- Reception is interrupt-driven into a 256-byte buffer, so a burst of up to
  256 bytes (three full-length commands) is accepted while the main loop is busy.
  Lost bytes are counted and reported on the debug console.
- Responses are sent from the main loop with a polled transmitter; a
  full-length response takes about 8 ms at 115200 baud.
