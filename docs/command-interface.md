# Operator command interface

Interface control document for the operator commands accepted by Node B
(HLR-014). The same frames will be carried over UDP once networking is added;
today they arrive on Node B's **USART3** (PB10 TX / PB11 RX, 115200 baud, 8N1).
The debug console on USART2 is separate and unaffected.

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

When the frame is corrupt, its sequence number cannot be trusted, so the NAK
uses `0`. The sender should treat a `0` NAK as "my last frame was damaged" and
retransmit.

## Retransmission

If a request has the same sequence number as the previous request, the
previous response is sent again and the command is **not executed a second
time**. A sender that loses a reply can therefore retransmit safely, which
matters for commands that change state. Rejected corrupt frames (`FRAME`,
`CHECKSUM`, `SEQ`, `LENGTH`) do not affect this. Senders should increment the
sequence number for each new command.

## Commands

| Command | Arguments | ACK fields | Notes |
|---------|-----------|------------|-------|
| `PING` | none | none | Liveness check |
| `VERSION` | none | `<major.minor.patch>` | Firmware version |
| `STATUS` | none | `<uptime_ms>,<reset_count>,<accepted>,<rejected>` | Counters exclude the STATUS request itself and replays |
| `TLM_RATE` | `[<hz>]` | `<hz>` | Without an argument, returns the current rate. With one, sets it; valid range 10 to 50 Hz (HLR-012 requires at least 10 Hz). Default 10 Hz. |

## Timing and limits

- Reception is interrupt-driven into a 128-byte buffer, so a burst of up to
  128 bytes (about 1.5 full-length commands) is accepted while the main loop is busy.
  Lost bytes are counted and reported on the debug console.
- Responses are sent from the main loop with a polled transmitter; a
  full-length response takes about 8 ms at 115200 baud.
