# Autopilot link

Interface control document for the serial link between Node A and the
autopilot (flight controller), and the safety logic that drives it
(HLR-001 to HLR-006).

Implementation: [`ap_msg.c`](../firmware/common/src/ap_msg.c) (sentences),
[`ap_link.c`](../firmware/node_a/src/ap_link.c) (UART),
[`safety.c`](../firmware/node_a/src/safety.c) (decisions),
[`flight_mode.c`](../firmware/common/src/flight_mode.c) (transition table),
[`sim/plant/autopilot.py`](../sim/plant/autopilot.py) (the simulated autopilot).

The flight controller itself is not part of this project: Node A is the
mission and safety computer, which tells an autopilot what to do and checks
that it does. In the simulation the plant model plays the autopilot.

## Physical layer

| Parameter | Value |
|-----------|-------|
| UART | Node A UART5: PC12 TX, PD2 RX (AF8) |
| Format | 115200 baud, 8N1, ASCII lines ending in CR LF |
| Reception | Interrupt-driven into a ring buffer, parsed by ControlTask every 20 ms |

## Sentences

Both use NMEA-style framing: `$<body>*<checksum>`, the checksum being the XOR of
the characters between `$` and `*` as two upper-case hex digits.

### Node A → autopilot: `$VDCMD`

```
$VDCMD,<mode>,<avoid>,<min_distance_dm>*hh
```

| Field | Values |
|-------|--------|
| mode | `MISSION` (fly the plan), `HOLD` (hold position), `RTH` (return to the start of the route), `LAND` (land where you are) |
| avoid | `1`: keep at least `min_distance_dm` from every conductor; `0`: no avoidance order |
| min_distance_dm | 0.1 m; 150 while avoiding, 0 otherwise |

Sent every 100 ms, and at once when the mode or the avoidance order changes,
so a change reaches the autopilot in the same 20 ms control cycle that decided
it. Example: `$VDCMD,MISSION,1,150*3D`.

### Autopilot → Node A: `$VDAPS`

```
$VDAPS,<battery_pct>,<mode>*hh
```

| Field | Values |
|-------|--------|
| battery_pct | State of charge, 0 to 100 % |
| mode | The mode the autopilot is flying (same names as above) |

Sent at 5 Hz. Node A treats the autopilot as silent after 1 s without a valid
sentence: the SAFETY message then reports the battery as unknown and clears
the autopilot-OK flag, and the ground station raises AUTOPILOT SILENT.
Sentences with a bad checksum or invalid fields are counted and dropped.

## Safety logic (Node A, ControlTask, every 20 ms)

| Input | Rule | Action |
|-------|------|--------|
| GPS fix and route geometry | Distance to the nearest conductor below 12 m | PROXIMITY_WARNING; order the autopilot to keep 15 m (`avoid=1`) until the distance is back above 15 m (HLR-001, HLR-002) |
| Node B's B_STATUS | More than 3 s since the last ground station contact, once one has been heard | RETURN_TO_HOME (HLR-003) |
| `$VDAPS` battery | Below 20 % | RETURN_TO_HOME (HLR-004) |
| `$VDAPS` battery | Below 10 % | LAND (HLR-004) |
| Node B's MODE_REQ | Operator request, through the transition table | Change mode or reject and log (HLR-005, HLR-006) |

- The distance comes from Node A's own copy of the route geometry
  ([`route.c`](../firmware/node_a/src/route.c)); it is unit-tested against the
  plant's Python implementation.
- The warning is logged with the age of the GPS fix that triggered it, which
  shows it was raised within one control cycle of the new position (HLR-002
  requires 100 ms).
- Automatic triggers fire once, when their condition starts, and only ever
  towards a safer mode; nothing leaves LAND.
- Link-loss detection starts only once a ground station has been heard, so a
  vehicle on the bench without one does not return home. The link age is
  Node B's report plus the time since that report, so a silent Node B also
  counts as a lost link.
- The flight-mode transition table is in
  [command-interface.md](command-interface.md#mode).

## The simulated autopilot

[`sim/plant/autopilot.py`](../sim/plant/autopilot.py) parses `$VDCMD` and
flies the ordered mode with the plant's vehicle model:

| Mode | Vehicle |
|------|---------|
| MISSION | Follows the flight plan along the route |
| HOLD | Holds its position |
| RTH | Flies back along the route to the start, at the plan's normal 20 m offset |
| LAND | Descends at 1 m/s where it is, and stays down |

An avoidance order sets a keep-out distance of the ordered minimum plus 1 m.
The autopilot keeps it until the order is withdrawn *and* its own plan no
longer comes closer than that, so withdrawing the order (which Node A does at
15 m) does not send the vehicle straight back towards the line. The battery
drains at a set rate while airborne (0.05 %/s by default; the battery
scenario starts at 22 % and drains at 0.5 %/s). A fault-injection option makes
it ignore avoidance orders.

In the co-simulation the plant's `$VDAPS` sentences are written into Node A's
UART5, and Node A's output is captured by Renode into a file that the
co-simulation reads after each 100 ms step (see [simulation.md](simulation.md)).

## Tests

- Unit: `tests/unit/test_ap_msg.c` (sentences), `test_flight_mode.c` (the
  transition table, all 128 cases), `test_route.c` (distance).
- Open loop, exact inputs: `tests/robot/system_safety.robot`.
- Closed loop with the plant: `tests/integration/test_mission.py` (close pass)
  and `test_safety.py` (link loss, battery).
- The simulated autopilot: `tests/python/test_autopilot.py`.
