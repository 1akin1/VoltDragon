# Demo Video

A plan for a three-minute video of VoltDragon: what to show, how to produce each
piece, and narration to go with it. Most of the footage is generated: the ground
station clips come from real co-simulated flights, rendered from the telemetry
Node B sent.

## Producing the footage

**Ground station clips** (about 20 minutes, unattended), on Linux or WSL with the
firmware built and the dev extras installed:

```sh
bash tools/demo/make_clips.sh build/demo
```

This flies five scenarios in the co-simulation, records Node B's telemetry inside
Renode (no network setup or root needed) and renders each flight as a video of the
ground-station display, with a caption, the map following the vehicle, at mission
speed:

| Clip | Scenario | Mission time |
|------|----------|-------------:|
| `close-pass` | the inspection pass near the conductors, avoidance working | 45 s |
| `no-avoidance` | the same pass with an autopilot that ignores the avoidance order | 45 s |
| `link-loss` | the ground station's heartbeat stops at 12 s | 25 s |
| `battery` | battery at 22 %, draining at 0.5 %/s | 30 s |
| `prop-damage` | a propeller is damaged at 15 s | 25 s |

Clips are `.mp4` (H.264, 1500 × 1000, about 1 MB each) if `ffmpeg` is installed
(`sudo apt install ffmpeg`), otherwise `.gif` (up to 15 MB). To re-render from the
saved telemetry without flying again: `bash tools/demo/make_clips.sh --reuse build/demo`.
A single flight can be rendered by hand:

```sh
python -m sim.mission --scenario link-loss --duration 25 --record build/demo/link-loss.tlm
python -m ground_station.display --replay build/demo/link-loss.tlm --video link-loss.mp4 \
    --follow 250 --speed 2 --title "Ground link lost at 12 s"
```

**README GIFs**: `bash tools/demo/readme_gifs.sh build/demo` turns the clips into
the GIFs in `docs/images` (3x speed, 900 pixels wide, 1 to 2 MB each).

**Screen recordings** (the rest), for example with OBS Studio at 1920 × 1080:
the Renode consoles, the bus monitor's log, and pages of the repository on GitHub.

**Editing**: any editor will do (Clipchamp, Shotcut, DaVinci Resolve). Speed the
longer clips up 1.5-2x; the captions stay readable.

## Storyboard

| Time | Scene | Footage |
|------|-------|---------|
| 0:00 | Title | README heading and the architecture diagram on GitHub |
| 0:15 | Boot | `renode renode/system.resc`: both nodes' consoles starting |
| 0:40 | Close pass | `close-pass` clip |
| 1:05 | Magnetometer corruption | `no-avoidance` clip |
| 1:30 | Failsafes | `link-loss` and `battery` clips, 2x |
| 1:55 | Edge AI | `prop-damage` clip |
| 2:15 | ARINC 429 | Renode log window with the bus monitor's lines |
| 2:30 | Process | GitHub Actions run, traceability matrix, MISRA and MC/DC reports |
| 2:55 | End | repository link |

## Narration

**Title.** VoltDragon is the avionics for a drone that inspects high-voltage power
lines. There is no hardware: two STM32 microcontrollers run real firmware in the
Renode emulator, and a Python model flies the vehicle around them.

**Boot.** Node A reads the IMU, the GPS and the autopilot, and runs the safety
logic and a neural network under FreeRTOS. Node B is the gateway: it takes Node A's
data over CAN and sends telemetry to the ground station over Ethernet, and ARINC
429 words to the avionics bus. Every interface is driven at register level, from
the startup code up.

**Close pass.** The flight plan takes the vehicle within 4 metres of a conductor.
Below 12 metres Node A raises a proximity warning and orders the autopilot to keep
15 metres, in the same 20-millisecond cycle. The vehicle never gets closer than
10 metres.

**Magnetometer corruption.** Here the autopilot ignores that order, as a fault
injection. Near the line, the current's magnetic field corrupts the compass: Node A
detects it from the field strength and dip angle (the red part of the track) and
switches to the GPS course for its heading until the field is clean again.

**Failsafes.** When the ground station falls silent, Node A returns home three
seconds later. With the battery draining, it returns home below 20 percent and
lands below 10. Every mode change goes through one transition table, and nothing
leaves LAND.

**Edge AI.** At 15 seconds a propeller is damaged. A quantised neural network on
Node A, running on TensorFlow Lite Micro in two kilobytes, classifies the vibration
and raises the alarm, which reaches the ground station in about a second. (In the
battery clip, one window scores 100 percent as the landing starts; the alarm needs
three faulty windows out of four, so it stays quiet. That is worth pointing at.)

**ARINC 429.** Node B also sends altitude, heading and distance to the line as
ARINC 429 words. The transmitter's model decodes every word on its own, so the
tests check the firmware's encoding against an independent decoder.

**Process.** The project follows a DO-178C-style process: 20 high-level
requirements, refined into 97 low-level ones, traced to the code and to 217 tests,
and that matrix is checked on every commit. The C code has no open MISRA finding,
and the safety logic has full MC/DC coverage. All of it runs in CI: unit tests,
76 system tests in Renode and 21 closed-loop flights.

**End.** The code, the requirements and the reports are on GitHub.
