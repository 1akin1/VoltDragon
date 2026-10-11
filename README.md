# VoltDragon

[![CI](https://github.com/1akin1/VoltDragon/actions/workflows/ci.yml/badge.svg)](https://github.com/1akin1/VoltDragon/actions/workflows/ci.yml)
[![MISRA C:2012](https://img.shields.io/badge/MISRA%20C%3A2012-0%20open%20findings-brightgreen)](docs/misra-report.md)
[![MC/DC](https://img.shields.io/badge/MC%2FDC%20safety%20logic-100%25-brightgreen)](docs/mcdc-report.md)
[![Traceability](https://img.shields.io/badge/requirements-20%20HLR%20%E2%86%92%2097%20LLR%20%E2%86%92%20217%20tests-blue)](docs/traceability.md)

Avionics software for a **power-line inspection drone**, written from the reset vector
up and verified **entirely in simulation**: no hardware needed to build, run or test it.

Two **STM32F407 (Cortex-M4F)** nodes run their real firmware in the
[Renode](https://renode.io) emulator. A Python **plant model** flies the vehicle
along a high-voltage line, with wind, sensor noise, the line's magnetic field and
rotor faults, and closes the loop with the firmware. A Python **ground station**
shows the flight and sends commands. The work follows a **DO-178C-style process**:
requirements traced to code and tests, MISRA C, and MC/DC coverage of the safety
logic, all enforced in CI.

![A co-simulated flight: the autopilot ignores the avoidance order, the vehicle comes within 4.5 m of a conductor, Node A detects the corrupted magnetometer and switches to the GPS course](docs/images/no-avoidance.gif)

*A co-simulated flight, replayed from the telemetry Node B sent (3x speed). The
autopilot has been made to ignore Node A's avoidance order: the vehicle comes
within 4.5 m of a conductor, the line's field corrupts the magnetometer (red track),
and Node A switches its heading to the GPS course (purple) until the field is clean.*

## At a glance

- **Two firmware images, about 10 400 lines of first-party C**: startup code, linker
  script, register-level drivers for I2C, SPI, UART, CAN and Ethernet, FreeRTOS on one
  node and a bare-metal super-loop on the other.
- **Safety logic** that keeps the vehicle 10 m from the line, returns home on link
  loss or low battery and lands on critical battery, with **100 % MC/DC** coverage.
- **Edge AI on the MCU**: an INT8 neural network on TensorFlow Lite Micro, in a
  2 KB arena, detects propeller and bearing faults.
- **Avionics buses**: CAN with end-to-end CRCs, UDP telemetry through lwIP on a
  first-party Ethernet driver, and ARINC 429.
- **Verified at every level in CI**: 12 host unit-test programs, MC/DC measurement,
  MISRA analysis, 76 system tests on the emulated hardware and 21 closed-loop
  co-simulated flights, plus a generated HLR → LLR → code → test traceability matrix.

## See it fly

Each clip is a real co-simulated flight, rendered from Node B's telemetry by
`tools/demo/make_clips.sh` ([demo plan](docs/demo.md)). Click a clip to see it full size.

| Close pass: avoidance (HLR-001, HLR-002) | Damaged propeller: Edge AI (HLR-009) |
|:---:|:---:|
| [![Close pass](docs/images/close-pass.gif)](docs/images/close-pass.gif) | [![Damaged propeller](docs/images/prop-damage.gif)](docs/images/prop-damage.gif) |
| Planned at 4 m from a conductor; below 12 m Node A orders the autopilot to keep 15 m, and the vehicle never gets within 10 m. | A propeller is damaged at 15 s; the on-board classifier's fault score jumps and the alarm reaches the ground station about 1 s later. |
| **Ground link lost (HLR-003)** | **Battery failsafes (HLR-004)** |
| [![Link loss](docs/images/link-loss.gif)](docs/images/link-loss.gif) | [![Battery](docs/images/battery.gif)](docs/images/battery.gif) |
| The ground station's heartbeat stops at 12 s; 3 s later Node A returns the vehicle home. | Below 20 % the vehicle returns home, below 10 % it lands. One vibration window spikes as the landing starts; the 3-of-4 vote keeps the alarm off. |

## What it does

| Area | Implemented | Evidence |
|------|-------------|----------|
| Bare metal | Own startup code and linker script, register-level drivers for every peripheral, watchdog, HardFault capture, a reset record that survives resets | [debugging](docs/debugging.md), `node_a_boot.robot` |
| Sensors | LSM9DS1 IMU over I2C at 100 Hz, NMEA GPS, a flight data recorder on SPI flash that never blocks the control loop | `node_a_imu.robot`, `node_a_flashlog.robot` |
| Networks | CAN between the nodes (sequence counter and CRC on every frame), UDP telemetry and commands with lwIP on Node B's own Ethernet driver, ARINC 429 output | [CAN](docs/can-messages.md), [telemetry](docs/telemetry.md), [ARINC 429](docs/arinc429.md) |
| Real time | FreeRTOS, rate-monotonic tasks, static allocation, a watchdog fed only while every task runs; a priority inversion reproduced and fixed | [RTOS](docs/rtos.md) |
| Safety | Keeps 10 m from the line, returns home on link loss or low battery, lands on critical battery; operator modes through one transition table | [safety logic](docs/autopilot-link.md), [MC/DC report](docs/mcdc-report.md) |
| Fault detection | Detects magnetometer corruption near the line and falls back to the GPS course; an INT8 neural network detects propeller and bearing faults on the MCU | [simulation](docs/simulation.md), [Edge AI](docs/edge-ai.md) |
| Process | 20 high-level and 97 low-level requirements traced to code and tests; MISRA C:2012 with documented deviations; 100 % MC/DC of the safety logic | [traceability](docs/traceability.md), [MISRA](docs/misra-report.md), [MC/DC](docs/mcdc-report.md) |

## Results

Measured in the co-simulation and checked by the tests on every commit:

| Requirement | Result |
|-------------|--------|
| Never closer than 10 m to the line (HLR-001) | The close pass, planned at 4 m from a conductor, stays more than 10 m away |
| Proximity warning within 100 ms (HLR-002) | In the same 20 ms control cycle as the GPS fix |
| Return to home after 3 s without the ground link (HLR-003) | Link declared lost 3.0-3.3 s after the last heartbeat, return to home within 50 ms of that |
| Vibration fault reported within 2 s (HLR-009) | About 1 s from fault onset to the ground station |
| Priority inversion (Phase 4 demonstration) | Worst-case wait 85 ms with a semaphore, 5 ms with priority inheritance |
| Vibration classifier | INT8 model matches FP32 (99.98 % of test windows), no false alarm in 3 h of fault-free flight |

| Verification | Count |
|--------------|------:|
| Host unit test programs (ctest) | 12 |
| Python tests (pytest) | 150 |
| Renode system tests (Robot Framework) | 76 |
| Closed-loop co-simulated flights | 21 tests over 6 scenarios |
| Test cases traced to a requirement | 217 |
| Open MISRA C:2012 findings | 0 |
| MC/DC of the safety logic (`safety.c`) | 24 / 24 conditions |

## Where to look

For a quick review of the code, these files show the most:

| File | Why |
|------|-----|
| [`startup_stm32f4.c`](firmware/common/src/startup_stm32f4.c), [`stm32f407vg.ld`](firmware/common/linker/stm32f407vg.ld) | Reset handler, vector table and memory layout, including a `.noinit` reset record |
| [`fault.c`](firmware/common/src/fault.c) | HardFault entry in assembly, stacked-frame capture and CFSR decoding, kept across the reset |
| [`safety.c`](firmware/node_a/src/safety.c), [`test_safety.c`](tests/unit/test_safety.c) | The safety logic and the test set that gives it 100 % MC/DC |
| [`tasks.c`](firmware/node_a/src/tasks.c) | FreeRTOS tasks, watchdog supervision, the priority-inversion demonstration |
| [`flashlog.c`](firmware/node_a/src/flashlog.c) | A flight data recorder that never makes a task wait for the flash |
| [`eth.c`](firmware/node_b/src/net/eth.c) | Ethernet MAC and DMA descriptor driver underneath lwIP |
| [`arinc429.c`](firmware/common/src/arinc429.c), [`Arinc429Transmitter.cs`](renode/Arinc429Transmitter.cs) | ARINC 429 encoding, and the Renode model that decodes it independently |
| [`vib_features.c`](firmware/node_a/src/vib_features.c) | Fixed-size DFT features for the classifier, matching the Python reference |
| [`cosim.py`](sim/cosim.py) | Lockstep co-simulation of the plant model and both nodes |
| [`trace.py`](tools/trace/trace.py), [`misra.py`](tools/misra/misra.py), [`mcdc.py`](tools/coverage/mcdc.py) | The process checks CI runs on every commit |

## Problems worth mentioning

Things the project ran into, and how they were found and fixed:

- **Priority inversion, reproduced on purpose.** With a plain semaphore on the
  recorder, a medium-priority task kept the lock holder from running and
  ControlTask waited up to 85 ms, missing 3 deadlines; with a priority-inheritance
  mutex it waited 5 ms and missed none ([RTOS](docs/rtos.md)).
- **Checking code against something independent.** The C feature extraction is
  checked against the Python training code, the INT8 inference bit for bit
  against the reference interpreter, the telemetry encoder against the ground
  station's decoder with a shared reference packet, and the ARINC 429 encoder
  against a separate decoder in the Renode model.
- **The process tools found real defects.** Generating the traceability matrix
  turned up an LLR naming a test that no longer existed, and a wildcard that had
  linked a requirement to tests in five unrelated modules. Measuring MC/DC showed
  that no test checked an operator override request while landing.
- **Tools that disagree with the code.** cppcheck 2.10 silently ignored the
  `-isystem` include paths, so its first MISRA run never saw the FreeRTOS and lwIP
  headers; the checker now passes them explicitly and fails on any missing header.
  In CI, the MISRA report differed from the local one: its count of findings in
  third-party headers depended on how the submodules were checked out. That count
  is no longer recorded, and both checkers now print a diff when they fail.
- **An emulator is not hardware.** Renode's IMU model scales its readings
  differently from the datasheet and does not model the RCC reset flags; the
  co-simulation compensates for the first, and the reset cause falls back to a
  marker in the reset record for the second. What simulation cannot show is
  listed under *Limitations* below.

## Architecture

```mermaid
flowchart LR
    plant["<b>Plant model</b> (Python)<br/>vehicle, wind, autopilot<br/>IMU, GPS, line field, faults"]
    subgraph renode["Renode emulator"]
        direction TB
        a["<b>Node A</b>: sensing and safety<br/>FreeRTOS · IMU · GPS · flash recorder<br/>safety logic · TFLite Micro classifier"]
        b["<b>Node B</b>: gateway<br/>lwIP UDP telemetry · commands<br/>ARINC 429 output"]
        a -- "CAN 500 kbit/s" --> b
    end
    gs["<b>Ground station</b> (Python)<br/>map, plots, alarms, heartbeat"]
    plant -- "I2C, UART: sensors, autopilot" --> a
    a -- "UART: orders" --> plant
    b -- "UDP telemetry" --> gs
    gs -- "commands" --> b
```

Buses, pins, tasks and an end-to-end safety event: [architecture](docs/architecture.md).

## Quick start

Linux (or WSL on Windows) with `arm-none-eabi-gcc`, CMake 3.20+, Ninja, Python 3.11+
and Renode 1.17.

```sh
git clone --recurse-submodules https://github.com/1akin1/VoltDragon.git
cd VoltDragon
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"

# Build both nodes' firmware
cmake --preset debug
cmake --build --preset debug

# Run both nodes on a shared CAN bus (debug keys and GDB: docs/debugging.md)
renode renode/system.resc

# Fly the inspection mission: the plant model drives both nodes (docs/simulation.md)
python -m sim.mission --duration 90 --record build/mission.tlm
python -m ground_station.display --replay build/mission.tlm
# Other scenarios: no-avoidance, link-loss, battery, prop-damage, bearing-wear
python -m sim.mission --scenario link-loss --duration 25
```

To watch a flight live, Node B's Ethernet can be bridged to the host instead
(`sudo python -m sim.mission --tap`, then `python -m ground_station.display`; see
[telemetry](docs/telemetry.md)).

### Tests and checks

```sh
# Host unit tests (native gcc)
cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests

# Renode system tests (needs: pip install -r /opt/renode/tests/requirements.txt)
renode-test -r build/robot tests/robot/*.robot

# Closed-loop flights, checked against the plant's ground truth (about 20 minutes)
python -m pytest tests/integration

# Python tests and lint
pytest && ruff check .

# Process checks (each fails if its report is out of date)
python tools/trace/trace.py --check      # traceability matrix
python3 tools/misra/misra.py --check     # MISRA C:2012, cppcheck 2.10, after cmake --preset debug
python3 tools/coverage/mcdc.py --check   # MC/DC, clang 19
```

Retraining the vibration classifier (optional; the trained model is committed):
`pip install -e ".[ml]"`, then `python -m ml.dataset --out build/ml/dataset.npz` and
`python -m ml.train --dataset build/ml/dataset.npz` ([Edge AI](docs/edge-ai.md)).

## Documentation

| Topic | Pages |
|-------|-------|
| Overview | [architecture](docs/architecture.md), [roadmap](docs/roadmap.md), [demo video plan](docs/demo.md) |
| Requirements and process | [HLR](docs/requirements/HLR.md), [LLR](docs/requirements/LLR.md), [traceability matrix](docs/traceability.md), [MISRA compliance plan](docs/misra-deviations.md), [MISRA report](docs/misra-report.md), [MC/DC report](docs/mcdc-report.md) |
| Node A | [RTOS](docs/rtos.md), [autopilot link and safety logic](docs/autopilot-link.md), [Edge AI](docs/edge-ai.md), [vibration model report](docs/vibration-model-report.md) |
| Node B and buses | [CAN messages](docs/can-messages.md), [telemetry](docs/telemetry.md), [command interface](docs/command-interface.md), [ARINC 429](docs/arinc429.md) |
| Simulation and debugging | [simulation](docs/simulation.md), [debugging](docs/debugging.md), [Robot tests](tests/robot/README.md) |

## Repository layout

| Path | Contents |
|------|----------|
| `firmware/node_a` | Sensing and safety node (FreeRTOS) |
| `firmware/node_b` | Gateway node (bare metal) |
| `firmware/common` | Startup code, drivers and protocols shared by both nodes |
| `cmake/` | Cross-compilation toolchain file |
| `renode/` | Renode platforms and scripts; the ARINC 429 transmitter model |
| `sim/` | Plant model (`sim/plant`), co-simulation and the mission runner |
| `ground_station/` | Display, telemetry decoder, command tool |
| `ml/` | Vibration dataset, features, training and INT8 quantisation; trained models |
| `tests/unit` | Host unit tests (ctest), including the MC/DC test set for the safety logic |
| `tests/robot` | Robot Framework system tests in Renode |
| `tests/integration` | Closed-loop flights: plant model and both nodes |
| `tests/python` | pytest: plant model, ground station, ML pipeline |
| `tools/` | Traceability (`trace`), MISRA (`misra`), MC/DC (`coverage`), demo clips (`demo`), GDB helpers, TFLM vendoring |
| `third_party/` | lwIP 2.2.1 and FreeRTOS (submodules), the TensorFlow Lite Micro subset Node A uses |

## Limitations: to be verified on hardware

Renode is **not cycle-accurate**, and some peripherals are simplified (for example,
the RCC reset flags are not modelled; see [debugging](docs/debugging.md)). Renode's
LSM9DS1 model scales its outputs by ideal counts per unit rather than the datasheet
sensitivities, so the firmware reads the gyroscope 5 % and the magnetometer 14.7 % high
in simulation (see [node_a_imu.robot](tests/robot/node_a_imu.robot)); the co-simulation
compensates for this, and for other model differences listed in
[simulation](docs/simulation.md). The magnetometer and accelerometer axes are assumed
aligned; a real LSM9DS1 needs its magnetometer axes remapped. Timing figures such as
interrupt latency and inference time do not reflect real hardware.

The vibration classifier is trained on simulated vibration, and at Node A's 100 Hz
IMU rate the rotor tones (about 80 Hz) alias; real hardware would read the LSM9DS1
FIFO at 952 Hz and need data from real airframes
([Edge AI: limitations](docs/edge-ai.md#limitations)). The ARINC 429 transmitter is
a model with its own command set, not a real chip's ([ARINC 429](docs/arinc429.md)).
Electrical concerns (I2C pull-ups, CAN termination, signal integrity) are not
simulated. The STM32F4 Ethernet MAC needs an AHB clock of at least 25 MHz, but both
nodes run from the 16 MHz HSI; Node B needs the PLL enabled (and the clock-dependent
settings updated) before its Ethernet can work on silicon.

## Status

All seven phases of the [roadmap](docs/roadmap.md) are complete; the demo video is
being recorded from the [plan](docs/demo.md). Next steps beyond the roadmap:
insulator and hot-spot detection from a simulated camera, automatic fault reports
with GPS positions, and porting the firmware to a real Nucleo board.
