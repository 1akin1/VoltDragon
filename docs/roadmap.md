# VoltDragon Roadmap

Total duration: about **6 weeks**, in 7 phases. Each phase ends with a milestone.

## Phase 0 - Setup (2-3 days)
- [x] Install Renode, arm-none-eabi-gcc, GDB, the Python environment and the Git repository
- [x] Write the first HLR list with 15-20 items ([HLR.md](requirements/HLR.md))

**Milestone:** The repo skeleton is ready and CI passes an empty build.

## Phase 1 - Single-node fundamentals (week 1)
- [x] Write our own startup code and linker script; bring up the UART at register level
- [x] Add a watchdog, a HardFault handler and a reset counter in a `.noinit` section

**Milestone:** Node A boots in Renode and prints logs to UART; a deliberate HardFault can be debugged.
✅ Verified by `tests/robot/node_a_boot.robot` (7 tests) and the GDB workflow in [debugging.md](debugging.md).

## Phase 2 - Sensors and communication (week 2)
- [x] Write the I2C driver and read the LSM9DS1 IMU at 100 Hz on Node A
- [x] Write the SPI driver and record flight data to the MT25Q flash at 10 Hz
- [x] Write the UART command parser for Node B, with host unit tests
- [x] Add Node B and set up CAN communication between the two nodes
- [x] Send UDP telemetry from Node B with lwIP and check it on the wire (Robot packet checks; Wireshark/Python receiver on a TAP link)

**Milestone:** IMU data travels from A to B over CAN, then on to the ground station over UDP.
✅ Verified by `tests/robot/system_telemetry.robot` and with the Python receiver over a TAP link ([telemetry.md](telemetry.md)).

## Phase 3 - Plant model and mission scenario (week 3)
- [x] Python flight model: progress along the line, wind, sensor noise ([simulation.md](simulation.md))
- [x] Add a magnetometer-corruption scenario near the line, and its detection on Node A (HLR-008)
- [x] Add a map and telemetry plots to the ground station, with alarms
- [x] Added beyond the original plan: a simulated GPS receiver (NMEA on UART4), so the
      position on the map comes from the vehicle's own telemetry rather than from the simulator

**Milestone:** The UAV flies along the simulated line and can be monitored from the ground station.
✅ Verified by `tests/integration/test_mission.py` (plant model driving both nodes in Renode)
and with the ground-station display over a TAP link ([simulation.md](simulation.md)).

## Phase 4 - RTOS and safety logic (week 4)
- [x] Move to FreeRTOS: `ImuTask`, `ControlTask`, `CanTxTask`, `LogTask` ([rtos.md](rtos.md));
      `AiTask` arrived with its model in Phase 5
- [x] Write the safe-distance check and the return-to-home-on-link-loss state machine,
      with the battery failsafes and operator mode commands ([autopilot-link.md](autopilot-link.md))
- [x] Reproduce a priority-inversion scenario and fix it with a mutex (85 ms → 5 ms worst-case wait)
- [x] Added beyond the original plan: an autopilot link (UART5) and a simulated autopilot
      in the plant, so the safety logic is tested in closed loop, not only open loop;
      Node B → Node A CAN messages; a ground station heartbeat; telemetry version 3

**Milestone:** The safety scenarios are verified with Robot Framework tests.
✅ Verified open loop by `tests/robot/system_safety.robot` and `node_a_rtos.robot`, and in
closed loop by `tests/integration/test_mission.py` (the close pass stays 10 m clear) and
`test_safety.py` (link loss, battery).

## Phase 5 - Edge AI (week 5)
- [x] Inject a motor/propeller vibration fault in the plant model and collect data: a damaged
      propeller or a worn motor bearing, 400 randomised flights ([edge-ai.md](edge-ai.md))
- [x] Train a small model, quantise it to INT8 and report the accuracy difference against FP32:
      99.98 % for both on the test windows, no false alarm in a 3 h soak
      ([report](vibration-model-report.md))
- [x] Run it on Node A with TFLite Micro and add the fault alarm to telemetry (CAN HEALTH
      message, telemetry version 4, ground-station alarm and fault-score plot)
- [x] Added beyond the original plan: two fault classes instead of one; the C features and the
      on-target model checked bit for bit against the Python reference; a vendored, minimal
      TFLite Micro subset with a script that regenerates it

**Milestone:** An injected fault is detected on the MCU and shows up at the ground station.
✅ Verified by `tests/integration/test_vibration.py` (the plant's fault at 15.0 s reaches Node B
at 15.999 s, HLR-009: within 2 s), `tests/robot/node_a_ai.robot` and `system_telemetry.robot`.

## Phase 6 - Process and documentation (week 6)
- [ ] Produce the HLR -> LLR -> code -> test traceability matrix
- [ ] Produce a MISRA report (cppcheck) and an MC/DC coverage report for one module
- [ ] Add ARINC 429 output to Node B (optional)
- [ ] Write the English README, architecture diagram and a short demo video

**Milestone:** The project is ready to show in a portfolio.

> If an interview is coming up, prioritise Phases 1, 2, 4 and 6 and simplify Phase 5.

## Future extensions
- Image processing: insulator damage or thermal hot-spot detection (Gazebo camera + YOLO nano)
- Mark fault locations on a map with GPS and generate reports automatically
- Port the firmware to a real Nucleo board
