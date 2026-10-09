# Low-Level Requirements (LLR)

Status: **Draft v0.5** · Phase 4

Low-level requirements refine the [high-level requirements](HLR.md) into
statements that can be implemented and tested directly. The full
HLR → LLR → code → test traceability matrix is produced in Phase 6.

## Start-up and platform

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-001 | Reset_Handler shall enable the FPU (CP10/CP11 full access) before executing any other code. | (derived) | `startup_stm32f4.c` | Inspection |
| LLR-002 | Reset_Handler shall copy `.data` from flash to RAM and zero `.bss` before calling `main()`. | (derived) | `startup_stm32f4.c` | All Robot tests (boot) |
| LLR-003 | The `.noinit` section shall not be initialised by start-up code or by ELF loaders, and shall be placed at the start of RAM. | HLR-017 | `stm32f407vg.ld` | `Should Count Consecutive Software Resets` |
| LLR-004 | System initialisation shall enable the integer divide-by-zero trap (`SCB_CCR.DIV_0_TRP`). | HLR-010 | `startup_stm32f4.c` | `Should Record A Divide By Zero Fault Across Reset` |
| LLR-005 | The link shall fail if less than 4 KiB of RAM remains for the main stack. | (derived) | `stm32f407vg.ld` | Build |

## Watchdog and reset history

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-010 | Each node shall start the IWDG with a nominal timeout of 500 ms before entering its main loop. | HLR-016 | `iwdg.c`, `main.c` | `Should Boot And Report A Cold Start` |
| LLR-011 | Node B's main loop shall reload the IWDG on every iteration; Node A's reload is supervised (LLR-113). If the loop or a task stalls, the node shall be reset within 500 ms (nominal). | HLR-016 | `main.c` (Node B), `tasks.c` (Node A) | `Should Reset Through The Watchdog When The Main Loop Hangs`, `Should Recover From A Stalled Task Through The Watchdog` |
| LLR-012 | The reset record shall be protected by a magic number and a CRC-32. On a mismatch it shall be reinitialised and the reset treated as power-on. | HLR-017 | `reset_info.c` | `Should Boot And Report A Cold Start` |
| LLR-013 | The reset counter shall increment on every warm reset and be reported in the boot log. | HLR-017 | `reset_info.c`, `node_boot.c` | `Should Count Consecutive Software Resets` |
| LLR-014 | The reset cause shall be taken from the RCC reset flags only if they can be cleared. Otherwise a firmware-requested reset shall be identified by a marker in the reset record. | HLR-017 | `reset_info.c` | `Should Count Consecutive Software Resets` |

## Fault handling

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-020 | On NMI, HardFault, MemManage, BusFault or UsageFault, the handler shall capture the stacked registers, EXC_RETURN, CFSR, HFSR, MMFAR and BFAR. | HLR-017 | `fault.c` | `Should Record ... Fault Across Reset` (×3) |
| LLR-021 | The fault handler shall store the captured state in the reset record before attempting any output. | HLR-017 | `fault.c` | `Should Record ... Fault Across Reset` (×3) |
| LLR-022 | After reporting, the fault handler shall halt if a debugger is attached, otherwise perform a software reset. | HLR-016 | `fault.c` | `Should Record ... Fault Across Reset` (×3) |
| LLR-023 | After a fault-induced reset, the boot log shall report the recorded fault with the decoded CFSR causes. | HLR-017 | `node_boot.c`, `fault.c` | `Should Record ... Fault Across Reset` (×3) |

## Console

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-030 | The console shall use USART2 at 115200 baud, 8N1, on PA2/PA3. | (derived) | `board.c`, `uart.c` | All Robot tests |
| LLR-031 | UART transmit waits shall be bounded so that logging can never block indefinitely. | HLR-016 | `uart.c` | Inspection |
| LLR-032 | Each log line shall carry a timestamp in seconds and milliseconds since boot and a severity letter. | (derived) | `log.c` | `Should Print A Heartbeat Every Second` |

## Sensor bus and IMU (Node A)

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-040 | The sensor bus shall be I2C1 on PB6 (SCL) / PB7 (SDA), open-drain, in standard mode at 100 kHz. | (derived) | `board.c`, `i2c.c` | All `node_a_imu.robot` tests |
| LLR-041 | Every I2C status wait shall be bounded. On a missing acknowledge, bus error, lost arbitration or timeout, the driver shall end the transfer with a STOP and return an error code. | HLR-010 | `i2c.c` | `Should Keep Running Without An Imu` |
| LLR-042 | At start-up Node A shall check the LSM9DS1 WHO_AM_I values (0x68 accel/gyro, 0x3D magnetometer) and configure ±2 g, ±245 dps and ±4 gauss full scale. If the check fails, it shall log the cause and keep running without IMU data. | HLR-010 | `lsm9ds1.c`, `imu.c` | `Should Detect The Imu At Boot`, `Should Keep Running Without An Imu` |
| LLR-043 | Node A shall read the accelerometer, gyroscope and magnetometer every 10 ms (100 Hz). The samples shall be taken by ImuTask on a fixed period (`xTaskDelayUntil`), so the rate does not drift with execution time. | HLR-007 | `imu.c`, `tasks.c` | `Should Sample The Imu At 100 Hz` |
| LLR-044 | Raw readings shall be converted with the datasheet sensitivities (0.061 mg, 8.75 mdps and 0.14 mgauss per LSB), rounded to the nearest unit. | HLR-007 | `lsm9ds1.c` | `Should Report The Sensor Values Fed To The Model`, `Should Track Changing Sensor Values` |
| LLR-045 | A failed IMU read shall mark the sample invalid until the next successful read, and an invalid sample shall not be returned to its users. | HLR-010 | `imu.c` | Inspection (runtime fault injection planned) |
| LLR-046 | Once per second Node A shall log the number of samples taken in the last second, the number of failed reads and the latest sample. | (derived) | `imu.c` | `Should Sample The Imu At 100 Hz` |

## Flight data recorder (Node A)

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-050 | The flight-data flash shall be an MT25Q on SPI1 (PA5/PA6/PA7, mode 0, 8 MHz) with a software chip select on PA4. Every SPI status wait shall be bounded. | (derived) | `board.c`, `spi.c`, `mt25q.c` | All `node_a_flashlog.robot` tests |
| LLR-051 | At start-up Node A shall check the flash JEDEC ID (Micron, 8 to 16 MiB). If the check fails, it shall log the cause and keep running without the recorder. | HLR-018 | `mt25q.c`, `flashlog.c` | `Should Find An Empty Flash On First Boot`, `Should Keep Running Without A Flash` |
| LLR-052 | The log shall be an append-only array of 64-byte records, each holding a magic number, type, payload length, a sequence number equal to its slot index, a timestamp and a CRC-32. | HLR-018 | `flashlog.c` | `Should Record The Boot And The Imu Data` |
| LLR-053 | At start-up the end of the log shall be found by a binary search for the first erased slot, and new records shall be appended after the existing ones. | HLR-018 | `flashlog.c` | `Should Keep The Log Across A Reset` |
| LLR-054 | Node A shall record a boot record (reset cause and count) at start-up and the latest valid IMU sample every 100 ms (10 Hz). | HLR-018 | `main.c`, `tasks.c` | `Should Record At 10 Hz Without Losses`, `Should Record The Boot And The Imu Data` |
| LLR-055 | Flash program and erase operations shall not be waited for by the tasks that queue records. Records shall be queued in RAM (8 records) under a mutex and written by LogTask through a polled state machine. | HLR-007, HLR-018 | `flashlog.c`, `mt25q.c` | `Should Sample The Imu At 100 Hz` (rate unaffected), Inspection |
| LLR-056 | Each 4 KiB subsector shall be erased just before its first record is written. | HLR-018 | `flashlog.c` | `Should Cross Erase Boundaries Without Errors` |
| LLR-057 | Each record shall be read back and compared after programming. On a mismatch, the slot shall be left as written and the record retried in the next slot. | HLR-018 | `flashlog.c` | Inspection (fault injection planned) |
| LLR-058 | Records that cannot be queued or written (queue full, flash full) shall be counted as dropped. Flash errors shall be counted, and both counts shall be logged once per second. | HLR-018 | `flashlog.c` | `Should Record At 10 Hz Without Losses` |

## Operator command interface (Node B)

Protocol details: [command-interface.md](../command-interface.md).

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-060 | Node B shall receive operator commands on USART3 (PB10/PB11, 115200 baud, 8N1) through an interrupt-driven 256-byte buffer, and shall count bytes lost to UART overruns or a full buffer. | HLR-014 | `board.c`, `uart.c`, `commands.c` | `Should Answer Back To Back Commands In Order` |
| LLR-061 | A frame shall start at `$` (any `$` restarts assembly) and end at CR or LF. Empty lines shall be ignored. Lines over 80 characters shall be rejected with `LENGTH`. | HLR-014 | `cmd_protocol.c` | Unit: `line_*`; Robot: `Should Resynchronise After Line Noise`, `Should Reject Corrupted Frames` |
| LLR-062 | A frame shall be rejected with `FRAME`, `CHECKSUM` or `SEQ`, and sequence number 0, if it is malformed, its XOR checksum does not match, or its sequence number is not in 1..65535. | HLR-013, HLR-014 | `cmd_protocol.c`, `cmd_dispatch.c` | Unit: `parse_rejects_*`; Robot: `Should Reject Corrupted Frames` |
| LLR-063 | A valid frame with an unknown verb shall be rejected with `UNKNOWN`; a wrong argument count or an argument value refused by the command shall be rejected with `ARGS`. Rejected commands shall have no effect. | HLR-014 | `cmd_dispatch.c`, `commands.c` | Unit: `rejects_*`; Robot: `Should Reject Unknown Commands`, `Should Reject Invalid Arguments` |
| LLR-064 | Every complete line shall receive exactly one response, `ACK` or `NAK`, carrying the request's sequence number when it can be trusted and a valid checksum. | HLR-014 | `cmd_dispatch.c`, `commands.c` | All `node_b_commands.robot` tests (responses are matched including their checksum) |
| LLR-065 | A request identical to the previous one (same sequence number and content) shall be answered with the previous response without executing the command again. A different request reusing the sequence number shall be executed. | HLR-014 | `cmd_dispatch.c` | Unit: `replays_*`, `different_request_with_same_sequence_is_executed`; Robot: `Should Replay The Response To A Retransmitted Command` |
| LLR-066 | Node B shall support `PING`, `VERSION`, `STATUS`, `CAN`, `TLM_RATE` (10 to 50 Hz, default 10 Hz) and `MODE` (LLR-126). | HLR-012, HLR-014 | `commands.c` | `Should Answer Ping And Version`, `Should Report Status Counters`, `Should Set And Query The Telemetry Rate` |
| LLR-067 | Node B shall log each command and its outcome on the debug console. | (derived) | `commands.c` | `Should Replay The Response To A Retransmitted Command` |

## Inter-node CAN link

Message layout: [can-messages.md](../can-messages.md).

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-070 | Both nodes shall use CAN1 (PB8/PB9) at 500 kbit/s with 11-bit identifiers, automatic retransmission and automatic bus-off recovery. Controller mode changes shall be bounded waits. | HLR-011 | `board.c`, `can.c` | All `system_can.robot` tests |
| LLR-071 | Every message shall be 8 bytes: 6 payload bytes, a per-identifier sequence counter and a CRC-8/SAE-J1850 over the identifier and the first 7 bytes. | HLR-013 | `can_msg.c`, `crc8.c` | Unit: `test_can_msg`; Robot: `Should Reject A Corrupted Frame` |
| LLR-072 | Node A shall send STATUS, ACCEL, GYRO, MAG, GPS_LAT, GPS_LON, NAV and SAFETY at 50 Hz each, in a fixed schedule of ten 2 ms slots. The IMU messages shall be sent only while the IMU sample is valid, and the position messages only while the GPS fix is fresh. | HLR-011 | `can_tx.c` | `Should Deliver Node A Data To Node B At 50 Hz`, `Should Flag Missing Imu Data From Node A`; integration: `test_can_and_telemetry_run_without_losses` (400 frames/s) |
| LLR-073 | Node B shall accept only identifiers 0x100..0x10F, and Node A only 0x200..0x20F, using the hardware acceptance filter. | HLR-013 | `can.c`, `can_rx.c`, `can_tx.c` | `Should Filter Out Foreign Identifiers In Hardware` |
| LLR-074 | Node B shall reject and count frames with a wrong length or CRC, and shall not use their content. | HLR-010, HLR-013 | `can_rx.c` | `Should Reject A Corrupted Frame` |
| LLR-075 | Node B shall count gaps of 1..127 in each identifier's sequence counter as lost frames, and shall resynchronise without counting on larger or backward jumps and when STATUS shows that Node A restarted. | HLR-013 | `can_msg.c`, `can_rx.c` | Unit: `sequence_tracker_*`; Robot: `Should Count A Lost Frame`, `Should Resynchronise When Node A Restarts` |
| LLR-076 | Node B shall keep the latest Node A data with the arrival time of the last valid frame, and report frame counts, error counts and data age once per second and through the `CAN` command. | HLR-010, HLR-014 | `can_rx.c`, `commands.c` | `Should Deliver Node A Data To Node B At 50 Hz`, `Should Report Link Statistics On The Command Interface`, `Should Report No Data Without Node A` |
| LLR-078 | Node B shall send B_STATUS every 100 ms with the time since the last intact ground station command and whether one has been heard. Node A shall reject and count Node B frames with a wrong length or CRC. | HLR-003, HLR-013 | `can_rx.c`, `can_tx.c` | Robot: `Should Keep The Mission While Heartbeats Arrive And Return Home When They Stop` |
| LLR-077 | Node A shall provide debug-console fault injection for the CAN link: wrong CRC, skipped sequence number and a foreign identifier. | (derived) | `can_tx.c`, `main.c` | `system_can.robot` fault tests |

## Network and telemetry (Node B)

Packet layout and network settings: [telemetry.md](../telemetry.md).

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-080 | Node B shall use the Ethernet MAC over RMII with an 802.3 PHY at MDIO address 0. Every MDIO and DMA reset wait shall be bounded, and a missing PHY shall be logged with the node continuing without network. | HLR-012 | `board.c`, `eth.c`, `net.c` | `Should Keep Running Without Ethernet` |
| LLR-081 | Node B shall bring up the link from the main loop by polling the PHY every 100 ms, and shall configure the MAC for the speed and duplex common to both link partners. | HLR-012 | `eth.c`, `net.c` | All `system_telemetry.robot` tests (100 Mbit/s full duplex) |
| LLR-082 | Node B shall run lwIP without an operating system (IPv4, ARP, ICMP, UDP) with the static address 192.168.10.2/24. | HLR-012 | `net.c`, `lwipopts.h` | `Should Answer Commands Over Udp` |
| LLR-083 | Node B shall broadcast an 88-byte telemetry packet (version 3) to UDP port 5600 at the rate set by `TLM_RATE` (10 Hz default), carrying Node A's latest sensor data, position, heading and its source, field strength, data ages and freshness, both nodes' uptime and reset counts, the CAN link counters, Node A's safety state (mode, distance, battery, flags, last request) and the ground link age. | HLR-012 | `telemetry.c`, `tlm_msg.c` | `Should Broadcast Telemetry At 10 Hz`, `Should Carry Node A Imu Data To The Ground Station`, `Should Flag Fresh Node A Data`, `Should Carry Node A Safety State`, `Should Change The Telemetry Rate On Command` |
| LLR-084 | Every telemetry packet shall carry a magic number, a version, a sequence number that advances per packet sent, and a CRC-32 over the rest of the packet. | HLR-013 | `tlm_msg.c` | Unit: `test_tlm_msg`; pytest: `test_telemetry.py` (same reference packet) |
| LLR-085 | Node B shall accept operator commands on UDP port 5601, one frame per datagram, process them like UART commands with a separate retransmission cache, and send each reply to the sender. | HLR-014 | `net.c`, `commands.c` | `Should Answer Commands Over Udp`, `Should Reject A Corrupted Udp Command` |

## GPS and navigation (Node A)

Design: [simulation.md](../simulation.md) describes the scenario these are verified in.

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-090 | Node A shall receive NMEA 0183 from a GPS receiver on UART4 (PC10/PC11, 9600 baud) through an interrupt-driven buffer, and shall parse GGA and RMC sentences with any talker ID, in integer fixed point. | HLR-010 | `gps.c`, `nmea.c` | Unit: `test_nmea`; Robot: `Should Parse A Gps Fix`; integration: `test_gps_runs_at_5_fixes_per_second` |
| LLR-091 | A sentence with a framing, checksum or field error shall be counted and shall leave the previous fix unchanged. | HLR-010 | `nmea.c`, `gps.c` | Unit: `rejects_bad_sentences_without_changing_the_fix`; Robot: `Should Count A Corrupted Gps Sentence` |
| LLR-092 | A GPS fix older than 1 s shall be treated as lost. | HLR-010 | `gps.c` | Robot: `Should Report No Fix Without A Gps` |
| LLR-093 | Node A shall estimate the gravity direction with a complementary filter: gyroscope propagation every sample, and a correction towards the accelerometer with a 2 s time constant only while the accelerometer reads within 3 % of 1 g. | HLR-008 | `nav.c` | Integration: `test_magnetometer_is_flagged_only_near_the_line` (no false alarm while accelerating) |
| LLR-094 | Node A shall compute the true heading from the tilt-compensated magnetometer and the local declination (5 degrees east). | HLR-008 | `nav.c` | Robot: `Should Compute The Magnetometer Heading`; integration: `test_magnetometer_heading_matches_truth_in_cruise` |
| LLR-095 | Node A shall treat a magnetometer sample as disturbed if its field strength differs from 500 mG by more than 15 %, or its dip angle from 56 degrees by more than 8 degrees. The magnetometer shall be declared disturbed after 5 consecutive disturbed samples and usable again after 100 consecutive good ones. | HLR-008 | `nav.c` | Robot: `Should Stop Using A Disturbed Magnetometer And Recover`; integration: `test_magnetometer_is_flagged_only_near_the_line` |
| LLR-096 | While the magnetometer is disturbed, Node A shall not use its heading; it shall use the GPS course over ground when the ground speed is at least 1 m/s, and otherwise report no heading. | HLR-008 | `nav.c` | Robot: `Should Fall Back To The Gps Course`; integration: `test_heading_falls_back_to_gps_course_while_disturbed` |

## Ground station

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-100 | The ground station shall decode version 3 telemetry, reject packets with a wrong length, magic, version or CRC, and count lost packets from the sequence number. | HLR-013, HLR-019 | `ground_station/telemetry.py` | pytest: `test_telemetry.py` |
| LLR-101 | The ground station shall show the route, the vehicle's track and position, the distance to the nearest conductor, the measured field strength and the heading with its source. | HLR-019 | `ground_station/display.py`, `state.py` | pytest: `test_display_renders_a_replay`; inspection of a mission snapshot |
| LLR-102 | The ground station shall raise alarms for: no telemetry for 1 s; distance to a conductor below 12 m (warning) and 10 m (critical), from the GPS position and the route geometry; stale Node A data; no GPS fix; IMU invalid; magnetometer disturbed; recorder off; packet loss and CAN errors in the last 5 s; and from Node A's safety state: RETURN_TO_HOME and LAND (critical), HOLD, vehicle lost the command link, battery low and critical, autopilot silent, avoiding, and safety state unknown. Critical alarms shall be listed first. | HLR-001 to HLR-004, HLR-019 | `ground_station/state.py` | pytest: `test_ground_station.py` |
| LLR-103 | The ground station display shall send `PING` to Node B once a second while it receives live telemetry, unless disabled. | HLR-003 | `ground_station/command.py`, `display.py` | pytest: `test_heartbeat_pings_once_a_second_and_counts_replies` |

## Real-time operating system (Node A)

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-110 | Node A shall run FreeRTOS with five tasks in rate-monotonic priority order: CanTxTask (5, 2 ms), ImuTask (4, 10 ms), ControlTask (3, 20 ms), LoadTask (2, demonstration only) and LogTask (1, 10 ms), each periodic task on a fixed period. | HLR-007, HLR-011 | `tasks.c` | `Should Run The Tasks Under FreeRTOS` |
| LLR-111 | All kernel objects (tasks, stacks, mutexes) shall be statically allocated; dynamic allocation shall be disabled. | (derived) | `FreeRTOSConfig.h`, `tasks.c`, `lock.h` | Build (no heap linked) |
| LLR-112 | Interrupt handlers shall not call the kernel, and shall run above `configMAX_SYSCALL_INTERRUPT_PRIORITY` so the kernel never masks them. | HLR-016 | `board.c`, `FreeRTOSConfig.h` | Inspection |
| LLR-113 | LogTask shall reload the IWDG only if CanTxTask, ImuTask and ControlTask have each completed a cycle since the previous reload, checked every 100 ms. | HLR-016 | `tasks.c` | `Should Recover From A Stalled Task Through The Watchdog` |
| LLR-114 | Data shared between tasks (IMU sample, GPS fix, navigation state, safety state, recorder queue, log output) shall be accessed under mutexes with priority inheritance. | HLR-016 | `imu.c`, `gps.c`, `nav.c`, `safety.c`, `flashlog.c`, `log.c` | `Should Bound The Wait With Priority Inheritance` |
| LLR-115 | A stack overflow or a failed kernel assertion shall be logged and shall reset the node. Each task shall keep at least 100 words of its stack unused. | HLR-016, HLR-017 | `rtos_port.c` | `Should Report Task Stack Margins` |
| LLR-116 | Node A shall provide a debug-console demonstration of priority inversion: the recorder lock as a binary semaphore (`i`) and as a mutex (`m`), reporting ControlTask's longest wait and missed deadlines. | (derived) | `tasks.c`, `flashlog.c` | `Should Show Priority Inversion With A Plain Semaphore`, `Should Bound The Wait With Priority Inheritance` |

## Flight modes and safety (Node A)

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-120 | Mode changes shall follow the transition table: RETURN_TO_HOME and LAND reachable from any mode but LAND; MISSION and HOLD to each other on operator request only; out of RETURN_TO_HOME to MISSION or HOLD on operator request with override only; none out of LAND. | HLR-005, HLR-006 | `flight_mode.c` | Unit: `test_flight_mode.c` (all 128 cases) |
| LLR-121 | Every 20 ms Node A shall compute the distance from the latest fresh GPS fix to the nearest conductor of the route. Below 12 m it shall raise PROXIMITY_WARNING and order the autopilot to keep 15 m, in the same cycle; the warning and the order shall end when the distance exceeds 15 m. | HLR-001, HLR-002 | `safety.c`, `route.c` | Unit: `test_route.c`; Robot: `Should Warn And Keep Away From The Line`; integration: `test_close_pass_keeps_10_m_from_the_line`, `test_proximity_warning_and_avoidance_order` |
| LLR-122 | Once a ground station has been heard, Node A shall enter RETURN_TO_HOME when the ground link age (Node B's report plus the time since that report) exceeds 3 s. | HLR-003 | `safety.c` | Robot: `Should Keep The Mission While Heartbeats Arrive And Return Home When They Stop`, `Should Not Lose The Link Before A Ground Station Is Heard`; integration: `test_link_loss_returns_home_after_3_s` |
| LLR-123 | Node A shall enter RETURN_TO_HOME when the autopilot's battery report falls below 20 %, and LAND below 10 %, each when the condition starts. | HLR-004 | `safety.c` | Robot: `Should Return Home On Low Battery And Land On Critical Battery`; integration: `test_low_battery_returns_home_then_critical_battery_lands` |
| LLR-124 | Node A shall apply operator mode requests from Node B through the transition table, log and count rejected ones, and report the last handled request id. A request id repeated within 500 ms shall be ignored. | HLR-005, HLR-006 | `safety.c` | Robot: `Should Need An Override To Leave Return To Home`, `Should Hold And Resume On Operator Request`, `Should Treat Land As Final` |
| LLR-125 | Node A shall send its safety state (mode, distance, battery, flags, last request id) in the CAN SAFETY message at 50 Hz. | HLR-012 | `can_tx.c`, `safety.c` | Robot: `Should Return Home On Low Battery And Land On Critical Battery` (Node B's report) |
| LLR-126 | Node B's `MODE <mode> [OVERRIDE]` command shall check the request against the transition table and the mode Node A is expected to be in (the last forwarded request's mode until Node A confirms it or 500 ms pass, otherwise Node A's reported mode), answer `REFUSED` for a transition not allowed or before Node A has reported its mode, and forward allowed requests in a MODE_REQ frame. | HLR-005, HLR-006, HLR-014 | `commands.c`, `can_rx.c` | Robot: `Should Need An Override To Leave Return To Home`, `Should Reject Malformed Mode Requests`, `Should Treat Land As Final` |

## Autopilot link (Node A)

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-130 | Node A shall talk to the autopilot on UART5 (PC12/PD2, 115200 baud) with checksummed `$VDCMD` and `$VDAPS` sentences (docs/autopilot-link.md); reception shall be interrupt-driven. | HLR-002 | `ap_link.c`, `ap_msg.c` | Unit: `test_ap_msg.c` |
| LLR-131 | Node A shall send `$VDCMD` with the current mode and avoidance order every 100 ms, and at once when either changes. | HLR-002 to HLR-004 | `safety.c` | Robot: `Should Report The Safety State At Start`, `Should Warn And Keep Away From The Line` |
| LLR-132 | Node A shall count and drop invalid `$VDAPS` sentences, and treat the autopilot as silent (battery unknown) after 1 s without a valid one. | HLR-010 | `ap_link.c` | Unit: `test_ap_msg.c`; Robot: `Should Report The Safety State At Start` |
