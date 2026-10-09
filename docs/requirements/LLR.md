# Low-Level Requirements (LLR)

Status: **Draft v0.3** · Phase 2

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
| LLR-011 | The main loop shall reload the IWDG on every iteration. If the loop stalls, the node shall be reset within 500 ms (nominal). | HLR-016 | `main.c` | `Should Reset Through The Watchdog When The Main Loop Hangs` |
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
| LLR-043 | Node A shall read the accelerometer, gyroscope and magnetometer every 10 ms (100 Hz). If the main loop falls 20 ms or more behind, the missed samples shall be skipped rather than read in a burst. | HLR-007 | `imu.c` | `Should Sample The Imu At 100 Hz` |
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
| LLR-054 | Node A shall record a boot record (reset cause and count) at start-up and the latest valid IMU sample every 100 ms (10 Hz). | HLR-018 | `main.c` | `Should Record At 10 Hz Without Losses`, `Should Record The Boot And The Imu Data` |
| LLR-055 | Flash program and erase operations shall not be waited for in the main loop. Records shall be queued in RAM (8 records) and written by a polled state machine. | HLR-007, HLR-018 | `flashlog.c`, `mt25q.c` | `Should Sample The Imu At 100 Hz` (rate unaffected), Inspection |
| LLR-056 | Each 4 KiB subsector shall be erased just before its first record is written. | HLR-018 | `flashlog.c` | `Should Cross Erase Boundaries Without Errors` |
| LLR-057 | Each record shall be read back and compared after programming. On a mismatch, the slot shall be left as written and the record retried in the next slot. | HLR-018 | `flashlog.c` | Inspection (fault injection planned) |
| LLR-058 | Records that cannot be queued or written (queue full, flash full) shall be counted as dropped. Flash errors shall be counted, and both counts shall be logged once per second. | HLR-018 | `flashlog.c` | `Should Record At 10 Hz Without Losses` |

## Operator command interface (Node B)

Protocol details: [command-interface.md](../command-interface.md).

| ID | Requirement | Parent | Code | Test |
|----|-------------|--------|------|------|
| LLR-060 | Node B shall receive operator commands on USART3 (PB10/PB11, 115200 baud, 8N1) through an interrupt-driven 128-byte buffer, and shall count bytes lost to UART overruns or a full buffer. | HLR-014 | `board.c`, `uart.c`, `commands.c` | `Should Answer Back To Back Commands In Order` |
| LLR-061 | A frame shall start at `$` (any `$` restarts assembly) and end at CR or LF. Empty lines shall be ignored. Lines over 80 characters shall be rejected with `LENGTH`. | HLR-014 | `cmd_protocol.c` | Unit: `line_*`; Robot: `Should Resynchronise After Line Noise`, `Should Reject Corrupted Frames` |
| LLR-062 | A frame shall be rejected with `FRAME`, `CHECKSUM` or `SEQ`, and sequence number 0, if it is malformed, its XOR checksum does not match, or its sequence number is not in 1..65535. | HLR-013, HLR-014 | `cmd_protocol.c`, `cmd_dispatch.c` | Unit: `parse_rejects_*`; Robot: `Should Reject Corrupted Frames` |
| LLR-063 | A valid frame with an unknown verb shall be rejected with `UNKNOWN`; a wrong argument count or an argument value refused by the command shall be rejected with `ARGS`. Rejected commands shall have no effect. | HLR-014 | `cmd_dispatch.c`, `commands.c` | Unit: `rejects_*`; Robot: `Should Reject Unknown Commands`, `Should Reject Invalid Arguments` |
| LLR-064 | Every complete line shall receive exactly one response, `ACK` or `NAK`, carrying the request's sequence number when it can be trusted and a valid checksum. | HLR-014 | `cmd_dispatch.c`, `commands.c` | All `node_b_commands.robot` tests (responses are matched including their checksum) |
| LLR-065 | A request with the same sequence number as the previous one shall be answered with the previous response without executing the command again. | HLR-014 | `cmd_dispatch.c` | Unit: `replays_*`; Robot: `Should Replay The Response To A Retransmitted Command` |
| LLR-066 | Node B shall support `PING`, `VERSION`, `STATUS` and `TLM_RATE` (10 to 50 Hz, default 10 Hz). | HLR-012, HLR-014 | `commands.c` | `Should Answer Ping And Version`, `Should Report Status Counters`, `Should Set And Query The Telemetry Rate` |
| LLR-067 | Node B shall log each command and its outcome on the debug console. | (derived) | `commands.c` | `Should Replay The Response To A Retransmitted Command` |
