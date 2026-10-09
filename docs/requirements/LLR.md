# Low-Level Requirements (LLR)

Status: **Draft v0.2** · Phase 2

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
