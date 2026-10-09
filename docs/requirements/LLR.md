# Low-Level Requirements (LLR)

Status: **Draft v0.1** · Phase 1

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
