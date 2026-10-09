# Running and debugging in Renode

All commands run from the repository root inside WSL (Debian).

## Run Node A

```sh
cmake --preset debug && cmake --build --preset debug
renode renode/node_a.resc                              # GUI, opens a UART window
renode --console --disable-gui renode/node_a.resc      # headless, UART lines in the log
```

Type `start` at the Renode monitor prompt. The console prints the boot banner, the
reset cause and counter, and a heartbeat every second.

### Debug keys

Send single characters on the UART window (or `usart2 WriteChar <ascii>` in the monitor):

| Key | Action |
|-----|--------|
| `f` | HardFault from an undefined instruction (`UNDEFINSTR`) |
| `z` | Divide-by-zero fault (`DIVBYZERO`) |
| `s` | Invalid-state fault, branch to ARM state (`INVSTATE`) |
| `w` | Hang the main loop; the watchdog resets the node after about 500 ms |
| `r` | Software reset |
| `?` | Help |
| `d` | Node A only: read back and print the last 5 flight-log records from SPI flash |

## Fault handling

Every fault exception ends up in `fault_handler_c()` (`firmware/common/src/fault.c`), which:

1. reads the stacked exception frame (`r0-r3, r12, lr, pc, xPSR`) from MSP or PSP,
   depending on bit 2 of `EXC_RETURN`;
2. reads `CFSR`, `HFSR`, `MMFAR` and `BFAR`;
3. stores everything in the `.noinit` reset record (CRC-protected);
4. prints a report and decodes the CFSR bits;
5. halts on `bkpt` if a debugger is attached (`DHCSR.C_DEBUGEN`), otherwise resets.

On the next boot the record is printed again under "previous run ended with a fault".
This is useful when no debugger was attached.

Example:

```
[     2.502] E HardFault (exception 3)
[     2.502] E   pc=0x08000270 lr=0x0800086d xpsr=0x81000000 exc_return=0xfffffff9
[     2.502] E   cfsr=0x00010000 hfsr=0x40000000 mmfar=0x00000000 bfar=0x00000000
                 cause: UNDEFINSTR FORCED
```

`FORCED` means a configurable fault (here a UsageFault) was escalated to HardFault,
because the dedicated fault handlers are not enabled in `SHCSR`. To find the source
line from the saved PC without a debugger:

```sh
arm-none-eabi-addr2line -e build/debug/firmware/node_a/node_a.elf -f 0x08000270
```

## Debugging a HardFault with GDB

Start Renode with a GDB server:

```sh
renode --console --disable-gui -e 'include @renode/node_a.resc; machine StartGdbServer 3333'
```

In a second terminal:

```sh
gdb-multiarch -x tools/gdb/hardfault.gdb build/debug/firmware/node_a/node_a.elf
(gdb) target remote :3333
(gdb) continue
# press 'f' in the UART window; GDB stops at fault_handler_c
(gdb) fault-frame
faulting pc : 0x08000270
fault_trigger + 20 in section .text
0x8000270 is in fault_trigger (firmware/common/src/fault.c:154).
154             __asm volatile("udf #0");
caller (lr) : 0x08000881
node_debug_console_poll + 81 in section .text
CFSR=0x00010000 HFSR=0x40000000 MMFAR=0x00000000 BFAR=0x00000000
```

`fault-frame` is defined in `tools/gdb/hardfault.gdb`. It decodes the hardware-stacked
frame, so it still works when GDB cannot unwind through the exception.

## Emulator differences (to be verified in the hardware phase)

| Item | Renode behaviour | Firmware handling |
|------|------------------|-------------------|
| RCC_CSR reset flags | Power-on flags always set; `RMVF` is ignored | Flags are only trusted if they read back as cleared. Otherwise the reset cause comes from the `.noinit` record, and watchdog/pin resets report `UNKNOWN`. |
| `SCB_CCR.DIV_0_TRP` | Ignored by default | `renode/node_a.resc` sets `nvic FilterCcrDiv0Write false` |
| SysTick clock | 72 MHz in the stock model | `renode/stm32f407.repl` sets 16 MHz to match the HSI |
| `DHCSR.C_DEBUGEN` | Reads 0 with GDB attached | Use a breakpoint on `fault_handler_c` instead of relying on `bkpt` |
| LSI accuracy | Exactly 32 kHz | Real LSI varies (17-47 kHz); size watchdog margins accordingly |
