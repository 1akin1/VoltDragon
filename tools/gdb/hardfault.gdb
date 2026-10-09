# GDB helpers for post-mortem analysis of Cortex-M faults.
#
# Usage:  gdb-multiarch -x tools/gdb/hardfault.gdb build/debug/firmware/node_a/node_a.elf
#         (gdb) target remote :3333
#         (gdb) continue          -- then trigger the fault, e.g. press 'f' on the console

set pagination off
set print pretty on

# Stop as soon as any fault exception reaches the common C handler.
break fault_handler_c

# Show where the fault happened, using the exception frame stacked by hardware.
define fault-frame
    printf "faulting pc : 0x%08x\n", frame[6]
    info symbol frame[6]
    list *frame[6]
    printf "caller (lr) : 0x%08x\n", frame[5]
    info symbol frame[5]
    printf "CFSR=0x%08x HFSR=0x%08x MMFAR=0x%08x BFAR=0x%08x\n", *(unsigned int *)0xE000ED28, *(unsigned int *)0xE000ED2C, *(unsigned int *)0xE000ED34, *(unsigned int *)0xE000ED38
end
document fault-frame
Decode the stacked exception frame inside fault_handler_c: faulting PC and caller, and the fault status registers.
end
