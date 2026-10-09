# MISRA C:2012 deviations

Each deviation names the rule, where it applies, why it is needed and how the
risk is controlled. Compliance is checked with cppcheck's MISRA addon in Phase 6
(HLR-020).

| ID | Rule | Location | Justification | Mitigation |
|----|------|----------|---------------|------------|
| D-001 | 17.1 (`<stdarg.h>` shall not be used) | `log.c` | A printf-style interface keeps diagnostic logging readable. | Only a small, fixed set of conversions is supported. GCC's `format(printf)` attribute checks every call at compile time, and logging is not used in control paths. |
| D-002 | 11.4 (conversion between pointer and integer) | `stm32f4_regs.h` | Memory-mapped peripherals are reached through fixed addresses. | Addresses come from RM0090. Register layouts are verified with `_Static_assert`. |
| D-003 | 18.2 / 18.3 (pointer subtraction/comparison between different objects) | `startup_stm32f4.c` | Section boundaries are separate linker symbols, but they delimit one contiguous region. | The linker script guarantees ordering and 4-byte alignment of the boundaries. |
| D-004 | 11.1 (conversion between function pointer and other type) | `fault.c`, `fault_trigger()` | Branching to an address with the Thumb bit cleared is deliberately invalid, to test the fault handler. | Only reachable from the debug console, to be removed or compiled out for flight builds. |
| D-005 | 2.1 / 1.2 (unreachable code, language extensions) | `fault.c`, `startup_stm32f4.c` | Naked functions and inline assembly are required for exception entry, `wfi`, `dsb`/`isb` and `udf`. | Assembly is limited to short, documented sequences. |
