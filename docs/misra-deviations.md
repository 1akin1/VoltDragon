# MISRA C:2012 compliance plan and deviations

HLR-020 requires the flight software to comply with the project's MISRA C:2012
subset and every deviation to be documented. This page defines that subset and
records each deviation. It is also the input of the checker: `tools/misra/misra.py`
runs cppcheck's MISRA addon, sorts every finding into one of the tables below and
writes the [MISRA report](misra-report.md). CI fails on any finding that no table
accounts for, and on a false-positive entry that no longer matches anything.

## Scope

The subset applies to the project's C code: `firmware/common`, `firmware/node_a`
and `firmware/node_b`, including the generated model array `vib_model_data.c`.

Not in scope:

- Third-party code, like FreeRTOS, lwIP and the vendored TensorFlow Lite Micro
  (`third_party/`). The checker analyses its headers as part of our code, but it
  reports findings located in them separately and does not count them.
- Node A's thin C++ wrapper around TensorFlow Lite Micro (`vib_model.cc`,
  `tflm_port.cc`). MISRA C does not cover C++. The wrapper is kept small: it
  allocates nothing, uses no exceptions or RTTI, and is reached only through the C
  interface in `vib_model.h`. `tflm_port.cc` formats TFLM's messages with
  `vsnprintf`, under the same reasoning as D-001.
- Host unit tests (`tests/unit`), which are not flight software.

## Guideline categories

All mandatory and required rules apply. All advisory rules apply except the ones
disapplied below, as MISRA Compliance:2020 allows for advisory guidelines.

cppcheck's addon checks the rules that can be decided from the source; the
[report](misra-report.md) lists how many. The other rules, and the directives,
are covered by code review.

## Disapplied advisory rules

| Rule | Category | Justification |
|------|----------|---------------|
| 15.5 | Advisory | A function should have a single point of exit. Early returns on invalid arguments and failed checks keep the normal path unindented and short; forcing a single exit adds flags or nesting, which is a larger source of error in this code than an early return. |
| 8.9 | Advisory | An object should be defined at block scope if it is used in one function. Lookup tables (the flight-mode transition table, the CAN schedule, the route geometry) and large buffers are kept at file scope next to the definitions they mirror, so each can be reviewed against its source document in one place. They all have internal linkage (`static`). |

## Deviations

| ID | Rules | Files | Justification | Mitigation |
|----|-------|-------|---------------|------------|
| D-001 | 17.1 | `log.c` | A printf-style interface keeps diagnostic logging readable. | Only a small, fixed set of conversions is supported. GCC's `format(printf)` attribute checks every call at compile time, and logging is not used in control paths. |
| D-002 | 11.4 | `*` | Memory-mapped peripherals are reached through fixed addresses, and the Ethernet DMA descriptors hold buffer addresses as 32-bit bus addresses. | Peripheral addresses come from RM0090 and are used only through the macros in `stm32f4_regs.h`, whose register layouts are verified with `_Static_assert`. DMA buffers are statically allocated and word aligned. |
| D-003 | 8.11, 18.2, 18.3 | `startup_stm32f4.c` | The section boundaries are separate linker symbols, declared as arrays of unknown size, but they delimit one contiguous region. | The linker script guarantees their ordering and 4-byte alignment. cppcheck's `comparePointers` check is suppressed inline on the two loops for the same reason. |
| D-004 | 11.1, 11.6 | `fault.c` | `fault_trigger()` branches to an address with the Thumb bit cleared, and divides by zero, deliberately, to test the fault handler. | Only reachable from the debug console, to be removed or compiled out for flight builds. cppcheck's `zerodiv` check is suppressed inline on the division. |
| D-005 | 1.2, 2.1 | `fault.c`, `startup_stm32f4.c` | Naked functions and inline assembly are required for exception entry, `wfi`, `dsb`/`isb` and `udf`. | Assembly is limited to short, documented sequences. |
| D-006 | 10.8 | `net.c` | lwIP's `IP4_ADDR` macro casts composite expressions internally. lwIP is adopted code and its macros are its interface. | The macro is used only with constant arguments, to set Node B's fixed addresses. |

## Tool false positives

Findings that cppcheck 2.10 reports where the code does not break the rule. Each
entry matches findings by rule, file and a regular expression on the source line.

| ID | Rule | Files | Line pattern | Why the code complies |
|----|------|-------|--------------|-----------------------|
| FP-01 | 10.4 | `stm32f4_regs.h` | `offsetof\(` | `offsetof` yields a `size_t`, so both operands of `==` are essentially unsigned. |
| FP-02 | 12.1 | `log.c`, `eth.c`, `telemetry.c` | `[?:]` | Every operand of the conditional operator is a primary or parenthesised expression; cppcheck 2.10 reports these conditional expressions regardless. |
| FP-03 | 11.8 | `board.c` | `rmii_pins\[i\]\.port` | The `const` qualifies the table element, not the pointed-to registers: the pointer is passed by value and no qualifier is removed. |
| FP-04 | 20.10 | `fault.c` | `FAULT_ENTRY_ASM` | The macro uses no `#` or `##` operator; `#4` is an immediate operand inside an assembly string. |
| FP-05 | 21.1 | `stm32f4_regs.h` | `_Static_assert static_assert` | The `#define` is only seen by the C++ compiler (`__cplusplus`), for the TFLM wrapper; no C translation unit defines a reserved identifier. |
