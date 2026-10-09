/**
 * @file fault.h
 * @brief Fault exception handling and deliberate fault injection for testing.
 *
 * On any fault (HardFault, MemManage, BusFault, UsageFault, NMI) the handler:
 *   1. captures the stacked registers and fault status registers,
 *   2. saves them in the persistent reset record (HLR-017),
 *   3. prints a fault report on the console,
 *   4. stops at a breakpoint if a debugger is attached, otherwise
 *   5. performs a software reset.
 */
#ifndef FAULT_H
#define FAULT_H

#include "reset_info.h"

typedef enum
{
    FAULT_TEST_UNDEFINED_INSTRUCTION = 0,
    FAULT_TEST_DIVIDE_BY_ZERO,
    FAULT_TEST_INVALID_STATE
} fault_test_t;

/** Prints a fault record in human-readable form. */
void fault_print(const fault_record_t *record);

/** Deliberately triggers a fault of the given kind. For testing only. */
void fault_trigger(fault_test_t kind);

#endif /* FAULT_H */
