/**
 * @file reset_info.h
 * @brief Reset cause, reset counter and last-fault record kept across resets (HLR-017).
 *
 * The record lives in the .noinit RAM section, which the startup code does not
 * initialise. Its integrity is checked with a magic number and a CRC-32, so
 * random RAM contents after power-on are detected and discarded.
 */
#ifndef RESET_INFO_H
#define RESET_INFO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    RESET_CAUSE_POWER_ON = 0,   /**< Cold boot: no valid record in RAM */
    RESET_CAUSE_PIN,            /**< NRST pin */
    RESET_CAUSE_BROWN_OUT,      /**< Supply dropped below the BOR threshold */
    RESET_CAUSE_SOFTWARE,       /**< Requested by the firmware (e.g. after a fault) */
    RESET_CAUSE_IWDG,           /**< Independent watchdog expired */
    RESET_CAUSE_WWDG,           /**< Window watchdog expired */
    RESET_CAUSE_LOW_POWER,      /**< Illegal low-power mode entry */
    RESET_CAUSE_UNKNOWN         /**< Warm reset with no hardware or software cause recorded */
} reset_cause_t;

/** CPU state captured by the fault handler. */
typedef struct
{
    uint32_t exception;     /**< Active exception number (IPSR) */
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;            /**< Link register at the time of the fault */
    uint32_t pc;            /**< Address of the faulting instruction */
    uint32_t xpsr;
    uint32_t exc_return;    /**< EXC_RETURN value of the fault handler */
    uint32_t cfsr;          /**< Configurable Fault Status Register */
    uint32_t hfsr;          /**< HardFault Status Register */
    uint32_t mmfar;         /**< MemManage fault address */
    uint32_t bfar;          /**< BusFault address */
} fault_record_t;

/**
 * Validates the persistent record, determines the reset cause and increments
 * the reset counter. Must be called once, early in main().
 */
void reset_info_init(void);

/** Cause of the most recent reset. */
reset_cause_t reset_info_cause(void);

/** Human-readable name of a reset cause. */
const char *reset_info_cause_name(reset_cause_t cause);

/**
 * Whether the RCC reset flags could be cleared and so identify this reset.
 * When false, only POWER_ON, SOFTWARE and UNKNOWN (watchdog or pin) can be
 * distinguished, using the persistent record alone.
 */
bool reset_info_flags_reliable(void);

/** Number of warm resets since the last cold boot. */
uint32_t reset_info_count(void);

/**
 * Returns the fault that ended the previous run, if any.
 * @return true and copies the record to @p out if a fault was recorded.
 */
bool reset_info_last_fault(fault_record_t *out);

/** Saves a fault record so it can be reported after the next reset. */
void reset_info_store_fault(const fault_record_t *record);

/** Records a software reset and resets the MCU. Does not return. */
__attribute__((noreturn)) void reset_info_system_reset(void);

#endif /* RESET_INFO_H */
