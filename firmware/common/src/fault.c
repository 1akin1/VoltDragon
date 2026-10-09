/**
 * @file fault.c
 * @brief Fault exception handling and deliberate fault injection for testing.
 */
#include "fault.h"

#include <stdint.h>

#include "log.h"
#include "stm32f4_regs.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void fault_handler_c(const uint32_t *frame, uint32_t exc_return);

#define IPSR_EXCEPTION_MASK (0x1FFUL)
#define HFSR_FORCED         (1UL << 30)

/* Stacked exception frame layout (ARMv7-M basic frame). */
enum
{
    FRAME_R0 = 0,
    FRAME_R1,
    FRAME_R2,
    FRAME_R3,
    FRAME_R12,
    FRAME_LR,
    FRAME_PC,
    FRAME_XPSR
};

/*
 * Common entry for all fault exceptions, installed directly in the vector
 * table under each handler name. Bit 2 of EXC_RETURN tells whether the
 * faulting code used the main (MSP) or process (PSP) stack; the stacked frame
 * is passed to the C handler together with EXC_RETURN.
 */
#define FAULT_ENTRY_ASM                 \
    "tst   lr, #4            \n"        \
    "ite   eq                \n"        \
    "mrseq r0, msp           \n"        \
    "mrsne r0, psp           \n"        \
    "mov   r1, lr            \n"        \
    "b     fault_handler_c   \n"

__attribute__((naked)) void NMI_Handler(void)        { __asm volatile(FAULT_ENTRY_ASM); }
__attribute__((naked)) void HardFault_Handler(void)  { __asm volatile(FAULT_ENTRY_ASM); }
__attribute__((naked)) void MemManage_Handler(void)  { __asm volatile(FAULT_ENTRY_ASM); }
__attribute__((naked)) void BusFault_Handler(void)   { __asm volatile(FAULT_ENTRY_ASM); }
__attribute__((naked)) void UsageFault_Handler(void) { __asm volatile(FAULT_ENTRY_ASM); }

static const char *exception_name(uint32_t exception)
{
    const char *name;

    switch (exception)
    {
        case 2U:  name = "NMI";        break;
        case 3U:  name = "HardFault";  break;
        case 4U:  name = "MemManage";  break;
        case 5U:  name = "BusFault";   break;
        case 6U:  name = "UsageFault"; break;
        default:  name = "Exception";  break;
    }
    return name;
}

/* Names of the CFSR bits: MMFSR [7:0], BFSR [15:8], UFSR [31:16]. */
static const char *cfsr_bit_name(uint32_t bit)
{
    static const char *const names[32] = {
        "IACCVIOL", "DACCVIOL", 0, "MUNSTKERR", "MSTKERR", "MLSPERR", 0, "MMARVALID",
        "IBUSERR", "PRECISERR", "IMPRECISERR", "UNSTKERR", "STKERR", "LSPERR", 0, "BFARVALID",
        "UNDEFINSTR", "INVSTATE", "INVPC", "NOCP", 0, 0, 0, 0,
        "UNALIGNED", "DIVBYZERO", 0, 0, 0, 0, 0, 0,
    };
    return (bit < 32U) ? names[bit] : 0;
}

void fault_print(const fault_record_t *record)
{
    LOG_ERROR("%s (exception %lu)", exception_name(record->exception), record->exception);
    LOG_ERROR("  pc=0x%08lx lr=0x%08lx xpsr=0x%08lx exc_return=0x%08lx",
              record->pc, record->lr, record->xpsr, record->exc_return);
    LOG_ERROR("  r0=0x%08lx r1=0x%08lx r2=0x%08lx r3=0x%08lx r12=0x%08lx",
              record->r0, record->r1, record->r2, record->r3, record->r12);
    LOG_ERROR("  cfsr=0x%08lx hfsr=0x%08lx mmfar=0x%08lx bfar=0x%08lx",
              record->cfsr, record->hfsr, record->mmfar, record->bfar);

    log_printf("                 cause:");
    for (uint32_t bit = 0U; bit < 32U; ++bit)
    {
        const char *name = cfsr_bit_name(bit);
        if (((record->cfsr & (1UL << bit)) != 0U) && (name != 0))
        {
            log_printf(" %s", name);
        }
    }
    if ((record->hfsr & HFSR_FORCED) != 0U)
    {
        log_printf(" FORCED");
    }
    log_printf("\r\n");
}

void fault_handler_c(const uint32_t *frame, uint32_t exc_return)
{
    fault_record_t record;
    uint32_t ipsr;

    __asm volatile("mrs %0, ipsr" : "=r"(ipsr));

    record.exception = ipsr & IPSR_EXCEPTION_MASK;
    record.r0 = frame[FRAME_R0];
    record.r1 = frame[FRAME_R1];
    record.r2 = frame[FRAME_R2];
    record.r3 = frame[FRAME_R3];
    record.r12 = frame[FRAME_R12];
    record.lr = frame[FRAME_LR];
    record.pc = frame[FRAME_PC];
    record.xpsr = frame[FRAME_XPSR];
    record.exc_return = exc_return;
    record.cfsr = SCB->CFSR;
    record.hfsr = SCB->HFSR;
    record.mmfar = SCB->MMFAR;
    record.bfar = SCB->BFAR;

    reset_info_store_fault(&record);

    LOG_ERROR("*** FAULT ***");
    fault_print(&record);

    if ((CORE_DHCSR & CORE_DHCSR_C_DEBUGEN) != 0U)
    {
        LOG_ERROR("debugger attached: halting");
        /* Inspect 'record' here, or the stacked frame at 'frame'. */
        __asm volatile("bkpt #0");
    }

    LOG_ERROR("resetting");
    log_flush();
    reset_info_system_reset();
}

void fault_trigger(fault_test_t kind)
{
    switch (kind)
    {
        case FAULT_TEST_UNDEFINED_INSTRUCTION:
            /* Permanently undefined instruction: UsageFault (UNDEFINSTR). */
            __asm volatile("udf #0");
            break;

        case FAULT_TEST_DIVIDE_BY_ZERO:
        {
            /* Traps because startup sets SCB_CCR.DIV_0_TRP: UsageFault (DIVBYZERO). */
            volatile uint32_t divisor = 0U;
            volatile uint32_t result = 100U / divisor;
            (void)result;
            break;
        }

        case FAULT_TEST_INVALID_STATE:
        {
            /* Branch with the Thumb bit clear: UsageFault (INVSTATE). */
            volatile uint32_t address = (uint32_t)&fault_trigger & ~1UL;
            void (*func)(void) = (void (*)(void))address;
            func();
            break;
        }

        default:
            break;
    }
}
