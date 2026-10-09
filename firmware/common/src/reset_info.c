/**
 * @file reset_info.c
 * @brief Reset cause, reset counter and last-fault record kept across resets (HLR-017).
 */
#include "reset_info.h"

#include <stddef.h>

#include "stm32f4_regs.h"

#define RESET_INFO_MAGIC        (0x51A7B007UL)
#define SW_RESET_MARKER         (0x5EF7BEEFUL)
#define RCC_CSR_RESET_FLAGS     (RCC_CSR_BORRSTF | RCC_CSR_PINRSTF | RCC_CSR_PORRSTF | \
                                 RCC_CSR_SFTRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | \
                                 RCC_CSR_LPWRRSTF)

typedef struct
{
    uint32_t       magic;
    uint32_t       reset_count;
    uint32_t       sw_reset_marker;  /**< Set just before a firmware-requested reset */
    uint32_t       fault_valid;
    fault_record_t fault;
    uint32_t       crc;              /**< CRC-32 of all preceding fields */
} persistent_record_t;

__attribute__((section(".noinit")))
static persistent_record_t s_record;

/* Snapshot taken at boot; s_record itself is updated for the next reset. */
static reset_cause_t  s_cause;
static bool           s_flags_reliable;
static bool           s_had_fault;
static fault_record_t s_last_fault;

/* Bitwise CRC-32 (IEEE 802.3, reflected). Small and fast enough for ~70 bytes. */
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static uint32_t record_crc(void)
{
    return crc32((const uint8_t *)&s_record, offsetof(persistent_record_t, crc));
}

static void record_seal(void)
{
    s_record.crc = record_crc();
}

static bool record_is_valid(void)
{
    return (s_record.magic == RESET_INFO_MAGIC) && (s_record.crc == record_crc());
}

/* Decodes RCC_CSR flags, most specific first (a power-on also sets PIN and BOR). */
static reset_cause_t cause_from_rcc(uint32_t csr)
{
    reset_cause_t cause = RESET_CAUSE_UNKNOWN;

    if ((csr & RCC_CSR_IWDGRSTF) != 0U)
    {
        cause = RESET_CAUSE_IWDG;
    }
    else if ((csr & RCC_CSR_WWDGRSTF) != 0U)
    {
        cause = RESET_CAUSE_WWDG;
    }
    else if ((csr & RCC_CSR_LPWRRSTF) != 0U)
    {
        cause = RESET_CAUSE_LOW_POWER;
    }
    else if ((csr & RCC_CSR_SFTRSTF) != 0U)
    {
        cause = RESET_CAUSE_SOFTWARE;
    }
    else if ((csr & RCC_CSR_PORRSTF) != 0U)
    {
        cause = RESET_CAUSE_POWER_ON;
    }
    else if ((csr & RCC_CSR_BORRSTF) != 0U)
    {
        cause = RESET_CAUSE_BROWN_OUT;
    }
    else if ((csr & RCC_CSR_PINRSTF) != 0U)
    {
        cause = RESET_CAUSE_PIN;
    }
    else
    {
        /* No flag set: emulators may not model RCC_CSR. */
    }
    return cause;
}

void reset_info_init(void)
{
    const uint32_t csr = RCC->CSR;
    RCC->CSR = csr | RCC_CSR_RMVF;

    /*
     * The flags must read back as cleared, otherwise they cannot tell this
     * reset apart from earlier ones (Renode, for example, keeps the power-on
     * flags set forever). In that case fall back to software tracking only.
     */
    s_flags_reliable = ((RCC->CSR & RCC_CSR_RESET_FLAGS) == 0U);
    s_cause = s_flags_reliable ? cause_from_rcc(csr) : RESET_CAUSE_UNKNOWN;
    s_had_fault = false;

    if (!record_is_valid())
    {
        /* Cold boot or corrupted record: start from a clean state. */
        s_record.magic = RESET_INFO_MAGIC;
        s_record.reset_count = 0U;
        s_record.sw_reset_marker = 0U;
        s_record.fault_valid = 0U;
        if ((!s_flags_reliable) || ((csr & RCC_CSR_RESET_FLAGS) == 0U))
        {
            s_cause = RESET_CAUSE_POWER_ON;
        }
    }
    else
    {
        s_record.reset_count = s_record.reset_count + 1U;

        if ((s_cause == RESET_CAUSE_UNKNOWN) && (s_record.sw_reset_marker == SW_RESET_MARKER))
        {
            s_cause = RESET_CAUSE_SOFTWARE;
        }
        if (s_record.fault_valid != 0U)
        {
            s_had_fault = true;
            s_last_fault = s_record.fault;
        }
        s_record.sw_reset_marker = 0U;
        s_record.fault_valid = 0U;
    }

    record_seal();
}

reset_cause_t reset_info_cause(void)
{
    return s_cause;
}

const char *reset_info_cause_name(reset_cause_t cause)
{
    const char *name;

    switch (cause)
    {
        case RESET_CAUSE_POWER_ON:  name = "POWER_ON";  break;
        case RESET_CAUSE_PIN:       name = "PIN";       break;
        case RESET_CAUSE_BROWN_OUT: name = "BROWN_OUT"; break;
        case RESET_CAUSE_SOFTWARE:  name = "SOFTWARE";  break;
        case RESET_CAUSE_IWDG:      name = "IWDG";      break;
        case RESET_CAUSE_WWDG:      name = "WWDG";      break;
        case RESET_CAUSE_LOW_POWER: name = "LOW_POWER"; break;
        default:                    name = "UNKNOWN";   break;
    }
    return name;
}

bool reset_info_flags_reliable(void)
{
    return s_flags_reliable;
}

uint32_t reset_info_count(void)
{
    return s_record.reset_count;
}

bool reset_info_last_fault(fault_record_t *out)
{
    if (s_had_fault)
    {
        *out = s_last_fault;
    }
    return s_had_fault;
}

void reset_info_store_fault(const fault_record_t *record)
{
    s_record.fault = *record;
    s_record.fault_valid = 1U;
    record_seal();
}

void reset_info_system_reset(void)
{
    s_record.sw_reset_marker = SW_RESET_MARKER;
    record_seal();

    __asm volatile("dsb" ::: "memory");
    SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    __asm volatile("dsb" ::: "memory");

    for (;;)
    {
        /* Wait for the reset to take effect. */
    }
}
