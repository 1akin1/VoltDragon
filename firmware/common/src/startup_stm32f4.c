/**
 * @file startup_stm32f4.c
 * @brief Vector table and reset handler for the STM32F407, written in C.
 *
 * Reset sequence:
 *   1. Enable the FPU (code is built with -mfloat-abi=hard).
 *   2. Copy .data from flash to RAM and clear .bss. .noinit is left untouched.
 *   3. Configure the core (VTOR, fault traps) in system_init().
 *   4. Call main().
 *
 * MISRA C:2012 deviation D-003 (Rule 18.3/18.2): linker-provided section
 * boundaries are compared and subtracted although they are distinct objects.
 */
#include <stdint.h>

#include "stm32f4_regs.h"

/* Symbols defined by the linker script. */
extern uint32_t _estack[];
extern uint32_t _sidata[];
extern uint32_t _sdata[];
extern uint32_t _edata[];
extern uint32_t _sbss[];
extern uint32_t _ebss[];

extern int main(void);

typedef void (*isr_t)(void);

/** Number of device-specific interrupts on the STM32F407 (IRQ0..IRQ81). */
#define DEVICE_IRQ_COUNT (82U)
/** Number of Cortex-M system exception slots after the initial SP. */
#define SYSTEM_VECTOR_COUNT (15U)

typedef struct
{
    uint32_t *initial_sp;
    isr_t     handlers[SYSTEM_VECTOR_COUNT + DEVICE_IRQ_COUNT];
} vector_table_t;

void Reset_Handler(void);
void Default_Handler(void);

/* Fault exceptions are implemented in fault.c. */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

/* Everything else defaults to Default_Handler and may be overridden. */
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

void SVC_Handler(void) WEAK_ALIAS;
void DebugMon_Handler(void) WEAK_ALIAS;
void PendSV_Handler(void) WEAK_ALIAS;
void SysTick_Handler(void) WEAK_ALIAS;

void WWDG_IRQHandler(void) WEAK_ALIAS;
void PVD_IRQHandler(void) WEAK_ALIAS;
void TAMP_STAMP_IRQHandler(void) WEAK_ALIAS;
void RTC_WKUP_IRQHandler(void) WEAK_ALIAS;
void FLASH_IRQHandler(void) WEAK_ALIAS;
void RCC_IRQHandler(void) WEAK_ALIAS;
void EXTI0_IRQHandler(void) WEAK_ALIAS;
void EXTI1_IRQHandler(void) WEAK_ALIAS;
void EXTI2_IRQHandler(void) WEAK_ALIAS;
void EXTI3_IRQHandler(void) WEAK_ALIAS;
void EXTI4_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream0_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream1_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream2_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream3_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream4_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream5_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream6_IRQHandler(void) WEAK_ALIAS;
void ADC_IRQHandler(void) WEAK_ALIAS;
void CAN1_TX_IRQHandler(void) WEAK_ALIAS;
void CAN1_RX0_IRQHandler(void) WEAK_ALIAS;
void CAN1_RX1_IRQHandler(void) WEAK_ALIAS;
void CAN1_SCE_IRQHandler(void) WEAK_ALIAS;
void EXTI9_5_IRQHandler(void) WEAK_ALIAS;
void TIM1_BRK_TIM9_IRQHandler(void) WEAK_ALIAS;
void TIM1_UP_TIM10_IRQHandler(void) WEAK_ALIAS;
void TIM1_TRG_COM_TIM11_IRQHandler(void) WEAK_ALIAS;
void TIM1_CC_IRQHandler(void) WEAK_ALIAS;
void TIM2_IRQHandler(void) WEAK_ALIAS;
void TIM3_IRQHandler(void) WEAK_ALIAS;
void TIM4_IRQHandler(void) WEAK_ALIAS;
void I2C1_EV_IRQHandler(void) WEAK_ALIAS;
void I2C1_ER_IRQHandler(void) WEAK_ALIAS;
void I2C2_EV_IRQHandler(void) WEAK_ALIAS;
void I2C2_ER_IRQHandler(void) WEAK_ALIAS;
void SPI1_IRQHandler(void) WEAK_ALIAS;
void SPI2_IRQHandler(void) WEAK_ALIAS;
void USART1_IRQHandler(void) WEAK_ALIAS;
void USART2_IRQHandler(void) WEAK_ALIAS;
void USART3_IRQHandler(void) WEAK_ALIAS;
void EXTI15_10_IRQHandler(void) WEAK_ALIAS;
void RTC_Alarm_IRQHandler(void) WEAK_ALIAS;
void OTG_FS_WKUP_IRQHandler(void) WEAK_ALIAS;
void TIM8_BRK_TIM12_IRQHandler(void) WEAK_ALIAS;
void TIM8_UP_TIM13_IRQHandler(void) WEAK_ALIAS;
void TIM8_TRG_COM_TIM14_IRQHandler(void) WEAK_ALIAS;
void TIM8_CC_IRQHandler(void) WEAK_ALIAS;
void DMA1_Stream7_IRQHandler(void) WEAK_ALIAS;
void FSMC_IRQHandler(void) WEAK_ALIAS;
void SDIO_IRQHandler(void) WEAK_ALIAS;
void TIM5_IRQHandler(void) WEAK_ALIAS;
void SPI3_IRQHandler(void) WEAK_ALIAS;
void UART4_IRQHandler(void) WEAK_ALIAS;
void UART5_IRQHandler(void) WEAK_ALIAS;
void TIM6_DAC_IRQHandler(void) WEAK_ALIAS;
void TIM7_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream0_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream1_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream2_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream3_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream4_IRQHandler(void) WEAK_ALIAS;
void ETH_IRQHandler(void) WEAK_ALIAS;
void ETH_WKUP_IRQHandler(void) WEAK_ALIAS;
void CAN2_TX_IRQHandler(void) WEAK_ALIAS;
void CAN2_RX0_IRQHandler(void) WEAK_ALIAS;
void CAN2_RX1_IRQHandler(void) WEAK_ALIAS;
void CAN2_SCE_IRQHandler(void) WEAK_ALIAS;
void OTG_FS_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream5_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream6_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream7_IRQHandler(void) WEAK_ALIAS;
void USART6_IRQHandler(void) WEAK_ALIAS;
void I2C3_EV_IRQHandler(void) WEAK_ALIAS;
void I2C3_ER_IRQHandler(void) WEAK_ALIAS;
void OTG_HS_EP1_OUT_IRQHandler(void) WEAK_ALIAS;
void OTG_HS_EP1_IN_IRQHandler(void) WEAK_ALIAS;
void OTG_HS_WKUP_IRQHandler(void) WEAK_ALIAS;
void OTG_HS_IRQHandler(void) WEAK_ALIAS;
void DCMI_IRQHandler(void) WEAK_ALIAS;
void CRYP_IRQHandler(void) WEAK_ALIAS;
void HASH_RNG_IRQHandler(void) WEAK_ALIAS;
void FPU_IRQHandler(void) WEAK_ALIAS;

__attribute__((section(".isr_vector"), used))
const vector_table_t g_vector_table = {
    .initial_sp = _estack,
    .handlers = {
        /* Cortex-M system exceptions */
        Reset_Handler,
        NMI_Handler,
        HardFault_Handler,
        MemManage_Handler,
        BusFault_Handler,
        UsageFault_Handler,
        0, 0, 0, 0,                     /* Reserved */
        SVC_Handler,
        DebugMon_Handler,
        0,                              /* Reserved */
        PendSV_Handler,
        SysTick_Handler,

        /* STM32F407 device interrupts (IRQ0..IRQ81) */
        WWDG_IRQHandler,                /*  0 */
        PVD_IRQHandler,
        TAMP_STAMP_IRQHandler,
        RTC_WKUP_IRQHandler,
        FLASH_IRQHandler,
        RCC_IRQHandler,
        EXTI0_IRQHandler,
        EXTI1_IRQHandler,
        EXTI2_IRQHandler,
        EXTI3_IRQHandler,
        EXTI4_IRQHandler,               /* 10 */
        DMA1_Stream0_IRQHandler,
        DMA1_Stream1_IRQHandler,
        DMA1_Stream2_IRQHandler,
        DMA1_Stream3_IRQHandler,
        DMA1_Stream4_IRQHandler,
        DMA1_Stream5_IRQHandler,
        DMA1_Stream6_IRQHandler,
        ADC_IRQHandler,
        CAN1_TX_IRQHandler,
        CAN1_RX0_IRQHandler,            /* 20 */
        CAN1_RX1_IRQHandler,
        CAN1_SCE_IRQHandler,
        EXTI9_5_IRQHandler,
        TIM1_BRK_TIM9_IRQHandler,
        TIM1_UP_TIM10_IRQHandler,
        TIM1_TRG_COM_TIM11_IRQHandler,
        TIM1_CC_IRQHandler,
        TIM2_IRQHandler,
        TIM3_IRQHandler,
        TIM4_IRQHandler,                /* 30 */
        I2C1_EV_IRQHandler,
        I2C1_ER_IRQHandler,
        I2C2_EV_IRQHandler,
        I2C2_ER_IRQHandler,
        SPI1_IRQHandler,
        SPI2_IRQHandler,
        USART1_IRQHandler,
        USART2_IRQHandler,
        USART3_IRQHandler,
        EXTI15_10_IRQHandler,           /* 40 */
        RTC_Alarm_IRQHandler,
        OTG_FS_WKUP_IRQHandler,
        TIM8_BRK_TIM12_IRQHandler,
        TIM8_UP_TIM13_IRQHandler,
        TIM8_TRG_COM_TIM14_IRQHandler,
        TIM8_CC_IRQHandler,
        DMA1_Stream7_IRQHandler,
        FSMC_IRQHandler,
        SDIO_IRQHandler,
        TIM5_IRQHandler,                /* 50 */
        SPI3_IRQHandler,
        UART4_IRQHandler,
        UART5_IRQHandler,
        TIM6_DAC_IRQHandler,
        TIM7_IRQHandler,
        DMA2_Stream0_IRQHandler,
        DMA2_Stream1_IRQHandler,
        DMA2_Stream2_IRQHandler,
        DMA2_Stream3_IRQHandler,
        DMA2_Stream4_IRQHandler,        /* 60 */
        ETH_IRQHandler,
        ETH_WKUP_IRQHandler,
        CAN2_TX_IRQHandler,
        CAN2_RX0_IRQHandler,
        CAN2_RX1_IRQHandler,
        CAN2_SCE_IRQHandler,
        OTG_FS_IRQHandler,
        DMA2_Stream5_IRQHandler,
        DMA2_Stream6_IRQHandler,
        DMA2_Stream7_IRQHandler,        /* 70 */
        USART6_IRQHandler,
        I2C3_EV_IRQHandler,
        I2C3_ER_IRQHandler,
        OTG_HS_EP1_OUT_IRQHandler,
        OTG_HS_EP1_IN_IRQHandler,
        OTG_HS_WKUP_IRQHandler,
        OTG_HS_IRQHandler,
        DCMI_IRQHandler,
        CRYP_IRQHandler,
        HASH_RNG_IRQHandler,            /* 80 */
        FPU_IRQHandler,
    },
};

/**
 * Core configuration performed before main(): vector table location and
 * trapping of integer division by zero (reported as a UsageFault).
 */
static void system_init(void)
{
    SCB->VTOR = (uint32_t)&g_vector_table;
    SCB->CCR |= SCB_CCR_DIV_0_TRP;
    __asm volatile("dsb\n\tisb" ::: "memory");
}

void Reset_Handler(void)
{
    /* The FPU must be enabled before the compiler emits any VFP instruction. */
    SCB_CPACR |= SCB_CPACR_CP10_CP11_FULL;
    __asm volatile("dsb\n\tisb" ::: "memory");

    const uint32_t *src = _sidata;
    for (uint32_t *dst = _sdata; dst < _edata; ++dst)
    {
        *dst = *src;
        ++src;
    }

    for (uint32_t *dst = _sbss; dst < _ebss; ++dst)
    {
        *dst = 0U;
    }

    system_init();

    (void)main();

    /* main() must never return; park the core so the watchdog resets it. */
    for (;;)
    {
    }
}

/**
 * Catches any interrupt without a dedicated handler. It is a programming
 * error, so it is treated like a fault: the core spins until the watchdog
 * resets it and the reset cause is reported on the next boot.
 */
void Default_Handler(void)
{
    for (;;)
    {
    }
}
