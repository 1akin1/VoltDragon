/**
 * @file stm32f4_regs.h
 * @brief Minimal, hand-written register map for the STM32F407 and Cortex-M4 core.
 *
 * Only the peripherals used by VoltDragon are described. Offsets follow
 * RM0090 (STM32F4 reference manual) and the ARMv7-M Architecture Reference
 * Manual. Layouts are checked at compile time with static assertions.
 *
 * MISRA C:2012 deviation D-002 (Rule 11.4): integer-to-pointer casts are used
 * to map peripheral base addresses. See docs/misra-deviations.md.
 */
#ifndef STM32F4_REGS_H
#define STM32F4_REGS_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Cortex-M4 core peripherals                                                */
/* ------------------------------------------------------------------------- */

/** System Control Block (0xE000ED00). */
typedef struct
{
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR[3];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    volatile uint32_t DFSR;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
} scb_regs_t;

_Static_assert(offsetof(scb_regs_t, CFSR) == 0x28U, "SCB layout");
_Static_assert(offsetof(scb_regs_t, BFAR) == 0x38U, "SCB layout");

/** SysTick timer (0xE000E010). */
typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} systick_regs_t;

#define SCB         ((scb_regs_t *)0xE000ED00UL)
#define SYSTICK     ((systick_regs_t *)0xE000E010UL)
#define SCB_CPACR   (*(volatile uint32_t *)0xE000ED88UL)
/** NVIC interrupt set-enable registers, one bit per IRQ number. */
#define NVIC_ISER   ((volatile uint32_t *)0xE000E100UL)
#define CORE_DHCSR  (*(volatile uint32_t *)0xE000EDF0UL)

#define SCB_AIRCR_VECTKEY       (0x05FAUL << 16)
#define SCB_AIRCR_SYSRESETREQ   (1UL << 2)
#define SCB_CCR_DIV_0_TRP       (1UL << 4)
#define SCB_CPACR_CP10_CP11_FULL (0xFUL << 20)

#define SYSTICK_CTRL_ENABLE     (1UL << 0)
#define SYSTICK_CTRL_TICKINT    (1UL << 1)
#define SYSTICK_CTRL_CLKSOURCE  (1UL << 2)

#define CORE_DHCSR_C_DEBUGEN    (1UL << 0)

/* ------------------------------------------------------------------------- */
/* STM32F4 peripherals                                                       */
/* ------------------------------------------------------------------------- */

/** Reset and Clock Control (0x40023800). */
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t          RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t          RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t          RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t          RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint32_t          RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t          RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} rcc_regs_t;

_Static_assert(offsetof(rcc_regs_t, AHB1ENR) == 0x30U, "RCC layout");
_Static_assert(offsetof(rcc_regs_t, APB1ENR) == 0x40U, "RCC layout");
_Static_assert(offsetof(rcc_regs_t, CSR) == 0x74U, "RCC layout");

#define RCC_AHB1ENR_GPIOAEN     (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN     (1UL << 1)
#define RCC_APB1ENR_USART2EN    (1UL << 17)
#define RCC_APB1ENR_USART3EN    (1UL << 18)
#define RCC_APB1ENR_I2C1EN      (1UL << 21)
#define RCC_APB1RSTR_I2C1RST    (1UL << 21)
#define RCC_APB2ENR_SPI1EN      (1UL << 12)

#define RCC_CSR_RMVF            (1UL << 24)
#define RCC_CSR_BORRSTF         (1UL << 25)
#define RCC_CSR_PINRSTF         (1UL << 26)
#define RCC_CSR_PORRSTF         (1UL << 27)
#define RCC_CSR_SFTRSTF         (1UL << 28)
#define RCC_CSR_IWDGRSTF        (1UL << 29)
#define RCC_CSR_WWDGRSTF        (1UL << 30)
#define RCC_CSR_LPWRRSTF        (1UL << 31)

/** General-purpose I/O port. */
typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} gpio_regs_t;

_Static_assert(offsetof(gpio_regs_t, AFR) == 0x20U, "GPIO layout");

#define GPIO_MODE_OUTPUT        (1UL)
#define GPIO_MODE_AF            (2UL)
#define GPIO_PUPD_PULL_UP       (1UL)

/** USART (STM32F4 variant with SR/DR registers). */
typedef struct
{
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} usart_regs_t;

_Static_assert(offsetof(usart_regs_t, CR1) == 0x0CU, "USART layout");

#define USART_SR_ORE            (1UL << 3)
#define USART_SR_RXNE           (1UL << 5)
#define USART_SR_TC             (1UL << 6)
#define USART_SR_TXE            (1UL << 7)
#define USART_CR1_RE            (1UL << 2)
#define USART_CR1_RXNEIE        (1UL << 5)
#define USART_CR1_TE            (1UL << 3)
#define USART_CR1_UE            (1UL << 13)

/** I2C (STM32F4 variant with SR1/SR2 registers). */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} i2c_regs_t;

_Static_assert(offsetof(i2c_regs_t, SR1) == 0x14U, "I2C layout");
_Static_assert(offsetof(i2c_regs_t, FLTR) == 0x24U, "I2C layout");

#define I2C_CR1_PE              (1UL << 0)
#define I2C_CR1_START           (1UL << 8)
#define I2C_CR1_STOP            (1UL << 9)
#define I2C_CR1_ACK             (1UL << 10)
#define I2C_CR1_POS             (1UL << 11)
#define I2C_CR1_SWRST           (1UL << 15)
#define I2C_SR1_SB              (1UL << 0)
#define I2C_SR1_ADDR            (1UL << 1)
#define I2C_SR1_BTF             (1UL << 2)
#define I2C_SR1_RXNE            (1UL << 6)
#define I2C_SR1_TXE             (1UL << 7)
#define I2C_SR1_BERR            (1UL << 8)
#define I2C_SR1_ARLO            (1UL << 9)
#define I2C_SR1_AF              (1UL << 10)
#define I2C_SR2_BUSY            (1UL << 1)

/** SPI (I2S registers not used). */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} spi_regs_t;

_Static_assert(offsetof(spi_regs_t, DR) == 0x0CU, "SPI layout");
_Static_assert(offsetof(spi_regs_t, I2SPR) == 0x20U, "SPI layout");

#define SPI_CR1_CPHA            (1UL << 0)
#define SPI_CR1_CPOL            (1UL << 1)
#define SPI_CR1_MSTR            (1UL << 2)
#define SPI_CR1_BR_SHIFT        (3U)
#define SPI_CR1_BR_MASK         (7UL << SPI_CR1_BR_SHIFT)
#define SPI_CR1_SPE             (1UL << 6)
#define SPI_CR1_SSI             (1UL << 8)
#define SPI_CR1_SSM             (1UL << 9)
#define SPI_SR_RXNE             (1UL << 0)
#define SPI_SR_TXE              (1UL << 1)
#define SPI_SR_OVR              (1UL << 6)
#define SPI_SR_BSY              (1UL << 7)

/** Independent watchdog (0x40003000). */
typedef struct
{
    volatile uint32_t KR;
    volatile uint32_t PR;
    volatile uint32_t RLR;
    volatile uint32_t SR;
} iwdg_regs_t;

#define IWDG_KEY_RELOAD         (0xAAAAUL)
#define IWDG_KEY_UNLOCK         (0x5555UL)
#define IWDG_KEY_START          (0xCCCCUL)

#define RCC     ((rcc_regs_t *)0x40023800UL)
#define GPIOA   ((gpio_regs_t *)0x40020000UL)
#define GPIOB   ((gpio_regs_t *)0x40020400UL)
#define USART2  ((usart_regs_t *)0x40004400UL)
#define USART3  ((usart_regs_t *)0x40004800UL)

/* Interrupt numbers (RM0090 table 61). */
#define USART3_IRQN (39U)
#define I2C1    ((i2c_regs_t *)0x40005400UL)
#define SPI1    ((spi_regs_t *)0x40013000UL)
#define IWDG    ((iwdg_regs_t *)0x40003000UL)

#endif /* STM32F4_REGS_H */
