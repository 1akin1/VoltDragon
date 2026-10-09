/**
 * @file board.c
 * @brief Board-level clock and pin configuration.
 */
#include "board.h"

#define CONSOLE_TX_PIN  (2U)
#define CONSOLE_RX_PIN  (3U)
#define CONSOLE_ALT_TX_PIN  (5U)
#define CONSOLE_ALT_RX_PIN  (6U)
#define CAN_RX_PIN      (8U)
#define CAN_TX_PIN      (9U)
#define COMMAND_TX_PIN  (10U)
#define COMMAND_RX_PIN  (11U)
#define SENSOR_SCL_PIN  (6U)
#define SENSOR_SDA_PIN  (7U)
#define FLASH_CS_PIN    (4U)
#define FLASH_SCK_PIN   (5U)
#define FLASH_MISO_PIN  (6U)
#define FLASH_MOSI_PIN  (7U)
#define GPIO_AF4_I2C    (4UL)
#define GPIO_AF5_SPI    (5UL)
#define GPIO_AF7_USART  (7UL)
#define GPIO_AF8_UART4  (8UL)
#define GPS_TX_PIN      (10U)
#define GPS_RX_PIN      (11U)
#define AP_TX_PIN       (12U)   /* PC12 */
#define AP_RX_PIN       (2U)    /* PD2 */
#define GPIO_AF9_CAN    (9UL)
#define GPIO_AF11_ETH   (11UL)

static void gpio_set_alternate(gpio_regs_t *port, uint32_t pin, uint32_t af)
{
    const uint32_t mode_shift = pin * 2U;
    const uint32_t af_index = pin / 8U;
    const uint32_t af_shift = (pin % 8U) * 4U;

    port->MODER = (port->MODER & ~(3UL << mode_shift)) | (GPIO_MODE_AF << mode_shift);
    port->AFR[af_index] = (port->AFR[af_index] & ~(0xFUL << af_shift)) | (af << af_shift);
}

static void gpio_set_open_drain_pull_up(gpio_regs_t *port, uint32_t pin)
{
    const uint32_t pupd_shift = pin * 2U;

    port->OTYPER |= (1UL << pin);
    port->PUPDR = (port->PUPDR & ~(3UL << pupd_shift)) | (GPIO_PUPD_PULL_UP << pupd_shift);
}

static void gpio_set_output(gpio_regs_t *port, uint32_t pin)
{
    const uint32_t mode_shift = pin * 2U;

    port->MODER = (port->MODER & ~(3UL << mode_shift)) | (GPIO_MODE_OUTPUT << mode_shift);
}

void board_init(board_console_pins_t console_pins)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    /* Read back so the clock is running before the peripheral is touched. */
    (void)RCC->APB1ENR;

    if (console_pins == BOARD_CONSOLE_PD5_PD6)
    {
        gpio_set_alternate(GPIOD, CONSOLE_ALT_TX_PIN, GPIO_AF7_USART);
        gpio_set_alternate(GPIOD, CONSOLE_ALT_RX_PIN, GPIO_AF7_USART);
    }
    else
    {
        gpio_set_alternate(GPIOA, CONSOLE_TX_PIN, GPIO_AF7_USART);
        gpio_set_alternate(GPIOA, CONSOLE_RX_PIN, GPIO_AF7_USART);
    }
}

void board_gps_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    (void)RCC->APB1ENR;

    gpio_set_alternate(GPIOC, GPS_TX_PIN, GPIO_AF8_UART4);
    gpio_set_alternate(GPIOC, GPS_RX_PIN, GPIO_AF8_UART4);
}

void board_autopilot_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    (void)RCC->APB1ENR;

    gpio_set_alternate(GPIOC, AP_TX_PIN, GPIO_AF8_UART4);   /* AF8 serves UART4/5 and USART6 */
    gpio_set_alternate(GPIOD, AP_RX_PIN, GPIO_AF8_UART4);
}

void board_command_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    gpio_set_alternate(GPIOB, COMMAND_TX_PIN, GPIO_AF7_USART);
    gpio_set_alternate(GPIOB, COMMAND_RX_PIN, GPIO_AF7_USART);
}

void board_can_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    (void)RCC->APB1ENR;

    gpio_set_alternate(GPIOB, CAN_RX_PIN, GPIO_AF9_CAN);
    gpio_set_alternate(GPIOB, CAN_TX_PIN, GPIO_AF9_CAN);
}

typedef struct
{
    gpio_regs_t *port;
    uint32_t     pin;
} pin_t;

void board_eth_init(void)
{
    static const pin_t rmii_pins[] = {
        { GPIOA, 1U },  /* REF_CLK */
        { GPIOA, 2U },  /* MDIO */
        { GPIOA, 7U },  /* CRS_DV */
        { GPIOC, 1U },  /* MDC */
        { GPIOC, 4U },  /* RXD0 */
        { GPIOC, 5U },  /* RXD1 */
        { GPIOG, 11U }, /* TX_EN */
        { GPIOG, 13U }, /* TXD0 */
        { GPIOG, 14U }, /* TXD1 */
    };

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOGEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* The interface type is latched while the MAC is in reset (RM0090 section 33.4.4). */
    RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;
    SYSCFG_PMC |= SYSCFG_PMC_MII_RMII_SEL;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST;

    for (uint32_t i = 0U; i < (sizeof(rmii_pins) / sizeof(rmii_pins[0])); ++i)
    {
        gpio_set_alternate(rmii_pins[i].port, rmii_pins[i].pin, GPIO_AF11_ETH);
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN | RCC_AHB1ENR_ETHMACRXEN;
    (void)RCC->AHB1ENR;
}

void board_sensor_bus_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    /* Open-drain must be set before the pins are switched to the I2C function. */
    gpio_set_open_drain_pull_up(GPIOB, SENSOR_SCL_PIN);
    gpio_set_open_drain_pull_up(GPIOB, SENSOR_SDA_PIN);
    gpio_set_alternate(GPIOB, SENSOR_SCL_PIN, GPIO_AF4_I2C);
    gpio_set_alternate(GPIOB, SENSOR_SDA_PIN, GPIO_AF4_I2C);
}

void board_flash_bus_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    /* Deselect before the pin becomes an output, so the flash never sees a glitch. */
    board_flash_select(false);
    gpio_set_output(GPIOA, FLASH_CS_PIN);
    gpio_set_alternate(GPIOA, FLASH_SCK_PIN, GPIO_AF5_SPI);
    gpio_set_alternate(GPIOA, FLASH_MISO_PIN, GPIO_AF5_SPI);
    gpio_set_alternate(GPIOA, FLASH_MOSI_PIN, GPIO_AF5_SPI);
}

void board_flash_select(bool selected)
{
    /* BSRR: the low half sets a pin, the high half resets it. */
    GPIOA->BSRR = selected ? (1UL << (FLASH_CS_PIN + 16U)) : (1UL << FLASH_CS_PIN);
}
