/**
 * @file board.c
 * @brief Board-level clock and pin configuration.
 */
#include "board.h"

#define CONSOLE_TX_PIN  (2U)
#define CONSOLE_RX_PIN  (3U)
#define SENSOR_SCL_PIN  (6U)
#define SENSOR_SDA_PIN  (7U)
#define GPIO_AF4_I2C    (4UL)
#define GPIO_AF7_USART  (7UL)

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

void board_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    /* Read back so the clock is running before the peripheral is touched. */
    (void)RCC->APB1ENR;

    gpio_set_alternate(GPIOA, CONSOLE_TX_PIN, GPIO_AF7_USART);
    gpio_set_alternate(GPIOA, CONSOLE_RX_PIN, GPIO_AF7_USART);
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
