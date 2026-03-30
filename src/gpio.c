/*
 * GPIO pin assignments:
 *   PA2  - USART2_TX (AF7) -> ST-Link VCP
 *   PA3  - USART2_RX (AF7)
 *   PA5  - AIN1 (GPIO output, motor direction)
 *   PA6  - TIM3_CH1 (AF2) -> TB6612FNG PWMA
 *   PA8  - AIN2 (GPIO output, motor direction)
 *   PA9  - STBY (GPIO output, motor enable)
 *   PB8  - I2C1_SCL (AF4, open-drain)
 *   PB9  - I2C1_SDA (AF4, open-drain)
 */

#include "stm32f411xe.h"

static void gpio_af(GPIO_TypeDef *port, uint8_t pin, uint8_t af) {
    port->MODER &= ~(3U << (pin * 2));
    port->MODER |=  (2U << (pin * 2));  /* AF mode */
    if (pin < 8)
        port->AFR[0] = (port->AFR[0] & ~(0xFU << (pin * 4))) | ((uint32_t)af << (pin * 4));
    else
        port->AFR[1] = (port->AFR[1] & ~(0xFU << ((pin - 8) * 4))) | ((uint32_t)af << ((pin - 8) * 4));
}

static void gpio_output(GPIO_TypeDef *port, uint8_t pin) {
    port->MODER &= ~(3U << (pin * 2));
    port->MODER |=  (1U << (pin * 2));  /* Output mode */
    port->OSPEEDR |= (2U << (pin * 2)); /* High speed */
}

void gpio_init(void) {
    /* USART2: PA2 TX, PA3 RX */
    gpio_af(GPIOA, 2, 7);
    gpio_af(GPIOA, 3, 7);

    /* TIM3_CH1 PWM: PA6 */
    gpio_af(GPIOA, 6, 2);
    GPIOA->OSPEEDR |= (2U << (6 * 2));

    /* Motor direction: PA5 = AIN1, PA8 = AIN2 */
    gpio_output(GPIOA, 5);
    gpio_output(GPIOA, 8);

    /* Motor standby: PA9 = STBY (pull high to enable) */
    gpio_output(GPIOA, 9);
    GPIOA->BSRR = (1U << 9); /* STBY high = active */

    /* I2C1: PB8 SCL, PB9 SDA (open-drain with external 4.7K pull-ups) */
    gpio_af(GPIOB, 8, 4);
    gpio_af(GPIOB, 9, 4);
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    GPIOB->OSPEEDR |= (3U << (8 * 2)) | (3U << (9 * 2));
}
