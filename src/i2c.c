/*
 * I2C1 bare-metal driver at 400 kHz (Fast Mode)
 * PB8 = SCL, PB9 = SDA
 * External 4.7K pull-ups required on both lines to 3.3V
 */

#include "stm32f411xe.h"
#include "hal.h"

#define I2C_TIMEOUT 10000

void i2c_init(void) {
    /* Reset I2C1 */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 = 0;
    I2C1->CR2 = 50;  /* APB1 peripheral clock = 50 MHz */

    /* Fast mode 400 kHz: CCR = 50 MHz / (25 * 400 kHz) = 5 (duty cycle 16/9) */
    I2C1->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 5;

    /* Max rise time: 300 ns * 50 MHz + 1 = 16 */
    I2C1->TRISE = 16;

    I2C1->CR1 |= I2C_CR1_PE;
}

static int i2c_wait_flag(volatile uint32_t *reg, uint32_t flag) {
    uint32_t timeout = I2C_TIMEOUT;
    while (!(*reg & flag)) {
        if (--timeout == 0) return -1;
    }
    return 0;
}

static int i2c_start(uint8_t addr, uint8_t rw) {
    I2C1->CR1 |= I2C_CR1_START;
    if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB) < 0) return -1;

    I2C1->DR = (addr << 1) | rw;
    if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR) < 0) return -1;

    /* Clear ADDR by reading SR1 then SR2 */
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    return 0;
}

int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    /* Wait for bus free */
    uint32_t timeout = I2C_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && --timeout);
    if (timeout == 0) return -1;

    if (i2c_start(addr, 0) < 0) return -1;

    I2C1->DR = reg;
    if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE) < 0) return -1;

    I2C1->DR = val;
    if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF) < 0) return -1;

    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    uint32_t timeout = I2C_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && --timeout);
    if (timeout == 0) return -1;

    /* Write register address */
    if (i2c_start(addr, 0) < 0) return -1;

    I2C1->DR = reg;
    if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF) < 0) return -1;

    /* Repeated start for read */
    if (len == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_START;
        if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB) < 0) return -1;

        I2C1->DR = (addr << 1) | 1;
        if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR) < 0) return -1;
        (void)I2C1->SR1;
        (void)I2C1->SR2;

        I2C1->CR1 |= I2C_CR1_STOP;
        if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE) < 0) return -1;
        buf[0] = I2C1->DR;
    } else {
        I2C1->CR1 |= I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_START;
        if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB) < 0) return -1;

        I2C1->DR = (addr << 1) | 1;
        if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR) < 0) return -1;
        (void)I2C1->SR1;
        (void)I2C1->SR2;

        for (uint16_t i = 0; i < len; i++) {
            if (i == len - 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                I2C1->CR1 |= I2C_CR1_STOP;
            }
            if (i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE) < 0) return -1;
            buf[i] = I2C1->DR;
        }
    }

    return 0;
}
