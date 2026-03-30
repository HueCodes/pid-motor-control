#ifndef HAL_H
#define HAL_H

/* Bare-metal peripheral access for STM32F411RE */
/* These wrap direct register manipulation */

#include <stdint.h>

void     rcc_init(void);        /* PLL to 100 MHz SYSCLK */
void     gpio_init(void);       /* All project GPIOs */
void     timer_init(void);      /* TIM3 PWM (20 kHz), TIM2 for 1 kHz control loop */
void     i2c_init(void);        /* I2C1 at 400 kHz (PB8/PB9) */
void     uart_init(void);       /* USART2 at 115200 (PA2/PA3, ST-Link VCP) */

/* I2C register-level operations */
int      i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
int      i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/* UART non-blocking IO */
void     uart_send_byte(uint8_t c);
void     uart_send_string(const char *s);
void     uart_printf(const char *fmt, ...);
int      uart_read_byte(uint8_t *c);

/* Timing */
uint32_t millis(void);
void     delay_ms(uint32_t ms);

/* PWM duty cycle: 0-4999 maps to 0-100% */
void     timer_set_pwm(uint32_t ccr);

#endif
