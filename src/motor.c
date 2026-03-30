/*
 * TB6612FNG motor driver control
 * Sign-magnitude drive: direction pins set polarity, PWM controls speed
 *
 * Wiring:
 *   PA6 (TIM3_CH1) -> PWMA
 *   PA5             -> AIN1
 *   PA8             -> AIN2
 *   PA9             -> STBY (held high in gpio_init)
 */

#include "motor.h"
#include "hal.h"
#include "stm32f411xe.h"

#define PWM_MAX 4999

/* AIN1 = PA5, AIN2 = PA8 */
#define AIN1_SET()   (GPIOA->BSRR = (1U << 5))
#define AIN1_CLR()   (GPIOA->BSRR = (1U << (5 + 16)))
#define AIN2_SET()   (GPIOA->BSRR = (1U << 8))
#define AIN2_CLR()   (GPIOA->BSRR = (1U << (8 + 16)))

void motor_init(void) {
    motor_brake();
}

void motor_set(float duty, motor_dir_t dir) {
    if (duty < 0) duty = 0;
    if (duty > 1.0f) duty = 1.0f;

    switch (dir) {
    case MOTOR_DIR_FORWARD:
        AIN1_SET(); AIN2_CLR();
        break;
    case MOTOR_DIR_REVERSE:
        AIN1_CLR(); AIN2_SET();
        break;
    case MOTOR_DIR_BRAKE:
        AIN1_SET(); AIN2_SET();
        break;
    case MOTOR_DIR_COAST:
        AIN1_CLR(); AIN2_CLR();
        break;
    }

    timer_set_pwm((uint32_t)(duty * PWM_MAX));
}

void motor_set_effort(float effort) {
    if (effort > 0.01f) {
        motor_set(effort, MOTOR_DIR_FORWARD);
    } else if (effort < -0.01f) {
        motor_set(-effort, MOTOR_DIR_REVERSE);
    } else {
        motor_brake();
    }
}

void motor_brake(void) {
    AIN1_SET();
    AIN2_SET();
    timer_set_pwm(PWM_MAX);
}

void motor_coast(void) {
    AIN1_CLR();
    AIN2_CLR();
    timer_set_pwm(0);
}
