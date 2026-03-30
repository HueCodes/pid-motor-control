/*
 * TIM3_CH1: 20 kHz PWM for motor driver (PA6)
 * TIM2: 1 kHz interrupt for PID control loop
 * SysTick: 1 ms tick for millis()/delay_ms()
 */

#include "stm32f411xe.h"
#include "hal.h"

static volatile uint32_t tick_ms;

void timer_init(void) {
    /* SysTick: 1 ms tick (100 MHz / 100000 = 1 kHz) */
    SysTick_Config(100000);

    /* TIM3: 20 kHz PWM, channel 1
     * Timer clock = 100 MHz (APB1 timers get 2x when prescaler != 1)
     * 100 MHz / 1 / 5000 = 20 kHz */
    TIM3->PSC  = 0;
    TIM3->ARR  = 4999;
    TIM3->CCR1 = 0;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos)  /* PWM mode 1 */
                 | TIM_CCMR1_OC1PE;              /* Preload enable */
    TIM3->CCER  = TIM_CCER_CC1E;                 /* Enable CH1 output */
    TIM3->CR1   = TIM_CR1_ARPE;                  /* Auto-reload preload */
    TIM3->EGR   = TIM_EGR_UG;                    /* Load preload regs */
    TIM3->CR1  |= TIM_CR1_CEN;                   /* Start */

    /* TIM2: 1 kHz control loop interrupt
     * 100 MHz / 100 / 1000 = 1 kHz */
    TIM2->PSC  = 99;       /* /100 -> 1 MHz tick */
    TIM2->ARR  = 999;      /* /1000 -> 1 kHz */
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1  = TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void timer_set_pwm(uint32_t ccr) {
    if (ccr > 4999) ccr = 4999;
    TIM3->CCR1 = ccr;
}

void SysTick_Handler(void) {
    tick_ms++;
}

uint32_t millis(void) {
    return tick_ms;
}

void delay_ms(uint32_t ms) {
    uint32_t start = tick_ms;
    while ((tick_ms - start) < ms);
}
