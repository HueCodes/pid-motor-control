/*
 * Clock configuration: HSE (8 MHz from ST-Link) -> PLL -> 100 MHz SYSCLK
 * APB1 = 50 MHz (timers = 100 MHz), APB2 = 100 MHz
 */

#include "stm32f411xe.h"

void rcc_init(void) {
    /* Enable HSE (from ST-Link MCO output) */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /* Configure PLL: HSE/8 * 200 / 2 = 100 MHz */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
                  | (8  << RCC_PLLCFGR_PLLM_Pos)   /* /8  */
                  | (200 << RCC_PLLCFGR_PLLN_Pos)   /* *200 */
                  | (0  << RCC_PLLCFGR_PLLP_Pos);   /* /2 (PLLP=0 means /2) */

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Flash latency: 3 wait states for 100 MHz at 3.3V */
    FLASH->ACR = FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    /* AHB = /1, APB1 = /2, APB2 = /1 */
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* Enable peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN
                  | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_USART2EN;
}
