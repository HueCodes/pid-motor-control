/*
 * USART2 at 115200 baud (PA2 TX, PA3 RX)
 * Connected to ST-Link VCP -- no extra USB cable needed
 * Interrupt-driven RX with ring buffer for CLI input
 */

#include "stm32f411xe.h"
#include "hal.h"
#include <stdarg.h>
#include <stdio.h>

#define RX_BUF_SIZE 256

static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head;
static volatile uint32_t rx_tail;

void uart_init(void) {
    /* 115200 baud from 50 MHz APB1 clock
     * BRR = 50000000 / 115200 = 434.03 -> mantissa=27, fraction=2
     * Or just use the integer: 50000000/115200 ~= 434 (0x1B2) */
    USART2->BRR = 0x1B2;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART2_IRQn, 3);
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t c = USART2->DR;
        uint32_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}

void uart_send_byte(uint8_t c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void uart_send_string(const char *s) {
    while (*s) {
        if (*s == '\n') uart_send_byte('\r');
        uart_send_byte(*s++);
    }
}

int uart_read_byte(uint8_t *c) {
    if (rx_head == rx_tail) return 0;
    *c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

void uart_printf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_send_string(buf);
}
