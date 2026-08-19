// uart.c - UART 16550 driver
#include "uart.h"
#include <stdint.h>

#define UART_BASE   0x10001000UL

#define UART_RBR    (*(volatile uint8_t *)(UART_BASE + (0 << 2)))  // Receive Buffer
#define UART_THR    (*(volatile uint8_t *)(UART_BASE + (0 << 2)))  // Transmit Holding
#define UART_IER    (*(volatile uint8_t *)(UART_BASE + (1 << 2)))  // Interrupt Enable
#define UART_FCR    (*(volatile uint8_t *)(UART_BASE + (2 << 2)))  // FIFO Control
#define UART_LCR    (*(volatile uint8_t *)(UART_BASE + (3 << 2)))  // Line Control
#define UART_MCR    (*(volatile uint8_t *)(UART_BASE + (4 << 2)))  // Modem Control
#define UART_LSR    (*(volatile uint8_t *)(UART_BASE + (5 << 2)))  // Line Status
#define UART_DLL    (*(volatile uint8_t *)(UART_BASE + (0 << 2)))  // Divisor Latch Low
#define UART_DLM    (*(volatile uint8_t *)(UART_BASE + (1 << 2)))  // Divisor Latch High

#define UART_LSR_DR     0x01  // Data Ready
#define UART_LSR_THRE   0x20  // THR Empty

void uart_init(void) {
    // Already initialized by testbench, but set up anyway
    UART_IER = 0x00;        // Disable interrupts
    UART_LCR = 0x80;        // Enable DLAB
    UART_DLL = 0x01;        // Divisor low
    UART_DLM = 0x00;        // Divisor high
    UART_LCR = 0x03;        // 8N1, disable DLAB
    UART_FCR = 0x07;        // Enable FIFO, clear
    UART_MCR = 0x00;        // No modem control
}

void uart_putc(char c) {
    while ((UART_LSR & UART_LSR_THRE) == 0);
    UART_THR = c;
}

int uart_getc(void) {
    if (UART_LSR & UART_LSR_DR) {
        return UART_RBR;
    }
    return -1;
}

void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}
