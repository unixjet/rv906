// uart.h - UART driver for 16550
#ifndef UART_H
#define UART_H

void uart_init(void);
void uart_putc(char c);
int  uart_getc(void);
void uart_puts(const char *s);

#endif
