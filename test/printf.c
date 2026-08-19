// printf.c - Simple printf implementation (no libc dependency)
#include <stdarg.h>
#include <stdint.h>

// UART registers
#define UART_BASE   0x10001000UL
#define UART_THR    (*(volatile uint8_t *)(UART_BASE + (0 << 2)))
#define UART_LSR    (*(volatile uint8_t *)(UART_BASE + (5 << 2)))
#define UART_LSR_THRE 0x20

static void uart_putc(char c) {
    while ((UART_LSR & UART_LSR_THRE) == 0);
    UART_THR = c;
}

static void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

static void print_int(int val, int base, int width, char pad, int is_signed) {
    char buf[32];
    int i = 0;
    int neg = 0;
    unsigned int uval;

    if (is_signed && val < 0) {
        neg = 1;
        uval = -val;
    } else {
        uval = val;
    }

    if (uval == 0) {
        buf[i++] = '0';
    } else {
        while (uval > 0) {
            int digit = uval % base;
            buf[i++] = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
            uval /= base;
        }
    }

    if (neg) buf[i++] = '-';

    // Padding
    while (i < width) buf[i++] = pad;

    // Print in reverse
    while (i > 0) uart_putc(buf[--i]);
}

static void print_long(long val, int base, int width, char pad, int is_signed) {
    char buf[32];
    int i = 0;
    int neg = 0;
    unsigned long uval;

    if (is_signed && val < 0) {
        neg = 1;
        uval = -val;
    } else {
        uval = val;
    }

    if (uval == 0) {
        buf[i++] = '0';
    } else {
        while (uval > 0) {
            int digit = uval % base;
            buf[i++] = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
            uval /= base;
        }
    }

    if (neg) buf[i++] = '-';

    while (i < width) buf[i++] = pad;
    while (i > 0) uart_putc(buf[--i]);
}

int printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    int count = 0;
    char c;

    while ((c = *fmt++) != 0) {
        if (c != '%') {
            if (c == '\n') uart_putc('\r');
            uart_putc(c);
            count++;
            continue;
        }

        // Parse format specifier
        char pad = ' ';
        int width = 0;
        int is_long = 0;

        c = *fmt++;
        if (c == '0') {
            pad = '0';
            c = *fmt++;
        }

        while (c >= '0' && c <= '9') {
            width = width * 10 + (c - '0');
            c = *fmt++;
        }

        if (c == 'l') {
            is_long = 1;
            c = *fmt++;
        }

        switch (c) {
            case 'd':
            case 'i':
                if (is_long)
                    print_long(va_arg(ap, long), 10, width, pad, 1);
                else
                    print_int(va_arg(ap, int), 10, width, pad, 1);
                break;
            case 'u':
                if (is_long)
                    print_long(va_arg(ap, unsigned long), 10, width, pad, 0);
                else
                    print_int(va_arg(ap, unsigned int), 10, width, pad, 0);
                break;
            case 'x':
            case 'X':
                if (is_long)
                    print_long(va_arg(ap, unsigned long), 16, width, pad, 0);
                else
                    print_int(va_arg(ap, unsigned int), 16, width, pad, 0);
                break;
            case 'p':
                uart_puts("0x");
                print_long((unsigned long)va_arg(ap, void*), 16, sizeof(void*)*2, '0', 0);
                break;
            case 's': {
                const char *s = va_arg(ap, const char*);
                if (s == 0) s = "(null)";
                uart_puts(s);
                break;
            }
            case 'c':
                uart_putc((char)va_arg(ap, int));
                break;
            case '%':
                uart_putc('%');
                break;
            default:
                uart_putc('%');
                uart_putc(c);
                break;
        }
    }

    va_end(ap);
    return count;
}

int puts(const char *s) {
    uart_puts(s);
    uart_putc('\r');
    uart_putc('\n');
    return 0;
}

int putchar(int c) {
    if (c == '\n') uart_putc('\r');
    uart_putc(c);
    return c;
}
