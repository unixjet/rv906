# Makefile for rocketM tests
#
# entry.S modes:
#   Default:     Simple mode (no libc, no trap)
#   -DWITH_TRAP: Add trap handler support
#   -DWITH_LIBC: Use libc (printf, argc/argv)

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC = $(CROSS_COMPILE)gcc

CFLAGS = -march=rv64imac_zicsr -mabi=lp64 -mcmodel=medany -O2 -g
CFLAGS += -fno-builtin -ffreestanding -nostdlib -static
LDFLAGS = -T link.ld -nostartfiles -nostdlib

all: clint_test.out plic_test.out intr_test.out hello.out

clint_test.out: entry.S clint_test.c uart.c syscalls.c link.ld
	$(CC) $(CFLAGS) -DWITH_TRAP -DWITH_LIBC $(LDFLAGS) -o $@ entry.S clint_test.c uart.c syscalls.c -lc -lgcc

plic_test.out: entry.S plic_test.c uart.c syscalls.c link.ld
	$(CC) $(CFLAGS) -DWITH_TRAP -DWITH_LIBC $(LDFLAGS) -o $@ entry.S plic_test.c uart.c syscalls.c -lc -lgcc

intr_test.out: entry.S intr_test.c uart.c syscalls.c link.ld
	$(CC) $(CFLAGS) -DWITH_TRAP -DWITH_LIBC $(LDFLAGS) -o $@ entry.S intr_test.c uart.c syscalls.c -lc -lgcc

hello.out: entry.S hello.c uart.c syscalls.c link.ld
	$(CC) $(CFLAGS) -DWITH_TRAP -DWITH_LIBC $(LDFLAGS) -o $@ entry.S hello.c uart.c syscalls.c -lc -lgcc

clean:
	rm -f *.out *.o

.PHONY: all clean
