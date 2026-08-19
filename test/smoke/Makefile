CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC = $(CROSS_COMPILE)gcc

smoke.out: tohost.S smoke.ld
	$(CC) -march=rv64i -mabi=lp64 -nostdlib -nostartfiles -static \
	      -T smoke.ld -o $@ tohost.S

clean:
	rm -f smoke.out

.PHONY: clean
