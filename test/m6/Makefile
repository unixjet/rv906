# rv906 M6 directed interrupt test build (Task 7)
#
#   make -C test/m6            # build all ELFs into test/m6/build/
#   make -C test/m6 dump       # objdump -d each ELF into build/<name>.dump
#   make -s -C test/m6 list    # print the test names, one per line
#   make -C test/m6 clean
#
# Run one (tohost protocol; PASS = tohost 1):
#   timeout 300 bin/verisim/testbench --print-result test/m6/build/m6-msip.elf
#
# Run all:
#   bash test/m6/run_all.sh
#
# Test set (design doc, docs/superpowers/specs/
# 2026-09-08-m6-interrupts-linux-design.md):
#   msip        M-mode self-interrupt (msip pin, cause 3|bit63)
#   mtip        mtimecmp timer tick (cause 7) + time CSR check
#   plic_uart   UART IRQ -> PLIC source 7 -> MEIP (cause 11|bit63)
#   ssip_deleg  mideleg[1]; S-mode SSIP -> S trap (cause 1|bit63)
#   stip_sbi    SBI-model timer: M handler sets STIP -> S trap (5|bit63)
#   priority    MEIP+MSIP+MTIP all pending -> MEIP claimed first
#   vectored    mtvec[0]=1 -> epc = base + 4*cause
#   wfi_wake    wfi returns on pending MTIP; SIE=0+SSIP returns at once
#
# Convention: .text @ 0x80000000 (reset vector), .tohost @ 0x7FFFF000
# (uncached aperture) -- see common.ld.

M6_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR   := $(M6_DIR)build

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

CFLAGS  = -march=rv64imac_zicsr -mabi=lp64 -O2 -nostdlib -g
LDFLAGS = -T $(M6_DIR)common.ld -nostdlib

# TESTS is globbed from the .S sources present, so the suite grows
# incrementally as each milestone adds a test (Task 2 = msip; the full
# set lands at Task 7). `make list` and the build always match the
# sources that exist.
TESTS := $(basename $(notdir $(wildcard $(M6_DIR)/*.S)))
ELFS  = $(addprefix $(BUILD_DIR)/m6-,$(addsuffix .elf,$(TESTS)))

all: $(ELFS)

$(BUILD_DIR)/m6-%.elf: $(M6_DIR)%.S $(M6_DIR)common.ld | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

dump: $(ELFS)
	@for e in $(ELFS); do $(OBJDUMP) -d $$e > $$(echo $$e | sed 's/\.elf$$/.dump/'); done

list:
	@for t in $(TESTS); do echo m6-$$t; done

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all dump list clean
