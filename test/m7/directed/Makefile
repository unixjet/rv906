# rv906 M7 directed trigger test build (Task 2)
#
#   make -C test/m7/directed     # build all ELFs into test/m7/directed/build/
#   make -C test/m7/directed clean
#
# Run one (tohost protocol; PASS = tohost 1):
#   timeout 300 bin/verisim/testbench --print-result test/m7/directed/build/m7-break_no_skip.elf
#
# Run all:
#   bash test/m7/directed/run_directed.sh
#
# Test set:
#   break_no_skip  execute trigger (mcontrol, eq, M-mode, action=breakpoint):
#                  traps exactly once per armed-PC hit (mcause 3,
#                  mepc == armed PC), never executes the triggered
#                  instruction, trap count == expected or FAIL.
#
# Convention: .text @ 0x80000000 (reset vector), .tohost @ 0x7FFFF000
# (uncached aperture) -- see common.ld.

M7_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR   := $(M7_DIR)build

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

CFLAGS  = -march=rv64imac_zicsr -mabi=lp64 -O2 -nostdlib -g
LDFLAGS = -T $(M7_DIR)common.ld -nostdlib

TESTS := $(basename $(notdir $(wildcard $(M7_DIR)/*.S)))
ELFS  = $(addprefix $(BUILD_DIR)/m7-,$(addsuffix .elf,$(TESTS)))

all: $(ELFS)

$(BUILD_DIR)/m7-%.elf: $(M7_DIR)%.S $(M7_DIR)common.ld | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

dump: $(ELFS)
	@for e in $(ELFS); do $(OBJDUMP) -d $$e > $$(echo $$e | sed 's/\.elf$$/.dump/'); done

list:
	@for t in $(TESTS); do echo m7-$$t; done

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all dump list clean
