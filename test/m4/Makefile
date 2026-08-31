# rv906 M4 test suite -- the privilege / virtual-memory suites.
#
#   make -C test/m4                 # build every ELF of every list
#   make -s -C test/m4 list-si      # the 7 rv64si-p names, one per line
#   make -s -C test/m4 list-mi      # the 17 rv64mi-p names, one per line
#   make -s -C test/m4 list-v       # the 86 rv64u*-v names, one per line
#   make -C test/m4 dump            # objdump -d each ELF
#   make -C test/m4 clean
#
# Run one with the full-stack simulator (tohost protocol; PASS = tohost 1):
#   timeout 300 bin/verisim/testbench --print-result test/m4/build/rv64si-p-csr.elf
#
# MIRROR of rv12's test/m4 (rv12/test/m4/Makefile), adapted to rv906's simpler
# harness: rv906 has NO ISS-diff checker and no rung mechanism -- the runner is
# `bin/verisim/testbench --print-result` + grep PASS, same as test/m2's
# run_all.sh. So the four-list Makefile shape is kept (si / mi / v / mmu) but
# the rungs/ISS machinery is dropped.
#
# THE NO-$(wildcard) PROHIBITION IS INHERITED (test/m2/Makefile's argument):
# the lists below are SPELLED OUT, copied verbatim from the upstream Makefrags,
# so a stray .S cannot silently join a suite and move a pass count.
#
# THE TEST SOURCES ARE NOT VENDORED. The p-env (linker script + riscv_test.h +
# encoding.h) lives in test/m2/env (shared BY PATH, not copied -- there is
# nothing M4-specific about it). The v-env override lives in test/m4/env/v
# (rv906v.ld + riscv_test.h, the uncached-tohost treatment); it is only needed
# for the v suite (built once the MMU exists, Task 9). The upstream .S files are
# compiled IN PLACE out of the riscv-tests checkout named by RISCV_TESTS.
#
# PRIVILEGE MODE OF THE TEST BODIES: the p-env RVTEST_CODE_BEGIN runs the
# per-test init macro (RVTEST_RV64M / RVTEST_RV64S), which sets mstatus.MPP
# appropriately before the mret into the body. rv64mi bodies run in M-mode,
# rv64si bodies in S-mode. rv906's M4 privilege machine (CSR.v) provides the
# M/S/U modes these exercise.

M4_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
RV906_DIR   := $(abspath $(M4_DIR)../..)
M2_DIR      := $(RV906_DIR)/test/m2
ENV_V_DIR   := $(M4_DIR)env/v
BUILD_DIR   := $(M4_DIR)build

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

RISCV_TESTS ?= /home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/rv-test/riscv-tests

#-----------------------------------------------------------------------------
# List 1: SI_TESTS -- 7. COPIED VERBATIM from isa/rv64si/Makefrag's
# rv64si_sc_tests, in its order. The p env: these tests build their OWN page
# tables where they need translation at all (dirty, icache-alias) and run
# everything else in M/S-mode untranslated, which is why upstream has no
# rv64si_v_tests line.
#-----------------------------------------------------------------------------
SI_TESTS = \
	csr \
	dirty \
	icache-alias \
	ma_fetch \
	scall \
	wfi \
	sbreak

#-----------------------------------------------------------------------------
# List 2: MI_TESTS -- 17. COPIED VERBATIM from isa/rv64mi/Makefrag's
# rv64mi_sc_tests, in its order.
#-----------------------------------------------------------------------------
MI_TESTS = \
	breakpoint \
	csr \
	mcsr \
	illegal \
	ma_fetch \
	ma_addr \
	scall \
	sbreak \
	ld-misaligned \
	lw-misaligned \
	lh-misaligned \
	sh-misaligned \
	sw-misaligned \
	sd-misaligned \
	zicntr \
	instret_overflow \
	pmpaddr

#-----------------------------------------------------------------------------
# List 3: V_TESTS -- 86 (built once the MMU exists, Task 9). COPIED VERBATIM
# from the three Makefrags' sc_tests lists with the -v- prefix: rv64ui (54) +
# rv64um (13) + rv64ua (19) = 86. Same sources as test/m2/m3, different env.
#-----------------------------------------------------------------------------
RV64UI_V_TESTS = \
	add addi addiw addw \
	and andi \
	auipc \
	beq bge bgeu blt bltu bne \
	simple \
	fence_i \
	jal jalr \
	lb lbu lh lhu lw lwu ld ld_st \
	lui \
	ma_data \
	or ori \
	sb sh sw sd st_ld \
	sll slli slliw sllw \
	slt slti sltiu sltu \
	sra srai sraiw sraw \
	srl srli srliw srlw \
	sub subw \
	xor xori

RV64UM_V_TESTS = \
	div divu divuw divw \
	mul mulh mulhsu mulhu mulw \
	rem remu remuw remw

RV64UA_V_TESTS = \
	amoadd_d amoand_d amomax_d amomaxu_d \
	amomin_d amominu_d amoor_d amoxor_d \
	amoswap_d \
	amoadd_w amoand_w amomax_w amomaxu_w \
	amomin_w amominu_w amoor_w amoxor_w \
	amoswap_w \
	lrsc

#-----------------------------------------------------------------------------
# List 4: MMU_TESTS -- directed MMU shapes (added in Task 9).
#-----------------------------------------------------------------------------
MMU_TESTS =

SI_ELFS  = $(addprefix $(BUILD_DIR)/rv64si-p-,$(addsuffix .elf,$(SI_TESTS)))
MI_ELFS  = $(addprefix $(BUILD_DIR)/rv64mi-p-,$(addsuffix .elf,$(MI_TESTS)))
V_TESTS  = $(addprefix rv64ui-v-,$(RV64UI_V_TESTS)) \
           $(addprefix rv64um-v-,$(RV64UM_V_TESTS)) \
           $(addprefix rv64ua-v-,$(RV64UA_V_TESTS))
V_ELFS   = $(addprefix $(BUILD_DIR)/,$(addsuffix .elf,$(V_TESTS)))
MMU_ELFS = $(addprefix $(BUILD_DIR)/,$(addsuffix .elf,$(MMU_TESTS)))
ELFS     = $(SI_ELFS) $(MI_ELFS) $(V_ELFS) $(MMU_ELFS)

#-----------------------------------------------------------------------------
# Compile lines
#-----------------------------------------------------------------------------
# MARCH: `a` is in it because the directed MMU set (Task 9) contains an AMO
# shape; `c` is left out so the assembler cannot silently compress a body whose
# encodings the tests care about (test/m2's argument).
MARCH   = rv64ima_zicsr_zifencei

# MARCH_V carries an `f` for the ASSEMBLER alone (rv12's M-3 measurement):
# without it env/v/vm.c fails to assemble an fssr in its never-executed FP
# reference path. No FP is actually run. Added when the v suite is built.
MARCH_V = rv64imaf_zicsr_zifencei

# The p-env linker script is test/m2's BY PATH (nothing M4-specific). The
# v-env script is test/m4/env/v/rv906v.ld (uncached tohost), built in Task 9.
LDSCRIPT   = $(M2_DIR)/env/rv906.ld
LDSCRIPT_V = $(ENV_V_DIR)/rv906v.ld

V_ENV_SRCS = $(RISCV_TESTS)/env/v/entry.S $(RISCV_TESTS)/env/v/vm.c \
             $(RISCV_TESTS)/env/v/string.c

CFLAGS_COMMON = -mabi=lp64 -static -mcmodel=medany \
                -fvisibility=hidden -nostdlib -nostartfiles \
                -Wl,--no-warn-rwx-segments \
                -I $(RISCV_TESTS)/isa/macros/scalar

# p-env include path: test/m2/env FIRST so riscv_test.h finds rv906's header.
# MHCR boot preamble ON by default (design doc S7.3) -- same as test/m2.
CFLAGS   = $(CFLAGS_COMMON) -I $(M2_DIR)/env
RV906_BOOT_MHCR ?= 1
ifeq ($(RV906_BOOT_MHCR),1)
CFLAGS += -DRV906_BOOT_MHCR
endif

# v-env include order is load-bearing (rv12's argument): local env/v FIRST so
# #include_next finds upstream; ENTROPY pinned for reproducibility.
CFLAGS_V = $(CFLAGS_COMMON) -I $(ENV_V_DIR) -I $(RISCV_TESTS)/env/v \
           -DENTROPY=0x5a -std=gnu99 -O2

DEPS   = $(LDSCRIPT) $(M2_DIR)/env/riscv_test.h $(M2_DIR)/env/encoding.h
DEPS_V = $(LDSCRIPT_V) $(ENV_V_DIR)/riscv_test.h $(V_ENV_SRCS) \
         $(RISCV_TESTS)/env/v/riscv_test.h $(RISCV_TESTS)/env/p/riscv_test.h

.PHONY: all dump clean list-si list-mi list-v list-mmu si mi v mmu

all: $(ELFS)

# Convenience: build one suite at a time (si/mi now, v/mmu once the MMU
# exists in Task 9).
si:  $(SI_ELFS)
mi:  $(MI_ELFS)
v:   $(V_ELFS)
mmu: $(MMU_ELFS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

#-----------------------------------------------------------------------------
# Build rules
#-----------------------------------------------------------------------------
$(BUILD_DIR)/rv64si-p-%.elf: $(RISCV_TESTS)/isa/rv64si/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

$(BUILD_DIR)/rv64mi-p-%.elf: $(RISCV_TESTS)/isa/rv64mi/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

$(BUILD_DIR)/rv64ui-v-%.elf: $(RISCV_TESTS)/isa/rv64ui/%.S $(DEPS_V) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_V) $(CFLAGS_V) -T $(LDSCRIPT_V) $(V_ENV_SRCS) $< -o $@

$(BUILD_DIR)/rv64um-v-%.elf: $(RISCV_TESTS)/isa/rv64um/%.S $(DEPS_V) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_V) $(CFLAGS_V) -T $(LDSCRIPT_V) $(V_ENV_SRCS) $< -o $@

$(BUILD_DIR)/rv64ua-v-%.elf: $(RISCV_TESTS)/isa/rv64ua/%.S $(DEPS_V) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_V) $(CFLAGS_V) -T $(LDSCRIPT_V) $(V_ENV_SRCS) $< -o $@

$(BUILD_DIR)/mmu_%.elf: $(M4_DIR)mmu_%.S $(M4_DIR)mmu_env.h $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

#-----------------------------------------------------------------------------
# INTERFACE: one test name per line, no decoration. CALL AS make -s -C test/m4
# list-si (etc). The -s is load-bearing (test/m2's argument).
#-----------------------------------------------------------------------------
list-si:
	@for t in $(SI_TESTS); do echo rv64si-p-$$t; done

list-mi:
	@for t in $(MI_TESTS); do echo rv64mi-p-$$t; done

list-v:
	@for t in $(V_TESTS); do echo $$t; done

list-mmu:
	@for t in $(MMU_TESTS); do echo $$t; done

dump: $(ELFS)
	@for e in $(ELFS); do \
	    n=`basename $$e .elf`; \
	    $(OBJDUMP) -d -M no-aliases $$e > $(BUILD_DIR)/$$n.dump; \
	done

clean:
	rm -rf $(BUILD_DIR)
