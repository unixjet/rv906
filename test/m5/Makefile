# rv906 M5 test suite -- the scalar FPU (F/D) riscv-tests build scaffolding.
#
#   make -C test/m5                 # build all 46 ELFs into test/m5/build/
#   make -s -C test/m5 list-uf      # the 11 rv64uf-p names, one per line
#   make -s -C test/m5 list-ud      # the 12 rv64ud-p names, one per line
#   make -s -C test/m5 list-ufv     # the 11 rv64uf-v names, one per line
#   make -s -C test/m5 list-udv     # the 12 rv64ud-v names, one per line
#   make -C test/m5 dump            # objdump -d each ELF
#   make -C test/m5 clean
#
# M5 Task 10 (design doc docs/superpowers/specs/2026-09-05-m5-fpu-design.md,
# D12 + task-table row 10): BUILD ONLY. This Makefile's job is to make the
# 46 upstream riscv-tests FP-compliance ELFs COMPILE against rv906's env.
# It does NOT run them against the RTL -- that is Task 11 (Acceptance +
# close-out). Do not add a run/sweep target here.
#
# Run one with the full-stack simulator (tohost protocol; PASS = tohost 1),
# once Task 11 wires this suite into a sweep:
#   timeout 300 bin/verisim/testbench --print-result test/m5/build/rv64uf-p-fadd.elf
#
# MIRROR of test/m4/Makefile's shape (si/mi/v/mmu four-list pattern), with
# the four lists renamed uf/ud/ufv/udv for the FP suites. Same no-$(wildcard)
# discipline as test/m2 and test/m4 (spelled out below, argued there): a
# stray .S added to the upstream isa/rv64uf or isa/rv64ud directories must
# not silently join the suite and move a pass/ELF count.
#
# THE TEST SOURCES ARE NOT VENDORED. The p-env (linker script + riscv_test.h
# + encoding.h) is test/m2/env, reused BY PATH exactly as test/m4's p-suite
# does -- nothing FP-specific is needed there: test/m2/env/riscv_test.h
# already carries the full upstream RVTEST_RV64UF/RVTEST_FP_ENABLE macro set
# (it was adapted from upstream's own env/p/riscv_test.h, which defines those
# macros regardless of whether M2's own test lists used them), and
# test/m2/env/encoding.h already has MSTATUS_FS. The v-env override is
# test/m4/env/v (rv906v.ld + the riscv_test.h layer that adds EXTRA_INIT and
# #include_nexts to upstream's env/v/riscv_test.h, which itself #includes
# upstream's env/p/riscv_test.h for RVTEST_RV64UF -- a wholly upstream chain,
# unrelated to test/m2's local header). The upstream .S files are compiled IN
# PLACE out of the riscv-tests checkout named by RISCV_TESTS below.
#
# 46 ELFS, NOT 48 -- VERIFIED, NOT ASSUMED. The design doc's own D12 originally
# miscounted this as "48 ELFs" (`{12 names} x {rv64uf,rv64ud} x {p,v}`),
# copying rv12's structural-test-inclusive 12-name list onto BOTH families.
# That arithmetic is wrong: rv64uf has no `structural` test (verified against
# isa/rv64uf/Makefrag's rv64uf_sc_tests -- 11 names -- vs isa/rv64ud/Makefrag's
# rv64ud_sc_tests -- 12 names, `structural` being double-only). rv12's own
# test/m5/Makefile header independently says "46 SUITE ELFs" for the same
# reason. Real count: 11 (uf-p) + 12 (ud-p) + 11 (uf-v) + 12 (ud-v) = 46. The
# design doc has been corrected to match (see D12 and the task/verification-
# plan lines that used to say 48).

M5_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
RV906_DIR   := $(abspath $(M5_DIR)../..)
M2_DIR      := $(RV906_DIR)/test/m2
ENV_V_DIR   := $(RV906_DIR)/test/m4/env/v
BUILD_DIR   := $(M5_DIR)build

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

RISCV_TESTS ?= /home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/rv-test/riscv-tests

#-----------------------------------------------------------------------------
# Test lists -- SPELLED OUT ON PURPOSE, do NOT replace with $(wildcard ...).
# Copied verbatim from upstream isa/rv64uf/Makefrag's rv64uf_sc_tests (11)
# and isa/rv64ud/Makefrag's rv64ud_sc_tests (12), re-verified directly
# against those two files while writing this Makefile:
#
#   $ sed -n '5,10p' $(RISCV_TESTS)/isa/rv64uf/Makefrag
#   rv64uf_sc_tests = \
#           fadd fdiv fclass fcmp fcvt fcvt_w fmadd fmin \
#           ldst move recoding \
#
#   $ sed -n '5,10p' $(RISCV_TESTS)/isa/rv64ud/Makefrag
#   rv64ud_sc_tests = \
#           fadd fdiv fclass fcmp fcvt fcvt_w fmadd fmin \
#           ldst move structural recoding \
#
# rv64ud has exactly one more name than rv64uf: `structural`, a double-only
# test (no single-precision structural-hazard equivalent upstream). This is
# a real, verified asymmetry -- not something to "fix" into a matching 12/12.
#-----------------------------------------------------------------------------

UF_TESTS = \
	fadd fdiv fclass fcmp fcvt fcvt_w fmadd fmin \
	ldst move recoding

UD_TESTS = \
	fadd fdiv fclass fcmp fcvt fcvt_w fmadd fmin \
	ldst move structural recoding

# The v-suite runs the SAME upstream sc_tests lists with a -v- prefix
# (upstream's own rv64uf_v_tests/rv64ud_v_tests convention) -- same sources,
# different env (test/m4/env/v instead of test/m2/env).
UFV_TESTS = $(UF_TESTS)
UDV_TESTS = $(UD_TESTS)

UF_ELFS  = $(addprefix $(BUILD_DIR)/rv64uf-p-,$(addsuffix .elf,$(UF_TESTS)))
UD_ELFS  = $(addprefix $(BUILD_DIR)/rv64ud-p-,$(addsuffix .elf,$(UD_TESTS)))
UFV_ELFS = $(addprefix $(BUILD_DIR)/rv64uf-v-,$(addsuffix .elf,$(UFV_TESTS)))
UDV_ELFS = $(addprefix $(BUILD_DIR)/rv64ud-v-,$(addsuffix .elf,$(UDV_TESTS)))
ELFS     = $(UF_ELFS) $(UD_ELFS) $(UFV_ELFS) $(UDV_ELFS)

#-----------------------------------------------------------------------------
# Compile lines
#-----------------------------------------------------------------------------
# MARCH: test/m2's own base march (rv64im_zicsr_zifencei) plus `f`+`d` for
# the p-suite's real FP encodings. No `a` (the p-suite FP tests don't need
# it) and `c` deliberately left OUT, same rationale as test/m2/test/m4: the
# assembler must not be allowed to silently compress a body whose exact
# encodings these compliance tests care about.
MARCH   = rv64imfd_zicsr_zifencei

# MARCH_V: test/m4's own v-march (rv64imaf_zicsr_zifencei, which already
# carries `f` -- added in M4 for the ASSEMBLER alone, so env/v/vm.c's
# never-executed FP reference path assembles; see test/m4/Makefile's own
# comment) plus `d`, since the v-suite now needs to assemble AND execute
# real double-precision encodings too.
MARCH_V = rv64imafd_zicsr_zifencei

# p-env: test/m2/env, BY PATH -- nothing FP-specific needed there (see
# header comment above). v-env: test/m4/env/v, BY PATH, same as test/m4's
# own v-suite rule.
LDSCRIPT   = $(M2_DIR)/env/rv906.ld
LDSCRIPT_V = $(ENV_V_DIR)/rv906v.ld

V_ENV_SRCS = $(RISCV_TESTS)/env/v/entry.S $(RISCV_TESTS)/env/v/vm.c \
             $(RISCV_TESTS)/env/v/string.c

CFLAGS_COMMON = -mabi=lp64 -static -mcmodel=medany \
                -fvisibility=hidden -nostdlib -nostartfiles \
                -Wl,--no-warn-rwx-segments \
                -I $(RISCV_TESTS)/isa/macros/scalar

# p-env include path: test/m2/env FIRST so riscv_test.h finds rv906's header
# (same as test/m4's own p-suite rule). MHCR boot preamble ON by default
# (design doc S7.3), same as test/m2/test/m4 -- honored inside
# test/m2/env/riscv_test.h itself via #ifdef RV906_BOOT_MHCR.
CFLAGS = $(CFLAGS_COMMON) -I $(M2_DIR)/env
RV906_BOOT_MHCR ?= 1
ifeq ($(RV906_BOOT_MHCR),1)
CFLAGS += -DRV906_BOOT_MHCR
endif

# v-env include order is load-bearing (rv12's argument, inherited via
# test/m4): local env/v FIRST so #include_next finds upstream's env/v
# second; ENTROPY pinned for reproducibility.
CFLAGS_V = $(CFLAGS_COMMON) -I $(ENV_V_DIR) -I $(RISCV_TESTS)/env/v \
           -DENTROPY=0x5a -std=gnu99 -O2

DEPS   = $(LDSCRIPT) $(M2_DIR)/env/riscv_test.h $(M2_DIR)/env/encoding.h
DEPS_V = $(LDSCRIPT_V) $(ENV_V_DIR)/riscv_test.h $(V_ENV_SRCS) \
         $(RISCV_TESTS)/env/v/riscv_test.h $(RISCV_TESTS)/env/p/riscv_test.h

.PHONY: all dump clean list-uf list-ud list-ufv list-udv uf ud ufv udv

all: $(ELFS)

# Convenience: build one suite at a time.
uf:  $(UF_ELFS)
ud:  $(UD_ELFS)
ufv: $(UFV_ELFS)
udv: $(UDV_ELFS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

#-----------------------------------------------------------------------------
# Build rules
#-----------------------------------------------------------------------------
$(BUILD_DIR)/rv64uf-p-%.elf: $(RISCV_TESTS)/isa/rv64uf/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

$(BUILD_DIR)/rv64ud-p-%.elf: $(RISCV_TESTS)/isa/rv64ud/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

$(BUILD_DIR)/rv64uf-v-%.elf: $(RISCV_TESTS)/isa/rv64uf/%.S $(DEPS_V) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_V) $(CFLAGS_V) -T $(LDSCRIPT_V) $(V_ENV_SRCS) $< -o $@

$(BUILD_DIR)/rv64ud-v-%.elf: $(RISCV_TESTS)/isa/rv64ud/%.S $(DEPS_V) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_V) $(CFLAGS_V) -T $(LDSCRIPT_V) $(V_ENV_SRCS) $< -o $@

#-----------------------------------------------------------------------------
# INTERFACE: one test name per line, no decoration. CALL AS
# make -s -C test/m5 list-uf (etc). The -s is load-bearing (test/m2's
# argument: `make -C` turns on --print-directory, which writes an
# Entering/Leaving banner to stdout before this makefile is even read).
#-----------------------------------------------------------------------------
list-uf:
	@for t in $(UF_TESTS); do echo rv64uf-p-$$t; done

list-ud:
	@for t in $(UD_TESTS); do echo rv64ud-p-$$t; done

list-ufv:
	@for t in $(UFV_TESTS); do echo rv64uf-v-$$t; done

list-udv:
	@for t in $(UDV_TESTS); do echo rv64ud-v-$$t; done

dump: $(ELFS)
	@for e in $(ELFS); do \
	    n=`basename $$e .elf`; \
	    $(OBJDUMP) -d -M no-aliases $$e > $(BUILD_DIR)/$$n.dump; \
	done

clean:
	rm -rf $(BUILD_DIR)
