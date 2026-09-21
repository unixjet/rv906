# rv906 M8 cross-check build (design doc
# docs/superpowers/specs/2026-09-21-m8-crosscheck-design.md, section 4.6)
#
#   make -C test/m8            # build every case ELF into test/m8/build/
#   make -C test/m8 list       # print the case names, one per line
#   make -C test/m8 dump       # objdump -d each ELF into build/<name>.dump
#   make -C test/m8 clean
#
# Run one (tohost protocol; PASS = tohost 1):
#   timeout 300 bin/verisim/testbench --print-result test/m8/build/csr.elf
#
# Run all (rv906 side):  bash test/m8/run_all.sh
# Run one (donor side):  bash test/m8/run_donor.sh <case>
#
# Source identity (D-M8-1): case bodies in cases/<name>/ are byte-identical
# copies of the donor smart_run tests/cases/<name>/ sources (csr verified
# by md5sum at T3), except the per-case deltas of design section 4 (none
# for csr). Glue is per-platform: crt0_m8.s (ported), link_m8.ld (ported),
# clib/ (ported; the only deltas are the two console-asm sites). The march
# string is the donor's T1-patched string verbatim
# (smart_run/tests/lib/Makefile:40): rv64imafdc_zicsr_zifencei / lp64d.

M8_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR   := $(M8_DIR)build

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

# D-M8-1: the donor's T1-patched march string, -O2 for the .s cases.
CFLAGS  = -march=rv64imafdc_zicsr_zifencei -mabi=lp64d -O2 -nostdlib -g
LDFLAGS = -T $(M8_DIR)link_m8.ld -nostdlib

# coremark CFLAGS (design section 4.5, both sides): the donor's
# tests/lib/Makefile:50 line MINUS the 4 vendor -m* flags (D-M8-2b:
# standard-toolchain wall) and MINUS -mtune=c906 (T6 deviation D1:
# rejected by standard gcc cc1 for .c files; removed donor-side by the
# approved patch 4 -- the rv906 line is identical-both-sides by omission),
# PLUS the 4 GCC>=14 acceptance tokens (T6 deviation D3 / approved
# patch 5: coremark's .c sources + clib target the older T-Head GCC,
# whose implicit declarations / int->pointer initializations are hard
# errors under GCC 14+; CFLAGS-only so the .c source identity of D-M8-1
# survives; identical-both-sides).
# Everything else byte-identical to the donor line, incl.
# -DITERATIONS=10000 (vestigial: core_main.c:178 hard-codes
# results[0].iterations = 2).
# T4b delta (rv906-only, platform link base): -mcmodel=medany. The xpack
# toolchain defaults to medlow, under which GCC emits ABSOLUTE lui-based
# data references (R_RISCV_HI20, verified by probe: default -> lui,
# -mcmodel=medany -> auipc). Absolute lui+addi is sign-extended 32-bit
# and cannot encode the 0x80000000+ link base (link_m8.ld image at the
# reset vector) -> "relocation truncated to fit: R_RISCV_HI20". medany is
# the same code model test/m2/Makefile already uses for its 0x80000000 C
# tests; it changes only the addressing mode (PC-relative auipc), not the
# benchmark semantics. No donor counterpart needed: the donor image is at
# 0x0 where either model encodes fine (verified: the donor-side build in
# T6 linked clean at 0x0).
COREMARK_CFLAGS = -march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -O3 -static \
  -funroll-all-loops -finline-limit=500 -fgcse-sm -fno-schedule-insns \
  --param max-rtl-if-conversion-unpredictable-cost=100 -fno-code-hoisting \
  -DITERATIONS=10000 \
  -Wno-implicit-function-declaration -Wno-implicit-int -Wno-int-conversion \
  -include stdint.h

# clib objects (ported donor clib; needed by the C cases only). NOTE:
# fputc.c and printf.c BOTH define fputc (the donor flow carries the same
# latent duplicate; its coremark link has never been reached, T6). When
# coremark lands (T4) exactly one of the two objects is linked -- decided
# there; both objects build fine standalone.
CLIB_DIR  = $(M8_DIR)clib
CLIB_OBJS = $(addprefix $(BUILD_DIR)/clib/,$(patsubst %.c,%.o,$(notdir $(wildcard $(CLIB_DIR)/*.c))))

# Assembly cases: one .s body per cases/<name>/ dir (csr now; interrupt/
# MMU at T4/T5). Coremark (multi-.c) has its own rule below and is inert
# until cases/coremark/ exists.
CASES := $(patsubst $(M8_DIR)cases/%/,%,$(sort $(dir $(wildcard $(M8_DIR)cases/*/*.s))))

# coremark sources (T4): present -> add the ELF to the default goal
COREMARK_CSRC = $(wildcard $(M8_DIR)cases/coremark/*.c)
COREMARK_HDRS = $(wildcard $(M8_DIR)cases/coremark/*.h)
ifneq ($(COREMARK_CSRC),)
CASES += coremark
endif

ELFS  := $(addprefix $(BUILD_DIR)/,$(addsuffix .elf,$(CASES)))

all: $(ELFS)

$(BUILD_DIR):
	mkdir -p $@

# ported crt0 (built once; linked into every case)
$(BUILD_DIR)/crt0_m8.o: $(M8_DIR)crt0_m8.s $(M8_DIR)link_m8.ld | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# clib objects (ported donor clib; needed by the C cases only).
# T4b: built with COREMARK_CFLAGS -- identical-both-sides: the donor flow
# compiles the clib TUs (fputc/intc/uart/vtimer) with the same coremark
# CFLAGS line (tests/lib/Makefile:50) as the case .c bodies, so the
# four GCC>=14 acceptance tokens (T6 patch 5) cover them too. Plain
# CFLAGS would reject uart.h's uint32_t and intc.c's int->pointer inits.
# (Also T4b: the pattern prereq gained its missing slash -- CLIB_DIR has
# no trailing one; surfaced now because coremark is the first case to
# build clib objects.)
$(BUILD_DIR)/clib/%.o: $(CLIB_DIR)/%.c | $(BUILD_DIR)/clib
	$(CC) $(COREMARK_CFLAGS) -c $< -o $@

$(BUILD_DIR)/clib:
	mkdir -p $@

# Assembly case: byte-identical donor body + crt0 (no clib objects needed;
# the bodies reference only main/__exit/__fail, all in body+crt0). One
# explicit rule per case dir; each dir must hold exactly one .s body.
define build_case
$(BUILD_DIR)/$(1).elf: $(2) $(BUILD_DIR)/crt0_m8.o | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $(2) -o $$@.case.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $$@ $$@.case.o $(BUILD_DIR)/crt0_m8.o
endef
$(foreach c,$(filter-out coremark,$(CASES)), \
  $(eval case_srcs_$(c) := $(wildcard $(M8_DIR)cases/$(c)/*.s)))
$(foreach c,$(filter-out coremark,$(CASES)), \
  $(if $(case_srcs_$(c)), \
    $(if $(filter-out $(firstword $(case_srcs_$(c))),$(case_srcs_$(c))), \
      $(error case '$(c)' has multiple .s bodies: $(case_srcs_$(c))), \
      $(eval $(call build_case,$(c),$(firstword $(case_srcs_$(c)))))), \
    $(error case '$(c)' has no .s body in cases/$(c)/)))

# coremark (T4): the .c bodies (byte-identical) + clib + crt0, compiled
# with COREMARK_CFLAGS. Inert until cases/coremark/ is ported.
# T4b: link line gains -lc -lgcc after the objects (donor flow's exact
# link ends "-lc -lgcc <objs> -lm"; stdlib provides printf/puts/memset --
# newlib stdout reaches the console through the linked fputc, i.e.
# clib/printf.c's THR 0x10000000 writer). fputc.o stays excluded from
# the link (duplicate fputc symbol with printf.c; printf.c's is the one
# that targets the rv906 console).
# T4b: crt0_m8.o is FIRST on the line. GCC compiles the C `main` into a
# `.text.startup` input section, so with the case objects first the
# image start (and the ISS reset vector 0x80000000 -- the harness NOTEs
# "the ISS follows the reset vector", e_entry is ignored) would be main's
# prologue, not __start: no boot pokes, sp/mtvec unset. crt0 first
# restores the documented crt0-first image order (donor linker.lcf's
# explicit `crt0.o (.text)`-first reference; link_m8.ld header).
ifneq ($(COREMARK_CSRC),)
$(BUILD_DIR)/coremark.elf: $(COREMARK_CSRC) $(COREMARK_HDRS) $(BUILD_DIR)/crt0_m8.o $(CLIB_OBJS) | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/coremark
	@for f in $(COREMARK_CSRC); do \
	  $(CC) $(COREMARK_CFLAGS) -I$(M8_DIR)clib -c $$f -o $(BUILD_DIR)/coremark/$$(basename $$(basename $$f .c)).o || exit 1; \
	done
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(BUILD_DIR)/crt0_m8.o $(BUILD_DIR)/coremark/*.o $(filter-out $(BUILD_DIR)/clib/fputc.o,$(CLIB_OBJS)) -lc -lgcc -lm
endif

dump: $(ELFS)
	@for e in $(ELFS); do $(OBJDUMP) -d $$e > $$(echo $$e | sed 's/\.elf$$/.dump/'); done

list:
	@for t in $(CASES); do echo $$t; done

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all dump list clean
