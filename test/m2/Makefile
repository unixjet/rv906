# rv906 M2 riscv-tests build (rv64ui + rv64um + one local rv64uc, p-env)
#
#   make -C test/m2            # build all 68 ELFs into test/m2/build/
#   make -C test/m2 dump       # objdump -d each ELF into build/<name>.dump
#   make -s -C test/m2 list    # print the 68 test names, one per line
#                              # (the -s is load-bearing -- see `list` below)
#   make -C test/m2 clean
#
# Run one with the full-stack simulator (tohost protocol; PASS = tohost 1):
#   timeout 300 bin/verisim/testbench --print-result test/m2/build/rv64ui-p-add.elf
#
# THE TEST SOURCES ARE NOT VENDORED, WITH ONE EXCEPTION. Only the env
# (linker script + p-env header + encoding.h) lives here; the upstream .S
# files are compiled IN PLACE out of the riscv-tests checkout named by
# RISCV_TESTS below. The one source that IS in this repository is rvc.S,
# next to this makefile: a local, non-upstream directed test for the
# compressed instruction set, and the only test in the suite compiled with
# the C extension.
#
# Which checkout: /home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/rv-test/
# riscv-tests (plan 8.4 candidate 1; the two other workspace checkouts are
# content-identical for isa/rv64ui + isa/rv64um -- 54 + 13 tests each --
# and this one sits under the rvproc tree, so it is the one we reference).
# Recorded here at the vendoring site per plan 8.4.
#
# PRIVILEGE MODE OF THE TEST BODIES -- DECIDED AND RECORDED HERE (same call
# as rv12's test/m2):
#   We keep the stock p-env behaviour. `1: csrwi mstatus, 0` in
#   RVTEST_CODE_BEGIN wipes mstatus.MPP one instruction before the (empty)
#   RVTEST_RV64U init, so the `mret` drops into the test body in **U-mode**
#   and the terminating ecall reports **cause 8** (CAUSE_USER_ECALL). The
#   p-env trap_vector (env/riscv_test.h) checks mcause against CAUSE_USER_
#   ECALL / CAUSE_SUPERVISOR_ECALL / CAUSE_MACHINE_ECALL and branches to
#   write_tohost on any of them, so U-mode cause 8 is handled. We
#   deliberately do NOT add the RVTEST_ENABLE_MACHINE idiom
#   (`csrs mstatus, MSTATUS_MPP`).
#
# MHCR / caches: EVERY MHCR enable resets to 0 on rv906 (faithful to the
# C906 donor), so the ICache and DCache are OFF out of reset. Design doc
# S7.3 makes the architectural boot preamble MANDATORY for the M2
# acceptance sweep (otherwise MHCR.de=0 and the DCache is never exercised),
# so it is compiled in BY DEFAULT -- the Makefile defines RV906_BOOT_MHCR
# unless overridden:
#   make -C test/m2 RV906_BOOT_MHCR=0     # caches OFF (Task 10.1 sanity)

M2_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ENV_DIR     := $(M2_DIR)env
BUILD_DIR   := $(M2_DIR)build

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

# Upstream riscv-tests checkout supplying isa/rv64ui, isa/rv64um and
# isa/macros/scalar/test_macros.h. Overridable:
#   make -C test/m2 RISCV_TESTS=/path/to/riscv-tests
RISCV_TESTS ?= /home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/rv-test/riscv-tests

#-----------------------------------------------------------------------------
# Test lists
#-----------------------------------------------------------------------------
# SPELLED OUT ON PURPOSE -- do NOT replace with $(wildcard $(RISCV_TESTS)/isa/
# rv64ui/*.S). The upstream Makefrag lists (rv64ui_sc_tests in
# isa/rv64ui/Makefrag, rv64um_sc_tests in isa/rv64um/Makefrag) are the
# contract for what the suite contains; a stray or newly added .S in that
# directory must not silently join the suite and change the pass count.
# Copied from those Makefrags verbatim: 54 + 13 = 67.

RV64UI_TESTS = \
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

RV64UM_TESTS = \
	div divu divuw divw \
	mul mulh mulhsu mulhu mulw \
	rem remu remuw remw

# The one LOCAL test: test/m2/rvc.S, built as rv64uc-p-rvc.elf. The `rv64uc`
# prefix is upstream's family name for compressed-instruction tests, so the
# name slots into the same rv64u<ext>-p-<name> namespace as the other 67 even
# though the source is ours. 67 + 1 = 68.
RV64UC_TESTS = rvc

# M3 Task 7: RV64A atomics (AMO + LR/SC). From isa/rv64ua/Makefrag's
# rv64ua_sc_tests. These need the `a` extension in -march (MARCH_A below).
RV64UA_TESTS = \
	amoadd_w amoadd_d \
	amoand_w amoand_d \
	amomax_w amomax_d \
	amomaxu_w amomaxu_d \
	amomin_w amomin_d \
	amominu_w amominu_d \
	amoor_w amoor_d \
	amoswap_w amoswap_d \
	amoxor_w amoxor_d \
	lrsc

RV64UI_ELFS = $(addprefix $(BUILD_DIR)/rv64ui-p-,$(addsuffix .elf,$(RV64UI_TESTS)))
RV64UM_ELFS = $(addprefix $(BUILD_DIR)/rv64um-p-,$(addsuffix .elf,$(RV64UM_TESTS)))
RV64UC_ELFS = $(addprefix $(BUILD_DIR)/rv64uc-p-,$(addsuffix .elf,$(RV64UC_TESTS)))
RV64UA_ELFS = $(addprefix $(BUILD_DIR)/rv64ua-p-,$(addsuffix .elf,$(RV64UA_TESTS)))
ELFS        = $(RV64UI_ELFS) $(RV64UM_ELFS) $(RV64UC_ELFS) $(RV64UA_ELFS)

#-----------------------------------------------------------------------------
# Compile line
#-----------------------------------------------------------------------------
# THE -march STRING IS THE ONLY THING THAT DIFFERS BETWEEN THE TWO GROUPS:
#
#   MARCH      rv64im_zicsr_zifencei    -- the 67 UPSTREAM tests
#     NOT upstream's rv64g: rv906 is RV64IMC + Zicsr (+ F/D since the M5
#     swap; misa now reports I|M|C|F|D = 0x800000000000112C), and the
#     tests themselves are pure I/M sources. zifencei is required because
#     rv64ui-p-fence_i executes `fence.i`, which since binutils' split of the
#     base ISA is no longer part of plain `i`. `c` is left OUT so that the
#     assembler cannot silently compress these bodies -- the upstream sources
#     and their expected encodings are 32-bit, and keeping them that way is
#     what makes rv64uc-p-rvc the single, deliberate source of RVC coverage.
#
#   MARCH_RVC  rv64imc_zicsr_zifencei   -- rv64uc-p-rvc. The `c` is the whole
#     point of that test. Without it the ISS's RVC decoder (m2_iss.h,
#     Iss::rvc_expand) is never exercised by the suite at all: every one of
#     the other 67 ELFs would still pass with the decoder stubbed out. This
#     is the ONE test that must be built with the C extension; if you find
#     yourself adding `c` to MARCH, don't -- that would hand the compressing
#     decision to the assembler in 67 places instead of keeping it in one
#     auditable file.
#
# Everything else on the compile line is identical for both groups.
# -mcmodel=medany + -static + -nostdlib/-nostartfiles: no runtime, _start IS
# the reset vector, and every symbol (tohost at 0x7FFF_F000, text at
# 0x80000000) is reached pc-relative.
# -Wl,--no-warn-rwx-segments: moving .tohost below the text base (see
# env/rv906.ld) puts the writable .tohost page and the executable .text.init
# into ONE PT_LOAD, which binutils >= 2.39 flags as an RWX segment. That
# warning is about W^X hardening for OS-loaded binaries; this ELF is loaded
# into a flat testbench memory model with no MMU and no page permissions, so
# the condition is meaningless here and the merged segment is what makes the
# ELF loader write .tohost at all. Nothing else in the build is silenced.
MARCH     = rv64im_zicsr_zifencei
MARCH_RVC = rv64imc_zicsr_zifencei
MARCH_A   = rv64ima_zicsr_zifencei

CFLAGS = -mabi=lp64 -static -mcmodel=medany \
         -fvisibility=hidden -nostdlib -nostartfiles \
         -Wl,--no-warn-rwx-segments \
         -I $(ENV_DIR) -I $(RISCV_TESTS)/isa/macros/scalar

# MHCR boot preamble: ON by default (design doc S7.3 -- the acceptance sweep
# must genuinely exercise the DCache). RV906_BOOT_MHCR=0 builds it out for
# the caches-off sanity run (Task 10.1).
RV906_BOOT_MHCR ?= 1
ifeq ($(RV906_BOOT_MHCR),1)
CFLAGS += -DRV906_BOOT_MHCR
endif

LDSCRIPT = $(ENV_DIR)/rv906.ld

DEPS = $(LDSCRIPT) $(ENV_DIR)/riscv_test.h $(ENV_DIR)/encoding.h

.PHONY: all dump clean list

all: $(ELFS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/rv64ui-p-%.elf: $(RISCV_TESTS)/isa/rv64ui/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

$(BUILD_DIR)/rv64um-p-%.elf: $(RISCV_TESTS)/isa/rv64um/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

# The local RVC test -- source in THIS directory. Identical to the two rules
# above in every other respect (same linker script, same include paths, same
# medany/nostdlib set); only the -march differs. It is the only SUITE member
# built with the C extension.
$(BUILD_DIR)/rv64uc-p-%.elf: $(M2_DIR)%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_RVC) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

# M3 Task 7: RV64A atomics. Only -march differs (adds the `a` extension).
$(BUILD_DIR)/rv64ua-p-%.elf: $(RISCV_TESTS)/isa/rv64ua/%.S $(DEPS) | $(BUILD_DIR)
	$(CC) -march=$(MARCH_A) $(CFLAGS) -T $(LDSCRIPT) $< -o $@

# Disassembly of every ELF, for debugging a failing test.
dump: $(ELFS)
	@for e in $(ELFS); do \
	    n=`basename $$e .elf`; \
	    $(OBJDUMP) -d -M no-aliases $$e > $(BUILD_DIR)/$$n.dump; \
	done

# INTERFACE (consumed by test/m2/run_all.sh's M2 sweep): print the 68 test
# names, one per line, no decoration. The ELF for name <n> is
# test/m2/build/<n>.elf. Keep the format exactly as is.
#
#   CALL IT AS:  make -s -C test/m2 list        (exactly 68 lines)
#
# The -s matters. GNU make turns on --print-directory whenever it is given
# -C, and it writes its "Entering/Leaving directory" banner to STDOUT before
# this makefile is even read -- so a plain `make -C test/m2 list` yields 70
# lines and no setting inside this file can prevent it (verified on GNU Make
# 4.3: MAKEFLAGS += --no-print-directory and .SILENT both come too late).
# -s (or --no-print-directory) suppresses the banner; a consumer that cannot
# pass it should filter with `grep -E '^rv64u'`.
list:
	@for t in $(RV64UI_TESTS); do echo rv64ui-p-$$t; done
	@for t in $(RV64UM_TESTS); do echo rv64um-p-$$t; done
	@for t in $(RV64UC_TESTS); do echo rv64uc-p-$$t; done

clean:
	rm -rf $(BUILD_DIR)
