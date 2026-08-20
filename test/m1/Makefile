# rv906 M1 directed fetch tests (plan Task 5.3; reused unmodified by Task 6)
#
#   make -C test/m1            # build every test/m1/*.S into a .out ELF
#   make -C test/m1 clean
#
# File-structure precedent: rv12's own test/m1/Makefile (style reference
# only, per the plan). Reused unchanged: the CROSS_COMPILE toolchain, the
# -nostdlib/-static/-mno-relax flags. Changed from rv12's own copy per the
# M1 design spec S4.2: rv906/C906 targets the RV64IMAFDC baseline scalar
# subset (RVC included), so the march string used here is
# rv64imac_zicsr -- rv12's C910 clone used the narrower rv64imc.
#
# Run one with the full-stack simulator (PASS convention: exit status 1):
#   timeout 60 bin/verisim/testbench --print-result --m1-rung=1 test/m1/seq.out
#
# The unit benches under test/m1/unit have their own Makefile and are NOT
# built from here.

CROSS_COMPILE ?= /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
CC       = $(CROSS_COMPILE)gcc
OBJDUMP  = $(CROSS_COMPILE)objdump

# rv64imac_zicsr: the M1 front end is RVC-aware and the directed tests need
# both instruction lengths; zicsr/a/m are part of C906's real baseline ISA
# even though this fetch-only front end never executes CSR/atomic/mul-div
# content. -nostdlib/-nostartfiles/-static: there is no runtime, _start is
# the reset vector. -mno-relax keeps the linker from rewriting code placed
# at hand-computed (JR_PAD) addresses.
CFLAGS  = -march=rv64imac_zicsr -mabi=lp64 -mno-relax \
          -nostdlib -nostartfiles -static -I$(CURDIR)

SRCS = $(wildcard *.S)
OUTS = $(SRCS:.S=.out)

.PHONY: all clean dis
all: $(OUTS)

%.out: %.S common.ld macros.h
	$(CC) $(CFLAGS) -T common.ld -o $@ $<

# Handy during test authoring: make dis T=iss_selftest
dis: $(OUTS)
	$(OBJDUMP) -d -M no-aliases $(T).out

clean:
	rm -f $(OUTS)
