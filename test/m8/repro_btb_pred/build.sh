#!/bin/bash
# M8 T4c directed repro build -- BPU predicted-taken-branch / valid+correct
# BTB-hit fetch-redirect (fixed at rtl/BPU.v:824). See repro.c header.
#
# Self-contained: builds build/repro.elf under THIS directory, NOT
# test/m8/build/ -- so run_all.sh's `build/*.elf` glob never picks it up and
# it stays out of the donor parity set. The link is the m8 C-case link from
# test/m8/Makefile's coremark rule (crt0_m8.o first, clib with fputc.o
# excluded, newlib -lc -lgcc -lm, -T link_m8.ld), with the single repro.c
# body in place of the coremark .c files.
#
#   bash build.sh     # -> build/repro.elf
#   bash run.sh       # PASS on fixed RTL; TIMEOUT -> FAIL if BPU.v:824 regresses
#
# Prereq: `make verisim` has produced bin/verisim/testbench (for run.sh only).

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
M8="$REPO/test/m8"
OUT="$HERE/build"
CC="/opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc"

# D-M8-1 donor-verbatim march string + -mcmodel=medany (T4b: 0x80000000 base).
CFLAGS="-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -O2 -nostdlib -g"
# clib (and the .c body) compile with the coremark CFLAGS: the 4 GCC>=14
# acceptance tokens cover uint32_t / int->pointer inits in the clib TUs.
COREMARK_CFLAGS="-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -O3 -static \
  -funroll-all-loops -finline-limit=500 -fgcse-sm -fno-schedule-insns \
  --param max-rtl-if-conversion-unpredictable-cost=100 -fno-code-hoisting \
  -Wno-implicit-function-declaration -Wno-implicit-int -Wno-int-conversion \
  -include stdint.h"

mkdir -p "$OUT"

# 1. crt0 (built once, reused).
if [ ! -f "$OUT/crt0_m8.o" ] || [ "$M8/crt0_m8.s" -nt "$OUT/crt0_m8.o" ]; then
  $CC $CFLAGS -c "$M8/crt0_m8.s" -o "$OUT/crt0_m8.o"
fi

# 2. clib objects, EXCLUDING fputc.o (duplicate fputc symbol with printf.c;
#    printf.c's targets the rv906 console). Rebuilt if stale.
mkdir -p "$OUT/clib"
for f in intc printf syscalls uart vtimer; do
  if [ ! -f "$OUT/clib/$f.o" ] || [ "$M8/clib/$f.c" -nt "$OUT/clib/$f.o" ]; then
    $CC $COREMARK_CFLAGS -I"$M8/clib" -c "$M8/clib/$f.c" -o "$OUT/clib/$f.o"
  fi
done

# 3. the repro body.
$CC $COREMARK_CFLAGS -c "$HERE/repro.c" -o "$OUT/repro.o"

# 4. link: crt0 FIRST, clib (excl fputc.o), newlib.
$CC $CFLAGS -T "$M8/link_m8.ld" -nostdlib -o "$OUT/repro.elf" \
  "$OUT/crt0_m8.o" "$OUT/repro.o" \
  "$OUT/clib/intc.o" "$OUT/clib/printf.o" "$OUT/clib/syscalls.o" \
  "$OUT/clib/uart.o" "$OUT/clib/vtimer.o" \
  -lc -lgcc -lm

echo "built $OUT/repro.elf"
