#!/bin/bash
# test/m2/run_all.sh -- M2 riscv-tests full-matrix runner (Task 10.1).
#
# Runs every test/m2/build ELF through the simulator and reports PASS/FAIL.
# The acceptance gate is the caches-on build (the default test/m2/build).
# Usage:
#   bash test/m2/run_all.sh                 # caches-on acceptance sweep
#   bash test/m2/run_all.sh build_nocache   # caches-off sanity cross-check
#
# Exit code 0 iff every test PASSes.

cd "$(dirname "$0")/../.."   # repo root (the worktree)

BUILD_SUBDIR="${1:-build}"
BUILD_DIR="test/m2/${BUILD_SUBDIR}"

if [ ! -d "$BUILD_DIR" ]; then
    echo "run_all.sh: no $BUILD_DIR -- build it first:" >&2
    echo "  make -C test/m2   (caches on)" >&2
    echo "  make -C test/m2 RV906_BOOT_MHCR=0 BUILD_DIR=\$PWD/test/m2/$BUILD_SUBDIR   (caches off)" >&2
    exit 2
fi

pass=0; fail=0; faillist=""
for elf in "$BUILD_DIR"/rv64u*.elf; do
    name=$(basename "$elf" .elf)
    if timeout 90 bin/verisim/testbench --print-result "$elf" 2>&1 | grep -q "PASS"; then
        echo "PASS  $name"; pass=$((pass+1))
    else
        echo "FAIL  $name"; fail=$((fail+1)); faillist="$faillist $name"
    fi
done

echo "SUITE (${BUILD_SUBDIR}): PASS=$pass FAIL=$fail"
[ -n "$faillist" ] && echo "FAILED:$faillist"
exit "$fail"
