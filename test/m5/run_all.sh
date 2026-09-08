#!/bin/bash
# test/m5/run_all.sh -- M5 FP compliance suite runner (Task 11 acceptance).
#
# Runs all 46 test/m5/build ELFs (rv64uf-p/rv64ud-p/rv64uf-v/rv64ud-v,
# Task 10) through the full-stack simulator and reports PASS/FAIL, same
# tohost-via---print-result protocol as test/m2/run_all.sh.
#
# Usage: bash test/m5/run_all.sh
# Exit code 0 iff every test PASSes.

cd "$(dirname "$0")/../.."   # repo root (the worktree)

BUILD_DIR="test/m5/build"

if [ ! -d "$BUILD_DIR" ]; then
    echo "run_all.sh: no $BUILD_DIR -- build it first: make -C test/m5" >&2
    exit 2
fi

pass=0; fail=0; faillist=""
for elf in "$BUILD_DIR"/rv64uf-*.elf "$BUILD_DIR"/rv64ud-*.elf; do
    name=$(basename "$elf" .elf)
    if timeout 120 bin/verisim/testbench --print-result "$elf" 2>&1 | grep -q "PASS"; then
        echo "PASS  $name"; pass=$((pass+1))
    else
        echo "FAIL  $name"; fail=$((fail+1)); faillist="$faillist $name"
    fi
done

echo "M5 SUITE: PASS=$pass FAIL=$fail"
[ -n "$faillist" ] && echo "FAILED:$faillist"
exit "$fail"
