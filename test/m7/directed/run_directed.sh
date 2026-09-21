#!/bin/bash
# test/m7/directed/run_directed.sh -- M7 directed trigger suite runner (Task 2).
#
# Runs every test/m7/directed/build ELF through the simulator and reports
# PASS/FAIL (tohost protocol; PASS = tohost 1).
# Exit code 0 iff every test PASSes.

cd "$(dirname "$0")/../../.."   # repo root (the worktree)

BUILD_DIR="test/m7/directed/build"

shopt -s nullglob
elfs=("$BUILD_DIR"/m7-*.elf)
if [ ${#elfs[@]} -eq 0 ]; then
    echo "run_directed.sh: no ELFs in $BUILD_DIR -- build first:" >&2
    echo "  make -C test/m7/directed" >&2
    exit 2
fi

pass=0; fail=0; faillist=""
for elf in "${elfs[@]}"; do
    name=$(basename "$elf" .elf)
    if timeout 300 bin/verisim/testbench --print-result "$elf" 2>&1 | grep -q "PASS"; then
        echo "PASS  $name"; pass=$((pass+1))
    else
        echo "FAIL  $name"; fail=$((fail+1)); faillist="$faillist $name"
    fi
done

echo "M7 DIRECTED SUITE: PASS=$pass FAIL=$fail"
[ -n "$faillist" ] && echo "FAILED:$faillist"
exit "$fail"
