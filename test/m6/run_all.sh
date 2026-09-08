#!/bin/bash
# test/m6/run_all.sh -- M6 directed interrupt suite runner (Task 7).
#
# Runs every test/m6/build ELF through the simulator and reports
# PASS/FAIL (tohost protocol; PASS = tohost 1).
# Exit code 0 iff every test PASSes.

cd "$(dirname "$0")/../.."   # repo root (the worktree)

BUILD_DIR="test/m6/build"

shopt -s nullglob
elfs=("$BUILD_DIR"/m6-*.elf)
if [ ${#elfs[@]} -eq 0 ]; then
    echo "run_all.sh: no ELFs in $BUILD_DIR -- build first:" >&2
    echo "  make -C test/m6" >&2
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

echo "M6 SUITE: PASS=$pass FAIL=$fail"
[ -n "$faillist" ] && echo "FAILED:$faillist"
exit "$fail"
