#!/bin/bash
# test/m7/run_debug.sh -- M7 Task 6 JTAG DMI smoke runner.
#
# Runs a short existing ELF (an m6 suite test by default -- the core just
# needs to be running; the smoke's dmactive set/clear does not halt it)
# through the simulator with --m7-debug and gates on M7-DEBUG-PASS
# (JTAG TLR, IDCODE 0x10000B6F, dtmcs version/abits, dmactive=1,
# dmstatus.version==2 + anyhalted==0, dmactive=0).
#
# Usage: bash test/m7/run_debug.sh [elf]

cd "$(dirname "$0")/../.."   # repo root (the worktree)

ELF="${1:-test/m6/build/m6-msip.elf}"
if [ ! -f "$ELF" ]; then
    echo "run_debug.sh: no such ELF: $ELF" >&2
    echo "  build the m6 suite first: make -C test/m6" >&2
    exit 2
fi
if [ ! -x bin/verisim/testbench ]; then
    echo "run_debug.sh: bin/verisim/testbench missing -- build first: make verisim" >&2
    exit 2
fi

LOG=/tmp/m7_debug_run.log
# NOTE: the testbench exit code is the tohost word (1 on a PASSing ELF
# run), not a 0/1 status -- gate on the M7-DEBUG-PASS marker, same
# convention as test/m6/run_all.sh gating on the tohost "PASS" line.
timeout 300 bin/verisim/testbench --m7-debug "$ELF" > "$LOG" 2>&1
rc=$?

grep "^\[m7\]\|^M7-DEBUG" "$LOG"

if grep -q "M7-DEBUG-PASS" "$LOG"; then
    echo "M7 DEBUG SUITE: PASS ($ELF)"
    exit 0
else
    echo "M7 DEBUG SUITE: FAIL ($ELF, sim exit=$rc); last lines of $LOG:"
    tail -15 "$LOG"
    exit 1
fi
