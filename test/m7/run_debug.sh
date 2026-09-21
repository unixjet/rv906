#!/bin/bash
# test/m7/run_debug.sh -- M7 Task 7 debug e2e runner.
#
# Runs the spin ELF (test/m7/directed/debug_spin.S -- an infinite,
# RVC-free loop at a known PC with known pre-loop register values) through
# the simulator with --m7-debug and gates on M7-DEBUG-PASS, which now
# covers the FULL extended sequence:
#   base 6 steps (JTAG TLR, IDCODE 0x10000B6F, dtmcs version/abits,
#                dmactive set/clear, dmstatus.version==2)
#   extended   (halt + dpc/dcsr; abstract GPR+CSR r/w; cmderr 4+2; ITR;
#                progbuf raw + memory r/w; single-step; resume; dret)
# The spin ELF never terminates, so the testbench exits (0/1) right after
# the smoke; the M7-DEBUG-PASS marker gates everything.
#
# Usage: bash test/m7/run_debug.sh [elf]

cd "$(dirname "$0")/../.."   # repo root (the worktree)

ELF="${1:-test/m7/directed/build/m7-debug_spin.elf}"
if [ ! -f "$ELF" ]; then
    echo "run_debug.sh: no such ELF: $ELF" >&2
    echo "  build the spin ELF first: make -C test/m7/directed" >&2
    exit 2
fi
if [ ! -x bin/verisim/testbench ]; then
    echo "run_debug.sh: bin/verisim/testbench missing -- build first: make verisim" >&2
    exit 2
fi

LOG=/tmp/m7_debug_run.log
# The testbench exits 0 on M7-DEBUG-PASS / 1 on FAIL (the spin ELF never
# terminates, so the harness exits right after the smoke). We still gate on
# the M7-DEBUG-PASS marker (most robust) and report the sim exit code.
timeout 300 bin/verisim/testbench --m7-debug "$ELF" > "$LOG" 2>&1
rc=$?

grep "^\[m7\]\|^M7-DEBUG" "$LOG"

if grep -q "M7-DEBUG-PASS" "$LOG"; then
    echo "M7 DEBUG SUITE: PASS ($ELF, sim exit=$rc)"
    exit 0
else
    echo "M7 DEBUG SUITE: FAIL ($ELF, sim exit=$rc); last lines of $LOG:"
    tail -25 "$LOG"
    exit 1
fi
