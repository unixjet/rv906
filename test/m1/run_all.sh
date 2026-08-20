#!/usr/bin/env bash
# run_all.sh - M1 rung-1 regression gate (plan Task 6.2)
#
# Runs every test/m1/*.S build (both plain and --sink-stall) through the
# full-stack simulator at --m1-rung=1 and prints a clean pass/fail summary.
# This is the standing regression gate for Tasks 7-9: once BTB/BHT/RAS land,
# the SAME suite gets re-run at every rung the plan's chicken-bit ladder adds
# (design doc S4.3) -- nothing here is rung-1-specific except the CLI flag
# below, which a caller can override.
#
# Usage:
#   test/m1/run_all.sh                  # rung 1 (default), from anywhere
#   test/m1/run_all.sh --m1-rung=2       # override the rung
#
# fencei.S needs one extra, hand-derived flag (see that file's own header
# comment for how <addr>/<word32>/<commit> were computed from its real
# disassembly): --fencei-patch=0x80000004:0x1FC0006F:27. Every other test
# runs with no extra flags.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TB="$ROOT/bin/verisim/testbench"
TESTDIR="$ROOT/test/m1"

RUNG="${1:---m1-rung=1}"

FENCEI_FLAG="--fencei-patch=0x80000004:0x1FC0006F:27"

TESTS="seq rvc_mix jal_chain callret ind_jr dense_br thrash uncached fencei mixed iss_selftest"

if [ ! -x "$TB" ]; then
    echo "run_all.sh: $TB not found or not executable -- run 'make verisim' first" >&2
    exit 2
fi

pass=0
fail=0
failed_names=""

run_one() {
    local name="$1" mode_flag="$2" mode_label="$3" extra="$4"
    local out="$TESTDIR/$name.out"
    if [ ! -f "$out" ]; then
        echo "SKIP  $name ($mode_label): $out not built (run 'make -C test/m1' first)"
        fail=$((fail + 1))
        failed_names="$failed_names $name:$mode_label(missing)"
        return
    fi
    local log
    log="$(cd "$ROOT" && timeout 60 "$TB" --print-result "$RUNG" $mode_flag $extra "$out" 2>&1)"
    if echo "$log" | grep -q "^$out: PASS\.$" || echo "$log" | grep -q ": PASS\.$"; then
        echo "PASS  $name ($mode_label)"
        pass=$((pass + 1))
    else
        echo "FAIL  $name ($mode_label)"
        echo "$log" | tail -20 | sed 's/^/      /'
        fail=$((fail + 1))
        failed_names="$failed_names $name:$mode_label"
    fi
}

echo "=== M1 rung-1 regression ($RUNG) ==="
for t in $TESTS; do
    extra=""
    if [ "$t" = "fencei" ]; then
        extra="$FENCEI_FLAG"
    fi
    run_one "$t" "" "plain" "$extra"
    run_one "$t" "--sink-stall" "sink-stall" "$extra"
done

echo "==============================================="
echo "PASS: $pass   FAIL: $fail"
if [ "$fail" -ne 0 ]; then
    echo "Failed:$failed_names"
    exit 1
fi
exit 0
