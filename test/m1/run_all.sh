#!/usr/bin/env bash
# run_all.sh - M1 regression gate (plan Tasks 6.2, 10.1)
#
# Two modes:
#
#   test/m1/run_all.sh                  # single-rung mode: rung 1 (default)
#   test/m1/run_all.sh --m1-rung=N      # single-rung mode: override the rung
#                                        # (bring-up loop, Tasks 6-9's own gate)
#
#   test/m1/run_all.sh --full-matrix    # THE M1 ACCEPTANCE MATRIX (Task 10.1):
#
#       11 tests x 4 rungs x --sink-stall {off,on}                =  88 runs
#       11 tests x rung 4 x --sink-stall {off,on} x --inv-test    =  22 runs
#                                                                  -----------
#                                                                    110 runs
#
#     The rungs walk the predictor chicken-bit ladder (1 = all predictors
#     off, 2 = +RAS, 3 = +BTB, 4 = +BHT -- design doc S4.3). Every test is
#     run twice per rung, with and without --sink-stall (FetchSink's
#     pseudo-random id_stall mode), because the stall pattern changes which
#     instructions share a commit group without changing which instructions
#     commit. The --inv-test pass pulses the bht/btb invalidate bits mid-run
#     at rung 4, the only rung where both invalidatable arrays hold live
#     state.
#
#     THE INVARIANT THIS MODE ASSERTS (design doc S4.1 / plan Task 10.1):
#     predictors change WHEN an instruction is fetched, never WHICH
#     instructions commit. So for a given test, the "N instructions
#     compared" count the online checker reports (parsed from RVProcTest.cpp's
#     own `[checker] <N> instructions compared ...` line) must be IDENTICAL
#     across every one of that test's 10 runs (4 rungs x 2 stall modes, plus
#     the rung-4 --inv-test pair, which shares rung 4's expected count). A
#     differing slot count is treated as a FAILURE (nonzero exit, printed
#     loudly) even when every individual run says PASS -- this is the online
#     checker's own predictor-agnostic design promise, made an explicit,
#     automated assertion here (same invariant rv12's own run_all.sh asserts
#     for C910's 6-rung ladder).
#
# fencei.S needs one extra, hand-derived flag in BOTH modes (see that file's
# own header comment for how <addr>/<word32>/<commit> were computed from its
# real disassembly): --fencei-patch=0x80000004:0x1FC0006F:27. Every other
# test runs with no extra flags.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TB="$ROOT/bin/verisim/testbench"
TESTDIR="$ROOT/test/m1"

FENCEI_FLAG="--fencei-patch=0x80000004:0x1FC0006F:27"

TESTS="seq rvc_mix jal_chain callret ind_jr dense_br thrash uncached fencei mixed iss_selftest"

if [ ! -x "$TB" ]; then
    echo "run_all.sh: $TB not found or not executable -- run 'make verisim' first" >&2
    exit 2
fi

pass=0
fail=0
failed_names=""

extra_flags_for() {
    case "$1" in
    fencei) echo "$FENCEI_FLAG" ;;
    *)      echo "" ;;
    esac
}

#-----------------------------------------------------------------------------
# Single-rung mode (Tasks 6.2-9.4's own bring-up gate) -- unchanged interface.
#-----------------------------------------------------------------------------
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

if [ "${1:-}" != "--full-matrix" ]; then
    RUNG="${1:---m1-rung=1}"

    echo "=== M1 single-rung regression ($RUNG) ==="
    for t in $TESTS; do
        extra="$(extra_flags_for "$t")"
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
fi

#-----------------------------------------------------------------------------
# --full-matrix mode (Task 10.1): every test x every rung x --sink-stall
# {off,on}, plus --inv-test at rung 4, with the slot-count invariant check.
#-----------------------------------------------------------------------------
RESULTS_FILE="$(mktemp "${TMPDIR:-/tmp}/rv906-m1-matrix.XXXXXX")" || exit 2
trap 'rm -f "$RESULTS_FILE"' EXIT INT TERM

# run_matrix_one <test> <rung> <stall-label> <stall-flag> <extra-flags> <tag>
#   tag is the column label used in both the per-run print and the results
#   file ("1".."4" for the four plain rungs, "4i" for the rung-4 --inv-test
#   pass), so the summary can print a test x rung table and so the
#   slot-count check can tell "rung 4" and "rung 4 + --inv-test" runs apart
#   while still cross-checking them against the same expected count.
run_matrix_one() {
    local t="$1" rung="$2" stall_label="$3" stall_flag="$4" extra="$5" tag="$6"
    local out="$TESTDIR/$t.out"
    if [ ! -f "$out" ]; then
        echo "SKIP  $t (rung $tag, $stall_label): $out not built (run 'make -C test/m1' first)"
        fail=$((fail + 1))
        failed_names="$failed_names $t:$tag:$stall_label(missing)"
        return
    fi
    local log
    log="$(cd "$ROOT" && timeout 60 "$TB" --print-result "--m1-rung=$rung" $stall_flag $extra "$out" 2>&1)"
    local slots cycles
    slots="$(printf '%s\n' "$log" | sed -n 's/\[checker\] \([0-9]*\) instructions compared.*/\1/p')"
    cycles="$(printf '%s\n' "$log" | sed -n 's/.* in \([0-9]*\) cycles,.*/\1/p')"
    if printf '%s\n' "$log" | grep -q ": PASS\.$"; then
        printf 'PASS  %-14s rung=%-3s %-11s slots=%-7s cycles=%s\n' \
            "$t" "$tag" "$stall_label" "${slots:--}" "${cycles:--}"
        pass=$((pass + 1))
        printf '%s %s %s %s %s\n' "$t" "$tag" "$stall_label" "${slots:--}" "${cycles:--}" >> "$RESULTS_FILE"
    else
        echo "FAIL  $t (rung $tag, $stall_label)"
        printf '%s\n' "$log" | tail -20 | sed 's/^/      /'
        fail=$((fail + 1))
        failed_names="$failed_names $t:$tag:$stall_label"
    fi
}

echo "=== M1 FULL MATRIX (plan Task 10.1): 11 tests x 4 rungs x --sink-stall{off,on} + rung-4 --inv-test ==="
for rung in 1 2 3 4; do
    printf '\n--- rung %s ---\n' "$rung"
    for t in $TESTS; do
        extra="$(extra_flags_for "$t")"
        run_matrix_one "$t" "$rung" "off" "" "$extra" "$rung"
        run_matrix_one "$t" "$rung" "on"  "--sink-stall" "$extra" "$rung"
    done
done

printf '\n--- rung 4 + --inv-test ---\n'
for t in $TESTS; do
    extra="$(extra_flags_for "$t")"
    run_matrix_one "$t" 4 "off" "--inv-test"                "$extra" "4i"
    run_matrix_one "$t" 4 "on"  "--sink-stall --inv-test"   "$extra" "4i"
done

echo ""
echo "==============================================="
echo "PASS: $pass   FAIL: $fail"
if [ "$fail" -ne 0 ]; then
    echo "Failed:$failed_names"
fi

#-----------------------------------------------------------------------------
# SUMMARY: cycles table (test x rung, --sink-stall off) + slot-count invariant
#-----------------------------------------------------------------------------
awk -v tests="$TESTS" '
{
    t=$1; tag=$2; stall=$3; slots=$4; cyc=$5;
    if (stall == "off") cycoff[t SUBSEP tag] = cyc;
    if (slots != "-") {
        if (!(t in slotref)) { slotref[t] = slots }
        else if (slotref[t] != slots) { slotbad[t] = slotbad[t] " " tag ":" slots }
    }
}
END {
    nt = split(tests, T, " ");
    cols[1]="1"; cols[2]="2"; cols[3]="3"; cols[4]="4"; cols[5]="4i"; nc=5;

    printf "\n================ SUMMARY: cycles, --sink-stall off ================\n";
    printf "%-14s", "test";
    for (i=1; i<=nc; i++) printf "%9s", "r" cols[i];
    printf "%9s\n", "slots";
    for (j=1; j<=nt; j++) {
        t=T[j];
        printf "%-14s", t;
        for (i=1; i<=nc; i++) {
            v = ((t SUBSEP cols[i]) in cycoff) ? cycoff[t SUBSEP cols[i]] : "-";
            printf "%9s", v;
        }
        printf "%9s\n", (t in slotref) ? slotref[t] : "-";
    }

    printf "\nslot-count invariant (must be identical across every run of a test): ";
    nbad = 0;
    for (t in slotbad) nbad++;
    if (nbad == 0) print "OK for all tests";
    else {
        print "MISMATCH";
        for (t in slotbad) printf "  %-14s ref=%s mismatches:%s\n", t, slotref[t], slotbad[t];
    }
    exit (nbad > 0) ? 1 : 0;
}
' "$RESULTS_FILE"
invariant_rc=$?

echo ""
if [ "$fail" -eq 0 ] && [ "$invariant_rc" -eq 0 ]; then
    echo "ALL PASS ($pass runs), slot-count invariant holds for all 11 tests"
    exit 0
else
    [ "$fail" -ne 0 ] && echo "FAILURES PRESENT ($fail run(s) failed)"
    [ "$invariant_rc" -ne 0 ] && echo "SLOT-COUNT INVARIANT VIOLATED"
    exit 1
fi
