#!/bin/bash
# test/m8/run_donor.sh -- M8 donor-side runner (design doc
# 2026-09-21-m8-crosscheck-design.md section 4.6; T1/T6 run notes).
#
# usage: bash test/m8/run_donor.sh <case>
#
# Runs ONE donor case through the smart_run flow under iverilog:
#   1. make buildcase CASE=<c> SIM=iverilog SHELL=/bin/bash
#      (case objects only -- the RTL design is NOT recompiled)
#   2. cd work && vvp xuantie_core.vvp
#      REUSES work/xuantie_core.vvp (57,808,961 bytes, the T1 artifact) --
#      this script REFUSES to run (and never compiles) if it is missing.
#
# Verdict: work/run_case.report (byte-exact "TEST PASS"/"TEST FAIL"; the
# tb opens it in APPEND mode, so it is removed before each run). Cycles:
# the tb's `$finish called at <simtime> (100ps)` line at 10 ns/cycle --
# cycles = simtime/100 (e.g. 273650 -> 2736.5). The full vvp transcript is
# saved to records/<case>.donor.vvp.log; the JSON record
# (records/<case>.donor.json) mirrors the rv906-side shape for compare.py.
#
# Runtime requirements (T1): SHELL=/bin/bash on every make invocation
# (dash rejects smart_cfg.mk's `>&` redirections); CODE_BASE_PATH and
# TOOL_EXTENSION exported.
#
# Exit code: 0 = TEST PASS, 1 = TEST FAIL / build or sim error, 2 = usage.

set -u
CASE="${1:-}"
if [ -z "$CASE" ]; then
    echo "usage: bash test/m8/run_donor.sh <case>" >&2
    exit 2
fi

# The donor tree lives in the MAIN checkout (it is untracked, so it is not
# part of this git worktree). Override with M8_DONOR_ROOT if it moves.
MAIN_ROOT="${M8_DONOR_ROOT:-/home/vlsilab/zhouz/workspace/C2RTL/rvproc/RVProc6/vla/riscv/rv906}"
DONOR="$MAIN_ROOT/refs/openc906/smart_run"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
RECORDS="$REPO_ROOT/test/m8/records"
mkdir -p "$RECORDS"

VVP="$DONOR/work/xuantie_core.vvp"
if [ ! -f "$VVP" ]; then
    echo "run_donor.sh: $VVP is missing." >&2
    echo "refusing to recompile the RTL (T3 rule: the vvp is reused, never rebuilt)" >&2
    exit 2
fi
vvp_size=$(stat -c %s "$VVP")

export CODE_BASE_PATH="$MAIN_ROOT/refs/openc906/C906_RTL_FACTORY"
export TOOL_EXTENSION=/opt/riscv/bin
export SHELL=/bin/bash

TOOLCHAIN="riscv64-unknown-elf-gcc 15.1.0 (/opt/riscv/bin) + iverilog 12"
MARCH="rv64imafdc_zicsr_zifencei -mabi=lp64d (T1-patched donor string)"

# per-case source deltas on the DONOR side (md5-ledgered in the design
# doc section 9 / T1+T6 notes). T3: csr has none.
case "$CASE" in
    csr)        deltas="[]" ;;
    MMU)        deltas='["mxstatus -> 0x7c0 (6 sites, T6 patch 1)"]' ;;
    exception)  deltas='["dcache.ciall commented (T6 patch 2)", "mhcr -> 0x7c1 (T6 patch 2)"]' ;;
    coremark)   deltas='["4 vendor -m* flags dropped (T6 patch 3)", "-mtune=c906 dropped (patch 4)"]' ;;
    *)          deltas="[]" ;;
esac

write_record() {  # $1=verdict $2=cycles $3=raw_report $4=build_ok $5=log
    python3 - "$CASE" "$1" "$2" "$3" "$4" "$5" "$TOOLCHAIN" "$MARCH" "$deltas" "$RECORDS" <<'PYEOF'
import json, sys
case, verdict, cycles, raw, build_ok, vvp_log, toolchain, march, deltas, records = sys.argv[1:11]
excerpt = ""
try:
    with open(vvp_log, errors="replace") as f:
        lines = [l.rstrip("\n") for l in f if not l.startswith("cycle ")]
    excerpt = "\n".join(lines)[:2048]
except OSError:
    pass
rec = {
    "case": case,
    "side": "donor",
    "verdict": verdict,
    "cycles": float(cycles) if cycles not in ("", None) else None,
    "console_excerpt": excerpt,
    "raw_report": raw,
    "build_ok": build_ok,
    "toolchain": toolchain,
    "march": march,
    "deltas": json.loads(deltas),
}
out = f"{records}/{case}.donor.json"
with open(out, "w") as f:
    json.dump(rec, f, indent=2)
PYEOF
}

cd "$DONOR" || exit 2

echo "== buildcase CASE=$CASE (vvp reused, size $vvp_size) =="
make buildcase CASE="$CASE" SIM=iverilog SHELL=/bin/bash
build_rc=$?
if [ $build_rc -ne 0 ]; then
    echo "run_donor.sh: buildcase failed (rc=$build_rc); see work/${CASE}_build.case.log" >&2
    write_record "BUILD_FAIL" "" "" "false" "$RECORDS/$CASE.donor.vvp.log"
    exit 1
fi
if [ -s "work/${CASE}_build.case.log" ]; then
    echo "run_donor.sh: WARNING -- non-empty build log:" >&2
    cat "work/${CASE}_build.case.log" >&2
fi

cd "$DONOR/work" || exit 2
rm -f run_case.report   # tb opens it in APPEND mode -- start clean

echo "== vvp xuantie_core.vvp =="
vvp xuantie_core.vvp > "$RECORDS/$CASE.donor.vvp.log" 2>&1
vvp_rc=$?
[ $vvp_rc -ne 0 ] && echo "run_donor.sh: vvp exited rc=$vvp_rc" >&2

raw_report=$(tr -d '\n' < run_case.report 2>/dev/null)
case "$raw_report" in
    "TEST PASS") verdict="PASS" ;;
    "TEST FAIL") verdict="FAIL" ;;
    "")          verdict="NO_REPORT" ;;
    *)           verdict="UNKNOWN:$raw_report" ;;
esac

simtime=$(grep -o '\$finish called at [0-9]*' "$RECORDS/$CASE.donor.vvp.log" | tail -1 | grep -o '[0-9]*$')
if [ -n "$simtime" ]; then
    cycles=$(awk -v t="$simtime" 'BEGIN{printf "%.1f", t/100}')
else
    cycles=""
fi

write_record "$verdict" "$cycles" "$raw_report" "true" "$RECORDS/$CASE.donor.vvp.log"

echo "$CASE (donor): $raw_report${cycles:+  cycles=$cycles}"
[ "$verdict" = "PASS" ] && exit 0 || exit 1
