#!/bin/bash
# M8 T4c directed repro runner -- BPU predicted-taken-branch / valid+correct
# BTB-hit fetch-redirect (fixed at rtl/BPU.v:824). See repro.c header.
#
# Verdict (tohost protocol, same convention as test/m8/run_all.sh):
#   PASS    = tohost=1 (the program ran to completion)          -> exit 0
#   TIMEOUT = the run hung (tohost never written)                -> FAIL, exit 1
#   FAIL    = a tohost trap/failure value                        -> FAIL, exit 1
#
# On the FIXED RTL this PASSES in ~7k cycles (printf %08x is fast). If
# BPU.v:824 regresses to the verbatim donor formula, the fetch desyncs and
# the run HANGS -> the timeout below fires -> FAIL. So a green run.sh is the
# regression guard for the BPU.v:824 fix. Timeout is generous (120s): well
# beyond the ~7k-cycle post-fix run, yet bounded so a hang cannot stall CI.

set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
ELF="$HERE/build/repro.elf"
[ -f "$ELF" ] || bash "$HERE/build.sh"

TB="$REPO/bin/verisim/testbench"
[ -x "$TB" ] || { echo "ERROR: $TB missing -- run 'make verisim' first"; exit 2; }

log="$HERE/build/repro.log"
timeout 120 "$TB" --print-result "$ELF" > "$log" 2>&1
rc=$?

if grep -q "PASS" "$log"; then
  cycles=$(grep -o 'final_cycles [0-9]*' "$log" | tail -1 | awk '{print $2}')
  echo "PASS  repro_btb_pred (cycles=${cycles:-?})"
  exit 0
elif grep -q "FAIL" "$log"; then
  echo "FAIL  repro_btb_pred (verdict=FAIL)"
  exit 1
elif [ "$rc" -eq 124 ]; then
  echo "FAIL  repro_btb_pred (verdict=TIMEOUT: hang -- BPU.v:824 regression?)"
  exit 1
else
  echo "FAIL  repro_btb_pred (verdict=ERROR rc=$rc)"
  exit 1
fi
