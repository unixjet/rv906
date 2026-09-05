#!/bin/bash
cd "$(dirname "$0")"
TB=../../bin/verisim/testbench
pass=0; fail=0; faillist=""
for elf in build/rv64si-*.elf build/rv64mi-*.elf build/mmu_*.elf; do
  name=$(basename "$elf" .elf)
  out=$(timeout 60 "$TB" --print-result "$elf" 2>&1)
  rc=$?
  if echo "$out" | grep -qi "PASS"; then
    echo "PASS  $name"; pass=$((pass+1))
  else
    echo "FAIL  $name  rc=$rc"
    echo "$out" | tail -5 | sed 's/^/      /'
    fail=$((fail+1)); faillist="$faillist $name"
  fi
done
echo "SI+MI+MMU: PASS=$pass FAIL=$fail"
echo "FAILED:$faillist"
