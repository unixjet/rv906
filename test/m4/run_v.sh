#!/bin/bash
# M4 v-suite: 86 ELFs (rv64ui-v 54 + rv64um-v 13 + rv64ua-v 19).
cd "$(dirname "$0")"
TB=../../bin/verisim/testbench
pass=0; fail=0; faillist=""
for elf in build/rv64ui-v-*.elf build/rv64um-v-*.elf build/rv64ua-v-*.elf; do
  name=$(basename "$elf" .elf)
  out=$(timeout 120 "$TB" --print-result "$elf" 2>&1)
  rc=$?
  if echo "$out" | grep -qi "PASS"; then
    echo "PASS  $name"; pass=$((pass+1))
  else
    echo "FAIL  $name  rc=$rc"
    echo "$out" | tail -5 | sed 's/^/      /'
    fail=$((fail+1)); faillist="$faillist $name"
  fi
done
echo "V-SUITE: PASS=$pass FAIL=$fail"
echo "FAILED:$faillist"
