#!/bin/bash
cd "$(dirname "$0")"
TB=../../bin/verisim/testbench
for name in rv64ui-v-simple rv64ui-v-add rv64ua-v-lrsc rv64um-v-div; do
  elf=build/$name.elf
  out=$(timeout 60 "$TB" --print-result "$elf" 2>&1)
  rc=$?
  if echo "$out" | grep -qi "PASS"; then
    echo "PASS  $name"
  else
    echo "FAIL  $name  rc=$rc"
    echo "$out" | tail -6 | sed 's/^/      /'
  fi
done
