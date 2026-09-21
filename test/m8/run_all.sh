#!/bin/bash
# test/m8/run_all.sh -- M8 rv906-side runner (design doc
# 2026-09-21-m8-crosscheck-design.md section 4.6).
#
# Runs every test/m8/build ELF through the Verilator testbench
# (tohost protocol; PASS = tohost 1), exactly the m6 pattern:
#   timeout <t> bin/verisim/testbench --print-result <elf> | grep -q PASS
# and writes one JSON record per case to test/m8/records/<case>.rv906.json
# ({case, side, verdict, cycles, console_excerpt, toolchain, march,
# deltas[]} -- the compare.py input). Cycles come from the harness's
# `final_cycles` line (TestBench.cpp, D-M8-6 option b).
#
# coremark gets its own (longer) timeout per design section 4.5.
#
# Exit code: number of non-PASS cases (0 = all PASS).

cd "$(dirname "$0")/../.."   # repo root (the worktree)

M8="test/m8"
BUILD_DIR="$M8/build"
RECORDS="$M8/records"
mkdir -p "$RECORDS"

TOOLCHAIN="xpack riscv-none-elf-gcc 15.2.0 (/opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin)"
MARCH="rv64imafdc_zicsr_zifencei -mabi=lp64d"

shopt -s nullglob
elfs=("$BUILD_DIR"/*.elf)
if [ ${#elfs[@]} -eq 0 ]; then
    echo "run_all.sh: no ELFs in $BUILD_DIR -- build first:" >&2
    echo "  make -C test/m8" >&2
    exit 2
fi

pass=0; fail=0; faillist=""
for elf in "${elfs[@]}"; do
    name=$(basename "$elf" .elf)
    if [ "$name" = "coremark" ]; then tmo=7200; else tmo=300; fi
    log="$RECORDS/$name.rv906.log"

    timeout "$tmo" bin/verisim/testbench --print-result "$elf" > "$log" 2>&1
    rc=$?

    if grep -q "PASS" "$log"; then
        verdict="PASS"
    elif grep -q "FAIL" "$log"; then
        verdict="FAIL"
    elif [ "$rc" -eq 124 ]; then
        verdict="TIMEOUT"
    else
        verdict="ERROR"
    fi

    cycles=$(grep -o 'final_cycles [0-9]*' "$log" | tail -1 | awk '{print $2}')
    [ -n "$cycles" ] || cycles=""

    # per-case source deltas (design section 4 ledger); glue deltas are
    # global (crt0/link/clib) and live in the design doc, not per case.
    # T3: csr has none (byte-identical body). T4/T5 fill these in.
    case "$name" in
        csr)        deltas="[]" ;;
        MMU)        deltas='["satp PPN pinned 0x40->0x81000 (root table in MEM, clear of image)","1G leaf PPN pinned 0x0->0x80000 (VA[0,1G)->PA[0x80000000,1G): covers S-mode data 0x30000->PA 0x80030000)","+1G L1[1] 0x40000,0x40000 (tohost VA 0x7FFFF000 -> PA 0x7FFFF000): rv906 image at 0x80000000 puts tohost in a 1G page the donor case (image at 0x0) never needed","+1G L1[2] 0x80000,0x80000 (S-mode code TEST1 VA 0x800002b8 -> PA 0x800002b8): same design-doc gap"]' ;;
        interrupt)  deltas='["PLICBASE_M 0x4000000000 -> 0x0C000000 (rv906 PLIC base, rtl/RVProcAXI.v:156; offsets map 1:1, rtl/PLIC.v:9-17)","source: donor INTPEND 0x1000 backdoor (writable one-shot pending, sw 0x2) -> real UART IRQ, PLIC source 7 (IER@0x10000004=2, device/uart16550.cpp:101-104 -> G_io_pins_uart_irq, rtl/RVProcAXI.v:680); rv906 pending reg is RO","reordered init (M6 pattern): UART IER assert is the LAST poke, after set_mthreshold_off -- a level source poked before threshold-off would raise meip while the store is in flight and take the trap at the store retire boundary (mepc=wfi) -> wfi would block after mret","prio[7]=4 @0x0C00001C (donor 0xa out of range: rv906 prio is 3-bit), enable bit7 @0x0C002000, DROPPED donor INTIE_HART 0x80 second-level poke (rv906 single-context: 0x2000 is the context-0 enable)","handler: IER deassert (li x13,0x10000004; li x12,0; sb x12,0(x13)) between claim and complete, replacing 3 of the 4 nops -- mandatory on rv906: THRE level stays high while IER set, so without it the interrupt re-fires after mret and the case loops in claim forever","handler stack frame: mstatus slot 0x16(x2)->0x10(x2) in stack_push/stack_pop (donor 0x16=22 is an unaligned sd on rv906, which traps misaligned at issue, LSU.v contract 3 / design 4.2 item 5; donor C906 tolerated it via mxstatus.mm; same 24-byte frame, donor -24/+16 push/pop asymmetry kept)"]' ;;
        *)          deltas="[]" ;;
    esac

    python3 - "$name" "$verdict" "$cycles" "$deltas" "$MARCH" "$TOOLCHAIN" "$log" "$RECORDS" <<'PYEOF'
import json, sys
case, verdict, cycles, deltas, march, toolchain, log, records = sys.argv[1:9]
excerpt = ""
try:
    with open(log, errors="replace") as f:
        lines = [l.rstrip("\n") for l in f if not l.startswith("cycle ")]
    excerpt = "\n".join(lines)[:2048]
except OSError:
    pass
rec = {
    "case": case,
    "side": "rv906",
    "verdict": verdict,
    "cycles": int(cycles) if cycles else None,
    "console_excerpt": excerpt,
    "toolchain": toolchain,
    "march": march,
    "deltas": json.loads(deltas),
}
out = f"{records}/{case}.rv906.json"
with open(out, "w") as f:
    json.dump(rec, f, indent=2)
PYEOF

    if [ "$verdict" = "PASS" ]; then
        echo "PASS  $name (cycles=${cycles:-?})"
        pass=$((pass+1))
    else
        echo "FAIL  $name (verdict=$verdict, cycles=${cycles:-?})"
        fail=$((fail+1)); faillist="$faillist $name"
    fi
done

echo "M8 RV906 SUITE: PASS=$pass FAIL=$fail"
[ -n "$faillist" ] && echo "FAILED:$faillist"
exit "$fail"
