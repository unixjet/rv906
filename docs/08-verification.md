// 8 Verification gates (M4 complete)

## §8.14 M3b full LSU acceptance (§7.4 "Complete LSU")

After implementing the full non-blocking LSU (Tasks A–G: LFB rework, RDL + VB decoupling, PFB stride prefetch + MHINT, AMR streaming-store disabler), verify against the following gates:

### Gate chain (must pass all):

```bash
# 1. Clean Verilog build
make verisim 2>&1 | grep -icE "%error|error:"    # should be 0

# 2. Unit suite (lsu_tb.cpp T1–T13)
make -C test/m2/unit run                          # UNIT-SUITE-PASS

# 3. Full regression sweep (rv64ui-p / rv64um-p / rv64uc-p / rv64ua-p)
bash test/m2/run_all.sh                           # 86/87 (only documented rv64ui-p-ma_data)

# 4. Atomic operation tests
bash /tmp/run_atomic.sh                           # PASS=19 FAIL=0
```

### Expected results (green = pass, red = known failure):

| Test suite               | Pass   | Fail   | Notes                               |
|--------------------------|--------|--------|-------------------------------------|
| `make verisim`           | 0 errors | —    | No new Verilator warnings           |
| Unit suite (T1–T13)      | PASS   | —      | 13 directed tests covering all paths|
| Sweep (build)            | 86     | 1      | `rv64ui-p-ma_data` documented       |
| RV64U atomic (lrsc + 18) | 19     | 0      | All atomic ops correct              |

### Directed-test coverage (lsu_tb.cpp):

| Test | Scenario                                          | Block(s) validated                      |
|------|---------------------------------------------------|-----------------------------------------|
| T5   | Single-outstanding miss + dirty-victim writeback  | FRZ path, victim-select                 |
| T6   | Hit-under-miss (HIT load during MISS refill)      | LFB deferral, hit path                  |
| T7   | Multi-miss tracking (two misses)                  | LFB multi-entry tracking                |
| T8   | Miss-to-in-flight-line replay                     | LFB addr_hit, replay logic              |
| T9   | LFB-full backpressure                             | Backpressure on full LFB                |
| T10  | VB decoupled dirty-writeback (refill→writeback)   | VB handoff, refilling before eviction   |
| T11  | PFB stride prefetch ahead + hit                   | PC-stride training, lookahead, hitpath  |
| T12  | AMR wa disable after stride training              | AMR FSM FUNC state, gate dcache_wa      |
| T13  | STB forward under miss                            | Store parks, retries, drains to cache   |

### Gate-level behavior verification:

1. **LFB 8-entry non-blocking:** Deferred loads do NOT block the main pipe; hits complete while a miss is in flight. Multiple misses tracked, completion order matches issue order (bus serialization).
2. **Dirty-victim decoupling:** When a demand miss evicts a dirty victim, the refill read happens BEFORE the writeback (not blocked on it); fence waits for undrained VB writeback (`!vb_vld`).
3. **Prefetch:** With pref_en=1, strided stores train the PFB which prefetches lines ahead of demand; prefetched lines hit. With pref_en=0, no prefetch traffic.
4. **AMR write-allocate disable:** After `N` line-completion events (threshold from MHINT.amr), subsequent store misses bypass allocation and write directly through.
5. **STB consolidation:** Stores that replay behind a deferred load retry after drain, land in STB, and forward data to subsequent loads (no redundant cache writes).

### Known adaptation differences (documented):

- Bus serialized via single-port AXI (donor would support parallel refs on multi-port memory); this does NOT change functional behavior but simplifies implementation.
- 512-bit single-beat refill (vs donor's 4×128b WRAP); critical-word-first degenerates into whole-line return.
- PIPT D-cache (single-bank, no aliasing); VB only handles dirty-victim writeback, not alias read-back.

---

**Verification log (this session):** `git status` shows clean working tree after committing all Task F changes. All gates verified green in the preceding commands. M3b fully implemented and tested.

## §8.15 M4 privilege + MMU + PMP acceptance (§7.4 "privilege complete + MMU + PMP + counter CSRs")

After implementing the privilege machine (M/S/U + delegation + counters),
the 128-entry Sv39 MMU + hardware PTW, and the 8-entry PMP (Tasks 0–9),
verify against the following two-sided gate chain: the pre-M4 battery
must stay bit-identical MMU-off (the OFF path, G1), and the new
privilege/VM suites must pass MMU-on (the ON path).

### Gate chain (must pass all):

```bash
# 1. Clean Verilog build
make verisim 2>&1 | grep -icE "%error|error:"        # should be 0

# 2. Unit suite (csr_tb/mmu_tb/lsu_tb privilege + TLB + PTW + servant rows)
make -C test/m2/unit run                              # UNIT-SUITE-PASS

# --- OFF path (G1): identical to the pre-M4 baseline, satp=0 reset ---
# 3. Full regression sweep
bash test/m2/run_all.sh                               # 86/87 (only documented rv64ui-p-ma_data)
# 4. Atomic operation tests
bash /tmp/run_atomic.sh                               # PASS=19 FAIL=0
# 5. Full v-suite (identity-mapped, MMU-off)
bash /tmp/run_vsuite_full.sh                          # 85/86 (only documented rv64ui-v-ma_data)

# --- ON path: real Sv39 translation, satp=Sv39 ---
# 6. Directed si/mi/mmu suite
bash test/m4/run_directed.sh                          # SI+MI+MMU: PASS=30 FAIL=0
# 7. Full v-suite under real translation
#   (iterate build/rv64ui-v-*.elf build/rv64um-v-*.elf build/rv64ua-v-*.elf
#    through bin/verisim/testbench --print-result, same runner shape as
#    run_directed.sh)
```

### Expected results (green = pass, red = known failure):

| Test suite                          | Pass | Fail | Notes                                    |
|--------------------------------------|------|------|-------------------------------------------|
| `make verisim`                       | 0 errors | — | No new Verilator warnings                |
| Unit suite                           | PASS | —    | csr_tb/mmu_tb/lsu_tb privilege+TLB+PTW rows |
| Sweep (OFF path)                     | 86   | 1    | `rv64ui-p-ma_data` documented              |
| RV64U atomic (OFF path)              | 19   | 0    | Bit-identical to pre-M4 baseline           |
| v-suite (OFF path, MMU-off)          | 85   | 1    | `rv64ui-v-ma_data` documented              |
| rv64si-p (ON path)                   | 7    | 0    | csr/dirty/icache-alias/ma_fetch/scall/sbreak/wfi |
| rv64mi-p (ON path)                   | 17   | 0    | incl. pmpaddr, instret_overflow, breakpoint (no-triggers escape) |
| Directed mmu set (ON path)           | 6    | 0    | ptwalk/pmp/pmpstore/misalign/ldst_samepage/amo |
| v-suite (ON path, real Sv39)         | 85   | 1    | Same `rv64ui-v-ma_data` deviation under translation |

### Directed-test coverage (M4-specific):

| Test / row                        | Scenario                                                        |
|------------------------------------|-------------------------------------------------------------------|
| mmu_ptwalk                         | 3-level walk incl. superpage leaves, PTE-in-D$ coherence          |
| mmu_pmp / mmu_pmpstore              | PMP deny on fetch/load/store; STB/LFB drain race (found here)     |
| mmu_misalign                       | Misaligned access under active translation                        |
| mmu_ldst_samepage                  | Load+store to the same translated page, no false miss             |
| mmu_amo                            | AMO under Sv39 (found the double-dispatch race, see LSU §M4)      |
| rv64si-p-dirty                     | Software A/D (no HW update); D-M4-1 store/AMO/SC-fault-on-D=0     |
| rv64si-p-wfi / rv64mi-p-illegal    | TW/TVM/TSR trap arms; WFI real since M6 Task 4 (D-M4-7 discharged)  |
| rv64mi-p-pmpaddr                   | PMP WARL/NAPOT-readback conformance (no deviation needed, N §…)   |
| rv64mi-p-breakpoint                | No-triggers escape (real tselect/tdata1/tdata2/tcontrol, D-M4-5)  |
| rv64mi-p-instret_overflow          | Write-suppresses-increment minstret semantics                     |

### Known deviations (documented, full ledger in the M4 design doc §2.3):

- D-M4-1: D-bit check enabled for stores/AMO/SC (donor has it commented out).
- D-M4-2: MAEE dropped — PMA always from rv906's own sysmap-style table.
- D-M4-4/D11: single 128-entry fully-associative flop TLB (no uTLB/jTLB split).
- D-M4-5/D12: T-Head SMIR/SMEL/SMEH/SMCIR + tlbp/tlbr/tlbwi/tlbwr dropped.
- D-M4-6: sfence.vma over-invalidates (whole TLB for every flavor).
- D-M4-7: DISCHARGED at M6 Task 4 — WFI is a real hold-until-wake on
  (mip & mie) with no MIE/SIE/priv gate (M6 doc D-M6-2).
- D-M4-8: NA4 stays dead (donor ties it 0 too).
- D-M4-9: DISCHARGED at M6 Task 3 — `time` (0xC01) is a live CLINT
  mtime mirror.
- D-M4-11: "THE SWAP" (Task 8) closed by construction — CSR.v's privilege
  decode was live from Task 1 onward, so G1 was satisfied continuously by
  every task's gate run rather than by one flip commit.

**Verification log (M4 session):** working tree clean after the Task
0–9 commits; the full gate chain above (unit suite, 86/87 sweep, 19/19
atomics, 85/86 OFF-path v-suite, 30/30 directed si+mi+mmu, 85/86 ON-path
v-suite under real Sv39 translation) was re-run and confirmed green from
a clean rebuild, bit-identical to the pre-M4 baseline on the OFF path.
M4 fully implemented and tested; §7.4's M4 acceptance criteria
("rv64si/mi + vm tests pass") are met.

---

## §8.16 M5 scalar FPU acceptance (§7.4 "scalar FPU (F/D)")

After the FPU cluster (`rtl/FPU.v`, new; see `docs/09-fpu.md`) plus the
IDU/RTU/CSR integration and the Task 11 bug-fix round, verify against:

### Gate chain (must pass all):

```bash
# 1. Clean Verilog build
make verisim 2>&1 | grep -icE "%error|error:"    # should be 0

# 2. Unit suite (13 benches incl. fpu_tb.cpp T1–T22)
make -C test/m2/unit run                          # UNIT-SUITE-PASS

# 3. M5 FP compliance suite (46 ELFs: uf/ud x p/v x 10 test bodies + structural)
bash test/m5/run_all.sh                           # 46/46

# 4. M2/M3 integer regression (FPU-off path must be bit-identical)
bash test/m2/run_all.sh                           # 86/87 (documented rv64ui-p-ma_data)

# 5. Atomic operations
bash /tmp/run_atomic.sh                           # PASS=19 FAIL=0
```

### Expected results:

| Test suite                  | Pass | Fail | Notes                              |
|-----------------------------|------|------|------------------------------------|
| `make verisim`              | 0    | —    | No new Verilator warnings          |
| Unit suite (13 benches)     | PASS | —    | fpu_tb T1–T22 incl. FMAU oracle    |
| M5 FP suite (46 ELFs)       | 46   | 0    | uf/ud, p+v envs                    |
| M2/M3 sweep                 | 86   | 1    | `rv64ui-p-ma_data` (pre-existing)  |
| RV64U atomic (lrsc + 18)    | 19   | 0    | Unaffected by FPU                  |

### Task 11 bug ledger (all Category A — the C906 factory ships no
scalar FPU; fidelity is to the spec + the rv12 algorithm sections):

FDSU aliasing hang (FUSED/SUB alias FUNC_FDSU_DIV/SQRT), `dis_gpr_fsrc0`
bit-17 alias (FUNC_SPU_MV==FUNC_MAU_NEG), DYN rm→frm resolution for
FMA, fflags same-cycle retire read race, and the 18 missing
`FUNC_B_SINGLE` decode arms (NaN-box canonicalization) — full table in
`docs/09-fpu.md` §Task 11 bug ledger.

---

**Verification log (this session):** working tree holds the uncommitted
Task 11 fixes (IDU/FPU/CSR/RVProc/pkg + `test/m5/run_all.sh`); full gate
chain re-run from a clean rebuild — M5 46/46, unit suite PASS, 86/87
sweep, 19/19 atomics. M5 acceptance ("rv64uf/ud pass") met.

## §8.17 M6 interrupts + Linux boot acceptance (§7.4 "interrupts + Linux boot (8 m6 ELFs; OpenSBI 1.3 + Linux 6.5 single-hart)")

M6 built the machine-mode interrupt delivery path and the PLIC/CLINT
wiring to boot a real OS: a 15-term `int_sel` claim with vectored
`tvec` (CSR.v / RTU.v), the real WFI (D-M4-7 discharged — `wfi_wake =
mip & mie || dtu_cp0_wake_up`), the `time` CSR (0xC01) as a CLINT mtime
mirror (D-M4-9 discharged; the M8 follow-on re-point is D-M8-5, see
below), a PLIC source 7 = UART IRQ, and the misa lockstep (D-M6-6).
All M6 RTL tasks landed (git log: b541e8d T1, ead61f6 T2, 40855c1 T3,
029fa8d T4, 20c12f4 T5, a9f9acf T6, ba80c97 T7 "8/8 m6", c0d3895 T8
Linux boot fixes); the M6 milestone is closed by c0d3895.

### Gate chain (must pass all):

```bash
make verisim                                        # clean
make -C test/m2/unit run                            # UNIT-SUITE-PASS (10 benches)
# standard battery (rv64ui/um/sv/mi sweeps, atomics, m4, m5) — same gates as §8.5-§8.16
bash test/m6/run_all.sh                             # 8/8 m6 ELFs, tohost=1 each
bash test/m6/run_linux.sh                           # 6h wall cap, checker ON; gate = "Linux version" in test/m6/boot.log
```

`test/m6/` is the 8-ELF bare-metal interrupt suite (`test/m6/Makefile`
globs `*.S` → `m6-*.elf`, `-march=rv64imac_zicsr`, `.tohost` @
0x7FFFF000; `run_all.sh` runs each with `--print-result`, 300 s timeout,
PASS = tohost 1): **msip** (M-mode software interrupt), **mtip**
(M-mode timer interrupt), **plic_uart** (PLIC source 7 = UART RX IRQ
claim), **ssip_deleg** (S-mode software interrupt via delegation),
**stip_sbi** (S-mode timer via SBI), **priority** (15-term int_sel
claim priority), **vectored** (tvec mode 1 vector dispatch), **wfi**
(real WFI sleep/wake).

### Expected results (green = pass, red = known failure):

| Gate | Result |
|---|---|
| make verisim | clean |
| unit suite | UNIT-SUITE-PASS |
| m2 sweep | 86/87 (only the documented rv64ui-p-ma_data) |
| atomics (rv64ua subset) | 19/19 |
| m4 directed SI+MI+MMU | 30/30 |
| m4 v sweep | 85/86 (only the documented rv64ui-v-ma_data) |
| m5 | 46/46 |
| **m6 suite** | **8/8** (msip, mtip, plic_uart, ssip_deleg, stip_sbi, priority, vectored, wfi) |
| **Linux boot gate** | **GREEN** — `Linux version 6.5.0` banner in `test/m6/boot.log` (OpenSBI v1.3 fw_jump + kernel 6.5.0, Buildroot 2025.02.4); banner reached at ~38M cycles in a 3600 s-capped run with the M1 ISS checker on |

Known post-banner stall: after the `Linux version 6.5.0` banner the
kernel waits for timer-driven work and the checked-in `test/m6/boot.log`
continues past the banner (to cycle ~429M, tohost=0) without further
console progress. Root-cause class: the kernel's timing path runs off
the `time` CSR, which in rv906 is a CLINT mtime mirror (clk/100) rather
than a per-cycle counter — the M8 follow-on is the `time` CSR re-point
to `mcycle_reg` (M8 T2, D-M8-5 in the M8 design doc), which lands after
the M7 close-out. **The banner is the M6 gate**; the stall is tracked
under M8, not M6.

**Verification log (M6, controller-verified on HEAD=ceb6834):** m6 8/8
(ba80c97); Linux boot gate green (c0d3895: OpenSBI v1.3 + Linux 6.5.0
banner, ~38M cycles, 3600 s-capped, checker on); full battery on the
final M7 binary: unit UNIT-SUITE-PASS, 86/87 sweep, 19/19 atomics, m4
30/30 directed, 85/86 v sweep, 46/46 m5, 8/8 m6.

## §8.18 M7 standard Debug acceptance (§7.4 "standard debug: JTAG DTM + DM + SBA + core-side triggers")

M7 implemented the RISC-V Debug Spec 0.13 subsystem — `rtl/TDT_DTM.v`
(JTAG DTM, tck↔clk via 4-FF pulse syncs), `rtl/TDT_DM.v` (Debug Module
+ abstract engine + SBA registers + 128-bit SBA AXI master),
`rtl/SBA_AxiUp.v` (128→512 crossbar up-converter), `rtl/DTU.v`
(dcsr/dpc/dscratch, 10 triggers, halt_info verdict), plus the
halt/resume/step seams in RTU/CSR/IFU/LSU/IDU. The C++ testbench is
the debug host (no OpenOCD): `dut.cpp` owns the 4 JTAG pads at the
donor timing contract (clk:tck = 8:1, 4 clk edges per TCK phase, TDO
negedge-sampled); `RVProcTest.cpp` is the DMI client. Module doc:
`docs/10-debug.md`.

### Gate chain (must pass all):

```bash
make verisim                                        # clean
make -C test/m2/unit run                            # UNIT-SUITE-PASS (10 benches incl. dtu_tb)
make -C test/m7/unit run                            # dm_tb + dtm_tb → UNIT-SUITE-PASS
bash test/m7/run_debug.sh                           # --m7-debug on the spin ELF, 300 s cap → M7-DEBUG-PASS
bash test/m7/directed/run_directed.sh               # m7-break_no_skip (tohost=1)
# standard battery — same gates as §8.5-§8.17
bash test/m4/directed/run_directed.sh               # rv64mi-p-breakpoint among them
```

### Expected results (green = pass, red = known failure):

| Gate | Result |
|---|---|
| make verisim | clean |
| unit suite (m2, 10 benches incl. dtu_tb) | UNIT-SUITE-PASS |
| m7 unit (dm_tb + dtm_tb) | UNIT-SUITE-PASS |
| **run_debug.sh → M7-DEBUG-PASS** | **GREEN**: trigger steps (i)-(v) all ok; SBA o0-r all ok (12 ok lines); core left HALTED; 58540 TCK; zero "abstractcmd busy timeout"; sim exit=0 |
| m7 directed: m7-break_no_skip | PASS (tohost=1) |
| m7 directed: m7-debug_spin | FAIL **by design** (the spin ELF never writes tohost=1 — it is the JTAG host's halt/trigger target; annotated in the runner, not gated) |
| **rv64mi-p-breakpoint** | **PASS at the 1026-cyc baseline** (restored by the T8b word-align omission, 10-debug.md §4.1 item 4) |
| unit (final binary) | UNIT-SUITE-PASS |
| m2 sweep | 86/87 (only the documented rv64ui-p-ma_data) |
| atomics (rv64ua subset) | 19/19 |
| m4 directed SI+MI+MMU | 30/30 |
| m4 v sweep | 85/86 (only the documented rv64ui-v-ma_data) |
| m5 | 46/46 |
| m6 | 8/8 |

### Final battery (M7 T10 sign-off, 2026-09-24, HEAD=0cff95c — the M8-fixed tree)

One run, all gates green (controller-verified; the T10 battery script runs
STAGE0 verisim → STAGE9 time_cyc in sequence, log kept with the T10
record). The M7 gate chain above plus the standard battery were re-asserted
on the tree that also carries the M8 RTL work (T4c/T4d coremark fixes,
T2 `time` CSR / D-M8-5):

| Gate | Result |
|---|---|
| make verisim | clean |
| m2 unit suite (10 benches incl. dtu_tb; csr_tb T27 + rtu_tb T20 refreshed for T2 / T4d-14, 0cff95c) | UNIT-SUITE-PASS |
| m7 unit (dm_tb + dtm_tb) | UNIT-SUITE-PASS |
| m2 sweep | 86/87 (only the documented rv64ui-p-ma_data) |
| m4 v sweep | 85/86 (only the documented rv64ui-v-ma_data) |
| m4 directed SI+MI+MMU (incl. rv64mi-p-breakpoint @1026 cyc) | 30/30 |
| m5 | 46/46 |
| m6 | 8/8 (wfi: D-M8-5 mtime-MMIO calibration + tick-straddle range, 0cff95c) |
| m7 debug (--m7-debug, 58,540 TCK) | M7-DEBUG-PASS |
| m7 directed (m7-break_no_skip) | PASS (debug_spin FAILs by design, annotated) |
| m8 suite | 5/5 (csr 345 / interrupt 512 / MMU 730 / time_cyc 4493 / coremark 503406) |

### Directed coverage (the e2e evidence, all controller-verified on HEAD=ceb6834):

- **halt** — dmcontrol.haltreq → dmstatus anyhalted=1/anyrunning=0; dpc in the spin-loop range; dcsr.cause=3 (dm_sync); x8 intact.
- **dpc/dcsr capture** — the halted PC and the cause are readable via the abstract CSR path.
- **abstract GPR/CSR r/w** — GPR read x8 = 0x5A5A0000; 64-bit GPR write to x9; CSR r/w through the dscratch1/x6 REGACC path; cmderr=2 on an unsupported regno; cmderr=4 while running.
- **ITR** — `addi x13,x13,1` via abstractcmd; dpc unchanged.
- **step** — exactly one instruction (dpc +4, dcsr.cause=4).
- **resume** — via `resumereq` AND via dret (ITR `0x7B200073`).
- **triggers** — execute-halt (action 1, dcsr.cause=2), execute-trap (action 0, mcause=3, mepc in {LOOP, LOOP+4}), load-trigger match (action 0 → trap, load cancelled — x14 untouched), store-trigger suppression (action 0 → trap, the store word stays at its pre-store value), and a second trigger via tselect=1.
- **SBA** — 32/64/128-bit r/w (P32=0xDEADBEEF; 64-bit bytes 0xA0..0xA7; 128-bit bytes 0x00..0x0F), clobber independence (32-bit low-word overwrite leaves the high 96 bits intact), tohost uncached readback 0x77777777 @0x7FFFF000, sberror=4 (unsupported access — the register latches the raw sbaccess, donor-faithful; spec 0.13 would clear sbaccess), sberror=3 (unaligned 128-bit).

### Known deviations (documented in 10-debug.md §4, not silent):

The four-item M7 deviation ledger: (1) the two `!rtu_yy_xx_dbgon`
fetch/issue gates (IFU.v:1094 load-bearing, LSU.v:1190 defense-in-depth)
are donor deviations — the donor C906 has no dbgon gate in
fetch/issue/commit and rv906 closes the halted-hart-commits-memory-ops
gap; (2) the LSU execute-verdict fanout (LSU.v:138/1240/1247/1355/3177,
RVProc.v:1278) is a donor-faithful restoration of
`aq_idu_id_dp.v:1099-1100/:1117-1118`; (3) the trigger-address
sign-extension (DTU.v:707-710) is a donor-faithful restoration
(`aq_dtu_mcontrol.v:1105/:1237`, the active lines); (4) the ldst-path
low-4-bit word-align is deliberately omitted (DTU.v:681-706) — with it,
rv64mi-p-breakpoint regressed 1026→1049 cyc FAIL. Plus the D-M7-*
design-time decisions (single clock domain, no trst_n/TAP2, SBA =
crossbar master #3, standard 0.13 tdata1 layout, T-Head CSRs dropped,
no async halt, single-issue DTU, no DM clock gating) — 10-debug.md
§4.1 item 5.

