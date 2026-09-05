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
| rv64si-p-wfi / rv64mi-p-illegal    | TW/TVM/TSR trap arms; WFI flushing no-op (D-M4-7)                  |
| rv64mi-p-pmpaddr                   | PMP WARL/NAPOT-readback conformance (no deviation needed, N §…)   |
| rv64mi-p-breakpoint                | No-triggers escape (real tselect/tdata1/tdata2/tcontrol, D-M4-5)  |
| rv64mi-p-instret_overflow          | Write-suppresses-increment minstret semantics                     |

### Known deviations (documented, full ledger in the M4 design doc §2.3):

- D-M4-1: D-bit check enabled for stores/AMO/SC (donor has it commented out).
- D-M4-2: MAEE dropped — PMA always from rv906's own sysmap-style table.
- D-M4-4/D11: single 128-entry fully-associative flop TLB (no uTLB/jTLB split).
- D-M4-5/D12: T-Head SMIR/SMEL/SMEH/SMCIR + tlbp/tlbr/tlbwi/tlbwr dropped.
- D-M4-6: sfence.vma over-invalidates (whole TLB for every flavor).
- D-M4-7: WFI is a flushing no-op until M6's interrupt path exists.
- D-M4-8: NA4 stays dead (donor ties it 0 too).
- D-M4-9: `time` CSR (0xC01) deferred to M6/CLINT.
- D-M4-11: "THE SWAP" (Task 8) closed by construction — CSR.v's privilege
  decode was live from Task 1 onward, so G1 was satisfied continuously by
  every task's gate run rather than by one flip commit.

---

**Verification log (this session):** working tree clean after the Task
0–9 commits; the full gate chain above (unit suite, 86/87 sweep, 19/19
atomics, 85/86 OFF-path v-suite, 30/30 directed si+mi+mmu, 85/86 ON-path
v-suite under real Sv39 translation) was re-run and confirmed green from
a clean rebuild, bit-identical to the pre-M4 baseline on the OFF path.
M4 fully implemented and tested; §7.4's M4 acceptance criteria
("rv64si/mi + vm tests pass") are met.
