# M4: Privilege (M/S/U) + Sv39 MMU + PMP — Design

Date: 2026-08-31
Status: Approved (plan: `~/.claude/plans/composed-waddling-unicorn.md`;
this doc graduates it into the project spec set)
Parent: `2026-08-20-rv906-c906-clone-design.md` §7.4 row M4: *"privilege
complete + MMU (jTLB, PTW, sv39) + PMP + counter CSRs"*, acceptance
*"rv64si/mi + vm tests pass"*.
Predecessor milestone: M3b (non-blocking LSU: LFB/RDL/VB/PFB/AMR, commits
`0be68a5`..`88caf4c`, all gates green: unit suite T1–T13, sweep 86/87 with
only documented `rv64ui-p-ma_data`, atomics 19/19).
Sibling precedent: **rv12 (C910 clone) completed M4 first** — its design
(`rv12/docs/superpowers/specs/2026-08-28-m4-mmu-design.md`), `rtl/MMU.v`
(4002 lines), `rtl/PMP.v` (460 lines) and `test/m4/` are the primary
references; every rv12 lesson re-measured against the C906 donor before
adoption here.
Extraction notes (normative C906 facts): `notes/2026-08-31-c906-m4-
extraction.md`, cited below as **N §x**.

## 1. Goal

Replace the two identity MMU stubs in `RVProc.v` with the donor's MMU
cluster, give the machine S-mode and U-mode, and make translation,
protection and delegation real. Concretely:

1. **Privilege machine** in CSR.v: pm register (M/S/U), mstatus full arm
   set, the S CSR bank, medeleg/mideleg delegation, sret, per-privilege
   ecall vectors, CSR access qualification, counter surface (mcounteren/
   scounteren + cycle/instret user aliases + minstret auto-increment).
2. **MMU** behind the frozen M1/M2 port list: 128-entry fully-associative
   flop TLB (rv906 simplification of the donor's uTLB+jTLB hierarchy),
   single-outstanding 3-level Sv39 PTW served through the LSU, PMA
   attributes from rv906's own contract-5 table, identity+PMP path for
   M-mode and satp-bare.
3. **PMP**: 8 entries, 4KB granularity, TOR/NA4(dead)/NAPOT, L-lock,
   donor-conformant readback (the C906 donor already conforms — unlike the
   C910 donor rv12 had to fix).

**The milestone says it once, in its own voice, because it sets the shape:**

1. **M4 is a drop-in by construction, and the proof obligation is the OFF
   path.** M1/M2 froze the donor's ENTIRE MMU port list on both sides and
   the pipeline hooks are already in place (ICache's stall-on-!pa_vld
   logic; LSU's `mmu_lsu_pa_vld` port; RTU's vec 12/13/15 readiness). The
   real risk is not the new translation logic but keeping the untranslated
   machine bit-identical — acceptance is therefore two-sided: the full
   M2/M3 battery MMU-off unchanged (gate G1, at the swap commit), and the
   new si/mi/v suites MMU-on.
2. **The C906's MMU is sv39-only, fault-on-A, and the tests force the D
   policy.** The stock riscv-tests v-environment (`env/v/vm.c`) was built
   for exactly this machine class: software sets A/D in every PTE it
   installs and its fault handler software-sets A/D on demand — so the
   hardware must FAULT on A=0 and on D=0-stores (no hardware A/D update).
   The C906 donor has the D-check commented out (aq_mmu_ptw.v:699-700);
   rv906 enables it (deviation D-M4-1, required by rv64si-p-dirty + the
   privileged spec; rv12 precedent).

## 2. Scope

### 2.1 In scope (geometries/donor spans in the extraction notes)

| # | Item | Donor origin | Notes ref |
|---|---|---|---|
| S1 | **`rtl/MMU.v`** — the cluster in the stub's frozen ports: TLB, PTW, regs, tlb-inv sequencer, attributes function, with `rtl/PMP.v` as its child (rv12 P18 seam; donor has PMP as MMU's sibling, aq_top.v:837) | `mmu/rtl/aq_mmu_top.v` | N §A.1 |
| S2 | **TLB** — 128-entry fully-associative FLOP TLB: entry {vld, vpn[26:0], asid[15:0], pgs[2:0], g, ppn[27:0], flg}, round-robin replacement, invalidate-all/ASID/VA counter walks, flush on any satp write; one lookup per port per cycle (I + D); same-cycle hit PA | donor jTLB (aq_mmu_jtlb.v: 64×2 SRAM) + uTLBs (aq_mmu_utlb.v: 2×10 FA) collapsed — D-M4-4 | N §A.3, §A.4 |
| S3 | **PTW** — 3-level Sv39 FSM, per level PMP-check → read (via LSU servant) → PTE-check; superpage leaves (1G@lvl1, 2M@lvl2) + alignment checks; MXR/SUM both directions; fault-on-A + fault-on-D-for-stores (D-M4-1); single outstanding; abort drain on sfence/flush | `mmu/rtl/aq_mmu_ptw.v` (20-state) | N §A.5 |
| S4 | **The PTW memory servant** — probe the D-cache array first (dirty-PTE coherence), fall through to a shared-axi_r bus read gated on !vb_vld; walk start gated on !any_stb_vld; single outstanding (rv12 P3, adapted — N §A.8 argues the coherence case) | `lsu/rtl/aq_lsu_mcic.v` shape | N §A.8 |
| S5 | **MMU regs** — satp WARL (MODE∈{0=Bare, 8=Sv39}, whole-write gated — donor aq_cp0_prtc_csr.v:139-140 shape), ASID/PPN storage, mmu_en = Sv39 && priv != M, satp-write TLB flush | `mmu/rtl/aq_mmu_regs.v` (T-Head SMIR/SMEL/SMEH/SMCIR dropped, D-M4-5) | N §A.2 |
| S6 | **tlb-inv sequencer (sfence.vma subset)** — CSR→MMU invalidate handshake; whole-TLB invalidate for every rs1/rs2 flavor (D-M4-6 over-invalidation); completion pulse | `mmu/rtl/aq_mmu_tlboper.v` INVASID/INVALL/INVVA subset | N §A.6 |
| S7 | **The attributes function** — ONE function, two sources: rv906's contract-5 PMA table (already in the MMU stub) when translated, and the off template (today's identity+PMA behavior bit-exact) when MMU off or priv==M — MAEE dropped (D-M4-2) | `mmu/rtl/aq_mmu_sysmap.v` shape | N §A.9 |
| S8 | **CSR privilege machine** — pm FSM (donor aq_cp0_trap_csr.v:590-631); mstatus arms MPP/SPP/MPIE/SPIE/SIE/SUM/MXR/MPRV/TSR/TW/TVM/FS/SD (MPP WARL 10→00; reset MPP=11, SPP=1); sstatus as S-view; S bank stvec/sepc/scause/stval/sscratch/satp-routing/scounteren; medeleg/mideleg donor masks + mdeleg_vld routing; sret; ecall vec 8/9/11; CSR access qualification (donor aq_cp0_iui.v:602-619); trap-PC mux on POST-trap pm (donor timing subtlety, aq_rtu_retire.v:993-1035) | `cp0/rtl/aq_cp0_trap_csr.v`, `aq_cp0_iui.v`, `aq_cp0_regs.v` | N §B |
| S9 | **Counter surface** — mcounteren(0x306)/scounteren(0x106), user RO aliases cycle(0xC00)/instret(0xC02) gated by the counteren chain, minstret auto-increment from RTU retire with WRITE-TAKES-PRECEDENCE (rv64mi-p-instret_overflow); time(0xC01) deferred to M6 (D-M4-9) | `cp0/rtl/aq_cp0_hpcp_csr.v` | N §B.6 |
| S10 | **Debug-trigger escape CSRs** — tselect/tdata1/tdata2/tcontrol (0x7A0-7A5) real-but-zero-triggers so rv64mi-p-breakpoint's "unsupported type" skip fires (D-M4-5); M7 replaces with real triggers | `cp0/rtl/aq_cp0_regs.v:826-831` | N §B.7 |
| S11 | **PMP** — 8 entries (donor hardwires 8; entries 8-15 read zero), pmpcfg0/pmpcfg2/pmpaddr0-7 decode arms in CSR.v, NAPOT trailing-ones masks + all-ones catch-all, TOR subtract chain, NA4 dead (donor), lowest-hit priority, flg={L,X,W,R}, M-default 0111, deny equation with M-mode L-bypass, lock incl. lock-below-TOR, DONOR-CONFORMANT NAPOT readback (aq_pmp_regs.v:429-436, G=10 — NOT a deviation for C906, unlike C910/rv12); PMP results cached in TLB entry flags + re-checked per access at hit; channels fetch/data/PTW + mach/bare path (every access PMP-checked, donor PTW_MACH_PMP collapsed to a combinational branch, D14) | `pmp/rtl/` (all four files) | N §C |
| S12 | **LSU integration** — AG wait-state flop with same-cycle bypass (D1); LSU fault generation vec 13/15/5/7 + tval=VA (extend the reply_fire expt machinery); MPRV priv resolution for lsu_mmu_priv_mode | `lsu/rtl/aq_lsu_ag.v:1053-1056,1364-1368,674-675` | N §D |
| S13 | **IFU integration** — fetch-fault channel IFU→IDU→RTU (vec 12/1, epc=tval=fetch PC); ICache stall logic already present (no edit); flush aborts walks | `ifu/rtl/aq_ifu_icache.v:723-757`, `cp0/rtl/aq_cp0_iui.v:658-661,688-689` | N §D.2 |
| S14 | **sfence.vma carrier** — IDU decode arm (funct7=0001001) + CP0_FUNC_SFENCE (+ SRET/WFI arms); CSR sequencer: STB quiescence wait (fence-like) → MMU tlb-inv handshake → complete; TW/TVM checks | `cp0/rtl/aq_cp0_fence_inst.v:178-182`, `aq_cp0_rst_ctrl.v:523-591` | N §B.4 |
| S15 | **test/m4** — Makefile (v/si/mi/mmu lists spelled out), env/v override (rv906v.ld + EXTRA_INIT tohost-megapage PTE), directed MMU set adapted from rv12's six | rv12 `test/m4/` | N §E |

### 2.2 Out of scope / deferred

| Item | Milestone | Argument |
|---|---|---|
| T-Head SMIR/SMEL/SMEH/SMCIR (0x9C0-9C3) + TLBP/TLBR/TLBWI/TLBWR software-refill ops | dropped (D-M4-5) | diagnostic-only; no boot/test consumer; Linux + riscv-tests use hardware walk + sfence only |
| MAEE (mxstatus bit21, PMA-in-PTE) | dropped (D-M4-2) | reset-1 in C906 would make all stock-test pages uncacheable; rv12 P6 precedent |
| `time` CSR (0xC01) + mtime wiring | M6 | zicntr.S probes only cycle/instret |
| WFI real wake-on-interrupt | M6 | D-M4-7: flushing no-op until interrupts exist; rv64si-p-wfi passes on the no-op |
| Vectored mtvec/stvec (mode bit0 stored) | decide at task 1 | illegal.S's vectored probe skips gracefully when bit0 reads 0; donor stores bit0 — default plan: store it, keep direct-mode trap-PC (no base+4*cause) until a test needs it |
| Hardware A/D update | never | fault-on-A/D is the machine class the tests target (v-env software A/D) |
| sfence.vma VA/ASID-qualified invalidation precision | dropped (D-M4-6) | whole-TLB over-invalidation is architecturally safe; perf-only |
| I-cache invalidate on sfence.vma | dropped (D-M4-6) | rv906 ICache is physically tagged (ptag=PA[39:12], ICache.v:456-457); remaps miss on tag naturally (rv12 measured the same at its icache-alias close-out) |
| PMP huge-page-cross demotion (PTW_{1G,2M}_PMP1/2 states) | dropped | rv906 PMP is 4KB-granular: a superpage cannot straddle a PMP boundary inconsistently — the donor's demotion only matters for sub-page PMP regions, which 4K granularity forbids (argument recorded at the RTL site) |
| Interrupt delegation first execution | M6 | mideleg storage + routing built at M4 (dark until an interrupt source exists) |

### 2.3 Documented deviations (the ledger — record at RTL sites too)

- **D-M4-1**: D-check for stores ENABLED (C906 donor has it commented out,
  aq_mmu_ptw.v:699-700) — required by rv64si-p-dirty + the v-env + the
  privileged spec. rv12 precedent (its C910 donor has the arm).
- **D-M4-2**: MAEE dropped (PMA always from rv906's contract-5 table).
- **D-M4-3 is NOT a deviation for rv906**: the C906 donor's pmpaddr
  readback is already conformant (aq_pmp_regs.v:429-436: NAPOT reads
  {stored[0],9'h1ff}, OFF/TOR read 0 → G=10). rv12's D-M4-3 was a
  C910-only fix; rv906 ports the donor verbatim.
- **D-M4-4**: single 128-entry FA flop TLB replaces the donor's
  uTLB(2×10)+jTLB(64×2 SRAM) two-level structure — rv906's single-issue
  in-order pipe needs one lookup per cycle; multi-hit impossible by
  construction (single writer).
- **D-M4-5**: T-Head SMIR/SMEL/SMEH/SMCIR + tlbp/tlbr/tlbwi/tlbwr dropped
  (diagnostic). rv64mi-p-breakpoint passes via the no-triggers escape.
- **D-M4-6**: sfence.vma over-invalidates (whole TLB for every flavor) and
  skips the donor's follow-on I-cache invalidate (physical tags).
- **D-M4-7**: WFI = flushing no-op until M6.
- **D-M4-8**: NA4 stays dead (donor aq_pmp_comp_hit.v:101 ties it 0) —
  donor-faithful; pmpaddr.S doesn't probe access blocking.
- **D-M4-9**: time CSR deferred to M6.
- **D-M4-10** (Task 4, more-correct-than-donor like D-M4-1/3/8): SUM never
  excuses a supervisor FETCH from a U page. The donor's walker S→U arm is
  `pte_u && supv && !sum` unconditionally (aq_mmu_ptw.v:678) — under SUM=1
  it wrongly lets S-mode EXECUTE from a U page too. The privileged spec is
  explicit SUM governs loads/stores only, never execution. rv906 qualifies
  the excuse to data accesses (`!(sum && !fetch)`), applied identically at
  the walker (MMU.v SECTION 6.3) and the TLB-hit predicate (SECTION 4) —
  matches rv12's identical, already-shipped fix (its own D-M4-9, a
  different numbering scheme; rv12 applies it at both its uTLB hit arm and
  its walker). mmu_tb T19 pins both SUM values.

## 3. References

- Donor tree: `refs/openc906/C906_RTL_FACTORY/gen_rtl/` — `mmu/rtl/`
  (aq_mmu_top/jtlb/ptw/utlb/tlboper/regs/sysmap/arb/plru + arrays),
  `pmp/rtl/` (aq_pmp_top/regs/comp_hit/acc), `cp0/rtl/` (aq_cp0_trap_csr/
  aq_cp0_iui/aq_cp0_regs/aq_cp0_prtc_csr/aq_cp0_hpcp_csr/aq_cp0_ext_csr/
  aq_cp0_fence_inst/aq_cp0_rst_ctrl), `lsu/rtl/aq_lsu_ag.v`,
  `lsu/rtl/aq_lsu_mcic.v`, `ifu/rtl/aq_ifu_icache.v`,
  `rtu/rtl/aq_rtu_retire.v`.
- rv12 M4: `rv12/docs/superpowers/specs/2026-08-28-m4-mmu-design.md`,
  `rv12/rtl/MMU.v`, `rv12/rtl/PMP.v`, `rv12/test/m4/`.
- Tests: `RISCV_TESTS = /home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/
  rv-test/riscv-tests` — isa/rv64si (7), isa/rv64mi (17), env/v (vm.c
  Sv39 demand-paging env), isa/rv64u*/Makefrag v-lists (54+13+19=86).
- rv906 anchors: `rtl/MMU.v` (frozen-port stub), `rtl/ICache.v:235-260`
  (stall-on-!pa_vld already present), `rtl/LSU.v:178` (mmu_lsu_pa_vld
  port, unconsumed), `rtl/RTU.v:566-576,661-678,736-830` (trap machinery),
  `test/m2/env/rv906.ld` (uncached tohost convention).

## 4. Architecture

### 4.1 Key design decisions (D1–D14)

- **D1 — LSU DTLB-miss: AG WAIT STATE with same-cycle bypass.** One flop
  `ag_wait_r`: on the dispatch cycle, pa_vld=1 → issue proceeds EXACTLY as
  today (bit-identical OFF path); pa_vld=0 → ag_wait_r sets, lsu_idu_full
  gains the term, IDU holds the op in EX1 (verified: IDU keeps driving
  idu_lsu_ex1_src*_data live from EX1 registers with forwarding updates,
  IDU.v:1385-1394), `lsu_mmu_va_vld = ag_valid || ag_wait_r` keeps the MMU
  request level-valid during the walk; RTU flush clears ag_wait_r. No comb
  loop (pa_vld from the live-VA tag compare; the new full term is
  registered).
- **D2 — IFU needs NO stall edit** (ICache already stalls on !pa_vld);
  M4 builds the missing fetch-fault path (icache_ipack_acc_err/_pgflt are
  dropped in IFU.v's _unused_ok today, IFU.v:476): IFU→IDU fetch-exception
  channel, vec 1/12, epc=tval=fetch PC.
- **D3 — PTW servant in LSU.v**: probe D-cache array FIRST (dirty-PTE
  coherence), bus read gated on !vb_vld && !axi_r_active, walk start gated
  on !any_stb_vld (STB drains on idle cycles; drained STB makes the
  array-only probe coherent without STB forwarding); sfence.vma also waits
  for STB quiescence. Single outstanding walk.
- **D4 — MMU shape**: single 128-entry FA flop TLB + one outstanding PTW
  (details D11–D14 below).
- **D5 — sfence.vma as CP0-class op** (CP0_FUNC_SFENCE): STB quiescence
  wait → CSR→MMU tlb-inv handshake; whole-TLB for every flavor.
- **D6 — WFI = flushing no-op** (rv12 precedent) until M6.
- **D7 — privilege machine in CSR.v** (S8 above).
- **D8 — counters** (S9 above).
- **D9 — PMP** (S11 above).
- **D10 — test infra** (S15 above).
- **D11 — TLB simplification**: flop-based FA (no SRAM), round-robin
  replacement, ASID+G match, invalidate as counter-walks over flops,
  satp-write flush.
- **D12 — T-Head TLB-op CSRs dropped** (D-M4-5).
- **D13 — superpage support KEPT** (2M/1G leaves + alignment): v-env maps
  a 2M kernel megapage — required, not optional. PMP-cross split states
  dropped (page-granular PMP).
- **D14 — mach/bare path**: priv==M or satp-bare → identity translate +
  PMA + PMP-check (donor's mach-tagged entries + PTW_MACH_PMP collapsed to
  a combinational branch).

### 4.2 Trap plumbing (who raises what)

- MMU raises pgflt/accflt per port only (donor shape): IFU port encodes
  fetch page-fault in prot[4] + access_fault pin; LSU port has
  page_fault/access_fault pins.
- CP0 (CSR.v) encodes fetch causes 12/1 (tval = faulting PC) — new
  IFU→IDU→CSR path.
- LSU encodes data causes 13/15 (page) and 5/7 (access) at the AG/REPLY
  fault point, tval = faulting VA (extend the existing reply_fire expt
  machinery, currently misalign-only vec 4/6).
- Trap entry: RTU broadcasts (expt_vld/int/vec, epc, tval); CSR.v selects
  M vs S capture set by mdeleg_vld; pm updates on the expt_vld cycle so
  the trap-PC mux (pm==M ? mtvec : stvec) is settled when RTU samples
  cp0_rtu_trap_pc (donor timing subtlety).

## 5. Task decomposition

| # | Task | Gate focus |
|---|---|---|
| 0 | Design doc (this file) + extraction notes | docs |
| 1 | CSR privilege machine (S8/S9/S10) | csr_tb expansion; rv64si-p-csr/scall/sbreak/wfi; rv64mi-p-csr/mcsr/illegal/zicntr/instret_overflow/breakpoint |
| 2 | PMP.v (S11) | rv64mi-p-pmpaddr; directed pmp tests |
| 3 | MMU.v skeleton (S5/S7) | G5 off-template equivalence |
| 4 | TLB + PTW (S2/S3) | mmu_tb translation rows; rv64si-p-dirty/icache-alias/ma_fetch |
| 5 | LSU servant + integration (S4/S12) | lsu_tb servant + AG-wait rows; rv64si-p-dirty MPRV arms |
| 6 | IFU integration (S13) | rv64si-p-ma_fetch; directed fetch-fault |
| 7 | sfence.vma carrier (S6/S14) | rv64si-p-dirty; v-env sfence discipline |
| 8 | THE SWAP (S1) | G1 OFF-path identity (battery bit-identical MMU-off) |
| 9 | test/m4 infra + directed set (S15) | build only |
| 10 | Acceptance + close-out | all gates |

Sequencing rules (rv12 P19 inherited): new-CSR decode acceptance flips ONLY
at the swap commit (task 8); p-env boots write pmpaddr/pmpcfg/satp/
medeleg/mideleg today and must keep "silently absorbed" until then. Tasks
that move cycle counts land strictly before or after the G1 measurement
commit.

## 6. Verification design

**Pass bar (two-sided):**
1. **OFF path (G1):** after the swap commit, the ENTIRE existing battery
   (unit suite incl. mmu_tb PMA rows, sweep 86/87, atomics 19/19) runs
   MMU-off (satp=0 reset) with behavior bit-identical to the pre-swap
   commit. satp MODE=0 keeps the identity+PMA template; the AG wait-state
   bypass keeps issue timing identical (same-cycle pa_vld).
2. **ON path:** rv64si-p 7/7; rv64mi-p 17/17 (deviations documented);
   v suites rv64ui-v 54 + rv64um-v 13 + rv64ua-v 19 = 86 ELFs; directed
   MMU set (ptwalk dirty-PTE coherence + VB-writeback race twin, pmp,
   pmpstore, misalign-under-page, ldst_samepage, amo-under-vm).

**Standard gates every task:** `make verisim` clean; `make -C
test/m2/unit run` → UNIT-SUITE-PASS; `bash test/m2/run_all.sh` → 86/87
(documented rv64ui-p-ma_data); `/tmp/run_atomic.sh` → 19/0.

**Unit benches:** csr_tb gains privilege/S-bank/delegation/counter rows;
mmu_tb gains translation/PTE-permission/superpage/invalidate/PMP rows
(PMP benched through MMU.v top); lsu_tb gains servant + AG-wait-state rows
(including the dirty-PTE-walk coherence case and the VB-writeback race
twin of rv12's MED-5).

## 7. Risks / open points

- **Trap-PC mux timing**: the donor's one-cycle-delayed sampling must be
  replicated (pm updated on expt_vld; mux settled when RTU samples). Pin
  with a directed trap-from-S test early in task 1.
- **rv64mi-p-illegal MPP probe**: MPP must be genuinely writable (the test
  writes MPP=S and reads back); the WARL 10→00 rule must hold.
- **v-env tohost**: the uncached-megapage PTE treatment (rv12 mechanism)
  is load-bearing — a wrong constant fails loudly (first do_tohost store
  page-faults), not silently.
- **minstret write-precedence**: rv12 R15 found the naive increment fails
  instret_overflow; implement write-suppresses-increment from the start.
- **Cycle-count identity at G1**: none of tasks 1–8 is expected to move
  the OFF-path instruction counts; measure at the swap commit and compare
  against the parent.
