# 6. Retire and Control/Status: RTU and CSR

This chapter describes `rtl/RTU.v` (retire) and `rtl/CSR.v` (control/status
registers, the C906 "CP0"). RTU is the in-order retire point: it latches the
EX1 completion, retires it, drives the architectural register write-back
buses, and declares traps and front-end flushes. CSR holds the M-mode CSR
file and the MHCR/MXSTATUS configuration fan-out.

**How to read it.** Section 1 is the principle. Section 2 is RTU, section 3
is CSR. Section 4 is the C906 cross-reference. Section 5 is the design
discussion.

**Normative documents.** Contract
`docs/superpowers/specs/2026-08-20-m2-integer-design.md`; C906 facts in
`notes/2026-08-20-c906-rtu-extraction.md`. Donor citations relative to
`refs/openc906/C906_RTL_FACTORY/gen_rtl/{rtu,cp0}/rtl/`.

---

## 1. Principle: in-order retire, single write-back point

C906 is single-issue in-order, so retire is trivial in width (one instruction
per cycle) but not in responsibility: the retire point is where the
instruction becomes architectural. RTU

1. latches the EX1 completion (`ex2_retire_vld`),
2. drives the two architectural write-back buses (`wb0` = the rbus arbiter
   winner, `wb1` = the LSU's dedicated late port),
3. declares synchronous exceptions (illegal/ecall/ebreak from CP0, misalign
   from LSU) and interrupts, latching `mepc`/`mcause`/`mtval`,
4. drives front-end flush/changeflow on trap, mret, and fence.i.

---

## 2. RTU (`rtl/RTU.v`)

- **Retire latch** (`ex2_retire_vld`, `ex2_cur_pc`, `ex2_next_pc`,
  `ex2_inst_*`): one register stage mirroring the donor's
  `aq_rtu_dp`/`aq_rtu_retire` split. `ex2_cur_pc` is the retiring
  instruction's PC (public for the harness's PC mirror).
- **Write-back buses**: `wb0` is the rbus arbiter winner (ALU/BJU fwd merged,
  then CP0, then DIV, then MUL priority — `aq_rtu_rbus.v`), registered one
  cycle; `wb1` is the LSU's late port, a pure combinational passthrough (the
  donor adds no register there either). The LSU writeback-race grants
  (`div_wb_grant`, `mul_wb_grant`) gate DIV/MUL write-back against an EX1
  write-back claiming the same cycle.
- **Traps**: `retire_trap_vld` on a synchronous exception or interrupt;
  latches `retire_trap_chgflw_vld`, redirects to `cp0_rtu_trap_pc` (mtvec).
- **Flush FSM** (`FLUSH_IDLE/FE/WAIT/BE`): drains the pipe after a
  trap/mret/fence.i before re-enabling commit.
- **PC-gen retire feedback** (`rtu_iu_ex1_cmplt`, `rtu_iu_ex1_inst_len`,
  `rtu_iu_ex1_inst_split`): tells IU's `bju_pcgen_pc` tracker that the EX1
  instruction completed and by how much to advance (Task 7.3; LSU has a
  separate early `lsu_rtu_ex1_cmplt_for_pcgen` because a store leaves EX1
  before its memory op finishes).

---

## 3. CSR (`rtl/CSR.v`)

- **M-mode CSR file**: `mstatus`, `misa` (read-only-ish), `mtvec` (direct
  mode), `mepc`, `mcause`, `mtval`, `mscratch`, `mhartid`, the performance
  counters `mcycle`/`minstret`, plus the T-Head custom `MXSTATUS`/`MHCR`.
- **MHCR/MXSTATUS fan-out** (design doc S2.3.6): MHCR.ie/de/wa/rse/bpe/btbe
  drive `cp0_ifu_icache_en`, `cp0_lsu_dcache_en`, `cp0_lsu_wa`,
  `cp0_ifu_ras_en`, `cp0_ifu_bht_en`, `cp0_ifu_btb_en`. `MXSTATUS.mm`
  (`cp0_lsu_mm`) is a real R/W flop resetting to 1 but **not consulted** by
  the misalign decision in M2 (contract 3, chapter 7).
- **Trap entry/exit**: latches `mepc`/`mcause`/`mtval` on
  `rtu_yy_xx_expt_vld`, provides `cp0_rtu_trap_pc` (mtvec), handles mret.
- **FENCE/FENCE.I sequencer** (Task 10, `rv64ui-p-fence_i`): FENCE/FENCE.I
  hold in EX1 until the LSU is quiescent (`lsu_cp0_stb_empty`). FENCE.I then
  runs the donor's FENC→CDCA→IICA sequence (`aq_cp0_fence_inst.v`): drive the
  LSU D-cache clean walk (`cp0_lsu_dcache_clean`, writes back every dirty
  line), then the I-cache INV_ALL (`cp0_ifu_icache_inv_req`), then complete
  with a front-end changeflow so the first post-fence fetch reads fresh
  state. Plain FENCE completes once the LSU is quiescent.

---

## 4. C906 file cross-reference

| rv906 | C906 donor file(s) |
|---|---|
| `rtl/RTU.v` retire/datapath | `aq_rtu_retire.v`, `aq_rtu_dp.v` |
| `rtl/RTU.v` flush FSM | `aq_rtu_ctrl.v` |
| `rtl/RTU.v` rbus | `aq_rtu_rbus.v` |
| `rtl/CSR.v` CSR file | `aq_cp0_regs.v`, `aq_cp0_trap_csr.v` |
| `rtl/CSR.v` fence sequencer | `aq_cp0_fence_inst.v` |
| `rtl/CSR.v` MHCR/MXSTATUS | `aq_cp0_regs.v` |

---

## 5. Design discussion: deviations, findings, and what stayed unbuilt

**Deliberate deviations / findings:**

- **`cmplt_dp` is a retire heartbeat, not a write-back select** (Task 4
  discovery, encoded in `csr_tb` T14): it must fire for *any* non-flushed CP0
  dispatch (ecall/ebreak/mret/fence/fence.i included) or RTU's retire latch
  would never latch those instructions.
- **FENCE.I got a real serialize+clean+invalidate in Task 10** (was
  previously a documented no-op). `rv64ui-p-fence_i` is in the M2 acceptance
  set and self-modifying code needs the I-side invalidate + D-cache
  write-back to be visible. Faithful to `aq_cp0_fence_inst.v`.
- **No S-mode / no interrupts beyond the M-mode external/software/timer
  pins** (M2 scope): `mideleg`/`medeleg` exist but M2 retires everything to
  M-mode; the interrupt path is M6.

**Deliberately unbuilt in M2:** S/U-mode trap delegation, the full interrupt
controller integration (M6), debug CSRs (M7), and the donor's vector/FP CSR
state.

---

*Next: chapter 7 (`07-lsu.md`) covers the load/store unit; chapter 8
(`08-verification.md`) covers the harness.*
