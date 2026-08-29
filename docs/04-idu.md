# 4. The Integer Decode Unit: IDU

This chapter describes `rtl/IDU.v`, the milestone-M2 instruction-decode +
operand-read + dispatch stage. It sits between the front end (`IFU.v`,
chapter 2) and the execute units (`IU.v` chapter 5, `LSU.v` chapter 7,
`CSR.v` chapter 6) and is the single-issue in-order bottleneck of the C906:
one instruction decoded per cycle, one dispatched into EX1 per cycle.

**How to read it.** Section 1 is the principle: what a single-issue in-order
decode stage has to do, and why C906's version is a single combinational
stage plus a write-back scoreboard rather than a multi-issue rename machine.
Section 2 is this implementation. Section 3 is the C906 file cross-reference.
Section 4 is the design discussion: the deviations rv906 chose deliberately,
the bugs found bringing M2 up, and what stayed deliberately unbuilt.

**Normative documents.** The contract is
`docs/superpowers/specs/2026-08-20-m2-integer-design.md`; the C906 facts were
extracted into `notes/2026-08-20-c906-idu-extraction.md`. Donor citations are
relative to `refs/openc906/C906_RTL_FACTORY/gen_rtl/idu/rtl/`.

---

## 1. Principle: one combinational stage, scoreboard in hardware

C906 is single-issue and in-order. There is no register rename and no
out-of-order window, so the decode stage's job reduces to four things, all
done in one combinational stage (`ID`/`DIS` in the donor's naming):

1. **Decode** the 32- or 16-bit instruction into an execution-unit one-hot
   (`EU_ALU/BJU/MULT/DIV/LSU/CP0`), a function code, an immediate, and the
   source/destination register numbers. RVC (16-bit) instructions are
   decoded directly, not expanded to their 32-bit equivalents.
2. **Read the operands** out of the 31-entry architectural register file.
3. **Detect hazards** against in-flight writes (the write-back-table / WBT
   scoreboard) and against the three in-flight writeback buses (fwd0/1/2),
   stalling dispatch on a true RAW/WAW dependency that no forward can cover.
4. **Dispatch** into EX1, one instruction per cycle, unless EX1 is occupied
   by a multi-cycle operation that backpressures (mul/div/LSU/BJU-entry).

Because there is exactly one instruction in EX1 at a time, the "scoreboard"
is small: a 32-entry busy-bit table (one bit per architectural register, x0
excluded) plus a 2-bit counter per entry to tolerate two overlapping writes
to the same register. This is the donor's `aq_idu_id_wbt`.

The forward path matters for the common 1- and 2-back dependencies: an ALU
result produced in EX1 reaches a dependent instruction the very next cycle
through `fwd0` (the EX1 result bus), a result one cycle older through
`fwd1`/`fwd2`. A dependency the forwards cannot satisfy stalls dispatch until
the write-back lands; the WBT busy-bit clears on the write-back pulse.

RVC decode is done directly (not by expanding to 32-bit): the 16-bit opcode
space is classified in parallel with the 32-bit path and a length bit
(`is32`) selects between them. This matters for the PC-bookkeeping in
`IU.v`/`IFU.v` (a compressed instruction advances the PC by 2, not 4).

---

## 2. This implementation

`rtl/IDU.v` is one module containing, in order:

| Section | Role | Donor source |
|---|---|---|
| DECODE | 32-bit `casez` + 16-bit `casez`, `is32` select, immediates | `aq_idu_id_decd.v` |
| WBT | 32-entry busy-bit scoreboard (x0 hardwired ready) | `aq_idu_id_wbt.v`/`_entry.v` |
| GPR | 31-entry register file + read-during-write merge | `aq_idu_id_gpr.v`/`_gated_reg.v` |
| FORWARD MUX | 3-way fwd0/1/2 operand mux | `aq_idu_id_dp.v:639-727` |
| RAW/WAW HAZARD | producer-type-aware stall exceptions | `aq_idu_id_ctrl.v:398-536` |
| EX1 REGISTER + LATE FORWARD | the one pipeline register + wb0/wb1 late forward | `aq_idu_id_dp.v:893-997` |
| EU DISPATCH | EX1 issue-gate, EU one-hot select | `aq_idu_id_ctrl.v:619-663` |

Key behaviors:

- **`is32 = (inst[1:0] == 2'b11)`** selects the 32-bit decode; everything
  else is the 16-bit RVC path. The output-value tables match the donor
  bit-for-bit for every covered mnemonic (contract: value-faithful, not
  line-for-line).
- **The WBT** (`wbt_wb_r[1:31]`, `wbt_cnt_r`, `wbt_type_r`) is created on a
  non-stalled dispatch (`wbt_create0`, gated on `!iu_idu_br_cancel` exactly
  like the donor's per-entry create, `aq_idu_id_wbt_entry.v:75`) and cleared
  by the write-back pulse (`rtu_idu_wb0_vld`/`wb1_vld`). The 2-bit counter
  tolerates two overlapping writes; a third would be a WAW the stall logic
  catches.
- **The GPR read** uses the donor's read-during-write merge
  (`aq_idu_id_gpr_gated_reg.v:86-109`): a same-cycle write-back to the read
  register returns the write data, not the stale stored value. This is
  load-bearing for 2-back dependencies (see §4, bug #2).
- **RAW stall exceptions** are producer-type-aware (`aq_idu_id_ctrl.v`): an
  ALU/BJU producer never stalls (its forward always covers it); an LSU
  producer stalls unless the consumer is a conditional branch that can park
  in the BJU entry (see chapter 5); a MULT producer with `cnt==2` stalls.
- **The EX1 register** (`ex1_vld_r`/`ex1_eu_r`/`ex1_*_r`) is the single
  pipeline register. It is cancelled by `rtu_idu_flush_fe || iu_idu_br_cancel`
  (the donor's `iu_yy_xx_cancel`, `aq_idu_id_ctrl.v:604`) and advances when
  not stalled. A late forward (`lf0/lf1/lf2`) patches a latched-but-not-ready
  operand from wb0/wb1 while the instruction sits in EX1.
- **FENCE/FENCE.I** dispatch to `EU_CP0` and serialize through CSR.v's
  fence sequencer (chapter 6): they hold in EX1 (`cp0_idu_fencei_full` folds
  into the EU-full gate) until the LSU is quiescent and the clean/invalidate
  walks complete.

---

## 3. C906 file cross-reference

| rv906 (`rtl/IDU.v` section) | C906 donor file(s) |
|---|---|
| DECODE | `aq_idu_id_decd.v` |
| WBT | `aq_idu_id_wbt.v`, `aq_idu_id_wbt_entry.v` |
| GPR | `aq_idu_id_gpr.v`, `aq_idu_id_gpr_gated_reg.v` |
| FORWARD MUX | `aq_idu_id_dp.v` (639-727) |
| RAW/WAW HAZARD | `aq_idu_id_ctrl.v` (398-536) |
| EX1 REGISTER + LATE FORWARD | `aq_idu_id_dp.v` (893-997) |
| EU DISPATCH | `aq_idu_id_ctrl.v` (619-663) |
| (glue) | `aq_idu_top.v` |

---

## 4. Design discussion: deviations, findings, and what stayed unbuilt

**Deliberate deviations (documented, design-doc-sanctioned):**

- **Single combinational decode stage.** The donor splits decode across a few
  sel/case stages; rv906 collapses them into one combinational stage because
  the output *values* are what matter (contract: value-faithful, not
  line-for-line). Timing-equivalent, simpler to verify.
- **RVC decoded directly, not expanded.** The donor also decodes RVC
  natively; rv906 matches. The length bit (`is32`) feeds the PC bookkeeping.
- **No split/serialization FSM for CP0 ops** (contract 10): M2 CP0 ops
  complete in EX1 with no multi-cycle split. FENCE/FENCE.I serialize through
  CSR.v's fence sequencer (added in Task 10 for `rv64ui-p-fence_i`), which is
  a real donor behavior (`aq_cp0_fence_inst.v`).
- **`ctrl_ex1_internal_stall`/`ctrl_ex1_issue_stall` simplified** to the M2
  conditions (mul issue-stall, BJU-entry global-full); the donor's additional
  stall sources don't exist in M2's contracts.

**Bugs found during M2 bring-up (fixed in this milestone):**

1. **WBT create must be gated on `!iu_idu_br_cancel`** (branch-mispredict
   flush). Without the gate, a wrong-path instruction on the not-taken side
   of a mispredicted branch armed its destination's busy-bit, which was never
   written back, leaving the register permanently "pending" and wedging a
   later dependent branch (first seen on `rv64ui-p-beq`). Matches the donor's
   per-entry create gate (`aq_idu_id_wbt_entry.v:75`).
2. **GPR read-during-write merge is load-bearing.** Without the same-cycle
   write-back merge, a 2-back dependency (`addi a1; addi a2; add a4,a1,a2`)
   read `a1` stale and computed the wrong sum (`rv64ui-p-add` test 3/4).
   Matches `aq_idu_id_gpr_gated_reg.v:86-109`.
3. **`c_lw_imm`/`c_lwsp_imm` were transposed** (Task 10, `rv64uc-p-rvc`).
   The C.LW (CL-format) offset `{inst[5],inst[12:10],inst[6],2'b0}` and the
   C.LWSP (CI-format) offset `{inst[3:2],inst[12],inst[6:4],2'b0}` had been
   swapped; C.SW/C.SD (which reuse `c_lw_imm`/`c_ld_imm`) then computed a
   wrong store address. Fixed and covered by `rv64uc-p-rvc`.

**Deliberately unbuilt in M2** (contract 10 / design doc): FP/vector decode
(all FP/vector/custom/AMO/LR-SC/sfence.vma/sret/wfi/dret decode to illegal),
the donor's split-FSM CP0 serialization, and a decode-time JAL-target
correction path (BJU handles redirects; see chapter 5).

---

*Next: chapter 5 (`05-iu.md`) covers the integer execute units the IDU
dispatches into; chapter 6 (`06-rtu-csr.md`) covers retire + CSR; chapter 7
(`07-lsu.md`) covers the load/store unit; chapter 8 (`08-verification.md`)
covers the harness that proves all of this.*
