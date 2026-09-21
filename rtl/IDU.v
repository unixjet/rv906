//=============================================================================
// IDU.v - decode + WBT scoreboard + GPR + RTU-forward mux + EU dispatch
//                                                     (M2 Task 5: real body)
//=============================================================================
// C906 files covered:
//   gen_rtl/idu/rtl/aq_idu_top.v      (glue)
//   gen_rtl/idu/rtl/aq_idu_id_decd.v  (decoder: 6-way coarse classify then
//                                       per-class casez tables, incl. RVC)
//   gen_rtl/idu/rtl/aq_idu_id_wbt.v   (+ _entry.v; 32-entry busy-bit
//                                       scoreboard)
//   gen_rtl/idu/rtl/aq_idu_id_gpr.v   (+ _gated_reg.v; 31-entry GPR + x0)
//   gen_rtl/idu/rtl/aq_idu_id_dp.v    (decoder mux, WBT/GPR read-address
//                                       gen, 3-source forward mux, EX1 reg)
//   gen_rtl/idu/rtl/aq_idu_id_ctrl.v  (RAW/WAW stall, EU one-hot select,
//                                       EX1 issue-enable gating,
//                                       idu_ifu_id_stall)
// References: design doc S4.1/S4.2/S5 (unit graph, id_ex1_t, pipeline-stage
// table), IDU extraction note (all sections), Task 5's own extraction pass
// against the real RTL (decd.v's full main casez + RVC casez + immediate
// generator, wbt.v/wbt_entry.v/ctrl.v's except-clauses, gpr.v/
// gpr_gated_reg.v's collision case, dp.v's forward mux/late-forward,
// ctrl.v's EU dispatch formulas) -- all cited file:line below.
//
// M2's decode is a CLEAN-ROOM reimplementation of the MECHANISM the notes
// describe, not a line-for-line port of the donor's multi-stage sel/case
// multiplexer scheme (decd.v computes a 14-way src1_imm_sel then a separate
// case; this file computes each instruction's immediate directly within its
// own decode arm, using 5 precomputed 32-bit-format immediate wires plus a
// dozen precomputed RVC-format immediate wires shared across arms) -- the
// OUTPUT VALUES match the donor bit-for-bit for every covered mnemonic,
// confirmed against decd.v's own formulas line by line (Task 5's research
// pass).
//
// CONTRACT-14 DESIGN CHOICE (load-bearing, not incidental): every RVC
// mnemonic that has a 32-bit twin decodes into the EXACT SAME EU one-hot +
// FUNC_* constant as that twin (e.g. c.addi -> EU_ALU/ALU_FUNC_ADD, the
// same constant "addi" itself produces) -- there is NO separate FUNC_C_*
// constant table, unlike the donor (whose decd.v assigns FUNC_C_ADDI, a
// value distinct from FUNC_ADD, to c.addi). This is deliberate: IU.v (Task
// 3, already committed and unit-bench-verified) only ever compares
// idu_iu_ex1_func against the pinned ALU_FUNC_*/BJU_FUNC_*/MULT_FUNC_*
// constants -- it has zero knowledge of any FUNC_C_* value and was never
// going to be revisited to learn one. Task 5.5's own bench requirement
// ("RVC pairs decoding to the exact same EU/FUNC/*_vld shape as their
// 32-bit twin") is this exact requirement stated in the plan text, not an
// inference made here. Concretely this means each RVC arm must synthesize
// operands (register-vs-immediate slot assignment, and the immediate's
// exact bit-shift/sign-extension) so the REUSED 32-bit FUNC's execute-side
// semantics still produce the architecturally-correct result -- e.g.
// c.lui's raw 6-bit field must be shifted <<12 here (ALU_FUNC_LUI expects a
// pre-shifted src1, per the 32-bit LUI's own imm20<<12 convention) even
// though the donor's own FUNC_C_LUI-keyed hardware would have consumed the
// unshifted 6-bit field itself.
//
// PORT-LIST AMENDMENTS (documented, not silent -- same discipline CSR.v's
// Task 2 "TASK 2 DISCOVERED GAP" and IU.v's/RTU.v's Task 3/4 "PORT-LIST
// AMENDMENT" notes used):
//  1. `idu_iu_ex1_bht_pred`/`_src0_reg`/`_src1_reg` (outputs) -- IU.v's own
//     Task 3 header ALREADY documents needing these three inputs (added to
//     IU.v's own frozen list) but the matching IDU.v OUTPUT amendment never
//     actually landed (confirmed by re-reading this file's own git history
//     before starting this task: only the Task-1-fixup `idu_lsu_ex1_sel`
//     amendment is present). Added here, closing the loop IU.v opened --
//     without them, BJU's BHT-mismatch feedback and its own 1-entry
//     LSU-dependent buffer's register-number matching (IU note S4.4) have
//     no data source at all.
//  2. `rtu_idu_pipeline_empty` (input) -- RTU.v (Task 4, already committed)
//     exports this (RTU note S6's drain-status signal, "for IDU's own full-
//     drain waits e.g. fence.i-adjacent serialization") but Task 1's frozen
//     IDU.v skeleton never grew a matching input. Added here for interface
//     completeness against RTU.v's real port list; NOT consumed by any
//     logic in this body -- M2's FENCE/FENCE.I dispatch as ordinary single-
//     beat EU_CP0 ops with no split/serialization FSM (contract 10), so
//     nothing in this task's scope actually needs a drain-wait on it. A
//     landing pad, no live effect today, matching IU.v's/RTU.v's own prior
//     "nothing consumes this port yet" amendments.
//  Neither amendment breaks anything today: nothing instantiates IDU.v
//  alongside IU.v/RTU.v yet (RVProc.v still runs FetchSink.v until Task 7).
//
// DISCOVERED-BUT-NOT-FIXED GAP (flagged per the task's own instruction to
// flag rather than silently guess): `rvproc_pkg.sv`'s `id_ex1_t` layout (`
// ID_EX1_*` defines, Task 1) pins a PC field and an IMM field. Task 3's real
// IU.v body turned out NOT to need either crossing this boundary: BJU
// self-tracks PC via its own `bju_pcgen_pc` register (fed by
// `cp0_xx_mrvbr`/`ifu_iu_chgflw_pc`, IU note S4.5) and forwards it to CP0
// directly (`iu_cp0_ex1_cur_pc`) -- CP0 never receives a PC from IDU either
// (confirmed: no `idu_cp0_ex1_pc`-shaped port exists anywhere). And every
// real consumer (IU.v/CSR.v) expects the immediate ALREADY MERGED into
// src1_data/src2_data (confirmed directly: IU.v's ALU reads `alu_src1 =
// idu_iu_ex1_src1_data` and uses it AS the ADDI/ADD operand with no separate
// imm port anywhere on IU.v's frozen list) -- so the pinned "carry imm
// through as its own field, let each consumer decide" design never
// materialized once Task 3 was actually built against it. Net effect: this
// body pre-merges the selected immediate into src1_data/src2_data at ID/DIS
// (matching the donor's OWN actual behavior, not the aspirational pkg
// comment), and the internal EX1 payload's "pc" bit range is tied to 0 with
// no IFU-sourced input (no `ifu_idu_id_pc`-shaped port exists on this file's
// frozen skeleton, and none is added -- there is no real consumer to justify
// one). This is a real, load-bearing finding for whoever revisits
// `rvproc_pkg.sv`'s id_ex1_t commentary, not a bug in this task's own body.
//
// SEAM NOTES (rv906 decomposition, carried over from the skeleton):
//  * ID and Dispatch are the SAME combinational stage (IDU note S2) -- no
//    register between decode and hazard-check/EU-select. The ONLY register
//    inside IDU.v proper is the EX1 latch. WBT/GPR (31 flop entries each +
//    x0) are entry arrays (generate-for loops), per umbrella S6.2 rule 6,
//    not per-entry modules.
//  * Every bypass path into this module's operand-read logic is sourced
//    EXCLUSIVELY from RTU (`rtu_idu_fwd0/1/2_*`, `rtu_idu_wb0/1_*`) --
//    matches contract 1's "IDU never sees any of the four IU->RTU
//    writeback buses directly."
//  * `idu_lsu_ex1_dp_sel` and `idu_lsu_ex1_sel` are BOTH real, distinct
//    signals, confirmed against ctrl.v:634/646 respectively (see prior
//    task's header note, unchanged): `_dp_sel` = ungated-on-commit early
//    select; `_sel` additionally gated on `rtu_idu_commit`.
//  * `ctrl_ex1_internal_stall`/`ctrl_ex1_issue_stall` drop the donor's
//    `lsu_idu_global_full`/`cp0_idu_issue_stall` terms -- neither port
//    exists anywhere in M2's contracts (LSU.v is Task 6; CSR.v, Task 2, is
//    single-cycle combinational with no stall port at all) -- dropped, not
//    guessed, consistent with contract 8's "whatever single stall signal
//    LSU exposes" (singular; LSU only ever gets `lsu_idu_full`).
//  * Illegal-instruction dispatch is forced to EU_CP0 regardless of what
//    the raw opcode bits would otherwise decode to (ctrl.v:573-574's
//    donor mechanism, simplified here to the one relevant M2 condition:
//    `dis_eu_final = dis_illegal ? EU_CP0 : dis_eu_raw`) -- without this,
//    CSR.v's `idu_cp0_ex1_illegal`-gated trap path (`ex1_illegal =
//    idu_cp0_ex1_sel && ... && idu_cp0_ex1_illegal`) would never fire for a
//    genuinely illegal instruction, since `idu_cp0_ex1_sel` would never
//    assert if decode routed it to e.g. EU_ALU instead.
//=============================================================================

import rvproc_pkg::*;

module IDU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IFU -> IDU : single-instruction handoff (unchanged since M1).
    //=========================================================================
    input  wire [31:0]              ifu_idu_id_inst,
    input  wire                     ifu_idu_id_inst_vld,
    input  wire [1:0]               ifu_idu_id_bht_pred,
    // M4 Task 6 (S13, D2): fetch-exception tags, IFU.v's own new ports --
    // consumed at ID (dis_fetch_pgflt/_accflt below), forcing EU_CP0
    // dispatch exactly like an illegal decode does.
    input  wire                     ifu_idu_id_fault_pgflt,
    input  wire                     ifu_idu_id_fault_accflt,
    // M7 Task 2: the DTU's execute-trigger halt_info for the delivered
    // instruction (IFU.v's ifu_idu_id_halt_info -- the DTU's live verdict
    // on the ibuf-head instruction, same sideband-at-head pattern as
    // ifu_idu_id_bht_pred). Latched into ex1_halt_info_r
    // like the other EX1 context bits and piped to the RTU via the IU
    // (see idu_iu_ex1_halt_info below). 0 for a trigger-free fetch.
    input  wire [TDT_HINFO_WIDTH-1:0] ifu_idu_id_halt_info,
    output wire                     idu_ifu_id_stall,

    //=========================================================================
    // IDU -> IU : EX1 dispatch, IU's slice of id_ex1_t (matches IU.v's
    // input port group exactly, including the Task 3/Task 5 port-list
    // amendment for bht_pred/src0_reg/src1_reg -- see header).
    //=========================================================================
    output wire                     idu_iu_ex1_inst_vld,
    output wire                     idu_iu_ex1_pipedown_vld,
    output wire                     idu_iu_ex1_alu_sel,
    output wire                     idu_iu_ex1_bju_sel,
    output wire                     idu_iu_ex1_bju_br_sel,
    output wire                     idu_iu_ex1_mult_sel,
    output wire                     idu_iu_ex1_div_sel,
    output wire [FUNC_WIDTH-1:0]    idu_iu_ex1_func,
    output wire [63:0]              idu_iu_ex1_src0_data,
    output wire                     idu_iu_ex1_src0_ready,
    output wire [63:0]              idu_iu_ex1_src1_data,
    output wire                     idu_iu_ex1_src1_ready,
    output wire [63:0]              idu_iu_ex1_src2_data,
    output wire                     idu_iu_ex1_src2_ready,
    output wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_dst0_reg,
    output wire [1:0]               idu_iu_ex1_bht_pred,
    output wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_src0_reg,
    output wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_src1_reg,
    // Task 7.3: the EX1 instruction's LENGTH (1 = 32-bit, 0 = 16-bit RVC),
    // latched with the rest of the EX1 data. Donor ref: aq_iu_bju.v:127
    // `input idu_iu_ex1_inst_len` (bju.v's PC-generator/writeback length
    // source) and aq_idu_id_dp.v's ex1_t. Closes the IU.v header's
    // documented "fixed +4" scope gap (see that file's Task 7.3 note).
    output wire                     idu_iu_ex1_inst_len,
    // M7 Task 2: the EX1-resident instruction's execute-trigger halt_info,
    // piped to the RTU through the IU (the IDU has no direct RTU port).
    // The IU passes it through combinationally (like the ALU's own
    // dispatch-operand style); it is stable for multi-cycle IU residency
    // because ctrl_ex1_eu_full (above) holds the EX1 reload, so the
    // IDU's ex1_halt_info_r keeps the RESIDENT op's value every dp cycle.
    output wire [TDT_HINFO_WIDTH-1:0] idu_iu_ex1_halt_info,

    //=========================================================================
    // IDU -> LSU : EX1 dispatch, LSU's slice of id_ex1_t (matches LSU.v's
    // input port group exactly -- see the header note on the "dp_sel" name).
    //=========================================================================
    output wire                     idu_lsu_ex1_dp_sel,
    output wire                     idu_lsu_ex1_sel,
    // M4 Task 5 fix (entry-cycle gap): `idu_lsu_ex1_sel` minus its own
    // `!lsu_idu_full` gate -- LSU needs this UNGATED "an LSU op truly sits
    // in EX1 right now" signal to detect a fresh DTLB-miss entry without
    // circularity (lsu_idu_full itself must not appear on the input side of
    // that detection -- see LSU.v's `ag_raw_ready` comment).
    output wire                     idu_lsu_ex1_raw_vld,
    output wire [FUNC_WIDTH-1:0]    idu_lsu_ex1_func,
    output wire [63:0]              idu_lsu_ex1_src0_data,
    output wire                     idu_lsu_ex1_src0_ready,
    output wire [63:0]              idu_lsu_ex1_src1_data,
    output wire                     idu_lsu_ex1_src1_ready,
    output wire [63:0]              idu_lsu_ex1_src2_data,
    output wire                     idu_lsu_ex1_src2_ready,
    output wire [GPR_IDX_WIDTH-1:0] idu_lsu_ex1_dst0_reg,
    // M5 Task 4c: 1 for FLW/FLD (destination is FRF, not GPR) -- see D8.
    output wire                     idu_lsu_ex1_dst0_frf,
    // Task 7.3: EX1 instruction length (1=32b,0=16b) for the LSU slice --
    // the completing-LSU-instruction length the RTU's pcgen inst_len mux
    // needs (aq_rtu_dp.v:367 `dp_ex1_inst_len = lsu_rtu_ex1_inst_len`).
    output wire                     idu_lsu_ex1_inst_len,

    //=========================================================================
    // IDU -> CSR : EX1 dispatch, CSR's slice of id_ex1_t (matches CSR.v's
    // input port group exactly).
    //=========================================================================
    output wire                     idu_cp0_ex1_sel,
    output wire [FUNC_WIDTH-1:0]    idu_cp0_ex1_func,
    output wire [31:0]              idu_cp0_ex1_opcode,
    output wire                     idu_cp0_ex1_illegal,
    // M4 Task 6 (S13, D2): fetch-fault siblings of idu_cp0_ex1_illegal --
    // CSR.v traps vec 12/1 instead of vec 2 when either is set (donor
    // aq_cp0_iui.v:645-666's own priority: pgflt > accflt > illegal).
    output wire                     idu_cp0_ex1_fetch_pgflt,
    output wire                     idu_cp0_ex1_fetch_accflt,
    output wire [63:0]              idu_cp0_ex1_src0_data,
    output wire [63:0]              idu_cp0_ex1_src1_data,
    output wire [GPR_IDX_WIDTH-1:0] idu_cp0_ex1_dst0_reg,
    // Task 7.3: EX1 instruction length (1=32b,0=16b) for the CSR slice
    // (c.ebreak is 16-bit). aq_rtu_dp.v:371 cp0 arm.
    output wire                     idu_cp0_ex1_inst_len,

    //=========================================================================
    // IDU -> FPU : M5 Task 2 (D2/D-M5-3) -- FRF read-port outputs, latched
    // the same EX1 cycle as the GPR ex1_src*_data_r trio (SECTION FRF EX1
    // LATCH below). NOT YET CONNECTED to any consumer -- FPU.v doesn't
    // exist until Task 3, so these dangle at the RVProc.v instantiation
    // until then (safe: `-Wno-fatal`, test/m2/unit/Makefile:31). No
    // OP-FP/LOAD-FP/STORE-FP decode arm exists yet either (Task 4 owns
    // all of those) -- these three read fixed FP instruction-field
    // positions unconditionally (SECTION FRF below), exactly like
    // imm_i/imm_s/etc in SECTION DECODE.
    //=========================================================================
    output wire [63:0]              idu_fpu_ex1_fsrc0_data,
    output wire [63:0]              idu_fpu_ex1_fsrc1_data,
    output wire [63:0]              idu_fpu_ex1_fsrc2_data,

    // M5 Task 4: FALU sub-block dispatch selects + shared func/rm bus.
    // Mirrors idu_iu_ex1_alu_sel's shape exactly (ex1_eu_r[EU_FP_SEL] &&
    // !stall && commit), split three ways by which func bits are set
    // (fadd: add/sub/cmp/minmax; fspu: sign-inject/classify; fcnvt: f2f
    // convert). Task 4a's FEQ/FLT/FLE arms set FUNC_CMP (fadd_sel) and its
    // FCLASS arms set FUNC_CLASS (fspu_sel) -- fcnvt_sel alone stays
    // permanently 0 until Task 4b/7 add the arms that set FUNC_CVT_*.
    // func reuses ex1_func_r (already EU-agnostic, same as idu_iu_ex1_func/
    // idu_lsu_ex1_func/idu_cp0_ex1_func); rm is unused by compare/classify
    // (Task 4a) and tied 0 until Task 4b's arithmetic arms decode it.
    // NB: all of this is LIVE as of THE SWAP (M5 Task 9): the FP decode
    // arms no longer hardwire d32_illegal, so ex1_eu_r[EU_FP_SEL] now sets
    // for legal FP encodings and these selects fire. dis_eu_final (below)
    // still forces EU_CP0 for genuinely-illegal encodings, unchanged.
    output wire                     idu_fpu_ex1_fadd_sel,
    output wire                     idu_fpu_ex1_fspu_sel,
    output wire                     idu_fpu_ex1_fcnvt_sel,
    // M5 Task 5: FMAU dispatch select. Identified by FUNC_MAU_MUL alone
    // (rvproc_pkg.sv) -- unlike the three siblings above, FUSED/SUB/NEG
    // can't serve as FMAU's own identifying OR-group (plain fmul sets
    // FUSED=0 with SUB/NEG don't-care), so IDU sets one dedicated bit
    // uniformly across all five FMAU-class instructions instead.
    output wire                     idu_fpu_ex1_fmau_sel,
    // M5 Task 6: FDSU (div/sqrt) dispatch select. Identified by
    // FUNC_FDSU_DIV | FUNC_FDSU_SQRT (rvproc_pkg.sv), same OR-group shape
    // as idu_fpu_ex1_fadd_sel's ADD/SUB/CMP/MAX/MIN -- AND-gated on
    // !FUNC_MAU_MUL because FUSED(5)/SUB(7) are aliased with the FMA
    // fused flags and every fused FMA op sets them (M5 Task 11 fix).
    // Additionally gated on !fpu_idu_fdsu_full (FPU.v's busy-flop output)
    // -- same shape as idu_iu_ex1_div_sel's own !iu_idu_div_full term
    // (IU.v SECTION DIV is this signal's structural precedent, design doc
    // D1).
    output wire                     idu_fpu_ex1_fdsu_sel,
    output wire [FUNC_WIDTH-1:0]    idu_fpu_ex1_func,
    output wire [2:0]               idu_fpu_ex1_rm,
    // M5 Task 4b: destination register tag (rd), same pass-through shape as
    // idu_iu_ex1_dst0_reg/idu_cp0_ex1_dst0_reg -- FPU forwards it unchanged
    // to fpu_rtu_ex1_falu_preg for RTU's FRF/GPR writeback routing.
    output wire [GPR_IDX_WIDTH-1:0] idu_fpu_ex1_dst0_reg,

    //=========================================================================
    // RTU -> IDU : the exclusive bypass network (IDU note S6) + the 2
    // architectural commit ports.
    //=========================================================================
    input  wire [63:0]              rtu_idu_fwd0_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd0_reg,
    input  wire                     rtu_idu_fwd0_vld,
    input  wire [63:0]              rtu_idu_fwd1_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd1_reg,
    input  wire                     rtu_idu_fwd1_vld,
    input  wire [63:0]              rtu_idu_fwd2_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd2_reg,
    input  wire                     rtu_idu_fwd2_vld,
    input  wire [63:0]              rtu_idu_wb0_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb0_reg,
    input  wire                     rtu_idu_wb0_vld,
    input  wire [63:0]              rtu_idu_wb1_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb1_reg,
    input  wire                     rtu_idu_wb1_vld,
    // M5 Task 2 (D2): FRF write ports, sized/split like GPR's wb0/wb1
    // rather than the donor's single write port (aq_vidu_vid_gpr_fp.v:28-30
    // `vpu_vidu_fp_wb_reg/_data/_vld`, one port only, servicing the vector
    // issue slot) -- rv906 mirrors RTU.v's OWN wb0/wb1 producer split
    // instead (RTU.v:46-68: wb0 = arbitrated ALU/BJU/CP0/DIV/MUL compute
    // result, wb1 = dedicated, uncontested LSU load result): wbf0 will
    // carry the FALU/FMAU/FDSU compute result (Task 3/5/6/7), wbf1 the
    // dedicated FLW/FLD load result (Task 4/D8), the same compute-vs-LSU
    // split GPR already uses. NOT YET DRIVEN -- RTU.v gains no FPU-facing
    // ports until Task 3+; tied 0 at the RVProc.v instantiation until then.
    input  wire [63:0]              rtu_idu_wbf0_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_wbf0_reg,
    input  wire                     rtu_idu_wbf0_vld,
    input  wire [63:0]              rtu_idu_wbf1_data,
    input  wire [GPR_IDX_WIDTH-1:0] rtu_idu_wbf1_reg,
    input  wire                     rtu_idu_wbf1_vld,

    //=========================================================================
    // IU -> IDU : point-to-point stall/full signals (contract 8; matches
    // IU.v's output group exactly).
    //=========================================================================
    input  wire                     iu_idu_mult_issue_stall,
    input  wire                     iu_idu_mult_full,
    input  wire                     iu_idu_div_full,
    input  wire                     iu_idu_bju_full,
    input  wire                     iu_idu_bju_global_full,

    //=========================================================================
    // FPU -> IDU : FDSU busy/full (M5 Task 6, mirrors iu_idu_div_full's
    // shape exactly -- design doc D1's structural precedent).
    //=========================================================================
    input  wire                     fpu_idu_fdsu_full,

    //=========================================================================
    // LSU -> IDU : the single EX1 issue-gate stall signal (contract 8;
    // confirmed name `lsu_idu_full`, aq_idu_id_ctrl.v:634).
    //=========================================================================
    input  wire                     lsu_idu_full,

    //=========================================================================
    // CP0 -> IDU : FENCE/FENCE.I EX1-hold backpressure (Task 10.1,
    // rv64ui-p-fence_i): held while the fence waits for LSU quiescence;
    // folds into ctrl_ex1_eu_full below so EX1 keeps the fence.
    //=========================================================================
    input  wire                     cp0_idu_fencei_full,

    //=========================================================================
    // RTU -> IDU : flush/drain group (RTU note S6). `rtu_idu_pipeline_empty`
    // is a Task 5 port-list amendment closing RTU.v's Task 4 loop -- see
    // header; not consumed by any logic below (landing pad only).
    //=========================================================================
    input  wire                     rtu_idu_flush_fe,
    input  wire                     iu_idu_br_cancel,    // donor aq_idu_id_ctrl.v:604 (iu_yy_xx_cancel); ORs into the EX1-inst-valid cancel
    input  wire                     rtu_idu_flush_stall,
    input  wire                     rtu_idu_flush_wbt,
    input  wire                     rtu_idu_commit,
    input  wire                     rtu_idu_commit_for_bju,
    input  wire                     rtu_idu_pipeline_empty,
    // M7 Task 1: debug-mode broadcast for the dret legality rule (donor
    // aq_idu_id_decd.v:857 -- dret outside debug is illegal). Wired to
    // rtu_yy_xx_dbgon at RVProc.v; tied 1 in the IDU unit bench (whose
    // pre-existing rows never decode dret anyway -- OFF-path identity).
    input  wire                     rtu_yy_xx_dbgon
);

    //=========================================================================
    // FEEDBACK SECTION (umbrella S6.2 rule 4) -- every signal flowing
    // backward against the nominal IFU->IDU->IU/LSU/CP0 flow:
    //   rtu_idu_fwd0/1/2_*, rtu_idu_wb0/1_*   <- RTU, the exclusive bypass
    //                                            network (IDU note S6)
    //   iu_idu_mult_issue_stall/_mult_full/
    //     _div_full/_bju_full/_bju_global_full <- IU, EX1 issue-gate inputs
    //   lsu_idu_full                          <- LSU (not built until Task
    //                                            6; tied 0 in the bench's
    //                                            idle stimulus)
    //   rtu_idu_flush_fe/_flush_stall/_flush_wbt/
    //     _commit/_commit_for_bju/_pipeline_empty <- RTU, flush/drain group
    //=========================================================================

    //=========================================================================
    // SECTION DECODE (IDU note S3) -- ID/DIS is one combinational stage
    // (IDU note S2). `is32` mirrors the donor's `decd_length`
    // (decd.v:448), just written the RVC-detect way round. Every mnemonic
    // NOT explicitly covered below (FP/FP-load-store, vector, C-SKY
    // custom-0 cache/perf, AMO/LR/SC, sfence.vma, sret/wfi/dret) falls to
    // the `default:` arm of whichever casez it lands in and is flagged
    // illegal -- this IS the closed illegal-decode list Task 5.1/5.5 ask
    // for; no separate classifier/sub-decoder exists for any of them
    // because every one of them is out of M2's scope (contract 10, design
    // doc S2.3.4) and none has a case arm to match.
    //=========================================================================
    wire [31:0] inst = ifu_idu_id_inst;
    wire        is32 = (inst[1:0] == 2'b11);

    // ---- standard RV32/64 immediate formats, computed once, reused by
    // every 32-bit arm that needs one (decd.v:544-545,551,598-601 verbatim
    // formulas, confirmed by Task 5's research pass) ----
    wire [63:0] imm_i = {{52{inst[31]}}, inst[31:20]};
    wire [63:0] imm_s = {{53{inst[31]}}, inst[30:25], inst[11:7]};
    wire [63:0] imm_b = {{51{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    wire [63:0] imm_j = {{44{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    wire [63:0] imm_u = {{32{inst[31]}}, inst[31:12], 12'b0};

    // ---- RVC immediate formats, computed once (decd.v:528-533,552-559,
    // 604-606 verbatim formulas, confirmed by Task 5's research pass).
    // c_imm6 is unmasked (matches decd.v's own unmasked case 14'h04 --
    // masking only applied by the donor for RVC branches/c.jr/c.jalr, which
    // this file handles by simply not using c_imm6 for those forms at all,
    // see the RVC decode block below) ----
    wire [63:0] c_imm6        = {{58{inst[12]}}, inst[12], inst[6:2]};
    wire [63:0] c_lui_imm     = {{46{inst[12]}}, inst[12], inst[6:2], 12'b0};
    wire [63:0] c_addi4spn_imm = {54'b0, inst[10:7], inst[12:11], inst[5], inst[6], 2'b0};
    wire [9:0]  c_addi16sp_raw = {inst[12], inst[4:3], inst[5], inst[2], inst[6], 4'b0};
    wire [63:0] c_addi16sp_imm = {{54{c_addi16sp_raw[9]}}, c_addi16sp_raw};
    // BUG FIX (rv64uc-p-rvc test 18): these two formulas were SWAPPED --
    // c_lw_imm carried the C.LWSP (CI) encoding and c_lwsp_imm the C.LW (CL)
    // one, so c.sw computed base+offset with offset bits drawn from the
    // rs2'/rd' fields (store landed +0xC0 away, rv64uc-p-rvc test 18).
    // C.LW/C.SW (CL/CS, RVC spec): offset = {inst[5], inst[12:10], inst[6], 2'b0}.
    wire [63:0] c_lw_imm       = {57'b0, inst[5], inst[12:10], inst[6], 2'b0};
    wire [63:0] c_ld_imm       = {56'b0, inst[6:5], inst[12:10], 3'b0};
    // C.LWSP (CI, RVC spec): offset = {inst[3:2], inst[12], inst[6:4], 2'b0}.
    wire [63:0] c_lwsp_imm     = {56'b0, inst[3:2], inst[12], inst[6:4], 2'b0};
    wire [63:0] c_swsp_imm     = {56'b0, inst[8:7], inst[12:9], 2'b0};
    wire [63:0] c_ldsp_imm     = {55'b0, inst[4:2], inst[12], inst[6:5], 3'b0};
    wire [63:0] c_sdsp_imm     = {55'b0, inst[9:7], inst[12:10], 3'b0};
    wire [63:0] c_j_imm  = {{52{inst[12]}}, inst[12], inst[8], inst[10:9], inst[6], inst[7],
                            inst[2], inst[11], inst[5:3], 1'b0};
    wire [63:0] c_br_imm = {{55{inst[12]}}, inst[12], inst[6:5], inst[2], inst[11:10],
                            inst[4:3], 1'b0};

    // ---- RVC register-field extraction (standard RV64C encoding; the
    // compressed 3-bit forms address x8-x15, IDU note S3.4-adjacent) ----
    wire [4:0] c_rd_rs1_5 = inst[11:7];              // full 5-bit rd/rs1 field
    wire [4:0] c_rs2_5    = inst[6:2];                // full 5-bit rs2 field
    wire [4:0] c_rd_rs1_3 = {2'b01, inst[9:7]};        // rd'/rs1' (+8)
    wire [4:0] c_rs2_3    = {2'b01, inst[4:2]};        // rs2'/rd' (+8, loads use it as rd')

    // ---- 32-bit main decode (decd.v:1541-2169's casez, keyed identically
    // for direct traceability) ----
    reg [EU_WIDTH-1:0]   d32_eu;
    reg [FUNC_WIDTH-1:0] d32_func;
    reg                  d32_illegal;
    reg                  d32_src0_vld, d32_src1_vld, d32_src1_imm_vld;
    reg                  d32_src2_vld, d32_src2_imm_vld, d32_dst0_vld;
    reg [4:0]            d32_src0_reg, d32_src1_reg, d32_src2_reg, d32_dst0_reg;
    reg [63:0]           d32_src1_imm, d32_src2_imm;
    // M5 Task 4c: LOAD-FP/STORE-FP destination/source-data tags -- FRF, not
    // GPR, so they stay separate from dst0_vld/src2_vld (which would create
    // a GPR wbt entry / route src2 through the GPR forward mux). No FRF
    // scoreboard exists in this design, so these need no _vld-style
    // readiness companion of their own.
    reg                  d32_dst0_frf, d32_src2_frf;

    always @* begin
        // default init (decd.v:1519-1539's own top-of-block zero-init
        // pattern) -- every field not touched by the matched arm stays 0.
        d32_eu           = {EU_WIDTH{1'b0}};
        d32_func         = {FUNC_WIDTH{1'b0}};
        d32_illegal      = 1'b0;
        d32_src0_vld     = 1'b0;
        d32_src1_vld     = 1'b0;
        d32_src1_imm_vld = 1'b0;
        d32_src2_vld     = 1'b0;
        d32_src2_imm_vld = 1'b0;
        d32_dst0_vld     = 1'b0;
        d32_dst0_frf     = 1'b0;
        d32_src2_frf     = 1'b0;
        d32_src0_reg     = inst[19:15];   // rs1 field, unconditionally (decd.v:666)
        d32_src1_reg     = inst[24:20];   // rs2 field (R-type slot)
        d32_src2_reg     = inst[24:20];   // rs2 field (store-data slot)
        d32_dst0_reg     = inst[11:7];    // rd field, unconditionally (decd.v:741-742)
        d32_src1_imm     = 64'd0;
        d32_src2_imm     = 64'd0;

        casez ({inst[31:25], inst[14:12], inst[6:2]})
            15'b?????_?????01101: begin  // lui
                d32_eu = EU_ALU; d32_func = ALU_FUNC_LUI;
                d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_u; d32_dst0_vld = 1'b1;
            end
            15'b??????????00101: begin  // auipc
                d32_eu = EU_BJU; d32_func = BJU_FUNC_AUIPC;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_u; d32_dst0_vld = 1'b1;
            end
            15'b??????????11011: begin  // jal
                d32_eu = EU_BJU; d32_func = BJU_FUNC_JAL;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_j; d32_dst0_vld = 1'b1;
            end
            15'b???????00011001: begin  // jalr
                d32_eu = EU_BJU; d32_func = BJU_FUNC_JALR;
                d32_src0_vld = 1'b1; d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????00011000: begin  // beq
                d32_eu = EU_BJU; d32_func = BJU_FUNC_BEQ;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_b;
            end
            15'b???????00111000: begin  // bne
                d32_eu = EU_BJU; d32_func = BJU_FUNC_BNE;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_b;
            end
            15'b???????10011000: begin  // blt
                d32_eu = EU_BJU; d32_func = BJU_FUNC_BLT;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_b;
            end
            15'b???????10111000: begin  // bge
                d32_eu = EU_BJU; d32_func = BJU_FUNC_BGE;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_b;
            end
            15'b???????11011000: begin  // bltu
                d32_eu = EU_BJU; d32_func = BJU_FUNC_BLTU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_b;
            end
            15'b???????11111000: begin  // bgeu
                d32_eu = EU_BJU; d32_func = BJU_FUNC_BGEU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1;
                d32_src2_imm_vld = 1'b1; d32_src2_imm = imm_b;
            end
            15'b???????00000000: begin  // lb
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LB;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????00100000: begin  // lh
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LH;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????01000000: begin  // lw
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????01100000: begin  // ld
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LD;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????10000000: begin  // lbu
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LBU;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????10100000: begin  // lhu
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LHU;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????11000000: begin  // lwu
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LWU;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????00001000: begin  // sb
                d32_eu = EU_LSU; d32_func = LSU_FUNC_SB;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_s;
                d32_src2_vld = 1'b1;
            end
            15'b???????00101000: begin  // sh
                d32_eu = EU_LSU; d32_func = LSU_FUNC_SH;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_s;
                d32_src2_vld = 1'b1;
            end
            15'b???????01001000: begin  // sw
                d32_eu = EU_LSU; d32_func = LSU_FUNC_SW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_s;
                d32_src2_vld = 1'b1;
            end
            15'b???????01101000: begin  // sd
                d32_eu = EU_LSU; d32_func = LSU_FUNC_SD;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_s;
                d32_src2_vld = 1'b1;
            end
            //-----------------------------------------------------------------
            // M5 Task 4c: LOAD-FP/STORE-FP (opcode 0000111/0100111 ->
            // inst[6:2]=00001/01001). D8: reuses the exact same EU_LSU
            // AG/DC/STB/LFB machinery as the integer loads/stores above
            // (donor FUNC low-nibble bit-exact, see rvproc_pkg.sv). The base
            // address (src0/src1_imm) works exactly like lw/sw; only the
            // destination (loads) / data operand (stores) differs: FRF, not
            // GPR, so dst0_vld/src2_vld stay 0 here (no GPR wbt entry, no
            // GPR forward-mux routing) and dst0_frf/src2_frf mark the FRF
            // side instead (dst0_frf consumed by LSU.v's new dst0_frf tag;
            // src2_frf overrides dis_src2_data to frf_src1_data below, SECTION
            // FRF). Live as of THE SWAP (M5 Task 9, D11): these now decode
            // as legal EU_LSU ops (d32_illegal left 0), exactly like the
            // OP-FP arms above.
            15'b???????01000001: begin  // flw
                d32_eu = EU_LSU; d32_func = LSU_FUNC_FLW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_frf = 1'b1;
            end
            15'b???????01100001: begin  // fld
                d32_eu = EU_LSU; d32_func = LSU_FUNC_FLD;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_frf = 1'b1;
            end
            15'b???????01001001: begin  // fsw
                d32_eu = EU_LSU; d32_func = LSU_FUNC_FSW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_s;
                d32_src2_frf = 1'b1;
            end
            15'b???????01101001: begin  // fsd
                d32_eu = EU_LSU; d32_func = LSU_FUNC_FSD;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_s;
                d32_src2_frf = 1'b1;
            end
            //-----------------------------------------------------------------
            // M3 Task 7: RV64A atomics (opcode 0x2F -> inst[6:2]=01011).
            // key = {inst[31:25](funct7), inst[14:12](funct3), inst[6:2]}.
            // Fixed from 14 bits to 15 bits: AMOs have opcode 01011 at bottom,
            // with funct3=010 for W-width or 011 for D-width. LR funct5=00010,
            // SC funct5=00011; the amo* catch-all below covers the rest.
            //-----------------------------------------------------------------
            15'b00010??01001011: begin  // lr.w (funct5=00010, funct3=010)
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LR_W;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = 64'd0;
                d32_dst0_vld = 1'b1;
            end
            15'b00010??01101011: begin  // lr.d (funct3=011)
                d32_eu = EU_LSU; d32_func = LSU_FUNC_LR_D;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = 64'd0;
                d32_dst0_vld = 1'b1;
            end
            15'b00011??01001011: begin  // sc.w (funct5=00011, funct3=010)
                d32_eu = EU_LSU; d32_func = LSU_FUNC_SC_W;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = 64'd0;
                d32_src2_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b00011??01101011: begin  // sc.d (funct3=011)
                d32_eu = EU_LSU; d32_func = LSU_FUNC_SC_D;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = 64'd0;
                d32_src2_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b???????01001011,   // amo*.w (funct5 != lr/sc matched above)
            15'b???????01101011: begin  // amo*.d
                // M3 audit: only the NINE defined AMO funct5s decode here.
                // The donor's decode table lists exactly these (aq_idu_id_
                // decd.v:2028-2052); reserved funct5 values fall through to
                // illegal there. rv906's catch-all previously accepted them
                // and amo_alu_compute returned 0 -- executing a reserved
                // encoding as "store zero". Trap as illegal instead.
                casez (inst[31:27])
                    5'b00000, 5'b00001, 5'b00100, 5'b01000, 5'b01100,
                    5'b10000, 5'b10100, 5'b11000, 5'b11100: begin
                        d32_eu = EU_LSU;
                        // func = {AMO prefix 0x01, 3'b000, funct5, width, 2'b00};
                        // width = funct3[1:0] (10=W, 11=D) lands in func[3:2].
                        d32_func = {8'h01, 3'b000, inst[31:27], inst[13:12], 2'b00};
                        d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = 64'd0;
                        d32_src2_vld = 1'b1; d32_dst0_vld = 1'b1;
                    end
                    default: d32_illegal = 1'b1;
                endcase
            end
            15'b???????00000100: begin  // addi
                d32_eu = EU_ALU; d32_func = ALU_FUNC_ADD;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????01000100: begin  // slti
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLT;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????01100100: begin  // sltiu
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLTU;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????10000100: begin  // xori
                d32_eu = EU_ALU; d32_func = ALU_FUNC_XOR;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????11000100: begin  // ori
                d32_eu = EU_ALU; d32_func = ALU_FUNC_OR;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????11100100: begin  // andi
                d32_eu = EU_ALU; d32_func = ALU_FUNC_AND;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b000000?00100100: begin  // slli
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLL;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b000000?10100100: begin  // srli
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRL;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b010000?10100100: begin  // srai
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRA;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????00000110: begin  // addiw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_ADDW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b000000000100110: begin  // slliw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLLW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b000000010100110: begin  // srliw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRLW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b010000010100110: begin  // sraiw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRAW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b000000000001110: begin  // addw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_ADDW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b010000000001110: begin  // subw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SUBW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000000101110: begin  // sllw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLLW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000010101110: begin  // srlw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRLW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b010000010101110: begin  // sraw
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRAW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000000001100: begin  // add
                d32_eu = EU_ALU; d32_func = ALU_FUNC_ADD;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b010000000001100: begin  // sub
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SUB;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000000101100: begin  // sll
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLL;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000001001100: begin  // slt
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLT;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000001101100: begin  // sltu
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SLTU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000010001100: begin  // xor
                d32_eu = EU_ALU; d32_func = ALU_FUNC_XOR;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000010101100: begin  // srl
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRL;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b010000010101100: begin  // sra
                d32_eu = EU_ALU; d32_func = ALU_FUNC_SRA;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000011001100: begin  // or
                d32_eu = EU_ALU; d32_func = ALU_FUNC_OR;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000011101100: begin  // and
                d32_eu = EU_ALU; d32_func = ALU_FUNC_AND;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000100001100: begin  // mul
                d32_eu = EU_MULT; d32_func = MULT_FUNC_MUL;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000100101100: begin  // mulh
                d32_eu = EU_MULT; d32_func = MULT_FUNC_MULH;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000101001100: begin  // mulhsu
                d32_eu = EU_MULT; d32_func = MULT_FUNC_MULHSU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000101101100: begin  // mulhu
                d32_eu = EU_MULT; d32_func = MULT_FUNC_MULHU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000110001100: begin  // div
                d32_eu = EU_DIV; d32_func = DIV_FUNC_DIV;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000110101100: begin  // divu
                d32_eu = EU_DIV; d32_func = DIV_FUNC_DIVU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000111001100: begin  // rem
                d32_eu = EU_DIV; d32_func = DIV_FUNC_REM;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000111101100: begin  // remu
                d32_eu = EU_DIV; d32_func = DIV_FUNC_REMU;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000100001110: begin  // mulw
                d32_eu = EU_MULT; d32_func = MULT_FUNC_MULW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000110001110: begin  // divw
                d32_eu = EU_DIV; d32_func = DIV_FUNC_DIVW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000110101110: begin  // divuw
                d32_eu = EU_DIV; d32_func = DIV_FUNC_DIVUW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000111001110: begin  // remw
                d32_eu = EU_DIV; d32_func = DIV_FUNC_REMW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b000000111101110: begin  // remuw
                d32_eu = EU_DIV; d32_func = DIV_FUNC_REMUW;
                d32_src0_vld = 1'b1; d32_src1_vld = 1'b1; d32_dst0_vld = 1'b1;
            end
            15'b???????00000011: begin d32_eu = EU_CP0; d32_func = CP0_FUNC_FENCE;  end  // fence
            15'b???????00100011: begin d32_eu = EU_CP0; d32_func = CP0_FUNC_FENCEI; end  // fence.i
            15'b000000000011100: begin  // ecall / ebreak (rs1/rd must be 0, decd_i_illegal-equiv)
                d32_eu      = EU_CP0;
                d32_func    = inst[20] ? CP0_FUNC_EBREAK : CP0_FUNC_ECALL;
                d32_illegal = (inst[19:15] != 5'd0) || (inst[11:7] != 5'd0);
            end
            15'b001100000011100: begin  // mret (rs1/rd must be 0)
                d32_eu      = EU_CP0;
                d32_func    = CP0_FUNC_MRET;
                d32_illegal = (inst[19:15] != 5'd0) || (inst[11:7] != 5'd0);
            end
            // M7 Task 1: dret (donor aq_idu_id_decd.v:2110-2114; funct7=
            // 0111101, funct3=000, SYSTEM -- the standard 0x7B200073
            // encoding). Legality rule per donor decd.v:857: a dret OUTSIDE
            // debug mode is illegal ({rs2,rd} must also be 0).
            15'b011110100011100: begin  // dret
                d32_eu      = EU_CP0;
                d32_func    = CP0_FUNC_DRET;
                d32_illegal = (inst[19:15] != 5'd0) || (inst[11:7] != 5'd0)
                            || !rtu_yy_xx_dbgon;
            end
            // M4: sret / wfi share funct7=0001000, funct3=000, split by rs2
            // (rs2=2 sret, rs2=5 wfi). Privilege-based legality (TSR/TW/U-
            // mode) is checked in CSR.v, not here.
            15'b000100000011100: begin
                d32_eu      = EU_CP0;
                d32_func    = (inst[24:20] == 5'd5) ? CP0_FUNC_WFI : CP0_FUNC_SRET;
                d32_illegal = (inst[19:15] != 5'd0) || (inst[11:7] != 5'd0);
            end
            // M4: sfence.vma (funct7=0001001). rs1=VA, rs2=ASID; rv906 over-
            // invalidates (whole TLB), so the operands are ignored; legality
            // (U-mode / S-with-TVM) is checked in CSR.v.
            15'b000100100011100: begin
                d32_eu      = EU_CP0;
                d32_func    = CP0_FUNC_SFENCE;
                d32_illegal = (inst[11:7] != 5'd0);
            end
            15'b???????00111100: begin  // csrrw
                d32_eu = EU_CP0; d32_func = CP0_FUNC_CSRRW;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????01011100: begin  // csrrs
                d32_eu = EU_CP0; d32_func = CP0_FUNC_CSRRS;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????01111100: begin  // csrrc
                d32_eu = EU_CP0; d32_func = CP0_FUNC_CSRRC;
                d32_src0_vld = 1'b1; d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i;
                d32_dst0_vld = 1'b1;
            end
            15'b???????10111100: begin  // csrrwi
                d32_eu = EU_CP0; d32_func = CP0_FUNC_CSRRWI;
                d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i; d32_dst0_vld = 1'b1;
            end
            15'b???????11011100: begin  // csrrsi
                d32_eu = EU_CP0; d32_func = CP0_FUNC_CSRRSI;
                d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i; d32_dst0_vld = 1'b1;
            end
            15'b???????11111100: begin  // csrrci
                d32_eu = EU_CP0; d32_func = CP0_FUNC_CSRRCI;
                d32_src1_imm_vld = 1'b1; d32_src1_imm = imm_i; d32_dst0_vld = 1'b1;
            end
            // M5 Task 4 (part a): OP-FP compare/classify -- xvld-class, GPR
            // destination (rd, via the EXISTING d32_dst0_vld/_reg path,
            // default-set to inst[11:7] above -- unchanged GPR WBT/wb0/wb1
            // machinery). Sources are FRF-read (idu_fpu_ex1_fsrc0/1_data,
            // SECTION FRF -- unconditional reads off inst[19:15]/[24:20],
            // no GPR src_vld gating needed since GPR is never read here).
            // d32_illegal is 0 for these legal FP encodings as of THE SWAP
            // (M5 Task 9, D11): misa.F/D are now 1 and the FP arms no
            // longer hardwire illegal=1, so d32_eu=EU_FP now reaches the
            // FPU. dis_eu_final (SECTION EU DISPATCH, below) still forces
            // EU_CP0 whenever dis_illegal is set (genuinely-illegal
            // encodings only). rs2=00000 for fclass is NOT separately
            // checked (harmless).
            15'b1010000_010_10100: begin  // feq.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_CMP] = 1'b1; d32_func[FUNC_CMP_FEQ] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1010000_001_10100: begin  // flt.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_CMP] = 1'b1; d32_func[FUNC_CMP_LT] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1010000_000_10100: begin  // fle.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_CMP] = 1'b1; d32_func[FUNC_CMP_LE] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1010001_010_10100: begin  // feq.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_CMP] = 1'b1; d32_func[FUNC_CMP_FEQ] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1010001_001_10100: begin  // flt.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_CMP] = 1'b1; d32_func[FUNC_CMP_LT] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1010001_000_10100: begin  // fle.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_CMP] = 1'b1; d32_func[FUNC_CMP_LE] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1110000_001_10100: begin  // fclass.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_CLASS] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1110001_001_10100: begin  // fclass.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_CLASS] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            // M5 Task 4 (part b): OP-FP add/sub/minmax/sgnj/f2f-convert --
            // FRF-destination (unlike part a's GPR-destined compare/classify).
            // d32_dst0_vld is deliberately left 0 here: fpu_rtu_ex1_falu_fvld
            // (FPU.v SECTION 12) is what marks these FRF-bound, computed
            // there from the same FUNC bits set below -- no separate
            // "FRF dest valid" decode signal is needed. d32_dst0_reg (the rd
            // field, default-set above) still flows through unconditionally
            // to idu_fpu_ex1_dst0_reg for the eventual FRF writeback tag.
            // Live as of THE SWAP (M5 Task 9): legal FP encodings now
            // dispatch to EU_FP (see part a's note above).
            // M5 Task 11 BUG 4 (rv64ud-p/v-move test 40, Category A --
            // rv906-own decode gap; the public C906 factory ships no scalar
            // FPU, so no donor precedent): every single-precision FP arm
            // sets FUNC_B_SINGLE -- FPU.v's NaN-box check (box_check_en =
            // f_single, FPU.v:354; a_cnan/b_cnan -> canonical-NaN
            // substitution) was enabled ONLY by fcvt.d.s, so single ops on
            // a broken-box 64-bit source (fsgnj.s on a qNaN double) passed
            // the raw low 32 bits through instead of canonicalizing.
            // No-op for every properly-boxed value (a_cnan=0), so all
            // previously passing ELFs are bit-identical; the double arms
            // set FUNC_DOUBLE instead (and must NOT set B_SINGLE).
            15'b0000000_???_10100: begin  // fadd.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_ADD] = 1'b1;
            end
            15'b0000001_???_10100: begin  // fadd.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_ADD] = 1'b1;
            end
            15'b0000100_???_10100: begin  // fsub.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_SUB] = 1'b1;
            end
            15'b0000101_???_10100: begin  // fsub.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_SUB] = 1'b1;
            end
            15'b0010100_000_10100: begin  // fmin.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MIN] = 1'b1;
            end
            15'b0010100_001_10100: begin  // fmax.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MAX] = 1'b1;
            end
            15'b0010101_000_10100: begin  // fmin.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_MIN] = 1'b1;
            end
            15'b0010101_001_10100: begin  // fmax.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_MAX] = 1'b1;
            end
            15'b0010000_000_10100: begin  // fsgnj.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_SPU_SGN] = 1'b1; d32_func[FUNC_SPU_SGN_J] = 1'b1;
            end
            15'b0010000_001_10100: begin  // fsgnjn.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_SPU_SGN] = 1'b1; d32_func[FUNC_SPU_SGN_N] = 1'b1;
            end
            15'b0010000_010_10100: begin  // fsgnjx.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_SPU_SGN] = 1'b1; d32_func[FUNC_SPU_SGN_X] = 1'b1;
            end
            15'b0010001_000_10100: begin  // fsgnj.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_SPU_SGN] = 1'b1; d32_func[FUNC_SPU_SGN_J] = 1'b1;
            end
            15'b0010001_001_10100: begin  // fsgnjn.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_SPU_SGN] = 1'b1; d32_func[FUNC_SPU_SGN_N] = 1'b1;
            end
            15'b0010001_010_10100: begin  // fsgnjx.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_SPU_SGN] = 1'b1; d32_func[FUNC_SPU_SGN_X] = 1'b1;
            end
            // fcvt.s.d (double->single, a "narrow" conversion) / fcvt.d.s
            // (single->double, "widen") -- rs2 selects the pair (00001/00000)
            // like fclass's rs2=00000 above; not separately legality-checked
            // (harmless).
            15'b0100000_???_10100: begin  // fcvt.s.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_CVT_NARROW] = 1'b1;
            end
            15'b0100001_???_10100: begin  // fcvt.d.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1; d32_func[FUNC_CVT_WIDDEN] = 1'b1;
            end
            // M5 Task 5: plain fmul (OP-FP, opcode[6:2]=10100, funct5=00010)
            // -- FRF-destined like fadd/fsub above (d32_dst0_vld left 0, same
            // reasoning). FUNC_MAU_FUSED=0 (SUB/NEG don't-care) dispatches
            // FPU.v's non-fused multiply path; FUNC_MAU_MUL identifies this
            // as an FMAU-class op for idu_fpu_ex1_fmau_sel (SECTION EU
            // DISPATCH below).
            15'b0001000_???_10100: begin  // fmul.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1;
            end
            15'b0001001_???_10100: begin  // fmul.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_MAU_MUL] = 1'b1;
            end
            // M5 Task 6: fdiv/fsqrt (OP-FP, funct5=00011/01011). rs2 is
            // fdiv's real second source register for the div arms and
            // fsqrt's fixed-00000 mode field for the sqrt arms -- neither
            // matters here since the 15-bit casez key is {funct7,funct3,
            // opcode} only (rs2, inst[24:20], was never part of the key --
            // see the casez declaration above), so fdiv's rs2 flows to
            // idu_fpu_ex1_fsrc1_data exactly like fadd's real rs2 does,
            // with no wildcarding risk.
            15'b0001100_???_10100: begin  // fdiv.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_FDSU_DIV] = 1'b1;
            end
            15'b0001101_???_10100: begin  // fdiv.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_FDSU_DIV] = 1'b1;
            end
            15'b0101100_???_10100: begin  // fsqrt.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_FDSU_SQRT] = 1'b1;
            end
            15'b0101101_???_10100: begin  // fsqrt.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_FDSU_SQRT] = 1'b1;
            end
            // M5 Task 5: R4-type FMA family (MADD/MSUB/NMSUB/NMADD major
            // opcodes 10000/10001/10010/10011, per LOAD-FP=00001/STORE-FP=
            // 01001's sibling precedent above). rs3=inst[31:27] occupies
            // what would be funct7[6:2] for R-type, and fmt=inst[26:25]
            // occupies funct7[1:0] -- so the 15-bit casez key's top 5 bits
            // (rs3) are wildcarded here, unlike every R-type arm above where
            // that field is a fixed funct7. fmt selects S/D (00/01); rm
            // (inst[14:12]) stays wildcarded exactly like fadd/fsub's `???`
            // above (FPU.v's rounding-mode consumer doesn't gate decode).
            // {NEG,SUB,FUSED} per rvproc_pkg.sv's donor-cited encoding:
            // fmadd=001, fmsub=011, fnmsub=111, fnmadd=101.
            15'b?????00???10000: begin  // fmadd.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
            end
            15'b?????01???10000: begin  // fmadd.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
            end
            15'b?????00???10001: begin  // fmsub.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
                d32_func[FUNC_MAU_SUB] = 1'b1;
            end
            15'b?????01???10001: begin  // fmsub.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
                d32_func[FUNC_MAU_SUB] = 1'b1;
            end
            15'b?????00???10010: begin  // fnmsub.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
                d32_func[FUNC_MAU_SUB] = 1'b1; d32_func[FUNC_MAU_NEG] = 1'b1;
            end
            15'b?????01???10010: begin  // fnmsub.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
                d32_func[FUNC_MAU_SUB] = 1'b1; d32_func[FUNC_MAU_NEG] = 1'b1;
            end
            15'b?????00???10011: begin  // fnmadd.s
                d32_eu = EU_FP; d32_func[FUNC_B_SINGLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
                d32_func[FUNC_MAU_NEG] = 1'b1;
            end
            15'b?????01???10011: begin  // fnmadd.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_MAU_MUL] = 1'b1; d32_func[FUNC_MAU_FUSED] = 1'b1;
                d32_func[FUNC_MAU_NEG] = 1'b1;
            end
            // M5 Task 7: fcvt.{w,wu,l,lu}.{s,d} -- float->int, xvld-class,
            // GPR destination (part-a pattern, like feq/flt/fle/fclass
            // above). rs2 (inst[24:20]) is NOT part of the 15-bit casez
            // key (funct7/funct3/opcode only), so all four of w/wu/l/lu
            // share one arm per FP-source-width and are told apart here
            // by inst[21:20] -- the RISC-V rs2 encoding for this family is
            // itself {wide64,unsigned} (00=w,01=wu,10=l,11=lu), which maps
            // directly onto FUNC_CVT_WIDE64/FUNC_CVT_UNSIGNED with no
            // reordering.
            15'b1100000_???_10100: begin  // fcvt.w/wu/l/lu.s
                d32_eu = EU_FP; d32_func[FUNC_CVT_INT] = 1'b1; d32_func[FUNC_CVT_F2I] = 1'b1;
                d32_func[FUNC_CVT_UNSIGNED] = inst[20]; d32_func[FUNC_CVT_WIDE64] = inst[21];
                d32_dst0_vld = 1'b1;
            end
            15'b1100001_???_10100: begin  // fcvt.w/wu/l/lu.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_CVT_INT] = 1'b1; d32_func[FUNC_CVT_F2I] = 1'b1;
                d32_func[FUNC_CVT_UNSIGNED] = inst[20]; d32_func[FUNC_CVT_WIDE64] = inst[21];
                d32_dst0_vld = 1'b1;
            end
            // fcvt.s/d.{w,wu,l,lu} -- int->float, FRF-destination (part-b
            // pattern: d32_dst0_vld left 0, same reasoning as fadd/fsub).
            15'b1101000_???_10100: begin  // fcvt.s.w/wu/l/lu
                d32_eu = EU_FP; d32_func[FUNC_CVT_INT] = 1'b1;
                d32_func[FUNC_CVT_UNSIGNED] = inst[20]; d32_func[FUNC_CVT_WIDE64] = inst[21];
            end
            15'b1101001_???_10100: begin  // fcvt.d.w/wu/l/lu
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1;
                d32_func[FUNC_CVT_INT] = 1'b1;
                d32_func[FUNC_CVT_UNSIGNED] = inst[20]; d32_func[FUNC_CVT_WIDE64] = inst[21];
            end
            // fmv.x.w/fmv.x.d -- fp->int passthrough, xvld-class, GPR
            // destination (part-a pattern). fmv.w.x/fmv.d.x -- int->fp
            // passthrough, FRF-destination (part-b pattern). rs2=00000
            // (fixed) is not separately checked, same convention as
            // fclass's rs2=00000 above (harmless).
            // FUNC_SPU_SGN is set here (not just FUNC_SPU_MV) so that
            // idu_fpu_ex1_fspu_sel's dispatch OR-term stays on the
            // already-safe FUNC_SPU_SGN bit rather than on FUNC_SPU_MV,
            // which aliases FUNC_MAU_NEG's bit position -- see the
            // fspu_sel comment above.
            15'b1110000_000_10100: begin  // fmv.x.w
                d32_eu = EU_FP; d32_func[FUNC_SPU_SGN] = 1'b1;
                d32_func[FUNC_SPU_MV] = 1'b1; d32_func[FUNC_SPU_MV_XF] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1110001_000_10100: begin  // fmv.x.d
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_SPU_SGN] = 1'b1;
                d32_func[FUNC_SPU_MV] = 1'b1; d32_func[FUNC_SPU_MV_XF] = 1'b1;
                d32_dst0_vld = 1'b1;
            end
            15'b1111000_000_10100: begin  // fmv.w.x
                d32_eu = EU_FP; d32_func[FUNC_SPU_SGN] = 1'b1;
                d32_func[FUNC_SPU_MV] = 1'b1;
            end
            15'b1111001_000_10100: begin  // fmv.d.x
                d32_eu = EU_FP; d32_func[FUNC_DOUBLE] = 1'b1; d32_func[FUNC_SPU_SGN] = 1'b1;
                d32_func[FUNC_SPU_MV] = 1'b1;
            end
            default: d32_illegal = 1'b1;   // FP/vector/AMO-LR-SC/custom-0/
                                            // dret/any unallocated encoding
                                            // (5.1's closed illegal-decode list;
                                            // sret/wfi/sfence.vma decoded above, M4)
        endcase
    end

    // ---- 16-bit RVC decode (decd.v:1246-1507's casez, keyed identically) ----
    reg [EU_WIDTH-1:0]   d16_eu;
    reg [FUNC_WIDTH-1:0] d16_func;
    reg                  d16_illegal;
    reg                  d16_src0_vld, d16_src1_vld, d16_src1_imm_vld;
    reg                  d16_src2_vld, d16_src2_imm_vld, d16_dst0_vld;
    reg [4:0]            d16_src0_reg, d16_src1_reg, d16_src2_reg, d16_dst0_reg;
    reg [63:0]           d16_src1_imm, d16_src2_imm;

    always @* begin
        d16_eu           = {EU_WIDTH{1'b0}};
        d16_func         = {FUNC_WIDTH{1'b0}};
        d16_illegal      = 1'b0;
        d16_src0_vld     = 1'b0;
        d16_src1_vld     = 1'b0;
        d16_src1_imm_vld = 1'b0;
        d16_src2_vld     = 1'b0;
        d16_src2_imm_vld = 1'b0;
        d16_dst0_vld     = 1'b0;
        d16_src0_reg     = 5'd0;
        d16_src1_reg     = 5'd0;
        d16_src2_reg     = 5'd0;
        d16_dst0_reg     = 5'd0;
        d16_src1_imm     = 64'd0;
        d16_src2_imm     = 64'd0;

        casez ({inst[15:10], inst[6:5], inst[1:0]})
            10'b000???_??00: begin  // c.addi4spn
                d16_eu = EU_ALU; d16_func = ALU_FUNC_ADD;
                d16_src0_vld = 1'b1; d16_src0_reg = 5'd2;   // sp
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_addi4spn_imm;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rs2_3;
                d16_illegal = (inst[12:5] == 8'd0);          // nzuimm==0 reserved
            end
            10'b001???_??00, 10'b101???_??00,               // c.fld / c.fsd
            10'b001???_??10, 10'b101???_??10: d16_illegal = 1'b1;  // c.fldsp / c.fsdsp (no FP in M2)
            10'b010???_??00: begin  // c.lw
                d16_eu = EU_LSU; d16_func = LSU_FUNC_LW;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_lw_imm;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rs2_3;
            end
            10'b011???_??00: begin  // c.ld
                d16_eu = EU_LSU; d16_func = LSU_FUNC_LD;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_ld_imm;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rs2_3;
            end
            10'b110???_??00: begin  // c.sw
                d16_eu = EU_LSU; d16_func = LSU_FUNC_SW;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_lw_imm;
                d16_src2_vld = 1'b1; d16_src2_reg = c_rs2_3;
            end
            10'b111???_??00: begin  // c.sd
                d16_eu = EU_LSU; d16_func = LSU_FUNC_SD;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_ld_imm;
                d16_src2_vld = 1'b1; d16_src2_reg = c_rs2_3;
            end
            10'b000???_??01: begin  // c.addi / c.nop
                d16_eu = EU_ALU; d16_func = ALU_FUNC_ADD;
                d16_src0_vld = (inst[11:7] != 5'd0); d16_src0_reg = c_rd_rs1_5;
                d16_src1_imm_vld = (inst[11:7] != 5'd0); d16_src1_imm = c_imm6;
                d16_dst0_vld = (inst[11:7] != 5'd0); d16_dst0_reg = c_rd_rs1_5;
            end
            10'b001???_??01: begin  // c.addiw
                d16_eu = EU_ALU; d16_func = ALU_FUNC_ADDW;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_5;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_imm6;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
                d16_illegal = (inst[11:7] == 5'd0);          // rd==x0 reserved
            end
            10'b010???_??01: begin  // c.li
                d16_eu = EU_ALU; d16_func = ALU_FUNC_ADD;
                d16_src0_reg = 5'd0;                          // x0 (no real src)
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_imm6;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
            end
            10'b011???_??01: begin  // c.addi16sp / c.lui
                if (inst[11:7] == 5'd2) begin  // c.addi16sp
                    d16_eu = EU_ALU; d16_func = ALU_FUNC_ADD;
                    d16_src0_vld = 1'b1; d16_src0_reg = 5'd2;  // sp
                    d16_src1_imm_vld = 1'b1; d16_src1_imm = c_addi16sp_imm;
                    d16_dst0_vld = 1'b1; d16_dst0_reg = 5'd2;
                end else begin                  // c.lui
                    d16_eu = EU_ALU; d16_func = ALU_FUNC_LUI;
                    d16_src1_imm_vld = 1'b1; d16_src1_imm = c_lui_imm;
                    d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
                end
                d16_illegal = ({inst[12], inst[6:2]} == 6'd0);  // zero-imm reserved (both forms)
            end
            10'b100?00_??01: begin  // c.srli
                d16_eu = EU_ALU; d16_func = ALU_FUNC_SRL;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_imm6;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100?01_??01: begin  // c.srai
                d16_eu = EU_ALU; d16_func = ALU_FUNC_SRA;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_imm6;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100?10_??01: begin  // c.andi
                d16_eu = EU_ALU; d16_func = ALU_FUNC_AND;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_imm6;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100011_0001: begin  // c.sub
                d16_eu = EU_ALU; d16_func = ALU_FUNC_SUB;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_3;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100011_0101: begin  // c.xor
                d16_eu = EU_ALU; d16_func = ALU_FUNC_XOR;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_3;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100011_1001: begin  // c.or
                d16_eu = EU_ALU; d16_func = ALU_FUNC_OR;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_3;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100011_1101: begin  // c.and
                d16_eu = EU_ALU; d16_func = ALU_FUNC_AND;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_3;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100111_0001: begin  // c.subw
                d16_eu = EU_ALU; d16_func = ALU_FUNC_SUBW;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_3;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b100111_0101: begin  // c.addw
                d16_eu = EU_ALU; d16_func = ALU_FUNC_ADDW;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_3;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_3;
            end
            10'b101???_??01: begin  // c.j
                d16_eu = EU_BJU; d16_func = BJU_FUNC_JAL;
                d16_src2_imm_vld = 1'b1; d16_src2_imm = c_j_imm;
                d16_dst0_reg = 5'd0;    // no link write (x0, harmless)
            end
            10'b110???_??01: begin  // c.beqz
                d16_eu = EU_BJU; d16_func = BJU_FUNC_BEQ;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = 5'd0;      // compare vs x0
                d16_src2_imm_vld = 1'b1; d16_src2_imm = c_br_imm;
            end
            10'b111???_??01: begin  // c.bnez
                d16_eu = EU_BJU; d16_func = BJU_FUNC_BNE;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_3;
                d16_src1_vld = 1'b1; d16_src1_reg = 5'd0;      // compare vs x0
                d16_src2_imm_vld = 1'b1; d16_src2_imm = c_br_imm;
            end
            10'b000???_??10: begin  // c.slli
                d16_eu = EU_ALU; d16_func = ALU_FUNC_SLL;
                d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_5;
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_imm6;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
            end
            10'b010???_??10: begin  // c.lwsp
                d16_eu = EU_LSU; d16_func = LSU_FUNC_LW;
                d16_src0_vld = 1'b1; d16_src0_reg = 5'd2;      // sp
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_lwsp_imm;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
                d16_illegal = (inst[11:7] == 5'd0);            // rd==x0 reserved
            end
            10'b011???_??10: begin  // c.ldsp
                d16_eu = EU_LSU; d16_func = LSU_FUNC_LD;
                d16_src0_vld = 1'b1; d16_src0_reg = 5'd2;      // sp
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_ldsp_imm;
                d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
                d16_illegal = (inst[11:7] == 5'd0);            // rd==x0 reserved
            end
            10'b1000??_??10: begin  // c.jr / c.mv
                if (inst[6:2] == 5'd0) begin    // c.jr
                    d16_eu = EU_BJU; d16_func = BJU_FUNC_JALR;
                    d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_5;
                    d16_src2_imm_vld = 1'b1; d16_src2_imm = 64'd0;
                    d16_dst0_reg = 5'd0;         // no link write
                    d16_illegal = (inst[11:7] == 5'd0);  // rs1==x0 reserved
                end else begin                    // c.mv
                    d16_eu = EU_ALU; d16_func = ALU_FUNC_ADD;
                    d16_src0_vld = 1'b1; d16_src0_reg = c_rs2_5;      // value moved
                    d16_src1_imm_vld = 1'b1; d16_src1_imm = 64'd0;
                    d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
                end
            end
            10'b1001??_??10: begin  // c.jalr / c.add / c.ebreak
                if (inst[6:2] == 5'd0) begin
                    if (inst[11:7] == 5'd0) begin  // c.ebreak
                        d16_eu = EU_CP0; d16_func = CP0_FUNC_EBREAK;
                    end else begin                  // c.jalr
                        d16_eu = EU_BJU; d16_func = BJU_FUNC_JALR;
                        d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_5;
                        d16_src2_imm_vld = 1'b1; d16_src2_imm = 64'd0;
                        d16_dst0_vld = 1'b1; d16_dst0_reg = 5'd1;  // implicit x1 (link)
                    end
                end else begin                        // c.add
                    d16_eu = EU_ALU; d16_func = ALU_FUNC_ADD;
                    d16_src0_vld = 1'b1; d16_src0_reg = c_rd_rs1_5;
                    d16_src1_vld = 1'b1; d16_src1_reg = c_rs2_5;
                    d16_dst0_vld = 1'b1; d16_dst0_reg = c_rd_rs1_5;
                end
            end
            10'b110???_??10: begin  // c.swsp
                d16_eu = EU_LSU; d16_func = LSU_FUNC_SW;
                d16_src0_vld = 1'b1; d16_src0_reg = 5'd2;      // sp
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_swsp_imm;
                d16_src2_vld = 1'b1; d16_src2_reg = c_rs2_5;
            end
            10'b111???_??10: begin  // c.sdsp
                d16_eu = EU_LSU; d16_func = LSU_FUNC_SD;
                d16_src0_vld = 1'b1; d16_src0_reg = 5'd2;      // sp
                d16_src1_imm_vld = 1'b1; d16_src1_imm = c_sdsp_imm;
                d16_src2_vld = 1'b1; d16_src2_reg = c_rs2_5;
            end
            default: d16_illegal = 1'b1;   // reserved 16-bit encodings
        endcase
    end

    // ---- final decode mux (decd.v's own decd_sel[0]/[1] peer-path mux,
    // simplified to 2 paths since every other class is illegal in M2) +
    // the illegal->EU_CP0 dispatch override (ctrl.v:573-574's mechanism,
    // simplified -- see header) ----
    wire [EU_WIDTH-1:0]   dis_eu_raw    = is32 ? d32_eu           : d16_eu;
    wire [FUNC_WIDTH-1:0] dis_func      = is32 ? d32_func         : d16_func;
    wire                  dis_illegal   = is32 ? d32_illegal      : d16_illegal;
    wire                  dis_src0_vld  = is32 ? d32_src0_vld     : d16_src0_vld;
    wire                  dis_src1_vld  = is32 ? d32_src1_vld     : d16_src1_vld;
    wire                  dis_src1_imm_vld = is32 ? d32_src1_imm_vld : d16_src1_imm_vld;
    wire                  dis_src2_vld  = is32 ? d32_src2_vld     : d16_src2_vld;
    wire                  dis_src2_imm_vld = is32 ? d32_src2_imm_vld : d16_src2_imm_vld;
    wire                  dis_dst0_vld  = is32 ? d32_dst0_vld     : d16_dst0_vld;
    wire [4:0]            dis_src0_reg5 = is32 ? d32_src0_reg     : d16_src0_reg;
    wire [4:0]            dis_src1_reg5 = is32 ? d32_src1_reg     : d16_src1_reg;
    wire [4:0]            dis_src2_reg5 = is32 ? d32_src2_reg     : d16_src2_reg;
    wire [4:0]            dis_dst0_reg5 = is32 ? d32_dst0_reg     : d16_dst0_reg;
    wire [63:0]           dis_src1_imm  = is32 ? d32_src1_imm     : d16_src1_imm;
    wire [63:0]           dis_src2_imm  = is32 ? d32_src2_imm     : d16_src2_imm;
    // M5 Task 4c: FLW/FLD/FSW/FSD FRF tags -- no 16-bit (RVC) FP load/store
    // decode exists yet (c.fld/c.fsd stay illegal, "no FP in M2" above), so
    // the d16 side is always 0.
    wire                  dis_dst0_frf  = is32 && d32_dst0_frf;
    wire                  dis_src2_frf  = is32 && d32_src2_frf;
    // M5 Task 4b: OP-FP's funct3 field IS the rm field for FADD/FSUB/FCVT.f2f
    // (dynamic rounding mode) and a fixed sub-op selector (already captured
    // in FUNC bits above) for FMIN/FMAX/FSGNJ* -- passing it through
    // unconditionally is harmless for the latter group since FPU.v's
    // fspu/minmax result paths never consult idu_fpu_ex1_rm.
    wire [2:0]            dis_rm        = inst[14:12];

    // M4 Task 6: the fetch-fault tags, live at ID exactly like `inst` itself
    // (IFU.v's own SECTION IBUF anchors both at ibuf_head/h0). Force EU_CP0
    // dispatch the SAME way an illegal decode does -- the synthetic NOP's
    // OWN decode (dis_illegal would read false for it; ADDI x0,x0,0 is a
    // genuinely legal encoding) never gets a say once either fault flag is
    // set.
    wire dis_fetch_pgflt  = ifu_idu_id_fault_pgflt;
    wire dis_fetch_accflt = ifu_idu_id_fault_accflt;

    wire [EU_WIDTH-1:0] dis_eu_final = (dis_illegal || dis_fetch_pgflt || dis_fetch_accflt)
                                      ? EU_CP0 : dis_eu_raw;

    // producer-type tag for THIS instruction, used both for WBT-create and
    // the WAW-except "new producer type" comparison (rvproc_pkg.sv's
    // resolved dp_wb_dst0_type OR-mux -- ALU/BJU/MULT/LSU only, DIV/CP0/
    // illegal fall to OTHER, cited not re-derived per the task text).
    //
    // M3 Task 7 FIX: AMO/LR/SC are tagged WB_INT_TYPE_OTHER, not LSU. NOTE
    // the DONOR tags them WB_INT_TYPE_LSU (its producer-type is purely the
    // EU select, aq_idu_id_dp.v:566-570) and gets away with it because its
    // pipelined LSU writes AMO/LR/SC results back at LOAD latency (SC result
    // generated at DC, aq_lsu_dc.v:1744,1941; AMO/LR rd at DA, 2417-2419), so
    // the load->condbr RAW exemption (raw0_except term 3) is safe there.
    // THIS clone's LSU is different: it is late-writeback -- an AMO/LR/SC
    // holds the LSU through a read-modify-write (AMO) or a store-conditional
    // sequence, so the register writeback lands many cycles after dispatch.
    // Tagging them OTHER removes the condbr exemption and forces consumers
    // to stall until the real writeback. Deliberate, documented adaptation
    // (docs/08-verification.md §8.13), not a donor transcription.
    wire dis_is_amo_lrsc = (dis_eu_raw == EU_LSU) &&
                           (dis_func[19:12] == 8'h01 || dis_func == LSU_FUNC_LR_W
                            || dis_func == LSU_FUNC_LR_D || dis_func == LSU_FUNC_SC_W
                            || dis_func == LSU_FUNC_SC_D);
    wire [2:0] dis_dst0_type = (dis_eu_final == EU_ALU)  ? WB_INT_TYPE_ALU  :
                               (dis_eu_final == EU_BJU)  ? WB_INT_TYPE_BJU  :
                               (dis_eu_final == EU_MULT) ? WB_INT_TYPE_MULT :
                               (dis_eu_final == EU_LSU)  ? (dis_is_amo_lrsc ? WB_INT_TYPE_OTHER
                                                                            : WB_INT_TYPE_LSU) :
                                                            WB_INT_TYPE_OTHER;

    // producer-type-aware except flags this instruction's OWN class needs
    // (bit-flag reuse, IDU note S3.3: FUNC_STORE_SEL=bit0/FUNC_CONDBR_SEL=
    // bit6, confirmed bit-exact for every pinned LSU_FUNC_*/BJU_FUNC_*
    // constant against aq_idu_cfig.h -- not re-derived per-mnemonic).
    wire dis_is_store  = (dis_eu_raw == EU_LSU) && dis_func[0];
    wire dis_is_condbr = (dis_eu_raw == EU_BJU) && dis_func[6];

    //=========================================================================
    // SECTION WBT (IDU note S5.1, aq_idu_id_wbt.v/_entry.v) -- 32-entry
    // busy-bit + producer-type + outstanding-count scoreboard. Entry 0 (x0)
    // hardwired always-ready (wbt.v:207-208); entries 1-31 real flops
    // (wbt_entry.v, entry array per umbrella S6.2 rule 6, generate-for
    // here rather than 31 module instances).
    //=========================================================================
    function [31:0] onehot32(input [4:0] regnum, input vld);
        onehot32 = vld ? ({31'b0, 1'b1} << regnum) : 32'd0;
    endfunction

    reg       wbt_wb_r   [1:31];
    reg [1:0] wbt_cnt_r  [1:31];
    reg [2:0] wbt_type_r [1:31];

    wire [31:0] wbt_wb_en = onehot32(rtu_idu_wb0_reg[4:0], rtu_idu_wb0_vld)
                          | onehot32(rtu_idu_wb1_reg[4:0], rtu_idu_wb1_vld);

    // create only on a non-stalled dispatch (wbt.v:777-798's own gating --
    // no split concept in M2, contract 10, so `ctrl_wbt_dis_inst_vld` is
    // simply `ifu_idu_id_inst_vld`).
    //
    // Gated on `!iu_idu_br_cancel` exactly like the donor's per-entry create
    // (aq_idu_id_wbt_entry.v:75: `create_en = (create0_en_x || create1_en_x)
    // && !iu_yy_xx_cancel`): on a branch-mispredict redirect the ID-stage
    // (wrong-path) instruction is cancelled from EX1 and must NOT arm a WBT
    // busy-bit -- otherwise its busy-bit is never written back and the
    // register reads as permanently pending (rv64ui-p-beq: a fallthrough
    // `addi ra,ra,1` armed x1's busy-bit the cycle the taken `beqz` redirected,
    // then parked the later `bne ra,t2`).
    wire [31:0] wbt_create0 = onehot32(dis_dst0_reg5,
                                        dis_dst0_vld && ifu_idu_id_inst_vld && !ctrl_dis_stall
                                        && !iu_idu_br_cancel);
    genvar gi;
    generate
        for (gi = 1; gi <= 31; gi = gi + 1) begin : g_wbt
            wire create_en = wbt_create0[gi];
            wire wb_en_x   = wbt_wb_en[gi];
            wire cnt_is_1  = (wbt_cnt_r[gi] == 2'd1);
            wire cnt_is_2  = (wbt_cnt_r[gi] == 2'd2);

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)                       wbt_wb_r[gi] <= 1'b1;
                else if (rtu_idu_flush_wbt)        wbt_wb_r[gi] <= 1'b1;
                else if (create_en)                wbt_wb_r[gi] <= 1'b0;
                else if (wb_en_x && cnt_is_1)       wbt_wb_r[gi] <= 1'b1;
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)                        wbt_cnt_r[gi] <= 2'd0;
                else if (rtu_idu_flush_wbt)         wbt_cnt_r[gi] <= 2'd0;
                else if (create_en && wb_en_x)      wbt_cnt_r[gi] <= wbt_cnt_r[gi];
                else if (create_en)                 wbt_cnt_r[gi] <= wbt_cnt_r[gi] + 2'd1;
                else if (wb_en_x)                   wbt_cnt_r[gi] <= wbt_cnt_r[gi] - 2'd1;
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)                        wbt_type_r[gi] <= 3'd0;
                else if (rtu_idu_flush_wbt)         wbt_type_r[gi] <= 3'd0;
                else if (create_en)                 wbt_type_r[gi] <= dis_dst0_type;
            end
        end
    endgenerate

    // 7-bit read port {WB_CNT2, CNT[1:0], TYPE[2:0], VLD} (wbt_entry.v:97-98,
    // 116,153; WB_INT_* bit positions confirmed cfig.h:161-166).
    function [6:0] wbt_info(input [4:0] regnum);
        reg wb_en_x, cnt_is_1, cnt_is_2;
        begin
            if (regnum == 5'd0)
                wbt_info = 7'b0000001;   // x0: VLD=1, everything else 0
            else begin
                wb_en_x  = wbt_wb_en[regnum];
                cnt_is_1 = (wbt_cnt_r[regnum] == 2'd1);
                cnt_is_2 = (wbt_cnt_r[regnum] == 2'd2);
                wbt_info = {(wb_en_x && cnt_is_2), wbt_cnt_r[regnum], wbt_type_r[regnum],
                            (wbt_wb_r[regnum] || (wb_en_x && cnt_is_1))};
            end
        end
    endfunction

    wire [6:0] wbt_src0_info = wbt_info(dis_src0_reg5);
    wire [6:0] wbt_src1_info = wbt_info(dis_src1_reg5);
    wire [6:0] wbt_src2_info = wbt_info(dis_src2_reg5);
    wire [6:0] wbt_dst0_info = wbt_info(dis_dst0_reg5);

    wire        src0_vld_wbt = wbt_src0_info[0];
    wire [2:0]  src0_type    = wbt_src0_info[3:1];
    wire [1:0]  src0_cnt     = wbt_src0_info[5:4];
    wire        src1_vld_wbt = wbt_src1_info[0];
    wire [2:0]  src1_type    = wbt_src1_info[3:1];
    wire [1:0]  src1_cnt     = wbt_src1_info[5:4];
    wire        src2_vld_wbt = wbt_src2_info[0];
    wire [2:0]  src2_type    = wbt_src2_info[3:1];
    wire [1:0]  src2_cnt     = wbt_src2_info[5:4];
    wire        src2_cnt2    = wbt_src2_info[6];
    wire        dst0_vld_wbt = wbt_dst0_info[0];
    wire [2:0]  dst0_type_wbt= wbt_dst0_info[3:1];
    wire [1:0]  dst0_cnt     = wbt_dst0_info[5:4];
    wire        dst0_cnt2    = wbt_dst0_info[6];

    //=========================================================================
    // SECTION GPR (IDU note S5.4, aq_idu_id_gpr.v/_gated_reg.v) -- 31-entry
    // gated register file + hardwired x0, 3 read ports, 2 write ports.
    //
    // Task 7.2: the array is declared [0:31] (not [1:31]) so that the
    // architectural register number maps 1:1 onto the array index for
    // the verisim harness (rtl/verisim.h CPU_GPR, restore-checklist item
    // 1). x0's READ path is unchanged -- gpr_read() below still hardwires
    // regnum==0 to 64'd0 -- and a writeback targeting x0 (rd=x0 is a
    // RISC-V architectural no-op) lands in dead storage at gpr_r[0] that
    // nothing ever reads. Behavior bit-identical; only the array shape
    // and the public mark changed.
    //=========================================================================
    reg [63:0] gpr_r [0:31] /* verilator public */;

    wire [31:0] gpr_wb0_oh = onehot32(rtu_idu_wb0_reg[4:0], rtu_idu_wb0_vld);
    wire [31:0] gpr_wb1_oh = onehot32(rtu_idu_wb1_reg[4:0], rtu_idu_wb1_vld);

    genvar gj;
    generate
        for (gj = 0; gj < 32; gj = gj + 1) begin : g_gpr
            // same-cycle wb0==wb1 collision on THIS register: no 2'b11 arm
            // exists (gated_reg.v:86-99 verbatim) -- the write is silently
            // DROPPED (register holds its old value), not merged/prioritized.
            always @(posedge clk) begin
                case ({gpr_wb1_oh[gj], gpr_wb0_oh[gj]})
                    2'b01:   gpr_r[gj] <= rtu_idu_wb0_data;
                    2'b10:   gpr_r[gj] <= rtu_idu_wb1_data;
                    default: gpr_r[gj] <= gpr_r[gj];
                endcase
            end
        end
    endgenerate

    // Donor-faithful read-during-write (aq_idu_id_gpr_gated_reg.v:86-109, the
    // `read_data_y = write_data` line): the read port returns the SAME-CYCLE
    // writeback (wb0 or wb1) when it targets this register, else the stored
    // value. This is load-bearing, not an optimization: fwd0 covers only the
    // 1-back dependency (producer at EX1, its result not yet on wb0), while the
    // 2-back dependency (producer at EX2, result now on wb0/wb1) is seen ONLY
    // through this same-cycle merge -- without it a `addi a1; addi a2; add
    // a4,a1,a2` reads a1 stale and computes the wrong sum (rv64ui-p-add test_3/
    // test_4). The 2'b11 arm is deliberately absent: both wb0 and wb1 naming the
    // same register -> hold the old value, exactly like gated_reg.v:93-97. x0
    // stays hardwired to 0 (a writeback naming x0 is an architectural no-op).
    function [63:0] gpr_read(input [4:0] regnum);
        reg [1:0] wsel;
        begin
            wsel = {gpr_wb1_oh[regnum], gpr_wb0_oh[regnum]};
            if (regnum == 5'd0)
                gpr_read = 64'd0;
            else
                case (wsel)
                    2'b01:   gpr_read = rtu_idu_wb0_data;
                    2'b10:   gpr_read = rtu_idu_wb1_data;
                    default: gpr_read = gpr_r[regnum];
                endcase
        end
    endfunction

    wire [63:0] gpr_src0_data = gpr_read(dis_src0_reg5);
    wire [63:0] gpr_src1_data = gpr_read(dis_src1_reg5);
    wire [63:0] gpr_src2_data = gpr_read(dis_src2_reg5);

    //=========================================================================
    // SECTION FRF (M5 Task 2, D2) -- 32-entry FP register file, added
    // inline next to GPR rather than as a separate module: the donor's FP
    // register file (`aq_vidu_vid_gpr_fp.v`) lives in the `vidu` cluster
    // only because C906 bundles its vector engine and FPU together; rv906
    // has no vector extension, so there is no second top-level cluster to
    // put it in (D2/D3 in the M5 design doc).
    //
    // Structurally sized like GPR (2 write ports, 3 read ports) rather
    // than the donor's single vector-issue writeback port
    // (aq_vidu_vid_gpr_fp.v:28-30 `vpu_vidu_fp_wb_reg/_data/_vld`) -- see
    // the rtu_idu_wbf0/wbf1 port comment above for the reasoning (mirrors
    // RTU.v's own compute-vs-LSU wb0/wb1 split, not an arbitrary donor
    // deviation).
    //
    // No hardwired-zero entry: per spec f0 is a real architectural
    // register, unlike x0 -- frf_read() below has no x0-style special
    // case. Sized [0:31] for the same verisim-harness index-mapping
    // reason as GPR's [0:31] (Task 7.2's comment above).
    //
    // NOT YET CONNECTED to any consumer (M5 Task 2 scope): the write
    // ports are real new IDU.v inputs, undriven until Task 3 (FPU.v) and
    // Task 4 (LSU FRF-destination plumbing) exist. The read ports feed
    // new output ports (idu_fpu_ex1_fsrc0/1/2_data) so this task's idu_tb
    // rows can exercise reads/writes directly -- no OP-FP decode arm
    // exists yet (Task 4 owns all of those), so unlike GPR's
    // dis_src0_reg5 (decode-class-dependent, picking among several
    // possible instruction-field positions per format), the FP source
    // registers sit at FIXED bit positions for every FP instruction
    // format in the base ISA encoding (rs1=inst[19:15], rs2=inst[24:20],
    // rs3=inst[31:27] for the R4-type FMA family), so they're sliced
    // unconditionally here -- the same "computed regardless of decode
    // legality" convention this file already uses for imm_i/imm_s/etc.
    // in SECTION DECODE.
    //=========================================================================
    reg [63:0] frf_r [0:31] /* verilator public */;

    wire [31:0] frf_wbf0_oh = onehot32(rtu_idu_wbf0_reg[4:0], rtu_idu_wbf0_vld);
    wire [31:0] frf_wbf1_oh = onehot32(rtu_idu_wbf1_reg[4:0], rtu_idu_wbf1_vld);

    genvar fj;
    generate
        for (fj = 0; fj < 32; fj = fj + 1) begin : g_frf
            // Same same-cycle wbf0==wbf1 collision policy as GPR (silently
            // dropped, no 2'b11 arm) -- a consistent structural choice
            // paralleling gpr_r's gated_reg.v-derived policy, not a
            // distinct donor citation of its own (the donor's FRF has
            // only 1 write port and therefore no such collision case).
            always @(posedge clk) begin
                case ({frf_wbf1_oh[fj], frf_wbf0_oh[fj]})
                    2'b01:   frf_r[fj] <= rtu_idu_wbf0_data;
                    2'b10:   frf_r[fj] <= rtu_idu_wbf1_data;
                    default: frf_r[fj] <= frf_r[fj];
                endcase
            end
        end
    endgenerate

    // Read-during-write merge, mirroring gpr_read() (no x0 special case).
    function [63:0] frf_read(input [4:0] regnum);
        reg [1:0] wsel;
        begin
            wsel = {frf_wbf1_oh[regnum], frf_wbf0_oh[regnum]};
            case (wsel)
                2'b01:   frf_read = rtu_idu_wbf0_data;
                2'b10:   frf_read = rtu_idu_wbf1_data;
                default: frf_read = frf_r[regnum];
            endcase
        end
    endfunction

    // Fixed FP source-register field positions (base ISA encoding, same
    // across every OP-FP/LOAD-FP/STORE-FP/FMADD/FMSUB/FNMSUB/FNMADD
    // format) -- computed unconditionally, same convention as
    // imm_i/imm_s/etc above.
    wire [4:0] dis_fsrc0_reg5 = inst[19:15];
    wire [4:0] dis_fsrc1_reg5 = inst[24:20];
    wire [4:0] dis_fsrc2_reg5 = inst[31:27];

    wire [63:0] frf_src0_data = frf_read(dis_fsrc0_reg5);
    wire [63:0] frf_src1_data = frf_read(dis_fsrc1_reg5);
    wire [63:0] frf_src2_data = frf_read(dis_fsrc2_reg5);

    //=========================================================================
    // SECTION FWBT (M5 Task 11 root-cause fix) -- FP-register write-back
    // tracker, the FRF-side counterpart of SECTION WBT above.
    //
    // Donor-verified need, not an invented mechanism: the donor's vidu
    // cluster (which bundles FP+vector) carries its OWN 32-entry busy-bit
    // scoreboard for the FP/vector register file --
    // refs/openc906/.../gen_rtl/vidu/rtl/aq_vidu_vid_wbt.v +
    // aq_vidu_vid_wbt_entry.v -- structurally the same idea as
    // aq_idu_id_wbt.v's GPR scoreboard this file already clones above.
    // SECTION FRF's own M5 Task 2 comment ("No FRF scoreboard exists in
    // this design, so these need no _vld-style readiness companion of
    // their own") was a genuine clone omission, not a donor-faithful
    // simplification: LOAD-FP/FLD share the exact same variable-latency
    // LSU (AG/DC + the LFB deferred-completion path on a cache-line-fill
    // race, LSU.v SECTION AMO ALU comment area) that the GPR WBT +
    // fwd0/1/2 network exists to protect GPR consumers against, and
    // every FRF-destined OP-FP/FMA-family/FCVT/FMV.*.x result is equally
    // unprotected today (no fwdf0/1 bus exists on the FP side at all).
    // Root-caused via an instrumented run (test/m5/build/rv64uf-p-fadd.elf,
    // RV906_FRF_DBG probe, temporary/removed): `flw fa0,0(a0); flw fa1,
    // 4(a0); flw fa2,8(a0); lw a3,12(a0); fadd.s fa3,fa0,fa1` -- fa0/fa2's
    // FLW completions raced past fa1's (fa1 dispatched BEFORE fa2 but its
    // wbf strobe landed AFTER fa2's, cycle 659 vs 651, both after FADD's
    // own dispatch at cycle 652) with nothing stalling FADD's dispatch on
    // fa1 still being in flight -- FADD read stale (zero) frf_r[11] instead
    // of 1.0, exactly the "systemic dispatch/FRF plumbing" shape flagged
    // up front rather than N independent per-category FALU/FMAU/FDSU bugs.
    //
    // Unlike the GPR side, no forwarding bus exists here, so there is no
    // "except" fast path: every FRF producer's consumers simply wait for
    // the busy-bit to clear. frf_read()'s same-cycle wsel merge (SECTION
    // FRF above) still supplies the fresh value the very cycle the bit
    // clears (mirrors gpr_read()'s identical same-cycle merge, already
    // proven correct on the GPR side), so no data is lost -- at most one
    // dispatch slot of throughput is given up on a tight FP producer/
    // consumer pair. All 32 entries are real (f0 is architectural, unlike
    // x0), generated the same busy-bit+count shape as g_wbt above (no
    // producer-type field needed -- there is no forwarding to exempt
    // against).
    //
    // dis_dst0_frf_fpu derives the FRF-destination flag for every OP-FP/
    // FMA-family/FCVT/FMV.*.x arm from the SAME decode invariant those
    // arms' own comments already document: every EU_FP arm either sets
    // dis_dst0_vld=1 (GPR-destined: compare/classify, FCVT-to-int, FMV.x.*)
    // or leaves it 0 (FRF-destined: fadd/fsub/fmin/fmax/fsgnj*/FCVT f2f/
    // fmul/fdiv/fsqrt/the FMA family/FCVT-to-float/FMV.*.x) -- verified
    // against every arm in the casez above, not re-derived per-mnemonic.
    // dis_dst0_frf (LOAD-FP/FLD only) stays untouched -- it still feeds
    // idu_lsu_ex1_dst0_frf's LSU routing tag unchanged; dis_dst0_frf_all
    // is a NEW signal, consumed only by this scoreboard.
    //=========================================================================
    wire dis_dst0_frf_fpu = (dis_eu_final == EU_FP) && !dis_dst0_vld;
    wire dis_dst0_frf_all = dis_dst0_frf || dis_dst0_frf_fpu;

    reg       fwbt_wb_r  [0:31];
    reg [1:0] fwbt_cnt_r [0:31];

    wire [31:0] fwbt_wb_en = frf_wbf0_oh | frf_wbf1_oh;

    // Create on any non-stalled, non-cancelled dispatch whose destination
    // is FRF -- same create-gating shape as wbt_create0 above.
    wire [31:0] fwbt_create0 = onehot32(dis_dst0_reg5,
                                         dis_dst0_frf_all && ifu_idu_id_inst_vld
                                         && !ctrl_dis_stall && !iu_idu_br_cancel);

    genvar fk;
    generate
        for (fk = 0; fk <= 31; fk = fk + 1) begin : g_fwbt
            wire create_en = fwbt_create0[fk];
            wire wb_en_x   = fwbt_wb_en[fk];
            wire cnt_is_1  = (fwbt_cnt_r[fk] == 2'd1);

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)                    fwbt_wb_r[fk] <= 1'b1;
                else if (rtu_idu_flush_wbt)     fwbt_wb_r[fk] <= 1'b1;
                else if (create_en)             fwbt_wb_r[fk] <= 1'b0;
                else if (wb_en_x && cnt_is_1)   fwbt_wb_r[fk] <= 1'b1;
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)                      fwbt_cnt_r[fk] <= 2'd0;
                else if (rtu_idu_flush_wbt)       fwbt_cnt_r[fk] <= 2'd0;
                else if (create_en && wb_en_x)    fwbt_cnt_r[fk] <= fwbt_cnt_r[fk];
                else if (create_en)               fwbt_cnt_r[fk] <= fwbt_cnt_r[fk] + 2'd1;
                else if (wb_en_x)                 fwbt_cnt_r[fk] <= fwbt_cnt_r[fk] - 2'd1;
            end
        end
    endgenerate

    function fwbt_busy(input [4:0] regnum);
        reg wb_en_x, cnt_is_1;
        begin
            wb_en_x   = fwbt_wb_en[regnum];
            cnt_is_1  = (fwbt_cnt_r[regnum] == 2'd1);
            fwbt_busy = !(fwbt_wb_r[regnum] || (wb_en_x && cnt_is_1));
        end
    endfunction

    // Per-field FRF hazard gates -- NOT a blanket "any FRF-touching
    // dispatch checks all three fsrc fields" (a first cut at this that
    // did exactly that caused a real regression, root-caused via the
    // debug probe: LOAD-FP/STORE-FP's dis_fsrc0_reg5/dis_fsrc2_reg5 are
    // NOT register reads at all for those ops -- inst[19:15]/inst[31:27]
    // are the GPR address-base register and immediate/funct bits
    // respectively, computed "unconditionally" (SECTION FRF's own
    // comment) purely because every FP format shares those bit
    // positions. Gating rawf0/rawf2 on dis_dst0_frf_all (true for every
    // LOAD-FP) made `flw fa1,4(a0)` check whether FP register f10 was
    // busy -- and it very much was, since f10 IS fa0, the register the
    // PRIOR `flw fa0,0(a0)` had just started loading through that exact
    // same x10=a0 base -- a guaranteed false hit on every multi-flw
    // sequence, not a rare coincidence. That spurious stall delayed
    // fa1's own AG/DC entry into the LSU pipeline, which is what let it
    // land in the load-fill-buffer behind fa2 instead of ahead of it
    // and come back with fa2's data. Root cause was the over-broad gate,
    // not the LFB itself -- fixed at the gate, not by touching LSU.v.
    //
    // fsrc0 is a genuine FP source only for EU_FP ops NOT covered by
    // dis_gpr_fsrc0's int-sourced override (fcvt.{s,d}.{w,wu,l,lu} and
    // fmv.{w,d}.x read rs1 from the GPR, exactly like SECTION EX1's own
    // ex1_fsrc0_data_r mux already accounts for -- reusing that same
    // signal here instead of re-deriving it).
    wire rawf0_chk = (dis_eu_final == EU_FP) && !dis_gpr_fsrc0;
    // fsrc1 is a genuine FP source for every EU_FP sub-class (real rs2
    // operand for compare/add/sub/min/max/sgnj/mul/div/FMA; a fixed
    // rs2=00000 selector -- i.e. f0 -- for fclass/fsqrt/the f2f/int
    // conversions/fmv.*, so checking it there is at most a rare, no-op-
    // valued, still-harmless over-check, unlike fsrc0/fsrc2's proven-common
    // false hits above) plus STORE-FP's real FRF data source (dis_src2_frf).
    wire rawf1_chk = (dis_eu_final == EU_FP) || dis_src2_frf;
    // fsrc2 (rs3) is a genuine FP source ONLY for the R4-type FMA family
    // (fmadd/fmsub/fnmsub/fnmadd -- FUNC_MAU_MUL&&FUNC_MAU_FUSED both set,
    // unambiguous despite FUNC_MAU_FUSED's bit position being reused as
    // FUNC_FDSU_DIV for fdiv, which never sets FUNC_MAU_MUL). Every other
    // EU_FP arm's inst[31:27] is a fixed funct7-high-bits pattern, not a
    // register index -- same false-hit shape as fsrc0's bug above, so
    // this is gated precisely rather than left broad.
    wire dis_is_fma = dis_func[FUNC_MAU_MUL] && dis_func[FUNC_MAU_FUSED];
    wire rawf2_chk = (dis_eu_final == EU_FP) && dis_is_fma;

    wire rawf0 = rawf0_chk && fwbt_busy(dis_fsrc0_reg5);
    wire rawf1 = rawf1_chk && fwbt_busy(dis_fsrc1_reg5);
    wire rawf2 = rawf2_chk && fwbt_busy(dis_fsrc2_reg5);
    wire wawf  = dis_dst0_frf_all && fwbt_busy(dis_dst0_reg5);

    //=========================================================================
    // SECTION FORWARD MUX (IDU note S6, aq_idu_id_dp.v:639-727) -- 3-way
    // one-hot compare against rtu_idu_fwd0/1/2, falls to `{64{1'bx}}` on a
    // non-hit exactly like the donor (the mutual-exclusivity invariant that
    // prevents a real multi-hit is RTU's Task 4.2 assertion, not logic
    // living here). x0 is never a legitimate forward target (dp.v:663-665).
    //=========================================================================
    function fwd_hit(input [4:0] regnum);
        fwd_hit = (regnum != 5'd0)
               && ((rtu_idu_fwd0_vld && (rtu_idu_fwd0_reg[4:0] == regnum))
                || (rtu_idu_fwd1_vld && (rtu_idu_fwd1_reg[4:0] == regnum))
                || (rtu_idu_fwd2_vld && (rtu_idu_fwd2_reg[4:0] == regnum)));
    endfunction

    function [63:0] fwd_data(input [4:0] regnum);
        reg [2:0] sel;
        begin
            sel = {rtu_idu_fwd2_vld && (rtu_idu_fwd2_reg[4:0] == regnum),
                   rtu_idu_fwd1_vld && (rtu_idu_fwd1_reg[4:0] == regnum),
                   rtu_idu_fwd0_vld && (rtu_idu_fwd0_reg[4:0] == regnum)};
            case (sel)
                3'b001: fwd_data = rtu_idu_fwd0_data;
                3'b010: fwd_data = rtu_idu_fwd1_data;
                3'b100: fwd_data = rtu_idu_fwd2_data;
                default: fwd_data = {64{1'bx}};
            endcase
        end
    endfunction

    wire fwd_src0_vld = fwd_hit(dis_src0_reg5);
    wire fwd_src1_vld = fwd_hit(dis_src1_reg5);
    wire fwd_src2_vld = fwd_hit(dis_src2_reg5);

    // operand-mux composition (dp.v:737-738,754-760,783-789 verbatim shape:
    // src0 never takes an immediate; src1/src2 check !srcN_vld first).
    wire [63:0] dis_src0_data = fwd_src0_vld ? fwd_data(dis_src0_reg5) : gpr_src0_data;
    wire [63:0] dis_src1_data = !dis_src1_vld ? dis_src1_imm
                              : fwd_src1_vld  ? fwd_data(dis_src1_reg5) : gpr_src1_data;
    wire [63:0] dis_src2_data = dis_src2_frf ? frf_src1_data
                              : !dis_src2_vld ? dis_src2_imm
                              : fwd_src2_vld  ? fwd_data(dis_src2_reg5) : gpr_src2_data;

    // SRCn_RDY (dp.v:740-742,766-768,795-797 verbatim shape).
    wire dis_src0_rdy = src0_vld_wbt || fwd_src0_vld || !dis_src0_vld;
    wire dis_src1_rdy = src1_vld_wbt || fwd_src1_vld || !dis_src1_vld;
    wire dis_src2_rdy = src2_vld_wbt || fwd_src2_vld || !dis_src2_vld;

    //=========================================================================
    // SECTION RAW/WAW HAZARD (IDU note S5.2, aq_idu_id_ctrl.v:398-536) --
    // producer-type-aware except clauses. By this task, Task 3's
    // WB_INT_TYPE-for-DIV finding (DIV tags as OTHER, no fast-path
    // exemption -- rvproc_pkg.sv's own Task 3.6b resolution) and Task 4.2's
    // fwd0/1/2 mutual-exclusivity assertion are both already-resolved
    // facts, cited here rather than re-derived.
    //=========================================================================
    // src0/src1 RAW-except (ctrl.v:431-461, 3 terms each).
    wire raw0_except = (src0_type == WB_INT_TYPE_ALU) || (src0_type == WB_INT_TYPE_BJU)
                     || ((src0_type == WB_INT_TYPE_LSU) && dis_is_condbr
                         && (src0_cnt == 2'd0 || src0_cnt == 2'd1))
                     || (fwd_src0_vld
                         && !((src0_type == WB_INT_TYPE_LSU || src0_type == WB_INT_TYPE_MULT)
                              && (src0_cnt == 2'd2)));
    wire raw1_except = (src1_type == WB_INT_TYPE_ALU) || (src1_type == WB_INT_TYPE_BJU)
                     || ((src1_type == WB_INT_TYPE_LSU) && dis_is_condbr
                         && (src1_cnt == 2'd0 || src1_cnt == 2'd1))
                     || (fwd_src1_vld
                         && !((src1_type == WB_INT_TYPE_LSU || src1_type == WB_INT_TYPE_MULT)
                              && (src1_cnt == 2'd2)));
    // src2 RAW-except adds the 4th, store-data-from-load term (ctrl.v:478-484).
    wire raw2_except = (src2_type == WB_INT_TYPE_ALU) || (src2_type == WB_INT_TYPE_BJU)
                     || ((src2_type == WB_INT_TYPE_LSU) && dis_is_condbr
                         && (src2_cnt == 2'd0 || src2_cnt == 2'd1))
                     || (fwd_src2_vld
                         && !((src2_type == WB_INT_TYPE_LSU || src2_type == WB_INT_TYPE_MULT)
                              && (src2_cnt == 2'd2)))
                     || ((src2_type == WB_INT_TYPE_LSU) && dis_is_store
                         && (src2_cnt == 2'd0 || src2_cnt == 2'd1 || src2_cnt2));

    wire raw0 = dis_src0_vld && !src0_vld_wbt && !raw0_except;
    wire raw1 = dis_src1_vld && !src1_vld_wbt && !raw1_except;
    wire raw2 = dis_src2_vld && !src2_vld_wbt && !raw2_except;

    // dst0 WAW-except (ctrl.v:508-521; dst1 doesn't exist in M2's decode --
    // no instruction ever sets a dst1_vld -- so no dst1 term is needed).
    wire waw0_except =
           (((dst0_type_wbt == WB_INT_TYPE_LSU)  && (dis_dst0_type == WB_INT_TYPE_LSU))
         || ((dst0_type_wbt == WB_INT_TYPE_MULT) && (dis_dst0_type == WB_INT_TYPE_MULT)))
           && (dst0_cnt == 2'd0 || dst0_cnt == 2'd1 || dst0_cnt2)
        || (dst0_type_wbt == WB_INT_TYPE_ALU) || (dst0_type_wbt == WB_INT_TYPE_BJU);

    wire waw0 = dis_dst0_vld && !dst0_vld_wbt && !waw0_except;

    // M5 Task 11: FRF-side hazard terms (rawf0/1/2/wawf, SECTION FWBT
    // above) fold in here exactly like the GPR raw0/1/2/waw0 terms --
    // same dis_dep_stall consumer (ctrl_dis_stall), same effect (hold this
    // dispatch in ID for another cycle).
    wire dis_dep_stall = raw0 || raw1 || raw2 || waw0
                        || rawf0 || rawf1 || rawf2 || wawf;

    //=========================================================================
    // SECTION EX1 REGISTER + LATE FORWARD (IDU note S2/S6, aq_idu_id_dp.v:
    // 893-997, ctrl.v:598-616) -- the ONLY register inside IDU.v proper.
    //=========================================================================
    reg                     ex1_vld_r;
    reg [EU_WIDTH-1:0]      ex1_eu_r;
    reg [FUNC_WIDTH-1:0]    ex1_func_r;
    reg [63:0]              ex1_src0_data_r, ex1_src1_data_r, ex1_src2_data_r;
    reg                     ex1_src0_rdy_r, ex1_src1_rdy_r, ex1_src2_rdy_r;
    reg [4:0]               ex1_src0_reg_r, ex1_src1_reg_r, ex1_src2_reg_r;
    reg [4:0]               ex1_dst0_reg_r;
    // M5 Task 4c: FRF-vs-GPR destination tag for LSU-class ops (FLW/FLD),
    // latched alongside ex1_dst0_reg_r the same way.
    reg                     ex1_dst0_frf_r;
    reg [1:0]               ex1_bht_pred_r;
    reg [31:0]              ex1_opcode_r;
    reg                     ex1_illegal_r;
    // M5 Task 4b: latched funct3/rm field, feeds idu_fpu_ex1_rm below.
    reg [2:0]               ex1_rm_r;
    // M4 Task 6: fetch-fault siblings of ex1_illegal_r, latched the same
    // cycle/same way.
    reg                     ex1_fetch_pgflt_r, ex1_fetch_accflt_r;
    // Task 7.3: latched length of the EX1 instruction (1=32b,0=16b). Source
    // is the combinational `is32` discriminator (line ~273), latched the
    // same cycle the EX1 data registers load. Feeds idu_iu_ex1_inst_len /
    // idu_lsu_ex1_inst_len / idu_cp0_ex1_inst_len below.
    reg                     ex1_inst_len_r;
    // M7 Task 2: the EX1-resident instruction's execute-trigger halt_info
    // (the 22-bit TDT_HINFO bundle the IFU delivers live at the ibuf head;
    // donor aq_idu_id_dp.v's ex1_halt_info). Latched/reloaded with the
    // rest of the EX1 context; feeds idu_iu_ex1_halt_info below.
    reg [TDT_HINFO_WIDTH-1:0] ex1_halt_info_r;

    // EX1 issue-gate stall terms (ctrl.v:669-692, M2-simplified per header:
    // M2-simplified per header: no lsu_idu_global_full port exists; the CP0
    // issue-stall is the fencei_full EX1-hold, Task 10.1).
    wire ctrl_ex1_eu_full = (ex1_eu_r[EU_BJU_SEL]  && iu_idu_bju_full)
                          || (ex1_eu_r[EU_MULT_SEL] && iu_idu_mult_full)
                          || (ex1_eu_r[EU_DIV_SEL]  && iu_idu_div_full)
                          || (ex1_eu_r[EU_LSU_SEL]  && lsu_idu_full)
                          || (ex1_eu_r[EU_CP0_SEL]  && cp0_idu_fencei_full)
                          // M5 Task 6: holds the FDSU op resident in EX1
                          // while FPU.v's busy-flop churns, same shape as
                          // the EU_DIV_SEL term above (design doc D1).
                          // M5 Task 11 fix: same FUSED/FDSU_DIV aliasing
                          // exclusion as idu_fpu_ex1_fdsu_sel -- without it
                          // a fused FMA op parked in EX1 would stall the
                          // pipe via fpu_idu_fdsu_full.
                          || (ex1_eu_r[EU_FP_SEL]
                              && (ex1_func_r[FUNC_FDSU_DIV] || ex1_func_r[FUNC_FDSU_SQRT])
                              && !ex1_func_r[FUNC_MAU_MUL]
                              && fpu_idu_fdsu_full);
    wire ctrl_ex1_issue_stall    = ex1_vld_r && iu_idu_mult_issue_stall;
    // A store's store-data operand (src2) is deliberately exempted from the
    // dispatch-time RAW stall when its producer is a still-in-flight LSU op
    // (raw2_except's WB_INT_TYPE_LSU/dis_is_store term, ~line 1269) -- the
    // design instead relies on the EX1-resident lf2_hit/rtu_idu_wb1 late
    // forward (below) to backfill src2 while the store sits parked in EX1.
    // Donor aq_lsu_ag.v:497,667 computes the analogous
    // `ag_src2_depd = !idu_lsu_ex1_src2_ready` and factors it into its own
    // `ag_stall`/`ag_req_buffer_src2_depd` (aq_lsu_ag.v:885-930,1065),
    // snooping a forward bus while parked until the value lands -- i.e. the
    // donor's LSU never accepts a store whose data isn't ready yet. Without
    // an equivalent hold here, `idu_lsu_ex1_sel` (~line 1401) would fire
    // unconditionally once !ctrl_ex1_internal_stall, handing LSU a stale
    // (reset-value) src2 before the producing load's writeback (wb1) lands.
    wire ex1_is_store          = ex1_eu_r[EU_LSU_SEL] && ex1_func_r[0];
    wire ex1_store_src2_unrdy  = ex1_vld_r && ex1_is_store && !ex1_src2_rdy_r;
    wire ctrl_ex1_internal_stall = (ex1_vld_r && iu_idu_bju_global_full) || ex1_store_src2_unrdy;
    wire ctrl_ex1_stall = ctrl_ex1_eu_full || ctrl_ex1_issue_stall || ctrl_ex1_internal_stall;

    wire ctrl_dis_stall = rtu_idu_flush_stall || ctrl_ex1_stall || dis_dep_stall;

    wire ctrl_pipedown_inst_vld = ifu_idu_id_inst_vld && !ctrl_dis_stall;
    wire adv = !ctrl_ex1_stall;   // "advance" -- reload EX1 from ID/DIS this cycle

    // EX1-resident late forward (dp.v:893-997) -- checks ONLY wb0/wb1
    // (not fwd0/1/2), for an operand latched-but-not-ready, matched against
    // the LATCHED register number (not the current dispatch's).
    wire lf0_wb0 = rtu_idu_wb0_vld && (rtu_idu_wb0_reg[4:0] == ex1_src0_reg_r);
    wire lf0_wb1 = rtu_idu_wb1_vld && (rtu_idu_wb1_reg[4:0] == ex1_src0_reg_r);
    wire lf0_hit = !ex1_src0_rdy_r && (lf0_wb0 || lf0_wb1);
    wire lf1_wb0 = rtu_idu_wb0_vld && (rtu_idu_wb0_reg[4:0] == ex1_src1_reg_r);
    wire lf1_wb1 = rtu_idu_wb1_vld && (rtu_idu_wb1_reg[4:0] == ex1_src1_reg_r);
    wire lf1_hit = !ex1_src1_rdy_r && (lf1_wb0 || lf1_wb1);
    wire lf2_wb0 = rtu_idu_wb0_vld && (rtu_idu_wb0_reg[4:0] == ex1_src2_reg_r);
    wire lf2_wb1 = rtu_idu_wb1_vld && (rtu_idu_wb1_reg[4:0] == ex1_src2_reg_r);
    wire lf2_hit = !ex1_src2_rdy_r && (lf2_wb0 || lf2_wb1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex1_vld_r <= 1'b0;
            ex1_eu_r  <= {EU_WIDTH{1'b0}};
        end else if (rtu_idu_flush_fe || iu_idu_br_cancel) begin
            ex1_vld_r <= 1'b0;
            ex1_eu_r  <= {EU_WIDTH{1'b0}};
        end else if (adv) begin
            ex1_vld_r <= ctrl_pipedown_inst_vld;
            // Guard against a stale/spurious EU tag riding along when this
            // dispatch is NOT actually valid (bubble, or a hazard-blocked
            // dispatch that still had adv=1 because ctrl_ex1_stall itself
            // only reflects the EX1-RESIDENT instruction, not the incoming
            // one) -- every idu_*_ex1_*_sel formula below keys off
            // ex1_eu_r alone, so ex1_vld_r==0 must imply ex1_eu_r==0.
            ex1_eu_r  <= ctrl_pipedown_inst_vld ? dis_eu_final : {EU_WIDTH{1'b0}};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex1_func_r      <= {FUNC_WIDTH{1'b0}};
            ex1_src0_data_r <= 64'd0; ex1_src1_data_r <= 64'd0; ex1_src2_data_r <= 64'd0;
            ex1_src0_rdy_r  <= 1'b0;  ex1_src1_rdy_r  <= 1'b0;  ex1_src2_rdy_r  <= 1'b0;
            ex1_src0_reg_r  <= 5'd0;  ex1_src1_reg_r  <= 5'd0;  ex1_src2_reg_r  <= 5'd0;
            ex1_dst0_reg_r  <= 5'd0;
            ex1_dst0_frf_r  <= 1'b0;
            ex1_bht_pred_r  <= 2'd0;
            ex1_opcode_r    <= 32'd0;
            ex1_illegal_r   <= 1'b0;
            ex1_fetch_pgflt_r  <= 1'b0;
            ex1_fetch_accflt_r <= 1'b0;
            ex1_inst_len_r  <= 1'b0;
            ex1_rm_r        <= 3'b000;
            ex1_halt_info_r <= {TDT_HINFO_WIDTH{1'b0}};
        end else if (adv) begin
            ex1_func_r      <= dis_func;
            ex1_src0_data_r <= dis_src0_data; ex1_src0_rdy_r <= dis_src0_rdy; ex1_src0_reg_r <= dis_src0_reg5;
            ex1_src1_data_r <= dis_src1_data; ex1_src1_rdy_r <= dis_src1_rdy; ex1_src1_reg_r <= dis_src1_reg5;
            ex1_src2_data_r <= dis_src2_data; ex1_src2_rdy_r <= dis_src2_rdy; ex1_src2_reg_r <= dis_src2_reg5;
            ex1_dst0_reg_r  <= dis_dst0_reg5;
            ex1_dst0_frf_r  <= dis_dst0_frf;
            ex1_bht_pred_r  <= ifu_idu_id_bht_pred;
            ex1_opcode_r    <= inst;
            ex1_illegal_r   <= dis_illegal;
            ex1_fetch_pgflt_r  <= dis_fetch_pgflt;
            ex1_fetch_accflt_r <= dis_fetch_accflt;
            ex1_inst_len_r  <= is32;
            ex1_rm_r        <= dis_rm;
            ex1_halt_info_r <= ifu_idu_id_halt_info;
        end else begin
            if (lf0_hit) begin ex1_src0_data_r <= lf0_wb0 ? rtu_idu_wb0_data : rtu_idu_wb1_data; ex1_src0_rdy_r <= 1'b1; end
            if (lf1_hit) begin ex1_src1_data_r <= lf1_wb0 ? rtu_idu_wb0_data : rtu_idu_wb1_data; ex1_src1_rdy_r <= 1'b1; end
            if (lf2_hit) begin ex1_src2_data_r <= lf2_wb0 ? rtu_idu_wb0_data : rtu_idu_wb1_data; ex1_src2_rdy_r <= 1'b1; end
        end
    end

    //=========================================================================
    // SECTION FRF EX1 LATCH (M5 Task 2, D2) -- mirrors ex1_src*_data_r
    // above exactly: unconditional on `adv`, no EU-dispatch/legality
    // gating (no FP consumer exists yet to need either), no late-forward
    // network (Task 4/5 add hazard tracking once FMA exists).
    //=========================================================================
    // M5 Task 7: fcvt.{s,d}.{w,wu,l,lu} and fmv.{w.x,d.x} read their
    // source from the INTEGER regfile (rs1, inst[19:15]) rather than the
    // FP regfile -- dis_src0_data (the already-forwarding-resolved GPR
    // value read via the SAME inst[19:15] field, see dis_src0_reg5) is
    // substituted for frf_src0_data on exactly these two int-sourced
    // op groups. Every other FP op (dis_gpr_fsrc0=0) is bit-identical to
    // before this task.
    // M5 Task 11 BUG 1 (Category A, rv906-own): the FUNC_SPU_MV term is
    // AND-gated on FUNC_SPU_SGN because FUNC_SPU_MV(17) aliases FUNC_MAU_NEG
    // (rvproc_pkg.sv:932) -- fnmsub/fnmadd genuinely set that bit position,
    // so a bare FUNC_SPU_MV read would spuriously route the GPR read
    // (dis_src0_data) onto the FMA addend (a) operand in place of the FRF
    // read. The four FMV decode arms (~1124-1141) all set FUNC_SPU_SGN
    // alongside FUNC_SPU_MV (see the comment there); no other instruction
    // class sets the pair, so FMAU NEG ops must not select the GPR source.
    wire dis_gpr_fsrc0 = (dis_func[FUNC_CVT_INT] && !dis_func[FUNC_CVT_F2I])
                       || (dis_func[FUNC_SPU_SGN] && dis_func[FUNC_SPU_MV] && !dis_func[FUNC_SPU_MV_XF]);
    reg [63:0] ex1_fsrc0_data_r, ex1_fsrc1_data_r, ex1_fsrc2_data_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex1_fsrc0_data_r <= 64'd0; ex1_fsrc1_data_r <= 64'd0; ex1_fsrc2_data_r <= 64'd0;
        end else if (adv) begin
            ex1_fsrc0_data_r <= dis_gpr_fsrc0 ? dis_src0_data : frf_src0_data;
            ex1_fsrc1_data_r <= frf_src1_data;
            ex1_fsrc2_data_r <= frf_src2_data;
        end
    end

    assign idu_fpu_ex1_fsrc0_data = ex1_fsrc0_data_r;
    assign idu_fpu_ex1_fsrc1_data = ex1_fsrc1_data_r;
    assign idu_fpu_ex1_fsrc2_data = ex1_fsrc2_data_r;

    //=========================================================================
    // SECTION EU DISPATCH (IDU note S7, ctrl.v:619-663) -- the EX1
    // issue-gate: `!ctrl_ex1_internal_stall && rtu_idu_commit &&
    // !<EU>_idu_full` per EU. An instruction can sit valid-but-not-issuing
    // in EX1 (ex1_vld_r=1 but none of the *_sel below fire) -- this is NOT
    // the same condition as ex1_vld_r itself (IDU note S7's "Critically").
    //=========================================================================
    assign idu_iu_ex1_inst_vld     = ex1_vld_r;
    assign idu_iu_ex1_pipedown_vld = ex1_vld_r && !ctrl_ex1_stall && rtu_idu_commit;

    assign idu_iu_ex1_alu_sel    = ex1_eu_r[EU_ALU_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit;
    assign idu_iu_ex1_bju_sel    = ex1_eu_r[EU_BJU_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit && !iu_idu_bju_full;
    assign idu_iu_ex1_bju_br_sel = ex1_eu_r[EU_BJU_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit_for_bju && !iu_idu_bju_full;
    assign idu_iu_ex1_mult_sel   = ex1_eu_r[EU_MULT_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit && !iu_idu_mult_full;
    assign idu_iu_ex1_div_sel    = ex1_eu_r[EU_DIV_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit && !iu_idu_div_full;
    assign idu_cp0_ex1_sel       = ex1_eu_r[EU_CP0_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit;
    assign idu_lsu_ex1_sel       = ex1_eu_r[EU_LSU_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit && !lsu_idu_full;
    assign idu_lsu_ex1_dp_sel    = ex1_eu_r[EU_LSU_SEL]  && !ctrl_ex1_internal_stall && !lsu_idu_full;
    // M5 Task 4: FP dispatch selects (see the port declaration comment).
    assign idu_fpu_ex1_fadd_sel  = ex1_eu_r[EU_FP_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit
                                  && (ex1_func_r[FUNC_ADD] || ex1_func_r[FUNC_SUB]
                                      || ex1_func_r[FUNC_CMP] || ex1_func_r[FUNC_MAX] || ex1_func_r[FUNC_MIN]);
    // M5 Task 7: fmv.{x.w,w.x,x.d,d.x} dispatch into FSPU rides the
    // decode arms also setting FUNC_SPU_SGN (see below) -- NOT a new
    // bare OR-term on FUNC_SPU_MV, because FUNC_SPU_MV aliases
    // FUNC_MAU_NEG's bit position (rvproc_pkg.sv), which IS genuinely
    // set by fnmadd.{s,d}/fnmsub.{s,d} (FMAU); a bare OR on it here
    // would spuriously co-fire fspu_sel alongside fmau_sel for those
    // four instructions and (SECTION 13's fspu>fmau priority) corrupt
    // their result. FUNC_SPU_MV stays purely AND-gated (spu_op_mv_fx/
    // mv_xf in FPU.v), which is safe. FUNC_CVT_INT (FCNVT dispatch,
    // below) has no such collision: its aliased bit (FUNC_CMP_FNE) is
    // never set by any decode arm.
    assign idu_fpu_ex1_fspu_sel  = ex1_eu_r[EU_FP_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit
                                  && (ex1_func_r[FUNC_SPU_SGN] || ex1_func_r[FUNC_CLASS]);
    assign idu_fpu_ex1_fcnvt_sel = ex1_eu_r[EU_FP_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit
                                  && (ex1_func_r[FUNC_CVT_WIDDEN] || ex1_func_r[FUNC_CVT_NARROW] || ex1_func_r[FUNC_CVT_INT]);
    // M5 Task 5: FUNC_MAU_MUL alone identifies FMAU-class ops (see the
    // port declaration comment -- FUSED/SUB/NEG can't serve this role
    // since plain fmul sets FUSED=0 with SUB/NEG don't-care).
    assign idu_fpu_ex1_fmau_sel  = ex1_eu_r[EU_FP_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit
                                  && ex1_func_r[FUNC_MAU_MUL];
    // M5 Task 6: FDSU dispatch select, mirrors idu_iu_ex1_div_sel's own
    // !iu_idu_div_full term (design doc D1's structural precedent) --
    // without it this would keep pulsing while the held EX1 op waits out
    // FPU.v's busy-flop FSM (ctrl_ex1_eu_full above is what actually holds
    // the op resident in EX1; this term stops it from re-dispatching every
    // one of those held cycles).
    // M5 Task 11 fix: FUSED(5)/SUB(7) are aliased with FUNC_FDSU_DIV/SQRT
    // (rvproc_pkg.sv) -- every fused FMA op sets FUNC_MAU_MUL(19) AND
    // FUNC_MAU_FUSED(5) (decode arms above, ~1044-1077), so without the
    // !FUNC_MAU_MUL exclusion fmadd/fmsub/fnmadd/fnmsub spuriously fire
    // fdsu_sel and dispatch the FDSU FSM on FMA operands: 14/29 cycles
    // later fdsu_cmplt_now raises a second (wrong) fvld/wbf0 into the FMA
    // dst (corrupting result + fflags) and breaks the FWBT count (hang).
    // fdiv/fsqrt never set FUNC_MAU_MUL (decode arms ~1020-1029), so the
    // exclusion is exact.
    assign idu_fpu_ex1_fdsu_sel  = ex1_eu_r[EU_FP_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit
                                  && (ex1_func_r[FUNC_FDSU_DIV] || ex1_func_r[FUNC_FDSU_SQRT])
                                  && !ex1_func_r[FUNC_MAU_MUL]
                                  && !fpu_idu_fdsu_full;
    assign idu_fpu_ex1_func      = ex1_func_r;
    assign idu_fpu_ex1_rm        = ex1_rm_r;
    assign idu_fpu_ex1_dst0_reg  = {1'b0, ex1_dst0_reg_r};
    // M4 Task 5 fix: same term set as idu_lsu_ex1_sel, minus !lsu_idu_full --
    // see the port declaration comment (~line 186).
    assign idu_lsu_ex1_raw_vld   = ex1_eu_r[EU_LSU_SEL]  && !ctrl_ex1_internal_stall && rtu_idu_commit;

    assign idu_iu_ex1_func       = ex1_func_r;
    assign idu_iu_ex1_src0_data  = ex1_src0_data_r;
    assign idu_iu_ex1_src0_ready = ex1_src0_rdy_r;
    assign idu_iu_ex1_src1_data  = ex1_src1_data_r;
    assign idu_iu_ex1_src1_ready = ex1_src1_rdy_r;
    assign idu_iu_ex1_src2_data  = ex1_src2_data_r;
    assign idu_iu_ex1_src2_ready = ex1_src2_rdy_r;
    assign idu_iu_ex1_dst0_reg   = {1'b0, ex1_dst0_reg_r};
    assign idu_iu_ex1_bht_pred   = ex1_bht_pred_r;
    assign idu_iu_ex1_src0_reg   = {1'b0, ex1_src0_reg_r};
    assign idu_iu_ex1_src1_reg   = {1'b0, ex1_src1_reg_r};
    assign idu_iu_ex1_inst_len   = ex1_inst_len_r;
    assign idu_iu_ex1_halt_info  = ex1_halt_info_r;

    assign idu_lsu_ex1_func       = ex1_func_r;
    assign idu_lsu_ex1_src0_data  = ex1_src0_data_r;
    assign idu_lsu_ex1_src0_ready = ex1_src0_rdy_r;
    assign idu_lsu_ex1_src1_data  = ex1_src1_data_r;
    assign idu_lsu_ex1_src1_ready = ex1_src1_rdy_r;
    assign idu_lsu_ex1_src2_data  = ex1_src2_data_r;
    assign idu_lsu_ex1_src2_ready = ex1_src2_rdy_r;
    assign idu_lsu_ex1_dst0_reg   = {1'b0, ex1_dst0_reg_r};
    // M5 Task 4c: FLW/FLD write FRF instead of GPR (D8) -- LSU.v threads
    // this alongside dst0_reg through AG/DC/LFB to pick the write-back
    // register file at commit.
    assign idu_lsu_ex1_dst0_frf   = ex1_dst0_frf_r;
    assign idu_lsu_ex1_inst_len   = ex1_inst_len_r;

    assign idu_cp0_ex1_func      = ex1_func_r;
    assign idu_cp0_ex1_opcode    = ex1_opcode_r;
    assign idu_cp0_ex1_illegal   = ex1_illegal_r;
    assign idu_cp0_ex1_fetch_pgflt  = ex1_fetch_pgflt_r;
    assign idu_cp0_ex1_fetch_accflt = ex1_fetch_accflt_r;
    assign idu_cp0_ex1_src0_data = ex1_src0_data_r;
    assign idu_cp0_ex1_src1_data = ex1_src1_data_r;
    assign idu_cp0_ex1_dst0_reg  = {1'b0, ex1_dst0_reg_r};
    assign idu_cp0_ex1_inst_len  = ex1_inst_len_r;

    //=========================================================================
    // SECTION FRONT-END STALL (ctrl.v:350-394, M2-simplified: no split FSM
    // exists, contract 10, so `ctrl_split_stall` is always 0).
    //=========================================================================
    assign idu_ifu_id_stall = ctrl_dis_stall;

    //=========================================================================
    // Unused-port hygiene: `rtu_idu_pipeline_empty` is a real, wired input
    // (port-list amendment, see header) with no consumer in this body --
    // referenced here only so lint tools never flag it as a dangling,
    // never-driven-anywhere mistake; it carries no logical effect.
    //=========================================================================
    wire dbg_pipeline_empty_seen = rtu_idu_pipeline_empty;

endmodule
