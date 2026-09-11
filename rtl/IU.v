//=============================================================================
// IU.v - integer execute pipe: ALU + BJU + MULT + DIV  (M2 Task 3: real body)
//=============================================================================
// C906 files covered:
//   gen_rtl/iu/rtl/aq_iu_top.v      (glue: one ALU, one BJU+addr_gen, one
//                                     MULT, one DIV -- genuinely single
//                                     execute pipe, IU note S0)
//   gen_rtl/iu/rtl/aq_iu_alu.v      (adder/shifter/logic/misc, comb, EX1)
//   gen_rtl/iu/rtl/aq_iu_bju.v      (comparator, mispredict/RAS/BHT
//                                     feedback, the 1-entry LSU-dependent
//                                     branch buffer)
//   gen_rtl/iu/rtl/aq_iu_addr_gen.v (shared 64b adder: branch/JAL/JALR
//                                     target + AUIPC's pc+imm)
//   gen_rtl/iu/rtl/aq_iu_mul.v      (33x33 Booth-radix-4, EX1-EX3)
//   gen_rtl/iu/rtl/aq_iu_div.v      (+ aq_iu_div_shift2_kernel.v; radix-4
//                                     2b/cycle non-restoring, memo buffer)
// References: design doc S2.1/S4.1/S4.3 (unit graph, file org), S6 (the
// four-writeback-bus decision, contract 1), IU extraction note (all
// sections), IDU note S6/S10 (RTU-sourced forwarding, no direct IU->IDU
// bypass wire).
//
// CLEAN-ROOM NOTE (read before comparing against the donor line-by-line):
// this body implements the MECHANISM each note section describes (one
// shared adder/shifter/comparator; the Booth-multiplier's variable-latency
// iteration shape; the divider's leading-1 early-out + memo buffer), using
// exact-equality decode against the pinned ALU_FUNC_*/BJU_FUNC_*/
// MULT_FUNC_*/DIV_FUNC_* constants (rvproc_pkg.sv, Task 3.6a) -- the same
// style CSR.v (Task 2) already established for CP0_FUNC_*. It does NOT
// re-derive C906's internal onehot operand-prepare bit-select signals
// (alu.v:198-236) or literally port the 33x33 Booth array / radix-4
// compare-subtract kernel gate-for-gate: MULT/DIV's actual VALUE is
// computed with ordinary Verilog arithmetic (a legitimate simplification --
// low bits of a product/quotient are the same either way, and synthesis-
// level multiplier/divider design was never this task's scope, IU note
// S12's own closing note) while the FSM states, iteration-count-derived
// variable latency, and RTU-grant-gated handshake are faithfully modeled,
// since THAT timing is what Task 9's bring-up ladder and RTU's writeback
// arbiter actually depend on.
//
// MAX/MIN/ADDSL RE-VERIFICATION (Task 3.1's own required re-check, not
// taken on faith from the note): grepped aq_idu_id_decd.v end to end.
// FUNC_MAX/FUNC_MIN/FUNC_MAXU/FUNC_MINU/FUNC_MAXW/... appear NOWHERE in the
// donor's decode tables (zero hits) -- genuinely dead, confirmed by BOTH
// alu.v's own commented-out result-select mux (alu.v:246-247,254-265,
// "4. max/min : src0 or src1") AND the fact decode never produces the func
// value that mux would need. ADDSL is a more precise story than the note's
// blanket phrasing suggests: `FUNC_ADDSL` IS produced by decd.v's "perf"
// custom-opcode table (decd.v:3189-3196), and alu.v's own addsl mechanism
// (`alu_adder_op_addsl = alu_func[5]`, alu.v:192, feeding
// `alu_adder_src1_tmp`, alu.v:218-220) is NOT commented out -- it is live,
// reachable RTL, unlike MAX/MIN. Regardless, ADDSL is not RV64IMC and is
// not in the design doc's/this task's own enumerated ALU op list (S2.1:
// "AND/OR/XOR; XThead REV/TST/FF0/FF1/MVEQZ/MVNEZ") -- so it is still not
// ported here, for the more precise reason "out of M2's own ISA target"
// rather than "dead in this C906 build" (which is only exactly true for
// MAX/MIN). No ALU_FUNC_ADDSL/MAX/MIN constant exists in rvproc_pkg.sv.
//
// PORT-LIST AMENDMENT (documented, not silent -- same discipline CSR.v's
// Task 2 "TASK 2 DISCOVERED GAP, FIXED HERE" note used for
// `cp0_rtu_trap_pc`): Task 1's frozen IU.v port list carried every BJU
// output the IFU-facing consumer side needs (`iu_ifu_bht_pred`, `_taken`,
// `_mispred`, `iu_ifu_ret_vld`, `iu_ifu_pc_mispred`, ...) but omitted THREE
// inputs those outputs cannot be produced without:
//   - `idu_iu_ex1_bht_pred[1:0]` -- the BHT prediction bits for the CURRENT
//     dispatch. Without this, `iu_ifu_bht_mispred`/`_pred` cannot be
//     computed at all (there is nothing to compare the resolved direction
//     against). The data already exists one hop upstream, frozen since M1
//     (`ifu_idu_id_bht_pred[1:0]`, RVProc.v's IFU<->IDU handoff) -- IDU's
//     Task 5 carries it the rest of the way into `idu_iu_ex1_*`.
//   - `idu_iu_ex1_src0_reg`/`idu_iu_ex1_src1_reg` (GPR_IDX_WIDTH each) --
//     the REGISTER NUMBERS of src0/src1 (not just their data+ready bits).
//     Needed for (a) `iu_ifu_ret_vld`/`iu_ifu_pc_mispred` (both are a
//     `rs1==x1`-vs-`!=x1` check on JALR, a register-NUMBER fact, not a
//     value fact) and (b) BJU's own entry buffer matching an incoming
//     `da_xx_fwd_dst_reg`/`lsu_iu_ex2_dest_reg` against the SPECIFIC
//     register the parked branch is waiting on (bju.v:430-431,453-456,
//     458-459) -- without register numbers this buffer could only guess
//     "any forward releases me," which risks accepting a value that was
//     never meant for this branch.
// Nothing instantiates IU.v yet (RVProc.v still runs FetchSink.v until
// Task 7), so this amendment breaks nothing today -- exactly the same
// "nothing consumes this port today" reasoning CSR.v's Task 2 note used.
//
// RESOLVED in Task 7.3 (the core-swap integration, which finally gave the
// pipeline a real RTU retire timing to re-verify against -- the trigger the
// Task 3 note asked for):
//   - `idu_iu_ex1_inst_len` (1=32b/0=16b RVC) is now a real IDU->IU input,
//     and BJU's PC increment is RVC-aware (`bju_inc_pc_live`/`bju_inc_pc_rt`,
//     aq_iu_bju.v:576-583). `iu_rtu_ex1_{alu,bju}_inst_len` now report the
//     real completing length (was tied to the "32-bit" encoding 1'b1).
//   - `rtu_iu_ex1_cmplt`/`_inst_len`/`_inst_split` (RTU retire feedback) are
//     now real RTU->IU inputs. `bju_pcgen_pc` advances on
//     `rtu_iu_ex1_cmplt && !rtu_iu_ex1_inst_split` (donor aq_iu_bju.v:696-697),
//     NOT the dispatch-side `idu_iu_ex1_inst_vld` stand-in -- that stand-in
//     was observed to over-advance the tracker ~91 instructions in RVC-mixed
//     code and self-pin it (see the PC-generator comment). `_inst_split` is
//     tied 0 in M2 (no split classes reach an EU); `_inst_len` is the
//     completing EU's length muxed in the RTU (aq_rtu_dp.v:350-379).
// Still OPEN scope gaps on this port list:
//   - No `rtu_iu_ex2_cur_pc`/`_next_pc` input (the donor's parked-entry
//     bht/hpcp recompute, aq_iu_bju.v:702-703) -- not needed by any M2
//     consumer.
//   - No `ifu_iu_ex1_pc_pred` input exists (the RAS-predicted-target BJU
//     would compare a JALR-return's real target against). The JALR-vs-RAS
//     mismatch path (`bju_pc_cmp_fail`/`bju_ras_mispred_vld`) is therefore
//     NOT built; `iu_rtu_ex2_bju_ras_mispred` is tied 1'b0. `iu_ifu_ret_vld`
//     /`iu_ifu_pc_mispred` (the register-NUMBER-only checks) ARE still
//     built for real via the port amendment above.
//   - BJU's redirect (`iu_ifu_tar_pc_vld`) is built to fire on exactly the
//     donor's own real conditions (conditional-branch BHT mismatch, or a
//     JALR whose rs1!=x1 i.e. a return the RAS should never have predicted)
//     -- it does NOT unconditionally redirect a "clean" JAL/JALR the way a
//     from-scratch design might default to for safety. This matches real
//     C906 (which relies on the front end's own BTB/decode-time mechanism
//     to already be at the right target by the time BJU resolves an
//     ordinary jump) but is worth re-confirming in Task 9.2's bring-up test
//     once IDU (Task 5) and the full pipe (Task 7) exist, since M2 does not
//     build an equivalent decode-time JAL-target correction path itself.
//
// SEAM NOTES (rv906 decomposition, carried over from the skeleton):
//  * IU.v houses ALL FOUR integer execute units (contract 0/IU note S0) --
//    no separate ALU.v/BJU.v/MULT.v/DIV.v file, per the design doc's S4.3
//    file-org table.
//  * IU exposes FOUR SEPARATE writeback buses to RTU (ALU/BJU/MULT/DIV,
//    contract 1) -- no merge happens inside this module. None of the four
//    buses carry exception/fault flags (IU note S10).
//  * The IFU-facing BJU redirect/BHT-feedback/RAS group is reused UNCHANGED
//    from M1 -- every pre-existing name/width was re-verified byte-for-byte
//    against `rtl/RVProc.v`'s current wire declarations (lines 275-291),
//    confirmation not renaming, per Task 3.3's explicit instruction.
//  * `da_xx_fwd_*`/`lsu_iu_ex2_*` (IU note S4.4) are consumed ONLY by BJU's
//    1-entry LSU-dependent conditional-branch buffer -- ordinary ALU/MULT/
//    DIV ops never see them.
//=============================================================================

import rvproc_pkg::*;

module IU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IDU -> IU : EX1 dispatch, IU's slice of the shared id_ex1_t payload
    // (design doc S4.2) + the per-sub-EU select/valid signals (confirmed
    // aq_idu_id_ctrl.v:620-621,627-631 -- `idu_iu_ex1_inst_vld`/
    // `_pipedown_vld` feed BJU's own PC-tracking logic, IU note S4.5; the
    // five `_sel` signals are already gated by commit/full checks on the
    // IDU side, ctrl.v:627-631).
    //=========================================================================
    input  wire                     idu_iu_ex1_inst_vld,
    input  wire                     idu_iu_ex1_pipedown_vld,
    input  wire                     idu_iu_ex1_alu_sel,
    input  wire                     idu_iu_ex1_bju_sel,
    input  wire                     idu_iu_ex1_bju_br_sel,
    input  wire                     idu_iu_ex1_mult_sel,
    input  wire                     idu_iu_ex1_div_sel,
    input  wire [FUNC_WIDTH-1:0]    idu_iu_ex1_func,
    input  wire [63:0]              idu_iu_ex1_src0_data,
    input  wire                     idu_iu_ex1_src0_ready,
    input  wire [63:0]              idu_iu_ex1_src1_data,
    input  wire                     idu_iu_ex1_src1_ready,
    input  wire [63:0]              idu_iu_ex1_src2_data,
    input  wire                     idu_iu_ex1_src2_ready,
    input  wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_dst0_reg,

    //=========================================================================
    // IDU -> IU : Task 3 port-list amendment (documented, see header) --
    // BJU-only consumers: the BHT prediction bits for this dispatch, and
    // the register NUMBERS of src0/src1 (idu_iu_ex1_src0/1_data above are
    // VALUES only). ALU/MULT/DIV never read these.
    //=========================================================================
    input  wire [1:0]               idu_iu_ex1_bht_pred,
    input  wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_src0_reg,
    input  wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_src1_reg,
    // Task 7.3 port-list amendment (closes the header's "fixed +4" scope
    // gap): the EX1 instruction's LENGTH (1=32-bit, 0=16-bit RVC), latched
    // in IDU's EX1. Drives BJU's RVC-aware PC increment and the real
    // iu_rtu_ex1_{alu,bju}_inst_len. Donor ref: aq_iu_bju.v:127.
    input  wire                     idu_iu_ex1_inst_len,

    //=========================================================================
    // IU -> IDU : point-to-point stall/full signals (contract 8; confirmed
    // names mul.v:597-598, div.v:764, bju.v:477-478).
    //=========================================================================
    output wire                     iu_idu_mult_issue_stall,
    output wire                     iu_idu_mult_full,
    output wire                     iu_idu_div_full,
    output wire                     iu_idu_bju_full,
    output wire                     iu_idu_bju_global_full,

    //=========================================================================
    // IU -> RTU : the four separate writeback buses (contract 1, IU note
    // S10). ALU (EX1, always-valid-if-selected): iu_rtu_ex1_alu_*.
    //=========================================================================
    output wire                     iu_rtu_ex1_alu_cmplt,
    output wire                     iu_rtu_ex1_alu_cmplt_dp,
    output wire [63:0]              iu_rtu_ex1_alu_data,
    output wire                     iu_rtu_ex1_alu_inst_len,
    output wire                     iu_rtu_ex1_alu_inst_split,
    output wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_alu_preg,
    output wire                     iu_rtu_ex1_alu_wb_dp,
    output wire                     iu_rtu_ex1_alu_wb_vld,

    // BJU (EX1, or later if the LSU-dependent entry resolves it) -- richer
    // than the others: also reports PC-increment bookkeeping and the
    // delayed-entry mispredict/dependent-LSU-changeflow cases (IU note S10).
    output wire                     iu_rtu_ex1_bju_cmplt,
    output wire                     iu_rtu_ex1_bju_cmplt_dp,
    output wire                     iu_rtu_ex1_bju_cmplt_for_pcgen,
    output wire [63:0]              iu_rtu_ex1_bju_data,
    output wire                     iu_rtu_ex1_bju_inst_len,
    output wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_bju_preg,
    output wire                     iu_rtu_ex1_bju_wb_dp,
    output wire                     iu_rtu_ex1_bju_wb_vld,
    output wire                     iu_rtu_ex1_branch_inst,
    output wire [PC_WIDTH-1:0]      iu_rtu_ex1_cur_pc,
    output wire [PC_WIDTH-1:0]      iu_rtu_ex1_next_pc,
    output wire                     iu_rtu_ex2_bju_ras_mispred,
    output wire                     iu_rtu_depd_lsu_chgflow_vld,
    output wire [PC_WIDTH-1:0]      iu_rtu_depd_lsu_chgflow_next_pc,

    // MULT (EX1 early-accept, EX3 data/writeback -- gated on
    // rtu_iu_mul_wb_grant, IU note S5/S10).
    output wire                     iu_rtu_ex1_mul_cmplt,
    output wire                     iu_rtu_ex1_mul_cmplt_dp,
    output wire [63:0]              iu_rtu_ex3_mul_data,
    output wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex3_mul_preg,
    output wire                     iu_rtu_ex3_mul_wb_vld,
    // Task 7.3: completing-MULT length for the RTU pcgen inst_len mux.
    // Tied 1 (32-bit): M2's RVC decoder (IDU.v) never emits a 16-bit
    // MULT (no c.mul in the d16_eu table), so a MULT completion is always
    // a 32-bit instruction.
    output wire                     iu_rtu_ex1_mul_inst_len,

    // DIV (EX1 early-accept, variable-EX-stage data/writeback -- gated on
    // rtu_iu_div_wb_grant, IU note S6/S10).
    output wire                     iu_rtu_ex1_div_cmplt,
    output wire                     iu_rtu_ex1_div_cmplt_dp,
    output wire [63:0]              iu_rtu_div_data,
    output wire [GPR_IDX_WIDTH-1:0] iu_rtu_div_preg,
    output wire                     iu_rtu_div_wb_dp,
    output wire                     iu_rtu_div_wb_vld,
    // Task 7.3: completing-DIV length for the RTU pcgen inst_len mux.
    // Tied 1 (32-bit): no 16-bit DIV in M2's RVC decoder.
    output wire                     iu_rtu_ex1_div_inst_len,

    //=========================================================================
    // RTU -> IU : the single-bit writeback-race grants for MULT/DIV -- RTU
    // is where the completion-vs-writeback-bus race is resolved, not an
    // IU-internal arbiter (IU note S0/S5/S6).
    //=========================================================================
    input  wire                     rtu_iu_mul_wb_grant,
    input  wire                     rtu_iu_div_wb_grant,

    //=========================================================================
    // RTU -> IU : Task 7.3 PC-generator retire feedback (closes the header's
    // "no rtu_iu_ex1_cmplt/_inst_split" scope gap). The donor's bju.v
    // (aq_iu_bju.v:148-151) advances its PC generator only on a genuine
    // RTU-confirmed EX1 completion of the completing instruction's length,
    // NOT on the dispatch-side idu_iu_ex1_inst_vld our Task 3 stand-in used.
    //   - rtu_iu_ex1_cmplt : the completing-EU OR (donor aq_rtu_ctrl.v:240
    //     ctrl_ex1_cmplt_for_pcgen).
    //   - rtu_iu_ex1_inst_len : the COMPLETING instruction's length (donor
    //     aq_rtu_dp.v:537 rtu_iu_ex1_inst_len = dp_ex1_inst_len) -- drives the
    //     RVC-aware +2/+4 PC increment (aq_iu_bju.v:581).
    //   - rtu_iu_ex1_inst_split : the completing instruction's split flag
    //     (aq_rtu_dp.v:537); gates the advance. Tied 0 in M2 (no split
    //     classes reach an EU -- see RTU.v note), so the gate is vacuous.
    //=========================================================================
    input  wire                     rtu_iu_ex1_cmplt,
    input  wire                     rtu_iu_ex1_inst_len,
    input  wire                     rtu_iu_ex1_inst_split,

    //=========================================================================
    // Already-frozen-since-M1 IFU-facing BJU ports, reused UNCHANGED --
    // names/widths copied verbatim from rtl/RVProc.v's current wire
    // declarations (lines 275-285). `iu_ifu_tar_pc` is intentionally 64 bits
    // (not PC_WIDTH) because that is RVProc.v's own already-frozen M1
    // declaration width.
    //=========================================================================
    output wire                     iu_ifu_tar_pc_vld,
    output wire [63:0]              iu_ifu_tar_pc,
    output wire                     iu_ifu_pc_mispred,
    output wire                     iu_ifu_bht_mispred,
    output wire                     iu_ifu_br_vld,
    output wire                     iu_ifu_bht_taken,
    output wire [1:0]               iu_ifu_bht_pred,
    output wire                     iu_ifu_link_vld,
    output wire                     iu_ifu_ret_vld,
    input  wire                     ifu_iu_chgflw_vld,
    input  wire [PC_WIDTH-1:0]      ifu_iu_chgflw_pc,

    //=========================================================================
    // Donor-faithful branch-mispredict cancel to the IDU. aq_iu_bju.v:783
    // `assign iu_yy_xx_cancel = iu_ifu_tar_pc_vld`; aq_idu_id_ctrl.v:604 ORs
    // it into the EX1-inst-valid cancel (`rtu_idu_flush_fe || iu_yy_xx_cancel`)
    // so the ID-stage wrong-path instruction is dropped from the ID->EX1
    // latch on the mispredict cycle. M2 has no branch predictor, so every
    // taken branch/jump mispredicts (predicted not-taken) and this cancel is
    // what keeps the superseded sequential instruction out of EX1.
    //=========================================================================
    output wire                     iu_idu_br_cancel,

    //=========================================================================
    // BJU's private LSU-dependent conditional-branch forward (IU note
    // S4.4) -- consumed ONLY by BJU's 1-entry buffer, not ALU/MULT/DIV.
    //=========================================================================
    input  wire [63:0]              da_xx_fwd_data,
    input  wire [GPR_IDX_WIDTH-1:0] da_xx_fwd_dst_reg,
    input  wire                     da_xx_fwd_vld,
    input  wire [63:0]              lsu_iu_ex2_data,
    input  wire                     lsu_iu_ex2_data_vld,
    input  wire [GPR_IDX_WIDTH-1:0] lsu_iu_ex2_dest_reg,

    //=========================================================================
    // IU -> CSR : BJU's own PC copy, passed through for mepc/trap-context
    // bookkeeping (IU note S4.5/S9 -- the only IU<->CP0 connection besides
    // config inputs).
    //=========================================================================
    output wire [PC_WIDTH-1:0]      iu_cp0_ex1_cur_pc,

    //=========================================================================
    // IU -> LSU : low 16 bits of the EX1 instruction's PC, consumed ONLY by
    // the LSU's PFB stride-prefetch trainer as its PC tag (donor
    // iu_lsu_ex1_cur_pc, aq_lsu_ag.v:196/685 -> dc_ld_pc, aq_lsu_dc.v:1459).
    // M3b Task D. PC_WIDTH-wide tracker, sliced to [15:0] at the port.
    //=========================================================================
    output wire [15:0]              iu_lsu_ex1_cur_pc,

    //=========================================================================
    // IU -> LSU : full-width EX1 instruction PC (the pcgen display), M6
    // Task 7. The LSU latches it at AG-issue into its own replying-op pc
    // (dc_pc_full_r / lfb_pc) so a delayed ST_REPLY/LFB-retire can report
    // the replying op's real PC to the RTU's epc path -- the 16-bit PFB
    // tag above is too narrow for an epc. Same source as iu_cp0_ex1_cur_pc
    // (bju_pcgen_pc): valid for the EX1-resident op, decays when EX1 is
    // empty, which is exactly why the LSU must latch it at issue.
    //=========================================================================
    output wire [PC_WIDTH-1:0]      iu_lsu_ex1_pc,

    //=========================================================================
    // CP0 -> IU : BJU's reset PC seed (already exists as an M1 port on
    // RVProc.v, `cp0_xx_mrvbr` -- restore-checklist item 4, IU note S4.5).
    //=========================================================================
    input  wire [PC_WIDTH-1:0]      cp0_xx_mrvbr
);

    //=========================================================================
    // FEEDBACK SECTION (umbrella S6.2 rule 4) -- every signal flowing
    // backward against the nominal IDU->IU->RTU flow, with its source:
    //   ifu_iu_chgflw_vld/_pc     <- IFU/RTU (via IFU), trap/exception redirect
    //   cp0_xx_mrvbr              <- CSR, reset PC seed
    //   da_xx_fwd_*               <- LSU's DA stage, BJU-only early forward
    //   lsu_iu_ex2_*              <- LSU's EX2 stage completion forward
    //   rtu_iu_mul_wb_grant       <- RTU, MULT EX3 writeback-port grant
    //   rtu_iu_div_wb_grant       <- RTU, DIV writeback-port grant
    //=========================================================================

    //=========================================================================
    // SECTION ALU (IU note S2) -- purely combinational, EX1-only, no
    // pipeline register. One shared 65-bit adder covers ADD/SUB/ADDW/SUBW/
    // SLT/SLTU/LUI via an operand-prepare mux selecting the width/sign
    // class (not a separate 32-bit path, per alu.v:198-236's mechanism);
    // one shared barrel-shift expression covers SLL/SRL/SRA/*W plus XThead
    // SRRI/SRRIW (rotate) and EXT/EXTU (bitfield extract); a plain AND/OR/
    // XOR logic block; a misc block for XThead REV/REVW/TST/TSTNBZ/FF0/FF1/
    // MVEQZ/MVNEZ. No MAX/MIN/ADDSL (see header). Decode is exact-equality
    // against the pinned ALU_FUNC_* constants.
    //=========================================================================
    wire [63:0] alu_src0 = idu_iu_ex1_src0_data;
    wire [63:0] alu_src1 = idu_iu_ex1_src1_data;
    wire [63:0] alu_src2 = idu_iu_ex1_src2_data;

    wire alu_is_lui   = (idu_iu_ex1_func == ALU_FUNC_LUI);
    wire alu_is_add   = (idu_iu_ex1_func == ALU_FUNC_ADD);
    wire alu_is_addw  = (idu_iu_ex1_func == ALU_FUNC_ADDW);
    wire alu_is_sub   = (idu_iu_ex1_func == ALU_FUNC_SUB);
    wire alu_is_subw  = (idu_iu_ex1_func == ALU_FUNC_SUBW);
    wire alu_is_slt   = (idu_iu_ex1_func == ALU_FUNC_SLT);
    wire alu_is_sltu  = (idu_iu_ex1_func == ALU_FUNC_SLTU);
    wire alu_is_adder_op = alu_is_lui || alu_is_add || alu_is_addw || alu_is_sub
                        || alu_is_subw || alu_is_slt || alu_is_sltu;

    wire alu_is_sll   = (idu_iu_ex1_func == ALU_FUNC_SLL);
    wire alu_is_sllw  = (idu_iu_ex1_func == ALU_FUNC_SLLW);
    wire alu_is_srl   = (idu_iu_ex1_func == ALU_FUNC_SRL);
    wire alu_is_srlw  = (idu_iu_ex1_func == ALU_FUNC_SRLW);
    wire alu_is_sra   = (idu_iu_ex1_func == ALU_FUNC_SRA);
    wire alu_is_sraw  = (idu_iu_ex1_func == ALU_FUNC_SRAW);
    wire alu_is_srri  = (idu_iu_ex1_func == ALU_FUNC_SRRI);
    wire alu_is_srriw = (idu_iu_ex1_func == ALU_FUNC_SRRIW);
    wire alu_is_ext   = (idu_iu_ex1_func == ALU_FUNC_EXT);
    wire alu_is_extu  = (idu_iu_ex1_func == ALU_FUNC_EXTU);
    wire alu_is_shift_op = alu_is_sll || alu_is_sllw || alu_is_srl || alu_is_srlw
                        || alu_is_sra || alu_is_sraw || alu_is_srri
                        || alu_is_srriw || alu_is_ext || alu_is_extu;

    wire alu_is_and = (idu_iu_ex1_func == ALU_FUNC_AND);
    wire alu_is_or  = (idu_iu_ex1_func == ALU_FUNC_OR);
    wire alu_is_xor = (idu_iu_ex1_func == ALU_FUNC_XOR);
    wire alu_is_logic_op = alu_is_and || alu_is_or || alu_is_xor;

    wire alu_is_rev    = (idu_iu_ex1_func == ALU_FUNC_REV);
    wire alu_is_revw   = (idu_iu_ex1_func == ALU_FUNC_REVW);
    wire alu_is_tst    = (idu_iu_ex1_func == ALU_FUNC_TST);
    wire alu_is_tstnbz = (idu_iu_ex1_func == ALU_FUNC_TSTNBZ);
    wire alu_is_ff0    = (idu_iu_ex1_func == ALU_FUNC_FF0);
    wire alu_is_ff1    = (idu_iu_ex1_func == ALU_FUNC_FF1);
    wire alu_is_mveqz  = (idu_iu_ex1_func == ALU_FUNC_MVEQZ);
    wire alu_is_mvnez  = (idu_iu_ex1_func == ALU_FUNC_MVNEZ);
    wire alu_is_misc_op = alu_is_rev || alu_is_revw || alu_is_tst || alu_is_tstnbz
                       || alu_is_ff0 || alu_is_ff1 || alu_is_mveqz || alu_is_mvnez;

    // ---- adder block (one shared 65-bit adder) ----
    wire alu_adder_word    = alu_is_addw || alu_is_subw;
    wire alu_adder_cmp     = alu_is_slt  || alu_is_sltu;
    wire alu_adder_cmp_uns = alu_is_sltu;
    wire alu_adder_sub     = alu_is_sub || alu_is_subw || alu_adder_cmp;

    wire [63:0] alu_adder_op0 = alu_is_lui ? 64'b0 : alu_src0;
    wire [63:0] alu_adder_op1 = alu_src1;

    wire [64:0] alu_adder_a65 =
          alu_adder_word   ? {{33{alu_adder_op0[31]}}, alu_adder_op0[31:0]}
        : alu_adder_cmp_uns ? {1'b0, alu_adder_op0}
        :                     {alu_adder_op0[63], alu_adder_op0};
    wire [64:0] alu_adder_b65_raw =
          alu_adder_word   ? {{33{alu_adder_op1[31]}}, alu_adder_op1[31:0]}
        : alu_adder_cmp_uns ? {1'b0, alu_adder_op1}
        :                     {alu_adder_op1[63], alu_adder_op1};
    wire [64:0] alu_adder_b65 = alu_adder_sub ? ~alu_adder_b65_raw : alu_adder_b65_raw;
    wire [64:0] alu_adder_sum65 = alu_adder_a65 + alu_adder_b65 + {64'b0, alu_adder_sub};

    wire [63:0] alu_adder_add_result = alu_adder_word
        ? {{32{alu_adder_sum65[31]}}, alu_adder_sum65[31:0]}
        : alu_adder_sum65[63:0];
    wire [63:0] alu_adder_cmp_result = {63'b0, alu_adder_sum65[64]};
    wire [63:0] alu_adder_result = alu_adder_cmp ? alu_adder_cmp_result : alu_adder_add_result;

    // ---- shift block (one shared barrel-shift expression) ----
    wire alu_shift_word = alu_is_sllw || alu_is_srlw || alu_is_sraw || alu_is_srriw;
    wire [5:0] alu_shamt = alu_shift_word ? {1'b0, alu_src1[4:0]} : alu_src1[5:0];

    wire [63:0] alu_sll_res     = alu_src0 << alu_shamt;
    wire [63:0] alu_sllw_res64  = {32'b0, alu_src0[31:0]} << alu_shamt;
    wire [63:0] alu_srl_res     = alu_src0 >> alu_shamt;
    wire [63:0] alu_srlw_res64  = {32'b0, alu_src0[31:0]} >> alu_shamt;
    wire [63:0] alu_sra_res     = $signed(alu_src0) >>> alu_shamt;
    wire [31:0] alu_sraw_res32  = $signed(alu_src0[31:0]) >>> alu_shamt[4:0];
    wire [63:0] alu_srri_res    = (alu_shamt == 6'd0) ? alu_src0
                                 : (alu_src0 >> alu_shamt) | (alu_src0 << (7'd64 - alu_shamt));
    wire [31:0] alu_srriw_res32 = (alu_shamt[4:0] == 5'd0) ? alu_src0[31:0]
                                 : (alu_src0[31:0] >> alu_shamt[4:0])
                                 | (alu_src0[31:0] << (6'd32 - alu_shamt[4:0]));

    wire [5:0] alu_ext_lsb       = alu_src1[5:0];
    wire [5:0] alu_ext_msb       = alu_src1[11:6];
    wire [5:0] alu_ext_width_m1  = alu_ext_msb - alu_ext_lsb;
    wire [63:0] alu_ext_shifted  = alu_src0 >> alu_ext_lsb;
    wire [63:0] alu_ext_mask     = (alu_ext_width_m1 == 6'd63) ? {64{1'b1}}
                                  : ((64'd1 << (alu_ext_width_m1 + 6'd1)) - 64'd1);
    wire alu_ext_sign_bit        = alu_ext_shifted[alu_ext_width_m1];
    wire [63:0] alu_ext_res      = (alu_ext_shifted & alu_ext_mask)
                                  | ((alu_is_ext && alu_ext_sign_bit) ? ~alu_ext_mask : 64'b0);

    wire [63:0] alu_shift_result =
          alu_is_sll   ? alu_sll_res
        : alu_is_sllw  ? {{32{alu_sllw_res64[31]}}, alu_sllw_res64[31:0]}
        : alu_is_srl   ? alu_srl_res
        : alu_is_srlw  ? {{32{alu_srlw_res64[31]}}, alu_srlw_res64[31:0]}
        : alu_is_sra   ? alu_sra_res
        : alu_is_sraw  ? {{32{alu_sraw_res32[31]}}, alu_sraw_res32}
        : alu_is_srri  ? alu_srri_res
        : alu_is_srriw ? {{32{alu_srriw_res32[31]}}, alu_srriw_res32}
        : (alu_is_ext || alu_is_extu) ? alu_ext_res
        : 64'b0;

    // ---- logic block ----
    wire [63:0] alu_logic_result =
          alu_is_and ? (alu_src0 & alu_src1)
        : alu_is_or  ? (alu_src0 | alu_src1)
        : alu_is_xor ? (alu_src0 ^ alu_src1)
        : 64'b0;

    // ---- misc block (XThead) ----
    // Shared leading-one finder (distance from bit63 down to the highest
    // set bit; 64 if the input is all-zero) -- used here for FF0/FF1 and,
    // for timing only, by DIV's iteration-count derivation below.
    function automatic [6:0] lead1_dist_msb(input [63:0] v);
        integer i;
        reg found;
        begin
            found = 1'b0;
            lead1_dist_msb = 7'd64;
            for (i = 63; i >= 0; i = i - 1) begin
                if (!found && v[i]) begin
                    lead1_dist_msb = {1'b0, (6'd63 - i[5:0])};
                    found = 1'b1;
                end
            end
        end
    endfunction

    wire [63:0] alu_rev_res  = {alu_src0[7:0], alu_src0[15:8], alu_src0[23:16], alu_src0[31:24],
                                 alu_src0[39:32], alu_src0[47:40], alu_src0[55:48], alu_src0[63:56]};
    wire [31:0] alu_revw32   = {alu_src0[7:0], alu_src0[15:8], alu_src0[23:16], alu_src0[31:24]};
    wire [63:0] alu_revw_res = {{32{alu_revw32[31]}}, alu_revw32};
    wire [63:0] alu_tst_res  = {63'b0, alu_src0[alu_src1[5:0]]};
    wire [63:0] alu_tstnbz_res = {
        (alu_src0[63:56] == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[55:48] == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[47:40] == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[39:32] == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[31:24] == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[23:16] == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[15:8]  == 8'b0) ? 8'hff : 8'h00,
        (alu_src0[7:0]   == 8'b0) ? 8'hff : 8'h00};
    wire [63:0] alu_ff0_res = {57'b0, lead1_dist_msb(~alu_src0)};
    wire [63:0] alu_ff1_res = {57'b0, lead1_dist_msb(alu_src0)};

    // MVEQZ/MVNEZ: a 3-source conditional move (IU note S2's own reading of
    // alu.v:589-802) -- src1 is the condition register (compared to 0),
    // src0 the move-value, src2 the destination's OLD value (IDU reads it
    // as if it were a plain source register aliasing dst0, alu.v:800-802).
    wire alu_mv_cond_nonzero = |alu_src1;
    wire alu_mv_sel_src2 = (alu_is_mveqz && alu_mv_cond_nonzero)
                        || (alu_is_mvnez && !alu_mv_cond_nonzero);
    wire [63:0] alu_mv_res = alu_mv_sel_src2 ? alu_src2 : alu_src0;

    wire [63:0] alu_misc_result =
          alu_is_rev    ? alu_rev_res
        : alu_is_revw   ? alu_revw_res
        : alu_is_tst    ? alu_tst_res
        : alu_is_tstnbz ? alu_tstnbz_res
        : alu_is_ff0    ? alu_ff0_res
        : alu_is_ff1    ? alu_ff1_res
        : (alu_is_mveqz || alu_is_mvnez) ? alu_mv_res
        : 64'b0;

    // ---- result merge (one-hot OR-mux across the four sub-blocks) ----
    wire [63:0] alu_result = ({64{alu_is_adder_op}}  & alu_adder_result)
                            | ({64{alu_is_shift_op}}  & alu_shift_result)
                            | ({64{alu_is_logic_op}}  & alu_logic_result)
                            | ({64{alu_is_misc_op}}   & alu_misc_result);

    wire alu_active = idu_iu_ex1_alu_sel;

    assign iu_rtu_ex1_alu_cmplt      = alu_active;
    assign iu_rtu_ex1_alu_cmplt_dp   = alu_active;
    assign iu_rtu_ex1_alu_data       = alu_result;
    assign iu_rtu_ex1_alu_inst_len   = idu_iu_ex1_inst_len; // Task 7.3: real RVC-aware length (was fixed 1'b1)
    assign iu_rtu_ex1_alu_inst_split = 1'b0;   // no split-instruction classes reach ALU in M2
    assign iu_rtu_ex1_alu_preg       = idu_iu_ex1_dst0_reg;
    assign iu_rtu_ex1_alu_wb_dp      = alu_active;
    assign iu_rtu_ex1_alu_wb_vld     = alu_active;

    //=========================================================================
    // SECTION BJU (IU note S4) -- private comparator (not the ALU's
    // adder); a shared 64-bit address-gen adder (IU note S3, folded in here
    // rather than a separate module per Task 1's own S4.3 file-org note)
    // for branch/JAL/JALR target and AUIPC's pc+imm; the 1-entry LSU-
    // dependent conditional-branch buffer; BJU's own self-tracked PC copy.
    //=========================================================================
    wire bju_is_beq  = (idu_iu_ex1_func == BJU_FUNC_BEQ);
    wire bju_is_bne  = (idu_iu_ex1_func == BJU_FUNC_BNE);
    wire bju_is_blt  = (idu_iu_ex1_func == BJU_FUNC_BLT);
    wire bju_is_bge  = (idu_iu_ex1_func == BJU_FUNC_BGE);
    wire bju_is_bltu = (idu_iu_ex1_func == BJU_FUNC_BLTU);
    wire bju_is_bgeu = (idu_iu_ex1_func == BJU_FUNC_BGEU);
    wire bju_is_jal_live   = (idu_iu_ex1_func == BJU_FUNC_JAL);
    wire bju_is_jalr_live  = (idu_iu_ex1_func == BJU_FUNC_JALR);
    wire bju_is_auipc_live = (idu_iu_ex1_func == BJU_FUNC_AUIPC);
    wire bju_uncond_live   = bju_is_jal_live || bju_is_jalr_live;

    // -----------------------------------------------------------------
    // Dependency check + 1-entry buffer (IU note S4.4): only conditional
    // branches (idu_iu_ex1_bju_br_sel) ever park here; JAL/JALR/AUIPC never
    // depend on a not-yet-ready source in this ISA (their only "source" is
    // JALR's rs1, which IDU would not dispatch un-ready per its own
    // scoreboard for non-branch ops -- IU note S4.4's own finding).
    // -----------------------------------------------------------------
    reg                        bju_entry_vld_r, bju_entry_src0_vld_r, bju_entry_src1_vld_r;
    reg  [63:0]                bju_entry_src0_r, bju_entry_src1_r;
    reg  [FUNC_WIDTH-1:0]      bju_entry_func_r;
    reg  [1:0]                 bju_entry_bht_pred_r;
    reg  [PC_WIDTH-1:0]        bju_entry_target_r, bju_entry_inc_pc_r, bju_entry_not_pred_pc_r;
    reg  [GPR_IDX_WIDTH-1:0]   bju_entry_src0_reg_r, bju_entry_src1_reg_r;
    // Task 7.3: the length of the branch PARKED in the entry (donor
    // aq_iu_bju.v:208 bju_inst_len_flop). Latched at entry creation; selects
    // the parked-entry length over the live one when the entry is resolving
    // (aq_iu_bju.v:806).
    reg                        bju_inst_len_flop;

    wire bju_src0_missing = idu_iu_ex1_bju_br_sel && !idu_iu_ex1_src0_ready;
    wire bju_src1_missing = idu_iu_ex1_bju_br_sel && !idu_iu_ex1_src1_ready;

    // Fast/early forward (da_xx_fwd_*) can resolve the dependency the same
    // cycle it is discovered, avoiding an entry create altogether.
    wire bju_da_fwd_src0_hit_now = bju_src0_missing && da_xx_fwd_vld
                                 && (da_xx_fwd_dst_reg == idu_iu_ex1_src0_reg);
    wire bju_da_fwd_src1_hit_now = bju_src1_missing && da_xx_fwd_vld
                                 && (da_xx_fwd_dst_reg == idu_iu_ex1_src1_reg);

    wire bju_depend_lsu_src0 = bju_src0_missing && !bju_da_fwd_src0_hit_now;
    wire bju_depend_lsu_src1 = bju_src1_missing && !bju_da_fwd_src1_hit_now;
    wire bju_depend_lsu      = bju_depend_lsu_src0 || bju_depend_lsu_src1;

    wire bju_create_entry = idu_iu_ex1_bju_sel && bju_depend_lsu && !bju_entry_vld_r;

    // Register number the entry (or the live dispatch) is waiting on, for
    // matching against an incoming forward's destination register.
    wire [GPR_IDX_WIDTH-1:0] bju_fwd_src0_reg_sel =
        bju_entry_vld_r ? bju_entry_src0_reg_r : idu_iu_ex1_src0_reg;
    wire [GPR_IDX_WIDTH-1:0] bju_fwd_src1_reg_sel =
        bju_entry_vld_r ? bju_entry_src1_reg_r : idu_iu_ex1_src1_reg;

    wire bju_da_hit0  = da_xx_fwd_vld       && (da_xx_fwd_dst_reg      == bju_fwd_src0_reg_sel);
    wire bju_da_hit1  = da_xx_fwd_vld       && (da_xx_fwd_dst_reg      == bju_fwd_src1_reg_sel);
    wire bju_lsu_hit0 = lsu_iu_ex2_data_vld && (lsu_iu_ex2_dest_reg    == bju_fwd_src0_reg_sel);
    wire bju_lsu_hit1 = lsu_iu_ex2_data_vld && (lsu_iu_ex2_dest_reg    == bju_fwd_src1_reg_sel);

    wire bju_entry_pop = bju_entry_vld_r && bju_entry_src0_vld_r && bju_entry_src1_vld_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bju_entry_vld_r <= 1'b0;
        else if (bju_create_entry)
            bju_entry_vld_r <= 1'b1;
        else if (bju_entry_pop)
            bju_entry_vld_r <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bju_entry_src0_vld_r <= 1'b0;
        else if (bju_entry_pop)
            bju_entry_src0_vld_r <= 1'b0;
        else if (bju_create_entry)
            bju_entry_src0_vld_r <= !bju_depend_lsu_src0;
        else if (bju_entry_vld_r && (bju_da_hit0 || bju_lsu_hit0))
            bju_entry_src0_vld_r <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bju_entry_src1_vld_r <= 1'b0;
        else if (bju_entry_pop)
            bju_entry_src1_vld_r <= 1'b0;
        else if (bju_create_entry)
            bju_entry_src1_vld_r <= !bju_depend_lsu_src1;
        else if (bju_entry_vld_r && (bju_da_hit1 || bju_lsu_hit1))
            bju_entry_src1_vld_r <= 1'b1;
    end

    // -----------------------------------------------------------------
    // Address-gen (IU note S3): one shared 64-bit adder for branch/JAL/
    // JALR target and AUIPC's pc+imm, computed at DISPATCH time (the
    // target is PC-relative-or-register-relative, independent of whether
    // a conditional branch's comparison operands are ready yet) --
    // matches the donor's own AG independence from the entry-pending path.
    // -----------------------------------------------------------------
    wire [PC_WIDTH-1:0] bju_pc_now     = bju_pcgen_pc;
    // Task 7.3: RVC-aware PC increments (donor aq_iu_bju.v:576-583). The old
    // +4-only increment assumed every instruction is 32-bit; with RVC in
    // scope (the M2 bring-up ladder's RVC-mix stream) a 16-bit instruction
    // must advance the PC by 2, not 4. Two variants, mirroring the donor's
    // separate `bju_inc_pc_wb_data` (live) and `bju_inc_pc_ext` (rtu):
    //   - bju_inc_pc_live : the DISPATCHED instruction's length (live), for
    //     the JAL writeback link + the not-predicted-PC / entry-inc latch.
    //   - bju_inc_pc_rt   : the COMPLETING instruction's length (RTU
    //     feedback rtu_iu_ex1_inst_len), for the PC generator's advance.
    // Addends sized to PC_WIDTH (avoid WIDTHEXPAND); 16-bit -> +2, 32-bit -> +4.
    localparam [PC_WIDTH-1:0] PC_INC16 = {{(PC_WIDTH-3){1'b0}}, 3'd2};
    localparam [PC_WIDTH-1:0] PC_INC32 = {{(PC_WIDTH-3){1'b0}}, 3'd4};
    wire [PC_WIDTH-1:0] bju_inc_pc_live = bju_pc_now + (idu_iu_ex1_inst_len ? PC_INC32 : PC_INC16);
    wire [PC_WIDTH-1:0] bju_inc_pc_rt   = bju_pc_now + (rtu_iu_ex1_inst_len  ? PC_INC32 : PC_INC16);

    // Sign-extend (not zero-extend) bju_pc_now: this adder also produces the
    // full 64b AUIPC writeback (bju_wb_data = ag_result_live below), so a
    // zero-extended base corrupts AUIPC's pc+imm result whenever the AUIPC
    // itself sits in Sv39 kernel space (PC bit PC_WIDTH-1 set) -- the same
    // class of bug as the iu_ifu_tar_pc fix above. The JAL/branch target use
    // of this adder (bju_target_now) truncates back to PC_WIDTH bits, so it
    // was unaffected either way.
    wire [63:0] ag_rs1_live = bju_is_jalr_live ? idu_iu_ex1_src0_data
                                                : {{(64-PC_WIDTH){bju_pc_now[PC_WIDTH-1]}}, bju_pc_now};
    wire [63:0] ag_rs2_live = idu_iu_ex1_src2_data;
    wire [63:0] ag_result_live = ag_rs1_live + ag_rs2_live;
    // JALR's target (rs1+imm) can have an odd LSB (imm[0] is a real encoded
    // bit); the spec requires clearing it before use. Donor aq_iu_bju.v:810
    // applies the identical clear on its final next-pc bus
    // (`{bju_next_pc_update[38:0],1'b0}`); rv906 has no separate raw-target
    // consumer (no RAS-vs-JALR compare wired, see S4.2/S4.3 note below), so
    // clearing here at the single target source is equivalent and covers
    // every downstream user (bju_target_pc/bju_next_pc AND bju_not_pred_pc).
    // JAL/branch targets are already even, so the unconditional clear is a
    // no-op for them.
    wire [PC_WIDTH-1:0] bju_target_now = {ag_result_live[PC_WIDTH-1:1], 1'b0};
    wire [PC_WIDTH-1:0] bju_not_pred_pc_now =
        idu_iu_ex1_bht_pred[1] ? bju_inc_pc_live : bju_target_now;

    // Entry data capture (create: latch fresh values; otherwise: absorb a
    // matching forward into whichever slot(s) it satisfies).
    always @(posedge clk) begin
        if (bju_create_entry) begin
            bju_entry_src0_r        <= idu_iu_ex1_src0_data;
            bju_entry_src1_r        <= idu_iu_ex1_src1_data;
            bju_entry_func_r        <= idu_iu_ex1_func;
            bju_entry_bht_pred_r    <= idu_iu_ex1_bht_pred;
            bju_entry_target_r      <= bju_target_now;
            bju_entry_inc_pc_r      <= bju_inc_pc_live;
            bju_entry_not_pred_pc_r <= bju_not_pred_pc_now;
            bju_entry_src0_reg_r    <= idu_iu_ex1_src0_reg;
            bju_entry_src1_reg_r    <= idu_iu_ex1_src1_reg;
            bju_inst_len_flop       <= idu_iu_ex1_inst_len;
        end
        else if (bju_entry_vld_r) begin
            if (bju_da_hit0)
                bju_entry_src0_r <= da_xx_fwd_data;
            else if (bju_lsu_hit0)
                bju_entry_src0_r <= lsu_iu_ex2_data;
            if (bju_da_hit1)
                bju_entry_src1_r <= da_xx_fwd_data;
            else if (bju_lsu_hit1)
                bju_entry_src1_r <= lsu_iu_ex2_data;
        end
    end

    // -----------------------------------------------------------------
    // Entry-or-live select, then the private comparator (IU note S4.1 --
    // its own comparator, never the ALU's adder).
    //
    // Donor aq_iu_bju.v:430-446: the LIVE-path operand is NOT the raw
    // dispatch operand when an LSU forward hits this very cycle --
    // bju_src0_raw = bju_lsu_wb_fwd_src0_vld ? da_xx_fwd_data : src0_tmp.
    // Without this mux a branch dispatched the cycle its source load
    // delivers data skips the park (bju_da_fwd_srcN_hit_now clears
    // depend_lsu) but compares the STALE register-file value
    // (rv64uc-p-rvc test-20 check C: bne resolved on the load's OLD
    // destination value, one cycle before writeback).
    // -----------------------------------------------------------------
    wire [63:0] bju_live_src0 = bju_da_fwd_src0_hit_now ? da_xx_fwd_data
                                                          : idu_iu_ex1_src0_data;
    wire [63:0] bju_live_src1 = bju_da_fwd_src1_hit_now ? da_xx_fwd_data
                                                          : idu_iu_ex1_src1_data;
    wire [FUNC_WIDTH-1:0] bju_func_sel = bju_entry_vld_r ? bju_entry_func_r : idu_iu_ex1_func;
    wire [63:0] bju_cmp_src0 = bju_entry_vld_r ? bju_entry_src0_r : bju_live_src0;
    wire [63:0] bju_cmp_src1 = bju_entry_vld_r ? bju_entry_src1_r : bju_live_src1;
    wire [1:0]  bju_bht_pred_sel = bju_entry_vld_r ? bju_entry_bht_pred_r : idu_iu_ex1_bht_pred;
    wire [PC_WIDTH-1:0] bju_target_pc  = bju_entry_vld_r ? bju_entry_target_r      : bju_target_now;
    wire [PC_WIDTH-1:0] bju_inc_pc     = bju_entry_vld_r ? bju_entry_inc_pc_r      : bju_inc_pc_rt;
    wire [PC_WIDTH-1:0] bju_not_pred_pc = bju_entry_vld_r ? bju_entry_not_pred_pc_r : bju_not_pred_pc_now;

    wire bju_cmp_is_beq  = (bju_func_sel == BJU_FUNC_BEQ);
    wire bju_cmp_is_bne  = (bju_func_sel == BJU_FUNC_BNE);
    wire bju_cmp_is_blt  = (bju_func_sel == BJU_FUNC_BLT);
    wire bju_cmp_is_bge  = (bju_func_sel == BJU_FUNC_BGE);
    wire bju_cmp_is_bltu = (bju_func_sel == BJU_FUNC_BLTU);
    wire bju_cmp_is_bgeu = (bju_func_sel == BJU_FUNC_BGEU);
    wire bju_is_cond_br_sel = bju_cmp_is_beq || bju_cmp_is_bne || bju_cmp_is_blt
                            || bju_cmp_is_bge || bju_cmp_is_bltu || bju_cmp_is_bgeu;

    wire bju_beq_taken = (bju_cmp_src0 == bju_cmp_src1);
    wire bju_ult        = (bju_cmp_src0 <  bju_cmp_src1);
    wire bju_slt         = ($signed(bju_cmp_src0) < $signed(bju_cmp_src1));

    wire bju_cond_br_taken_raw = (bju_cmp_is_beq  &&  bju_beq_taken)
                              || (bju_cmp_is_bne  && !bju_beq_taken)
                              || (bju_cmp_is_blt  &&  bju_slt)
                              || (bju_cmp_is_bge  && !bju_slt)
                              || (bju_cmp_is_bltu &&  bju_ult)
                              || (bju_cmp_is_bgeu && !bju_ult);

    // Unconditional ops (JAL/JALR) never park in the entry, so gating on
    // "not currently held in the entry" is enough to select them here.
    wire bju_uncond_sel = !bju_entry_vld_r && bju_uncond_live;
    wire bju_taken       = bju_uncond_sel || bju_cond_br_taken_raw;

    wire [PC_WIDTH-1:0] bju_next_pc = bju_taken ? bju_target_pc : bju_inc_pc;

    // -----------------------------------------------------------------
    // BHT mispredict + RAS-adjacent checks (IU note S4.2/S4.3). No
    // JALR-vs-RAS-target compare exists on this port list (see header) --
    // `iu_rtu_ex2_bju_ras_mispred` stays tied 0.
    // -----------------------------------------------------------------
    wire bju_cond_br_mispred = bju_is_cond_br_sel && (bju_cond_br_taken_raw ^ bju_bht_pred_sel[1]);
    wire bju_pc_reg_mispred  = bju_is_jalr_live && !bju_entry_vld_r
                            && (idu_iu_ex1_src0_reg[4:0] != 5'd1);

    // Donor aq_iu_bju.v:637-638,687-707 (`bju_bht_mispred_entry` /
    // `bju_not_ex1_chgflw` / `bju_not_ex1_tar_pc`): a parked-entry
    // conditional-branch mispredict must self-correct bju_pcgen_pc THE SAME
    // cycle the entry pops, ahead of (bypassing) the rtu_iu_ex1_cmplt-gated
    // advance below. Without this the entry clears next cycle, an unrelated
    // instruction occupies ex1_eu_r/ex1_func_r, and bju_pcgen_next_pc
    // silently falls through by +len from the branch's own PC instead of
    // applying its target -- the tracker permanently drifts onto the
    // fall-through path (found via BJUDBG/JMPDBG tracing on
    // rv64ui-v-simple.elf: bju_pcgen_pc advanced ...147c -> ...1480, +4
    // fall-through, across the mispredicted-taken branch's own pop cycle,
    // silently discarding tar_pc=...1558; the stale tracker then spuriously
    // re-fires mispredict logic at the unrelated PC ...1480, corrupting
    // next_pc for every later retiring instruction). The donor's RAS-mispred
    // OR-term (`bju_ras_mispred_vld`) is dropped: rv906 does not implement
    // RAS mispredict tracking (see the header's RAS note; `bju_ras_mispred_
    // vld`-equivalent stays absent), so `bju_not_ex1_chgflw` reduces to just
    // the BHT/entry term. Target is rv906's existing `bju_not_pred_pc` mux
    // (line below): already entry-selected, so on the pop cycle
    // (bju_entry_vld_r still 1) it reads the entry's captured
    // bju_entry_not_pred_pc_r -- exactly the donor's bju_not_pred_pc_flop.
    wire bju_bht_mispred_entry = bju_cond_br_mispred && bju_entry_pop;
    wire bju_not_ex1_chgflw    = bju_bht_mispred_entry;

    // "Resolves now" = either the entry just got both operands, or this is
    // a live (non-parking) dispatch resolving in the same EX1 cycle.
    wire bju_resolves_now = bju_entry_pop
                          || (idu_iu_ex1_bju_sel && !bju_entry_vld_r && !bju_create_entry);

    // PC-gen next pc (donor aq_iu_bju.v:572-581,624-625): the pcgen tracks the
    // EX1 register, so its non-taken fall-through is always the LIVE
    // `bju_pcgen_pc + len` (bju_inc_pc_ext = bju_cur_pc_ext + len, cur_pc =
    // bju_pcgen_pc), NOT the latched entry inc-pc. While a cond-branch is
    // parked in the entry the pcgen self-increments off the live pc to stay
    // locked to EX1; only a resolving+taken branch sends it to the target.
    wire [PC_WIDTH-1:0] bju_pcgen_next_pc =
        (bju_resolves_now && bju_taken) ? bju_target_pc : bju_inc_pc_rt;

    // rv906 M2 bring-up fix (rv64uc-p-rvc wrong-path pcgen drift): the donor
    // bju_tar_pc_vld formula (aq_iu_bju.v:649-653) fires only on BHT/RAS
    // mispredict -- for unconditional jumps the donor relies on the BTB /
    // IPACK-stage predictor redirect (aq_ifu_pred.v:725-733 pred_chgflw) to
    // have already re-pointed the front end, so no wrong-path instruction
    // ever completes behind a jump. With the predictors disabled (the M2
    // rung-0/1 pass bar, MHCR.BTB/BHT/RAS = 0) there is no such early
    // redirect for the IU's own pcgen tracker: the fall-through instruction
    // after the jump enters EX1, completes, and advances bju_pcgen_pc by its
    // length -- a PERMANENT drift (proven by $display tracing: c.j @0x80000000
    // correctly loaded the tracker with target 0x80000048, then the
    // wrong-path nop @0x80000002 completed next cycle and moved it to
    // 0x8000004a), corrupting every later PC-relative consumer (auipc
    // results, branch targets). Redirecting the unconditional resolve closes
    // this: iu_idu_br_cancel squashes the wrong-path dispatch at the EX1
    // boundary (IDU flush term beats the adv load), pcgen_ibuf_chgflw_vld
    // flushes the fetched fall-through halfwords, and the tracker keeps the
    // true target from the resolving jump's own bju_pcgen_next_pc update.
    // When BTB/BHT ARE enabled this duplicates a redirect the front end has
    // already taken -- a same-target re-flush, harmless (performance only).
    wire bju_uncond_redirect = bju_resolves_now && bju_uncond_live && !bju_entry_vld_r;

    wire bju_redirect_now = bju_resolves_now && (bju_cond_br_mispred || bju_pc_reg_mispred)
                          || bju_uncond_redirect;

    wire bju_ret_vld_raw  = bju_is_jalr_live && !bju_entry_vld_r
                          && (idu_iu_ex1_src0_reg[4:0] == 5'd1)
                          && (idu_iu_ex1_src0_reg != idu_iu_ex1_dst0_reg);
    wire bju_link_vld_raw = bju_uncond_live && !bju_entry_vld_r
                          && (idu_iu_ex1_dst0_reg[4:0] == 5'd1);

    assign iu_ifu_tar_pc_vld  = bju_redirect_now;
    assign iu_idu_br_cancel   = bju_redirect_now;   // donor aq_iu_bju.v:783 (iu_yy_xx_cancel = iu_ifu_tar_pc_vld)
    // Sign-extend (not zero-extend) to 64b -- matches every other PC-widening
    // site (IFU.v pcgen_ifpc: lines ~319,363,369, `{{(64-PC_WIDTH){pc[PC_WIDTH-1]}},pc}`).
    // Zero-extension here corrupted any Sv39 kernel-half (VA bit 38 set) BJU
    // redirect target into a non-canonical VA, tripping MMU.v's va_illegal
    // check and forcing a permanent vec-12 fetch-page-fault loop (found via
    // IFUDBG tracing: cnt=124 va ffffffffffe01 -> 000000ffffe01 on this path).
    assign iu_ifu_tar_pc      = {{(64-PC_WIDTH){bju_next_pc[PC_WIDTH-1]}}, bju_next_pc};
    assign iu_ifu_pc_mispred  = bju_resolves_now && bju_pc_reg_mispred;
    assign iu_ifu_bht_mispred = bju_cond_br_mispred;
    assign iu_ifu_br_vld      = bju_resolves_now && bju_is_cond_br_sel;
    assign iu_ifu_bht_taken   = bju_taken;
    assign iu_ifu_bht_pred    = bju_bht_pred_sel;
    assign iu_ifu_link_vld    = bju_resolves_now && bju_link_vld_raw;
    assign iu_ifu_ret_vld     = bju_resolves_now && bju_ret_vld_raw;

    // -----------------------------------------------------------------
    // BJU's own PC copy (IU note S4.5, restore-checklist item 4).
    //
    // TASK 7.3 -- the Task 3 stand-in is RETIRED. The old advance condition
    // (`bju_advances_pc = idu_iu_ex1_inst_vld || bju_entry_pop`, a dispatch-
    // side proxy) advanced the tracker every cycle EX1 held a valid
    // instruction, and the increment was a fixed +4 -- together they drove
    // the self-tracked PC ~91 instructions PAST the true retire PC in
    // RVC-mixed code, where it self-pinned (the sentinel `jal x0` recomputes
    // its target from the already-wrong PC). This is exactly the failure the
    // header's "re-verify this exact assumption once RTU's real retire
    // timing exists" note predicted.
    //
    // Donor-faithful mechanism (aq_iu_bju.v:680-700, the "PC Generator"
    // always block): a priority mux that advances the tracker ONLY on a
    // genuine RTU-confirmed EX1 completion of the completing instruction's
    // length (`rtu_iu_ex1_cmplt && !rtu_iu_ex1_inst_split`), redirecting on
    // the RTU changeflow (`ifu_iu_chgflw_vld`, highest non-reset priority)
    // and seeding from `cp0_xx_mrvbr` at reset. The advance VALUE is
    // `bju_next_pc` (line ~705), whose non-taken term is now the RVC-aware
    // `bju_inc_pc_rt` (aq_iu_bju.v:576-583,581).
    //
    // RE-VERIFIED (Task 9 mispredict bring-up, rv64ui-v-simple.elf): the
    // prior version of this comment claimed a mispredict's redirect "still
    // reaches this register through the higher-priority changeflow branch,"
    // i.e. that `ifu_iu_chgflw_vld` alone was sufficient. Empirically false
    // for a parked-entry resolve: BJUDBG/JMPDBG tracing caught the exact
    // predicted failure (a parked cond-branch resolves mispredicted-taken
    // with tar_pc=...1558, but `rtu_iu_ex1_cmplt` is 0 that cycle, so the
    // normal advance below misses the window; ~20 cycles later the tracker
    // is found to have fallen through +4 to ...1480 instead, then spuriously
    // re-triggers stale mispredict logic at that unrelated PC). The donor's
    // 3rd branch (`bju_not_ex1_chgflw`, aq_iu_bju.v:687-707) IS required and
    // is now ported below as `bju_not_ex1_chgflw`/`bju_bht_mispred_entry`
    // (defined above, near `bju_cond_br_mispred`) -- an immediate, same-
    // cycle self-correction of this tracker on a parked-entry BHT
    // mispredict, independent of `rtu_iu_ex1_cmplt` timing. The donor's
    // RAS-mispred OR-term stays dropped (RAS mispredict tracking not
    // implemented, per the header's RAS note).
    // -----------------------------------------------------------------
    reg [PC_WIDTH-1:0] bju_pcgen_pc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bju_pcgen_pc <= cp0_xx_mrvbr;
        else if (ifu_iu_chgflw_vld)
            bju_pcgen_pc <= ifu_iu_chgflw_pc;
        else if (bju_not_ex1_chgflw)
            bju_pcgen_pc <= bju_not_pred_pc;
        else if (rtu_iu_ex1_cmplt && !rtu_iu_ex1_inst_split)
            bju_pcgen_pc <= bju_pcgen_next_pc;
    end

    // -----------------------------------------------------------------
    // Backpressure to IDU (contract 8, bju.v:477-478).
    // -----------------------------------------------------------------
    assign iu_idu_bju_full        = bju_entry_vld_r &&  bju_entry_src0_vld_r && bju_entry_src1_vld_r;
    assign iu_idu_bju_global_full = bju_entry_vld_r && !(bju_entry_src0_vld_r && bju_entry_src1_vld_r);

    // -----------------------------------------------------------------
    // RTU writeback bus (IU note S10). Only JAL/JALR (link address =
    // pc+4) and AUIPC (pc+imm, riding this bus per IU note S2/S3) write a
    // register; conditional branches never do.
    // -----------------------------------------------------------------
    wire bju_writes_reg = (bju_uncond_sel || (bju_is_auipc_live && !bju_entry_vld_r))
                        && idu_iu_ex1_bju_sel;
    // Sign-extend the JAL/JALR link address (pc+len written to rd): same
    // zero-extension defect as ag_rs1_live/iu_ifu_tar_pc above -- a call
    // (jal/jalr) executed from kernel-space code (PC bit PC_WIDTH-1 set)
    // must produce a canonical return address for the later `ret` to work.
    wire [63:0] bju_wb_data = bju_uncond_sel
                            ? {{(64-PC_WIDTH){bju_inc_pc_live[PC_WIDTH-1]}}, bju_inc_pc_live}
                            : ag_result_live;

    assign iu_rtu_ex1_bju_cmplt            = bju_resolves_now;
    assign iu_rtu_ex1_bju_cmplt_dp         = bju_resolves_now;
    // for-pcgen completion (LSU aq_lsu_ag.v:1675 / RTU aq_rtu_ctrl.v:139
    // pattern, applied to the bju): the pcgen must advance in lockstep with
    // the EX1 register. A cond-branch that PARKS in the entry leaves EX1 the
    // same cycle it is created (ex1 advances to the next inst) but does not
    // retire until it pops, so its retire cmplt (bju_resolves_now) is low at
    // creation and would leave the pcgen frozen one inst behind -- desyncing
    // the auipc that reads bju_pcgen_pc (rv64ui-p-lb/sd). Fire the pcgen
    // advance whenever the bju occupies EX1 and is not yet parked
    // (resolving-in-EX1 OR creating-the-entry); a popped parked entry does not
    // re-advance here (the mispredict redirect moves the pcgen instead).
    assign iu_rtu_ex1_bju_cmplt_for_pcgen  = idu_iu_ex1_bju_sel && !bju_entry_vld_r;
    assign iu_rtu_ex1_bju_data             = bju_wb_data;
    assign iu_rtu_ex1_bju_inst_len         = bju_entry_vld_r ? bju_inst_len_flop : idu_iu_ex1_inst_len; // Task 7.3 (donor aq_iu_bju.v:806)
    assign iu_rtu_ex1_bju_preg             = idu_iu_ex1_dst0_reg;
    assign iu_rtu_ex1_bju_wb_dp            = bju_writes_reg;
    assign iu_rtu_ex1_bju_wb_vld           = bju_writes_reg;
    wire bju_is_cond_br_live = bju_is_beq || bju_is_bne || bju_is_blt
                             || bju_is_bge || bju_is_bltu || bju_is_bgeu;
    assign iu_rtu_ex1_branch_inst          = (idu_iu_ex1_bju_sel
                                               && (bju_is_cond_br_live || bju_uncond_live))
                                              || bju_entry_vld_r;
    assign iu_rtu_ex1_cur_pc               = bju_pcgen_pc;
    assign iu_rtu_ex1_next_pc              = bju_next_pc;
    assign iu_rtu_ex2_bju_ras_mispred      = 1'b0;   // no RAS-target-compare input, see header
    assign iu_rtu_depd_lsu_chgflow_vld     = bju_entry_pop && bju_cond_br_mispred;
    assign iu_rtu_depd_lsu_chgflow_next_pc = bju_not_pred_pc;

    //=========================================================================
    // SECTION MULT (IU note S5) -- one 33x33 Booth-radix-4 array reused
    // iteratively in the donor; here the correct VALUE is computed with
    // ordinary 65x65 signed arithmetic (see header's clean-room note),
    // while the FSM states (IDLE/SPLIT0/SPLIT1/CMPLT), the narrow-vs-wide
    // early-out judgement, the EX1-early-accept/EX3-writeback split, and
    // the RTU-grant-gated EX3 stall are all faithfully modeled -- these
    // are what Task 9's bring-up ladder and RTU's rbus arbiter depend on.
    //=========================================================================
    localparam MUL_IDLE = 2'b00, MUL_SPLIT0 = 2'b01, MUL_SPLIT1 = 2'b10, MUL_CMPLT = 2'b11;
    reg [1:0] mul_state;

    wire mul_is_mul    = (idu_iu_ex1_func == MULT_FUNC_MUL);
    wire mul_is_mulw   = (idu_iu_ex1_func == MULT_FUNC_MULW);
    // MULH itself needs no dedicated wire: it is neither MUL nor MULW, so
    // it falls to the "high word" default in mul_result below, same as
    // MULHU/MULHSU -- they differ only in operand signedness (next).
    wire mul_is_mulhu  = (idu_iu_ex1_func == MULT_FUNC_MULHU);
    wire mul_is_mulhsu = (idu_iu_ex1_func == MULT_FUNC_MULHSU);

    // Signedness per operand, per op (MUL/MULW's low-word result is
    // sign-independent -- a two's-complement identity -- so their class
    // choice below only affects the narrow-vs-wide judgement, not
    // correctness of the low-word result itself).
    wire mul_src0_signed = !mul_is_mulhu;
    wire mul_src1_signed = !(mul_is_mulhu || mul_is_mulhsu);

    wire [63:0] mul_src0 = idu_iu_ex1_src0_data;
    wire [63:0] mul_src1 = idu_iu_ex1_src1_data;

    wire mul_src0_fits33 = (mul_src0[63:32] == 32'b0)
                         || (mul_src0[63:32] == {32{1'b1}} && mul_src0_signed);
    wire mul_src1_fits33 = (mul_src1[63:32] == 32'b0)
                         || (mul_src1[63:32] == {32{1'b1}} && mul_src1_signed);
    wire mul_needs_split = !mul_is_mulw && !(mul_src0_fits33 && mul_src1_fits33);

    wire signed [64:0] mul_op0_65 = {mul_src0_signed & mul_src0[63], mul_src0};
    wire signed [64:0] mul_op1_65 = {mul_src1_signed & mul_src1[63], mul_src1};
    wire signed [129:0] mul_product130 = mul_op0_65 * mul_op1_65;
    wire [127:0] mul_full_product = mul_product130[127:0];

    wire [63:0] mul_low  = mul_full_product[63:0];
    wire [63:0] mul_high = mul_full_product[127:64];
    wire [63:0] mul_mulw_result = {{32{mul_low[31]}}, mul_low[31:0]};
    wire [63:0] mul_result = mul_is_mulw ? mul_mulw_result
                            : mul_is_mul ? mul_low
                            :              mul_high;

    wire mul_iter_start = idu_iu_ex1_mult_sel && mul_needs_split;
    wire mul_is_final_pass = (mul_state == MUL_IDLE && !mul_needs_split) || (mul_state == MUL_CMPLT);
    wire mul_ex1_valid_in   = idu_iu_ex1_mult_sel && mul_is_final_pass;

    reg          mul_ex2_valid;
    reg  [63:0]  mul_ex2_data;
    reg  [GPR_IDX_WIDTH-1:0] mul_ex2_preg;
    reg          mul_ex3_valid;
    reg  [63:0]  mul_ex3_data;
    reg  [GPR_IDX_WIDTH-1:0] mul_ex3_preg;

    wire mul_ex3_stall = mul_ex3_valid && !rtu_iu_mul_wb_grant;
    wire mul_ex2_stall = mul_ex3_stall && mul_ex2_valid;

    wire [1:0] mul_next_state;
    assign mul_next_state = (mul_state == MUL_IDLE)   ? (mul_iter_start ? MUL_SPLIT0 : MUL_IDLE)
                           : (mul_state == MUL_SPLIT0) ? (mul_ex2_stall ? MUL_SPLIT0 : MUL_SPLIT1)
                           : (mul_state == MUL_SPLIT1) ? (mul_ex2_stall ? MUL_SPLIT1 : MUL_CMPLT)
                           :                              MUL_IDLE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mul_state <= MUL_IDLE;
        else
            mul_state <= mul_next_state;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_ex2_valid <= 1'b0;
        end
        else if (!mul_ex2_stall) begin
            mul_ex2_valid <= mul_ex1_valid_in;
            if (mul_ex1_valid_in) begin
                mul_ex2_data <= mul_result;
                mul_ex2_preg <= idu_iu_ex1_dst0_reg;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_ex3_valid <= 1'b0;
        end
        else if (!mul_ex3_stall) begin
            mul_ex3_valid <= mul_ex2_valid;
            if (mul_ex2_valid) begin
                mul_ex3_data <= mul_ex2_data;
                mul_ex3_preg <= mul_ex2_preg;
            end
        end
    end

    assign iu_rtu_ex1_mul_cmplt    = idu_iu_ex1_mult_sel && mul_is_final_pass;
    assign iu_rtu_ex1_mul_cmplt_dp = idu_iu_ex1_mult_sel && mul_is_final_pass;
    // Task 7.3: completing-MULT length. Tied 1 (32-bit) -- M2's RVC decoder
    // has no 16-bit MULT (see the port-list note).
    assign iu_rtu_ex1_mul_inst_len = 1'b1;
    assign iu_rtu_ex3_mul_wb_vld   = mul_ex3_valid;
    assign iu_rtu_ex3_mul_data     = mul_ex3_data;
    assign iu_rtu_ex3_mul_preg     = mul_ex3_preg;

    assign iu_idu_mult_issue_stall = idu_iu_ex1_mult_sel
                                    && ((mul_state == MUL_IDLE && mul_iter_start)
                                        || mul_state == MUL_SPLIT0 || mul_state == MUL_SPLIT1);
    assign iu_idu_mult_full        = mul_ex2_valid && mul_ex3_valid && !rtu_iu_mul_wb_grant;

    //=========================================================================
    // SECTION DIV (IU note S6) -- radix-4, 2-bits/cycle non-restoring
    // divider in the donor; here the correct VALUE is computed with
    // ordinary Verilog division/modulo (clean-room, see header), while the
    // FSM states (IDLE/WFI2/ALIGN/ITER/CMPLT/WFWB), the abnormal-result and
    // 1-entry memo-buffer early-outs, and the DATA-DEPENDENT iteration
    // count (from the same leading-1-difference the donor's kernel uses,
    // giving the same ~4-36 cycle variable latency) are faithfully
    // modeled. DIV/DIVU/REM/REMU (+*W) share one core; `div_res_sel_
    // quotient` just picks which result rides the bus (IU note S6).
    //=========================================================================
    localparam DIV_IDLE = 3'b000, DIV_WFI2 = 3'b001, DIV_ALIGN = 3'b010,
               DIV_ITER  = 3'b011, DIV_CMPLT = 3'b100, DIV_WFWB = 3'b101;
    reg [2:0] div_state;

    wire div_is_word   = (idu_iu_ex1_func == DIV_FUNC_DIVW)  || (idu_iu_ex1_func == DIV_FUNC_DIVUW)
                       || (idu_iu_ex1_func == DIV_FUNC_REMW) || (idu_iu_ex1_func == DIV_FUNC_REMUW);
    wire div_is_signed = (idu_iu_ex1_func == DIV_FUNC_DIV) || (idu_iu_ex1_func == DIV_FUNC_DIVW)
                       || (idu_iu_ex1_func == DIV_FUNC_REM) || (idu_iu_ex1_func == DIV_FUNC_REMW);
    wire div_sel_quotient = (idu_iu_ex1_func == DIV_FUNC_DIV) || (idu_iu_ex1_func == DIV_FUNC_DIVW)
                          || (idu_iu_ex1_func == DIV_FUNC_DIVU) || (idu_iu_ex1_func == DIV_FUNC_DIVUW);

    wire [63:0] div_src0_raw = idu_iu_ex1_src0_data;
    wire [63:0] div_src1_raw = idu_iu_ex1_src1_data;
    // operand prepare: merge inst64/inst32 exactly like div.v:202-226
    wire [63:0] div_dividend = !div_is_word ? div_src0_raw
                             : div_is_signed ? {{32{div_src0_raw[31]}}, div_src0_raw[31:0]}
                             :                 {32'b0, div_src0_raw[31:0]};
    wire [63:0] div_divisor  = !div_is_word ? div_src1_raw
                             : div_is_signed ? {{32{div_src1_raw[31]}}, div_src1_raw[31:0]}
                             :                 {32'b0, div_src1_raw[31:0]};

    wire div_dividend_eq0 = (div_dividend == 64'b0);
    wire div_divisor_eq0  = (div_divisor  == 64'b0);
    wire [63:0] div_overflow_dividend = div_is_word ? 64'hffff_ffff_8000_0000 : 64'h8000_0000_0000_0000;
    wire div_res_overflow = div_is_signed && (div_dividend == div_overflow_dividend)
                          && (div_divisor == 64'hffff_ffff_ffff_ffff);
    wire div_abnormal_res_vld = div_dividend_eq0 || div_divisor_eq0 || div_res_overflow;

    // 1-entry memo/hit buffer (div.v:770-787): reuse the immediately
    // preceding op's result if operands/signedness/word-ness all match.
    // Only the IDENTITY needs its own registers -- the actual quotient/
    // remainder do NOT need a separate memo copy: a hit means this op's
    // operands are IDENTICAL to whichever op most recently populated
    // div_{dividend,divisor}_flop (by induction back through any chain of
    // hits to the real ITER-path op that started it, since div_memo_vld
    // starts at 0 and the first real, non-abnormal divide necessarily
    // populates those flops) -- so div_quotient_final/div_remainder_final
    // (below) already give the right answer for a hit, for free.
    reg  [63:0] div_memo_dividend, div_memo_divisor;
    reg         div_memo_signed, div_memo_word;
    reg         div_memo_vld;
    wire div_hit_buffer = div_memo_vld
                        && (div_memo_dividend == div_dividend) && (div_memo_divisor == div_divisor)
                        && (div_memo_signed == div_is_signed) && (div_memo_word == div_is_word);

    // Signed absolute values + leading-1 BIT POSITION (0=bit0 .. 63=bit63;
    // NOT the same convention as ALU's FF0/FF1 above, which measures
    // distance-from-the-MSB -- div.v's OWN `div_ff1_res` table is the
    // opposite sense, a plain bit-position encoder, confirmed directly:
    // div.v:431 `64'b1???...: pos=63` (bit63 set -> position 63) down to
    // div.v:494 `64'b0...0001: pos=0` (bit0 set -> position 0). A bigger
    // dividend needs a HIGHER position number than a bigger divisor for
    // "position difference == iterations needed" to make sense, so this
    // is deliberately `63 - lead1_dist_msb(...)`, not lead1_dist_msb(...)
    // itself.) Used ONLY to derive the data-dependent ITER cycle count
    // (timing fidelity, div.v:412-426/div_shift2_kernel.v:99-126) -- not
    // used to compute the actual value.
    wire [63:0] div_dividend_abs = (div_is_signed && div_dividend[63]) ? (~div_dividend + 64'b1) : div_dividend;
    wire [63:0] div_divisor_abs  = (div_is_signed && div_divisor[63])  ? (~div_divisor  + 64'b1) : div_divisor;
    wire [6:0] div_dividend_lead1 = 7'd63 - lead1_dist_msb(div_dividend_abs);
    wire [6:0] div_divisor_lead1  = 7'd63 - lead1_dist_msb(div_divisor_abs);
    wire [6:0] div_iter_total = (div_dividend_lead1 > div_divisor_lead1)
                              ? (div_dividend_lead1 - div_divisor_lead1) : 7'd0;

    reg [6:0] div_iter_left;   // ITER-state countdown, 2/cycle (timing only)

    // Latched (WFI2-captured) operand class for the CMPLT-stage result mux.
    reg div_signed_flop, div_word_flop, div_sel_quotient_flop;
    reg [63:0] div_dividend_flop, div_divisor_flop;

    // Gated by "div_state == DIV_IDLE" (a genuinely NEW dispatch being
    // evaluated), not just idu_iu_ex1_div_sel alone: the "well-behaved
    // dispatcher" contract (IU.v header) holds `div_sel` steady describing
    // the SAME instruction while DIV is busy, which would otherwise make
    // `div_hit_buffer` spuriously match ITSELF once the memo captures this
    // op's own identity at dispatch (fixed together with div_result_live
    // below -- both halves of the same self-reference bug).
    wire div_new_dispatch = idu_iu_ex1_div_sel && (div_state == DIV_IDLE);
    wire div_iter_start   = div_new_dispatch && !div_abnormal_res_vld && !div_hit_buffer;
    wire div_ex1_res_vld  = div_new_dispatch && (div_abnormal_res_vld || div_hit_buffer);

    wire div_ex2_enable_wb = rtu_iu_div_wb_grant;
    wire [2:0] div_next_state;

    // Fast-path entry note: for the abnormal/hit fast path the result is
    // valid in the DIV_IDLE dispatch cycle itself. If the writeback grant is
    // accepted THAT cycle the op is done and stays IDLE; only when the grant
    // is blocked (another EX1-group writeback winning the rbus) does it move
    // to WFWB to wait. Going to WFWB unconditionally (the old formula)
    // produced a SECOND writeback pulse once WFWB got its own grant -- the
    // WBT busy-bit counter saw one create but two writebacks, underflowed,
    // and the destination register read as permanently pending (rv64um-p-divu
    // wedge at test 9).
    assign div_next_state =
          (div_state == DIV_IDLE)  ? (div_iter_start ? DIV_WFI2
                                     : (div_ex1_res_vld && !div_ex2_enable_wb) ? DIV_WFWB
                                     : DIV_IDLE)
        : (div_state == DIV_WFI2)  ? DIV_ALIGN
        : (div_state == DIV_ALIGN) ? DIV_ITER
        : (div_state == DIV_ITER)  ? ((div_iter_left <= 7'd1) ? DIV_CMPLT : DIV_ITER)
        : (div_state == DIV_CMPLT || div_state == DIV_WFWB)
              ? (div_ex2_enable_wb
                   ? (div_iter_start ? DIV_WFI2 : (div_ex1_res_vld ? DIV_WFWB : DIV_IDLE))
                   : DIV_WFWB)
        : DIV_IDLE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            div_state <= DIV_IDLE;
        else
            div_state <= div_next_state;
    end

    always @(posedge clk) begin
        if (div_state == DIV_IDLE && div_iter_start) begin
            div_dividend_flop      <= div_dividend;
            div_divisor_flop       <= div_divisor;
            div_signed_flop        <= div_is_signed;
            div_word_flop          <= div_is_word;
            div_sel_quotient_flop  <= div_sel_quotient;
            div_iter_left          <= div_iter_total;
        end
        else if (div_state == DIV_ITER)
            div_iter_left <= (div_iter_left <= 7'd2) ? 7'd0 : (div_iter_left - 7'd2);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_memo_vld <= 1'b0;
        end
        else if (idu_iu_ex1_div_sel && div_state == DIV_IDLE) begin
            div_memo_vld <= 1'b1;
        end
    end
    always @(posedge clk) begin
        if (idu_iu_ex1_div_sel && div_state == DIV_IDLE) begin
            div_memo_dividend <= div_dividend;
            div_memo_divisor  <= div_divisor;
            div_memo_signed   <= div_is_signed;
            div_memo_word     <= div_is_word;
        end
    end

    wire [63:0] div_dividend_signed_abs = (div_signed_flop && div_dividend_flop[63])
                                         ? (~div_dividend_flop + 64'b1) : div_dividend_flop;
    wire [63:0] div_divisor_signed_abs  = (div_signed_flop && div_divisor_flop[63])
                                         ? (~div_divisor_flop  + 64'b1) : div_divisor_flop;
    wire [63:0] div_quotient_raw  = (div_divisor_signed_abs == 64'b0) ? 64'b0
                                   : (div_dividend_signed_abs / div_divisor_signed_abs);
    wire [63:0] div_remainder_raw = (div_divisor_signed_abs == 64'b0) ? div_dividend_signed_abs
                                   : (div_dividend_signed_abs % div_divisor_signed_abs);
    wire div_result_quotient_neg  = div_signed_flop && (div_dividend_flop[63] ^ div_divisor_flop[63]);
    wire div_result_remainder_neg = div_signed_flop && div_dividend_flop[63];
    wire [63:0] div_quotient_signed  = div_result_quotient_neg  ? (~div_quotient_raw  + 64'b1) : div_quotient_raw;
    wire [63:0] div_remainder_signed = div_result_remainder_neg ? (~div_remainder_raw + 64'b1) : div_remainder_raw;
    wire [63:0] div_quotient_final  = div_word_flop ? {{32{div_quotient_signed[31]}}, div_quotient_signed[31:0]}
                                                     : div_quotient_signed;
    wire [63:0] div_remainder_final = div_word_flop ? {{32{div_remainder_signed[31]}}, div_remainder_signed[31:0]}
                                                     : div_remainder_signed;

    // Abnormal-result mux (div.v:374-406), evaluated off the CURRENT
    // dispatch's own fields (abnormal results resolve same-cycle, before
    // any WFI2/ALIGN/ITER latch would matter).
    wire [63:0] div_overflow_quotient = div_overflow_dividend;
    wire [63:0] div_divzero_remainder = div_is_word ? {{32{div_dividend[31]}}, div_dividend[31:0]} : div_dividend;
    wire [63:0] div_abnormal_quotient  = div_res_overflow  ? div_overflow_quotient
                                        : div_divisor_eq0   ? 64'hffff_ffff_ffff_ffff
                                        : div_dividend_eq0  ? 64'b0
                                        : div_quotient_final;
    wire [63:0] div_abnormal_remainder = div_res_overflow  ? 64'b0
                                        : div_divisor_eq0   ? div_divzero_remainder
                                        : div_dividend_eq0  ? 64'b0
                                        : div_remainder_final;

    // `iu_rtu_div_wb_vld`/`_data` must be COMBINATIONALLY available the
    // instant a result resolves (matching the donor's own `iu_rtu_div_
    // wb_vld = div_cmplt || div_wfwb`, `iu_rtu_div_data = div_ex2_res`,
    // both plain combinational wires) -- a register here would delay the
    // pulse by a cycle relative to the FSM state it is supposed to track.
    // The abnormal/hit-buffer fast path is the one case that DOES need a
    // snapshot: its result is derived from the live `idu_iu_ex1_*` bus,
    // which IDU is free to replace with a different instruction the very
    // next cycle once this op's `div_ex1_res_vld` has been seen -- so it is
    // latched on the resolving cycle and the latch is used only on LATER
    // (already-resolved, still-waiting-for-grant) cycles, not the
    // resolving cycle itself.
    reg  [63:0] div_result_reg;
    reg  [GPR_IDX_WIDTH-1:0] div_preg_reg;

    wire div_cmplt_now = (div_state == DIV_CMPLT) || (div_state == DIV_IDLE && div_ex1_res_vld);

    // The live (this-cycle) result, valid exactly when div_cmplt_now is 1.
    // NOTE: the non-IDLE (real ITER-path) branch must NOT re-check
    // div_hit_buffer here -- by the time state==CMPLT, the memo was
    // already overwritten (at the original dispatch cycle) with THIS same
    // op's own identity, so a live re-check would spuriously "hit" against
    // itself. hit_buffer only ever matters at the IDLE dispatch decision
    // (div_ex1_res_vld above), which is why the abnormal/hit fast path is
    // folded into the DIV_IDLE branch instead.
    wire [63:0] div_result_live = (div_state == DIV_IDLE)
                                 ? (div_sel_quotient ? div_abnormal_quotient : div_abnormal_remainder)
                                 : (div_sel_quotient_flop ? div_quotient_final : div_remainder_final);

    // BUG FIX (rv64um div/rem hang): the destination register must be
    // latched AT DISPATCH, exactly like MUL's mul_ex2_preg (line ~983) --
    // the iter path keeps the DIV FSM busy for several cycles after the
    // instruction has already retired out of EX1, so at DIV_CMPLT the live
    // idu_iu_ex1_dst0_reg names SOME LATER instruction (or a cleared EX1),
    // and the result was written back to the wrong register while the real
    // destination's WBT busy-bit stayed armed forever (permanent RAW stall;
    // rv64um-p-divu wedged on test 2's `bne a4,...`).
    always @(posedge clk) begin
        if (div_new_dispatch)
            div_preg_reg <= idu_iu_ex1_dst0_reg;
    end

    always @(posedge clk) begin
        if (div_cmplt_now)
            div_result_reg <= div_result_live;
    end

    wire div_wb_now = div_cmplt_now || (div_state == DIV_WFWB);

    assign iu_rtu_ex1_div_cmplt    = idu_iu_ex1_div_sel;
    assign iu_rtu_ex1_div_cmplt_dp = idu_iu_ex1_div_sel;
    // Task 7.3: completing-DIV length. Tied 1 (32-bit) -- no 16-bit DIV in
    // M2's RVC decoder (see the port-list note).
    assign iu_rtu_ex1_div_inst_len = 1'b1;
    assign iu_rtu_div_wb_dp      = div_wb_now;
    assign iu_rtu_div_wb_vld     = div_wb_now;
    // Fast-path (abnormal/hit) completion fires in the DIV_IDLE dispatch
    // cycle itself, where the live EX1 dst is still the div's own; every
    // later cycle (DIV_CMPLT/DIV_WFWB of the iter path) uses the latch.
    assign iu_rtu_div_preg       = (div_cmplt_now && div_state == DIV_IDLE)
                                 ? idu_iu_ex1_dst0_reg : div_preg_reg;
    assign iu_rtu_div_data       = div_cmplt_now ? div_result_live : div_result_reg;

    // Donor aq_iu_div.v:764 defines div_full over the running states only;
    // the donor's CMPLT/WFWB grant-exemption is dead there because
    // aq_rtu_rbus.v:470 ties rbus_div_wb_grant_for_full=1'b1, so the donor
    // keeps EX1 held through the WB-grant cycle. The M2 port wired the live
    // grant into the full term instead, opening a one-cycle window: on the
    // CMPLT/WFWB grant cycle a DIV parked in EX1 during a long division is
    // released (adv=1) while div_new_dispatch still requires DIV_IDLE (true
    // only the cycle after) -- the div is evicted, latched nowhere, and its
    // WBT entry is orphaned (silent instruction loss; the next consumer of
    // the dst RAW-stalls forever). Holding EX1 through the grant cycle is
    // the donor's behavior; the parked div then dispatches on the following
    // DIV_IDLE cycle. (The donor's grant-cycle back-to-back latch,
    // aq_iu_div.v:317-319 div_is_idle/div_iter_start, stays unported; see
    // the FSM note -- rv906 already deviates with IDLE-cycle fast WB.)
    assign iu_idu_div_full = (div_state != DIV_IDLE);

    //=========================================================================
    // SECTION OUTPUT -- CSR-facing PC passthrough (IU note S4.5/S9).
    //=========================================================================
    assign iu_cp0_ex1_cur_pc = bju_pcgen_pc;
    // M3b Task D: PFB trainer's PC tag (donor aq_lsu_ag.v:685 takes the same
    // EX1 PC the donor IU broadcasts here).
    assign iu_lsu_ex1_cur_pc = bju_pcgen_pc[15:0];
    // M6 Task 7: full-width twin for the LSU's replying-op pc latch (see the
    // port comment -- same source, same validity caveats as the 16-bit tag).
    assign iu_lsu_ex1_pc     = bju_pcgen_pc;

endmodule
