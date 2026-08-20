//=============================================================================
// IDU.v - decode + WBT scoreboard + GPR + RTU-forward mux + EU dispatch
//                                                (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 5; this file freezes
// the port list only):
//   gen_rtl/idu/rtl/aq_idu_top.v      (glue)
//   gen_rtl/idu/rtl/aq_idu_id_decd.v  (decoder: 6-way coarse classify then
//                                       per-class casez tables, incl. RVC)
//   gen_rtl/idu/rtl/aq_idu_id_split.v (multi-beat splitter -- amo/lsd/che/
//                                       fnc; NOT needed for M2, contract 10)
//   gen_rtl/idu/rtl/aq_idu_id_wbt.v   (+ _entry.v; 32-entry busy-bit
//                                       scoreboard)
//   gen_rtl/idu/rtl/aq_idu_id_gpr.v   (+ _gated_reg.v; 31-entry GPR + x0)
//   gen_rtl/idu/rtl/aq_idu_id_dp.v    (decoder/splitter mux, WBT/GPR read-
//                                       address gen, 3-source forward mux,
//                                       the EX1 pipeline register)
//   gen_rtl/idu/rtl/aq_idu_id_ctrl.v  (RAW/WAW stall, EU one-hot select,
//                                       EX1 issue-enable gating,
//                                       idu_ifu_id_stall)
// References: design doc S4.1/S4.2/S5 (unit graph, id_ex1_t, pipeline-stage
// table), IDU extraction note (all sections, esp. S5 hazard scheme, S6
// forward network, S7 dispatch/issue-gate).
//
// SEAM NOTES:
//  * ID and Dispatch are the SAME combinational stage (IDU note S2) -- no
//    register between decode and hazard-check/EU-select. The ONLY register
//    inside IDU.v proper is the EX1 latch (id_ex1_t payload + EU one-hot
//    select, pinned in rvproc_pkg.sv). WBT/GPR (31 flop entries each + x0)
//    are entry arrays per umbrella S6.2 rule 6, not per-entry modules.
//  * `ifu_idu_id_inst`/`_inst_vld`/`_bht_pred` and `idu_ifu_id_stall` are
//    reused UNCHANGED from M1 -- names/widths copied verbatim from
//    rtl/RVProc.v's current wire declarations (lines 267-270); the stall
//    reason becomes real here instead of FetchSink's fake one.
//  * Every bypass path into this module's operand-read logic is sourced
//    EXCLUSIVELY from RTU (`rtu_idu_fwd0/1/2_*`, `rtu_idu_wb0/1_*`, IDU
//    note S6) -- there is no direct IU->IDU or LSU->IDU bypass wire on this
//    port list, matching contract 1's "IDU never sees any of the four
//    IU->RTU writeback buses directly."
//  * `idu_lsu_ex1_dp_sel` is named per the design doc's/plan's own
//    repeated spelling (design doc S4.1 unit graph, plan 1.2's LSU.v/IDU.v
//    bullets) even though `aq_idu_id_ctrl.v:634`'s confirmed real name is
//    `idu_lsu_ex1_sel` (no "dp"). Task 5 should verify which spelling is
//    actually intended when LSU.v's real body is wired up and rename if
//    the "dp" turns out to be a typo carried through the design doc.
//  * `lsu_idu_full` (LSU's single stall signal to IDU, contract 8) is
//    CONFIRMED directly from `aq_idu_id_ctrl.v:634`'s real consumer-side
//    reference (`!lsu_idu_full`), not guessed from the LSU note's prose.
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
    output wire                     idu_ifu_id_stall,

    //=========================================================================
    // IDU -> IU : EX1 dispatch, IU's slice of id_ex1_t (matches IU.v's
    // input port group exactly).
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

    //=========================================================================
    // IDU -> LSU : EX1 dispatch, LSU's slice of id_ex1_t (matches LSU.v's
    // input port group exactly -- see the header note on the "dp_sel" name).
    //=========================================================================
    output wire                     idu_lsu_ex1_dp_sel,
    output wire [FUNC_WIDTH-1:0]    idu_lsu_ex1_func,
    output wire [63:0]              idu_lsu_ex1_src0_data,
    output wire                     idu_lsu_ex1_src0_ready,
    output wire [63:0]              idu_lsu_ex1_src1_data,
    output wire                     idu_lsu_ex1_src1_ready,
    output wire [63:0]              idu_lsu_ex1_src2_data,
    output wire                     idu_lsu_ex1_src2_ready,
    output wire [GPR_IDX_WIDTH-1:0] idu_lsu_ex1_dst0_reg,

    //=========================================================================
    // IDU -> CSR : EX1 dispatch, CSR's slice of id_ex1_t (matches CSR.v's
    // input port group exactly).
    //=========================================================================
    output wire                     idu_cp0_ex1_sel,
    output wire [FUNC_WIDTH-1:0]    idu_cp0_ex1_func,
    output wire [31:0]              idu_cp0_ex1_opcode,
    output wire                     idu_cp0_ex1_illegal,
    output wire [63:0]              idu_cp0_ex1_src0_data,
    output wire [63:0]              idu_cp0_ex1_src1_data,
    output wire [GPR_IDX_WIDTH-1:0] idu_cp0_ex1_dst0_reg,

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
    // LSU -> IDU : the single EX1 issue-gate stall signal (contract 8;
    // confirmed name `lsu_idu_full`, aq_idu_id_ctrl.v:634).
    //=========================================================================
    input  wire                     lsu_idu_full,

    //=========================================================================
    // RTU -> IDU : flush/drain group (RTU note S6).
    //=========================================================================
    input  wire                     rtu_idu_flush_fe,
    input  wire                     rtu_idu_flush_stall,
    input  wire                     rtu_idu_flush_wbt,
    input  wire                     rtu_idu_commit,
    input  wire                     rtu_idu_commit_for_bju
);

    //=========================================================================
    // SKELETON BODY (plan Task 5 replaces it): every output inactive/0 --
    // no instruction is ever dispatched to any EU, and the front end is
    // held (matches FetchSink's own M1 "always stall" precedent style,
    // but for the real reason that IDU has no real decode yet).
    //=========================================================================
    assign idu_ifu_id_stall = 1'b1;

    assign idu_iu_ex1_inst_vld     = 1'b0;
    assign idu_iu_ex1_pipedown_vld = 1'b0;
    assign idu_iu_ex1_alu_sel      = 1'b0;
    assign idu_iu_ex1_bju_sel      = 1'b0;
    assign idu_iu_ex1_bju_br_sel   = 1'b0;
    assign idu_iu_ex1_mult_sel     = 1'b0;
    assign idu_iu_ex1_div_sel      = 1'b0;
    assign idu_iu_ex1_func         = {FUNC_WIDTH{1'b0}};
    assign idu_iu_ex1_src0_data    = 64'd0;
    assign idu_iu_ex1_src0_ready   = 1'b0;
    assign idu_iu_ex1_src1_data    = 64'd0;
    assign idu_iu_ex1_src1_ready   = 1'b0;
    assign idu_iu_ex1_src2_data    = 64'd0;
    assign idu_iu_ex1_src2_ready   = 1'b0;
    assign idu_iu_ex1_dst0_reg     = {GPR_IDX_WIDTH{1'b0}};

    assign idu_lsu_ex1_dp_sel      = 1'b0;
    assign idu_lsu_ex1_func        = {FUNC_WIDTH{1'b0}};
    assign idu_lsu_ex1_src0_data   = 64'd0;
    assign idu_lsu_ex1_src0_ready  = 1'b0;
    assign idu_lsu_ex1_src1_data   = 64'd0;
    assign idu_lsu_ex1_src1_ready  = 1'b0;
    assign idu_lsu_ex1_src2_data   = 64'd0;
    assign idu_lsu_ex1_src2_ready  = 1'b0;
    assign idu_lsu_ex1_dst0_reg    = {GPR_IDX_WIDTH{1'b0}};

    assign idu_cp0_ex1_sel         = 1'b0;
    assign idu_cp0_ex1_func        = {FUNC_WIDTH{1'b0}};
    assign idu_cp0_ex1_opcode      = 32'd0;
    assign idu_cp0_ex1_illegal     = 1'b0;
    assign idu_cp0_ex1_src0_data   = 64'd0;
    assign idu_cp0_ex1_src1_data   = 64'd0;
    assign idu_cp0_ex1_dst0_reg    = {GPR_IDX_WIDTH{1'b0}};

endmodule
