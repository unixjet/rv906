//=============================================================================
// RTU.v - retire unit: one-hot completion OR, EX1->EX2 retire latch,
//          exception/interrupt priority, flush FSM, rbus/wb arbitration
//                                                (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 4; this file freezes
// the port list only):
//   gen_rtl/rtu/rtl/aq_rtu_top.v    (glue)
//   gen_rtl/rtu/rtl/aq_rtu_ctrl.v   (one-hot cmplt OR -> retire_vld register)
//   gen_rtl/rtu/rtl/aq_rtu_dp.v     (EX1->EX2 retire-packet pipe register)
//   gen_rtl/rtu/rtl/aq_rtu_rbus.v   (writeback-value arbiter + 3 fwd ports)
//   gen_rtl/rtu/rtl/aq_rtu_retire.v (exception/int priority, flush FSM,
//                                     changeflow-PC mux, every RTU-> output)
//   gen_rtl/rtu/rtl/aq_rtu_wb.v     (2-port GPR writeback packaging)
//   gen_rtl/rtu/rtl/aq_rtu_int.v    (15-source interrupt-cause priority)
// References: design doc S4.1/S4.2/S6 (unit graph, rtu_ex2_t is RTU-
// internal, the writeback-bus decision), RTU extraction note (all
// sections, esp. S2 retire structure, S3 rbus/wb, S4 exception priority,
// S6 flush/redirect signal inventory, S7 CSR-writeback timing).
//
// SEAM NOTES:
//  * RTU is NOT a reorder buffer (contract/RTU note S0/S9) -- one un-
//    buffered EX1->EX2 pipeline register, retiring at most 1/cycle, 0/cycle
//    on any stall, no queue. `rtu_ex2_t` (design doc S4.2) is RTU-internal
//    (Task 4's own packed struct), never a port.
//  * The one-hot completion bus's 7th source, `vec_cmplt_dp`, has NO M2
//    port here at all -- there is no VPU in M2 (contract/RTU note S2's
//    `{alu,mul,bju,div,lsu,cp0,vec}_cmplt_dp`); Task 4's real body ties
//    that leg to 0 internally rather than accepting an external wire for
//    it.
//  * `rtu_ifu_chgflw_vld`/`_pc`/`rtu_ifu_flush_fe` are reused UNCHANGED from
//    M1 -- names/widths copied verbatim from rtl/RVProc.v's current wire
//    declarations (lines 287-289), already consumed by IFU.v since M1.
//  * KNOWN, DELIBERATE GAP (mirrors CSR.v's/M1's BPU.v Task-7-amendment
//    precedent, not a freeze violation): no `cp0_rtu_int_vld`-shaped
//    pre-masked-interrupt-vector input exists on this port list. M2's
//    minimal mie/mip model (contract 7) is far simpler than the donor's
//    full 15-source `cp0_rtu_int_vld[14:0]` (RTU note S5) -- whether M2's
//    interrupt-priority leg (wired-but-never-fires, contract 1) computes
//    priority from mie/mip directly inside RTU.v itself, or still takes a
//    pre-masked vector from CSR.v, is left for Task 4 to decide and record
//    (RTU note S5's own open item: the vector-building module lives in
//    cp0/rtl and was not traced). Adding this deliberately here rather
//    than guessing a shape/width now.
//  * Likewise, whether LSU's 3 distinct donor-side writeback paths (EX1
//    rbus-fast, EX2 one-cycle-later forward, and a separate late "wb port1"
//    -- RTU note S2/S3) collapse to the 2 paths this skeleton pins
//    (`lsu_rtu_ex1_*`/`_cmplt_dp` for rbus+one-hot-cmplt, `lsu_rtu_ex2_*`
//    for fwd2) or need a 3rd port added is the exact open item RTU note S2
//    flags as "not fully resolved from RTU-side RTL alone" -- Task 4/6
//    resolve it together against LSU's real body, not guessed here.
//=============================================================================

import rvproc_pkg::*;

module RTU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IU -> RTU : the four writeback buses (contract 1) -- ALU/BJU always
    // one-hot-exclusive with each other and with MULT/DIV's EX1 early-
    // accept; MULT's EX3 data/writeback and DIV's variable-EX-stage data/
    // writeback arrive separately, gated on this module's own wb_grant
    // outputs below.
    //=========================================================================
    input  wire                     iu_rtu_ex1_alu_cmplt,
    input  wire                     iu_rtu_ex1_alu_cmplt_dp,
    input  wire [63:0]              iu_rtu_ex1_alu_data,
    input  wire                     iu_rtu_ex1_alu_inst_len,
    input  wire                     iu_rtu_ex1_alu_inst_split,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_alu_preg,
    input  wire                     iu_rtu_ex1_alu_wb_dp,
    input  wire                     iu_rtu_ex1_alu_wb_vld,

    input  wire                     iu_rtu_ex1_bju_cmplt,
    input  wire                     iu_rtu_ex1_bju_cmplt_dp,
    input  wire [63:0]              iu_rtu_ex1_bju_data,
    input  wire                     iu_rtu_ex1_bju_inst_len,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_bju_preg,
    input  wire                     iu_rtu_ex1_bju_wb_dp,
    input  wire                     iu_rtu_ex1_bju_wb_vld,
    input  wire                     iu_rtu_ex1_branch_inst,
    input  wire [PC_WIDTH-1:0]      iu_rtu_ex1_cur_pc,
    input  wire [PC_WIDTH-1:0]      iu_rtu_ex1_next_pc,
    input  wire                     iu_rtu_ex2_bju_ras_mispred,
    input  wire                     iu_rtu_depd_lsu_chgflow_vld,
    input  wire [PC_WIDTH-1:0]      iu_rtu_depd_lsu_chgflow_next_pc,

    input  wire                     iu_rtu_ex1_mul_cmplt,
    input  wire                     iu_rtu_ex1_mul_cmplt_dp,
    input  wire [63:0]              iu_rtu_ex3_mul_data,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex3_mul_preg,
    input  wire                     iu_rtu_ex3_mul_wb_vld,

    input  wire                     iu_rtu_ex1_div_cmplt,
    input  wire                     iu_rtu_ex1_div_cmplt_dp,
    input  wire [63:0]              iu_rtu_div_data,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_div_preg,
    input  wire                     iu_rtu_div_wb_dp,
    input  wire                     iu_rtu_div_wb_vld,

    //=========================================================================
    // RTU -> IU : the two writeback-race grants (IU note S0/S5/S6).
    //=========================================================================
    output wire                     rtu_iu_mul_wb_grant,
    output wire                     rtu_iu_div_wb_grant,

    //=========================================================================
    // LSU -> RTU : lsu_rtu_t (design doc S4.2) -- see the header's "known,
    // deliberate gap" note on the exact path count.
    //=========================================================================
    input  wire                     lsu_rtu_ex1_cmplt,
    input  wire                     lsu_rtu_ex1_cmplt_dp,
    input  wire [63:0]              lsu_rtu_wb_data,
    input  wire [GPR_IDX_WIDTH-1:0] lsu_rtu_wb_preg,
    input  wire                     lsu_rtu_wb_vld,
    input  wire [63:0]              lsu_rtu_ex2_data,
    input  wire                     lsu_rtu_ex2_data_vld,
    input  wire                     lsu_rtu_expt_vld,
    input  wire [4:0]               lsu_rtu_expt_vec,
    input  wire [63:0]              lsu_rtu_tval,
    input  wire                     lsu_rtu_async_expt_vld,
    input  wire                     lsu_rtu_async_ld_inst,

    //=========================================================================
    // RTU -> LSU : "point of no return" acks LSU needs to release/drop any
    // buffered store state (RTU note S6, directly relevant to Task 6).
    //=========================================================================
    output wire                     rtu_lsu_expt_ack,
    output wire                     rtu_lsu_expt_exit,

    //=========================================================================
    // CSR -> RTU : cp0_rtu_t (design doc S4.2).
    //=========================================================================
    input  wire                     cp0_rtu_ex1_cmplt_dp,
    input  wire [63:0]              cp0_rtu_ex1_wb_data,
    input  wire [GPR_IDX_WIDTH-1:0] cp0_rtu_ex1_wb_preg,
    input  wire                     cp0_rtu_ex1_wb_vld,
    input  wire                     cp0_rtu_ex1_expt_vld,
    input  wire                     cp0_rtu_ex1_expt_int,
    input  wire [4:0]               cp0_rtu_ex1_expt_vec,
    input  wire                     cp0_rtu_ex1_chgflw,
    input  wire [PC_WIDTH-1:0]      cp0_rtu_ex1_chgflw_pc,

    //=========================================================================
    // RTU -> CSR : trap-entry capture (RTU note S7).
    //=========================================================================
    output wire                     rtu_yy_xx_expt_vld,
    output wire                     rtu_yy_xx_expt_int,
    output wire [4:0]               rtu_yy_xx_expt_vec,
    output wire                     rtu_yy_xx_flush_fe,
    output wire                     rtu_yy_xx_flush,
    output wire                     rtu_yy_xx_dbgon,
    output wire [PC_WIDTH-1:0]      rtu_cp0_epc,
    output wire [63:0]              rtu_cp0_tval,

    //=========================================================================
    // RTU -> IFU : redirect target + FE-kill pulse. Reused UNCHANGED from
    // M1 -- names/widths copied verbatim from rtl/RVProc.v's current wire
    // declarations (lines 287-289), already the signal IFU.v/BPU.v consume.
    //=========================================================================
    output wire                     rtu_ifu_chgflw_vld,
    output wire [PC_WIDTH-1:0]      rtu_ifu_chgflw_pc,
    output wire                     rtu_ifu_flush_fe,

    //=========================================================================
    // RTU -> IDU : forward/commit ports (contract 8's exception-side) +
    // the flush/drain signal group (RTU note S6).
    //=========================================================================
    output wire [63:0]              rtu_idu_fwd0_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd0_reg,
    output wire                     rtu_idu_fwd0_vld,
    output wire [63:0]              rtu_idu_fwd1_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd1_reg,
    output wire                     rtu_idu_fwd1_vld,
    output wire [63:0]              rtu_idu_fwd2_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd2_reg,
    output wire                     rtu_idu_fwd2_vld,
    output wire [63:0]              rtu_idu_wb0_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb0_reg,
    output wire                     rtu_idu_wb0_vld,
    output wire [63:0]              rtu_idu_wb1_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb1_reg,
    output wire                     rtu_idu_wb1_vld,
    output wire                     rtu_idu_flush_fe,
    output wire                     rtu_idu_flush_stall,
    output wire                     rtu_idu_flush_wbt,
    output wire                     rtu_idu_commit,
    output wire                     rtu_idu_commit_for_bju,
    output wire                     rtu_idu_pipeline_empty
);

    //=========================================================================
    // SKELETON BODY (plan Task 4 replaces it): every output inactive/0 --
    // nothing ever commits, nothing ever grants, nothing ever redirects.
    //=========================================================================
    assign rtu_iu_mul_wb_grant = 1'b0;
    assign rtu_iu_div_wb_grant = 1'b0;

    assign rtu_lsu_expt_ack  = 1'b0;
    assign rtu_lsu_expt_exit = 1'b0;

    assign rtu_yy_xx_expt_vld = 1'b0;
    assign rtu_yy_xx_expt_int = 1'b0;
    assign rtu_yy_xx_expt_vec = 5'd0;
    assign rtu_yy_xx_flush_fe = 1'b0;
    assign rtu_yy_xx_flush    = 1'b0;
    assign rtu_yy_xx_dbgon    = 1'b0;
    assign rtu_cp0_epc        = {PC_WIDTH{1'b0}};
    assign rtu_cp0_tval       = 64'd0;

    assign rtu_ifu_chgflw_vld = 1'b0;
    assign rtu_ifu_chgflw_pc  = {PC_WIDTH{1'b0}};
    assign rtu_ifu_flush_fe   = 1'b0;

    assign rtu_idu_fwd0_data = 64'd0;
    assign rtu_idu_fwd0_reg  = {GPR_IDX_WIDTH{1'b0}};
    assign rtu_idu_fwd0_vld  = 1'b0;
    assign rtu_idu_fwd1_data = 64'd0;
    assign rtu_idu_fwd1_reg  = {GPR_IDX_WIDTH{1'b0}};
    assign rtu_idu_fwd1_vld  = 1'b0;
    assign rtu_idu_fwd2_data = 64'd0;
    assign rtu_idu_fwd2_reg  = {GPR_IDX_WIDTH{1'b0}};
    assign rtu_idu_fwd2_vld  = 1'b0;
    assign rtu_idu_wb0_data  = 64'd0;
    assign rtu_idu_wb0_reg   = {GPR_IDX_WIDTH{1'b0}};
    assign rtu_idu_wb0_vld   = 1'b0;
    assign rtu_idu_wb1_data  = 64'd0;
    assign rtu_idu_wb1_reg   = {GPR_IDX_WIDTH{1'b0}};
    assign rtu_idu_wb1_vld   = 1'b0;

    // A stuck-low `rtu_idu_commit`/high `rtu_idu_flush_stall` would wedge
    // IDU's EX1 issue-gate forever once real bodies exist; the skeleton
    // still ties every output inactive/0 per the M1 precedent (Tasks 2-6
    // never wire these modules into RVProc.v, so this has no live effect
    // until Task 7).
    assign rtu_idu_flush_fe      = 1'b0;
    assign rtu_idu_flush_stall   = 1'b0;
    assign rtu_idu_flush_wbt     = 1'b0;
    assign rtu_idu_commit        = 1'b0;
    assign rtu_idu_commit_for_bju= 1'b0;
    assign rtu_idu_pipeline_empty= 1'b0;

endmodule
