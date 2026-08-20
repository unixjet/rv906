//=============================================================================
// IU.v - integer execute pipe: ALU + BJU + MULT + DIV  (M2 SKELETON: ports
//                                                         frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 3; this file freezes
// the port list only):
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
// SEAM NOTES (rv906 decomposition):
//  * IU.v houses ALL FOUR integer execute units (contract 0/IU note S0) --
//    there is no separate ALU.v/BJU.v/MULT.v/DIV.v file; per umbrella S6.2
//    rule 7 these stay logic-internal to IU.v rather than earning their own
//    files, mirroring the donor's own `aq_iu_top.v` module boundary (unlike
//    MULT/DIV's own umbrella-rule-7 example wording, which names them as
//    plausible own-file candidates in the abstract -- the design doc's own
//    S4.3 file org table pins all four inside `rtl/IU.v`, so that concrete
//    decision is followed here, not the rule's illustrative list).
//  * IU exposes FOUR SEPARATE writeback buses to RTU (ALU/BJU/MULT/DIV,
//    contract 1) -- no merge happens inside this module; RTU's own rbus
//    arbiter (Task 4) does the merge. None of the four buses carry
//    exception/fault flags (IU note S10) -- illegal-instruction/misaligned/
//    etc. arrive at RTU via CSR.v's/LSU.v's own paths instead.
//  * The IFU-facing BJU redirect/BHT-feedback/RAS group below is reused
//    UNCHANGED from M1 -- every name/width was re-verified byte-for-byte
//    against `rtl/RVProc.v`'s current wire declarations (lines 275-285),
//    not renamed or rewidened. `iu_ifu_tar_pc` is intentionally 64 bits
//    (not PC_WIDTH) because that is RVProc.v's own already-frozen M1
//    declaration width.
//  * `da_xx_fwd_*`/`lsu_iu_ex2_*` (IU note S4.4) are consumed ONLY by BJU's
//    1-entry LSU-dependent conditional-branch buffer -- ordinary ALU/MULT/
//    DIV ops never see them; any load-dependency stall for those units is
//    resolved upstream in IDU's scoreboard before `idu_iu_ex1_*` asserts.
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

    // DIV (EX1 early-accept, variable-EX-stage data/writeback -- gated on
    // rtu_iu_div_wb_grant, IU note S6/S10).
    output wire                     iu_rtu_ex1_div_cmplt,
    output wire                     iu_rtu_ex1_div_cmplt_dp,
    output wire [63:0]              iu_rtu_div_data,
    output wire [GPR_IDX_WIDTH-1:0] iu_rtu_div_preg,
    output wire                     iu_rtu_div_wb_dp,
    output wire                     iu_rtu_div_wb_vld,

    //=========================================================================
    // RTU -> IU : the single-bit writeback-race grants for MULT/DIV -- RTU
    // is where the completion-vs-writeback-bus race is resolved, not an
    // IU-internal arbiter (IU note S0/S5/S6).
    //=========================================================================
    input  wire                     rtu_iu_mul_wb_grant,
    input  wire                     rtu_iu_div_wb_grant,

    //=========================================================================
    // Already-frozen-since-M1 IFU-facing BJU ports, reused UNCHANGED --
    // names/widths copied verbatim from rtl/RVProc.v's current wire
    // declarations (lines 275-285). `iu_ifu_tar_pc` is 64b there, not
    // PC_WIDTH; do not narrow it here.
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
    // CP0 -> IU : BJU's reset PC seed (already exists as an M1 port on
    // RVProc.v, `cp0_xx_mrvbr` -- restore-checklist item 4, IU note S4.5).
    //=========================================================================
    input  wire [PC_WIDTH-1:0]      cp0_xx_mrvbr
);

    //=========================================================================
    // SKELETON BODY (plan Task 3 replaces it): every output inactive/0.
    //=========================================================================
    assign iu_idu_mult_issue_stall = 1'b0;
    assign iu_idu_mult_full        = 1'b0;
    assign iu_idu_div_full         = 1'b0;
    assign iu_idu_bju_full         = 1'b0;
    assign iu_idu_bju_global_full  = 1'b0;

    assign iu_rtu_ex1_alu_cmplt       = 1'b0;
    assign iu_rtu_ex1_alu_cmplt_dp    = 1'b0;
    assign iu_rtu_ex1_alu_data        = 64'd0;
    assign iu_rtu_ex1_alu_inst_len    = 1'b0;
    assign iu_rtu_ex1_alu_inst_split  = 1'b0;
    assign iu_rtu_ex1_alu_preg        = {GPR_IDX_WIDTH{1'b0}};
    assign iu_rtu_ex1_alu_wb_dp       = 1'b0;
    assign iu_rtu_ex1_alu_wb_vld      = 1'b0;

    assign iu_rtu_ex1_bju_cmplt              = 1'b0;
    assign iu_rtu_ex1_bju_cmplt_dp           = 1'b0;
    assign iu_rtu_ex1_bju_data               = 64'd0;
    assign iu_rtu_ex1_bju_inst_len           = 1'b0;
    assign iu_rtu_ex1_bju_preg               = {GPR_IDX_WIDTH{1'b0}};
    assign iu_rtu_ex1_bju_wb_dp              = 1'b0;
    assign iu_rtu_ex1_bju_wb_vld             = 1'b0;
    assign iu_rtu_ex1_branch_inst            = 1'b0;
    assign iu_rtu_ex1_cur_pc                 = {PC_WIDTH{1'b0}};
    assign iu_rtu_ex1_next_pc                = {PC_WIDTH{1'b0}};
    assign iu_rtu_ex2_bju_ras_mispred        = 1'b0;
    assign iu_rtu_depd_lsu_chgflow_vld       = 1'b0;
    assign iu_rtu_depd_lsu_chgflow_next_pc   = {PC_WIDTH{1'b0}};

    assign iu_rtu_ex1_mul_cmplt    = 1'b0;
    assign iu_rtu_ex1_mul_cmplt_dp = 1'b0;
    assign iu_rtu_ex3_mul_data     = 64'd0;
    assign iu_rtu_ex3_mul_preg     = {GPR_IDX_WIDTH{1'b0}};
    assign iu_rtu_ex3_mul_wb_vld   = 1'b0;

    assign iu_rtu_ex1_div_cmplt    = 1'b0;
    assign iu_rtu_ex1_div_cmplt_dp = 1'b0;
    assign iu_rtu_div_data         = 64'd0;
    assign iu_rtu_div_preg         = {GPR_IDX_WIDTH{1'b0}};
    assign iu_rtu_div_wb_dp        = 1'b0;
    assign iu_rtu_div_wb_vld       = 1'b0;

    assign iu_ifu_tar_pc_vld  = 1'b0;
    assign iu_ifu_tar_pc      = 64'd0;
    assign iu_ifu_pc_mispred  = 1'b0;
    assign iu_ifu_bht_mispred = 1'b0;
    assign iu_ifu_br_vld      = 1'b0;
    assign iu_ifu_bht_taken   = 1'b0;
    assign iu_ifu_bht_pred    = 2'd0;
    assign iu_ifu_link_vld    = 1'b0;
    assign iu_ifu_ret_vld     = 1'b0;

    assign iu_cp0_ex1_cur_pc = {PC_WIDTH{1'b0}};

endmodule
