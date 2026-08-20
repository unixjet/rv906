//=============================================================================
// LSU.v - AG -> DC -> DA load/store pipe, 4-entry STB, single-outstanding
//          miss stand-in, D-side AXI master   (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 6; this file freezes
// the port list only):
//   gen_rtl/lsu/rtl/aq_lsu_top.v   (glue/instantiation order, LSU note A1)
//   gen_rtl/lsu/rtl/aq_lsu_ag.v    (address-gen: adder, misalign detect,
//                                    MMU-stub request issue)
//   gen_rtl/lsu/rtl/aq_lsu_dc.v    (tag compare / DC+DA stages, byte
//                                    rotate + sign/zero-extend)
//   gen_rtl/lsu/rtl/aq_lsu_stb.v   (+ _entry.v; 4-entry store buffer,
//                                    byte-granular store-to-load forward)
//   gen_rtl/lsu/rtl/aq_lsu_rdl.v   (writes refill/store/dirty data into the
//                                    DCache SRAMs)
// References: design doc S2.1/S2.2/S2.3.1/S2.3.3/S4.1/S4.3, contract 2
// (MMU protocol), contract 3 (misalignment: trap-only, ignores MXSTATUS.mm),
// contract 4 (STB: clone as-is, no RTU-commit gate, but DOES need the
// local flush/cancel interlock), contract 9 (DCache geometry, no alias
// bank), contract 11 (victim writeback in scope), LSU note A2/A4/A5/A6/A7.
//
// SEAM NOTES:
//  * `idu_lsu_ex1_dp_sel` is spelled per the design doc's/plan's own
//    repeated usage (design doc S4.1, plan 1.2's LSU.v/IDU.v bullets),
//    matching IDU.v's output of the same name -- see IDU.v's header for
//    the note that `aq_idu_id_ctrl.v:634`'s real donor name is
//    `idu_lsu_ex1_sel` (no "dp"); Task 5/6 reconcile which spelling stands.
//  * `lsu_idu_full` (this module's single point-to-point stall signal to
//    IDU, contract 8) is CONFIRMED directly from `aq_idu_id_ctrl.v:634`'s
//    consumer-side reference, not guessed.
//  * `lsu_rtu_*`/`lsu_mmu_*`/`mmu_lsu_*` names/widths match RTU.v's/
//    MMU.v's own port groups exactly (this task's own skeletons) -- see
//    RTU.v's header for the flagged open item on whether the donor's 3
//    distinct LSU->RTU writeback paths collapse to the 2 this skeleton
//    pins, or need a 3rd port later.
//  * DCache.v is instantiated INSIDE this module (not at RVProc.v's top
//    level, see DCache.v's header) -- but Task 1 does not require actually
//    instantiating it yet (the task text: "freezing the seven new
//    modules' port lists does not require instantiating them anywhere
//    yet"), so this skeleton's body does not reference DCache.v at all.
//  * The AXI D-channel port group below is copied verbatim from
//    rtl/RVProc.v's existing outer port list (lines 90-127) -- the same
//    channel FetchSink.v currently drives with its tohost-only write FSM;
//    Task 7 moves the connection, not the shape.
//=============================================================================

import rvproc_pkg::*;

module LSU #(
    parameter DATA_WIDTH = 512,
    parameter ADDR_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IDU -> LSU : EX1 dispatch, LSU's slice of id_ex1_t (matches IDU.v's
    // output group exactly).
    //=========================================================================
    input  wire                     idu_lsu_ex1_dp_sel,
    input  wire [FUNC_WIDTH-1:0]    idu_lsu_ex1_func,
    input  wire [63:0]              idu_lsu_ex1_src0_data, // base addr reg
    input  wire                     idu_lsu_ex1_src0_ready,
    input  wire [63:0]              idu_lsu_ex1_src1_data, // offset imm
    input  wire                     idu_lsu_ex1_src1_ready,
    input  wire [63:0]              idu_lsu_ex1_src2_data, // store data
    input  wire                     idu_lsu_ex1_src2_ready,
    input  wire [GPR_IDX_WIDTH-1:0] idu_lsu_ex1_dst0_reg,  // load dest

    //=========================================================================
    // LSU -> IDU : the single EX1 issue-gate stall signal (contract 8;
    // confirmed name, aq_idu_id_ctrl.v:634).
    //=========================================================================
    output wire                     lsu_idu_full,

    //=========================================================================
    // LSU -> RTU : lsu_rtu_t (design doc S4.2) -- matches RTU.v's input
    // group exactly.
    //=========================================================================
    output wire                     lsu_rtu_ex1_cmplt,
    output wire                     lsu_rtu_ex1_cmplt_dp,
    output wire [63:0]              lsu_rtu_wb_data,
    output wire [GPR_IDX_WIDTH-1:0] lsu_rtu_wb_preg,
    output wire                     lsu_rtu_wb_vld,
    output wire [63:0]              lsu_rtu_ex2_data,
    output wire                     lsu_rtu_ex2_data_vld,
    output wire                     lsu_rtu_expt_vld,
    output wire [4:0]               lsu_rtu_expt_vec,
    output wire [63:0]              lsu_rtu_tval,
    output wire                     lsu_rtu_async_expt_vld,
    output wire                     lsu_rtu_async_ld_inst,

    //=========================================================================
    // RTU -> LSU : "point of no return" acks (RTU note S6) -- also the
    // signals that gate the STB-create-vs-flush interlock contract 4
    // requires (verified cycle-by-cycle in Task 6, not assumed here).
    //=========================================================================
    input  wire                     rtu_lsu_expt_ack,
    input  wire                     rtu_lsu_expt_exit,

    //=========================================================================
    // LSU -> MMU / MMU -> LSU : the DTLB request/response (contract 2),
    // matches MMU.v's DTLB port group exactly.
    //=========================================================================
    output wire [MMU_VA_WIDTH-1:0]  lsu_mmu_va,
    output wire                     lsu_mmu_va_vld,
    output wire [1:0]               lsu_mmu_priv_mode,
    output wire                     lsu_mmu_st_inst,
    input  wire [MMU_PA_WIDTH-1:0]  mmu_lsu_pa,
    input  wire                     mmu_lsu_pa_vld,
    input  wire                     mmu_lsu_ca,
    input  wire                     mmu_lsu_so,
    input  wire                     mmu_lsu_buf,
    input  wire                     mmu_lsu_sec,
    input  wire                     mmu_lsu_sh,
    input  wire                     mmu_lsu_page_fault,
    input  wire                     mmu_lsu_access_fault,

    //=========================================================================
    // CSR -> LSU : MHCR.de/wa + MXSTATUS.mm (matches CSR.v's output group
    // exactly).
    //=========================================================================
    input  wire                     cp0_lsu_dcache_en,
    input  wire                     cp0_lsu_mm,
    input  wire                     cp0_lsu_wa,

    //=========================================================================
    // AXI Master Interface - DCache (ch[1]) -- copied verbatim from
    // rtl/RVProc.v's existing outer port list (lines 90-127); moves here
    // from FetchSink.v's old tohost-only write FSM (design doc S2.1).
    //=========================================================================
    output wire                     axi_d_awvalid,
    input  wire                     axi_d_awready,
    output wire [ADDR_WIDTH-1:0]    axi_d_awaddr,
    output wire [7:0]               axi_d_awlen,
    output wire [2:0]               axi_d_awsize,
    output wire [1:0]               axi_d_awburst,
    output wire [3:0]               axi_d_awcache,
    output wire [2:0]               axi_d_awprot,

    output wire                     axi_d_wvalid,
    input  wire                     axi_d_wready,
    output wire [DATA_WIDTH-1:0]    axi_d_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_d_wstrb,
    output wire                     axi_d_wlast,

    input  wire                     axi_d_bvalid,
    output wire                     axi_d_bready,
    input  wire [1:0]               axi_d_bresp,

    output wire                     axi_d_arvalid,
    input  wire                     axi_d_arready,
    output wire [ADDR_WIDTH-1:0]    axi_d_araddr,
    output wire [7:0]               axi_d_arlen,
    output wire [2:0]               axi_d_arsize,
    output wire [1:0]               axi_d_arburst,
    output wire [3:0]               axi_d_arcache,
    output wire [2:0]               axi_d_arprot,

    input  wire                     axi_d_rvalid,
    output wire                     axi_d_rready,
    input  wire [DATA_WIDTH-1:0]    axi_d_rdata,
    input  wire [1:0]               axi_d_rresp,
    input  wire                     axi_d_rlast
);

    //=========================================================================
    // SKELETON BODY (plan Task 6 replaces it): every output inactive/0 --
    // LSU never issues, never completes, never drives the AXI bus.
    //=========================================================================
    assign lsu_idu_full = 1'b0;

    assign lsu_rtu_ex1_cmplt      = 1'b0;
    assign lsu_rtu_ex1_cmplt_dp   = 1'b0;
    assign lsu_rtu_wb_data        = 64'd0;
    assign lsu_rtu_wb_preg        = {GPR_IDX_WIDTH{1'b0}};
    assign lsu_rtu_wb_vld         = 1'b0;
    assign lsu_rtu_ex2_data       = 64'd0;
    assign lsu_rtu_ex2_data_vld   = 1'b0;
    assign lsu_rtu_expt_vld       = 1'b0;
    assign lsu_rtu_expt_vec       = 5'd0;
    assign lsu_rtu_tval           = 64'd0;
    assign lsu_rtu_async_expt_vld = 1'b0;
    assign lsu_rtu_async_ld_inst  = 1'b0;

    assign lsu_mmu_va       = {MMU_VA_WIDTH{1'b0}};
    assign lsu_mmu_va_vld   = 1'b0;
    assign lsu_mmu_priv_mode= 2'd0;
    assign lsu_mmu_st_inst  = 1'b0;

    assign axi_d_awvalid = 1'b0;
    assign axi_d_awaddr  = {ADDR_WIDTH{1'b0}};
    assign axi_d_awlen   = 8'd0;
    assign axi_d_awsize  = 3'd6;    // 64 bytes
    assign axi_d_awburst = 2'b01;   // INCR
    assign axi_d_awcache = 4'd0;
    assign axi_d_awprot  = 3'd0;
    assign axi_d_wvalid  = 1'b0;
    assign axi_d_wdata   = {DATA_WIDTH{1'b0}};
    assign axi_d_wstrb   = {DATA_WIDTH/8{1'b0}};
    assign axi_d_wlast   = 1'b0;
    assign axi_d_bready  = 1'b1;
    assign axi_d_arvalid = 1'b0;
    assign axi_d_araddr  = {ADDR_WIDTH{1'b0}};
    assign axi_d_arlen   = 8'd0;    // single beat (matches ICache's own
                                     // deviation 1, contract 17)
    assign axi_d_arsize  = 3'd6;
    assign axi_d_arburst = 2'b01;
    assign axi_d_arcache = 4'd0;
    assign axi_d_arprot  = 3'd0;
    assign axi_d_rready  = 1'b1;

endmodule
