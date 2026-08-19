//=============================================================================
// FetchSink.v - M1 SCAFFOLDING: fake BJU + fake RTU + CP0 stand-in +
//               harness config bank
//=============================================================================
// C906 files covered (the consumer side this fake stands in for):
//   gen_rtl/idu/rtl/aq_idu_top.v (ports)   (the single-instruction IFU->IDU
//                                           interface, IFU notes S2 item 6)
//   gen_rtl/iu/rtl/aq_iu_bju.v             (resolve: chgflw / bht / RAS
//                                           update signals -- OUT of the
//                                           extraction notes' scope; only
//                                           the IFU's own consumer-side
//                                           ports are confirmed, see IFU.v
//                                           and BPU.v headers)
//   gen_rtl/rtu/rtl/aq_rtu_*.v              (flush; C906 has no per-slot
//                                           retire bus to model here since
//                                           only one instruction commits per
//                                           cycle -- simpler than C910's
//                                           3-slot retire, design doc S4.1)
//   rtl/TestMaster.v                       (M0's proven D-side AXI write
//                                           FSM, lifted verbatim in plan
//                                           Task 4.1)
// References: design doc S4.1 (the FULL behavioural contract -- implement it
// from there, verbatim, in Task 4.1), plan "Global contracts" (JR_TARGET,
// direction rule, shadow call stack, harness config mechanism, RAS-faithful
// grading path). Body arrives in plan Task 4.1.
//
// This module is DELETED at M2: the IDU/IU/RTU take over its interfaces
// unchanged. Nothing here models C906 microarchitecture -- it is an oracle.
//
// CP0 STAND-IN: in the real RTL, `cp0_ifu_*` chicken bits and invalidate
// requests are driven by CP0 directly to ICache.v/BPU.v (IFU notes S5.1;
// BPU notes S8) -- NOT through IDU/IU/RTU. Since there is no CP0 module in
// M1, FetchSink hosts that config bank too (plan "Global contracts": harness
// config mechanism) and RVProc.v fans its outputs straight to ICache.v/
// BPU.v/IFU.v, mirroring the real direct-fan-out topology.
//=============================================================================

import rvproc_pkg::*;

module FetchSink #(
    parameter DATA_WIDTH  = 512,
    parameter ADDR_WIDTH  = 64,
    // tohost line; must agree with test/m1/common.ld and the testbench's ELF
    // symbol lookup (docs/08-verification.md M1 section, Task 5).
    parameter [63:0] TOHOST_ADDR  = ADDR_TOHOST,
    parameter [63:0] RESET_VECTOR = 64'h0000_0000_8000_0000
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IDU side: consume the IFU's single-instruction delivery (IFU notes
    // S2 item 6, S5.2)
    //=========================================================================
    input  wire [31:0]              ifu_idu_id_inst,
    input  wire                     ifu_idu_id_inst_vld,
    input  wire [1:0]               ifu_idu_id_bht_pred,
    output wire                     idu_ifu_id_stall,

    //=========================================================================
    // Fake BJU (design doc S4.1): registered resolve, direction rule
    // ^pc[7:4], decoded B/CB/J/CJ immediates, shadow call stack, JR_TARGET
    // formula. `iu_ifu_br_vld` fires for EVERY resolved conditional branch;
    // the mispredict-only signals gate PCGEN's redirect and RAS's pointer
    // snap-back.
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
    input  wire                     ifu_iu_chgflw_vld,      // IFU forwards RTU's redirect
    input  wire [PC_WIDTH-1:0]      ifu_iu_chgflw_pc,

    //=========================================================================
    // Fake RTU (design doc S4.1): front-end flush only -- C906 delivers one
    // instruction/cycle, so there is no per-slot retire bus to drive here,
    // unlike C910's 3-slot rtughr/RAS-mirror retire fan-out.
    //=========================================================================
    output wire                     rtu_ifu_chgflw_vld,
    output wire [PC_WIDTH-1:0]      rtu_ifu_chgflw_pc,
    output wire                     rtu_ifu_flush_fe,

    //=========================================================================
    // CP0 stand-in: ICache-facing chicken bits + invalidate (see module
    // header). Ports mirror ICache.v's cp0_ifu_* group exactly.
    //=========================================================================
    output wire                     cp0_ifu_icache_en,
    output wire                     cp0_ifu_iwpe,
    output wire                     cp0_ifu_icache_pref_en,
    output wire [63:0]              cp0_ifu_icache_inv_addr,
    output wire                     cp0_ifu_icache_inv_req,
    output wire [1:0]               cp0_ifu_icache_inv_type,
    input  wire                     ifu_cp0_icache_inv_done,

    //=========================================================================
    // CP0 stand-in: BPU-facing chicken bits + invalidate. Ports mirror
    // BPU.v's cp0_ifu_* group exactly. rung bits (--m1-rung=<1..4>, plan
    // "Global contracts") select which of these three are asserted.
    //=========================================================================
    output wire                     cp0_ifu_bht_en,
    output wire                     cp0_ifu_btb_en,
    output wire                     cp0_ifu_ras_en,
    output wire                     cp0_ifu_bht_inv,
    output wire                     cp0_ifu_btb_clr,
    input  wire                     bht_cp0_inv_done,       // PLACEHOLDER, see BPU.v header

    //=========================================================================
    // CP0 stand-in: boot / reset vector (IFU.v's only CP0-shaped input)
    //=========================================================================
    output wire [PC_WIDTH-1:0]      cp0_xx_mrvbr,

    //=========================================================================
    // D-side AXI write master (ch[1]) - tohost reporting only.
    // Port group copied from TestMaster.v's axi_d_* list; plan Task 4.1
    // lifts its write FSM verbatim (concurrent AW+W, BVALID watched every
    // cycle of the write state).
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
    // Harness config bank  (plan "Global contracts": harness config mechanism)
    //=========================================================================
    // THESE REGISTERS HAVE NO RTL DRIVER ON PURPOSE. RVProcTest.cpp pokes
    // them through the verisim.h signal paths after dut.init() and before
    // the first step (rung selection, stall mode, instruction budget), and
    // PULSES the cfg_*_inv/cfg_btb_clr bits mid-test for --inv-test.
    // The simulator zero-initializes them, so an un-poked run is rung 1
    // (every predictor off, ICache off) -- matching the ladder's floor.
    // Marked verilator public so the flattened model keeps them addressable.
    reg        cfg_icache_en       /* verilator public */;
    reg        cfg_iwpe            /* verilator public */;  // stays 0 in M1
    reg        cfg_icache_pref_en  /* verilator public */;
    reg        cfg_bht_en          /* verilator public */;  // ladder rung 4
    reg        cfg_btb_en          /* verilator public */;  // ladder rung 3
    reg        cfg_ras_en          /* verilator public */;  // ladder rung 2
    reg        cfg_icache_inv      /* verilator public */;  // --inv-test pulses
    reg        cfg_bht_inv         /* verilator public */;
    reg        cfg_btb_clr         /* verilator public */;
    reg        cfg_sink_stall      /* verilator public */;  // --sink-stall
    reg [31:0] cfg_max_insts       /* verilator public */;  // --max-insts

    assign cp0_ifu_icache_en      = cfg_icache_en;
    assign cp0_ifu_iwpe           = cfg_iwpe;
    assign cp0_ifu_icache_pref_en = cfg_icache_pref_en;
    assign cp0_ifu_icache_inv_addr= 64'd0;
    assign cp0_ifu_icache_inv_req = cfg_icache_inv;
    assign cp0_ifu_icache_inv_type= 2'd0;   // INV_ALL only in M1

    assign cp0_ifu_bht_en = cfg_bht_en;
    assign cp0_ifu_btb_en = cfg_btb_en;
    assign cp0_ifu_ras_en = cfg_ras_en;
    assign cp0_ifu_bht_inv= cfg_bht_inv;
    assign cp0_ifu_btb_clr= cfg_btb_clr;

    assign cp0_xx_mrvbr = RESET_VECTOR[PC_WIDTH-1:0];

    //=========================================================================
    // SKELETON BODY (plan Task 4.1 replaces it): consume nothing, resolve
    // nothing, report nothing. Every output is the inactive value.
    //=========================================================================
    assign idu_ifu_id_stall = 1'b0;

    assign iu_ifu_tar_pc_vld  = 1'b0;
    assign iu_ifu_tar_pc      = 64'd0;
    assign iu_ifu_pc_mispred  = 1'b0;
    assign iu_ifu_bht_mispred = 1'b0;
    assign iu_ifu_br_vld      = 1'b0;
    assign iu_ifu_bht_taken   = 1'b0;
    assign iu_ifu_bht_pred    = 2'd0;
    assign iu_ifu_link_vld    = 1'b0;
    assign iu_ifu_ret_vld     = 1'b0;

    assign rtu_ifu_chgflw_vld = 1'b0;
    assign rtu_ifu_chgflw_pc  = {PC_WIDTH{1'b0}};
    assign rtu_ifu_flush_fe   = 1'b0;

    assign axi_d_awvalid = 1'b0;
    assign axi_d_awaddr  = {ADDR_WIDTH{1'b0}};
    assign axi_d_awlen   = 8'd0;
    assign axi_d_awsize  = 3'd6;            // one full 64-byte beat
    assign axi_d_awburst = 2'b01;
    assign axi_d_awcache = 4'd0;
    assign axi_d_awprot  = 3'd0;
    assign axi_d_wvalid  = 1'b0;
    assign axi_d_wdata   = {DATA_WIDTH{1'b0}};
    assign axi_d_wstrb   = {DATA_WIDTH/8{1'b0}};
    assign axi_d_wlast   = 1'b0;
    assign axi_d_bready  = 1'b1;
    assign axi_d_arvalid = 1'b0;
    assign axi_d_araddr  = {ADDR_WIDTH{1'b0}};
    assign axi_d_arlen   = 8'd0;
    assign axi_d_arsize  = 3'd6;
    assign axi_d_arburst = 2'b01;
    assign axi_d_arcache = 4'd0;
    assign axi_d_arprot  = 3'd0;
    assign axi_d_rready  = 1'b1;

endmodule
