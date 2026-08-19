//=============================================================================
// RVProc.v - rv906 core shell v0.1  (M1 SKELETON: outer ports frozen)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_ifu_top.v   (the IFU/ICache/BPU glue, 863 lines of it)
//   gen_rtl/cpu/rtl/aq_cpu_*.v     (core-level instantiation, MMU hookup)
// References: design doc S3 (file organization: "IFU + ICache + BPU + MMU
// stub + FetchSink, drop-in replacement for TestMaster"), umbrella spec S6.2
// rule 1 ("top file is the table of contents").
//
// The outer port list is TestMaster.v's, VERBATIM (plan Task 1.3), so Task
// 4.2 turns RVProcAXI.v's core instance into RVProc with a one-word change.
// Until that swap, TestMaster.v and this file coexist and Verilator reports
// MULTITOP when both are on the same --lint-only invocation without a
// --top-module pin.
//
// M1 stage of the core: there is no decode, no execute and no retire yet.
// FetchSink stands in for IDU+IU+RTU+CP0 (design doc S4.1) and hosts the
// harness config bank; the MMU is the zero-latency bare-physical-mapping
// stub below (design doc S2.1), replaced by the real MMU in M4.
//=============================================================================

import rvproc_pkg::*;

module RVProc #(
    parameter XLEN = 64,
    parameter ILEN = 32,
    parameter RESET_VECTOR = 64'h80000000,
    parameter DATA_WIDTH = 512,
    parameter ADDR_WIDTH = 64
)(
    input  wire                 clk,
    input  wire                 rst_n,

    //=========================================================================
    // AXI Master Interface - ICache (ch[0])
    //=========================================================================
    // Write Address Channel (not used by ICache)
    output wire                     axi_i_awvalid,
    input  wire                     axi_i_awready,
    output wire [ADDR_WIDTH-1:0]    axi_i_awaddr,
    output wire [7:0]               axi_i_awlen,
    output wire [2:0]               axi_i_awsize,
    output wire [1:0]               axi_i_awburst,
    output wire [3:0]               axi_i_awcache,
    output wire [2:0]               axi_i_awprot,

    // Write Data Channel
    output wire                     axi_i_wvalid,
    input  wire                     axi_i_wready,
    output wire [DATA_WIDTH-1:0]    axi_i_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_i_wstrb,
    output wire                     axi_i_wlast,

    // Write Response Channel
    input  wire                     axi_i_bvalid,
    output wire                     axi_i_bready,
    input  wire [1:0]               axi_i_bresp,

    // Read Address Channel
    output wire                     axi_i_arvalid,
    input  wire                     axi_i_arready,
    output wire [ADDR_WIDTH-1:0]    axi_i_araddr,
    output wire [7:0]               axi_i_arlen,
    output wire [2:0]               axi_i_arsize,
    output wire [1:0]               axi_i_arburst,
    output wire [3:0]               axi_i_arcache,
    output wire [2:0]               axi_i_arprot,

    // Read Data Channel
    input  wire                     axi_i_rvalid,
    output wire                     axi_i_rready,
    input  wire [DATA_WIDTH-1:0]    axi_i_rdata,
    input  wire [1:0]               axi_i_rresp,
    input  wire                     axi_i_rlast,

    //=========================================================================
    // AXI Master Interface - DCache (ch[1])
    //=========================================================================
    // Write Address Channel
    output wire                     axi_d_awvalid,
    input  wire                     axi_d_awready,
    output wire [ADDR_WIDTH-1:0]    axi_d_awaddr,
    output wire [7:0]               axi_d_awlen,
    output wire [2:0]               axi_d_awsize,
    output wire [1:0]               axi_d_awburst,
    output wire [3:0]               axi_d_awcache,
    output wire [2:0]               axi_d_awprot,

    // Write Data Channel
    output wire                     axi_d_wvalid,
    input  wire                     axi_d_wready,
    output wire [DATA_WIDTH-1:0]    axi_d_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_d_wstrb,
    output wire                     axi_d_wlast,

    // Write Response Channel
    input  wire                     axi_d_bvalid,
    output wire                     axi_d_bready,
    input  wire [1:0]               axi_d_bresp,

    // Read Address Channel
    output wire                     axi_d_arvalid,
    input  wire                     axi_d_arready,
    output wire [ADDR_WIDTH-1:0]    axi_d_araddr,
    output wire [7:0]               axi_d_arlen,
    output wire [2:0]               axi_d_arsize,
    output wire [1:0]               axi_d_arburst,
    output wire [3:0]               axi_d_arcache,
    output wire [2:0]               axi_d_arprot,

    // Read Data Channel
    input  wire                     axi_d_rvalid,
    output wire                     axi_d_rready,
    input  wire [DATA_WIDTH-1:0]    axi_d_rdata,
    input  wire [1:0]               axi_d_rresp,
    input  wire                     axi_d_rlast,

    //=========================================================================
    // Interrupt Inputs (from CLINT and PLIC)
    //=========================================================================
    input  wire                 mtip,               // Machine Timer Interrupt Pending
    input  wire                 msip,               // Machine Software Interrupt Pending
    input  wire                 meip,               // Machine External Interrupt Pending

    //=========================================================================
    // Control/Status
    //=========================================================================
    output wire                 quitted
);

    // mtip/msip/meip are unconnected until the CSR file and the exception
    // vector path exist (M2+); M1 has no interrupt-taking machinery.

    //=========================================================================
    // IFU <-> ICache seam (see ICache.v's header for the port rationale)
    //=========================================================================
    wire                     cp0_ifu_icache_en;
    wire                     cp0_ifu_iwpe;
    wire                     cp0_ifu_icache_pref_en;
    wire [63:0]              cp0_ifu_icache_inv_addr;
    wire                     cp0_ifu_icache_inv_req;
    wire [1:0]               cp0_ifu_icache_inv_type;
    wire                     ifu_cp0_icache_inv_done;

    wire [63:0]              pcgen_icache_va;
    wire [33:0]              pcgen_icache_seq_tag;
    wire                     pcgen_icache_chgflw_vld;
    wire                     ctrl_icache_req_vld;
    wire                     ctrl_icache_abort;

    wire                     icache_pcgen_grant;
    wire [39:0]              icache_pcgen_addr;
    wire                     icache_pcgen_inst_vld;
    wire                     icache_ctrl_stall;

    wire [31:0]              icache_ipack_inst;
    wire                     icache_ipack_inst_vld;
    wire                     icache_ipack_acc_err;
    wire                     icache_ipack_pgflt;
    wire                     icache_ipack_unalign;

    //=========================================================================
    // ICache <-> MMU stub seam (design doc S2.1: bare physical mapping, zero
    // latency; M4 replaces this block with the real MMU without touching
    // ICache.v's port list)
    //=========================================================================
    wire                     ifu_mmu_abort;
    wire [MMU_VA_WIDTH-1:0]  ifu_mmu_va;
    wire                     ifu_mmu_va_vld;
    wire                     mmu_ifu_access_fault;
    wire [MMU_PA_WIDTH-1:0]  mmu_ifu_pa;
    wire                     mmu_ifu_pa_vld;
    wire [MMU_PROT_WIDTH-1:0] mmu_ifu_prot;

    // Bare physical mapping: PA page number = the low MMU_PA_WIDTH bits of
    // the VA page number, same cycle (no TLB, no page walk). PLACEHOLDER:
    // `mmu_ifu_prot`'s exact encoding is not confirmed from refs/openc906 in
    // this task's scope (M4 pins the real ITLB semantics) -- tied to "every
    // bit set" here as a permissive stand-in, not a claim about the real
    // encoding's bit meanings.
    assign mmu_ifu_pa           = ifu_mmu_va[MMU_PA_WIDTH-1:0];
    assign mmu_ifu_pa_vld       = ifu_mmu_va_vld;
    assign mmu_ifu_access_fault = 1'b0;
    assign mmu_ifu_prot         = {MMU_PROT_WIDTH{1'b1}};

    //=========================================================================
    // IFU <-> BPU seam (see BPU.v's header for the port rationale)
    //=========================================================================
    wire [PC_WIDTH-1:0]      pcgen_btb_ifpc;
    wire [PC_WIDTH-1:0]      pred_idpc;
    wire [31:0]              ipack_pred_inst0;
    wire                     ipack_pred_inst0_vld;
    wire [15:0]              ipack_pred_inst1;
    wire                     ipack_pred_inst1_vld;
    wire                     ipack_pred_h0_create;
    wire                     ipack_pred_h0_vld;
    wire                     ipack_pred_unalign;

    wire                     pred_pcgen_chgflw_vld;
    wire [PC_WIDTH-1:0]      pred_pcgen_chgflw_pc;
    wire                     pred_pcgen_curflw_vld;
    wire [PC_WIDTH-1:0]      pred_pcgen_curflw_pc;
    wire                     pred_ctrl_stall;
    wire                     pred_ipack_ret_stall;
    wire                     pred_ipack_delay_stall;
    wire                     pred_ipack_mask;
    wire                     pred_ibuf_chgflw_vld0;
    wire [1:0]               pred_ibuf_br_taken0;
    wire [1:0]               pred_ibuf_br_taken1;

    wire                     cp0_ifu_bht_en;
    wire                     cp0_ifu_btb_en;
    wire                     cp0_ifu_ras_en;
    wire                     cp0_ifu_bht_inv;
    wire                     cp0_ifu_btb_clr;
    wire                     bht_cp0_inv_done;

    //=========================================================================
    // IFU <-> IDU single-instruction handoff (frozen for M2)
    //=========================================================================
    wire [31:0]              ifu_idu_id_inst;
    wire                     ifu_idu_id_inst_vld;
    wire [1:0]               ifu_idu_id_bht_pred;
    wire                     idu_ifu_id_stall;

    //=========================================================================
    // IFU/BPU <-> fake BJU (IU) + fake RTU + CP0 stand-in (FetchSink)
    //=========================================================================
    wire                     iu_ifu_tar_pc_vld;
    wire [63:0]              iu_ifu_tar_pc;
    wire                     iu_ifu_pc_mispred;
    wire                     iu_ifu_bht_mispred;
    wire                     iu_ifu_br_vld;
    wire                     iu_ifu_bht_taken;
    wire [1:0]               iu_ifu_bht_pred;
    wire                     iu_ifu_link_vld;
    wire                     iu_ifu_ret_vld;
    wire                     ifu_iu_chgflw_vld;
    wire [PC_WIDTH-1:0]      ifu_iu_chgflw_pc;

    wire                     rtu_ifu_chgflw_vld;
    wire [PC_WIDTH-1:0]      rtu_ifu_chgflw_pc;
    wire                     rtu_ifu_flush_fe;

    wire [PC_WIDTH-1:0]      cp0_xx_mrvbr;

    //=========================================================================
    // ICache instance
    //=========================================================================
    ICache #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) u_icache (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .cp0_ifu_icache_en       (cp0_ifu_icache_en),
        .cp0_ifu_iwpe            (cp0_ifu_iwpe),
        .cp0_ifu_icache_pref_en  (cp0_ifu_icache_pref_en),
        .cp0_ifu_icache_inv_addr (cp0_ifu_icache_inv_addr),
        .cp0_ifu_icache_inv_req  (cp0_ifu_icache_inv_req),
        .cp0_ifu_icache_inv_type (cp0_ifu_icache_inv_type),
        .ifu_cp0_icache_inv_done (ifu_cp0_icache_inv_done),

        .pcgen_icache_va         (pcgen_icache_va),
        .pcgen_icache_seq_tag    (pcgen_icache_seq_tag),
        .pcgen_icache_chgflw_vld (pcgen_icache_chgflw_vld),
        .ctrl_icache_req_vld     (ctrl_icache_req_vld),
        .ctrl_icache_abort       (ctrl_icache_abort),

        .icache_pcgen_grant      (icache_pcgen_grant),
        .icache_pcgen_addr       (icache_pcgen_addr),
        .icache_pcgen_inst_vld   (icache_pcgen_inst_vld),
        .icache_ctrl_stall       (icache_ctrl_stall),

        .icache_ipack_inst       (icache_ipack_inst),
        .icache_ipack_inst_vld   (icache_ipack_inst_vld),
        .icache_ipack_acc_err    (icache_ipack_acc_err),
        .icache_ipack_pgflt      (icache_ipack_pgflt),
        .icache_ipack_unalign    (icache_ipack_unalign),

        .ifu_mmu_abort           (ifu_mmu_abort),
        .ifu_mmu_va              (ifu_mmu_va),
        .ifu_mmu_va_vld          (ifu_mmu_va_vld),
        .mmu_ifu_access_fault    (mmu_ifu_access_fault),
        .mmu_ifu_pa              (mmu_ifu_pa),
        .mmu_ifu_pa_vld          (mmu_ifu_pa_vld),
        .mmu_ifu_prot            (mmu_ifu_prot),

        .axi_i_arvalid           (axi_i_arvalid),
        .axi_i_arready           (axi_i_arready),
        .axi_i_araddr            (axi_i_araddr),
        .axi_i_arlen             (axi_i_arlen),
        .axi_i_arsize            (axi_i_arsize),
        .axi_i_arburst           (axi_i_arburst),
        .axi_i_arcache           (axi_i_arcache),
        .axi_i_arprot            (axi_i_arprot),
        .axi_i_rvalid            (axi_i_rvalid),
        .axi_i_rready            (axi_i_rready),
        .axi_i_rdata             (axi_i_rdata),
        .axi_i_rresp             (axi_i_rresp),
        .axi_i_rlast             (axi_i_rlast)
    );

    //=========================================================================
    // IFU instance
    //=========================================================================
    IFU u_ifu (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .ifu_idu_id_inst         (ifu_idu_id_inst),
        .ifu_idu_id_inst_vld     (ifu_idu_id_inst_vld),
        .ifu_idu_id_bht_pred     (ifu_idu_id_bht_pred),
        .idu_ifu_id_stall        (idu_ifu_id_stall),

        .pcgen_icache_va         (pcgen_icache_va),
        .pcgen_icache_seq_tag    (pcgen_icache_seq_tag),
        .pcgen_icache_chgflw_vld (pcgen_icache_chgflw_vld),
        .ctrl_icache_req_vld     (ctrl_icache_req_vld),
        .ctrl_icache_abort       (ctrl_icache_abort),

        .icache_pcgen_grant      (icache_pcgen_grant),
        .icache_pcgen_addr       (icache_pcgen_addr),
        .icache_pcgen_inst_vld   (icache_pcgen_inst_vld),
        .icache_ctrl_stall       (icache_ctrl_stall),
        .icache_ipack_inst       (icache_ipack_inst),
        .icache_ipack_inst_vld   (icache_ipack_inst_vld),
        .icache_ipack_acc_err    (icache_ipack_acc_err),
        .icache_ipack_pgflt      (icache_ipack_pgflt),
        .icache_ipack_unalign    (icache_ipack_unalign),

        .pcgen_btb_ifpc          (pcgen_btb_ifpc),
        .pred_idpc               (pred_idpc),
        .ipack_pred_inst0        (ipack_pred_inst0),
        .ipack_pred_inst0_vld    (ipack_pred_inst0_vld),
        .ipack_pred_inst1        (ipack_pred_inst1),
        .ipack_pred_inst1_vld    (ipack_pred_inst1_vld),
        .ipack_pred_h0_create    (ipack_pred_h0_create),
        .ipack_pred_h0_vld       (ipack_pred_h0_vld),
        .ipack_pred_unalign      (ipack_pred_unalign),

        .pred_pcgen_chgflw_vld   (pred_pcgen_chgflw_vld),
        .pred_pcgen_chgflw_pc    (pred_pcgen_chgflw_pc),
        .pred_pcgen_curflw_vld   (pred_pcgen_curflw_vld),
        .pred_pcgen_curflw_pc    (pred_pcgen_curflw_pc),
        .pred_ctrl_stall         (pred_ctrl_stall),
        .pred_ipack_ret_stall    (pred_ipack_ret_stall),
        .pred_ipack_delay_stall  (pred_ipack_delay_stall),
        .pred_ipack_mask         (pred_ipack_mask),
        .pred_ibuf_chgflw_vld0   (pred_ibuf_chgflw_vld0),
        .pred_ibuf_br_taken0     (pred_ibuf_br_taken0),
        .pred_ibuf_br_taken1     (pred_ibuf_br_taken1),

        .iu_ifu_tar_pc_vld       (iu_ifu_tar_pc_vld),
        .iu_ifu_tar_pc           (iu_ifu_tar_pc),
        .iu_ifu_pc_mispred       (iu_ifu_pc_mispred),
        .ifu_iu_chgflw_vld       (ifu_iu_chgflw_vld),
        .ifu_iu_chgflw_pc        (ifu_iu_chgflw_pc),

        .rtu_ifu_chgflw_vld      (rtu_ifu_chgflw_vld),
        .rtu_ifu_chgflw_pc       (rtu_ifu_chgflw_pc),
        .rtu_ifu_flush_fe        (rtu_ifu_flush_fe),

        .cp0_xx_mrvbr            (cp0_xx_mrvbr)
    );

    //=========================================================================
    // BPU instance
    //=========================================================================
    BPU u_bpu (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .cp0_ifu_bht_en          (cp0_ifu_bht_en),
        .cp0_ifu_btb_en          (cp0_ifu_btb_en),
        .cp0_ifu_ras_en          (cp0_ifu_ras_en),
        .cp0_ifu_bht_inv         (cp0_ifu_bht_inv),
        .cp0_ifu_btb_clr         (cp0_ifu_btb_clr),
        .bht_cp0_inv_done        (bht_cp0_inv_done),

        .pcgen_btb_ifpc          (pcgen_btb_ifpc),
        .pred_idpc               (pred_idpc),
        .ipack_pred_inst0        (ipack_pred_inst0),
        .ipack_pred_inst0_vld    (ipack_pred_inst0_vld),
        .ipack_pred_inst1        (ipack_pred_inst1),
        .ipack_pred_inst1_vld    (ipack_pred_inst1_vld),
        .ipack_pred_h0_create    (ipack_pred_h0_create),
        .ipack_pred_h0_vld       (ipack_pred_h0_vld),
        .ipack_pred_unalign      (ipack_pred_unalign),

        .pred_pcgen_chgflw_vld   (pred_pcgen_chgflw_vld),
        .pred_pcgen_chgflw_pc    (pred_pcgen_chgflw_pc),
        .pred_pcgen_curflw_vld   (pred_pcgen_curflw_vld),
        .pred_pcgen_curflw_pc    (pred_pcgen_curflw_pc),
        .pred_ctrl_stall         (pred_ctrl_stall),
        .pred_ipack_ret_stall    (pred_ipack_ret_stall),
        .pred_ipack_delay_stall  (pred_ipack_delay_stall),
        .pred_ipack_mask         (pred_ipack_mask),
        .pred_ibuf_chgflw_vld0   (pred_ibuf_chgflw_vld0),
        .pred_ibuf_br_taken0     (pred_ibuf_br_taken0),
        .pred_ibuf_br_taken1     (pred_ibuf_br_taken1),

        .iu_ifu_br_vld           (iu_ifu_br_vld),
        .iu_ifu_bht_taken        (iu_ifu_bht_taken),
        .iu_ifu_bht_pred         (iu_ifu_bht_pred),
        .iu_ifu_bht_mispred      (iu_ifu_bht_mispred),
        .iu_ifu_pc_mispred       (iu_ifu_pc_mispred),
        .iu_ifu_link_vld         (iu_ifu_link_vld),
        .iu_ifu_ret_vld          (iu_ifu_ret_vld),

        .rtu_ifu_flush_fe        (rtu_ifu_flush_fe)
    );

    //=========================================================================
    // FetchSink instance: fake BJU + fake RTU + CP0 stand-in
    //=========================================================================
    FetchSink #(
        .DATA_WIDTH   (DATA_WIDTH),
        .ADDR_WIDTH   (ADDR_WIDTH),
        .TOHOST_ADDR  (ADDR_TOHOST),
        .RESET_VECTOR (RESET_VECTOR)
    ) u_fetchsink (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .ifu_idu_id_inst         (ifu_idu_id_inst),
        .ifu_idu_id_inst_vld     (ifu_idu_id_inst_vld),
        .ifu_idu_id_bht_pred     (ifu_idu_id_bht_pred),
        .idu_ifu_id_stall        (idu_ifu_id_stall),

        .iu_ifu_tar_pc_vld       (iu_ifu_tar_pc_vld),
        .iu_ifu_tar_pc           (iu_ifu_tar_pc),
        .iu_ifu_pc_mispred       (iu_ifu_pc_mispred),
        .iu_ifu_bht_mispred      (iu_ifu_bht_mispred),
        .iu_ifu_br_vld           (iu_ifu_br_vld),
        .iu_ifu_bht_taken        (iu_ifu_bht_taken),
        .iu_ifu_bht_pred         (iu_ifu_bht_pred),
        .iu_ifu_link_vld         (iu_ifu_link_vld),
        .iu_ifu_ret_vld          (iu_ifu_ret_vld),
        .ifu_iu_chgflw_vld       (ifu_iu_chgflw_vld),
        .ifu_iu_chgflw_pc        (ifu_iu_chgflw_pc),

        .rtu_ifu_chgflw_vld      (rtu_ifu_chgflw_vld),
        .rtu_ifu_chgflw_pc       (rtu_ifu_chgflw_pc),
        .rtu_ifu_flush_fe        (rtu_ifu_flush_fe),

        .cp0_ifu_icache_en       (cp0_ifu_icache_en),
        .cp0_ifu_iwpe            (cp0_ifu_iwpe),
        .cp0_ifu_icache_pref_en  (cp0_ifu_icache_pref_en),
        .cp0_ifu_icache_inv_addr (cp0_ifu_icache_inv_addr),
        .cp0_ifu_icache_inv_req  (cp0_ifu_icache_inv_req),
        .cp0_ifu_icache_inv_type (cp0_ifu_icache_inv_type),
        .ifu_cp0_icache_inv_done (ifu_cp0_icache_inv_done),

        .cp0_ifu_bht_en          (cp0_ifu_bht_en),
        .cp0_ifu_btb_en          (cp0_ifu_btb_en),
        .cp0_ifu_ras_en          (cp0_ifu_ras_en),
        .cp0_ifu_bht_inv         (cp0_ifu_bht_inv),
        .cp0_ifu_btb_clr         (cp0_ifu_btb_clr),
        .bht_cp0_inv_done        (bht_cp0_inv_done),

        .cp0_xx_mrvbr            (cp0_xx_mrvbr),

        .axi_d_awvalid           (axi_d_awvalid),
        .axi_d_awready           (axi_d_awready),
        .axi_d_awaddr            (axi_d_awaddr),
        .axi_d_awlen             (axi_d_awlen),
        .axi_d_awsize            (axi_d_awsize),
        .axi_d_awburst           (axi_d_awburst),
        .axi_d_awcache           (axi_d_awcache),
        .axi_d_awprot            (axi_d_awprot),
        .axi_d_wvalid            (axi_d_wvalid),
        .axi_d_wready            (axi_d_wready),
        .axi_d_wdata             (axi_d_wdata),
        .axi_d_wstrb             (axi_d_wstrb),
        .axi_d_wlast             (axi_d_wlast),
        .axi_d_bvalid            (axi_d_bvalid),
        .axi_d_bready            (axi_d_bready),
        .axi_d_bresp             (axi_d_bresp),
        .axi_d_arvalid           (axi_d_arvalid),
        .axi_d_arready           (axi_d_arready),
        .axi_d_araddr            (axi_d_araddr),
        .axi_d_arlen             (axi_d_arlen),
        .axi_d_arsize            (axi_d_arsize),
        .axi_d_arburst           (axi_d_arburst),
        .axi_d_arcache           (axi_d_arcache),
        .axi_d_arprot            (axi_d_arprot),
        .axi_d_rvalid            (axi_d_rvalid),
        .axi_d_rready            (axi_d_rready),
        .axi_d_rdata             (axi_d_rdata),
        .axi_d_rresp             (axi_d_rresp),
        .axi_d_rlast             (axi_d_rlast)
    );

    //=========================================================================
    // I-side write channel: the ICache is a read-only master, so the write
    // half of ch[0] is tied inactive here (as TestMaster did for the whole
    // I-side group). It stays unused until the DCache lands in M3.
    //=========================================================================
    assign axi_i_awvalid = 1'b0;
    assign axi_i_awaddr  = {ADDR_WIDTH{1'b0}};
    assign axi_i_awlen   = 8'd0;
    assign axi_i_awsize  = 3'd6;
    assign axi_i_awburst = 2'b01;
    assign axi_i_awcache = 4'd0;
    assign axi_i_awprot  = 3'd0;
    assign axi_i_wvalid  = 1'b0;
    assign axi_i_wdata   = {DATA_WIDTH{1'b0}};
    assign axi_i_wstrb   = {DATA_WIDTH/8{1'b0}};
    assign axi_i_wlast   = 1'b0;
    assign axi_i_bready  = 1'b1;

    // Completion is reported through tohost, exactly as in M0.
    assign quitted = 1'b0;

endmodule
