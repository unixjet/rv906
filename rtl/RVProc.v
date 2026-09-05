//=============================================================================
// RVProc.v - rv906 core shell  (M2: real body, TASK 7)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_ifu_top.v   (the IFU/ICache/BPU glue, 863 lines of it)
//   gen_rtl/cpu/rtl/aq_cpu_*.v     (core-level instantiation, MMU hookup)
// References: design doc S3 (file organization), S4.1 (the unit graph),
// umbrella spec S6.2 rule 1 ("top file is the table of contents").
//
// The outer port list is TestMaster.v's, VERBATIM (plan Task 1.3) and is
// UNCHANGED by Task 7 -- RVProcAXI.v's `u_core` instance (the one-word M1
// Task 4.2 swap) connects to it exactly as before.
//
// M2 stage of the core (this task): the full integer pipeline is wired for
// real -- IFU -> IDU -> IU -> LSU -> RTU, plus CSR.v (the MHCR-derived
// config bank that FetchSink's harness bank stood in for) and the real
// MMU.v instance that REPLACES the M1 inline bare-physical-mapping ITLB
// stub (design doc S2.1) by serving BOTH ICache's frozen ITLB port group
// and LSU's DTLB port group (contract 2). FetchSink.v is deleted: its
// fake BJU/RTU/CP0 outputs are now the real modules' outputs, and its
// tohost-only D-side write FSM is replaced by LSU.v's real D-side AXI
// master (the same ch[1] channel, same outer port names -- the connection
// moved, the shape did not).
//
// The IFU/ICache/BPU interconnect wires (seams 1-3 below) are byte-for-
// byte what M1 froze them to; Task 7 only changes which module instance
// drives the back-end-facing wires (u_fetchsink -> u_iu/u_rtu/u_csr) and
// adds the back-end seams (4-11) and the six new instances. The per-seam
// comment banners below are the file's table of contents: read them first.
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

    // mtip/msip/meip terminate in CSR.v's mip wiring (contract 7); the
    // instance connection is at the bottom of this file.

    //=========================================================================
    // IFU <-> ICache seam (see ICache.v's header for the port rationale).
    // The `cp0_ifu_icache_*`/`ifu_cp0_icache_inv_done` group at the top of
    // this seam is the I-side slice of the CSR config fan-out (banner 4
    // below) -- M2 drives it from u_csr's MHCR-derived wires; the wire
    // names are unchanged from M1, only the driver instance moved
    // (u_fetchsink -> u_csr).
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
    // MMU seam (TASK 7: the `MMU` instance below replaces M1's inline
    // bare-physical-mapping stub block that used to occupy this section).
    // One module, two independent port groups (contract 2):
    //   * the ITLB group, names/widths byte-for-byte ICache.v's
    //     frozen-since-M1 `ifu_mmu_*`/`mmu_ifu_*` ports;
    //   * the DTLB group, names/widths byte-for-byte LSU.v's
    //     `lsu_mmu_*`/`mmu_lsu_*` ports.
    // BOTH request inputs carry the PAGE NUMBER (VPN), not the byte VA --
    // `ifu_mmu_va = icache_rd_addr[63:12]` (ICache.v), `lsu_mmu_va =
    // ag_addr[63:12]` (LSU.v; donor aq_lsu_ag.v:1566), and each requester
    // reassembles the PA from the page-number response itself.
    //
    // I-side cacheability: the M1 stub decoded VA byte bit 31
    // (`ifu_mmu_va[19]`); MMU.v's PMA range check (contract 5,
    // {vpn,12'b0} in [0x8000_0000, 0xFFFF_FFFF]) now decides instead.
    // The two agree across all of M1's test space (< 2GB); the PMA check
    // is the more correct of the two (HANDOFF "known latent note").
    // `mmu_ifu_prot[4:0]` = {pgflt,supv,ca,ba,sec} -- the same pinned
    // encoding ICache.v's header documents, now sourced from MMU.v.
    //=========================================================================
    wire                     ifu_mmu_abort;
    wire [MMU_VA_WIDTH-1:0]  ifu_mmu_va;
    wire                     ifu_mmu_va_vld;
    wire                     mmu_ifu_access_fault;
    wire [MMU_PA_WIDTH-1:0]  mmu_ifu_pa;
    wire                     mmu_ifu_pa_vld;
    wire [MMU_PROT_WIDTH-1:0] mmu_ifu_prot;

    wire [MMU_VA_WIDTH-1:0]  lsu_mmu_va;
    wire                     lsu_mmu_va_vld;
    wire [1:0]               lsu_mmu_priv_mode;
    wire                     lsu_mmu_st_inst;
    wire [MMU_PA_WIDTH-1:0]  mmu_lsu_pa;
    wire                     mmu_lsu_pa_vld;
    wire                     mmu_lsu_ca;
    wire                     mmu_lsu_so;
    wire                     mmu_lsu_buf;
    wire                     mmu_lsu_sec;
    wire                     mmu_lsu_sh;
    wire                     mmu_lsu_page_fault;
    wire                     mmu_lsu_access_fault;
    wire                     lsu_mmu_abort;

    //=========================================================================
    // M4 Task 5: the PTW memory-read servant channel (MMU.v <-> LSU.v,
    // group 5's frozen six wires) and the PMP check channels (MMU.v <->
    // PMP.v). Dead-tied at Task 4 (MMU.v's own header note); wired for real
    // here.
    //=========================================================================
    wire                     mmu_lsu_data_req;
    wire [PC_WIDTH-1:0]      mmu_lsu_data_req_addr;
    wire                     mmu_lsu_data_req_size;
    wire [63:0]              lsu_mmu_data;
    wire                     lsu_mmu_data_vld;
    wire                     lsu_mmu_bus_error;

    wire [PC_WIDTH-1:0]      mmu_pmp_fetch_pa;
    wire                     mmu_pmp_fetch_vld;
    wire [PC_WIDTH-1:0]      mmu_pmp_data_pa;
    wire                     mmu_pmp_load;
    wire                     mmu_pmp_store;
    wire                     mmu_pmp_data_vld;
    wire [1:0]               mmu_pmp_data_priv_mode;

    //=========================================================================
    // IFU <-> BPU seam (see BPU.v's header for the port rationale).
    // The `cp0_ifu_bht_en/_btb_en/_ras_en/_bht_inv/_btb_clr` group at the
    // bottom of this seam is the BPU slice of the CSR config fan-out --
    // M2 drives it from u_csr's MHCR-derived wires (u_fetchsink was the
    // driver in M1); wire names unchanged.
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
    wire                     ibuf_ipack_stall;   // Task 7.1 port-freeze amendment (BPU.v header)

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
    wire                     ifu_idu_id_fault_pgflt;    // M4 Task 6
    wire                     ifu_idu_id_fault_accflt;   // M4 Task 6
    wire                     idu_ifu_id_stall;

    //=========================================================================
    // IFU/BPU <-> IU (BJU) + RTU redirect seam. M1 froze these wires
    // against FetchSink's fake BJU/RTU; M2 drives them from the real
    // u_iu / u_rtu (names/widths unchanged -- IU.v's and RTU.v's headers
    // re-verified them byte-for-byte against this file's M1 declarations).
    // `cp0_xx_mrvbr` is the reset-vector seed, now driven by u_csr
    // (mrvbr for real) and consumed by u_ifu AND u_iu (BJU's PC seed).
    //=========================================================================
    wire                     iu_ifu_tar_pc_vld;
    wire [63:0]              iu_ifu_tar_pc;
    wire                     iu_ifu_pc_mispred;
    wire                     iu_ifu_bht_mispred;
    wire                     iu_idu_br_cancel;   // branch-mispredict cancel IU->IDU (donor iu_yy_xx_cancel)
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
    // IDU <-> IU : EX1 dispatch (IU's slice of id_ex1_t) + IU's five
    // point-to-point stall/full signals back (contract 8)
    //=========================================================================
    wire                     idu_iu_ex1_inst_vld;
    wire                     idu_iu_ex1_pipedown_vld;
    wire                     idu_iu_ex1_alu_sel;
    wire                     idu_iu_ex1_bju_sel;
    wire                     idu_iu_ex1_bju_br_sel;
    wire                     idu_iu_ex1_mult_sel;
    wire                     idu_iu_ex1_div_sel;
    wire [FUNC_WIDTH-1:0]    idu_iu_ex1_func;
    wire [63:0]              idu_iu_ex1_src0_data;
    wire                     idu_iu_ex1_src0_ready;
    wire [63:0]              idu_iu_ex1_src1_data;
    wire                     idu_iu_ex1_src1_ready;
    wire [63:0]              idu_iu_ex1_src2_data;
    wire                     idu_iu_ex1_src2_ready;
    wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_dst0_reg;
    wire [1:0]               idu_iu_ex1_bht_pred;
    wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_src0_reg;
    wire [GPR_IDX_WIDTH-1:0] idu_iu_ex1_src1_reg;
    // Task 7.3: EX1 instruction length (1=32b,0=16b RVC) -- the RVC-aware
    // PC increment source (IDU latches it; IU/LSU/CSR each consume their
    // slice; the RTU re-exposes the completing one back to IU's pcgen).
    wire                     idu_iu_ex1_inst_len;

    wire                     iu_idu_mult_issue_stall;
    wire                     iu_idu_mult_full;
    wire                     iu_idu_div_full;
    wire                     iu_idu_bju_full;
    wire                     iu_idu_bju_global_full;

    //=========================================================================
    // IDU <-> LSU : EX1 dispatch (LSU's slice of id_ex1_t) + LSU's single
    // EX1 issue-gate stall (contract 8)
    //=========================================================================
    wire                     idu_lsu_ex1_dp_sel;
    wire                     idu_lsu_ex1_sel;
    wire                     idu_lsu_ex1_raw_vld;
    wire [FUNC_WIDTH-1:0]    idu_lsu_ex1_func;
    wire [63:0]              idu_lsu_ex1_src0_data;
    wire                     idu_lsu_ex1_src0_ready;
    wire [63:0]              idu_lsu_ex1_src1_data;
    wire                     idu_lsu_ex1_src1_ready;
    wire [63:0]              idu_lsu_ex1_src2_data;
    wire                     idu_lsu_ex1_src2_ready;
    wire [GPR_IDX_WIDTH-1:0] idu_lsu_ex1_dst0_reg;
    wire                     idu_lsu_ex1_inst_len;   // Task 7.3
    wire                     lsu_idu_full;
    wire                     lsu_cp0_stb_empty;   // Task 10.1 fence.i quiescence
    wire                     cp0_lsu_dcache_clean; // Task 10.1 fence.i D-clean walk
    wire                     lsu_cp0_clean_done;
    wire                     cp0_idu_fencei_full; // Task 10.1 fence EX1-hold

    //=========================================================================
    // IDU/IU -> CSR : EX1 dispatch (CSR's slice of id_ex1_t) + BJU's PC
    // passthrough (the only IU<->CP0 data connection, IU note S4.5/S9)
    //=========================================================================
    wire                     idu_cp0_ex1_sel;
    wire [FUNC_WIDTH-1:0]    idu_cp0_ex1_func;
    wire [31:0]              idu_cp0_ex1_opcode;
    wire                     idu_cp0_ex1_illegal;
    wire                     idu_cp0_ex1_fetch_pgflt;    // M4 Task 6
    wire                     idu_cp0_ex1_fetch_accflt;   // M4 Task 6
    wire [63:0]              idu_cp0_ex1_src0_data;
    wire [63:0]              idu_cp0_ex1_src1_data;
    wire [GPR_IDX_WIDTH-1:0] idu_cp0_ex1_dst0_reg;
    wire                     idu_cp0_ex1_inst_len;   // Task 7.3
    // M5 Task 2: FRF read-port outputs. Consumed by FPU.v below (Task 3);
    // idu_fpu_ex1_fsrc2_data still has no consumer (fmadd family: Task 5).
    wire [63:0]              idu_fpu_ex1_fsrc0_data;
    wire [63:0]              idu_fpu_ex1_fsrc1_data;
    wire [63:0]              idu_fpu_ex1_fsrc2_data;

    // M5 Task 3: FPU.v FALU outputs. IDU decode does not yet route to
    // FPU (Task 4), so these are landing pads with no consumer yet.
    wire [63:0]              fpu_rtu_ex1_falu_fdata;
    wire [63:0]              fpu_rtu_ex1_falu_xdata;
    wire [4:0]               fpu_rtu_ex1_falu_fflags;
    wire                     fpu_rtu_ex1_falu_fvld;
    wire                     fpu_rtu_ex1_falu_xvld;
    wire [PC_WIDTH-1:0]      iu_cp0_ex1_cur_pc;
    wire [15:0]              iu_lsu_ex1_cur_pc;   // M3b Task D: PFB PC tag

    //=========================================================================
    // IU <-> RTU : the four separate writeback buses (contract 1) + RTU's
    // two MULT/DIV writeback-race grants
    //=========================================================================
    wire                     iu_rtu_ex1_alu_cmplt;
    wire                     iu_rtu_ex1_alu_cmplt_dp;
    wire [63:0]              iu_rtu_ex1_alu_data;
    wire                     iu_rtu_ex1_alu_inst_len;
    wire                     iu_rtu_ex1_alu_inst_split;
    wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_alu_preg;
    wire                     iu_rtu_ex1_alu_wb_dp;
    wire                     iu_rtu_ex1_alu_wb_vld;

    wire                     iu_rtu_ex1_bju_cmplt;
    wire                     iu_rtu_ex1_bju_cmplt_dp;
    wire                     iu_rtu_ex1_bju_cmplt_for_pcgen;
    wire [63:0]              iu_rtu_ex1_bju_data;
    wire                     iu_rtu_ex1_bju_inst_len;
    wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_bju_preg;
    wire                     iu_rtu_ex1_bju_wb_dp;
    wire                     iu_rtu_ex1_bju_wb_vld;
    wire                     iu_rtu_ex1_branch_inst;
    wire [PC_WIDTH-1:0]      iu_rtu_ex1_cur_pc;
    wire [PC_WIDTH-1:0]      iu_rtu_ex1_next_pc;
    wire                     iu_rtu_ex2_bju_ras_mispred;
    wire                     iu_rtu_depd_lsu_chgflow_vld;
    wire [PC_WIDTH-1:0]      iu_rtu_depd_lsu_chgflow_next_pc;

    wire                     iu_rtu_ex1_mul_cmplt;
    wire                     iu_rtu_ex1_mul_cmplt_dp;
    wire [63:0]              iu_rtu_ex3_mul_data;
    wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex3_mul_preg;
    wire                     iu_rtu_ex3_mul_wb_vld;
    wire                     iu_rtu_ex1_mul_inst_len;   // Task 7.3

    wire                     iu_rtu_ex1_div_cmplt;
    wire                     iu_rtu_ex1_div_cmplt_dp;
    wire [63:0]              iu_rtu_div_data;
    wire [GPR_IDX_WIDTH-1:0] iu_rtu_div_preg;
    wire                     iu_rtu_div_wb_dp;
    wire                     iu_rtu_div_wb_vld;
    wire                     iu_rtu_ex1_div_inst_len;   // Task 7.3

    wire                     rtu_iu_mul_wb_grant;
    wire                     rtu_iu_div_wb_grant;
    // Task 7.3: RTU -> IU PC-generator retire feedback (the completing
    // instruction's cmplt/length/split for IU's bju_pcgen_pc advance).
    wire                     rtu_iu_ex1_cmplt;
    wire                     rtu_iu_ex1_inst_len;
    wire                     rtu_iu_ex1_inst_split;

    //=========================================================================
    // LSU <-> RTU : lsu_rtu_t completion/exception bus + RTU's "point of
    // no return" acks (RTU note S6)
    //=========================================================================
    wire                     lsu_rtu_ex1_cmplt;
    wire                     lsu_rtu_ex1_cmplt_dp;
    wire                     lsu_rtu_ex1_cmplt_for_pcgen;   // Task 9.7 (donor aq_lsu_ag.v:1675)
    wire                     lsu_rtu_ex1_inst_len;   // Task 7.3
    wire [63:0]              lsu_rtu_wb_data;
    wire [GPR_IDX_WIDTH-1:0] lsu_rtu_wb_preg;
    wire                     lsu_rtu_wb_vld;
    wire [63:0]              lsu_rtu_ex2_data;
    wire                     lsu_rtu_ex2_data_vld;
    wire [GPR_IDX_WIDTH-1:0] lsu_rtu_ex2_dest_reg;
    wire                     lsu_rtu_expt_vld;
    wire [4:0]               lsu_rtu_expt_vec;
    wire [63:0]              lsu_rtu_tval;
    wire                     lsu_rtu_async_expt_vld;
    wire                     lsu_rtu_async_ld_inst;

    wire                     rtu_lsu_expt_ack;
    wire                     rtu_lsu_expt_exit;

    //=========================================================================
    // LSU -> IU : BJU's LSU-dependent-branch forwards (IU note S4.4).
    // WIRING DECISION (documented, Task 7): the donor's LSU carries TWO
    // distinct forward groups to IU -- the DA-stage `da_xx_fwd_*`
    // (aq_lsu_dc.v:2412-2414) and the DC-stage `lsu_iu_ex2_*`
    // (aq_lsu_dc.v:2210-2212), one stage apart. M2's LSU.v (frozen port
    // list) collapses both into its single REPLY-cycle output family
    // `lsu_rtu_ex2_data/_data_vld/_dest_reg` (LSU.v header: driven in the
    // same cycle as the load/store completes), so this file fans that ONE
    // group out to BOTH of IU's input groups. No new LSU ports are
    // invented -- this is a fan-out of a frozen output.
    //=========================================================================

    //=========================================================================
    // CSR <-> RTU : cp0_rtu_t (CSR's EX1 completion + trap-declaration bus
    // + mtvec redirect target) and the RTU->CSR trap-entry capture group
    //=========================================================================
    wire                     cp0_rtu_ex1_cmplt_dp;
    wire                     cp0_rtu_ex1_inst_len;   // Task 7.3
    wire [63:0]              cp0_rtu_ex1_wb_data;
    wire [GPR_IDX_WIDTH-1:0] cp0_rtu_ex1_wb_preg;
    wire                     cp0_rtu_ex1_wb_vld;
    wire                     cp0_rtu_ex1_expt_vld;
    wire                     cp0_rtu_ex1_expt_int;
    wire [4:0]               cp0_rtu_ex1_expt_vec;
    wire                     cp0_rtu_ex1_chgflw;
    wire [PC_WIDTH-1:0]      cp0_rtu_ex1_chgflw_pc;
    wire [PC_WIDTH-1:0]      cp0_rtu_trap_pc;

    wire                     rtu_yy_xx_expt_vld;
    wire                     rtu_yy_xx_expt_int;
    wire [4:0]               rtu_yy_xx_expt_vec;
    wire                     rtu_yy_xx_flush_fe;
    wire                     rtu_yy_xx_flush;
    wire [PC_WIDTH-1:0]      rtu_cp0_epc;
    wire [63:0]              rtu_cp0_tval;
    wire                     rtu_cp0_inst_retire;   // M4 Task 1: minstret inc

    //=========================================================================
    // RTU <-> IDU : the exclusive bypass network (fwd0/1/2 + wb0/1, IDU
    // note S6) + the flush/drain/commit group (RTU note S6)
    //=========================================================================
    wire [63:0]              rtu_idu_fwd0_data;
    wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd0_reg;
    wire                     rtu_idu_fwd0_vld;
    wire [63:0]              rtu_idu_fwd1_data;
    wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd1_reg;
    wire                     rtu_idu_fwd1_vld;
    wire [63:0]              rtu_idu_fwd2_data;
    wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd2_reg;
    wire                     rtu_idu_fwd2_vld;
    wire [63:0]              rtu_idu_wb0_data;
    wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb0_reg;
    wire                     rtu_idu_wb0_vld;
    wire [63:0]              rtu_idu_wb1_data;
    wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb1_reg;
    wire                     rtu_idu_wb1_vld;
    // M5 Task 2: FRF write ports. No producer exists until Task 3+ (RTU.v
    // gains no FPU-facing ports yet) -- tied 0 at the instantiation below.

    wire                     rtu_idu_flush_fe;
    wire                     rtu_idu_flush_stall;
    wire                     rtu_idu_flush_wbt;
    wire                     rtu_idu_commit;
    wire                     rtu_idu_commit_for_bju;
    wire                     rtu_idu_pipeline_empty;

    //=========================================================================
    // CSR -> LSU : MHCR.de/wa + MXSTATUS.mm (design doc S2.3.3/S2.3.6)
    //=========================================================================
    wire                     cp0_lsu_dcache_en;
    wire                     cp0_lsu_mm;
    wire                     cp0_lsu_wa;
    // M3b Task D: MHINT D-cache prefetch controls (CSR -> LSU PFB)
    wire                     cp0_lsu_dcache_pref_en;
    wire [1:0]               cp0_lsu_dcache_pref_dist;
    // M3b Task E: MHINT.amr (CSR -> LSU AMR)
    wire [1:0]               cp0_lsu_amr;
    // M4 Task 2: PMP register interface (CSR <-> PMP). The pmpcfg/pmpaddr
    // storage lives in PMP.v; CSR.v decodes/strobes/reads back.
    wire                     pmp_cfg0_wen;
    wire [63:0]              pmp_cfg0_wdata;
    wire [7:0]               pmp_addr_wen;
    wire [63:0]              pmp_addr_wdata;
    wire [2:0]               pmp_addr_rsel;
    wire [63:0]              pmp_cfg0_value;
    wire [63:0]              pmp_addr_value;
    wire [1:0]               cp0_pmp_priv_mode;
    wire                     pmp_fetch_deny;
    wire                     pmp_data_deny;
    // M4 Task 1: privilege + MMU controls (CSR -> MMU/LSU; the MMU/LSU give
    // them meaning at M4 Tasks 3-5).
    wire [1:0]               cp0_yy_priv_mode;
    wire [63:0]              cp0_mmu_satp_data;
    wire                     cp0_mmu_satp_wen;
    wire                     cp0_mmu_mxr;
    wire                     cp0_mmu_sum;
    // M4 Task 7: sfence.vma whole-TLB invalidate handshake (CSR <-> MMU).
    wire                     cp0_mmu_sfence_vld;
    wire                     mmu_cp0_sfence_done;
    wire                     cp0_lsu_mprv;
    wire [1:0]               cp0_lsu_mpp;

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
        .ifu_idu_id_fault_pgflt  (ifu_idu_id_fault_pgflt),
        .ifu_idu_id_fault_accflt (ifu_idu_id_fault_accflt),
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
        .ibuf_ipack_stall        (ibuf_ipack_stall),

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
        .ibuf_ipack_stall        (ibuf_ipack_stall),

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
        .iu_ifu_tar_pc_vld       (iu_ifu_tar_pc_vld),

        .rtu_ifu_flush_fe        (rtu_ifu_flush_fe)
    );

    //=========================================================================
    // MMU instance (contract 2): serves ICache's ITLB port group and
    // LSU's DTLB port group. No parameters; clk/rst_n for symmetry with
    // the rest of the core (the body is a combinational lookup today --
    // the M4 real MMU will use the clock).
    //=========================================================================
    MMU u_mmu (
        .clk                    (clk),
        .rst_n                  (rst_n),

        .ifu_mmu_abort          (ifu_mmu_abort),
        .ifu_mmu_va             (ifu_mmu_va),
        .ifu_mmu_va_vld         (ifu_mmu_va_vld),
        .mmu_ifu_access_fault   (mmu_ifu_access_fault),
        .mmu_ifu_pa             (mmu_ifu_pa),
        .mmu_ifu_pa_vld         (mmu_ifu_pa_vld),
        .mmu_ifu_prot           (mmu_ifu_prot),

        .lsu_mmu_va             (lsu_mmu_va),
        .lsu_mmu_va_vld         (lsu_mmu_va_vld),
        .lsu_mmu_priv_mode      (lsu_mmu_priv_mode),
        .lsu_mmu_st_inst        (lsu_mmu_st_inst),
        .mmu_lsu_pa             (mmu_lsu_pa),
        .mmu_lsu_pa_vld         (mmu_lsu_pa_vld),
        .mmu_lsu_ca             (mmu_lsu_ca),
        .mmu_lsu_so             (mmu_lsu_so),
        .mmu_lsu_buf            (mmu_lsu_buf),
        .mmu_lsu_sec            (mmu_lsu_sec),
        .mmu_lsu_sh             (mmu_lsu_sh),
        .mmu_lsu_page_fault     (mmu_lsu_page_fault),
        .mmu_lsu_access_fault   (mmu_lsu_access_fault),

        // M4 Task 5: the PTW memory-read servant, LSU.v's real body now.
        .mmu_lsu_data_req       (mmu_lsu_data_req),
        .mmu_lsu_data_req_addr  (mmu_lsu_data_req_addr),
        .mmu_lsu_data_req_size  (mmu_lsu_data_req_size),
        .lsu_mmu_data           (lsu_mmu_data),
        .lsu_mmu_data_vld       (lsu_mmu_data_vld),
        .lsu_mmu_bus_error      (lsu_mmu_bus_error),
        .lsu_mmu_abort          (lsu_mmu_abort),

        // M4 Task 5: PMP check channels, PMP.v's real instance now.
        .mmu_pmp_fetch_pa       (mmu_pmp_fetch_pa),
        .mmu_pmp_fetch_vld      (mmu_pmp_fetch_vld),
        .pmp_mmu_fetch_deny     (pmp_fetch_deny),
        .mmu_pmp_data_pa        (mmu_pmp_data_pa),
        .mmu_pmp_load           (mmu_pmp_load),
        .mmu_pmp_store          (mmu_pmp_store),
        .mmu_pmp_data_vld       (mmu_pmp_data_vld),
        .mmu_pmp_data_priv_mode (mmu_pmp_data_priv_mode),
        .pmp_mmu_data_deny      (pmp_data_deny),

        .cp0_mmu_satp_data      (cp0_mmu_satp_data),
        .cp0_mmu_satp_wen       (cp0_mmu_satp_wen),
        .cp0_mmu_mxr            (cp0_mmu_mxr),
        .cp0_mmu_sum            (cp0_mmu_sum),
        .cp0_mmu_sfence_vld     (cp0_mmu_sfence_vld),
        .mmu_cp0_sfence_done    (mmu_cp0_sfence_done),
        .cp0_yy_priv_mode       (cp0_yy_priv_mode)
    );

    //=========================================================================
    // IDU instance: decode + WBT + GPR + EU dispatch (Task 5)
    //=========================================================================
    IDU u_idu (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .ifu_idu_id_inst         (ifu_idu_id_inst),
        .ifu_idu_id_inst_vld     (ifu_idu_id_inst_vld),
        .ifu_idu_id_bht_pred     (ifu_idu_id_bht_pred),
        .ifu_idu_id_fault_pgflt  (ifu_idu_id_fault_pgflt),
        .ifu_idu_id_fault_accflt (ifu_idu_id_fault_accflt),
        .idu_ifu_id_stall        (idu_ifu_id_stall),

        .idu_iu_ex1_inst_vld     (idu_iu_ex1_inst_vld),
        .idu_iu_ex1_pipedown_vld (idu_iu_ex1_pipedown_vld),
        .idu_iu_ex1_alu_sel      (idu_iu_ex1_alu_sel),
        .idu_iu_ex1_bju_sel      (idu_iu_ex1_bju_sel),
        .idu_iu_ex1_bju_br_sel   (idu_iu_ex1_bju_br_sel),
        .idu_iu_ex1_mult_sel     (idu_iu_ex1_mult_sel),
        .idu_iu_ex1_div_sel      (idu_iu_ex1_div_sel),
        .idu_iu_ex1_func         (idu_iu_ex1_func),
        .idu_iu_ex1_src0_data    (idu_iu_ex1_src0_data),
        .idu_iu_ex1_src0_ready   (idu_iu_ex1_src0_ready),
        .idu_iu_ex1_src1_data    (idu_iu_ex1_src1_data),
        .idu_iu_ex1_src1_ready   (idu_iu_ex1_src1_ready),
        .idu_iu_ex1_src2_data    (idu_iu_ex1_src2_data),
        .idu_iu_ex1_src2_ready   (idu_iu_ex1_src2_ready),
        .idu_iu_ex1_dst0_reg     (idu_iu_ex1_dst0_reg),
        .idu_iu_ex1_bht_pred     (idu_iu_ex1_bht_pred),
        .idu_iu_ex1_src0_reg     (idu_iu_ex1_src0_reg),
        .idu_iu_ex1_src1_reg     (idu_iu_ex1_src1_reg),
        .idu_iu_ex1_inst_len     (idu_iu_ex1_inst_len),

        .idu_lsu_ex1_dp_sel      (idu_lsu_ex1_dp_sel),
        .idu_lsu_ex1_sel         (idu_lsu_ex1_sel),
        .idu_lsu_ex1_raw_vld     (idu_lsu_ex1_raw_vld),
        .idu_lsu_ex1_func        (idu_lsu_ex1_func),
        .idu_lsu_ex1_src0_data   (idu_lsu_ex1_src0_data),
        .idu_lsu_ex1_src0_ready  (idu_lsu_ex1_src0_ready),
        .idu_lsu_ex1_src1_data   (idu_lsu_ex1_src1_data),
        .idu_lsu_ex1_src1_ready  (idu_lsu_ex1_src1_ready),
        .idu_lsu_ex1_src2_data   (idu_lsu_ex1_src2_data),
        .idu_lsu_ex1_src2_ready  (idu_lsu_ex1_src2_ready),
        .idu_lsu_ex1_dst0_reg    (idu_lsu_ex1_dst0_reg),
        .idu_lsu_ex1_inst_len    (idu_lsu_ex1_inst_len),

        .idu_cp0_ex1_sel         (idu_cp0_ex1_sel),
        .idu_cp0_ex1_func        (idu_cp0_ex1_func),
        .idu_cp0_ex1_opcode      (idu_cp0_ex1_opcode),
        .idu_cp0_ex1_illegal     (idu_cp0_ex1_illegal),
        .idu_cp0_ex1_fetch_pgflt  (idu_cp0_ex1_fetch_pgflt),
        .idu_cp0_ex1_fetch_accflt (idu_cp0_ex1_fetch_accflt),
        .idu_cp0_ex1_src0_data   (idu_cp0_ex1_src0_data),
        .idu_cp0_ex1_src1_data   (idu_cp0_ex1_src1_data),
        .idu_cp0_ex1_dst0_reg    (idu_cp0_ex1_dst0_reg),
        .idu_cp0_ex1_inst_len    (idu_cp0_ex1_inst_len),

        .idu_fpu_ex1_fsrc0_data  (idu_fpu_ex1_fsrc0_data),
        .idu_fpu_ex1_fsrc1_data  (idu_fpu_ex1_fsrc1_data),
        .idu_fpu_ex1_fsrc2_data  (idu_fpu_ex1_fsrc2_data),

        .rtu_idu_fwd0_data       (rtu_idu_fwd0_data),
        .rtu_idu_fwd0_reg        (rtu_idu_fwd0_reg),
        .rtu_idu_fwd0_vld        (rtu_idu_fwd0_vld),
        .rtu_idu_fwd1_data       (rtu_idu_fwd1_data),
        .rtu_idu_fwd1_reg        (rtu_idu_fwd1_reg),
        .rtu_idu_fwd1_vld        (rtu_idu_fwd1_vld),
        .rtu_idu_fwd2_data       (rtu_idu_fwd2_data),
        .rtu_idu_fwd2_reg        (rtu_idu_fwd2_reg),
        .rtu_idu_fwd2_vld        (rtu_idu_fwd2_vld),
        .rtu_idu_wb0_data        (rtu_idu_wb0_data),
        .rtu_idu_wb0_reg         (rtu_idu_wb0_reg),
        .rtu_idu_wb0_vld         (rtu_idu_wb0_vld),
        .rtu_idu_wb1_data        (rtu_idu_wb1_data),
        .rtu_idu_wb1_reg         (rtu_idu_wb1_reg),
        .rtu_idu_wb1_vld         (rtu_idu_wb1_vld),
        .rtu_idu_wbf0_data       (64'd0),
        .rtu_idu_wbf0_reg        ({GPR_IDX_WIDTH{1'b0}}),
        .rtu_idu_wbf0_vld        (1'b0),
        .rtu_idu_wbf1_data       (64'd0),
        .rtu_idu_wbf1_reg        ({GPR_IDX_WIDTH{1'b0}}),
        .rtu_idu_wbf1_vld        (1'b0),

        .iu_idu_mult_issue_stall (iu_idu_mult_issue_stall),
        .iu_idu_mult_full        (iu_idu_mult_full),
        .iu_idu_div_full         (iu_idu_div_full),
        .iu_idu_bju_full         (iu_idu_bju_full),
        .iu_idu_bju_global_full  (iu_idu_bju_global_full),

        .lsu_idu_full            (lsu_idu_full),
        .cp0_idu_fencei_full     (cp0_idu_fencei_full),

        .rtu_idu_flush_fe        (rtu_idu_flush_fe),
        .iu_idu_br_cancel        (iu_idu_br_cancel),
        .rtu_idu_flush_stall     (rtu_idu_flush_stall),
        .rtu_idu_flush_wbt       (rtu_idu_flush_wbt),
        .rtu_idu_commit          (rtu_idu_commit),
        .rtu_idu_commit_for_bju  (rtu_idu_commit_for_bju),
        .rtu_idu_pipeline_empty  (rtu_idu_pipeline_empty)
    );

    //=========================================================================
    // IU instance: ALU + BJU + MULT + DIV (Task 3). `da_xx_fwd_*` and
    // `lsu_iu_ex2_*` both tie to LSU's single `lsu_rtu_ex2_*` output
    // family -- see the "LSU -> IU" seam banner above for the rationale
    // (donor's two stage-apart forwards collapsed to one REPLY-cycle
    // family in M2's frozen LSU.v port list).
    //=========================================================================
    IU u_iu (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .idu_iu_ex1_inst_vld     (idu_iu_ex1_inst_vld),
        .idu_iu_ex1_pipedown_vld (idu_iu_ex1_pipedown_vld),
        .idu_iu_ex1_alu_sel      (idu_iu_ex1_alu_sel),
        .idu_iu_ex1_bju_sel      (idu_iu_ex1_bju_sel),
        .idu_iu_ex1_bju_br_sel   (idu_iu_ex1_bju_br_sel),
        .idu_iu_ex1_mult_sel     (idu_iu_ex1_mult_sel),
        .idu_iu_ex1_div_sel      (idu_iu_ex1_div_sel),
        .idu_iu_ex1_func         (idu_iu_ex1_func),
        .idu_iu_ex1_src0_data    (idu_iu_ex1_src0_data),
        .idu_iu_ex1_src0_ready   (idu_iu_ex1_src0_ready),
        .idu_iu_ex1_src1_data    (idu_iu_ex1_src1_data),
        .idu_iu_ex1_src1_ready   (idu_iu_ex1_src1_ready),
        .idu_iu_ex1_src2_data    (idu_iu_ex1_src2_data),
        .idu_iu_ex1_src2_ready   (idu_iu_ex1_src2_ready),
        .idu_iu_ex1_dst0_reg     (idu_iu_ex1_dst0_reg),
        .idu_iu_ex1_bht_pred     (idu_iu_ex1_bht_pred),
        .idu_iu_ex1_src0_reg     (idu_iu_ex1_src0_reg),
        .idu_iu_ex1_src1_reg     (idu_iu_ex1_src1_reg),
        .idu_iu_ex1_inst_len     (idu_iu_ex1_inst_len),

        .iu_idu_mult_issue_stall (iu_idu_mult_issue_stall),
        .iu_idu_mult_full        (iu_idu_mult_full),
        .iu_idu_div_full         (iu_idu_div_full),
        .iu_idu_bju_full         (iu_idu_bju_full),
        .iu_idu_bju_global_full  (iu_idu_bju_global_full),

        .iu_rtu_ex1_alu_cmplt    (iu_rtu_ex1_alu_cmplt),
        .iu_rtu_ex1_alu_cmplt_dp (iu_rtu_ex1_alu_cmplt_dp),
        .iu_rtu_ex1_alu_data     (iu_rtu_ex1_alu_data),
        .iu_rtu_ex1_alu_inst_len (iu_rtu_ex1_alu_inst_len),
        .iu_rtu_ex1_alu_inst_split (iu_rtu_ex1_alu_inst_split),
        .iu_rtu_ex1_alu_preg     (iu_rtu_ex1_alu_preg),
        .iu_rtu_ex1_alu_wb_dp    (iu_rtu_ex1_alu_wb_dp),
        .iu_rtu_ex1_alu_wb_vld   (iu_rtu_ex1_alu_wb_vld),

        .iu_rtu_ex1_bju_cmplt    (iu_rtu_ex1_bju_cmplt),
        .iu_rtu_ex1_bju_cmplt_dp (iu_rtu_ex1_bju_cmplt_dp),
        .iu_rtu_ex1_bju_cmplt_for_pcgen (iu_rtu_ex1_bju_cmplt_for_pcgen),
        .iu_rtu_ex1_bju_data     (iu_rtu_ex1_bju_data),
        .iu_rtu_ex1_bju_inst_len (iu_rtu_ex1_bju_inst_len),
        .iu_rtu_ex1_bju_preg     (iu_rtu_ex1_bju_preg),
        .iu_rtu_ex1_bju_wb_dp    (iu_rtu_ex1_bju_wb_dp),
        .iu_rtu_ex1_bju_wb_vld   (iu_rtu_ex1_bju_wb_vld),
        .iu_rtu_ex1_branch_inst  (iu_rtu_ex1_branch_inst),
        .iu_rtu_ex1_cur_pc       (iu_rtu_ex1_cur_pc),
        .iu_rtu_ex1_next_pc      (iu_rtu_ex1_next_pc),
        .iu_rtu_ex2_bju_ras_mispred (iu_rtu_ex2_bju_ras_mispred),
        .iu_rtu_depd_lsu_chgflow_vld (iu_rtu_depd_lsu_chgflow_vld),
        .iu_rtu_depd_lsu_chgflow_next_pc (iu_rtu_depd_lsu_chgflow_next_pc),

        .iu_rtu_ex1_mul_cmplt    (iu_rtu_ex1_mul_cmplt),
        .iu_rtu_ex1_mul_cmplt_dp (iu_rtu_ex1_mul_cmplt_dp),
        .iu_rtu_ex3_mul_data     (iu_rtu_ex3_mul_data),
        .iu_rtu_ex3_mul_preg     (iu_rtu_ex3_mul_preg),
        .iu_rtu_ex3_mul_wb_vld   (iu_rtu_ex3_mul_wb_vld),
        .iu_rtu_ex1_mul_inst_len (iu_rtu_ex1_mul_inst_len),

        .iu_rtu_ex1_div_cmplt    (iu_rtu_ex1_div_cmplt),
        .iu_rtu_ex1_div_cmplt_dp (iu_rtu_ex1_div_cmplt_dp),
        .iu_rtu_div_data         (iu_rtu_div_data),
        .iu_rtu_div_preg         (iu_rtu_div_preg),
        .iu_rtu_div_wb_dp        (iu_rtu_div_wb_dp),
        .iu_rtu_div_wb_vld       (iu_rtu_div_wb_vld),
        .iu_rtu_ex1_div_inst_len (iu_rtu_ex1_div_inst_len),

        .rtu_iu_mul_wb_grant     (rtu_iu_mul_wb_grant),
        .rtu_iu_div_wb_grant     (rtu_iu_div_wb_grant),
        .rtu_iu_ex1_cmplt        (rtu_iu_ex1_cmplt),
        .rtu_iu_ex1_inst_len     (rtu_iu_ex1_inst_len),
        .rtu_iu_ex1_inst_split   (rtu_iu_ex1_inst_split),

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
        .iu_idu_br_cancel        (iu_idu_br_cancel),

        .da_xx_fwd_data          (lsu_rtu_ex2_data),
        .da_xx_fwd_dst_reg       (lsu_rtu_ex2_dest_reg),
        .da_xx_fwd_vld           (lsu_rtu_ex2_data_vld),
        .lsu_iu_ex2_data         (lsu_rtu_ex2_data),
        .lsu_iu_ex2_data_vld     (lsu_rtu_ex2_data_vld),
        .lsu_iu_ex2_dest_reg     (lsu_rtu_ex2_dest_reg),

        .iu_cp0_ex1_cur_pc       (iu_cp0_ex1_cur_pc),
        .iu_lsu_ex1_cur_pc       (iu_lsu_ex1_cur_pc),

        .cp0_xx_mrvbr            (cp0_xx_mrvbr)
    );

    //=========================================================================
    // FPU instance: FALU sub-block only (Task 3). IDU decode does not yet
    // drive the `_fadd_sel`/`_fspu_sel`/`_fcnvt_sel`/`_func`/`_rm` inputs
    // (Task 4 wires those up), so they're tied to inert constants here --
    // same pattern as the RTU->IDU tie-offs above. `idu_fpu_ex1_fsrc0/1_data`
    // are the real M5 Task 2 FRF read-port outputs. `fpu_rtu_ex1_falu_*`
    // outputs have no consumer yet (RTU wiring lands with Task 4).
    //=========================================================================
    FPU u_fpu (
        .clk                       (clk),
        .rst_n                     (rst_n),

        .idu_fpu_ex1_fadd_sel      (1'b0),
        .idu_fpu_ex1_fspu_sel      (1'b0),
        .idu_fpu_ex1_fcnvt_sel     (1'b0),
        .idu_fpu_ex1_func          ({FUNC_WIDTH{1'b0}}),
        .idu_fpu_ex1_rm            (3'b000),
        .idu_fpu_ex1_fsrc0_data    (idu_fpu_ex1_fsrc0_data),
        .idu_fpu_ex1_fsrc1_data    (idu_fpu_ex1_fsrc1_data),

        .fpu_rtu_ex1_falu_fdata    (fpu_rtu_ex1_falu_fdata),
        .fpu_rtu_ex1_falu_xdata    (fpu_rtu_ex1_falu_xdata),
        .fpu_rtu_ex1_falu_fflags   (fpu_rtu_ex1_falu_fflags),
        .fpu_rtu_ex1_falu_fvld     (fpu_rtu_ex1_falu_fvld),
        .fpu_rtu_ex1_falu_xvld     (fpu_rtu_ex1_falu_xvld)
    );

    //=========================================================================
    // LSU instance: AG/DC/DA pipe + 4-entry STB + D-side AXI master
    // (Task 6). The `axi_d_*` group moves here from FetchSink's tohost-only
    // write FSM -- same channel, same outer port names, real bus path now.
    // DCache is INSIDE this module (no separate top-level instance).
    //
    // ADDR_TOHOST confirmation (Task 7.1): the M2 Task-1 relocation of
    // tohost to rvproc_pkg's ADDR_TOHOST = 0x7FFF_F000 reaches this store
    // path unchanged -- 0x7FFF_F000 matches none of RVProcAXI's
    // CLINT/PLIC/UART base/mask pairs, so AXIAddrDecode.v routes it to
    // DEFAULT_SLAVE (SI_MEM), the same wide 512-bit MEM slave the M1
    // FetchSink tohost stores used (and the same routing test/m1/common.ld
    // relies on). No new decision; the value was already landed in Task 1.
    //=========================================================================
    LSU #(
        .DATA_WIDTH   (DATA_WIDTH),
        .ADDR_WIDTH   (ADDR_WIDTH)
    ) u_lsu (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .idu_lsu_ex1_dp_sel      (idu_lsu_ex1_dp_sel),
        .idu_lsu_ex1_sel         (idu_lsu_ex1_sel),
        .idu_lsu_ex1_raw_vld     (idu_lsu_ex1_raw_vld),
        .idu_lsu_ex1_func        (idu_lsu_ex1_func),
        .idu_lsu_ex1_src0_data   (idu_lsu_ex1_src0_data),
        .idu_lsu_ex1_src0_ready  (idu_lsu_ex1_src0_ready),
        .idu_lsu_ex1_src1_data   (idu_lsu_ex1_src1_data),
        .idu_lsu_ex1_src1_ready  (idu_lsu_ex1_src1_ready),
        .idu_lsu_ex1_src2_data   (idu_lsu_ex1_src2_data),
        .idu_lsu_ex1_src2_ready  (idu_lsu_ex1_src2_ready),
        .idu_lsu_ex1_dst0_reg    (idu_lsu_ex1_dst0_reg),
        .idu_lsu_ex1_inst_len    (idu_lsu_ex1_inst_len),
        .iu_lsu_ex1_cur_pc       (iu_lsu_ex1_cur_pc),

        .lsu_idu_full            (lsu_idu_full),
        .lsu_cp0_stb_empty       (lsu_cp0_stb_empty),
        .cp0_lsu_dcache_clean    (cp0_lsu_dcache_clean),
        .lsu_cp0_clean_done      (lsu_cp0_clean_done),

        .lsu_rtu_ex1_cmplt       (lsu_rtu_ex1_cmplt),
        .lsu_rtu_ex1_cmplt_dp    (lsu_rtu_ex1_cmplt_dp),
        .lsu_rtu_ex1_cmplt_for_pcgen (lsu_rtu_ex1_cmplt_for_pcgen),
        .lsu_rtu_ex1_inst_len    (lsu_rtu_ex1_inst_len),
        .lsu_rtu_wb_data         (lsu_rtu_wb_data),
        .lsu_rtu_wb_preg         (lsu_rtu_wb_preg),
        .lsu_rtu_wb_vld          (lsu_rtu_wb_vld),
        .lsu_rtu_ex2_data        (lsu_rtu_ex2_data),
        .lsu_rtu_ex2_data_vld    (lsu_rtu_ex2_data_vld),
        .lsu_rtu_ex2_dest_reg    (lsu_rtu_ex2_dest_reg),
        .lsu_rtu_expt_vld        (lsu_rtu_expt_vld),
        .lsu_rtu_expt_vec        (lsu_rtu_expt_vec),
        .lsu_rtu_tval            (lsu_rtu_tval),
        .lsu_rtu_async_expt_vld  (lsu_rtu_async_expt_vld),
        .lsu_rtu_async_ld_inst   (lsu_rtu_async_ld_inst),

        .rtu_lsu_expt_ack        (rtu_lsu_expt_ack),
        .rtu_lsu_expt_exit       (rtu_lsu_expt_exit),
        .rtu_yy_xx_flush_fe      (rtu_yy_xx_flush_fe),

        .lsu_mmu_va              (lsu_mmu_va),
        .lsu_mmu_va_vld          (lsu_mmu_va_vld),
        .lsu_mmu_priv_mode       (lsu_mmu_priv_mode),
        .lsu_mmu_st_inst         (lsu_mmu_st_inst),
        .mmu_lsu_pa              (mmu_lsu_pa),
        .mmu_lsu_pa_vld          (mmu_lsu_pa_vld),
        .mmu_lsu_ca              (mmu_lsu_ca),
        .mmu_lsu_so              (mmu_lsu_so),
        .mmu_lsu_buf             (mmu_lsu_buf),
        .mmu_lsu_sec             (mmu_lsu_sec),
        .mmu_lsu_sh              (mmu_lsu_sh),
        .mmu_lsu_page_fault      (mmu_lsu_page_fault),
        .mmu_lsu_access_fault    (mmu_lsu_access_fault),
        .lsu_mmu_abort           (lsu_mmu_abort),

        .mmu_lsu_data_req        (mmu_lsu_data_req),
        .mmu_lsu_data_req_addr   (mmu_lsu_data_req_addr),
        .mmu_lsu_data_req_size   (mmu_lsu_data_req_size),
        .lsu_mmu_data            (lsu_mmu_data),
        .lsu_mmu_data_vld        (lsu_mmu_data_vld),
        .lsu_mmu_bus_error       (lsu_mmu_bus_error),

        .cp0_lsu_dcache_en       (cp0_lsu_dcache_en),
        .cp0_lsu_mm              (cp0_lsu_mm),
        .cp0_lsu_wa              (cp0_lsu_wa),
        .cp0_lsu_dcache_pref_en  (cp0_lsu_dcache_pref_en),
        .cp0_lsu_dcache_pref_dist(cp0_lsu_dcache_pref_dist),
        .cp0_lsu_amr             (cp0_lsu_amr),
        .cp0_lsu_mprv            (cp0_lsu_mprv),
        .cp0_lsu_mpp             (cp0_lsu_mpp),
        .cp0_yy_priv_mode        (cp0_yy_priv_mode),

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
    // RTU instance: retire unit (Task 4). `rtu_yy_xx_dbgon` (the donor's
    // debug-on broadcast) has no M2 consumer -- no debug unit exists in
    // this milestone -- so it is left unconnected here, same as the
    // pre-existing empty `.G_mem_pin_intr()` pin in RVProcAXI.v.
    //=========================================================================
    RTU u_rtu (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .iu_rtu_ex1_alu_cmplt    (iu_rtu_ex1_alu_cmplt),
        .iu_rtu_ex1_alu_cmplt_dp (iu_rtu_ex1_alu_cmplt_dp),
        .iu_rtu_ex1_alu_data     (iu_rtu_ex1_alu_data),
        .iu_rtu_ex1_alu_inst_len (iu_rtu_ex1_alu_inst_len),
        .iu_rtu_ex1_alu_inst_split (iu_rtu_ex1_alu_inst_split),
        .iu_rtu_ex1_alu_preg     (iu_rtu_ex1_alu_preg),
        .iu_rtu_ex1_alu_wb_dp    (iu_rtu_ex1_alu_wb_dp),
        .iu_rtu_ex1_alu_wb_vld   (iu_rtu_ex1_alu_wb_vld),

        .iu_rtu_ex1_bju_cmplt    (iu_rtu_ex1_bju_cmplt),
        .iu_rtu_ex1_bju_cmplt_dp (iu_rtu_ex1_bju_cmplt_dp),
        .iu_rtu_ex1_bju_cmplt_for_pcgen (iu_rtu_ex1_bju_cmplt_for_pcgen),
        .iu_rtu_ex1_bju_data     (iu_rtu_ex1_bju_data),
        .iu_rtu_ex1_bju_inst_len (iu_rtu_ex1_bju_inst_len),
        .iu_rtu_ex1_bju_preg     (iu_rtu_ex1_bju_preg),
        .iu_rtu_ex1_bju_wb_dp    (iu_rtu_ex1_bju_wb_dp),
        .iu_rtu_ex1_bju_wb_vld   (iu_rtu_ex1_bju_wb_vld),
        .iu_rtu_ex1_branch_inst  (iu_rtu_ex1_branch_inst),
        .iu_rtu_ex1_cur_pc       (iu_rtu_ex1_cur_pc),
        .iu_rtu_ex1_next_pc      (iu_rtu_ex1_next_pc),
        .iu_rtu_ex2_bju_ras_mispred (iu_rtu_ex2_bju_ras_mispred),
        .iu_rtu_depd_lsu_chgflow_vld (iu_rtu_depd_lsu_chgflow_vld),
        .iu_rtu_depd_lsu_chgflow_next_pc (iu_rtu_depd_lsu_chgflow_next_pc),

        .iu_rtu_ex1_mul_cmplt    (iu_rtu_ex1_mul_cmplt),
        .iu_rtu_ex1_mul_cmplt_dp (iu_rtu_ex1_mul_cmplt_dp),
        .iu_rtu_ex1_mul_inst_len (iu_rtu_ex1_mul_inst_len),
        .iu_rtu_ex3_mul_data     (iu_rtu_ex3_mul_data),
        .iu_rtu_ex3_mul_preg     (iu_rtu_ex3_mul_preg),
        .iu_rtu_ex3_mul_wb_vld   (iu_rtu_ex3_mul_wb_vld),

        .iu_rtu_ex1_div_cmplt    (iu_rtu_ex1_div_cmplt),
        .iu_rtu_ex1_div_cmplt_dp (iu_rtu_ex1_div_cmplt_dp),
        .iu_rtu_ex1_div_inst_len (iu_rtu_ex1_div_inst_len),
        .iu_rtu_div_data         (iu_rtu_div_data),
        .iu_rtu_div_preg         (iu_rtu_div_preg),
        .iu_rtu_div_wb_dp        (iu_rtu_div_wb_dp),
        .iu_rtu_div_wb_vld       (iu_rtu_div_wb_vld),

        .rtu_iu_mul_wb_grant     (rtu_iu_mul_wb_grant),
        .rtu_iu_div_wb_grant     (rtu_iu_div_wb_grant),

        .rtu_iu_ex1_cmplt        (rtu_iu_ex1_cmplt),
        .rtu_iu_ex1_inst_len     (rtu_iu_ex1_inst_len),
        .rtu_iu_ex1_inst_split   (rtu_iu_ex1_inst_split),

        .lsu_rtu_ex1_cmplt       (lsu_rtu_ex1_cmplt),
        .lsu_rtu_ex1_cmplt_dp    (lsu_rtu_ex1_cmplt_dp),
        .lsu_rtu_ex1_cmplt_for_pcgen (lsu_rtu_ex1_cmplt_for_pcgen),
        .lsu_rtu_ex1_inst_len    (lsu_rtu_ex1_inst_len),
        .lsu_rtu_wb_data         (lsu_rtu_wb_data),
        .lsu_rtu_wb_preg         (lsu_rtu_wb_preg),
        .lsu_rtu_wb_vld          (lsu_rtu_wb_vld),
        .lsu_rtu_ex2_data        (lsu_rtu_ex2_data),
        .lsu_rtu_ex2_data_vld    (lsu_rtu_ex2_data_vld),
        .lsu_rtu_ex2_dest_reg    (lsu_rtu_ex2_dest_reg),
        .lsu_rtu_expt_vld        (lsu_rtu_expt_vld),
        .lsu_rtu_expt_vec        (lsu_rtu_expt_vec),
        .lsu_rtu_tval            (lsu_rtu_tval),
        .lsu_rtu_async_expt_vld  (lsu_rtu_async_expt_vld),
        .lsu_rtu_async_ld_inst   (lsu_rtu_async_ld_inst),

        .rtu_lsu_expt_ack        (rtu_lsu_expt_ack),
        .rtu_lsu_expt_exit       (rtu_lsu_expt_exit),

        .cp0_rtu_ex1_cmplt_dp    (cp0_rtu_ex1_cmplt_dp),
        .cp0_rtu_ex1_inst_len    (cp0_rtu_ex1_inst_len),
        .cp0_rtu_ex1_wb_data     (cp0_rtu_ex1_wb_data),
        .cp0_rtu_ex1_wb_preg     (cp0_rtu_ex1_wb_preg),
        .cp0_rtu_ex1_wb_vld      (cp0_rtu_ex1_wb_vld),
        .cp0_rtu_ex1_expt_vld    (cp0_rtu_ex1_expt_vld),
        .cp0_rtu_ex1_expt_int    (cp0_rtu_ex1_expt_int),
        .cp0_rtu_ex1_expt_vec    (cp0_rtu_ex1_expt_vec),
        .cp0_rtu_ex1_chgflw      (cp0_rtu_ex1_chgflw),
        .cp0_rtu_ex1_chgflw_pc   (cp0_rtu_ex1_chgflw_pc),
        .cp0_rtu_trap_pc         (cp0_rtu_trap_pc),

        .rtu_yy_xx_expt_vld      (rtu_yy_xx_expt_vld),
        .rtu_yy_xx_expt_int      (rtu_yy_xx_expt_int),
        .rtu_yy_xx_expt_vec      (rtu_yy_xx_expt_vec),
        .rtu_yy_xx_flush_fe      (rtu_yy_xx_flush_fe),
        .rtu_yy_xx_flush         (rtu_yy_xx_flush),
        .rtu_yy_xx_dbgon         (),
        .rtu_cp0_epc             (rtu_cp0_epc),
        .rtu_cp0_tval            (rtu_cp0_tval),
        .rtu_cp0_inst_retire     (rtu_cp0_inst_retire),

        .rtu_ifu_chgflw_vld      (rtu_ifu_chgflw_vld),
        .rtu_ifu_chgflw_pc       (rtu_ifu_chgflw_pc),
        .rtu_ifu_flush_fe        (rtu_ifu_flush_fe),

        .rtu_idu_fwd0_data       (rtu_idu_fwd0_data),
        .rtu_idu_fwd0_reg        (rtu_idu_fwd0_reg),
        .rtu_idu_fwd0_vld        (rtu_idu_fwd0_vld),
        .rtu_idu_fwd1_data       (rtu_idu_fwd1_data),
        .rtu_idu_fwd1_reg        (rtu_idu_fwd1_reg),
        .rtu_idu_fwd1_vld        (rtu_idu_fwd1_vld),
        .rtu_idu_fwd2_data       (rtu_idu_fwd2_data),
        .rtu_idu_fwd2_reg        (rtu_idu_fwd2_reg),
        .rtu_idu_fwd2_vld        (rtu_idu_fwd2_vld),
        .rtu_idu_wb0_data        (rtu_idu_wb0_data),
        .rtu_idu_wb0_reg         (rtu_idu_wb0_reg),
        .rtu_idu_wb0_vld         (rtu_idu_wb0_vld),
        .rtu_idu_wb1_data        (rtu_idu_wb1_data),
        .rtu_idu_wb1_reg         (rtu_idu_wb1_reg),
        .rtu_idu_wb1_vld         (rtu_idu_wb1_vld),
        .rtu_idu_flush_fe        (rtu_idu_flush_fe),
        .rtu_idu_flush_stall     (rtu_idu_flush_stall),
        .rtu_idu_flush_wbt       (rtu_idu_flush_wbt),
        .rtu_idu_commit          (rtu_idu_commit),
        .rtu_idu_commit_for_bju  (rtu_idu_commit_for_bju),
        .rtu_idu_pipeline_empty  (rtu_idu_pipeline_empty)
    );

    //=========================================================================
    // CSR instance: the minimal M-mode CSR file (Task 2). Drives the
    // I-side config bank (re-pointed from FetchSink's harness bank per
    // design doc S2.3.6's open integration item), the BPU config bank,
    // the LSU MHCR/MXSTATUS wires, and cp0_xx_mrvbr (reset vector, for
    // real from mrvbr). mtip/msip/meip land in its mip wiring (contract 7).
    // RESET_VECTOR is passed through from this module's own parameter so
    // mrvbr's reset value matches what IFU/BJU/ICache seed from (the same
    // value FetchSink's TOHOST-era RESET_VECTOR parameter used to carry).
    //=========================================================================
    CSR #(
        .RESET_VECTOR (RESET_VECTOR)
    ) u_csr (
        .clk                     (clk),
        .rst_n                   (rst_n),

        .idu_cp0_ex1_sel         (idu_cp0_ex1_sel),
        .idu_cp0_ex1_func        (idu_cp0_ex1_func),
        .idu_cp0_ex1_opcode      (idu_cp0_ex1_opcode),
        .idu_cp0_ex1_illegal     (idu_cp0_ex1_illegal),
        .idu_cp0_ex1_fetch_pgflt  (idu_cp0_ex1_fetch_pgflt),
        .idu_cp0_ex1_fetch_accflt (idu_cp0_ex1_fetch_accflt),
        .idu_cp0_ex1_src0_data   (idu_cp0_ex1_src0_data),
        .idu_cp0_ex1_src1_data   (idu_cp0_ex1_src1_data),
        .idu_cp0_ex1_dst0_reg    (idu_cp0_ex1_dst0_reg),
        .idu_cp0_ex1_inst_len    (idu_cp0_ex1_inst_len),
        .cp0_idu_fencei_full     (cp0_idu_fencei_full),

        .iu_cp0_ex1_cur_pc       (iu_cp0_ex1_cur_pc),

        .cp0_rtu_ex1_cmplt_dp    (cp0_rtu_ex1_cmplt_dp),
        .cp0_rtu_ex1_inst_len    (cp0_rtu_ex1_inst_len),
        .cp0_rtu_ex1_wb_data     (cp0_rtu_ex1_wb_data),
        .cp0_rtu_ex1_wb_preg     (cp0_rtu_ex1_wb_preg),
        .cp0_rtu_ex1_wb_vld      (cp0_rtu_ex1_wb_vld),
        .cp0_rtu_ex1_expt_vld    (cp0_rtu_ex1_expt_vld),
        .cp0_rtu_ex1_expt_int    (cp0_rtu_ex1_expt_int),
        .cp0_rtu_ex1_expt_vec    (cp0_rtu_ex1_expt_vec),
        .cp0_rtu_ex1_chgflw      (cp0_rtu_ex1_chgflw),
        .cp0_rtu_ex1_chgflw_pc   (cp0_rtu_ex1_chgflw_pc),

        .cp0_rtu_trap_pc         (cp0_rtu_trap_pc),

        .rtu_yy_xx_expt_vld      (rtu_yy_xx_expt_vld),
        .rtu_yy_xx_expt_int      (rtu_yy_xx_expt_int),
        .rtu_yy_xx_expt_vec      (rtu_yy_xx_expt_vec),
        .rtu_yy_xx_flush_fe      (rtu_yy_xx_flush_fe),
        .rtu_yy_xx_flush         (rtu_yy_xx_flush),
        .rtu_cp0_epc             (rtu_cp0_epc),
        .rtu_cp0_tval            (rtu_cp0_tval),
        .rtu_cp0_inst_retire     (rtu_cp0_inst_retire),

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

        .cp0_lsu_dcache_en       (cp0_lsu_dcache_en),
        .cp0_lsu_mm              (cp0_lsu_mm),
        .cp0_lsu_wa              (cp0_lsu_wa),
        .cp0_lsu_dcache_pref_en  (cp0_lsu_dcache_pref_en),
        .cp0_lsu_dcache_pref_dist(cp0_lsu_dcache_pref_dist),
        .cp0_lsu_amr             (cp0_lsu_amr),
        .pmp_cfg0_wen            (pmp_cfg0_wen),
        .pmp_cfg0_wdata          (pmp_cfg0_wdata),
        .pmp_addr_wen            (pmp_addr_wen),
        .pmp_addr_wdata          (pmp_addr_wdata),
        .pmp_addr_rsel           (pmp_addr_rsel),
        .pmp_cfg0_value          (pmp_cfg0_value),
        .pmp_addr_value          (pmp_addr_value),
        .cp0_pmp_priv_mode       (cp0_pmp_priv_mode),
        .cp0_yy_priv_mode        (cp0_yy_priv_mode),
        .cp0_mmu_satp_data       (cp0_mmu_satp_data),
        .cp0_mmu_satp_wen        (cp0_mmu_satp_wen),
        .cp0_mmu_mxr             (cp0_mmu_mxr),
        .cp0_mmu_sum             (cp0_mmu_sum),
        .cp0_mmu_sfence_vld      (cp0_mmu_sfence_vld),
        .mmu_cp0_sfence_done     (mmu_cp0_sfence_done),
        .cp0_lsu_mprv            (cp0_lsu_mprv),
        .cp0_lsu_mpp             (cp0_lsu_mpp),
        .lsu_cp0_stb_empty       (lsu_cp0_stb_empty),
        .cp0_lsu_dcache_clean    (cp0_lsu_dcache_clean),
        .lsu_cp0_clean_done      (lsu_cp0_clean_done),

        .cp0_xx_mrvbr            (cp0_xx_mrvbr),

        .mtip                    (mtip),
        .msip                    (msip),
        .meip                    (meip)
    );

    //=========================================================================
    // PMP instance (M4 Task 2, wired for real at Task 5): 8-entry physical
    // memory protection. The pmpcfg/pmpaddr storage lives here; CSR.v
    // decodes/strobes/reads back. chk_fetch_*/chk_data_*/chk_load/chk_store
    // now come from MMU.v's own PMP check channels (SECTION 7 of MMU.v --
    // time-shared among the mach/bare identity path, the TLB-hit live
    // re-check, and the walker's per-level PT-page check, D9/P14); the deny
    // outputs feed straight back into MMU.v's pmp_mmu_fetch_deny/
    // pmp_mmu_data_deny inputs (see the u_mmu instance above). No PMP
    // regions are configured at reset (M-mode default flg=0111), so the
    // OFF-path battery is unaffected by this wiring becoming real.
    //=========================================================================
    PMP u_pmp (
        .clk                (clk),
        .rst_n              (rst_n),
        .pmpcfg0_wen        (pmp_cfg0_wen),
        .pmpcfg0_wdata      (pmp_cfg0_wdata),
        .pmpaddr_wen        (pmp_addr_wen),
        .pmpaddr_wdata      (pmp_addr_wdata),
        .pmp_cfg0_value     (pmp_cfg0_value),
        .pmpaddr_rsel       (pmp_addr_rsel),
        .pmp_addr_value     (pmp_addr_value),
        .priv_mode          (cp0_pmp_priv_mode),
        .data_priv_mode     (mmu_pmp_data_priv_mode),
        .chk_fetch_pa       (mmu_pmp_fetch_pa),
        .chk_fetch_vld      (mmu_pmp_fetch_vld),
        .chk_data_pa        (mmu_pmp_data_pa),
        .chk_load           (mmu_pmp_load),
        .chk_store          (mmu_pmp_store),
        .chk_data_vld       (mmu_pmp_data_vld),
        .pmp_fetch_deny     (pmp_fetch_deny),
        .pmp_data_deny      (pmp_data_deny)
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
