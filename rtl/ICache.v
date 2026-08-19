//=============================================================================
// ICache.v - 32 KB 2-way VIPT L1 instruction cache  (M1 SKELETON: ports frozen)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_ifu_icache.v            (monolithic: tag/data SRAM
//                                                wrappers, hit judge, refill
//                                                FSM, prefetch FSM, CP0
//                                                invalidate/diag-read FSM,
//                                                AXI (BIU) master)
//   gen_rtl/ifu/rtl/aq_ifu_icache_tag_array.v  (1x aq_spsram_256x59)
//   gen_rtl/ifu/rtl/aq_ifu_icache_data_array.v (4x aq_spsram_2048x32)
// References: IFU pipeline extraction notes S1 (SRAM inventory), S3 (full
// geometry, tag row, refill, invalidate), S9 (read-directly spans). Body
// arrives in plan Task 2; this file freezes the port list only. Port names
// and widths below are the REAL aq_ifu_icache.v port list (read directly,
// icache.v:16-81 for the module header, :94-155 for widths) -- not a
// C910-by-analogy guess.
//
// SEAM NOTE (rv906 decomposition, differs from C910/rv12's split):
//  * C906's ICache is architecturally ONE monolithic module with its own
//    internal 2-cycle request/hit-check pipeline (IFU notes S0, S2 item 3) --
//    there is no separate IF/IP module pair to split the tag COMPARE across,
//    the way rv12 did for C910. rv906 keeps the whole tag/data/hit-check/
//    refill/invalidate/AXI-master function inside this ONE module, matching
//    the donor's own module boundary exactly (a smaller seam decision than
//    rv12's, not a bigger one).
//  * The MMU-facing port group (`ifu_mmu_*`/`mmu_ifu_*`) is a port of THIS
//    module, not of IFU.v -- confirmed from aq_ifu_icache.v's own port list
//    (icache.v:74-81,113-116,153-155: `mmu_ifu_pa` is used directly at
//    icache.v:729, `icache_pa[39:0] = {mmu_ifu_pa[27:0],
//    icache_rd_addr[11:0]}`). This differs from rv12's C910 seam, where
//    translation lived in the IFU (ifdp) because C910 splits tag-compare
//    across IF/IP stage modules; C906 has no such split, so the boundary
//    that was arbitrary for C910 is a confirmed structural fact here.
//  * Dropped entirely (per umbrella spec S6.3's no-clock-gating-cells rule,
//    and the parent design's HAD/debug and performance-tuning fencing):
//    `cpurst_b` (redundant with `rst_n`), `forever_cpuclk`/`cp0_ifu_icg_en`/
//    `pad_yy_icg_scan_en` (gated-clock/scan infrastructure), every `_gate`
//    companion signal (`icache_pcgen_grant_gate` etc. -- the low-power
//    clock-gating variant of an already-present enable), `hpcp_ifu_cnt_en`/
//    `ifu_hpcp_icache_access`/`ifu_hpcp_icache_miss` (perf counters, M8's
//    job), `cp0_ifu_icache_read_*`/`ifu_cp0_icache_read_data*`/
//    `icache_top_*` (the CP0 diagnostic cache-line-read path -- HAD-adjacent
//    debug infrastructure, deferred with the rest of HAD to M7, same
//    precedent rv12 set for C910's equivalent), `cp0_ifu_lpmd_req` (low-power
//    mode, not modeled), `ifu_yy_xx_no_op`, `icache_btb_grant`/
//    `icache_pred_inst_vld` (BTB/pred read their own PC-driven enable
//    straight from PCGEN inside IFU.v per the BPU notes' arbitration
//    description -- not gated through ICache; see IFU.v's header).
//=============================================================================

import rvproc_pkg::*;

module ICache #(
    parameter DATA_WIDTH = 512,             // rv906 SoC bus width (spec dev. 1)
    parameter ADDR_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // CP0 chicken bits + invalidate handshake (direct CP0 fan-out, per the
    // real RTL -- NOT routed through IFU's ctrl hub, IFU notes S5.1). In M1
    // these are driven by FetchSink's config bank (plan "Global contracts").
    //=========================================================================
    input  wire                     cp0_ifu_icache_en,      // icache.v port
    input  wire                     cp0_ifu_iwpe,            // way pred/bypass; 0 in M1
    input  wire                     cp0_ifu_icache_pref_en,  // next-line prefetch enable
    input  wire [63:0]              cp0_ifu_icache_inv_addr,
    input  wire                     cp0_ifu_icache_inv_req,  // fence.i / icache.iall
    input  wire [1:0]               cp0_ifu_icache_inv_type,
    output wire                     ifu_cp0_icache_inv_done,

    //=========================================================================
    // IFU -> ICache : fetch request, PCGEN-timed (IFU notes S2)
    //=========================================================================
    // pcgen_icache_va[63:0] = pcgen_fetch_pc[63:0] (icache.v:120); only bits
    // [39:0] are architecturally meaningful (PC_WIDTH). pcgen_icache_seq_tag
    // = pcgen_ifpc[39:6], the line tag (pcgen.v:311).
    input  wire [63:0]              pcgen_icache_va,
    input  wire [33:0]              pcgen_icache_seq_tag,
    input  wire                     pcgen_icache_chgflw_vld,
    input  wire                     ctrl_icache_req_vld,    // = ctrl_inst_fetch
    input  wire                     ctrl_icache_abort,      // = ctrl_if_cancel

    //=========================================================================
    // ICache -> IFU : grant / feedback to PCGEN
    //=========================================================================
    output wire                     icache_pcgen_grant,
    output wire [39:0]              icache_pcgen_addr,
    output wire                     icache_pcgen_inst_vld,
    output wire                     icache_ctrl_stall,      // -> ctrl_if_stall term

    //=========================================================================
    // ICache -> IFU : data output to IPACK (icache_ipack_inst is the ENTIRE
    // per-cycle bus -- 32 bits/2 halfwords, IFU notes S3/S4.1; no separate
    // predecode array, RVC boundaries are computed live downstream in IPACK)
    //=========================================================================
    output wire [31:0]              icache_ipack_inst,
    output wire                     icache_ipack_inst_vld,
    output wire                     icache_ipack_acc_err,
    output wire                     icache_ipack_pgflt,
    output wire                     icache_ipack_unalign,

    //=========================================================================
    // MMU/ITLB stub interface (icache.v's own ports; see SEAM NOTE above).
    // M1 drives a zero-latency bare-physical-mapping stub from RVProc.v
    // (design doc S2.1/S3); M4 swaps the implementation without touching
    // this port list.
    //=========================================================================
    output wire                     ifu_mmu_abort,
    output wire [MMU_VA_WIDTH-1:0]  ifu_mmu_va,
    output wire                     ifu_mmu_va_vld,
    input  wire                     mmu_ifu_access_fault,
    input  wire [MMU_PA_WIDTH-1:0]  mmu_ifu_pa,
    input  wire                     mmu_ifu_pa_vld,
    input  wire [MMU_PROT_WIDTH-1:0] mmu_ifu_prot,

    //=========================================================================
    // AXI read master (SoC I-side, ch[0]); channel names/widths copied from
    // TestMaster.v's axi_i_* group so RVProc.v passes them straight through.
    // Read-only master: the write channels of the I-side port stay in
    // RVProc.v (ICache never writes memory). Spec deviation 1: ONE 512-bit
    // single-beat read per 64B line, sliced internally (plan Task 2.1).
    //=========================================================================
    output wire                     axi_i_arvalid,
    input  wire                     axi_i_arready,
    output wire [ADDR_WIDTH-1:0]    axi_i_araddr,
    output wire [7:0]               axi_i_arlen,
    output wire [2:0]               axi_i_arsize,
    output wire [1:0]               axi_i_arburst,
    output wire [3:0]               axi_i_arcache,
    output wire [2:0]               axi_i_arprot,

    input  wire                     axi_i_rvalid,
    output wire                     axi_i_rready,
    input  wire [DATA_WIDTH-1:0]    axi_i_rdata,
    input  wire [1:0]               axi_i_rresp,
    input  wire                     axi_i_rlast
);

    //=========================================================================
    // SKELETON BODY (plan Task 2 replaces it): every output inactive/miss.
    //=========================================================================
    assign ifu_cp0_icache_inv_done = 1'b0;

    assign icache_pcgen_grant     = 1'b0;   // never grants -> PCGEN never advances
    assign icache_pcgen_addr      = 40'd0;
    assign icache_pcgen_inst_vld  = 1'b0;
    assign icache_ctrl_stall      = 1'b0;

    assign icache_ipack_inst      = 32'd0;
    assign icache_ipack_inst_vld  = 1'b0;
    assign icache_ipack_acc_err   = 1'b0;
    assign icache_ipack_pgflt     = 1'b0;
    assign icache_ipack_unalign   = 1'b0;

    assign ifu_mmu_abort  = 1'b0;
    assign ifu_mmu_va     = {MMU_VA_WIDTH{1'b0}};
    assign ifu_mmu_va_vld = 1'b0;

    assign axi_i_arvalid  = 1'b0;
    assign axi_i_araddr   = {ADDR_WIDTH{1'b0}};
    assign axi_i_arlen    = 8'd0;    // single beat (deviation 1)
    assign axi_i_arsize   = 3'd6;    // 64 bytes
    assign axi_i_arburst  = 2'b01;   // INCR
    assign axi_i_arcache  = 4'd0;
    assign axi_i_arprot   = 3'd0;
    assign axi_i_rready   = 1'b1;

endmodule
