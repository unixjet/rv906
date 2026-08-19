//=============================================================================
// IFU.v - instruction fetch unit  (M1 SKELETON: ports frozen)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_ifu_top.v      (the glue; rv906 hoists it into RVProc.v)
//   gen_rtl/ifu/rtl/aq_ifu_pcgen.v    (next-PC arbiter + redirect priority)
//   gen_rtl/ifu/rtl/aq_ifu_ctrl.v     (the whole stall/cancel hub, 132 lines)
//   gen_rtl/ifu/rtl/aq_ifu_ipack.v + _entry.v   (halfword re-slicing)
//   gen_rtl/ifu/rtl/aq_ifu_ibuf.v + _entry.v + _pop_entry.v (6-entry queue)
//   gen_rtl/ifu/rtl/aq_ifu_vec.v      (boot / reset-vector sequencer)
// References: IFU pipeline extraction notes S2 (pipeline stages, redirect
// priority S7), S4 (IPACK/IBUF), S5 (stall/flush topology), S6 (boot),
// S9 (read-directly spans); BPU extraction notes S4 (arbitration channels).
// Body arrives in plan Task 3; this file freezes the port list only.
//
// SEAM NOTES (rv906 decomposition):
//  * C906's IFU is FLAT: one small shared control hub (aq_ifu_ctrl.v, not
//    per-stage ctrl/dp pairs) and a monolithic ICache with its own internal
//    2-cycle pipeline (IFU notes S0). This file hoists pcgen + ctrl + ipack +
//    ibuf + the reduced boot FSM as SECTIONS, exactly as aq_ifu_top.v
//    instantiates them flat, one of each (top.v:459-840). Every wire that
//    still crosses a module boundary in the real RTL keeps its donor name
//    verbatim, so it stays greppable in refs/openc906.
//  * ICache seam: the MMU-facing port group and the tag/data/hit-check logic
//    live entirely in ICache.v (see its header) -- a confirmed structural
//    fact for C906, not an arbitrary rv906 seam choice the way it was for
//    C910/rv12.
//  * BPU seam: `aq_ifu_pred.v` (arbitration + predecode) is confirmed to live
//    INSIDE the predictor module set per the design doc's file org ("BPU.v:
//    BHT + BTB + RAS + the arbitration logic that combines them") -- so this
//    file forwards only the RAW fetched-bundle view (`ipack_pred_inst0/1`)
//    and the current ID-stage PC (`pred_idpc`) into BPU.v, exactly as
//    aq_ifu_ipack.v exposes them (ipack.v:456-465) -- IFU.v does NOT do any
//    branch/jump/link/return classification itself (confirmed: the real
//    `aq_ifu_pre_decd.v` is instantiated inside `aq_ifu_pred.v`, not
//    `aq_ifu_top.v` -- IFU notes S1).
//  * BTB's own PCGEN-stage tag read (`pcgen_btb_ifpc`, BPU notes S2.2) is
//    forwarded from PCGEN here as a plain PC bus; BPU.v does the [15:0] tag
//    slice and the target reconstruction itself.
//  * `idu_ifu_id_stall` is the ONLY signal crossing the IFU<->IDU boundary
//    (confirmed structurally, IFU notes S5.2) -- do not add a second one.
//  * Dropped/simplified for M1 (spec S2.2, umbrella S6.3): every `_gate`
//    companion signal; `rtu_ifu_dbg_mask`/`rtu_yy_xx_dbgon` (HAD-adjacent
//    debug, deferred to M7); the boot FSM's WARM_UP unit-pulse fan-out and
//    `ifu_rtu_reset_halt_req` (no IDU/IU/RTU/DTU exist yet to warm up or
//    halt for in M1 -- `aq_ifu_vec.v` is reduced to RESET->RUN + reset-vector
//    pcload only, same simplification rv12 made for C910's equivalent
//    module, design doc S2.1); low-power mode (`cp0_ifu_in_lpmd`/
//    `cp0_ifu_lpmd_req`) folded to "always inactive" inside the ctrl hub's
//    body rather than exposed as ports.
//  * PLACEHOLDER (best-effort, flag before Task 3/4 depend on it): the exact
//    resolve/redirect port WIDTHS below (`iu_ifu_tar_pc[63:0]`,
//    `rtu_ifu_chgflw_pc[39:0]`) are transcribed from the IFU pipeline note's
//    S5.4 port inventory, which characterizes them from aq_ifu_top.v's OWN
//    port list (in scope) rather than from `aq_iu_bju.v` (out of scope,
//    IU-side). The SIGNAL NAMES are solid; double-check the exact bit
//    layout against aq_ifu_top.v's port declarations directly before Task
//    4.1 depends on it if anything downstream looks off.
//=============================================================================

import rvproc_pkg::*;

module IFU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IFU -> IDU : single instruction/cycle (IFU notes S2 item 6, S5.2;
    // frozen for M2). `idu_ifu_id_stall` is the ONLY signal the other way.
    //=========================================================================
    output wire [31:0]              ifu_idu_id_inst,
    output wire                     ifu_idu_id_inst_vld,
    output wire [1:0]               ifu_idu_id_bht_pred,   // rides w/ the instr, ibuf.v:1357-1358
    input  wire                     idu_ifu_id_stall,

    //=========================================================================
    // IFU -> ICache : fetch request, PCGEN-timed (mirrors ICache.v exactly)
    //=========================================================================
    output wire [63:0]              pcgen_icache_va,
    output wire [33:0]              pcgen_icache_seq_tag,
    output wire                     pcgen_icache_chgflw_vld,
    output wire                     ctrl_icache_req_vld,
    output wire                     ctrl_icache_abort,

    //=========================================================================
    // ICache -> IFU : grant / data (mirrors ICache.v exactly)
    //=========================================================================
    input  wire                     icache_pcgen_grant,
    input  wire [39:0]              icache_pcgen_addr,
    input  wire                     icache_pcgen_inst_vld,
    input  wire                     icache_ctrl_stall,
    input  wire [31:0]              icache_ipack_inst,
    input  wire                     icache_ipack_inst_vld,
    input  wire                     icache_ipack_acc_err,
    input  wire                     icache_ipack_pgflt,
    input  wire                     icache_ipack_unalign,

    //=========================================================================
    // IFU -> BPU : PCGEN-stage BTB tag read + the fetched-bundle view pred.v
    // classifies internally (BPU notes S4; IFU notes S1 -- pre_decd lives in
    // BPU.v, not here).
    //=========================================================================
    output wire [PC_WIDTH-1:0]      pcgen_btb_ifpc,        // BTB slices [15:0] itself
    output wire [PC_WIDTH-1:0]      pred_idpc,             // current ID-stage PC
    output wire [31:0]              ipack_pred_inst0,      // = icache_ipack_inst, ID-stage view
    output wire                     ipack_pred_inst0_vld,
    output wire [15:0]              ipack_pred_inst1,
    output wire                     ipack_pred_inst1_vld,
    output wire                     ipack_pred_h0_create,   // straddle-carry state
    output wire                     ipack_pred_h0_vld,
    output wire                     ipack_pred_unalign,

    //=========================================================================
    // BPU -> IFU : the two redirect channels (BPU notes S4.3) + IPACK/IBUF
    // gating (IFU notes S5.4 / S8 for the pcgen priority levels these feed)
    //=========================================================================
    input  wire                     pred_pcgen_chgflw_vld,  // BHT/BTB "final" redirect (level 2)
    input  wire [PC_WIDTH-1:0]      pred_pcgen_chgflw_pc,
    input  wire                     pred_pcgen_curflw_vld,  // RAS return / delay replay (level 3)
    input  wire [PC_WIDTH-1:0]      pred_pcgen_curflw_pc,
    input  wire                     pred_ctrl_stall,        // -> ctrl_btb_stall only
    input  wire                     pred_ipack_ret_stall,
    input  wire                     pred_ipack_delay_stall,
    input  wire                     pred_ipack_mask,
    input  wire                     pred_ibuf_chgflw_vld0,
    input  wire [1:0]               pred_ibuf_br_taken0,
    input  wire [1:0]               pred_ibuf_br_taken1,

    //=========================================================================
    // IFU <- IU/BJU  (M1: FetchSink's fake BJU; M2: the real IU) - IFU
    // pipeline notes S5.4, PLACEHOLDER per the header note above.
    //=========================================================================
    input  wire                     iu_ifu_tar_pc_vld,      // redirect priority 2
    input  wire [63:0]              iu_ifu_tar_pc,
    input  wire                     iu_ifu_pc_mispred,

    //=========================================================================
    // IFU -> IU : forwards RTU's redirect onward (pcgen.v:325-326)
    //=========================================================================
    output wire                     ifu_iu_chgflw_vld,
    output wire [PC_WIDTH-1:0]      ifu_iu_chgflw_pc,

    //=========================================================================
    // IFU <- RTU  (M1: FetchSink's fake RTU; M2: the real RTU)
    //=========================================================================
    input  wire                     rtu_ifu_chgflw_vld,     // redirect priority 1 (ties over 2/3)
    input  wire [PC_WIDTH-1:0]      rtu_ifu_chgflw_pc,
    input  wire                     rtu_ifu_flush_fe,       // front-end flush (IBUF/IPACK direct)

    //=========================================================================
    // Boot / reset vector (aq_ifu_vec.v, reduced to RESET->RUN + pcload for
    // M1 -- see header note)
    //=========================================================================
    input  wire [PC_WIDTH-1:0]      cp0_xx_mrvbr
);

    //=========================================================================
    // SKELETON BODY (plan Task 3 replaces it): every output inactive.
    // Nothing is fetched, nothing is delivered, no predictor is looked up.
    //=========================================================================
    assign ifu_idu_id_inst     = 32'd0;
    assign ifu_idu_id_inst_vld = 1'b0;
    assign ifu_idu_id_bht_pred = 2'd0;

    assign pcgen_icache_va         = 64'd0;
    assign pcgen_icache_seq_tag    = 34'd0;
    assign pcgen_icache_chgflw_vld = 1'b0;
    assign ctrl_icache_req_vld     = 1'b0;
    assign ctrl_icache_abort       = 1'b0;

    assign pcgen_btb_ifpc        = {PC_WIDTH{1'b0}};
    assign pred_idpc             = {PC_WIDTH{1'b0}};
    assign ipack_pred_inst0      = 32'd0;
    assign ipack_pred_inst0_vld  = 1'b0;
    assign ipack_pred_inst1      = 16'd0;
    assign ipack_pred_inst1_vld  = 1'b0;
    assign ipack_pred_h0_create  = 1'b0;
    assign ipack_pred_h0_vld     = 1'b0;
    assign ipack_pred_unalign    = 1'b0;

    assign ifu_iu_chgflw_vld = 1'b0;
    assign ifu_iu_chgflw_pc  = {PC_WIDTH{1'b0}};

endmodule
