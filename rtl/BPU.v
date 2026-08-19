//=============================================================================
// BPU.v - branch prediction unit: BHT + BTB + RAS + arbitration
//                                                (M1 SKELETON: ports frozen)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_ifu_bht.v + aq_ifu_bht_array.v   (1x aq_spsram_1024x16)
//   gen_rtl/ifu/rtl/aq_ifu_btb.v + aq_ifu_btb_entry.v   (16 flop entries, CAM)
//   gen_rtl/ifu/rtl/aq_ifu_ras.v + aq_ifu_ras_entry.v   (4 flop entries)
//   gen_rtl/ifu/rtl/aq_ifu_pred.v                        (arbitration; also
//                                                          instantiates the
//                                                          out-of-scope
//                                                          aq_ifu_pre_decd.v)
// References: BPU extraction notes S0 (inventory), S1 (BHT+GHR), S2 (BTB),
// S3 (RAS), S4 (arbitration), S6 ("16Kb" resolution), S8 (chicken bits).
// Body arrives in plan Tasks 7 (RAS), 8 (BTB), 9 (BHT); this file freezes
// the port list only.
//
// SEAM NOTES (rv906 decomposition):
//  * C906's predictor is three SMALL, INDEPENDENT, differently-sized
//    structures (BPU notes S0 headline), not one unified table family like
//    C910's -- BHT is the only SRAM-backed one; BTB and RAS are pure flop
//    arrays (confirmed zero SRAM instantiations in either file, BPU notes
//    S2.1/S3.1). Do not port C910/rv12's BTB.v (SRAM set-associative) or
//    RAS.v (12+6 copy-back-repair) structure shapes onto these -- see the
//    per-structure comments below.
//  * `aq_ifu_pre_decd.v` (branch/jump/link/return classification + immediate
//    decode) is NOT instantiated in `aq_ifu_top.v` at all -- it lives inside
//    `aq_ifu_pred.v` (IFU pipeline notes S1), i.e. structurally inside THIS
//    module's scope. BPU.v therefore owns ALL classification/immediate-
//    decode logic; IFU.v forwards only the raw fetched-bundle view
//    (`ipack_pred_inst0/1`) and the current PC (`pred_idpc`) -- see IFU.v's
//    header for why this is a confirmed structural fact, not an arbitrary
//    seam choice.
//  * Two output redirect channels, not one (BPU notes S4.3): `chgflw` (BHT/
//    BTB, PCGEN priority level 2) is distinct from `curflw` (RAS return +
//    the same-bundle delay-replay of S4.2, PCGEN priority level 3). RAS
//    bypasses BTB entirely -- they never compare notes on the same
//    instruction (BPU notes S4.1).
//  * Confirmed DEAD ports, NOT cloned: `pred_bht_pc`/`iu_ifu_bht_cur_pc`
//    (grepped with zero hits outside the port list in the real RTL, BPU
//    notes S1.4) -- this BHT is pure-GHR-indexed despite the manual's
//    "Gshared" name.
//  * Dropped/deferred for M1: RAS has NO invalidate signal at all in the
//    real RTL (confirmed, BPU notes S3.3) -- do not add one. `_gate`
//    companion signals on every `iu_ifu_*` input (umbrella spec S6.3).
//    Priv-mode gating on RAS/BTB is not modeled in M1 (no privilege levels
//    below M-mode exist yet).
//=============================================================================

import rvproc_pkg::*;

module BPU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // CP0 chicken bits (BPU notes S8; the bring-up ladder drives these via
    // FetchSink's config bank in M1). RAS has no enable bit distinct from
    // "always on" in the real RTL either -- `cp0_ifu_ras_en` gates
    // `pred_ras_tar` degenerating to `pred_idpc` when off (BPU notes S4.4).
    //=========================================================================
    input  wire                     cp0_ifu_bht_en,
    input  wire                     cp0_ifu_btb_en,
    input  wire                     cp0_ifu_ras_en,
    input  wire                     cp0_ifu_bht_inv,        // 1024-cycle sweep
    input  wire                     cp0_ifu_btb_clr,        // instant, all 16 entries
    output wire                     bht_cp0_inv_done,        // PLACEHOLDER name (S8 note below)

    //=========================================================================
    // IFU -> BPU : PCGEN-stage BTB read + the raw fetched-bundle view (BPU
    // notes S4; pre_decd runs entirely inside this module, see header)
    //=========================================================================
    input  wire [PC_WIDTH-1:0]      pcgen_btb_ifpc,
    input  wire [PC_WIDTH-1:0]      pred_idpc,
    input  wire [31:0]              ipack_pred_inst0,
    input  wire                     ipack_pred_inst0_vld,
    input  wire [15:0]              ipack_pred_inst1,
    input  wire                     ipack_pred_inst1_vld,
    input  wire                     ipack_pred_h0_create,
    input  wire                     ipack_pred_h0_vld,
    input  wire                     ipack_pred_unalign,

    //=========================================================================
    // BPU -> IFU : the two redirect channels (BPU notes S4.3) + IPACK/IBUF
    // gating
    //=========================================================================
    output wire                     pred_pcgen_chgflw_vld,
    output wire [PC_WIDTH-1:0]      pred_pcgen_chgflw_pc,
    output wire                     pred_pcgen_curflw_vld,
    output wire [PC_WIDTH-1:0]      pred_pcgen_curflw_pc,
    output wire                     pred_ctrl_stall,
    output wire                     pred_ipack_ret_stall,
    output wire                     pred_ipack_delay_stall,
    output wire                     pred_ipack_mask,
    output wire                     pred_ibuf_chgflw_vld0,
    output wire [1:0]               pred_ibuf_br_taken0,
    output wire [1:0]               pred_ibuf_br_taken1,

    //=========================================================================
    // BJU update bus (M1: FetchSink's fake BJU; M2: the real IU). Confirmed
    // names/uses: `iu_ifu_br_vld` fires for EVERY resolved conditional
    // branch (BHT GHR shift + counter update); `iu_ifu_bht_taken` is the
    // resolved outcome; `iu_ifu_bht_pred` is the captured {A,B} predicted
    // counter state fed back for the update case table (BPU notes S1.5);
    // `iu_ifu_link_vld`/`iu_ifu_ret_vld` advance RAS's confirmed pointer
    // (BPU notes S3.2); `iu_ifu_bht_mispred`/`iu_ifu_pc_mispred` snap RAS's
    // speculative pointer back to the confirmed one.
    //=========================================================================
    input  wire                     iu_ifu_br_vld,
    input  wire                     iu_ifu_bht_taken,
    input  wire [1:0]               iu_ifu_bht_pred,
    input  wire                     iu_ifu_bht_mispred,
    input  wire                     iu_ifu_pc_mispred,
    input  wire                     iu_ifu_link_vld,
    input  wire                     iu_ifu_ret_vld,

    //=========================================================================
    // RTU flush (M1: FetchSink's fake RTU; M2: the real RTU) -- also snaps
    // RAS's speculative pointer back to the confirmed one (BPU notes S3.2).
    //=========================================================================
    input  wire                     rtu_ifu_flush_fe
);

    //=========================================================================
    // SKELETON BODY (plan Tasks 7-9 replace it): every predictor reports
    // "miss / not taken / no redirect", the rung-1 configuration.
    //=========================================================================
    // PLACEHOLDER (best-effort, flag before Task 9 depends on it): the BPU
    // extraction note never quotes an explicit "done" port name for the BHT
    // invalidate sweep FSM (aq_ifu_bht.v:319-382 confirms the FSM exists,
    // not its exact output port name) -- `bht_cp0_inv_done` is named by
    // analogy with ICache.v's confirmed `ifu_cp0_icache_inv_done`. Confirm
    // the real name against aq_ifu_bht.v's port list directly in Task 9.1.
    assign bht_cp0_inv_done = 1'b0;

    assign pred_pcgen_chgflw_vld = 1'b0;
    assign pred_pcgen_chgflw_pc  = {PC_WIDTH{1'b0}};
    assign pred_pcgen_curflw_vld = 1'b0;
    assign pred_pcgen_curflw_pc  = {PC_WIDTH{1'b0}};
    assign pred_ctrl_stall       = 1'b0;
    assign pred_ipack_ret_stall  = 1'b0;
    assign pred_ipack_delay_stall= 1'b0;
    assign pred_ipack_mask       = 1'b0;
    assign pred_ibuf_chgflw_vld0 = 1'b0;
    assign pred_ibuf_br_taken0   = 2'd0;
    assign pred_ibuf_br_taken1   = 2'd0;

endmodule
