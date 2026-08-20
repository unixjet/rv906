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
    // IFU -> BPU : TASK 7.1 PORT-FREEZE AMENDMENT. Real C906 fans
    // `ibuf_ipack_stall` (aq_ifu_ibuf.v's own room-check output) into BOTH
    // `aq_ifu_ipack.v` AND `aq_ifu_pred.v` (confirmed: `aq_ifu_pred.v` reads
    // a wire of this EXACT name at multiple sites -- `pred_bht_br_vld`,
    // `pred_ras_link_vld`, `pred_ras_ret_vld` all gate on `!ibuf_ipack_stall`
    // directly). Task 1's port freeze missed this fan-out (BPU.v had no
    // stall-visibility port at all) because nothing before Task 7 needed
    // prediction to react to backpressure -- RAS is the first structure that
    // must not re-fire a push/pop for a bundle IPACK is still holding stable
    // across a stall (`--sink-stall`/IBUF-full both hold IPACK's entries
    // valid-but-not-retired, so `ipack_pred_inst0/1` would otherwise repeat
    // identically for multiple cycles). Added here with the real signal's
    // own name for direct traceability; IFU.v/RVProc.v amended to match
    // (same precedent as the FetchSink `r_stall` wiring fix and the BTB
    // chgflw-channel note elsewhere in this file's header).
    //=========================================================================
    input  wire                     ibuf_ipack_stall,

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
    // IFU -> BPU : TASK 7.1 PORT-FREEZE AMENDMENT #2. Real aq_ifu_pred.v's
    // RAS-busy stall FSM clears on `pcgen_pred_flush_vld = rtu_ifu_chgflw_vld
    // || iu_ifu_tar_pc_vld` (aq_ifu_pred.v:667, :317-318) -- NOT a BHT/BTB-
    // specific signal despite living next to the chgflw arbitration code;
    // `iu_ifu_tar_pc_vld` is IU/BJU's own delayed-resolve redirect (already
    // live at every rung via FetchSink's fake BJU). Missing this term is a
    // real, load-bearing bug found during Task 7.3 bring-up (see the task's
    // completion report): a predicted return that turns out to be WRONG-PATH
    // (squashed by IU's redirect before FetchSink ever commits it) never
    // generates the `iu_ifu_ret_vld` this FSM's WAIT state is waiting for --
    // without also clearing on the redirect itself, RAS_WAIT deadlocks
    // forever (pred_ret_stall stays asserted, freezing IPACK, so nothing new
    // ever reaches FetchSink to eventually confirm or supersede the stale
    // prediction). Task 1's port freeze missed this the same way it missed
    // `ibuf_ipack_stall` -- nothing before Task 7 needed BPU to react to a
    // redirect's OWN validity pulse, only to its mispredict-classification
    // sub-signals (iu_ifu_bht_mispred/iu_ifu_pc_mispred, already ports).
    //=========================================================================
    input  wire                     iu_ifu_tar_pc_vld,

    //=========================================================================
    // RTU flush (M1: FetchSink's fake RTU; M2: the real RTU) -- also snaps
    // RAS's speculative pointer back to the confirmed one (BPU notes S3.2).
    //=========================================================================
    input  wire                     rtu_ifu_flush_fe
);

    //=========================================================================
    // PLACEHOLDER (best-effort, flag before Task 9 depends on it): the BPU
    // extraction note never quotes an explicit "done" port name for the BHT
    // invalidate sweep FSM (aq_ifu_bht.v:319-382 confirms the FSM exists,
    // not its exact output port name) -- `bht_cp0_inv_done` is named by
    // analogy with ICache.v's confirmed `ifu_cp0_icache_inv_done`. Confirm
    // the real name against aq_ifu_bht.v's port list directly in Task 9.1.
    //=========================================================================
    assign bht_cp0_inv_done = 1'b0;

    //=========================================================================
    // TASKS 8-9 STILL SKELETON: the BHT/BTB "chgflw" channel (direction +
    // target for ordinary branches/jumps, BPU notes S4.1/S4.3) does not
    // exist yet -- every jal/c.j/conditional-branch redirect still comes
    // from FetchSink's fake BJU, exactly as in rungs 1-2's predecessor
    // (unaffected by RAS coming online, confirmed by the rung-1 regression
    // re-run in Task 7.3). Also unimplemented: the BHT-side ID-stage delay/
    // replay mechanism (`delay_chgflw`, BPU notes S4.2) and IPACK's
    // `pred_ipack_delay_stall`/`pred_ipack_mask` gates it drives -- both
    // read as constant-false terms below, matching their real-RTL role
    // exactly when no BHT exists (a Task-7-reduced form of aq_ifu_pred.v's
    // formulas, not a guess -- see the RAS section comments for where each
    // dropped term will need reinstating in Tasks 8/9).
    //=========================================================================
    assign pred_pcgen_chgflw_vld  = 1'b0;
    assign pred_pcgen_chgflw_pc   = {PC_WIDTH{1'b0}};
    assign pred_ipack_delay_stall = 1'b0;
    assign pred_ipack_mask        = 1'b0;
    assign pred_ibuf_chgflw_vld0  = 1'b0;
    assign pred_ibuf_br_taken0    = 2'd0;
    assign pred_ibuf_br_taken1    = 2'd0;

    // Ports with no Task 7 consumer yet (BTB's own PCGEN-stage tag read, the
    // BHT chicken bits/invalidate, IU's confirmed-branch/BHT-update bus,
    // and the RVC-boundary flag IPACK forwards purely for BHT's second-
    // lookup trick, BPU notes S1.6) -- genuinely unused until Tasks 8/9,
    // not an oversight.
    wire _t89_unused_ok = &{1'b0, pcgen_btb_ifpc, cp0_ifu_bht_en, cp0_ifu_btb_en,
                             cp0_ifu_bht_inv, cp0_ifu_btb_clr, iu_ifu_br_vld,
                             iu_ifu_bht_taken, iu_ifu_bht_pred, ipack_pred_unalign};

    //=========================================================================
    // SECTION: RAS (plan Task 7.1) -- aq_ifu_ras.v + aq_ifu_ras_entry.v +
    // the RAS-relevant slice of aq_ifu_pred.v (S3, S4.1's "RAS bypasses BTB"
    // finding, S4.4's chicken-bit note), all confirmed by reading the real
    // files directly (refs/openc906/.../aq_ifu_ras.v, aq_ifu_pred.v:602-706).
    //
    // pre_decd (this module's own scope, per the header note): pcall/preturn
    // classification is the SAME x1-only rule independently confirmed twice
    // already (rtl/FetchSink.v TASK 4.1 FINDINGS item 2; m1_iss.h's header),
    // both derived from aq_iu_bju.v:637-667 cross-checked against
    // aq_idu_cfig.h's FUNC_* bit-11 table -- reproduced here a THIRD time,
    // independently, on the SPECULATIVE ID-stage bundle instead of
    // FetchSink's committed instruction (BPU.v is the one place this
    // classification must run on wrong-path instructions too, since real
    // RAS pushes/pops speculatively at ID-stage, before any resolve -- BPU
    // notes S3.2/S4.1). A shared classification bug between this copy and
    // FetchSink's would still show up: FetchSink's OWN resolve rule (Task
    // 7.2) recomputes the actual target independently and corrects any
    // wrong RAS-driven fetch, so a divergence here is caught by the online
    // checker's committed-stream comparison, not silently absorbed.
    //=========================================================================
    typedef struct packed {
        logic jal32;      // 32-bit jal (unconditional; feeds pred_jmp_vld0
                           // only -- BPU notes S4.1's "BHT/BTB supply
                           // direction/target only", not used for chgflw here)
        logic cjump;       // c.j
        logic pcall;       // dst==x1 && func11 (RAS push)
        logic preturn;     // jalr-family && src==x1 && !(src==dst && func11)
    } ras_class_t;

    function automatic ras_class_t ras_classify;
        input [31:0] x;
        reg jal32, jalr32, cjr, cjalr, cjump;
        reg [4:0] dst_preg, src0_reg;
        reg func11, jalr_fam, src_dst_eq;
        begin
            jal32  =  (x[6:0] == 7'b1101111);
            jalr32 = ({x[14:12], x[6:0]} == 10'b000_1100111);
            cjr    = ({x[15:12], x[6:0]} == 11'b1000_0000010) && (x[11:7] != 5'b0);
            cjalr  = ({x[15:12], x[6:0]} == 11'b1001_0000010) && (x[11:7] != 5'b0);
            cjump  = ({x[15:13], x[1:0]} == 5'b10101);

            // NOTE: jal32/jalr32 (opcode [6:0] ending `11`) and
            // cjr/cjalr/cjump (requiring [1:0] != `11`) are mutually
            // exclusive by construction from the bit patterns alone -- no
            // separate is32 gate is needed inside this function (mirrors
            // FetchSink.v's fs_decode, which has the same property).
            jalr_fam = jalr32 || cjr || cjalr;

            dst_preg = jal32  ? x[11:7] :
                       jalr32 ? x[11:7] :
                       cjalr  ? 5'd1    : 5'd0;
            src0_reg = jalr32          ? x[19:15] :
                       (cjr || cjalr)  ? x[11:7]  : 5'd0;

            func11 = jal32 || jalr32 || cjalr;

            ras_classify.jal32   = jal32;
            ras_classify.cjump   = cjump;
            ras_classify.pcall   = func11 && (dst_preg == 5'd1);
            src_dst_eq           = (src0_reg == dst_preg) && func11;
            ras_classify.preturn = jalr_fam && (src0_reg == 5'd1) && !src_dst_eq;
        end
    endfunction

    // Slot 0: the full 32-bit view. No explicit is32 gate needed inside
    // ras_classify itself (matching FetchSink.v's fs_decode precedent): the
    // 32-bit encodings (jal32/jalr32, opcode bits [6:0] ending in `11`) and
    // the 16-bit encodings (cjr/cjalr/cjump, quadrant/funct bits requiring
    // [1:0] != `11`) are mutually exclusive by construction from the bit
    // patterns alone, so classifying whichever 16 or 32 bits actually sit
    // in the slot through ONE call is both sufficient and exactly what a
    // real predecoder does (it doesn't get an extra "is this 16 or 32 bit"
    // side-channel either -- it reads the same bits `ras_classify` does).
    wire is32_0 = ipack_pred_inst0[1:0] == 2'b11;
    ras_class_t c0;
    always @* c0 = ras_classify(ipack_pred_inst0);
    wire c0_jmp = c0.jal32 || c0.cjump;   // pred_jmp_vld0 (S4.1): direct unconditional jump only

    // Slot 1: the design doc's "only ever a 16-bit/RVC half" (S2.1) --
    // zero-extend straight into the same classifier.
    ras_class_t c1;
    always @* c1 = ras_classify({16'b0, ipack_pred_inst1});

    wire pred_link_vld0 = ipack_pred_inst0_vld && c0.pcall;
    wire pred_ret_vld0  = ipack_pred_inst0_vld && c0.preturn;
    wire pred_jmp_vld0  = ipack_pred_inst0_vld && c0_jmp;
    wire pred_link_vld1 = ipack_pred_inst1_vld && c1.pcall;
    wire pred_ret_vld1  = ipack_pred_inst1_vld && c1.preturn;

    // a. RAS access signals (aq_ifu_pred.v:610-628). `pred_inst0_taken` in
    // the real RTL also has a BHT-direction term (`pred_br_vld0 &&
    // bht_pred_rslt[1]`) that reduces to constant-false with no BHT
    // structure built yet (Task 9) -- `pred_jmp_vld0` (a plain jal/c.j,
    // unconditionally "taken") is the only surviving contributor at this
    // rung, so `pred_inst0_taken` == `pred_jmp_vld0` for Task 7's purposes.
    // `delay_chgflw` (S4.2, BHT-only) is likewise structurally 0 (Task 9).
    wire pred_ras_link_vld0 = pred_link_vld0;
    wire pred_ras_link_vld1 = pred_link_vld1 && !pred_jmp_vld0;
    wire pred_ras_link_vld  = (pred_ras_link_vld0 || pred_ras_link_vld1)
                             && cp0_ifu_ras_en && !ibuf_ipack_stall;

    wire pred_ras_ret_vld0   = pred_ret_vld0;                    // (real: && !delay_chgflw, ==1 here)
    wire pred_ras_ret_vld1   = pred_ret_vld1 && !pred_jmp_vld0;
    wire pred_ras_ret_chgflw = pred_ras_ret_vld0 || pred_ras_ret_vld1;
    // DELIBERATE rv906 DIVERGENCE (documented, not a silent guess): the real
    // RTL's `pred_ras_ret_vld`/`pred_curflw` have NO `cp0_ifu_ras_en` term
    // at all (only the PUSH side is gated by it, aq_ifu_pred.v:613 vs
    // :620-623) -- when RAS is chicken-bit-disabled, real silicon still
    // pops/rotates the (never-written, all-zero) entries and still fires a
    // curflw redirect, just to `pred_idpc` (S4.4: `pred_ras_tar` degenerates
    // to `pred_idpc` when the enable is off) -- a same-address "redirect"
    // that costs a wasted re-fetch cycle but changes no committed
    // instruction (harmless under the M1 oracle's own "predictors change
    // WHEN, never WHICH" rule, design doc S4.1). rv906 additionally gates
    // this path by `cp0_ifu_ras_en` so rung 1 (RAS chicken-bit off) is
    // provably 100% silent on this channel, not just harmless -- chosen
    // because (a) real hardware's looser gate is UNOBSERVABLE by any test
    // in this predictor-agnostic suite (both choices commit the identical
    // instruction stream), (b) it removes a real, analyzed livelock risk
    // specific to rv906's own IBUF clone: unlike the real one-hot IBUF, this
    // clone's IPACK entries can stay valid-but-unretired for MANY consecutive
    // cycles under `--sink-stall` backpressure (IFU.v's own documented
    // structural deviation, SECTION IBUF), and `pcgen_buf_chgflw` (IFU.v)
    // is only ever set by a *delayed* chgflw event, never by curflw -- so an
    // un-gated curflw that stays asserted across a multi-cycle stall would
    // re-latch `pcgen_ifpc` to the SAME curflw target every single cycle
    // with no advance, hanging PCGEN. The RAS's own internal pointer state
    // machine below is cloned with NO such extra gate (only the OUTPUT
    // redirect is suppressed), so the pointer-resync fidelity Task 7 is
    // about stays bit-exact to aq_ifu_ras.v.
    wire pred_ras_ret_vld = pred_ras_ret_chgflw && cp0_ifu_ras_en && !ibuf_ipack_stall;

    // pred_h0_pc / pred_cur_pc (aq_ifu_pred.v:456-484, reduced form): tracks
    // the PC of a straddle-carried slot-0 instruction (IPACK's entry0 carry,
    // IFU notes S4) so a CALL that happens to be such an instruction still
    // pushes the correct return address. The real mux's OTHER terms
    // (pred_br_taken1/pred_delay_br_raw) are BHT-only and dropped (Task 9);
    // `pred_ras_link_vld1` is kept since it is RAS's own slot-1 case.
    reg [PC_WIDTH-1:0] pred_h0_pc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pred_h0_pc <= {PC_WIDTH{1'b0}};
        else if (ipack_pred_h0_create) pred_h0_pc <= {pred_idpc[PC_WIDTH-1:2], 2'b10};
    end

    wire [PC_WIDTH-1:0] pred_cur_pc = c0_jmp ? (ipack_pred_h0_vld ? pred_h0_pc : pred_idpc)
                                    : pred_ras_link_vld1 ? {pred_idpc[PC_WIDTH-1:2], 2'b10}
                                    : pred_idpc;

    // Push value: low RAS_PC_WIDTH bits of (call PC + inst length). Real RTL
    // computes the length purely from slot0's own width and defaults to the
    // 16-bit offset whenever the call is NOT in slot0 (aq_ifu_pred.v:625-626)
    // -- correct because slot1 is architecturally always 16-bit (S2.1).
    wire [RAS_PC_WIDTH-1:0] ras_link_offset = (pred_ras_link_vld0 && is32_0)
                                             ? {{(RAS_PC_WIDTH-3){1'b0}}, 3'd4}
                                             : {{(RAS_PC_WIDTH-3){1'b0}}, 3'd2};
    wire [RAS_PC_WIDTH-1:0] pred_ras_link_pc = pred_cur_pc[RAS_PC_WIDTH-1:0] + ras_link_offset;

    // b. RAS storage + dual pointer (aq_ifu_ras.v, cloned bit-exact). One
    // physical 4-entry content array; TWO one-hot pointers sharing it:
    // `ras_pop` (speculative -- selects both the read AND, one push ahead,
    // the write slot) and `ras_bju` (confirmed -- tracks the SAME rotation
    // but only on IU-resolved events, and is NEVER used to read content --
    // it exists purely as `ras_pop`'s restore point on misprediction). Push
    // rotates the pointer by -1 (mod RAS_DEPTH); pop rotates it by +1. THE
    // DEPTH-LIMITATION MECHANISM (BPU notes S3.2, design doc S2.1/S2.3.4):
    // recovery snaps `ras_pop <= ras_bju` (POINTER only) -- if more than
    // RAS_DEPTH calls have pushed since the last one IU actually confirmed,
    // the physical slot `ras_bju` points back to has already been
    // overwritten by a later push, so the next pop after a snap-back (or
    // simply the next pop once more than RAS_DEPTH calls are simultaneously
    // un-returned, see below) reads STALE, unrelated content -- a real,
    // deterministic misprediction FetchSink's own resolve (Task 7.2) always
    // catches and corrects, never silently commits.
    localparam RAS_IDLE = 1'b0, RAS_WAIT = 1'b1;

    reg [RAS_DEPTH-1:0]    ras_pop, ras_bju;
    reg                    ras_cur_st;
    reg [RAS_PC_WIDTH-1:0] ras_entry0, ras_entry1, ras_entry2, ras_entry3;

    wire [RAS_DEPTH-1:0] ras_pop_push_next = {ras_pop[0], ras_pop[RAS_DEPTH-1:1]};   // right-rotate
    wire [RAS_DEPTH-1:0] ras_pop_pop_next  = {ras_pop[RAS_DEPTH-2:0], ras_pop[RAS_DEPTH-1]}; // left-rotate

    wire ras_snap_back = rtu_ifu_flush_fe || iu_ifu_bht_mispred
                       || (iu_ifu_pc_mispred && !iu_ifu_link_vld);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ras_pop <= {{(RAS_DEPTH-1){1'b0}}, 1'b1};
        else if (ras_snap_back)
            ras_pop <= ras_bju;
        else if (pred_ras_link_vld)
            ras_pop <= ras_pop_push_next;
        else if (pred_ras_ret_vld && (ras_cur_st == RAS_IDLE))
            ras_pop <= ras_pop_pop_next;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ras_bju <= {{(RAS_DEPTH-1){1'b0}}, 1'b1};
        else if (iu_ifu_link_vld && !rtu_ifu_flush_fe)
            ras_bju <= {ras_bju[0], ras_bju[RAS_DEPTH-1:1]};
        else if (iu_ifu_ret_vld && !rtu_ifu_flush_fe)
            ras_bju <= {ras_bju[RAS_DEPTH-2:0], ras_bju[RAS_DEPTH-1]};
    end

    // Write port: the entry the pointer is about to rotate INTO on a push
    // (i.e. `ras_pop_push_next`'s one-hot bit) receives the new content --
    // algebraically identical to aq_ifu_ras.v's four explicit
    // `entryN_upd = pred_ras_link_vld && ras_pop[k]` equations (verified by
    // hand: e.g. entry3_upd's real condition is ras_pop[0], and
    // ras_pop_push_next[3] = ras_pop[0] by the rotate's own definition).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ras_entry0 <= {RAS_PC_WIDTH{1'b0}};
            ras_entry1 <= {RAS_PC_WIDTH{1'b0}};
            ras_entry2 <= {RAS_PC_WIDTH{1'b0}};
            ras_entry3 <= {RAS_PC_WIDTH{1'b0}};
        end
        else if (pred_ras_link_vld) begin
            if (ras_pop_push_next[0]) ras_entry0 <= pred_ras_link_pc;
            if (ras_pop_push_next[1]) ras_entry1 <= pred_ras_link_pc;
            if (ras_pop_push_next[2]) ras_entry2 <= pred_ras_link_pc;
            if (ras_pop_push_next[3]) ras_entry3 <= pred_ras_link_pc;
        end
    end

    // Read port: CURRENT ras_pop selects content (aq_ifu_ras.v:238-244).
    // Real RTL's default case is 24'bx (an unreachable one-hot state given
    // the invariant reset+rotate maintains); rv906 uses a defined 0 instead
    // to stay X-free for deterministic simulation -- harmless, since the
    // invariant is never actually violated.
    reg [RAS_PC_WIDTH-1:0] ras_tar_pc;
    always @* begin
        case (ras_pop)
            4'b0001: ras_tar_pc = ras_entry0;
            4'b0010: ras_tar_pc = ras_entry1;
            4'b0100: ras_tar_pc = ras_entry2;
            4'b1000: ras_tar_pc = ras_entry3;
            default: ras_tar_pc = {RAS_PC_WIDTH{1'b0}};
        endcase
    end

    // c. RAS result (aq_ifu_pred.v:656): reconstruct the full fetch address
    // from the stored 24-bit PC by concatenating the CURRENT ID-stage
    // bundle's upper bits -- predicted returns are only ever correct within
    // the same 16MiB-aligned region as the return site (BPU notes S3.3,
    // design doc S2.1's "materially looser than BTB's 64KiB, still a real
    // limit"). Chicken-bit-off degenerates to `pred_idpc` (S4.4).
    wire [PC_WIDTH-1:0] pred_ras_tar = cp0_ifu_ras_en
                                      ? {pred_idpc[PC_WIDTH-1:RAS_PC_WIDTH], ras_tar_pc}
                                      : pred_idpc;

    // d. RAS-busy stall FSM (aq_ifu_pred.v:659-696): at most one predicted
    // return in flight through ID-stage at a time -- IDLE moves to WAIT the
    // cycle a return is predicted; WAIT holds until IU confirms it
    // (`iu_ifu_ret_vld`), stalling further IPACK retirement via
    // `pred_ipack_ret_stall` so a second return can't race the first one's
    // resolve. Flush term = real RTL's `pcgen_pred_flush_vld || rtu_ifu_
    // flush_fe` exactly (aq_ifu_pred.v:667, :317-318) -- see the port note
    // above for why `iu_ifu_tar_pc_vld` belongs here: without it, a
    // predicted return that turns out to be wrong-path (IU redirects away
    // before ever committing it) leaves WAIT with no path back to IDLE,
    // since `iu_ifu_ret_vld` for that specific instance can then never fire.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                        ras_cur_st <= RAS_IDLE;
        else if (rtu_ifu_flush_fe || iu_ifu_tar_pc_vld)     ras_cur_st <= RAS_IDLE;
        else case (ras_cur_st)
            RAS_IDLE: ras_cur_st <= pred_ras_ret_vld ? RAS_WAIT : RAS_IDLE;
            RAS_WAIT: ras_cur_st <= iu_ifu_ret_vld    ? RAS_IDLE : RAS_WAIT;
            default:  ras_cur_st <= RAS_IDLE;
        endcase
    end

    wire pred_ret_stall = (ras_cur_st == RAS_WAIT) && pred_ras_ret_vld;

    //=========================================================================
    // Output to PCGEN (BPU notes S4.3's "curflw" channel -- RAS bypasses
    // BTB/chgflw entirely). `pred_curflw` in the real RTL is
    // `pred_ras_ret_chgflw || delay_chgflw`; the OR'd BHT-delay term is
    // Task 9 territory (structurally 0 here). Consolidated to exactly
    // `pred_ras_ret_vld` (see the divergence note above) rather than the
    // real RTL's un-gated-by-stall `pred_ras_ret_chgflw` -- this also means
    // the SAME signal drives both the RAS pointer's pop-rotate and the
    // redirect pulse, so a curflw pulse and a RAS pop always happen in
    // lockstep (no orphaned redirect with no matching pointer move, or vice
    // versa).
    //=========================================================================
    assign pred_pcgen_curflw_vld = pred_ras_ret_vld;
    assign pred_pcgen_curflw_pc  = pred_ras_tar;
    assign pred_ctrl_stall       = pred_ret_stall;   // real: also ORs BHT-delay terms (Task 9); no consumer exists yet either way (IFU.v header note)
    assign pred_ipack_ret_stall  = pred_ret_stall;

endmodule
