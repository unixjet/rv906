//=============================================================================
// BPU.v - branch prediction unit: BHT + BTB + RAS + arbitration
//                                (M1: BHT + RAS + BTB all real; ports frozen)
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
// RAS landed in plan Task 7, BTB in Task 8, BHT in Task 9 -- all three
// predictor structures and the full arbitration/delay-replay logic are real
// as of this file.
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
    // M8 T4b FIX (2nd round, port-freeze amendment #3): the PRED-stage
    // ARRIVAL event, required by the `pred_idpc_r` register below (see its
    // comment block). Donor aq_ifu_pred.v's `icache_pred_inst_vld` input
    // (aq_ifu_icache.v:1339, the ICache's PRED-stage hit/bypass level) is
    // what gates its `pred_idpc` register's load; rv906's equivalent of
    // that event is IFU.v's `icache_inst_vld` (= `icache_ipack_inst_vld
    // && !ctrl_ipack_cancel && !pred_ipack_mask`, the donor's own
    // aq_ifu_ipack.v:253 formula -- "a fresh fetch word is actually
    // landing in IPACK THIS cycle", one cycle BEFORE `ipack_pred_inst0_
    // vld` rises, which is exactly the offset the load needs). Task 1's
    // port freeze missed it because the register it feeds did not exist.
    input  wire                     icache_inst_vld,
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
    // TASK 9: BHT is now real (SECTION BHT below, near the end of this
    // file). `bht_cp0_inv_done` is driven from the invalidate FSM's own
    // `bht_inv_done` there -- the real RTL's matching output is
    // `ifu_cp0_bht_inv_done` (aq_ifu_bht.v:543), confirmed by reading the
    // module's own port list directly (Task 9.1); no rename needed since
    // this is an internal rv906 port name, not one exposed to C906 itself.
    // `pred_ipack_delay_stall`/`pred_ipack_mask`/`pred_ibuf_br_taken0/1` are
    // likewise driven for real at the bottom of this file now, from SECTION
    // BHT's `pred_delay_br_raw`/`bht_pred_rslt`.
    //
    // `pcgen_btb_ifpc` (BTB's real PCGEN-stage read address) remains
    // deliberately unused -- see SECTION BTB's own header note for why
    // rv906 collapses the real RTL's 2-stage CAM pipeline into a single
    // ID-stage-time lookup keyed off `pred_cur_pc` instead.
    //=========================================================================
    wire _t9_unused_ok = &{1'b0, pcgen_btb_ifpc};

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

    //=========================================================================
    // SECTION: BTB classification additions (plan Task 8.1). RAS's own
    // classification above (c0/c1) only ever needed link/ret/jmp for
    // SLOT 0 -- `pred_jmp_vld1` (slot 1's own jal/c.j membership) was never
    // named because nothing consumed it before BTB existed. con_br
    // (B-type / c.beqz / c.bnez) and the J/CJ/B/CB immediate decode are ALSO
    // new here -- reused verbatim from FetchSink.v's `fs_decode` (same
    // ISA-standard bit patterns; that file already cites reusing itself
    // from rv12's C910 clone for exactly this reason, style precedent).
    // Built now in the FULL form aq_ifu_pred.v itself uses (con_br AND jmp,
    // both slots), even though the BHT-dependent half of "taken" is
    // structurally zero until Task 9 (see `pred_inst0/1_taken` below) --
    // so Task 9 only has to OR in `bht_pred_rslt[1]`, nothing here changes.
    //=========================================================================
    wire c1_jmp = c1.jal32 || c1.cjump;
    wire pred_jmp_vld1 = ipack_pred_inst1_vld && c1_jmp;

    wire con_br0 = (ipack_pred_inst0[6:0] == 7'b1100011) && (ipack_pred_inst0[14:13] != 2'b01) // bxx
                || ({ipack_pred_inst0[15:14], ipack_pred_inst0[1:0]} == 4'b1101);              // c.beqz/c.bnez
    wire pred_br_vld0 = ipack_pred_inst0_vld && con_br0;

    // Slot 1 is only ever a 16-bit/RVC half (S2.1): the 32-bit B-type
    // pattern (opcode[1:0]=='11') can never appear in a genuine RVC
    // halfword (RVC's own defining property is opcode[1:0] != '11'), so
    // only the compressed c.beqz/c.bnez pattern is checked here.
    wire con_br1 = ({ipack_pred_inst1[15:14], ipack_pred_inst1[1:0]} == 4'b1101);
    wire pred_br_vld1 = ipack_pred_inst1_vld && con_br1;

    function automatic [PC_WIDTH-1:0] btb_imm;
        input [31:0] x;
        reg jal32, cjump, bxx, cbz;
        begin
            jal32 = (x[6:0] == 7'b1101111);
            cjump = ({x[15:13], x[1:0]} == 5'b10101);
            bxx   = (x[6:0] == 7'b1100011) && (x[14:13] != 2'b01);
            cbz   = ({x[15:14], x[1:0]} == 4'b1101);
            if (jal32)
                btb_imm = {{(PC_WIDTH-21){x[31]}},
                           x[31], x[19:12], x[20], x[30:21], 1'b0};
            else if (bxx)
                btb_imm = {{(PC_WIDTH-13){x[31]}},
                           x[31], x[7], x[30:25], x[11:8], 1'b0};
            else if (cjump)
                btb_imm = {{(PC_WIDTH-12){x[12]}},
                           x[12], x[8], x[10:9], x[6], x[7], x[2],
                           x[11], x[5:3], 1'b0};
            else if (cbz)
                btb_imm = {{(PC_WIDTH-9){x[12]}},
                           x[12], x[6:5], x[2], x[11:10], x[4:3], 1'b0};
            else
                btb_imm = {PC_WIDTH{1'b0}};
        end
    endfunction

    wire [PC_WIDTH-1:0] pred_imm0 = btb_imm(ipack_pred_inst0);
    wire [PC_WIDTH-1:0] pred_imm1 = btb_imm({16'b0, ipack_pred_inst1});

    // Branch-taken result (aq_ifu_pred.v:585-591, real formula, BHT now
    // real -- SECTION BHT below supplies `bht_pred_rslt`/`pred_inst0_bjtype`):
    //   pred_inst0_taken = pred_br_vld0 && bht_pred_rslt[1] || pred_jmp_vld0
    //   pred_inst1_taken = !pred_inst0_bjtype && pred_br_vld1
    //                      && bht_pred_rslt[1]
    //                      || !pred_inst0_taken && pred_jmp_vld1
    // `bht_pred_rslt`/`pred_inst0_bjtype` are combinational wires defined in
    // SECTION BHT further down this file -- Verilog wire semantics make the
    // forward reference here harmless (no combinational loop: bht_pred_rslt
    // depends only on the REGISTERED bht_dout_ff/bht_vghr, never on
    // pred_inst0/1_taken or anything downstream of them).
    wire pred_inst0_taken  = pred_br_vld0 && bht_pred_rslt[1] || pred_jmp_vld0;
    wire pred_inst1_taken  = !pred_inst0_bjtype && pred_br_vld1 && bht_pred_rslt[1]
                              || !pred_inst0_taken && pred_jmp_vld1;
    wire pred_inst0_bjtype = pred_br_vld0 || pred_jmp_vld0;   // aq_ifu_pred.v:585

    // a. RAS access signals (aq_ifu_pred.v:610-628). `pred_inst0_taken`
    // (defined just above, SECTION BTB classification) already reduces to
    // `pred_jmp_vld0` with no BHT built yet; `delay_chgflw` (S4.2,
    // BHT-only) is likewise structurally 0 (Task 9).
    // TASK 9 UPDATE: real aq_ifu_pred.v:612 guards slot-1's link/ret validity
    // with `!pred_inst0_taken` (aq_ifu_pred.v:612,618), not `!pred_jmp_vld0`
    // -- through Task 8 these were equivalent (pred_inst0_taken reduced to
    // exactly pred_jmp_vld0 with no BHT built), but now that pred_inst0_taken
    // also ORs in a real taken CONDITIONAL branch (SECTION BTB classification
    // above), a slot-1 call/return must ALSO be suppressed when slot 0 is a
    // taken branch (not just a taken jump) -- the general "slot 0 already
    // redirects, slot 1 is wrong-path" rule the real RTL always applied.
    wire pred_ras_link_vld0 = pred_link_vld0;
    wire pred_ras_link_vld1 = pred_link_vld1 && !pred_inst0_taken;
    wire pred_ras_link_vld  = (pred_ras_link_vld0 || pred_ras_link_vld1)
                             && cp0_ifu_ras_en && !ibuf_ipack_stall;

    // pred_ras_ret_vld0 (aq_ifu_pred.v:617): TASK 9 FIX -- the real formula's
    // `&& !delay_chgflw` term (flagged as a no-op placeholder by Task 7's own
    // comment, since delay_chgflw could not exist before BHT) is now real:
    // a predicted return in slot 0 must be suppressed while a PRIOR cycle's
    // delayed BHT redirect (SECTION BHT below) is still pending replay,
    // exactly as it suppresses an ordinary taken branch/jump redirect.
    wire pred_ras_ret_vld0   = pred_ret_vld0 && !delay_chgflw;
    wire pred_ras_ret_vld1   = pred_ret_vld1 && !pred_inst0_taken;
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
    // pushes the correct return address. The real mux's OTHER term
    // (`pred_delay_br_raw`) is BHT-only and dropped (Task 9); `pred_br_taken1`
    // is real's own OR term for "slot 1 is the taken one" -- TASK 8 FIX
    // (found while wiring BTB, not present when Task 7 wrote this line):
    // Task 7's original condition here was `pred_ras_link_vld1` alone, which
    // was correct for RAS's OWN need (a slot-1 CALL) but silently missed
    // real aq_ifu_pred.v's broader `pred_br_taken1 || pred_ras_link_vld1 ||
    // pred_delay_br_raw` (aq_ifu_pred.v:480) -- harmless through Task 7
    // since nothing else could make slot 1 "taken" yet, but BTB (this task)
    // introduces exactly that case (a plain slot-1 jal/c.j, `pred_jmp_vld1`,
    // with slot 0 not taken) and needs the CORRECT current-PC (slot 1's own
    // address) to compute its target/tag from. `pred_inst1_taken` (SECTION
    // BTB classification above) is the reduced-but-forward-compatible stand-
    // in for `pred_br_taken1` here, same reduction discipline as elsewhere
    // in this file.
    // TASK 9 FIX (found while wiring BHT, not present when Task 7/8 wrote
    // this line): the real mux's first-branch condition is `pred_br_taken0`
    // (aq_ifu_pred.v:475), i.e. "slot 0 is THE ONE redirecting" -- which
    // through Task 8 happened to coincide with `c0_jmp` (jal/c.j) only,
    // since no conditional branch could ever be "taken" without BHT. Now
    // that `pred_br_taken0` also covers a real taken conditional branch
    // (SECTION BTB above) AND is gated by `!delay_chgflw` (SECTION BHT
    // below), `c0_jmp` is no longer an equivalent stand-in -- switched to
    // the real signal. The second condition also gains `pred_delay_br_raw`
    // (aq_ifu_pred.v:480): when slot 0 is a branch predicted NOT-taken and
    // slot 1 is ALSO a branch (the same-row-second-lookup delay case,
    // SECTION BHT below), slot 1's own PC is needed here even though
    // `pred_inst1_taken` is structurally 0 in that exact case (BHT's
    // direction mux is occupied by slot 0 this cycle) -- without this term
    // `pred_br_tar`/BTB's tag-compare would incorrectly key off slot 0's PC
    // for what is actually slot 1's branch.
    // M8 T4b FIX (Class-B, coremark + m1-m5 hang; 2nd round after the
    // interrupt regression caught the first attempt's 1-cycle-late load):
    // restore the donor's OWN PRED-stage PC REGISTER, which Task 1
    // collapsed away. Donor aq_ifu_pred.v:444-453 keeps a REGISTER
    // `pred_idpc` (gated clock, aq_ifu_pred.v:398-408) that stays aligned
    // to the instruction BPU is classifying THIS cycle, and computes ALL
    // branch-base PCs from it (pred_h0_pc :461, pred_cur_pc :475-483,
    // pred_ras_tar :656-657, RAS link :628, BTB tag :804).
    //
    // TIMING CONTRACT (the subtle part, verified against the trace): the
    // donor's load source `pcgen_pred_ifpc` (aq_ifu_pcgen.v:285-293,319)
    // latches the fetch pointer on the GRANT cycle, so it already equals
    // the arriving word's base PC during the ARRIVAL cycle (icache data
    // lands one cycle after its grant, both here and in the donor). The
    // register loads on the ARRIVAL event (`icache_pred_inst_vld`,
    // aq_ifu_icache.v:1339), so it is correct from the bundle's FIRST
    // display cycle at the PRED stage (the IPACK entry is created at the
    // edge ending the arrival cycle, displayed from the next). Loading on
    // the entry-valid (`ipack_pred_inst0_vld`) instead -- or latching the
    // raw pointer on the arrival instead of the grant -- lands the
    // register ONE BUNDLE STALE on every first-display cycle after a
    // redirect: with the 1st-round version, the interrupt test's trap
    // redirect to 0x80000400 left pred_idpc_r at the pre-trap bundle
    // 0x80000444 for the cycle the new bundle's c.j@0x80000400 was
    // predicted (probe: [bpu] pcp=80000444 pbtk=1 vs [ifu] e1=80000400),
    // misdirecting the fetch and cascading into an inst-fetch-fault trap
    // loop. THE FIX (both halves, donor-faithful):
    //   1. IFU.v latches `pcgen_pipe_ifpc` on `icache_pcgen_grant` with
    //      `pcgen_fetch_pc` (aq_ifu_pcgen.v:285-293 verbatim shape; this
    //      SUPERSEDES that file's BUG FIX #4 arrival-latch, which only
    //      masked the race at the source while the BPU still consumed the
    //      raw pointer directly -- the donor never did that), and
    //   2. this register loads on the new `icache_inst_vld` input (the
    //      arrival event, see the port note above), gated by the donor's
    //      `pred_id_stall`.
    //
    // Donor-to-rv906 signal map (each confirmed against the real files):
    //   donor `icache_pred_inst_vld` -> `icache_inst_vld` (new input;
    //     rv906's aq_ifu_ipack.v:253 equivalent -- "fresh word landing in
    //     IPACK this cycle", cancel/mask already excluded, which in the
    //     donor are absorbed by the icache's own abort handling).
    //   donor `pred_id_stall` (aq_ifu_pred.v:741: `pred_ret_stall ||
    //     pred_delay_br || ibuf_pred_stall`) -> `pred_ret_stall ||
    //     pred_delay_br || ibuf_ipack_stall` -- the donor's `ibuf_pred_
    //     stall` IS `ibuf_ipack_stall` (BOTH alias `ibuf_stall`,
    //     aq_ifu_ibuf.v:1340,1345), so no term is missing. Blocking the
    //     load on `pred_ret_stall` matters in rv906 too: during RAS WAIT
    //     `pred_ipack_ret_stall` holds entry1 valid-but-unretired while
    //     `ibuf_ipack_stall` alone can be 0, so the raw fetch pointer
    //     would race ahead the same way.
    //   donor `pred_delay_br` -> `pred_delay_br` (same formula,
    //     aq_ifu_pred.v:548-549); donor `pcgen_pred_ifpc` -> the
    //     `pred_idpc` INPUT (the port keeps its donor-mapped name and its
    //     IFU.v driver, IFU.v's `assign pred_idpc = pcgen_pipe_ifpc` --
    //     untouched).
    // No gated clock cell: the donor's gate (idpc_icg_en,
    // aq_ifu_pred.v:398) only saves power -- its clock-off cycles map to
    // the hold branch below, so the ungated always block replicates the
    // behavior. `pred_ret_stall` (below, RAS section) and `pred_delay_br`
    // (further below, SECTION BHT k) are forward wire references, same
    // convention as this file's `bht_pred_rslt` forward reference.
    //=========================================================================
    reg [PC_WIDTH-1:0] pred_idpc_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pred_idpc_r <= {PC_WIDTH{1'b0}};
        else if (icache_inst_vld && !pred_ret_stall && !pred_delay_br
                 && !ibuf_ipack_stall)
            pred_idpc_r <= pred_idpc;                                  // donor :448-449
        else if (pred_delay_br)
            pred_idpc_r <= {pred_idpc_r[PC_WIDTH-1:2], 2'b10};         // donor :450-451
        else
            pred_idpc_r <= pred_idpc_r;                                // donor :452-453
    end

    reg [PC_WIDTH-1:0] pred_h0_pc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pred_h0_pc <= {PC_WIDTH{1'b0}};
        else if (ipack_pred_h0_create) pred_h0_pc <= {pred_idpc_r[PC_WIDTH-1:2], 2'b10};
    end

    wire [PC_WIDTH-1:0] pred_cur_pc = pred_br_taken0 ? (ipack_pred_h0_vld ? pred_h0_pc : pred_idpc_r)
                                    : (pred_ras_link_vld1 || pred_inst1_taken || pred_delay_br_raw) ? {pred_idpc_r[PC_WIDTH-1:2], 2'b10}
                                    : pred_idpc_r;

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
                                      ? {pred_idpc_r[PC_WIDTH-1:RAS_PC_WIDTH], ras_tar_pc}
                                      : pred_idpc_r;

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
    // SECTION: BTB (plan Task 8.1) -- aq_ifu_btb.v (753 lines) +
    // aq_ifu_btb_entry.v (163 lines) + the BTB-relevant slice of
    // aq_ifu_pred.v (S4.1's "BTB supplies target only" finding, S4.3's
    // chgflw-channel split), all confirmed by reading the real files
    // directly. 16 flop entries, each {valid, tag[15:0], target[15:0]}
    // (aq_ifu_btb_entry.v:55-57,89 -- BTB_ADDR_WIDTH=16 confirmed at both
    // aq_ifu_btb.v:154 and aq_ifu_btb_entry.v:89, matching
    // rvproc_pkg.sv's BTB_TAG_WIDTH/BTB_TARGET_WIDTH=16). This is a real
    // parallel CAM (aq_ifu_btb.v:220-583 instantiates 16 independent
    // `aq_ifu_btb_entry` copies, each with its OWN tag-compare,
    // aq_ifu_btb_entry.v:147-150) -- NOT an indexed SRAM lookup; do not
    // port C910/rv12's set-associative BTB.v shape onto this.
    //
    // COLLAPSED PIPELINE (a deliberate, documented rv906 simplification,
    // not a guess): the real RTL runs this CAM as a 2-STAGE pipeline --
    // the tag compare fires speculatively at PCGEN time against
    // `pcgen_btb_ifpc` (aq_ifu_btb.v:637), is flopped one cycle into
    // `btb_entry_hit_flop`/`btb_tar_vld` (aq_ifu_btb.v:647-732, gated by
    // `ctrl_btb_stall = pred_ctrl_stall || icache_ctrl_stall`,
    // aq_ifu_ctrl.v:101,127), and is validated/written back at ID-stage
    // time in `aq_ifu_pred.v` once the SAME instruction arrives there --
    // relying on a FIXED small number of cycles between PCGEN and ID-stage
    // (set by ICache's own fixed 2-cycle access + the real one-hot IBUF's
    // own tight, fixed-latency timing). rv906's IBUF clone does NOT have
    // that fixed-latency property (IFU.v's own header: "this clone's IPACK
    // entries can stay valid-but-unretired for MANY consecutive cycles"
    // under `--sink-stall`/backpressure) -- transplanting the real 2-stage
    // pipeline as-is would silently misalign the PCGEN-time tag-compare
    // address against whatever ID-stage bundle eventually shows up beside
    // it, for no observable benefit (the M1 oracle only cares about WHICH
    // instructions commit, never WHEN a predictor fires, design doc S4.1).
    // rv906 therefore does the tag compare, hit-target reconstruction,
    // ID-stage validation (`btb_mis_pred`), and the write-back all
    // TOGETHER, same-cycle, keyed off `pred_cur_pc` (the ID-stage bundle's
    // own PC, already correctly time-aligned by Task 7's RAS section) --
    // functionally identical CAM contents/hit-decision/FIFO-replacement
    // rule, one pipeline stage later than real silicon. Consequently:
    // `pcgen_btb_ifpc` (the real RTL's PCGEN-time read address) and
    // `ctrl_btb_stall` (the real RTL's stage-hold signal, which does not
    // even exist as a BPU.v port -- IFU.v's own Task 3 header already
    // noted this) are BOTH genuinely unused by this section; BTB's
    // redirect is NOT a separate early PCGEN priority level in rv906 --
    // it folds into the SAME `pred_pcgen_chgflw_vld/_pc` channel the
    // ID-stage's own "final" redirect already uses (IFU.v's Task 3 header:
    // "BTB's early redirect (Task 8) MUST be folded into BPU.v's chgflw
    // output; it cannot get its own PCGEN priority level without
    // reopening the port freeze"). This still preserves the
    // architecturally meaningful sense of "early": BPU fires this redirect
    // as soon as the branch/jump is DECODED, well before FetchSink's fake
    // BJU/IU ever resolves it -- "early" relative to resolution, just not
    // literally at the earliest possible pipeline stage.
    //
    // a. Tag compare (the CAM itself, aq_ifu_btb_entry.v:147-150 x16).
    //=========================================================================
    reg  [BTB_ENTRIES-1:0]        btb_vld;
    reg  [BTB_TAG_WIDTH-1:0]      btb_tag [0:BTB_ENTRIES-1];
    reg  [BTB_TARGET_WIDTH-1:0]   btb_tgt [0:BTB_ENTRIES-1];

    // Single shared tag-compare address (READ and WRITE use the SAME
    // address here, since the collapsed pipeline does both at once for the
    // SAME instruction -- the real RTL's `btb_rd_acc_tag`/`btb_wr_acc_tag`
    // split, aq_ifu_btb.v:637-638, exists only because ITS read/write are
    // at different pipeline stages).
    wire [BTB_TAG_WIDTH-1:0] btb_acc_tag = pred_cur_pc[BTB_TAG_WIDTH-1:0];

    wire [BTB_ENTRIES-1:0] btb_hit_vec;
    genvar gi;
    generate
        for (gi = 0; gi < BTB_ENTRIES; gi = gi + 1) begin : g_btb_hit
            assign btb_hit_vec[gi] = btb_vld[gi] && (btb_tag[gi] == btb_acc_tag);
        end
    endgenerate
    wire btb_hit_vld = |btb_hit_vec;

    // 16:1 target mux (aq_ifu_btb.v:656-694's case statement, reduced to a
    // priority-OR loop): the hit vector is invariantly one-hot-or-zero
    // under normal operation (any WRITE to a tag that already has a valid
    // matching entry always takes the in-place REPLACE path below, never
    // allocates a second entry for the same tag), so priority order among
    // bits never actually matters -- written as a loop instead of a case
    // statement to stay X-free for deterministic simulation (same
    // precedent as the RAS section's read mux above), unlike the real
    // RTL's `{BTB_ADDR_WIDTH{1'bx}}` default.
    reg [BTB_TARGET_WIDTH-1:0] btb_hit_tgt;
    integer hi;
    always @* begin
        btb_hit_tgt = {BTB_TARGET_WIDTH{1'b0}};
        for (hi = 0; hi < BTB_ENTRIES; hi = hi + 1)
            if (btb_hit_vec[hi]) btb_hit_tgt = btb_tgt[hi];
    end

    // b. Read-side result + ID-stage validation (aq_ifu_pred.v:596-598,
    // 720-726). Target reconstruction reuses the CURRENT bundle's own
    // upper PC bits (aq_ifu_btb.v:740) -- predicted targets are
    // constrained to the same 64KiB-aligned region as the branch's own PC
    // (design doc S2.1/S2.3.3).
    //=========================================================================
    wire [PC_WIDTH-1:0] pred_nxt_offset = (ipack_pred_unalign || pred_delay_br_raw)
                                         ? {{(PC_WIDTH-3){1'b0}}, 3'd2}
                                         : {{(PC_WIDTH-3){1'b0}}, 3'd4};   // aq_ifu_pred.v:487, TASK 9: delay term now real
    wire [PC_WIDTH-1:0] pred_nxt_pc     = pred_cur_pc + pred_nxt_offset;

    // pred_br_taken0 (aq_ifu_pred.v:592): TASK 9 -- the real `&& !delay_chgflw`
    // gate is now real (SECTION BHT below); through Task 8 this was a no-op
    // (delay_chgflw could not exist without BHT).
    wire pred_br_taken0 = pred_inst0_taken && !delay_chgflw;
    wire pred_br_taken1 = pred_inst1_taken;
    wire pred_br_taken  = pred_br_taken0 || pred_br_taken1;
    wire [PC_WIDTH-1:0] pred_br_imm = pred_inst0_taken ? pred_imm0 : pred_imm1;
    wire [PC_WIDTH-1:0] pred_br_tar = pred_cur_pc + pred_br_imm;

    wire                 btb_pred_tar_vld = cp0_ifu_btb_en && btb_hit_vld;
    wire [PC_WIDTH-1:0]  btb_pred_tar_pc  = {pred_cur_pc[PC_WIDTH-1:BTB_TAG_WIDTH], btb_hit_tgt};

    // btb_mis_pred (aq_ifu_pred.v:720-723): fires whenever BTB HAD a valid
    // prediction for this bundle and it disagrees -- either a genuine
    // wrong target, OR the actual outcome isn't even taken (catches BOTH
    // an ordinary stale/wrong entry AND the PC[15:0] aliasing case, design
    // doc S2.3.3 -- see that section for the full finding). Gated by
    // `ipack_pred_inst0_vld` so a cycle with no valid bundle at all can
    // never spuriously fire (matches real exactly; `btb_pred_tar_vld`
    // itself is intentionally NOT gated by inst0_vld, mirroring the real
    // RTL's independent `btb_tar_vld` flop).
    wire btb_mis_pred = (pred_br_tar != btb_pred_tar_pc || !pred_br_taken)
                      && btb_pred_tar_vld && ipack_pred_inst0_vld;

    // pred_chgflw / pred_chgflw_fin (aq_ifu_pred.v:725-733): BTB-first,
    // ID-stage-corrects-on-disagreement (BPU notes S4.1) -- if BTB had a
    // valid prediction and it was right, no redirect (fetch is already
    // following it); if BTB had no entry, ID-stage's own jmp-taken +
    // immediate-computed target drives the redirect directly.
    // `pred_delay_br_raw` (real's 2nd AND term, TASK 9: now real, SECTION
    // BHT below) suppresses this cycle's BTB-mispredict correction while a
    // same-row-second-lookup delay/replay is in flight for slot 1 instead.
    //
    // M8 T4c FIX (Class-B, coremark/m1-m5 hang): the verbatim donor formula
    // above is only correct in the donor's pipeline, where a taken branch
    // with a valid BTB entry is redirected by the BTB's OWN PCGEN-stage
    // channel -- `btb_xx_chgflw_vld`/`btb_pcgen_tar_pc` (aq_ifu_btb.v:71-74,
    // 740-747) consumed at aq_ifu_pcgen.v:279-282, one fetch BEFORE the
    // branch reaches this ID stage. rv906 collapses the 2-stage BTB CAM
    // into this ID stage (SECTION BTB header above) and exposes no
    // PCGEN-stage BTB channel (frozen ports), so in the "BTB hit, target
    // correct, BHT predicts taken" case the donor's ID-stage formula
    // evaluates to 0 AND nothing else redirects: the fetch keeps streaming
    // linearly past the branch while bju_pcgen tracks the resolved (taken)
    // path -- a prediction/fetch desync that corrupts the stream and
    // hangs coremark (repro: memmove's byte loop, bne@0x4012 predicted
    // taken + BTB hit; wrong-path fall-through ret/or/... execute, a
    // later branch mispredicts from the desynced bju_pcgen, redirects to a
    // bogus target, misaligned load traps, hang). Probe-confirmed
    // (dut_dbg [bpu]: btbv=1 btbmp=0 ibst=0 pbtk=1 pbtar=target pcp=br_pc,
    // no pcgen chgflw). The added `pred_br_taken` term fires this same
    // (only) ID-stage redirect channel for exactly the case the donor's
    // PCGEN-stage channel covered: taken + BTB hit. Target is `pred_tar`
    // = `pred_br_tar`, which equals the stored BTB target in the
    // hit-and-correct case by definition of `btb_mis_pred=0` (target
    // compare is part of it); the hit-and-wrong-target case already fired
    // via `btb_mis_pred` and still invalidates the entry (`btb_clr_one`).
    // Deviation from donor: redirect lands one stage later (ID-stage time
    // vs PCGEN-stage time) through the SAME pcgen level-1 slot the
    // donor's own ID-stage corrections use; committed stream is unchanged
    // (the donor's PCGEN redirect prevents exactly the same wrong-path
    // fetches this redirect now cancels via ctrl_if_cancel's existing
    // fan-out + the TASK 4c IPACK drop of the branch's own fall-through).
    wire pred_chgflw     = btb_pred_tar_vld
                         ? ((btb_mis_pred || pred_br_taken) && !pred_delay_br_raw)
                         : pred_br_taken;
    wire [PC_WIDTH-1:0] pred_tar = pred_br_taken ? pred_br_tar : pred_nxt_pc;
    // pred_curflw (RAS's own channel, S4.1 "RAS bypasses BTB entirely"):
    // TASK 9 -- the real `|| delay_chgflw` OR term is now real (SECTION BHT
    // below): a pending delayed BHT redirect is also a same-cycle "current
    // flow" correction, exactly like a RAS return.
    wire pred_curflw     = pred_ras_ret_chgflw || delay_chgflw;
    // LIVELOCK GATE (same class of risk Task 7's header/IFU.v's PCGEN
    // section already analyzed and fixed for `pred_pcgen_curflw_vld`, via
    // the SAME `!ibuf_ipack_stall` term): `ipack_pred_inst0/1_vld` and
    // their contents are FROZEN for as long as IPACK/IBUF hold a bundle
    // stalled-but-unretired (this file's own binary-pointer IBUF clone can
    // do this for MANY consecutive cycles under `--sink-stall`, IFU.v's
    // header), so without this gate `pred_chgflw_fin` would stay asserted
    // at the SAME target every one of those cycles, continuously
    // re-triggering `ctrl_icache_abort` and risking starving
    // `icache_pcgen_grant` from ever firing again -- added defensively
    // here for the identical reason Task 7 added it to curflw, even though
    // no concrete hang was observed; costs nothing (the redirect still
    // fires the one cycle the bundle actually retires) and removes the
    // whole risk class instead of relying on ICache.v's own abort-recovery
    // behavior being forgiving under indefinite re-assertion.
    wire pred_chgflw_fin = pred_chgflw && !pred_curflw && !pred_ras_ret_vld1
                         && !ibuf_ipack_stall;

    // c. Write side: install/replace + FIFO round-robin (aq_ifu_btb.v:
    // 593-623). `pred_btb_upd_vld` (aq_ifu_pred.v:795-796) fires for a
    // taken branch/jump that is NOT currently flagged as a BTB mispredict
    // -- i.e. either a brand-new taken jal/c.j (no entry yet, allocate via
    // FIFO) or an already-correct hit (in-place refresh, a content no-op).
    // A GENUINELY wrong hit (target mismatch or not-taken) takes the
    // OPPOSITE path below (`btb_clr_one`): the entry is INVALIDATED, not
    // immediately overwritten, exactly matching real hardware.
    // `ibuf_pred_hungry` (aq_ifu_ibuf.v:1346, `ibuf_empty ||
    // ibuf_five_avalbe || ibuf_four_avalbe` -- IBUF has room for at least
    // 2 more halfwords) has no rv906 IBUF-occupancy port on BPU.v's frozen
    // list; `!ibuf_ipack_stall` is used as its proxy here (a "don't
    // re-fire the same write every cycle a bundle sits stalled-but-
    // unretired" gate), the SAME proxy Task 7.1 already established for
    // RAS's own push/pop re-fire prevention -- a room-check and a
    // re-fire-prevention check are different concerns, but for a
    // write-enable that is otherwise idempotent (re-writing the SAME
    // tag/target is harmless, see above), the difference is unobservable.
    //=========================================================================
    wire pred_btb_upd_vld  = pred_br_taken && !btb_mis_pred && !ibuf_ipack_stall;
    wire btb_entry_upd_vld = pred_btb_upd_vld && cp0_ifu_btb_en;
    wire btb_entry_replace = btb_entry_upd_vld && btb_hit_vld;

    // btb_clr_one (aq_ifu_btb.v:593): `cp0_ifu_btb_en && btb_wr_hit_vld`
    // are already implied by `btb_mis_pred`'s own `btb_pred_tar_vld`
    // precondition above; kept as a bare alias for direct traceability
    // against the real formula's shape.
    wire btb_clr_one = btb_mis_pred;
    wire [BTB_ENTRIES-1:0] btb_entry_clr = ({BTB_ENTRIES{btb_clr_one}} & btb_hit_vec)
                                         | {BTB_ENTRIES{cp0_ifu_btb_clr}};
    wire [BTB_ENTRIES-1:0] btb_entry_upd = btb_entry_replace
                                         ? btb_hit_vec
                                         : ({BTB_ENTRIES{btb_entry_upd_vld}} & btb_fifo);

    // Round-robin FIFO pointer (aq_ifu_btb.v:604-614): advances (left-
    // rotate) ONLY on an allocate-new-entry update (miss on write); a
    // mispredict-clear instead snaps the pointer BACK to the just-freed
    // entry (aq_ifu_btb.v:610-611) so that slot is the next one reused,
    // rather than continuing to rotate past it.
    reg [BTB_ENTRIES-1:0] btb_fifo;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            btb_fifo <= {{(BTB_ENTRIES-1){1'b0}}, 1'b1};
        else if (btb_entry_upd_vld && !btb_hit_vld)
            btb_fifo <= {btb_fifo[BTB_ENTRIES-2:0], btb_fifo[BTB_ENTRIES-1]};
        else if (btb_clr_one)
            btb_fifo <= btb_hit_vec;
        else
            btb_fifo <= btb_fifo;
    end

    // Per-entry valid/tag/target storage (aq_ifu_btb_entry.v:117-142):
    // CLEAR takes priority over UPDATE for the valid bit (matching the
    // real per-entry module's own if/else-if order exactly, even though
    // Task 8's own write-enable formulas make the two conditions mutually
    // exclusive by construction -- `pred_btb_upd_vld` requires
    // `!btb_mis_pred` while `btb_clr_one` requires `btb_mis_pred`, so this
    // priority is defensive, not load-bearing). Tag/target content is
    // written ONLY on an update, never touched by a plain clear (a cleared
    // entry's stale content is harmless since `btb_vld` gates it).
    integer bi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btb_vld <= {BTB_ENTRIES{1'b0}};
            for (bi = 0; bi < BTB_ENTRIES; bi = bi + 1) begin
                btb_tag[bi] <= {BTB_TAG_WIDTH{1'b0}};
                btb_tgt[bi] <= {BTB_TARGET_WIDTH{1'b0}};
            end
        end
        else begin
            for (bi = 0; bi < BTB_ENTRIES; bi = bi + 1) begin
                if (btb_entry_clr[bi])
                    btb_vld[bi] <= 1'b0;
                else if (btb_entry_upd[bi])
                    btb_vld[bi] <= 1'b1;
                if (btb_entry_upd[bi]) begin
                    btb_tag[bi] <= btb_acc_tag;
                    btb_tgt[bi] <= pred_br_tar[BTB_TARGET_WIDTH-1:0];
                end
            end
        end
    end

    //=========================================================================
    // SECTION: BHT (plan Task 9.1/9.2) -- aq_ifu_bht.v (549 lines) +
    // aq_ifu_bht_array.v (123 lines) + the BHT-relevant slice of
    // aq_ifu_pred.v (S1.4's "pure GHR, no PC" finding, S1.6's same-row
    // second-lookup trick, S4.2's delay/replay mechanism), all confirmed by
    // reading the real files directly. Single 1024x16 SRAM
    // (`aq_spsram_1024x16`, aq_ifu_bht_array.v:102), indexed PURELY by GHR
    // -- `pred_bht_pc`/`iu_ifu_bht_cur_pc` are confirmed dead ports in the
    // real RTL (grepped with zero hits outside the port list, BPU notes
    // S1.4) and are NOT cloned here (this module's frozen ports don't even
    // carry a signal shaped like `pred_bht_pc` for that reason).
    //
    // TWO GHRs, not one, not three (Task 9.2 -- confirmed from
    // aq_ifu_bht.v:66,76 declarations and :196-220 update logic; matches the
    // BPU extraction note's own S1.3 finding, no C910-style spec/arch/
    // checkpoint-FIFO triple): `bht_ghr` (ARCHITECTURAL) shifts in the
    // ACTUAL resolved outcome on every `iu_ifu_br_vld`; `bht_vghr`
    // (SPECULATIVE) normally shifts in the PREDICTED outcome on every
    // `pred_bht_br_vld` (a branch reaching this cycle's ID-stage bundle),
    // but on `iu_ifu_bht_mispred` it instead RELOADS from
    // `{bht_ghr[HIS_WIDTH-2:0], iu_ifu_bht_taken}` -- i.e. resyncs from the
    // architectural register plus the just-resolved outcome, not from
    // vghr's own (wrong) history.
    //=========================================================================
    localparam BHT_HIS_W = BHT_GHR_WIDTH;   // aq_ifu_bht.v:131, HIS_WIDTH = IDX_WIDTH+4 = 14

    reg [BHT_HIS_W-1:0] bht_ghr;    // aq_ifu_bht.v:196-206
    reg [BHT_HIS_W-1:0] bht_vghr;   // aq_ifu_bht.v:208-220

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  bht_ghr <= {BHT_HIS_W{1'b0}};
        else if (cp0_ifu_bht_inv)    bht_ghr <= {BHT_HIS_W{1'b0}};
        else if (iu_ifu_br_vld)      bht_ghr <= {bht_ghr[BHT_HIS_W-2:0], iu_ifu_bht_taken};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                    bht_vghr <= {BHT_HIS_W{1'b0}};
        else if (cp0_ifu_bht_inv)      bht_vghr <= {BHT_HIS_W{1'b0}};
        else if (iu_ifu_bht_mispred)   bht_vghr <= {bht_ghr[BHT_HIS_W-2:0], iu_ifu_bht_taken};
        else if (pred_bht_br_vld)      bht_vghr <= {bht_vghr[BHT_HIS_W-2:0], bht_pred_taken};
    end

    // a. BHT access signal (aq_ifu_pred.v:497-498): fires whenever THIS
    // cycle's ID-stage bundle has a conditional branch in either slot.
    wire pred_bht_br_vld = (pred_br_vld0 || pred_br_vld1) && !ibuf_ipack_stall;

    // b. BHT invalidate sweep FSM (aq_ifu_bht.v:319-382): 3-state
    // IDLE/WRTE/READ, sweeps all BHT_ROWS=1024 rows over 1024 cycles
    // (BHT_INV_CYCLES, rvproc_pkg.sv). No separate ICG clock split here (no
    // clock gating cells in rv906, umbrella spec S6.3) -- one FSM, one
    // clock, unlike the real RTL's bht_clk/bht_inv_clk pair.
    localparam [1:0] BHT_INV_IDLE = 2'b00, BHT_INV_WRTE = 2'b10, BHT_INV_READ = 2'b11;
    reg [1:0] bht_inv_cur_st;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bht_inv_cur_st <= BHT_INV_IDLE;
        else case (bht_inv_cur_st)
            BHT_INV_IDLE: bht_inv_cur_st <= cp0_ifu_bht_inv ? BHT_INV_WRTE : BHT_INV_IDLE;
            BHT_INV_WRTE: bht_inv_cur_st <= bht_inv_done    ? BHT_INV_READ : BHT_INV_WRTE;
            BHT_INV_READ: bht_inv_cur_st <= BHT_INV_IDLE;
            default:      bht_inv_cur_st <= BHT_INV_IDLE;
        endcase
    end
    wire bht_inv_wr   = (bht_inv_cur_st == BHT_INV_WRTE);
    wire bht_inv_rd   = (bht_inv_cur_st == BHT_INV_READ);
    wire bht_inv_req  = bht_inv_wr || bht_inv_rd;

    reg [BHT_IDX_W-1:0] bht_inv_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          bht_inv_cnt <= {BHT_IDX_W{1'b0}};
        else if (bht_inv_wr) bht_inv_cnt <= bht_inv_cnt + 1'b1;
    end
    wire bht_inv_done = (bht_inv_cnt == {BHT_IDX_W{1'b1}});

    // c. Refill/update FSM (aq_ifu_bht.v:389-455): IDLE ->(mispred)-> READ1
    // -> READ2 -> WRTE -> IDLE, or IDLE ->(ordinary resolve, update-enabled)->
    // UPD -> IDLE. READ1/READ2 exist to re-derive the row/lane the NEXT
    // prediction needs (re-priming `bht_dout_ff`, see (e) below) after a
    // mispredict flushes the pipeline; WRTE/UPD both perform the actual
    // counter write via `bht_upd_vld`/`bht_miss_write` below.
    localparam [2:0] BHT_REF_IDLE  = 3'b000, BHT_REF_READ1 = 3'b001,
                     BHT_REF_READ2 = 3'b010, BHT_REF_WRTE  = 3'b110,
                     BHT_REF_UPD   = 3'b111;
    reg [2:0] bht_ref_cur_st;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bht_ref_cur_st <= BHT_REF_IDLE;
        else case (bht_ref_cur_st)
            BHT_REF_IDLE:  bht_ref_cur_st <= iu_ifu_bht_mispred ? BHT_REF_READ1
                                            : (iu_ifu_br_vld && bht_upd_en) ? BHT_REF_UPD
                                            : BHT_REF_IDLE;
            BHT_REF_READ1: bht_ref_cur_st <= BHT_REF_READ2;
            BHT_REF_READ2: bht_ref_cur_st <= BHT_REF_WRTE;
            BHT_REF_WRTE:  bht_ref_cur_st <= pred_bht_br_vld ? BHT_REF_WRTE : BHT_REF_IDLE;
            BHT_REF_UPD:   bht_ref_cur_st <= pred_bht_br_vld ? BHT_REF_UPD  : BHT_REF_IDLE;
            default:       bht_ref_cur_st <= BHT_REF_IDLE;
        endcase
    end
    wire bht_miss_read1 = (bht_ref_cur_st == BHT_REF_READ1);
    wire bht_miss_read2 = (bht_ref_cur_st == BHT_REF_READ2);
    wire bht_miss_write = (bht_ref_cur_st == BHT_REF_WRTE) && !pred_bht_br_vld;
    wire bht_upd_vld    = (bht_ref_cur_st == BHT_REF_UPD)  && !pred_bht_br_vld;

    // d. 2-bit saturating-counter update case table (aq_ifu_bht.v:457-509,
    // re-derived by hand from the case table, BPU notes S1.5): {A,B} planes
    // treated as a 2-bit number, increment on taken / decrement on
    // not-taken / saturate at 0 and 3 (no-op, `bht_upd_en=0`).
    reg       bht_upd_en;
    reg [1:0] bht_upd_val;
    always @* begin
        case ({iu_ifu_bht_pred, iu_ifu_bht_taken})
            3'b000: begin bht_upd_en = 1'b0; bht_upd_val = 2'b00; end
            3'b001: begin bht_upd_en = 1'b1; bht_upd_val = 2'b01; end
            3'b010: begin bht_upd_en = 1'b1; bht_upd_val = 2'b00; end
            3'b011: begin bht_upd_en = 1'b1; bht_upd_val = 2'b10; end
            3'b100: begin bht_upd_en = 1'b1; bht_upd_val = 2'b01; end
            3'b101: begin bht_upd_en = 1'b1; bht_upd_val = 2'b11; end
            3'b110: begin bht_upd_en = 1'b1; bht_upd_val = 2'b10; end
            3'b111: begin bht_upd_en = 1'b0; bht_upd_val = 2'b11; end
            default: begin bht_upd_en = 1'b0; bht_upd_val = 2'b00; end  // X-free deviation, same precedent as RAS/BTB's own default muxes
        endcase
    end

    // e. Reference-GHR capture (aq_ifu_bht.v:511-530) -- TASK 9.1 GHR
    // READ/WRITE-WINDOW FINDING (M1 spec S2.3.2, re-derived from THIS RTL,
    // not assumed to carry over from rv12's C910 resolution): captured on
    // EITHER a mispredict OR an ordinary update-enabled resolve, from the
    // ARCHITECTURAL `bht_ghr` (non-blocking read: this is `bht_ghr`'s value
    // from BEFORE this branch's own outcome shifts in this same edge, i.e.
    // the history this branch was actually predicted with on the happy
    // path, since resolution is in-order and single-issue here -- every
    // branch strictly older than this one has already both PREDICTED and
    // RESOLVED by the time this one resolves, so `bht_ghr` and the
    // `bht_vghr` this branch was predicted with hold the IDENTICAL 14-bit
    // value whenever none of those older branches mispredicted).
    //
    // Given that, the WRITE index (`bht_ref_vghr[13:4]` + lane `[2:0]`,
    // below) and the READ index used to make THIS branch's own prediction
    // (`bht_vghr[11:2]` + lane `[2:0]`, in (f) below) are two DIFFERENT
    // bit-slices of that SAME 14-bit value -- read covers bits [0:11]
    // (ages 0-11, the 12 freshest history bits); write covers bits
    // {0,1,2}u{4..13} (skips age 3 entirely, reaches back to ages 12-13
    // that read never touches). This is NOT explained away by pipeline
    // depth: `aq_ifu_pcgen.v` was read directly for this task (grepped
    // case-insensitively for "bht"/"ghr" -- zero matches anywhere in that
    // file) and contains no BHT-adjacent logic at all, so there is no
    // PCGEN-side mechanism reconciling the two windows the way rv12 found
    // one for C910's analogous BHT question. And the pipeline-depth
    // argument itself doesn't need PCGEN to fail here regardless: the
    // "happy path" value-equality above holds for ANY constant number of
    // pipeline stages between prediction and resolve, so a fixed latency
    // cannot be the source of a systematic 2-bit/skip-bit-3 RE-SLICING of
    // the identical register -- if it were "the same hash viewed at two
    // pipeline stages" (rv12's C910 finding), read and write would slice
    // the SAME bit positions, and they provably do not here.
    //
    // CONCLUSION: this is a REAL, as-shipped indexing discrepancy between
    // the row/lane a branch's prediction reads and the row/lane its own
    // resolve later updates -- re-derived independently from C906's own
    // RTL, the C910 resolution does NOT transfer. It is, however,
    // CORRECTNESS-HARMLESS by the same structural argument as the BTB
    // PC[15:0] aliasing finding (design doc S2.3.3): C906's front end
    // treats every predictor output as provisional, always re-validated
    // before it can affect which instructions commit (design doc S4.1) --
    // this discrepancy can only misdirect a branch's OWN counter update to
    // a different physical row/lane than the one that predicted it,
    // injecting extra destructive aliasing into the predictor's training
    // beyond what pure-GHR indexing (zero PC disambiguation) already
    // accepts by design. It degrades prediction ACCURACY/cycle count only
    // (M8 territory), never the committed instruction stream. rv906 clones
    // the discrepancy exactly as shipped -- read `bht_vghr[11:2]`/lane
    // `bht_vghr[2:0]`, write `bht_ref_vghr[13:4]`/lane `bht_ref_vghr[2:0]`
    // -- rather than "fixing" the two windows to agree.
    wire bht_upd_write = (bht_ref_cur_st == BHT_REF_IDLE) && iu_ifu_br_vld
                       && !iu_ifu_bht_mispred && bht_upd_en;

    reg [BHT_HIS_W-1:0] bht_ref_vghr;
    reg [1:0]           bht_ref_val;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bht_ref_vghr <= {BHT_HIS_W{1'b0}};
            bht_ref_val  <= 2'b0;
        end
        else if (iu_ifu_bht_mispred || bht_upd_write) begin
            bht_ref_vghr <= bht_ghr;
            bht_ref_val  <= bht_upd_val;
        end
    end
    wire [2:0]  bht_upd_idx = bht_ref_vghr[2:0];
    wire [15:0] bht_wr_val  = {{8{bht_ref_val[1]}}, {8{bht_ref_val[0]}}};

    // f. Index/lane derivation (aq_ifu_bht.v:246-316) -- pure GHR, no PC
    // contribution anywhere (Task 9.1, confirmed dead `pred_bht_pc`/
    // `iu_ifu_bht_cur_pc` ports, not cloned onto this module's port list at
    // all). Row index has 5 cases: invalidate sweep, mispred refill read1/
    // read2 (from the ARCHITECTURAL `bht_ghr` directly, one bit lower each,
    // re-priming (e)'s bypass registers -- not the same-branch update path),
    // mispred refill write / ordinary update (from `bht_ref_vghr`, see the
    // finding above), and the default normal-prediction read (from the
    // SPECULATIVE `bht_vghr`).
    wire [BHT_IDX_W-1:0] bht_idx =
          bht_inv_req      ? ({BHT_IDX_W{bht_inv_wr}} & bht_inv_cnt)
        : bht_miss_read1   ? bht_ghr[BHT_HIS_W-1:4]
        : bht_miss_read2   ? bht_ghr[BHT_HIS_W-2:3]
        : (bht_miss_write || bht_upd_vld) ? bht_ref_vghr[BHT_HIS_W-1:4]
        :                    bht_vghr[BHT_HIS_W-3:2];

    // g. SRAM request assembly (aq_ifu_bht.v:231-257) and instance (BHT is
    // the ONLY SRAM-backed predictor structure, BPU notes S0/S5 -- BTB/RAS
    // above are pure flop arrays).
    wire [BHT_LANES-1:0] bht_mis_wen = 8'b1 << bht_upd_idx;
    wire [15:0] bht_wen = bht_inv_wr ? 16'hffff
                        : (bht_miss_write || bht_upd_vld) ? {2{bht_mis_wen}}
                        : 16'b0;
    wire [15:0] bht_din = {16{!bht_inv_req}} & bht_wr_val;
    wire bht_cen = bht_inv_req
                || ((pred_bht_br_vld || bht_miss_read1 || bht_miss_read2
                     || bht_miss_write || bht_upd_vld) && cp0_ifu_bht_en);

    wire [15:0] bht_dout;
    SRAM #(
        .WIDTH (BHT_WIDTH),
        .DEPTH (BHT_ROWS)
    ) u_bht_array (
        .clk    (clk),
        .cen_n  (!bht_cen),
        .gwen_n (!(|bht_wen)),
        .wen_n  (~bht_wen),
        .addr   (bht_idx),
        .d      (bht_din),
        .q      (bht_dout)
    );

    // h. Bypass mux + row-holding register (aq_ifu_bht.v:275-316): the
    // real donor SRAM is registered-read, write-NOT-through (SRAM.v's own
    // documented contract matches exactly) -- `bht_dout_bypass` supplies a
    // just-written row's value for the one cycle a refill-write/update
    // would otherwise see stale (pre-write) `q`; `bht_dout_ff` latches the
    // row a normal prediction (or a mispred read2) actually consumes.
    reg        bht_bypass_sel;
    reg [15:0] bht_dout_bypass;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                              bht_bypass_sel <= 1'b0;
        else if (bht_miss_write || bht_upd_vld)   bht_bypass_sel <= 1'b1;
        else if (pred_bht_br_vld)                 bht_bypass_sel <= 1'b0;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                            bht_dout_bypass <= 16'b0;
        else if (bht_miss_write || bht_upd_vld) bht_dout_bypass <= bht_dout;
    end
    wire [15:0] bht_dout_rslt = bht_bypass_sel ? bht_dout_bypass : bht_dout;

    reg [15:0] bht_dout_ff;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                    bht_dout_ff <= 16'b0;
        else if (pred_bht_br_vld || bht_miss_read2)     bht_dout_ff <= bht_dout_rslt;
    end

    // i. Lane select / direction result (aq_ifu_bht.v:305-316). The A
    // ("taken") plane is bits [15:8], the B ("not-taken") plane is bits
    // [7:0], selected by the SAME one-hot lane (BPU notes S1.5).
    wire [BHT_LANES-1:0] bht_sel_way = 8'b1 << bht_vghr[2:0];
    wire [1:0] bht_sel_result = {|(bht_sel_way & bht_dout_ff[15:8]),
                                 |(bht_sel_way & bht_dout_ff[7:0])};
    wire bht_pred_taken = bht_sel_result[1];

    // j. Same-row second-lookup trick (Task 9.1, BPU notes S1.6): the array
    // is single-ported but a 2-wide fetch bundle can hold TWO conditional
    // branches; this reuses the row `pred_bht_br_vld` already fetched for
    // slot 0, picking a DIFFERENT one of the 8 lanes by substituting slot
    // 0's own (not-yet-resolved) predicted outcome for the lane's low bit
    // -- answering "what would the BHT have said for slot 1, assuming slot
    // 0's prediction becomes part of history" from data already sitting in
    // the just-read row, no second SRAM access needed. Consumed by the
    // delay/replay mechanism (k) below, exactly as real aq_ifu_pred.v does
    // (S4.2) -- still meaningful for rv906 even though only one
    // instruction/cycle reaches IDU downstream, since the ICache itself
    // fetches up to 2 halfwords/cycle into IPACK/IBUF (design doc S2.1) and
    // this is where those two halfwords' branch predictions are resolved.
    wire [2:0]            bht_mem_idx = {bht_vghr[1:0], bht_pred_taken};
    wire [BHT_LANES-1:0]  bht_mem_way = 8'b1 << bht_mem_idx;
    wire bht_pred_mem_taken = |(bht_mem_way & bht_dout_rslt[15:8]);

    wire [1:0] bht_pred_rslt = bht_sel_result;

    // k. Delay/replay for a not-taken-then-taken pair in one bundle
    // (aq_ifu_pred.v:546-581, S4.2): when slot 0 is a branch predicted
    // NOT-taken and slot 1 is ALSO a branch, the single BHT direction-mux
    // path is occupied by slot 0 this cycle, so slot 1's own redirect (if
    // its same-row lookup (j) says taken) is spliced in as a
    // same-cycle-deferred replay the FOLLOWING cycle instead of waiting a
    // full extra fetch round-trip.
    wire pred_delay_br_raw    = pred_br_vld0 && !bht_pred_rslt[1] && pred_br_vld1;
    wire pred_delay_br        = pred_delay_br_raw && !ibuf_ipack_stall;
    wire pred_delay_br1_taken = pred_br_vld0 && !bht_pred_rslt[1] && pred_br_vld1
                              && bht_pred_mem_taken && !ibuf_ipack_stall;
    wire pred_delay_reissue   = pred_br_vld0 && !bht_pred_rslt[1] && pred_br_vld1
                              && !ibuf_ipack_stall;
    wire pred_delay_taken     = pred_delay_br1_taken || pred_delay_reissue;
    wire [PC_WIDTH-1:0] delay_tar = pred_delay_br1_taken ? pred_br_tar : pred_nxt_pc;

    reg delay_chgflw;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                             delay_chgflw <= 1'b0;
        else if (pred_delay_taken)               delay_chgflw <= 1'b1;
        else if (delay_chgflw && !ibuf_ipack_stall) delay_chgflw <= 1'b0;
    end
    reg [PC_WIDTH-1:0] chgflw_pc_ff;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)               chgflw_pc_ff <= {PC_WIDTH{1'b0}};
        else if (pred_delay_br)   chgflw_pc_ff <= delay_tar;
    end

    // l. pred_ibuf_br_taken0/1 (aq_ifu_pred.v:599-600,787-788): the
    // captured 2-bit counter state riding alongside each halfword pushed
    // into IBUF (IFU.v's SECTION IBUF propagates this to
    // `ifu_idu_id_bht_pred`, Task 9's own IFU.v amendment) -- slot 1's is
    // gated by `!pred_inst0_bjtype` (real aq_ifu_pred.v:600): if slot 0 is
    // itself a branch/jump, this cycle's single BHT read result belongs to
    // slot 0, not slot 1.
    wire [1:0] pred_br_rslt0 = {2{pred_br_vld0}} & bht_pred_rslt;
    wire [1:0] pred_br_rslt1 = {2{pred_br_vld1 && !pred_inst0_bjtype}} & bht_pred_rslt;

    //=========================================================================
    // Output to PCGEN (BPU notes S4.3's "curflw" channel -- RAS bypasses
    // BTB/chgflw entirely). `pred_curflw` (SECTION BTB above) is
    // `pred_ras_ret_chgflw || delay_chgflw` (real aq_ifu_pred.v:738-739).
    // Both OR terms get the SAME rv906 livelock gate (`!ibuf_ipack_stall`,
    // Task 7's RAS-section rationale, extended here to the delay/replay
    // term for the identical reason: `delay_chgflw` is a level that stays
    // asserted across a multi-cycle `--sink-stall` stall exactly like a
    // held RAS-return curflw would, so it needs the same "only pulse the
    // one cycle IPACK's bundle actually retires" treatment) rather than the
    // real RTL's un-gated-by-stall `pred_ras_ret_chgflw`/`delay_chgflw`.
    // `pred_ras_ret_vld` already bakes this gate in (Task 7); applied here
    // to `delay_chgflw` directly since it has no equivalent pre-gated wire.
    //=========================================================================
    assign pred_pcgen_curflw_vld = pred_ras_ret_vld || (delay_chgflw && !ibuf_ipack_stall);
    assign pred_pcgen_curflw_pc  = pred_ras_ret_chgflw ? pred_ras_tar : chgflw_pc_ff;
    assign pred_ctrl_stall       = pred_ret_stall || pred_delay_br;   // aq_ifu_pred.v:741 (real also ORs ibuf_pred_stall, no rv906 equivalent exists)
    assign pred_ipack_ret_stall  = pred_ret_stall;
    assign pred_ipack_delay_stall = pred_delay_br_raw;                // aq_ifu_pred.v:782
    assign pred_ipack_mask        = pred_delay_br_raw;                // aq_ifu_pred.v:783
    assign pred_ibuf_br_taken0    = pred_br_rslt0;                    // aq_ifu_pred.v:787
    assign pred_ibuf_br_taken1    = pred_br_rslt1;                    // aq_ifu_pred.v:788
    assign bht_cp0_inv_done       = bht_inv_done;                     // real: ifu_cp0_bht_inv_done, aq_ifu_bht.v:543

    //=========================================================================
    // Output to PCGEN (BPU notes S4.3's "chgflw" channel -- BHT/BTB
    // ordinary branch/jump redirects, landed this task). Folded into the
    // SAME channel BPU.v has exposed since Task 1 (see SECTION BTB's
    // header note on why this task does not add a separate early-redirect
    // port). `pred_chgflw_fin`/`pred_tar` are defined in SECTION BTB above.
    //
    // pred_ibuf_chgflw_vld0 (aq_ifu_pred.v:713,786: `pred_chgflw_vld0 =
    // pred_br_taken0 || pred_ras_ret_vld0`) -- TASK 8 FIX (found while
    // wiring BTB, not present when Task 7 wrote this port): Task 7 tied
    // this output to constant 0 with the justification "provably always 0
    // while BPU.v ties both outputs inactive" -- true THEN (pred_br_taken0
    // did not exist yet), but `pred_ras_ret_vld0` was ALREADY real since
    // Task 7 and this port was never updated to reflect it, a latent gap
    // in Task 7's own reduction (IFU.v's IBUF section already has a real,
    // frozen consumer for this port -- it gates the 3rd/newest halfword's
    // push-count, ibuf.v:966-968). Wired to the real formula now that both
    // OR terms exist.
    //=========================================================================
    assign pred_pcgen_chgflw_vld = pred_chgflw_fin;
    assign pred_pcgen_chgflw_pc  = pred_tar;
    assign pred_ibuf_chgflw_vld0 = pred_br_taken0 || pred_ras_ret_vld0;

endmodule
