//=============================================================================
// BPU.v - branch prediction unit: BHT + BTB + RAS + arbitration
//                                (M1: RAS+BTB real, BHT skeleton; ports frozen)
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
// RAS landed in plan Task 7, BTB in Task 8 (both real below); BHT's body
// (Task 9) is still the frozen Task-1 skeleton -- only its chicken
// bits/ports are tied inactive, nothing else in this file depends on it
// existing yet.
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
    // TASK 9 STILL SKELETON: the BHT direction table itself does not exist
    // yet -- every CONDITIONAL-branch redirect still comes from FetchSink's
    // fake BJU, exactly as at rungs 1-2 (BTB, landed this task -- SECTION
    // BTB below -- only ever gets populated/validated by unconditional
    // jal/c.j at this rung, since nothing here can compute a conditional
    // branch's "taken" direction without BHT; see that section's
    // classification comments). Also unimplemented: the BHT-side ID-stage
    // delay/replay mechanism (`delay_chgflw`, BPU notes S4.2) and IPACK's
    // `pred_ipack_delay_stall`/`pred_ipack_mask` gates it drives -- both
    // read as constant-false terms below, matching their real-RTL role
    // exactly when no BHT exists (a reduced form of aq_ifu_pred.v's
    // formulas, not a guess -- Task 9 is where each dropped term gets
    // reinstated). `pred_ibuf_br_taken0/1` likewise stay 0: their real
    // formula ANDs a con_br-valid gate with `bht_pred_rslt[1:0]` (the
    // captured 2-bit counter state), which does not exist until Task 9.
    //=========================================================================
    assign pred_ipack_delay_stall = 1'b0;
    assign pred_ipack_mask        = 1'b0;
    assign pred_ibuf_br_taken0    = 2'd0;
    assign pred_ibuf_br_taken1    = 2'd0;

    // Ports with no consumer yet (the BHT chicken bits/invalidate, IU's
    // confirmed-branch/BHT-update bus, BPU notes S1.6/S4.4) -- genuinely
    // unused until Task 9, not an oversight. `pcgen_btb_ifpc` (BTB's real
    // PCGEN-stage read address) is ALSO deliberately left unused here even
    // though BTB is now real (SECTION BTB below) -- see that section's own
    // header note for why rv906 collapses the real RTL's 2-stage
    // (PCGEN-time speculative read, ID-time validate/write) CAM pipeline
    // into a single ID-stage-time lookup keyed off `pred_cur_pc` instead,
    // never consuming this port.
    wire _t9_unused_ok = &{1'b0, pcgen_btb_ifpc, cp0_ifu_bht_en,
                            cp0_ifu_bht_inv, iu_ifu_br_vld,
                            iu_ifu_bht_taken, iu_ifu_bht_pred};

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

    // Branch-taken result (aq_ifu_pred.v:585-598) -- BHT-DEPENDENT TERMS
    // DROPPED (Task 9 territory, no BHT structure exists yet): real
    // `pred_inst0_taken = pred_br_vld0 && bht_pred_rslt[1] || pred_jmp_vld0`
    // reduces to `pred_jmp_vld0` alone (the SAME reduction Task 7's RAS
    // section already used for its own need of this quantity, reproduced
    // here for BTB's independent use of it) -- meaning a REAL conditional
    // branch can never register as "taken" by BPU's own decode until
    // Task 9 lands BHT; only unconditional jal/c.j can. This is a real,
    // well-defined rung-3 behavior, not a guess: every conditional-branch
    // redirect at this rung continues to come from FetchSink's fake BJU
    // exactly as it did at rungs 1-2, and BTB itself is only ever
    // populated/validated by jal/c.j at this rung (SECTION BTB below) --
    // Task 9 ORs `bht_pred_rslt[1] && pred_br_vld{0,1}` back into both
    // lines below and nothing else in this file needs to change.
    // (real's `pred_inst0_bjtype = pred_br_vld0 || pred_jmp_vld0` gates the
    // dropped BHT-only term below and nothing else at this rung, so it is
    // genuinely not instantiated here -- Task 9 will need it again.)
    wire pred_inst0_taken  = /* pred_br_vld0 && bht_pred_rslt[1] || */ pred_jmp_vld0;
    wire pred_inst1_taken  = /* !pred_inst0_bjtype && pred_br_vld1 && bht_pred_rslt[1] || */
                              !pred_inst0_taken && pred_jmp_vld1;

    // pred_br_vld0/1 (con_br classification, above) are genuinely unused
    // until Task 9 -- built now so Task 9 only has to OR `bht_pred_rslt[1]`
    // into `pred_inst0/1_taken` above, nothing else here changes. Kept
    // alive via this bucket rather than deleted, matching this file's own
    // "genuinely unused, not an oversight" convention used elsewhere.
    wire _t9_con_br_unused_ok = &{1'b0, pred_br_vld0, pred_br_vld1};

    // a. RAS access signals (aq_ifu_pred.v:610-628). `pred_inst0_taken`
    // (defined just above, SECTION BTB classification) already reduces to
    // `pred_jmp_vld0` with no BHT built yet; `delay_chgflw` (S4.2,
    // BHT-only) is likewise structurally 0 (Task 9).
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
    reg [PC_WIDTH-1:0] pred_h0_pc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pred_h0_pc <= {PC_WIDTH{1'b0}};
        else if (ipack_pred_h0_create) pred_h0_pc <= {pred_idpc[PC_WIDTH-1:2], 2'b10};
    end

    wire [PC_WIDTH-1:0] pred_cur_pc = c0_jmp ? (ipack_pred_h0_vld ? pred_h0_pc : pred_idpc)
                                    : (pred_ras_link_vld1 || pred_inst1_taken) ? {pred_idpc[PC_WIDTH-1:2], 2'b10}
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
    wire [PC_WIDTH-1:0] pred_nxt_offset = ipack_pred_unalign
                                         ? {{(PC_WIDTH-3){1'b0}}, 3'd2}
                                         : {{(PC_WIDTH-3){1'b0}}, 3'd4};   // real also ORs pred_delay_br_raw (Task 9, structurally 0)
    wire [PC_WIDTH-1:0] pred_nxt_pc     = pred_cur_pc + pred_nxt_offset;

    wire pred_br_taken0 = pred_inst0_taken;    // real also ANDs !delay_chgflw (Task 9, structurally 0)
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
    // `pred_delay_br_raw` (real's 2nd AND term) is Task-9/BHT-only,
    // structurally 0 here.
    wire pred_chgflw     = btb_pred_tar_vld ? btb_mis_pred : pred_br_taken;
    wire [PC_WIDTH-1:0] pred_tar = pred_br_taken ? pred_br_tar : pred_nxt_pc;
    // pred_curflw (RAS's own channel, S4.1 "RAS bypasses BTB entirely"):
    // delay_chgflw OR term is Task-9/BHT-only, structurally 0 here.
    wire pred_curflw     = pred_ras_ret_chgflw;
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
