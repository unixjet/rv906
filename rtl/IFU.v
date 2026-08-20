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
//
// TASK 3 BODY (plan Task 3.1/3.2): real PCGEN/CTRL/IPACK/IBUF/boot logic,
// predictors absent (BPU.v's Task-1 skeleton ties every predictor output
// inactive; RAS/BTB/BHT arrive in Tasks 7-9). Section map mirrors
// aq_ifu_top.v's own flat instantiation order (BOOT/PCGEN/CTRL/IPACK/IBUF).
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
//  * CONFIRMED (Task 3, by reading aq_ifu_pcgen.v's port list directly):
//    `iu_ifu_tar_pc[63:0]` and `rtu_ifu_chgflw_pc[39:0]` widths match the
//    real RTL exactly (pcgen.v ports `input [63:0] iu_ifu_tar_pc` / `input
//    [39:0] rtu_ifu_chgflw_pc`) -- the Task 1 placeholder is resolved, no
//    width mismatch found.
//  * REDIRECT-PRIORITY SEAM (Task 3, confirmed from aq_ifu_pcgen.v:237-253
//    read directly): the real RTL has a dedicated PCGEN priority level for
//    the L0 BTB's own early redirect (`btb_xx_chgflw_vld`/`btb_pcgen_tar_pc`,
//    priority level 5, distinct from `aq_ifu_pred.v`'s level-2 "chgflw"
//    channel). rv906's Task 1 freeze does NOT expose a separate BTB-facing
//    pcgen port pair on IFU.v -- BPU.v's header states its `chgflw` output
//    is unified ("BHT/BTB"). This means BTB's early redirect (Task 8) MUST
//    be folded into BPU.v's `pred_pcgen_chgflw_vld/_pc` output; it cannot
//    get its own PCGEN priority level without reopening the port freeze.
//    Task 3's PCGEN therefore implements 5 levels (boot / delayed-chgflw /
//    same-cycle-curflw / ipack-reissue / sequential-increment), one fewer
//    than the real RTL's 6 non-hold levels -- documented at the PCGEN
//    section below and flagged in the Task 3 completion report.
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
    // TASK 7.1 PORT-FREEZE AMENDMENT (see BPU.v's header note at its own
    // matching input): real C906 fans aq_ifu_ibuf.v's own room-check output
    // into aq_ifu_pred.v too (not just aq_ifu_ipack.v) so RAS push/pop
    // doesn't re-fire every cycle a bundle sits stalled-but-unretired in
    // IPACK. Same net this file already computes as `ibuf_ipack_stall`
    // below (SECTION IBUF) -- just also exposed as a port now.
    output wire                     ibuf_ipack_stall,

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
    // SECTION: BOOT  (aq_ifu_vec.v reduced to RESET->RUN + reset-vector
    // pcload, plan Task 3.1). The real 4-state RESET/WARM_UP/HALT/IDLE FSM
    // (vec.v:157-160) exists only to (a) wait for a CP0 cache-invalidate-done
    // handshake before leaving RESET, (b) run a warm-up counter, then
    // (c) pulse per-unit "warm_up" signals and optionally halt for DTU. None
    // of (a)/(b)/(c)'s consumers exist on IFU.v's frozen port list in M1 (no
    // CP0/IDU/IU/RTU/DTU): there is no `cp0_ifu_rst_inv_done` input to wait
    // on, and the only real-RTL output IFU.v's ports still need is the one
    // that loads the reset vector into PCGEN (`vec_pcgen_rst_vld`,
    // vec.v:260). This reduces to a single one-shot pulse the cycle
    // immediately after `rst_n` deasserts -- the same simplification rv12
    // made for C910's equivalent boot module (design doc S2.1).
    //=========================================================================
    reg boot_rst_vld;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) boot_rst_vld <= 1'b1;
        else        boot_rst_vld <= 1'b0;
    end

    //=========================================================================
    // SECTION: PCGEN  (aq_ifu_pcgen.v -- next-PC arbiter / redirect-priority
    // mux, pcgen.v:237-253). With predictors absent this task, this is the
    // redirect engine for every taken direct branch not yet resolved by
    // IU/RTU (FetchSink's fake BJU/RTU in M1). Priority order, highest first
    // (real RTL numbering kept in comments so a diff against pcgen.v stays
    // legible; level 5 -- BTB's own early redirect -- does not exist as a
    // separate rv906 port, see header note above, so it is skipped):
    //   1. rtu_ifu_chgflw_vld || iu_ifu_tar_pc_vld || pred_pcgen_chgflw_vld
    //      ("delayed change-flow": RTU > IU/BJU > BPU's final redirect;
    //      RTU wins ties, pcgen.v:208-209)
    //   2. pred_pcgen_curflw_vld && !pcgen_buf_chgflw && !icache_pcgen_grant
    //      ("same-cycle correction": RAS-return / delay-replay -- LIVE as of
    //      Task 7: BPU.v's real RAS drives this whenever it predicts a
    //      return, gated by cp0_ifu_ras_en so it stays silent at rung 1,
    //      see BPU.v's RAS section)
    //   3. ipack_pcgen_reissue && icache_pcgen_inst_vld
    //      (IBUF-stall-triggered same-PC refetch, SECTION IPACK)
    //   6. icache_pcgen_grant                        (sequential +4 advance;
    //      folds `boot_rst_vld` in for free on the reset cycle, see BUG FIX
    //      #2 below -- this level is NOT preceded by its own boot term)
    //   7. else hold
    //
    // `boot_rst_vld` is NOT a distinct level in the SEQUENTIAL `pcgen_ifpc`
    // update below (see BUG FIX #2) -- only in the COMBINATIONAL
    // `pcgen_fetch_pc` mux that levels 3/6 both read from (BUG FIX #1).
    //
    // LIVELOCK CHECK (plan Task 3.2, explicit per-level audit): every level
    // above 6 is gated by an actual `_vld` wire sourced from FetchSink or
    // BPU.v, never by an unconditional/default-active internal signal.
    // Through Task 6, BPU.v's skeleton tied `pred_pcgen_chgflw_vld` and
    // `pred_pcgen_curflw_vld` to constant 0 while every predictor was
    // disabled, so levels 2's BPU term and level 3 were structurally
    // incapable of firing -- confirmed by reading BPU.v's body, not assumed.
    // TASK 7 UPDATE: `pred_pcgen_curflw_vld` is now real (BPU.v's RAS), so
    // this level DOES fire whenever a return is predicted at rung >= 2.
    // Re-audited for livelock at that point: `pcgen_chgflw_cur`'s own
    // `!pcgen_buf_chgflw` term only blocks a re-trigger for the one cycle
    // right after a LEVEL-1 (delayed chgflw) event, not after a level-2
    // (curflw) one -- so a curflw pulse held high for many consecutive
    // cycles (the risk Task 7 specifically analyzed: this file's own
    // binary-pointer IBUF, unlike the real one-hot design, can hold IPACK's
    // entries valid-but-unretired for a long `--sink-stall` backpressure
    // run, during which the SAME classified bundle would otherwise be
    // re-presented every cycle) could pin `pcgen_ifpc`/`pcgen_icache_va` at
    // the same curflw target forever. BPU.v's RAS section closes this by
    // gating its `pred_pcgen_curflw_vld` output with `!ibuf_ipack_stall`
    // (a new Task 7.1 port, see BPU.v's header) in addition to
    // `cp0_ifu_ras_en` -- the redirect only ever pulses for the single
    // cycle IPACK's bundle is actually live/retiring, so this level still
    // falls through cleanly whenever RTU/IU/BPU are silent OR stalled.
    // TASK 8 UPDATE: level 1's own BPU term, `pred_pcgen_chgflw_vld`, is now
    // also real (BPU.v's BTB, gated at rung >= 3 by `cp0_ifu_btb_en`).
    // Re-audited the same way: BPU.v's chgflw section applies the identical
    // `!ibuf_ipack_stall` gate to its own output (see BPU.v's own comment
    // at that exact line) for the identical reason -- `ipack_pred_inst0/1`
    // are frozen for the same multi-cycle-stall duration level 2's audit
    // already covers, so without the gate level 1 would face the same
    // continuously-re-triggered-abort risk. No new livelock surface here.
    //=========================================================================
    reg  [63:0]         pcgen_ifpc;          // pcgen.v:100; low 40b architectural, hi 24b sign-ext
    reg  [PC_WIDTH-1:0] pcgen_pipe_ifpc;     // pcgen.v:101; 1-cycle-delayed copy for BPU's ID-stage view
    reg                 pcgen_buf_chgflw;    // pcgen.v:99; 1-cycle latch after a delayed redirect

    // Real RTL's `pcgen_br_chgflw_vld` (pcgen.v:208-209) exists only to mask
    // `pcgen_br_chgflw_pc` into `pcgen_delay_chgflw_pc`'s OR-of-masked-terms
    // formula (pcgen.v:218-219); the if-else form below is a direct
    // simplification of that same OR-of-masks (both are only ever consumed
    // when `pcgen_delay_chgflw_vld` is 1, at which point exactly one of
    // rtu/pcgen_br is the true source) -- there is no separate wire for it
    // here since nothing else in this file reads it.
    wire [PC_WIDTH-1:0] pcgen_br_chgflw_pc = iu_ifu_tar_pc_vld ? iu_ifu_tar_pc[PC_WIDTH-1:0]
                                                                : pred_pcgen_chgflw_pc; // pcgen.v:210-211

    wire pcgen_delay_chgflw_vld = rtu_ifu_chgflw_vld || iu_ifu_tar_pc_vld
                               || pred_pcgen_chgflw_vld;                             // pcgen.v:212-214
    wire [PC_WIDTH-1:0] pcgen_delay_chgflw_pc = rtu_ifu_chgflw_vld ? rtu_ifu_chgflw_pc
                                                                    : pcgen_br_chgflw_pc; // pcgen.v:218-219

    wire pcgen_chgflw_cur = pred_pcgen_curflw_vld && !pcgen_buf_chgflw;              // pcgen.v:280

    // fwd decl: driven in SECTION IPACK below (mirrors ICache.v's own
    // "Verilog elaboration order doesn't care" convention for cross-section
    // combinational wires).
    wire ipack_pcgen_reissue;

    // No BTB-direct mux term (pcgen_chgflw_btb): see header note, this level
    // does not exist on rv906's frozen ports for M1.
    //
    // BUG FIX (Task 6 bring-up, found via the very first fetch of every M1
    // test): `boot_rst_vld` is a ONE-SHOT pulse that is HIGH during the same
    // cycle `ctrl_icache_req_vld` already reads 1 (IBUF is trivially "not
    // stalled" the instant reset deasserts, SECTION CTRL/IBUF -- nothing
    // gates fetch-enable on boot state, unlike the real vec.v FSM this file's
    // header documents collapsing away). Before this fix, `pcgen_fetch_pc`
    // fell through to the OLD (not-yet-loaded) `pcgen_ifpc` register
    // (reset value 0) on that exact cycle, since the reset-vector load
    // (`pcgen_ifpc <= cp0_xx_mrvbr`, the sequential block below) only lands
    // on THIS SAME clock edge -- one register stage later than
    // `pcgen_icache_va`/ICache's grant needs it. The result: ICache's very
    // first request captured `icache_rd_addr = 0`, not the reset vector, and
    // fetched/committed garbage from address 0 as if it were instruction 0 of
    // every test (root-caused with IFU.v/ICache.v $display tracing on
    // iss_selftest.S: `icache_rd_addr=0` at the very first icache_rd_cen,
    // `icache_ipack_inst=0` delivered and committed at PC 0x80000000 instead
    // of the real first opcode). Folding `boot_rst_vld` in as pcgen_fetch_pc's
    // OWN highest-priority term (mirroring how `pcgen_chgflw_cur` already
    // overrides same-cycle, pcgen.v-style) makes the combinational fetch
    // address agree with the sequential update on the very cycle both fire,
    // closing the race with no change to the boot FSM's own one-shot shape.
    wire [63:0] pcgen_fetch_pc = boot_rst_vld
                               ? {{(64-PC_WIDTH){1'b0}}, cp0_xx_mrvbr}
                               : pcgen_chgflw_cur
                               ? {{(64-PC_WIDTH){pred_pcgen_curflw_pc[PC_WIDTH-1]}}, pred_pcgen_curflw_pc}
                               : pcgen_ifpc;                                         // pcgen.v:281-283 (btb term dropped)
    wire [63:0] pcgen_ifpc_inc = {pcgen_fetch_pc[63:2], 2'b00} + 64'h4;              // pcgen.v:277

    // BUG FIX #2 (Task 6 bring-up, found running iss_selftest.S at rung 1
    // through the full RTL: entry #2's committed opcode at pc=0x80000004
    // came back as the SAME 32-bit word already delivered for pc=0x80000000,
    // i.e. the SECOND useful fetch silently re-read the FIRST one instead of
    // advancing). ROOT CAUSE: an earlier revision of this fix added its own
    // `else if (boot_rst_vld) pcgen_ifpc <= mrvbr;` branch HERE, in the
    // SEQUENTIAL update, ranked above `icache_pcgen_grant` (level 6). But
    // `icache_pcgen_grant` legitimately fires on the SAME cycle `boot_rst_vld`
    // is high (rv906's boot model has no fetch-enable gating during reset the
    // way the real vec.v FSM's `vec_ctrl_reset_mask` provides -- see the note
    // above), so that extra branch was clobbering a real, same-cycle grant's
    // advance: instead of latching `pcgen_ifpc_inc` (= mrvbr+4, correctly
    // computed from `pcgen_fetch_pc`, which the FIX ABOVE already threads
    // through `boot_rst_vld`), it re-latched mrvbr itself -- so `pcgen_ifpc`
    // never left the reset vector, and the SECOND ICache request re-issued
    // address 0 (now a cache HIT after the first refill), redelivering the
    // first word forever. THE FIX: do not special-case `boot_rst_vld` in this
    // sequential block at all. `pcgen_ifpc_inc` is already derived from
    // `pcgen_fetch_pc` (fixed above), so the pre-existing, unmodified
    // `icache_pcgen_grant` branch (level 6, below) already produces the
    // correct mrvbr+4 result on cycle 0 once that combinational fix is in
    // place -- no separate sequential priority level is needed or correct.
    // (Compare `pcgen_pipe_ifpc`'s own block just below, which DOES keep an
    // unconditional `boot_rst_vld` term: that register is BPU's ID-stage PC
    // view only, never re-consulted for fetch addressing, and its shape is a
    // faithful clone of the real RTL's own `vec_pcgen_rst_vld` handling of
    // `pcgen_pipe_ifpc` -- pcgen.v:285-293 has the identical asymmetry. The
    // real RTL gets away with an unconditional term in `pcgen_ifpc` too
    // (pcgen.v:239-240) only because `vec_ctrl_reset_mask` there guarantees
    // `icache_pcgen_grant` can never be 1 on the same cycle `vec_pcgen_rst_vld`
    // fires -- a guarantee rv906's collapsed boot FSM does not provide, so the
    // real RTL's shape does not transplant safely into this register.)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pcgen_ifpc <= 64'd0;
        end
        else if (iu_ifu_tar_pc_vld && !rtu_ifu_chgflw_vld) begin                     // level 2, IU/BJU wins:
            pcgen_ifpc <= iu_ifu_tar_pc;                                            // pass the full 64b target through unchanged (pcgen.v:210,261-262)
        end
        else if (pcgen_delay_chgflw_vld) begin                                      // level 2, RTU or BPU wins:
            pcgen_ifpc <= {{(64-PC_WIDTH){pcgen_delay_chgflw_pc[PC_WIDTH-1]}}, pcgen_delay_chgflw_pc}; // sign-extend the 40b target
        end
        else if (pcgen_chgflw_cur && !icache_pcgen_grant) begin                     // level 3 (dead in M1, see livelock check)
            pcgen_ifpc <= pcgen_fetch_pc;
        end
        else if (ipack_pcgen_reissue && icache_pcgen_inst_vld) begin                // level 4
            pcgen_ifpc <= {{(64-PC_WIDTH){icache_pcgen_addr[PC_WIDTH-1]}}, icache_pcgen_addr};
        end
        else if (icache_pcgen_grant) begin                                          // level 6
            pcgen_ifpc <= pcgen_ifpc_inc;
        end
        else begin                                                                  // level 7: hold
            pcgen_ifpc <= pcgen_ifpc;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pcgen_buf_chgflw <= 1'b0;
        else if (pcgen_delay_chgflw_vld) pcgen_buf_chgflw <= 1'b1;
        else if (pcgen_buf_chgflw) pcgen_buf_chgflw <= 1'b0;
    end                                                                             // pcgen.v:221-229

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pcgen_pipe_ifpc <= {PC_WIDTH{1'b0}};
        else if (boot_rst_vld) pcgen_pipe_ifpc <= cp0_xx_mrvbr;
        else if (icache_pcgen_grant) pcgen_pipe_ifpc <= pcgen_fetch_pc[PC_WIDTH-1:0];
        else pcgen_pipe_ifpc <= pcgen_pipe_ifpc;
    end                                                                             // pcgen.v:285-293

    // ---- Rename for Output (pcgen.v "Rename for Output") ------------------
    wire pcgen_ctrl_chgflw_vld = pcgen_delay_chgflw_vld || pred_pcgen_curflw_vld
                              || pcgen_buf_chgflw;                                   // pcgen.v:300-301
    wire pcgen_ibuf_chgflw_vld = rtu_ifu_chgflw_vld || iu_ifu_tar_pc_vld;            // pcgen.v:304-305, direct to IBUF (bypasses ctrl hub)

    assign pcgen_icache_chgflw_vld = pcgen_delay_chgflw_vld || pred_pcgen_curflw_vld
                                   || pcgen_buf_chgflw;                              // pcgen.v:308-309 (btb term dropped)
    assign pcgen_icache_va      = pcgen_fetch_pc;                                    // pcgen.v:310
    assign pcgen_icache_seq_tag = pcgen_ifpc[PC_WIDTH-1:6];                          // pcgen.v:311

    assign pcgen_btb_ifpc = {pcgen_ifpc[PC_WIDTH-1:16], pcgen_fetch_pc[15:0]};       // pcgen.v:314
    assign pred_idpc      = pcgen_pipe_ifpc;                                         // pcgen.v:319 (pcgen_pred_ifpc)

    assign ifu_iu_chgflw_vld = rtu_ifu_chgflw_vld;                                   // pcgen.v:325
    assign ifu_iu_chgflw_pc  = pcgen_delay_chgflw_pc;                                // pcgen.v:326

    //=========================================================================
    // SECTION: CTRL  (aq_ifu_ctrl.v, 132-line hub -- ported near-verbatim).
    // Low-power mode / debug-mask / vec-reset-mask are all dropped per this
    // file's header seam notes (folded to "always inactive"), so
    // `ctrl_inst_fetch` reduces to `ibuf_ctrl_inst_fetch` alone.
    // CONFIRMED (ctrl.v:124, read directly): the real RTL's analogous
    // `ctrl_pcgen_stall` assignment is COMMENTED OUT -- aq_ifu_ctrl.v does
    // NOT gate PCGEN at all. PCGEN's fetch-enable backpressure comes
    // directly from `icache_pcgen_grant` (SECTION PCGEN above), bypassing
    // this hub entirely -- replicated here by the simple absence of any
    // ctrl->pcgen wire, exactly matching the donor.
    // Cancel fan-out: `ctrl_if_cancel` reaches exactly 2 destinations in
    // this build (ICache/IPACK) -- the extraction note's 3rd destination,
    // BTB, doesn't exist as a real predictor yet (no `ctrl_btb_*` port on
    // IFU.v's frozen list).
    //=========================================================================
    // fwd decl: driven in SECTION IBUF below.
    wire ibuf_ctrl_inst_fetch;

    wire ctrl_inst_fetch = ibuf_ctrl_inst_fetch;                                     // ctrl.v:93-96, M1-reduced
    wire ctrl_if_cancel  = rtu_ifu_flush_fe || pcgen_ctrl_chgflw_vld;                // ctrl.v:106

    assign ctrl_icache_req_vld = ctrl_inst_fetch;                                    // ctrl.v:117
    assign ctrl_icache_abort   = ctrl_if_cancel;                                     // ctrl.v:118
    wire   ctrl_ipack_cancel   = ctrl_if_cancel;                                     // ctrl.v:121 (2nd of 2 M1 destinations)

    wire ctrl_ibuf_pop_en = !idu_ifu_id_stall;   // ctrl.v:113 -- THE single point idu_ifu_id_stall is consumed (IFU notes S5.1/S5.2)

    // `pred_ctrl_stall`/`icache_ctrl_stall` only ever fan out to
    // `ctrl_btb_stall` in the real RTL (ctrl.v:127) -- a port that does not
    // exist on IFU.v's frozen list in M1 (no BTB yet). Genuinely unused
    // here, not an oversight; likewise `iu_ifu_pc_mispred` is consumed
    // directly by BPU.v (RAS pointer resync, Task 7), not by IFU.v's own
    // pipeline -- confirmed absent from aq_ifu_pcgen.v's port list.
    // `icache_ipack_acc_err`/`_pgflt` and `pred_ibuf_br_taken0/1` are real
    // frozen ports with no M1 consumer either: the first pair has nowhere
    // to go (no exception path to IDU exists yet, SECTION IPACK below), and
    // the second pair is provably always 0 while BPU.v ties both outputs
    // inactive (SECTION IBUF below, `ifu_idu_id_bht_pred`) -- all four are
    // genuinely unused, not dropped by oversight. (`h1_32bit_vld` WAS in
    // this bucket through Task 3/5 -- see BUG FIX #3 in SECTION IPACK below
    // for why it is genuinely consumed now.)
    wire _unused_ok = &{1'b0, pred_ctrl_stall, icache_ctrl_stall, iu_ifu_pc_mispred,
                         icache_ipack_acc_err, icache_ipack_pgflt,
                         pred_ibuf_br_taken0, pred_ibuf_br_taken1};

    //=========================================================================
    // SECTION: IPACK  (aq_ifu_ipack.v + _entry.v -- 3 flop entries, ported
    // near-verbatim). ICG cells dropped (umbrella spec S6.3, no clock
    // gating cells); acc_err/pgflt/halt_info fields dropped since IFU.v's
    // frozen port list has NO exception path to IDU in M1 (no
    // ifu_idu_id_expt_*/halt_info ports -- confirmed by re-reading the port
    // list above; exception plumbing is an M2 IDU-side addition).
    // `icache_ipack_unalign` IS kept: it is fetch-alignment plumbing (which
    // halfword of the ICache's 32-bit read sits at the requested PC), not
    // fault reporting, and is load-bearing for correct entry1/entry2
    // creation (ipack.v:255).
    //
    // FLAGGED (see Task 3 report): the real ipack.v also reads
    // `pred_ipack_chgflw_vld0`, an `aq_ifu_pred.v` output that cancels a
    // lookahead second instruction -- confirmed (by reading aq_ifu_top.v's
    // wire list directly) to be a DIFFERENT wire from `pred_ibuf_chgflw_vld0`
    // (both exist as separate top-level wires there). It is NOT on IFU.v's
    // frozen port list. Since every BPU.v predictor output is tied inactive
    // for M1 (BPU.v's own skeleton body), this signal is PROVABLY always 0
    // in this milestone's configuration -- tied to a local constant below
    // rather than silently added as a new port. If a later task (7-9) finds
    // BPU needs to drive a real value here, add `pred_ipack_chgflw_vld0` as
    // a new IFU.v input then, with the same justification recorded at the
    // port-list freeze.
    //=========================================================================
    wire pred_ipack_chgflw_vld0 = 1'b0;   // see FLAGGED note above

    wire icache_inst_vld    = icache_ipack_inst_vld && !ctrl_ipack_cancel && !pred_ipack_mask; // ipack.v:253
    wire ipack_align_create = icache_inst_vld && !icache_ipack_unalign;                        // ipack.v:255
    // ibuf_ipack_stall is now a MODULE OUTPUT PORT (Task 7.1 amendment,
    // see the port-list note) driven in SECTION IBUF below -- no separate
    // internal fwd-declared wire needed, the port net serves both roles.
    wire ipack_buf_stall = pred_ipack_ret_stall || ibuf_ipack_stall;                           // ipack.v:256
    wire ipack_buf_flush = rtu_ifu_flush_fe || iu_ifu_tar_pc_vld || rtu_ifu_chgflw_vld;         // ipack.v:251-252, direct RTU/IU wiring (bypasses ctrl hub)

    reg         entry0_vld, entry1_vld, entry2_vld;
    reg  [15:0] entry0_inst, entry1_inst, entry2_inst;

    wire h0_vld       = entry0_vld && entry0_inst[1:0] == 2'b11;                     // ipack.v:373
    wire h1_16bit_vld = entry1_vld && entry1_inst[1:0] != 2'b11;                     // ipack.v:375
    wire h1_32bit_vld = entry1_vld && entry1_inst[1:0] == 2'b11;                     // ipack.v:376 (BUG FIX #3 below: genuinely consumed here)
    wire h2_16bit_vld = entry2_vld && entry2_inst[1:0] != 2'b11;                     // ipack.v:378
    wire h2_32bit_vld = entry2_vld && entry2_inst[1:0] == 2'b11 && !pred_ipack_chgflw_vld0; // ipack.v:379-380

    wire entry0_create_en = (!entry1_vld && h2_32bit_vld
                          || !h0_vld && h1_16bit_vld && h2_32bit_vld
                          || h0_vld && entry1_vld && h2_32bit_vld)
                          && !pred_ipack_chgflw_vld0 && !ipack_buf_stall;            // ipack.v:258-262
    wire entry1_create_en = ipack_align_create
                          && !((entry1_vld || h2_16bit_vld) && ipack_buf_stall)
                          && !pred_ipack_delay_stall;                                // ipack.v:263-265
    wire entry2_create_en = icache_inst_vld
                          && !(entry2_vld && ipack_buf_stall)
                          && !pred_ipack_delay_stall;                                // ipack.v:266-268

    wire [15:0] entry0_upd_inst = entry2_inst;                                       // ipack.v:276, the straddle carry
    wire [15:0] entry1_upd_inst = icache_ipack_inst[15:0];                           // ipack.v:277
    wire [15:0] entry2_upd_inst = icache_ipack_inst[31:16];                          // ipack.v:278

    wire entry0_retire_en = !ipack_buf_stall && entry1_vld;                          // ipack.v:291
    wire entry1_retire_en = !ipack_buf_stall;                                        // ipack.v:292
    wire entry2_retire_en = !ipack_buf_stall && !pred_ipack_delay_stall;             // ipack.v:293

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                     entry0_vld <= 1'b0;
        else if (ipack_buf_flush)       entry0_vld <= 1'b0;
        else if (entry0_create_en)      entry0_vld <= 1'b1;
        else if (entry0_retire_en)      entry0_vld <= 1'b0;
    end
    always @(posedge clk) begin
        if (entry0_create_en) entry0_inst <= entry0_upd_inst;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                     entry1_vld <= 1'b0;
        else if (ipack_buf_flush)       entry1_vld <= 1'b0;
        else if (entry1_create_en)      entry1_vld <= 1'b1;
        else if (entry1_retire_en)      entry1_vld <= 1'b0;
    end
    always @(posedge clk) begin
        if (entry1_create_en) entry1_inst <= entry1_upd_inst;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                     entry2_vld <= 1'b0;
        else if (ipack_buf_flush)       entry2_vld <= 1'b0;
        else if (entry2_create_en)      entry2_vld <= 1'b1;
        else if (entry2_retire_en)      entry2_vld <= 1'b0;
    end
    always @(posedge clk) begin
        if (entry2_create_en) entry2_inst <= entry2_upd_inst;
    end

    // ---- Valid instruction package (ipack.v:382-424) -----------------------
    wire        ipack_first_vld  = entry1_vld || (!h0_vld && h2_16bit_vld);          // ipack.v:388
    wire [31:0] ipack_first_inst = entry0_vld ? {entry1_inst, entry0_inst}
                                 : entry1_vld ? {entry2_inst, entry1_inst}
                                              : {entry2_inst, entry2_inst};          // ipack.v:389-391
    wire        ipack_secnd_vld  = (h0_vld || h1_16bit_vld) && h2_16bit_vld;         // ipack.v:393
    wire [15:0] ipack_secnd_inst = entry2_inst;                                      // ipack.v:395

    // ipack_one_16bit_vld (ipack.v:399-402) with pred_ipack_chgflw_vld0(=0
    // const)/pred_ipack_delay_stall(real port) terms simplified: the real
    // formula ANDs in `!(h2_16bit_vld && !pred_ipack_chgflw_vld0 &&
    // !pred_ipack_delay_stall)`; since the const-0 factor is always true,
    // this reduces to `!(h2_16bit_vld && !pred_ipack_delay_stall)`.
    wire ipack_one_16bit_vld = (!h0_vld && h1_16bit_vld && !(h2_16bit_vld && !pred_ipack_delay_stall))
                            || (!entry1_vld && h2_16bit_vld);
    wire ipack_all_vld = h0_vld && entry1_vld && h2_16bit_vld
                      && !pred_ipack_chgflw_vld0 && !pred_ipack_delay_stall;         // ipack.v:412-414

    // BUG FIX #3 (Task 6 bring-up, found running iss_selftest.S at rung 1:
    // pc=0x104's `addi x0,x0,0` -- a lone 32-bit instruction filling
    // entry1+entry2 with nothing else valid, no carry -- was silently
    // dropped; entry1/entry2 got overwritten by the next cycle's fresh
    // fetch before ever reaching IBUF, and the eventual push finally fired
    // several cycles later carrying whatever WRONG-PATH halfwords happened
    // to be sitting in entry1/entry2 by then). None of `ipack_one_16bit_vld`/
    // `ipack_secnd_vld`/`ipack_all_vld` (ipack.v:399-414, ported verbatim
    // above) cover this shape -- confirmed this is a REAL gap, not a
    // misreading, by finding the exact needed term COMMENTED OUT in the
    // real aq_ifu_ipack.v source:
    //   //assign ipack_one_32bit_vld = !h0_vld && h1_32bit_vld && entry2_vld
    //   //                           || h0_vld && entry1_vld && h2_32bit_vld;
    //   //assign ipack_retire_two = ipack_one_32bit_vld || ipack_two_16bit_vld;
    // i.e. real silicon's `ibuf.v` evidently does its OWN, fuller create-side
    // classification directly off h0_vld/h1_16bit_vld/h2_16bit_vld/entry-
    // valids (extraction note S9: "IBUF's mirrored create-side arbitration...
    // ibuf.v:1259-1313", flagged there as not fully traced) rather than
    // trusting ipack.v's three named retire-count flags as a complete
    // enumeration -- those three flags are demonstrably incomplete even in
    // the real chip. rv906's binary-pointer IBUF (this file's own SECTION
    // IBUF header note) chose to trust exactly those three flags as its sole
    // push-count source, so the gap real hardware papers over inside ibuf.v
    // is a live, reachable bug here -- and a common one: it fires on EVERY
    // standalone 32-bit instruction with nothing else valid the same cycle,
    // which is the ordinary case for any straight-line run of non-RVC code.
    // FIX: reinstate the real RTL's own (commented-out) formula, both
    // OR-terms -- the first covers "entry1 starts + entry2 completes,
    // no carry"; the second covers "carry(h0)+entry1 complete a 32-bit
    // instruction the SAME cycle entry2 starts a fresh straddle" (the
    // mirror-image case `entry0_create_en`'s own third OR-term already
    // carries entry2 forward for). Both retire exactly 2 halfwords, and
    // `ipack_first_inst`'s existing entry0-vs-entry1 priority mux (above)
    // already reconstructs the right pair for either case, so this folds
    // cleanly into the existing `ipack_ibuf_inst_two` slot rather than
    // needing a fourth push-count value.
    wire ipack_one_32bit_vld = (!h0_vld && h1_32bit_vld && entry2_vld)
                            || (h0_vld && entry1_vld && h2_32bit_vld);

    wire        ipack_retire_vld  = entry1_vld || h2_16bit_vld;                      // ipack.v:419
    wire [47:0] ipack_retire_inst = {entry2_inst, ipack_first_inst};                 // ipack.v:424

    // ---- Output to ibuf (ipack.v:456-497) ----------------------------------
    wire ipack_ibuf_inst_vld_raw = ipack_retire_vld;
    wire ipack_ibuf_inst_vld     = ipack_retire_vld && !ipack_buf_stall;
    wire ipack_ibuf_inst_one     = ipack_one_16bit_vld;
    wire ipack_ibuf_inst_two     = (ipack_secnd_vld || ipack_one_32bit_vld)
                                 && !pred_ipack_chgflw_vld0 && !pred_ipack_delay_stall; // BUG FIX #3
    wire ipack_ibuf_inst_all     = ipack_all_vld;
    wire [47:0] ipack_ibuf_inst  = ipack_retire_inst;

    assign ipack_pcgen_reissue = ibuf_ipack_stall && icache_inst_vld;                // ipack.v:491

    assign ipack_pred_inst0_vld = ipack_first_vld;
    assign ipack_pred_inst0     = ipack_first_inst;
    assign ipack_pred_inst1_vld = ipack_secnd_vld;
    assign ipack_pred_inst1     = ipack_secnd_inst;
    assign ipack_pred_h0_create = entry0_create_en;
    assign ipack_pred_h0_vld    = h0_vld;
    assign ipack_pred_unalign   = !entry1_vld && h2_16bit_vld;                       // ipack.v:472

    //=========================================================================
    // SECTION: IBUF  (aq_ifu_ibuf.v + _entry.v + _pop_entry.v -- 6-entry x
    // 16-bit circular halfword queue, push<=3/pop<=2, exactly ONE 32-bit
    // instruction/cycle to IDU (ibuf.v:1354-1356), the single-issue decode
    // boundary). `ctrl_ibuf_pop_en` (SECTION CTRL, = !idu_ifu_id_stall) is
    // the ONLY point `idu_ifu_id_stall` crosses into IFU control, per the
    // extraction note's confirmed topology (S5.2) -- consumed here, nowhere
    // else. IBUF's own flush is wired directly from RTU/PCGEN
    // (`ibuf_flush_en` below), bypassing the ctrl hub, exactly as ibuf.v:657
    // does (`rtu_ifu_flush_fe || pcgen_ibuf_chgflw_vld`, NOT `ctrl_if_cancel`).
    //
    // IMPLEMENTATION NOTE (documented deviation, not a silent guess -- see
    // Task 3 report): the real RTL implements this queue with one-hot
    // rotate-register push/pop pointers (push0/1/2, pop0/1) plus a separate
    // 2-deep "pop_entry" staging pair and an "ipack bypass when empty" fast
    // path that skips the queue for 1 cycle of latency when it's empty
    // (ibuf.v:938-971,1256-1330). rv906 replaces this with a functionally
    // equivalent BINARY head-pointer + occupancy-counter circular buffer
    // (same 6-entry depth, same push<=3/pop<=2 discipline, same FIFO
    // ordering, same full/stall conditions) WITHOUT the bypass fast path --
    // this costs up to a few extra cycles of fetch latency when the queue
    // drains to empty, but changes no committed instruction's value or
    // order (the M1 oracle is explicitly latency-agnostic, design doc S4.1:
    // "predictors change WHEN an instruction is fetched, never WHICH
    // instructions commit", and the same reasoning applies to this
    // queue-implementation choice). Chosen over a literal one-hot-per-entry
    // port because (a) the one-hot rotate scheme is an ASIC area/timing
    // trick with no externally observable behavioral difference from a
    // binary-pointer FIFO of the same depth/width, and (b) it matches this
    // project's "entry arrays, not per-entry modules" convention (design
    // doc S6.2).
    //=========================================================================
    reg  [15:0] ibuf_mem [0:5];
    reg  [2:0]  ibuf_head;     // 0..5, head-of-queue (oldest halfword) pointer
    reg  [2:0]  ibuf_count;    // 0..6, occupancy

    // `pred_ibuf_chgflw_vld0` (real, frozen port, tied 0 by BPU.v in M1)
    // faithfully gates ONLY the 3rd/newest halfword, exactly as
    // `ibuf_create2_en`'s own `!pred_ibuf_chgflw_vld0` term does in the real
    // RTL (ibuf.v:966-968) -- create0/create1 do NOT depend on it there, so
    // a cancelled 3rd halfword falls back to pushing 2, not 0.
    wire [1:0] ibuf_push_count_raw =
          ipack_ibuf_inst_all ? (pred_ibuf_chgflw_vld0 ? 2'd2 : 2'd3)
        : ipack_ibuf_inst_two ? 2'd2
        : ipack_ibuf_inst_one ? 2'd1
                              : 2'd0;
    wire [1:0] ibuf_push_count = ipack_ibuf_inst_vld ? ibuf_push_count_raw : 2'd0;

    // ibuf_entry_stall (ibuf.v:1190-1194, M1-reduced: the DTU halt-info term
    // is dropped, no DTU exists in M1) -- room check against the CURRENT
    // (pre-this-cycle) occupancy, using the RAW (ungated) push request so
    // this does not form a combinational loop through ipack_buf_stall.
    wire ibuf_entry_stall = ipack_ibuf_inst_vld_raw
                         && ({2'd0, ibuf_push_count_raw} > (4'd6 - {1'b0, ibuf_count}));
    assign ibuf_ipack_stall = ibuf_entry_stall;                                      // ibuf.v:1340
    assign ibuf_ctrl_inst_fetch = !ibuf_entry_stall;                                 // ibuf.v:1195,1337 (ibuf_inst_fetch is hardwired 1 in the real RTL)

    wire ibuf_flush_en = rtu_ifu_flush_fe || pcgen_ibuf_chgflw_vld;                  // ibuf.v:657

    // -- pop side: combinational head-of-queue peek. Content at `ibuf_head`
    // cannot change while idu_ifu_id_stall is asserted (pop below is gated
    // on ctrl_ibuf_pop_en, and push only ever writes the TAIL), so this is
    // stall-stable without a separate registered staging pair.
    wire [2:0]  ibuf_head1  = (ibuf_head == 3'd5) ? 3'd0 : ibuf_head + 3'd1;
    wire        ibuf_h0_vld = (ibuf_count != 3'd0);
    wire [15:0] ibuf_h0     = ibuf_mem[ibuf_head];
    wire        ibuf_h0_32  = ibuf_h0[1:0] == 2'b11;
    wire        ibuf_h1_vld = (ibuf_count >= 3'd2);
    wire [15:0] ibuf_h1     = ibuf_mem[ibuf_head1];

    wire       pop_entry_vld  = ibuf_h0_vld && (!ibuf_h0_32 || ibuf_h1_vld);
    wire [1:0] ibuf_pop_count = pop_entry_vld ? (ibuf_h0_32 ? 2'd2 : 2'd1) : 2'd0;
    wire       ibuf_pop_fire  = pop_entry_vld && ctrl_ibuf_pop_en;

    wire [3:0] ibuf_head_next_raw = {1'b0, ibuf_head} + {2'b00, ibuf_pop_count};     // max 5+2=7, fits 4b with room to spare
    wire [2:0] ibuf_head_next = (ibuf_head_next_raw >= 4'd6) ? (ibuf_head_next_raw[2:0] - 3'd6)
                                                              : ibuf_head_next_raw[2:0];

    // Tail write: up to 3 halfwords at {head+count, +1, +2} mod 6, in
    // natural program order (ipack_ibuf_inst[15:0]=oldest .. [47:32]=newest,
    // ipack.v:424) -- there is no bypass path to special-case (see note
    // above), so the push side never needs to reorder or skip a slot.
    wire [3:0] ibuf_tail0_raw = {1'b0, ibuf_head} + {1'b0, ibuf_count};              // max 5+6=11, needs the 4th bit
    wire [2:0] ibuf_tail0 = (ibuf_tail0_raw >= 4'd6) ? (ibuf_tail0_raw[2:0] - 3'd6) : ibuf_tail0_raw[2:0];
    wire [2:0] ibuf_tail1 = (ibuf_tail0 == 3'd5) ? 3'd0 : ibuf_tail0 + 3'd1;
    wire [2:0] ibuf_tail2 = (ibuf_tail1 == 3'd5) ? 3'd0 : ibuf_tail1 + 3'd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ibuf_head  <= 3'd0;
            ibuf_count <= 3'd0;
        end
        else if (ibuf_flush_en) begin
            ibuf_head  <= 3'd0;
            ibuf_count <= 3'd0;
        end
        else begin
            if (ibuf_pop_fire) ibuf_head <= ibuf_head_next;
            // Safe at 3b width: ibuf_push_count_raw is bounded above by
            // (6 - ibuf_count) whenever ibuf_entry_stall would otherwise
            // gate it off (see ibuf_entry_stall above), so the true value of
            // (count - pop + push) never exceeds 6 regardless of pop this
            // cycle -- no overflow/underflow at any intermediate step.
            ibuf_count <= ibuf_count - {1'b0, (ibuf_pop_fire ? ibuf_pop_count : 2'd0)}
                                     + {1'b0, ibuf_push_count};
        end
    end

    always @(posedge clk) begin
        if (ibuf_push_count >= 2'd1) ibuf_mem[ibuf_tail0] <= ipack_ibuf_inst[15:0];
        if (ibuf_push_count >= 2'd2) ibuf_mem[ibuf_tail1] <= ipack_ibuf_inst[31:16];
        if (ibuf_push_count >= 2'd3) ibuf_mem[ibuf_tail2] <= ipack_ibuf_inst[47:32];
    end

    // ---- Output to IDU (frozen ports) --------------------------------------
    assign ifu_idu_id_inst_vld = pop_entry_vld;                                      // ibuf.v:1354
    assign ifu_idu_id_inst     = {ibuf_h1, ibuf_h0};                                  // ibuf.v:1355-1356
    // ifu_idu_id_bht_pred (ibuf.v:1357-1358) rides pred_ibuf_br_taken{0,1}
    // alongside each halfword in the real RTL. BPU.v's Task-1 skeleton ties
    // BOTH br_taken0/1 to 2'b00 for as long as every predictor stays
    // disabled, so this field is PROVABLY always 0 in M1's configuration --
    // per-halfword tracking through the queue is deferred to Tasks 7-9,
    // which need to touch this section anyway to wire the RAS/BTB/BHT
    // redirect levels in; building unused plumbing for it now would be
    // pure waste (flagged in the Task 3 report, not a silent gap).
    assign ifu_idu_id_bht_pred = 2'b00;

endmodule
