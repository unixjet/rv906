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
    // M4 Task 6 (S13, D2): fetch-exception tags, riding alongside the
    // instruction exactly the way ifu_idu_id_bht_pred does (SECTION IBUF's
    // ibuf_fault_tag[] mirrors ibuf_tag[]'s own push/pop indexing). Set
    // when this "instruction" is actually a synthetic fault marker
    // (SECTION IPACK) -- IDU forces it to CP0/EU_CP0 dispatch, mirroring
    // the illegal-instruction path, and CSR.v traps vec 12/1.
    output wire                     ifu_idu_id_fault_pgflt,
    output wire                     ifu_idu_id_fault_accflt,
    // M7 Task 2: the DTU's execute-trigger halt_info for the instruction
    // currently at IFU's output, riding exactly the way
    // ifu_idu_id_bht_pred does (live sideband at the ibuf head -- the
    // donor's aq_ifu_pred.v PRED-stage position, see the feed note at the
    // file end). 0 when the DTU saw no execute-trigger match this cycle.
    output wire [TDT_HINFO_WIDTH-1:0] ifu_idu_id_halt_info,
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
    // M8 T4b FIX (2nd round): the PRED-stage ARRIVAL event, now a BPU input
    // (BPU.v's `pred_idpc_r` load gate -- see BPU.v's port note for the
    // donor mapping). Same net this file already computes as
    // `icache_inst_vld` in SECTION IPACK (ipack.v:253: "this cycle's fetch
    // data is real and not cancelled/masked") -- exposed as a port now.
    output wire                     icache_inst_vld,
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
    input  wire [PC_WIDTH-1:0]      cp0_xx_mrvbr,

    //=========================================================================
    // M7 Task 1: DTU debug ports (donor aq_dtu_top.v's ifu side +
    // aq_ifu_vec.v's halt-on-reset). dtu_ifu_debug_inst[_vld] is the DM's
    // itr instruction channel (aq_ifu_ibuf.v:949,1085,1098: while dbgon and
    // the vld pulse is live, the fetch entry is the injected word, not an
    // ICache fetch). rtu_yy_xx_dbgon masks ICache fetch while in/entering
    // debug (the donor masks on rtu_ifu_dbg_mask = dbg_mode_on_after_req,
    // aq_ifu_ctrl.v:95; rv906 uses the real rtu_yy_xx_dbgon = dbg_mode_on,
    // the same shape the ibuf injection mux uses). dtu_ifu_halt_on_reset
    // arms the reset-halt; ifu_rtu_reset_halt_req is the timing-0 halt
    // request back to the RTU (aq_ifu_vec.v:279).
    //=========================================================================
    input  wire [31:0]              dtu_ifu_debug_inst,
    input  wire                     dtu_ifu_debug_inst_vld,
    input  wire                     rtu_yy_xx_dbgon,
    input  wire                     dtu_ifu_halt_on_reset,
    output wire                     ifu_rtu_reset_halt_req,

    //=========================================================================
    // M7 Task 2: DTU execute-trigger channel. ifu_dtu_exe_addr[_vld] is the
    // PC of the instruction currently at the ibuf head (the one about to be
    // delivered to IDU this cycle), fed to the DTU's mcontrol execute
    // comparators. dtu_ifu_halt_info[_vld] is the DTU's single-cycle match
    // verdict, delivered live to IDU as ifu_idu_id_halt_info the same cycle
    // (the same sideband pattern as ifu_idu_id_bht_pred / the fault tags).
    // Matching at the head -- one stage behind the icache fetch -- is what
    // places the check at the donor's aq_ifu_pred.v:745-758 PRED-stage
    // position: a trigger armed while an instruction is still in flight
    // (fetched but not yet delivered) catches it at the head, which the
    // stock rv64mi-p-breakpoint test relies on (tdata1 write and tripwire
    // are 16 bytes apart in the same fetch burst).
    //=========================================================================
    output wire [PC_WIDTH-1:0]      ifu_dtu_exe_addr,
    output wire                     ifu_dtu_exe_addr_vld,
    input  wire [TDT_HINFO_WIDTH-1:0] dtu_ifu_halt_info,
    input  wire                     dtu_ifu_halt_info_vld
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
    // M7 Task 1: HALT-ON-RESET (donor aq_ifu_vec.v:157-209's HALT state,
    // reduced to rv906's RESET->RUN boot): after rst_n release, the first
    // fetch with dtu_ifu_halt_on_reset set raises ifu_rtu_reset_halt_req
    // (donor :279 `ifu_rtu_reset_halt_req = vec_sm_halt`) -- a one-cycle
    // pulse the RTU takes as a timing-0 halt (cause 5). The donor's level
    // (HALT state held one cycle) vs rv906's pulse is the same observable
    // timing: vec_sm_halt is asserted for exactly one cycle too
    // (vec.v:196-199 HALT->IDLE unconditionally).
    //=========================================================================
    reg reset_halt_req_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) reset_halt_req_r <= 1'b0;
        else        reset_halt_req_r <= dtu_ifu_halt_on_reset && boot_rst_vld;
    end
    assign ifu_rtu_reset_halt_req = reset_halt_req_r;

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

    // M8 T4b FIX (2nd round): GRANT-latched fetch pointer, aq_ifu_pcgen.v:
    // 285-293 verbatim shape. HISTORY: BUG FIX #4 (Task 9) re-timed this
    // register from "pcgen_fetch_pc on every icache_pcgen_grant" to
    // "icache_pcgen_addr on icache_inst_vld" (the arrival) after dense_br.S
    // region C corrupted the stream -- because at the time the BPU
    // CONSUMED THIS RAW POINTER DIRECTLY as its branch base PC, so any
    // grant that outran IPACK's held entries (delay/replay backlog)
    // mis-keyed every derived address by a word. That fix masked the race
    // at the source instead of fixing the architecture. The DONOR's
    // answer is two-part: keep the GRANT latch (this register is meant to
    // be the PRED-stage-aligned pointer, which the grant latch provides
    // EXACTLY one cycle early -- the arrival cycle -- because icache data
    // lands one cycle after its grant), and put the alignment REGISTER in
    // the BPU (BPU.v's `pred_idpc_r`, donor aq_ifu_pred.v:444-453), which
    // loads this pointer on the arrival event (`icache_inst_vld`, now a
    // BPU input too) and HOLDS it across the cycles the PRED stage lags
    // or is stalled (IBUF full / RAS WAIT / post-redirect gap). With the
    // BPU register in place the grant latch is safe again: in steady state
    // the raw pointer leads the displayed bundle by exactly the one cycle
    // the register's arrival-load absorbs, and on a redirect the raw
    // pointer re-latches the target on the first post-redirect grant so
    // the register is correct from the target's first display cycle
    // (the interrupt test's trap-to-0x80000400 case that the 1st-round
    // arrival-latch + entry-valid-load version got one bundle stale on).
    // `boot_rst_vld` keeps the unconditional top-priority term per BUG FIX
    // #2's audit (this register is never re-consulted for fetch
    // addressing; the donor's own vec_pcgen_rst_vld has the same
    // asymmetry, aq_ifu_pcgen.v:287-288).
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

    // M7 Task 1: the donor's debug fetch mask (aq_ifu_ctrl.v:93-96,
    // `ctrl_inst_fetch = ibuf_ctrl_inst_fetch && !(lpmd) && !rtu_ifu_dbg_mask
    // && !reset_mask`) -- ICache fetch stops while in debug mode so the only
    // instruction flow is the DTU's itr injection (SECTION IBUF below).
    wire ctrl_inst_fetch = ibuf_ctrl_inst_fetch && !rtu_yy_xx_dbgon;         // ctrl.v:93-96, M7: dbg_mask real
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
    // `pred_ibuf_br_taken0/1` WERE in this bucket through Task 8 (BPU.v
    // tied both to 0 with no BHT built) -- TASK 9: now real, consumed by
    // `ibuf_tag[]`'s push logic above, removed from this bucket.
    // (`h1_32bit_vld` WAS in this bucket through Task 3/5 -- see BUG FIX #3
    // in SECTION IPACK below for why it is genuinely consumed now.)
    // M4 Task 6: `icache_ipack_acc_err`/`_pgflt` moved OUT of this bucket --
    // genuinely consumed now, SECTION IPACK below.
    wire _unused_ok = &{1'b0, pred_ctrl_stall, icache_ctrl_stall, iu_ifu_pc_mispred};

    //=========================================================================
    // SECTION: IPACK  (aq_ifu_ipack.v + _entry.v -- 3 flop entries, ported
    // near-verbatim). ICG cells dropped (umbrella spec S6.3, no clock
    // gating cells); halt_info fields dropped (no DTU debug-trigger channel
    // exists). M4 Task 6: acc_err/pgflt ARE now consumed -- see the fault
    // injection below (design doc S13/D2).
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
    // frozen port list. Since every BPU.v predictor output was tied inactive
    // for M1 (BPU.v's own skeleton body), this signal was PROVABLY always 0
    // in that configuration -- tied to a local constant below rather than
    // silently added as a new port.
    //
    // TASK 4c FIX (M8, coremark + m2/m3/m4 hang): the tie-0 was the live
    // bug. Once the BPU was brought up real (Task 9), a predicted-taken
    // branch at IPACK slot 0 must cancel the fall-through carry's create/
    // retire for the SAME cycle, exactly as the donor does. The FLAGGED
    // premise ("a DIFFERENT wire from pred_ibuf_chgflw_vld0") is WRONG
    // against the real source: aq_ifu_pred.v:713,780,786 drive BOTH
    // `pred_ipack_chgflw_vld0` and `pred_ibuf_chgflw_vld0` from the SAME
    // wire (`pred_chgflw_vld0 = pred_br_taken0 || pred_ras_ret_vld0`).
    // BPU.v:1180 already drives that exact formula out as
    // `pred_ibuf_chgflw_vld0`, so no new IFU.v port / BPU.v output is
    // needed -- alias it here.
    //=========================================================================
    wire pred_ipack_chgflw_vld0 = pred_ibuf_chgflw_vld0;  // donor aq_ifu_pred.v:780,786

    assign icache_inst_vld   = icache_ipack_inst_vld && !ctrl_ipack_cancel && !pred_ipack_mask; // ipack.v:253
    wire ipack_align_create = icache_inst_vld && !icache_ipack_unalign;                        // ipack.v:255

    //-------------------------------------------------------------------------
    // M4 Task 6 (S13, D2): FETCH-FAULT INJECTION. `icache_ipack_acc_err`/
    // `_pgflt` ride WITH `icache_ipack_inst_vld` (ICache.v:459-460 FORCES
    // icache_hit -- hence icache_ipack_inst_vld -- true on a fault; the
    // "instruction" word itself is undefined array garbage). Rather than
    // let that garbage flow through the bit-pattern-driven h0/h1/h2
    // splicer below (which decides 16b-vs-32b/entry-count purely from the
    // instruction bits it sees -- undefined behavior on undefined input),
    // a fault OVERRIDES this cycle's contribution with a synthetic 32-bit
    // NOP (`ADDI x0,x0,0` = 32'h00000013, entry1=0x0013 [1:0]=11 "starts a
    // 32-bit instr", entry2=0x0000): a well-formed, deterministic shape
    // the EXISTING splicer handles completely unmodified, always retiring
    // via `ipack_one_32bit_vld`'s first term (h1_32bit_vld && entry2_vld,
    // BUG FIX #3 below) PROVIDED no straddle-carry (h0_vld) is pending --
    // see the entry0_vld always block below for why that's guaranteed.
    // The synthetic bits never architecturally execute (IDU forces this
    // exact shape to CP0/EU_CP0 dispatch below, mirroring the illegal-
    // instruction override) -- their only job is to occupy a clean,
    // predictable slot in the halfword pipe long enough to carry the
    // fault-tag registers (entry1_fault_pgflt_r/_accflt_r, below) to IBUF.
    //
    // STRADDLE EDGE CASE (documented limitation, not silently dropped):
    // if h0_vld is ALREADY set (a 32-bit instruction's first half was
    // fetched successfully last cycle and is still awaiting its second
    // half) the SAME cycle a fault fires, that straddling instruction's
    // second half is genuinely unfetchable (its half lives on the very
    // fetch that just faulted) -- architecturally it too must fault. This
    // implementation does not attempt to preserve THAT instruction's own
    // start PC for the trap (doing so would require carrying a PC field
    // through the halfword pipe end to end, a much larger change than
    // Task 6's own scope): the entry0_vld always block below
    // UNCONDITIONALLY drops any pending carry the same cycle a fault
    // fires, so the trap is instead reported at the FAULTING fetch's own
    // PC. Still traps correctly (RTU still redirects away from the
    // unexecutable page), just with a less precise epc/tval for this one
    // narrow case -- flagged here for a later milestone to tighten if a
    // test ever needs the exact straddling PC.
    //-------------------------------------------------------------------------
    wire ipack_fault_pgflt  = icache_inst_vld && icache_ipack_pgflt;
    wire ipack_fault_accflt = icache_inst_vld && icache_ipack_acc_err && !icache_ipack_pgflt;
    wire ipack_fault        = ipack_fault_pgflt || ipack_fault_accflt;
    // ibuf_ipack_stall is now a MODULE OUTPUT PORT (Task 7.1 amendment,
    // see the port-list note) driven in SECTION IBUF below -- no separate
    // internal fwd-declared wire needed, the port net serves both roles.
    wire ipack_buf_stall = pred_ipack_ret_stall || ibuf_ipack_stall;                           // ipack.v:256
    wire ipack_buf_flush = rtu_ifu_flush_fe || iu_ifu_tar_pc_vld || rtu_ifu_chgflw_vld;         // ipack.v:251-252, direct RTU/IU wiring (bypasses ctrl hub)

    reg         entry0_vld, entry1_vld, entry2_vld;
    reg  [15:0] entry0_inst, entry1_inst, entry2_inst;

    wire h0_vld      = entry0_vld && entry0_inst[1:0] == 2'b11;                     // ipack.v:373
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
    // M4 Task 6: the synthetic-NOP override (see the fault injection note
    // above) -- 16'h0013 is the LOWER half (`ADDI x0,x0,0`'s [1:0]=11 own
    // half), 16'h0000 the UPPER half.
    wire [15:0] entry1_upd_inst = ipack_fault ? 16'h0013 : icache_ipack_inst[15:0];   // ipack.v:277
    wire [15:0] entry2_upd_inst = ipack_fault ? 16'h0000 : icache_ipack_inst[31:16];  // ipack.v:278

    wire entry0_retire_en = !ipack_buf_stall && entry1_vld;                          // ipack.v:291
    wire entry1_retire_en = !ipack_buf_stall;                                        // ipack.v:292
    wire entry2_retire_en = !ipack_buf_stall && !pred_ipack_delay_stall;             // ipack.v:293

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                     entry0_vld <= 1'b0;
        else if (ipack_buf_flush)       entry0_vld <= 1'b0;
        // M4 Task 6: drop any pending straddle-carry the same cycle a fault
        // fires (the STRADDLE EDGE CASE note above) -- placed ahead of
        // entry0_create_en in this priority chain so a fault always wins.
        else if (ipack_fault)           entry0_vld <= 1'b0;
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
    // M4 Task 6: the fault tag rides alongside entry1_inst (entry1 always
    // carries the synthetic NOP's defining lower half, [1:0]==11, whenever
    // ipack_fault fired at creation) -- same no-reset style as entry1_inst
    // itself (read only while entry1_vld is set, matching the array's own
    // discipline).
    reg entry1_fault_pgflt_r, entry1_fault_accflt_r;
    always @(posedge clk) begin
        if (entry1_create_en) begin
            entry1_fault_pgflt_r  <= ipack_fault_pgflt;
            entry1_fault_accflt_r <= ipack_fault_accflt;
        end
    end

    // M7 Task 2: per-entry PC tracking for the execute-trigger comparator
    // feed (the head-PC read out below rides the ibuf like ibuf_tag[]).
    // Each PC latches with its entry's OWN create_en -- the exact same
    // clocked pairing as the entryN_inst registers above (same no-reset
    // discipline: read only while the entry's vld is set).
    //   entry1: fetch word's LOW half  -> base PC (icache_pcgen_addr)
    //   entry2: fetch word's HIGH half -> base PC + 2 -- EXCEPT an unaligned
    //           fetch (branch target with bit1 set): icache_pcgen_addr then
    //           holds the UNALIGNED target itself (pcgen_fetch_pc is passed
    //           raw, ICache.v:268,584), the icache aligns DOWN for the read,
    //           and the stream starts at the word's HIGH half -- which sits
    //           AT icache_pcgen_addr, not +2. (The low half is 2B BEFORE the
    //           target and is never created: ipack_align_create's own
    //           !icache_ipack_unalign gate, line above.)
    //   entry0: straddle carry = this word's entry2 (see entry0_upd_inst
    //           above) -> entry2_pc_r
    // The pushed halfwords are always 2B-consecutive in program order
    // (the halfword queue's own invariant), so the push PC needs only the
    // OLDEST half's PC (+2/+4 for tail1/tail2).
    reg [PC_WIDTH-1:0] entry0_pc_r, entry1_pc_r, entry2_pc_r;
    always @(posedge clk) begin
        if (entry1_create_en) entry1_pc_r <= icache_pcgen_addr;
    end
    always @(posedge clk) begin
        if (entry2_create_en)
            entry2_pc_r <= icache_pcgen_addr
                         + (icache_ipack_unalign ? {PC_WIDTH{1'b0}}
                                                  : {{(PC_WIDTH-2){1'b0}}, 2'd2});
    end
    always @(posedge clk) begin
        if (entry0_create_en) entry0_pc_r <= entry2_pc_r;
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
    wire        ipack_first_vld = entry1_vld || (!h0_vld && h2_16bit_vld);          // ipack.v:388
    wire [31:0] ipack_first_inst = entry0_vld ? {entry1_inst, entry0_inst}
                                 : entry1_vld ? {entry2_inst, entry1_inst}
                                              : {entry2_inst, entry2_inst};          // ipack.v:389-391
    wire        ipack_secnd_vld = (h0_vld || h1_16bit_vld) && h2_16bit_vld;         // ipack.v:393
    wire [15:0] ipack_secnd_inst = entry2_inst;                                      // ipack.v:395

    // ipack_one_16bit_vld -- verbatim donor formula (ipack.v:399-402).
    // (TASK 4c: this was previously simplified under the assumption that
    // pred_ipack_chgflw_vld0 was constant 0 -- the assumption that caused
    // the M8 coremark hang. The chgflw term inside the negation is
    // load-bearing: it lets a 16-bit predicted-taken branch at slot 0
    // retire as exactly ONE halfword even when slot 2 holds a valid
    // fall-through halfword, which must NOT be pushed or carried.)
    wire ipack_one_16bit_vld = (!h0_vld && h1_16bit_vld && !(h2_16bit_vld
                                 && !pred_ipack_chgflw_vld0
                                 && !pred_ipack_delay_stall))
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

    // BUG FIX #5 (Task 9 bring-up, found running dense_br.S's region
    // C/D boundary through the full RTL: a 32-bit instruction straddling
    // two fetch words -- h0_vld's carry case -- silently vanished from the
    // committed stream whenever its OWN completing cycle also classified
    // entry2 as a valid, DELAYED con_br partner). None of
    // `ipack_one_16bit_vld`/`ipack_one_32bit_vld`/`ipack_secnd_vld`/
    // `ipack_all_vld` cover "h0_vld && entry1_vld (a complete 32-bit
    // instruction) && entry2_vld is ALSO a valid con_br, but
    // `pred_ipack_delay_stall` (Task 9's delay/replay mechanism, BPU.v
    // SECTION BHT part (k)) is deferring it" -- `ipack_all_vld` is the
    // only formula that would otherwise retire h0+entry1+entry2 together,
    // and it is UNCONDITIONALLY killed by `!pred_ipack_delay_stall`
    // (matching the plain-RVC `ipack_one_16bit_vld` case's OWN documented
    // precedent of retiring the COMPLETE, non-deferred piece alone and
    // leaving entry2 for a later cycle) -- but nothing steps in to retire
    // the NOW-COMPLETE h0+entry1 pair by itself the way
    // `ipack_one_16bit_vld` does for a plain RVC entry1. Confirmed via a
    // temporary C++ probe (bring-up only, not left in the final RTL):
    // `entry0_retire_en`/`entry1_retire_en` (SECTION IBUF's own formulas,
    // unchanged) still unconditionally clear entry0/entry1 this cycle
    // regardless of push count, so the 32-bit instruction's VALID BITS
    // cleared while ZERO halfwords were ever pushed -- a silent drop, not
    // merely a stale value. THE FIX: a dedicated retire-2 term for exactly
    // this combination, OUTSIDE the `!pred_ipack_delay_stall` gate that
    // (correctly) suppresses `ipack_all_vld`/`ipack_secnd_vld`'s own
    // 3-piece and slot1-inclusive pushes -- this is NOT a case the real
    // RTL's own commented-out formula (BUG FIX #3 above) anticipated,
    // since it predates Task 9's delay mechanism entirely; reasoned and
    // derived locally from this file's own entry-retirement invariants,
    // not found pre-existing in the real source.
    // TASK 4c: the trigger is now the UNION
    // `(pred_ipack_delay_stall || pred_ipack_chgflw_vld0)` (was
    // `&& pred_ipack_delay_stall && !pred_ipack_chgflw_vld0`). The same
    // "complete h0+entry1 32-bit retires as a pair, third half discarded"
    // shape ALSO occurs when that 32-bit instruction IS the predicted-taken
    // branch at slot 0 (chgflw=1, e2 = fall-through that must be dropped).
    // The donor covers that shape inside its IBUF's mirrored create-side
    // classification (aq_ifu_ibuf.v:1259-1313), which rv906's binary IBUF
    // does not have (see BUG FIX #3 above: the flags are this design's sole
    // push-count source) -- so the pair must retire HERE or the taken
    // branch silently drops (entries retire unconditionally via
    // entry0/1_retire_en). The two triggers are mutually exclusive:
    // chgflw needs bht_pred_rslt[1] or a jump at slot 0
    // (aq_ifu_pred.v:592), delay_stall needs a NOT-taken branch at slot 0
    // (aq_ifu_pred.v:546) -- one instruction cannot be both -- so the OR
    // adds no new combination beyond the two covered shapes.
    wire ipack_h0_delay_vld = h0_vld && entry1_vld
                            && (pred_ipack_delay_stall || pred_ipack_chgflw_vld0);

    wire        ipack_retire_vld  = entry1_vld || h2_16bit_vld;                      // ipack.v:419
    wire [47:0] ipack_retire_inst = {entry2_inst, ipack_first_inst};                 // ipack.v:424

    // ---- Output to ibuf (ipack.v:456-497) ----------------------------------
    wire ipack_ibuf_inst_vld_raw = ipack_retire_vld;
    wire ipack_ibuf_inst_vld    = ipack_retire_vld && !ipack_buf_stall;
    wire ipack_ibuf_inst_one    = ipack_one_16bit_vld;
    // TASK 4c: the !pred_ipack_chgflw_vld0 gate now applies to
    // `ipack_secnd_vld` ONLY (its 2-halfword push includes entry2, which is
    // the fall-through AFTER the taken slot-0 branch and must be dropped),
    // NOT to `ipack_one_32bit_vld`: there the push is the slot-0 branch
    // itself, and in the first OR-term (!h0 && h1_32bit && e2_vld) entry2 is
    // the branch's OWN completing high half -- gating it would silently
    // drop any 32-bit predicted-taken branch that arrives with no pending
    // carry (e.g. an uncompressed bne/blt/bge). one_32bit's second
    // OR-term (h0 && e1 && h2_32bit_vld) already dies under chgflw via
    // h2_32bit_vld's own !pred_ipack_chgflw_vld0 (line ~632), so no extra
    // gate is needed there.
    wire ipack_ibuf_inst_two    = (ipack_secnd_vld
                                 && !pred_ipack_chgflw_vld0 && !pred_ipack_delay_stall)
                                 || (ipack_one_32bit_vld && !pred_ipack_delay_stall) // BUG FIX #3
                                 || ipack_h0_delay_vld;                              // BUG FIX #5
    wire ipack_ibuf_inst_all    = ipack_all_vld;
    wire [47:0] ipack_ibuf_inst  = ipack_retire_inst;

    // M4 Task 6: the fault tag, qualified to the EXACT retire shape a fault
    // always takes (ipack_one_32bit_vld's first term, entry1+entry2 both
    // freshly created together, no h0 carry -- guaranteed by the entry0_vld
    // carry-drop above) AND to the push actually completing this cycle
    // (ipack_ibuf_inst_vld, not merely being classified as ready to).
    wire ipack_ibuf_fault_pgflt  = ipack_ibuf_inst_vld && ipack_one_32bit_vld && entry1_fault_pgflt_r;
    wire ipack_ibuf_fault_accflt = ipack_ibuf_inst_vld && ipack_one_32bit_vld && entry1_fault_accflt_r;

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
    reg  [1:0]  ibuf_tag [0:5];   // TASK 9: per-halfword bht_pred tag, see below
    // M4 Task 6: per-halfword fault tag {pgflt,accflt}, mirroring ibuf_tag[]'s
    // own push/pop indexing exactly (pushed at tail0 alongside the fault
    // marker's low half, read out at ibuf_head alongside ifu_idu_id_bht_pred).
    reg  [1:0]  ibuf_fault_tag [0:5];
    // M7 Task 2: per-halfword PC, mirroring ibuf_tag[]'s push/pop indexing
    // (pushed at each tail with the halfword's own PC, read out at
    // ibuf_head). Feeds the DTU execute comparators at the HEAD (the donor's
    // aq_ifu_pred.v:745-758 PRED-stage position) -- see the entry0/1/2_pc_r
    // block above and the ifu_dtu_exe_addr assignment at the file end.
    reg  [PC_WIDTH-1:0] ibuf_pc [0:5];
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

    wire       pop_entry_vld = ibuf_h0_vld && (!ibuf_h0_32 || ibuf_h1_vld);
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

    // TASK 9: which instruction slot owns the SECOND pushed halfword
    // (tail1) -- fully determinable from wires IFU.v already computes
    // (SECTION IPACK above), no ambiguity: tail1 is instr0's OWN high half
    // whenever the 2-push came from a lone 32-bit instr0
    // (`ipack_one_32bit_vld`), from the 3-push ALL case (entry0+entry1 =
    // instr0's 32 bits, entry2 = instr1, `ipack_ibuf_inst_all`), OR from
    // BUG FIX #5's h0-carry-plus-delay 2-push (`ipack_h0_delay_vld` --
    // entry0+entry1 again form instr0's 32 bits, entry2 is deferred, not
    // pushed at all this cycle); otherwise (the 2-push came from
    // `ipack_secnd_vld`, two independent 16-bit halves) tail1 is instr1's
    // own halfword. `ipack_secnd_vld`/`ipack_one_32bit_vld`/
    // `ipack_h0_delay_vld` are mutually exclusive by construction
    // (h2_16bit_vld/h2_32bit_vld can't both be the deciding term, and
    // `ipack_h0_delay_vld` requires `pred_ipack_delay_stall` which the
    // other two do not gate on), so this is a clean split, not a priority
    // guess.
    wire ibuf_tail1_is_instr0 = ipack_ibuf_inst_all || ipack_one_32bit_vld || ipack_h0_delay_vld;

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

    // M7 Task 2: the pushed halfwords' PCs. tail0 is ALWAYS instr0's low
    // half (same source priority as the DATA mux above, ipack_first_inst:
    // h0-carry shapes push entry0 -- the previous word's carried low half --
    // otherwise entry1, and the unaligned entry2-only shape pushes entry2).
    // tail1/tail2 sit 2B further along (the queue's own consecutive-
    // halfword invariant).
    wire [PC_WIDTH-1:0] ibuf_push_pc0 = entry0_vld ? entry0_pc_r
                                   : entry1_vld ? entry1_pc_r
                                                : entry2_pc_r;

    always @(posedge clk) begin
        if (ibuf_push_count >= 2'd1) begin
            ibuf_mem[ibuf_tail0] <= ipack_ibuf_inst[15:0];
            ibuf_tag[ibuf_tail0] <= pred_ibuf_br_taken0;      // tail0 is ALWAYS instr0's low half
            // M4 Task 6: the fault tag rides on tail0 -- ipack_ibuf_fault_*
            // is only ever asserted on the exact 2-push shape a fault takes
            // (ipack_one_32bit_vld), whose tail0 is entry1's own low half,
            // exactly where entry1_fault_pgflt_r/_accflt_r were latched.
            ibuf_fault_tag[ibuf_tail0] <= {ipack_ibuf_fault_pgflt, ipack_ibuf_fault_accflt};
            // M7 Task 2: the halfword's own PC, tail0..tail2 at +0/+2/+4.
            ibuf_pc[ibuf_tail0] <= ibuf_push_pc0;
        end
        if (ibuf_push_count >= 2'd2) begin
            ibuf_mem[ibuf_tail1] <= ipack_ibuf_inst[31:16];
            ibuf_tag[ibuf_tail1] <= ibuf_tail1_is_instr0 ? pred_ibuf_br_taken0 : pred_ibuf_br_taken1;
            // Never read (SECTION IBUF only ever reads ibuf_fault_tag[ibuf_head],
            // h0's own slot) -- written 0 anyway, matching this array's own
            // no-stale-garbage discipline (see ibuf_tag[]'s own precedent).
            ibuf_fault_tag[ibuf_tail1] <= 2'b00;
            ibuf_pc[ibuf_tail1] <= ibuf_push_pc0 + {{(PC_WIDTH-2){1'b0}}, 2'd2};
        end
        if (ibuf_push_count >= 2'd3) begin
            ibuf_mem[ibuf_tail2] <= ipack_ibuf_inst[47:32];
            ibuf_tag[ibuf_tail2] <= pred_ibuf_br_taken1;      // only reachable via ipack_ibuf_inst_all: tail2 is instr1
            ibuf_fault_tag[ibuf_tail2] <= 2'b00;              // never read, see tail1's own note
            ibuf_pc[ibuf_tail2] <= ibuf_push_pc0 + {{(PC_WIDTH-3){1'b0}}, 3'd4};
        end
    end

    // ---- Output to IDU (frozen ports) --------------------------------------
    // M7 Task 1: debug-instruction (itr) injection (donor aq_ifu_ibuf.v:949-
    // 953,1085,1098 -- the donor inserts the DM's word into the ibuf's own
    // create path; rv906's ibuf has no separate create0 port, so the
    // equivalent is a delivery mux here). While dbgon, ctrl_inst_fetch is
    // masked and the halt flush already emptied the ibuf, so pop_entry_vld
    // is 0 and the injected word is the ONLY thing delivered -- no double
    // issue is possible. The word bypasses the ibuf (no head advance): the
    // donor's create path also consumes no ICache fetch entry.
    wire dtu_dbg_inst_deliver = rtu_yy_xx_dbgon && dtu_ifu_debug_inst_vld;
    assign ifu_idu_id_inst_vld = pop_entry_vld || dtu_dbg_inst_deliver;   // ibuf.v:1354
    assign ifu_idu_id_inst     = dtu_dbg_inst_deliver ? dtu_ifu_debug_inst
                                                      : {ibuf_h1, ibuf_h0}; // ibuf.v:1355-1356

    // ifu_idu_id_bht_pred (ibuf.v:1357-1358) rides pred_ibuf_br_taken{0,1}
    // alongside each halfword in the real RTL. TASK 9: BPU.v's BHT is real
    // now, and `pred_ibuf_br_taken0/1` carry the genuine captured 2-bit
    // counter state -- propagated through IBUF via `ibuf_tag[]` (populated
    // at push time above, per-halfword, using the SAME instr0-vs-instr1
    // attribution as the data array) and read out here at pop time keyed
    // off `ibuf_head`, i.e. h0's own tag -- h0 is the first (low) halfword
    // of whichever instruction is about to be delivered to IDU, exactly
    // where the real RTL anchors this field.
    assign ifu_idu_id_bht_pred = ibuf_tag[ibuf_head];
    // M4 Task 6: read out exactly where ifu_idu_id_bht_pred is -- h0's own
    // slot, the low half of whichever instruction (real or fault marker)
    // is about to be delivered to IDU.
    assign ifu_idu_id_fault_pgflt  = ibuf_fault_tag[ibuf_head][1];
    assign ifu_idu_id_fault_accflt = ibuf_fault_tag[ibuf_head][0];
    // M7 Task 2: the execute-trigger halt_info is the DTU's LIVE verdict on
    // the head instruction (computed this cycle, see the feed below) -- the
    // same sideband-at-head pattern as ifu_idu_id_bht_pred, latched by IDU
    // on its own adv. 0 unless the DTU matched an execute trigger against
    // the head PC this cycle. (The donor's equivalent is aq_ifu_pred.v's
    // pred_ibuf_halt_info0/1: the PRED stage's verdict riding into IBUF;
    // rv906 has no separate pred register stage, so the head-of-ibuf read
    // IS the pred position -- one stage behind the icache fetch, which is
    // what lets a trigger armed while the tripwire is still in flight
    // catch it, as the stock rv64mi-p-breakpoint test requires.)
    assign ifu_idu_id_halt_info = dtu_ifu_halt_info_vld ? dtu_ifu_halt_info
                                                        : {TDT_HINFO_WIDTH{1'b0}};

    //=========================================================================
    // M7 Task 2: DTU execute-trigger feed. The instruction at the ibuf head
    // (the one delivered to IDU this cycle, if the pop fires): its PC is
    // ibuf_pc[ibuf_head] (latched at push time, SECTION IBUF above). The
    // DTU compares it combinationally against its mcontrol execute
    // comparators and returns dtu_ifu_halt_info[_vld] live (no latching --
    // the IDU captures the verdict on its own adv, exactly like
    // ifu_idu_id_bht_pred). pop_entry_vld is the right vld: it is set only
    // while a COMPLETE instruction (16-bit, or 32-bit with both halves
    // queued) sits at the head, so a 32-bit instruction is compared at its
    // own low-half PC, matching the donor's pred_inst0_bkpt_pc (the low
    // half's PC, aq_ifu_pred.v:747-748). The DM-injected debug instruction
    // (dtu_dbg_inst_deliver above) is deliberately NOT fed -- it never
    // enters the ibuf and must not arm an execute breakpoint (the old
    // fetch-PC feed had the same exclusion; it also fixes a latent stale-
    // readout: the old ibuf_halt_info[] array kept pre-flush garbage at the
    // head during a debug-mode delivery).
    //=========================================================================
    assign ifu_dtu_exe_addr     = ibuf_pc[ibuf_head];
    assign ifu_dtu_exe_addr_vld = pop_entry_vld;

endmodule
