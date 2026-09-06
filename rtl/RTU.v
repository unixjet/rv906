//=============================================================================
// RTU.v - retire unit: one-hot completion OR, EX1->EX2 retire latch,
//          exception/interrupt priority, flush FSM, rbus/wb arbitration
//                                                     (M2 Task 4: real body)
//=============================================================================
// C906 files covered:
//   gen_rtl/rtu/rtl/aq_rtu_top.v    (glue)
//   gen_rtl/rtu/rtl/aq_rtu_ctrl.v   (one-hot cmplt OR -> retire_vld register)
//   gen_rtl/rtu/rtl/aq_rtu_dp.v     (EX1->EX2 retire-packet pipe register)
//   gen_rtl/rtu/rtl/aq_rtu_rbus.v   (writeback-value arbiter + 3 fwd ports)
//   gen_rtl/rtu/rtl/aq_rtu_retire.v (exception/int priority, flush FSM,
//                                     changeflow-PC mux, every RTU-> output)
//   gen_rtl/rtu/rtl/aq_rtu_wb.v     (2-port GPR writeback packaging)
//   gen_rtl/rtu/rtl/aq_rtu_int.v    (15-source interrupt-cause priority)
// References: design doc S4.1/S4.2/S6 (unit graph, rtu_ex2_t is RTU-
// internal, the writeback-bus decision), RTU extraction note (all
// sections, esp. S2 retire structure, S3 rbus/wb, S4 exception priority,
// S6 flush/redirect signal inventory, S7 CSR-writeback timing).
//
// PORT-LIST AMENDMENTS (documented, not silent -- same discipline CSR.v's
// Task 2 "TASK 2 DISCOVERED GAP" and IU.v's Task 3 "PORT-LIST AMENDMENT"
// notes used):
//  1. `cp0_rtu_trap_pc` (input, [PC_WIDTH-1:0]) -- CSR.v's OWN header
//     ("TASK 2 DISCOVERED GAP, FIXED HERE") already anticipated exactly
//     this: it added `cp0_rtu_trap_pc` as a CSR.v OUTPUT specifically
//     because "RTU.v itself does not exist yet... it is wired for the
//     first time when RTU.v's real body is built." Task 1's frozen RTU.v
//     skeleton had no matching input at all -- added here, closing the
//     loop CSR.v opened. Without it, NO trap could ever compute a redirect
//     target (RTU note S5/S7: `cp0_rtu_trap_pc`, read combinationally by
//     retire's changeflow-PC mux independent of CP0's own EX1 pulse).
//  2. `lsu_rtu_ex2_dest_reg` (input, [GPR_IDX_WIDTH-1:0]) -- resolves the
//     skeleton's own flagged open item (header note: "whether LSU's 3
//     distinct donor-side writeback paths... collapse to the 2 paths this
//     skeleton pins... is the exact open item RTU note S2 flags... Task
//     4/6 resolve it together"). `rtu_idu_fwd2` (LSU-EX2 forward, RTU note
//     S3) needs a DESTINATION REGISTER to be meaningful at all -- a
//     forward with data but no target register cannot be consumed by
//     IDU's forward mux. The skeleton had `lsu_rtu_ex2_data`/`_data_vld`
//     but no matching preg; added here, mirroring the donor's own
//     `lsu_rtu_ex2_dest_reg` (RTU note S3, `aq_rtu_rbus.v:334`) exactly.
//     LSU.v itself does not exist yet (Task 6), so nothing drives this
//     port for real today -- same "landing pad, no live effect yet"
//     reasoning IU.v's Task 3 amendments used.
//
// TASK 4's RESOLUTION OF THE RTU NOTE S2 "LSU WRITEBACK PATH COUNT" OPEN
// ITEM (the skeleton's own header flagged this for "Task 4/6... not
// guessed here"): the donor has THREE distinct LSU->RTU write-adjacent
// buses (`lsu_rtu_ex1_wb_*` feeding the rbus EX1-group arbiter;
// `lsu_rtu_ex2_*` feeding fwd2; `lsu_rtu_wb_*` feeding wb1 directly,
// bypassing the arbiter). M2's frozen skeleton only carries TWO LSU write-
// adjacent signal GROUPS, and -- critically -- their NAMES match the
// donor's `lsu_rtu_wb_*` (wb1) and `lsu_rtu_ex2_*` (fwd2) exactly, NOT the
// donor's separately-named `lsu_rtu_ex1_wb_*` (rbus EX1-group leg). Task 4
// therefore resolves this by NOT folding LSU into the rbus EX1-group
// arbiter at all: `lsu_rtu_wb_data/_preg/_vld` drives `rtu_idu_wb1`
// DIRECTLY (matching the donor's own aq_rtu_wb.v:180-197 treatment of its
// identically-named port, bit for bit), and the rbus's "EX1 group" is
// therefore ALU+BJU+CP0 only for M2 (not ALU+BJU+CP0+LSU-ex1 as in the
// full donor) -- DIV and MUL-EX3 still arbitrate against this narrowed EX1
// group with the exact same priority (EX1 group > DIV > MUL-EX3, RTU note
// S3). This gives LSU an uncontested, dedicated write port instead of
// forcing it to race ALU/BJU/CP0/DIV/MUL for `rtu_idu_wb0` -- a reasonable
// reading of the ACTUAL port names available, but a genuine judgment call;
// flagged prominently in the Task 4 completion report, not guessed
// silently. `lsu_rtu_ex1_cmplt`/`_cmplt_dp` remain independent of this
// choice -- they feed ONLY the one-hot completion/retire-heartbeat bus,
// never the rbus writeback arbiter, exactly like every other producer.
//
// KNOWN, DOCUMENTED SCOPE GAPS (flagged, not silently modeled as fully
// accurate -- Task 4 completion report restates these):
//  * `retire_cpu_no_op`/`retire_pipeline_empty` (RTU note S6, the flush
//    FSM's drain gate) omit the donor's `iu_xx_no_op`/`lsu_rtu_no_op`
//    qualifiers: NEITHER port exists anywhere in M2's contracts (IU.v,
//    already committed, exposes no `iu_xx_no_op`; LSU.v does not exist
//    yet). This body approximates drain status from what RTU itself can
//    observe (`!ex2_retire_vld && !wb0_vld && !wb1_vld`) -- LSU's STB is
//    explicitly NOT gated on this either way (contract 4: "STB drains
//    unconditionally... no queued/commit-gated write mechanism"), so
//    omitting the donor's `!lsu_rtu_ex1_buffer_vld` term is a deliberate,
//    contract-sanctioned simplification, not a gap; the `iu_xx_no_op` omission
//    is the one genuine fidelity gap, likely benign (IU's own `_full`/
//    `_issue_stall` backpressure already blocks new dispatch for as long
//    as MULT/DIV are busy, and both terminate with a wb pulse RTU DOES
//    see) but not independently proven -- Task 6 (which needs its own
//    `lsu_rtu_no_op`-analogous signal for its STB reasoning, contract 4)
//    is the natural place to revisit this together.
//  * CP0-sourced exceptions carry NO `tval` value on any port anywhere:
//    CSR.v exports `expt_vld/_int/_vec` only, no `expt_tval`. Illegal-
//    instruction (vec=2) IS nominally in the mtval allowlist (real RISC-V
//    semantics populate mtval with the offending opcode there), but this
//    body's `ex1_tval` is sourced ONLY from LSU's `lsu_rtu_tval` (gated on
//    `lsu` being the cmplt source) -- a CP0-sourced illegal-instruction
//    trap reads mtval=0 rather than the offending opcode. Not required for
//    M2's rv64ui/um pass bar (mtval's value is not part of the
//    RVTEST_PASS/FAIL protocol) and not silently worked around -- flagged
//    here and in the Task 4 completion report for whichever milestone
//    wants spec-exact mtval-on-illegal-instruction behavior (would need a
//    new CSR.v output port). M4 Task 6 NARROWS this gap without closing it:
//    a CP0-dispatched FETCH FAULT (vec 12/1) needs no new CSR.v port at
//    all, because tval == epc for a fetch fault (both are "the faulting
//    fetch PC") and this module already has that PC live as
//    `iu_rtu_ex1_cur_pc` the SAME EX1 cycle `ex2_cur_pc` (below) latches
//    it for epc -- `ex1_tval`'s own mux picks it out by vec (12/1) rather
//    than by a new port. The broader CP0-illegal-tval gap above is
//    UNCHANGED and still flagged for later.
//  * `iu_rtu_ex1_div_cmplt`/`_cmplt_dp` (and, for a narrow multiply with no
//    split needed, `iu_rtu_ex1_mul_cmplt`/`_cmplt_dp`) are, per IU.v's own
//    already-committed body AND the real donor (confirmed directly,
//    `gen_rtl/iu/rtl/aq_iu_div.v:755-756`: `iu_rtu_ex1_div_cmplt =
//    idu_iu_ex1_div_sel`), a plain LEVEL echo of the dispatch-select input,
//    held high for as long as IDU (Task 5, not yet built) holds `_sel`
//    asserted -- i.e. for DIV's *entire* multi-cycle busy span, not a
//    single-cycle pulse (iu_tb.cpp's own `div_op()` helper documents and
//    exercises exactly this: "the real 'is the DATA ready yet' signal is
//    iu_rtu_div_wb_vld, drained next"). This body ORs that signal into the
//    one-hot completion bus EXACTLY as specified (task 4.1's "OR'd into
//    dp_ex1_cmplt" and the donor's own `aq_rtu_ctrl.v`/`aq_rtu_dp.v`, which
//    apply no extra qualification either) -- faithful, not a bug Task 4
//    introduced or should "fix". Consequence, flagged for Tasks 5/8/9: once
//    IDU/the full pipe exist, `ex2_retire_vld` will be held high across a
//    multi-cycle DIV's *entire* residency, not just its final cycle --
//    anything that wants "one pulse per architecturally-retiring
//    instruction" (a future minstret auto-increment arm, the per-retire
//    oracle trace of design doc S7.3/contract 13) MUST edge-qualify this
//    signal or gate on the wb0/wb1 valid pulse instead of raw
//    `ex2_retire_vld`, not assume it is already a clean per-instruction
//    pulse. Design doc S7.3's "retire and architectural writeback are the
//    same cycle for every source" holds for the *final* cycle of any
//    producer's completion, but for DIV/narrow-MUL specifically,
//    `ex2_retire_vld` can precede the matching `rtu_idu_wb0_vld` pulse by
//    several cycles while the same instruction is still resident.
//  * `ifu_rtu_warm_up` (the donor's reset/pipeline-fill signal, gating
//    every donor register's "or warm_up" term) has no corresponding M2
//    port anywhere -- omitted outright, matching M1's own reset-vector-only
//    warm-up model (no separate warm-up pulse exists in this design).
//  * vstart/fs_dirty/vs_dirty/`vpu_rtu_*`/HPCP/DTU/MMU (RTU note S2/S3/S6):
//    M4/M5-only concepts with ZERO corresponding ports anywhere in M2's
//    contracts -- not modeled at all, not even as a dead-but-present leg,
//    since there is no port to attach one to (unlike the pending-
//    breakpoint/interrupt/debug-breakpoint legs, which DO have a
//    structural home in the exception-priority chain below).
//
// SEAM NOTES (rv906 decomposition, carried over from the skeleton):
//  * RTU is NOT a reorder buffer (contract/RTU note S0/S9) -- one un-
//    buffered EX1->EX2 pipeline register, retiring at most 1/cycle, 0/cycle
//    on any stall, no queue.
//  * The one-hot completion bus's 7th source, `vec_cmplt_dp`, has no M2
//    port -- tied 0 internally (no VPU in M2).
//  * `cp0_rtu_int_vld`-shaped interrupt input: DELIBERATELY not added.
//    M2's minimal mie/mip model (contract 7) computes no masked interrupt
//    vector anywhere (CSR.v exports no such port either), so the
//    interrupt-cause priority encoder below (a faithful clone of
//    `aq_rtu_int.v`'s casez table) is fed an internal, permanently-0
//    vector -- structurally present, provably never fires, exactly per
//    task 4.1's "wired but structurally never fire" framing. A future
//    milestone wiring real interrupts adds the port and threads it in.
//=============================================================================

import rvproc_pkg::*;

module RTU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IU -> RTU : the four writeback buses (contract 1) -- ALU/BJU always
    // one-hot-exclusive with each other and with MULT/DIV's EX1 early-
    // accept; MULT's EX3 data/writeback and DIV's variable-EX-stage data/
    // writeback arrive separately, gated on this module's own wb_grant
    // outputs below.
    //=========================================================================
    input  wire                     iu_rtu_ex1_alu_cmplt,
    input  wire                     iu_rtu_ex1_alu_cmplt_dp,
    input  wire [63:0]              iu_rtu_ex1_alu_data,
    input  wire                     iu_rtu_ex1_alu_inst_len,
    input  wire                     iu_rtu_ex1_alu_inst_split,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_alu_preg,
    input  wire                     iu_rtu_ex1_alu_wb_dp,
    input  wire                     iu_rtu_ex1_alu_wb_vld,

    input  wire                     iu_rtu_ex1_bju_cmplt,
    input  wire                     iu_rtu_ex1_bju_cmplt_dp,
    input  wire                     iu_rtu_ex1_bju_cmplt_for_pcgen,
    input  wire [63:0]              iu_rtu_ex1_bju_data,
    input  wire                     iu_rtu_ex1_bju_inst_len,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex1_bju_preg,
    input  wire                     iu_rtu_ex1_bju_wb_dp,
    input  wire                     iu_rtu_ex1_bju_wb_vld,
    input  wire                     iu_rtu_ex1_branch_inst,
    input  wire [PC_WIDTH-1:0]      iu_rtu_ex1_cur_pc,
    input  wire [PC_WIDTH-1:0]      iu_rtu_ex1_next_pc,
    input  wire                     iu_rtu_ex2_bju_ras_mispred,
    input  wire                     iu_rtu_depd_lsu_chgflow_vld,
    input  wire [PC_WIDTH-1:0]      iu_rtu_depd_lsu_chgflow_next_pc,

    input  wire                     iu_rtu_ex1_mul_cmplt,
    input  wire                     iu_rtu_ex1_mul_cmplt_dp,
    input  wire [63:0]              iu_rtu_ex3_mul_data,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_ex3_mul_preg,
    input  wire                     iu_rtu_ex3_mul_wb_vld,
    // Task 7.3: completing-MULT length for the pcgen inst_len mux (tied 1
    // upstream -- no 16-bit MULT in M2).
    input  wire                     iu_rtu_ex1_mul_inst_len,

    input  wire                     iu_rtu_ex1_div_cmplt,
    input  wire                     iu_rtu_ex1_div_cmplt_dp,
    input  wire [63:0]              iu_rtu_div_data,
    input  wire [GPR_IDX_WIDTH-1:0] iu_rtu_div_preg,
    input  wire                     iu_rtu_div_wb_dp,
    input  wire                     iu_rtu_div_wb_vld,
    // Task 7.3: completing-DIV length for the pcgen inst_len mux (tied 1
    // upstream -- no 16-bit DIV in M2).
    input  wire                     iu_rtu_ex1_div_inst_len,

    //=========================================================================
    // RTU -> IU : the two writeback-race grants (IU note S0/S5/S6).
    //=========================================================================
    output wire                     rtu_iu_mul_wb_grant,
    output wire                     rtu_iu_div_wb_grant,

    //=========================================================================
    // RTU -> IU : Task 7.3 PC-generator retire feedback (closes IU.v's
    // documented "no rtu_iu_ex1_cmplt/_inst_split" scope gap). Donor
    // aq_iu_bju.v:148-151 / aq_rtu_top.v:403-406:
    //   rtu_iu_ex1_cmplt      = ctrl_ex1_cmplt_for_pcgen (aq_rtu_ctrl.v:240)
    //   rtu_iu_ex1_inst_len   = dp_ex1_inst_len   (aq_rtu_dp.v:537)
    //   rtu_iu_ex1_inst_split = dp_ex1_inst_split (aq_rtu_dp.v:537)
    //=========================================================================
    output wire                     rtu_iu_ex1_cmplt,
    output wire                     rtu_iu_ex1_inst_len,
    output wire                     rtu_iu_ex1_inst_split,

    //=========================================================================
    // LSU -> RTU : lsu_rtu_t (design doc S4.2) -- see the header's LSU-
    // writeback-path resolution note. `lsu_rtu_ex2_dest_reg` is a Task 4
    // port amendment (see header).
    //=========================================================================
    input  wire                     lsu_rtu_ex1_cmplt,
    input  wire                     lsu_rtu_ex1_cmplt_dp,
    // Task 9.7 (class-B clone fix): the EARLY "for pcgen" LSU completion
    // (donor aq_lsu_ag.v:1675 ag_pipe_cmplt_normal). Drives ONLY the pcgen
    // completion OR -- NOT retire, which stays on the late `lsu_rtu_ex1_
    // cmplt_dp`. See the rtu_iu_ex1_cmplt note below for the full rationale.
    input  wire                     lsu_rtu_ex1_cmplt_for_pcgen,
    // Task 7.3: completing-LSU length for the pcgen inst_len mux
    // (donor aq_rtu_dp.v:367 lsu arm).
    input  wire                     lsu_rtu_ex1_inst_len,
    input  wire [63:0]              lsu_rtu_wb_data,
    input  wire [GPR_IDX_WIDTH-1:0] lsu_rtu_wb_preg,
    input  wire                     lsu_rtu_wb_vld,
    // M5 Task 4c (D8): FRF/GPR destination selector riding alongside the
    // lsu_rtu_wb_* payload above -- 1 = FLW/FLD completion (route to wbf1
    // below, NOT wb1/GPR).
    input  wire                     lsu_rtu_wb_dst_frf,
    input  wire [63:0]              lsu_rtu_ex2_data,
    input  wire                     lsu_rtu_ex2_data_vld,
    input  wire [GPR_IDX_WIDTH-1:0] lsu_rtu_ex2_dest_reg,
    input  wire                     lsu_rtu_expt_vld,
    input  wire [4:0]               lsu_rtu_expt_vec,
    input  wire [63:0]              lsu_rtu_tval,
    input  wire                     lsu_rtu_async_expt_vld,
    input  wire                     lsu_rtu_async_ld_inst,

    //=========================================================================
    // RTU -> LSU : "point of no return" acks LSU needs to release/drop any
    // buffered store state (RTU note S6, directly relevant to Task 6).
    //=========================================================================
    output wire                     rtu_lsu_expt_ack,
    output wire                     rtu_lsu_expt_exit,

    //=========================================================================
    // CSR -> RTU : cp0_rtu_t (design doc S4.2). `cp0_rtu_trap_pc` is a
    // Task 4 port amendment (see header) -- CSR.v (Task 2) already exports
    // it, anticipating exactly this.
    //=========================================================================
    input  wire                     cp0_rtu_ex1_cmplt_dp,
    // Task 7.3: completing-CP0 length for the pcgen inst_len mux
    // (donor aq_rtu_dp.v:371 cp0 arm).
    input  wire                     cp0_rtu_ex1_inst_len,
    input  wire [63:0]              cp0_rtu_ex1_wb_data,
    input  wire [GPR_IDX_WIDTH-1:0] cp0_rtu_ex1_wb_preg,
    input  wire                     cp0_rtu_ex1_wb_vld,
    input  wire                     cp0_rtu_ex1_expt_vld,
    input  wire                     cp0_rtu_ex1_expt_int,
    input  wire [4:0]               cp0_rtu_ex1_expt_vec,
    input  wire                     cp0_rtu_ex1_chgflw,
    input  wire [PC_WIDTH-1:0]      cp0_rtu_ex1_chgflw_pc,
    input  wire [PC_WIDTH-1:0]      cp0_rtu_trap_pc,

    //=========================================================================
    // FPU -> RTU : M5 Task 4b FALU EX1 writeback (design doc S7.3 style --
    // fvld/fdata/preg feed the new wbf0 FRF-writeback register below; xvld/
    // xdata/preg join the EX1-group rbus arbiter as a third leg alongside
    // ex1_fwd_vld/cp0_rtu_ex1_wb_vld (GPR-destined compare/classify results).
    //=========================================================================
    input  wire [63:0]              fpu_rtu_ex1_falu_fdata,
    input  wire [63:0]              fpu_rtu_ex1_falu_xdata,
    // M5 Task 8: the retiring FP op's accrued flags (donor-named family
    // aq_rtu_wb.v:268-269), sampled into the EX2 retire packet below.
    input  wire [4:0]               fpu_rtu_ex1_falu_fflags,
    input  wire                     fpu_rtu_ex1_falu_fvld,
    input  wire                     fpu_rtu_ex1_falu_xvld,
    input  wire [GPR_IDX_WIDTH-1:0] fpu_rtu_ex1_falu_preg,

    //=========================================================================
    // RTU -> CSR : trap-entry capture (RTU note S7).
    //=========================================================================
    output wire                     rtu_yy_xx_expt_vld,
    output wire                     rtu_yy_xx_expt_int,
    output wire [4:0]               rtu_yy_xx_expt_vec,
    output wire                     rtu_yy_xx_flush_fe,
    output wire                     rtu_yy_xx_flush,
    output wire                     rtu_yy_xx_dbgon,
    output wire [PC_WIDTH-1:0]      rtu_cp0_epc,
    output wire [63:0]              rtu_cp0_tval,
    // M4 Task 1: one pulse per retiring instruction for CSR.v's minstret
    // auto-increment (CSR.v's header "KNOWN, DELIBERATE GAP" discharged).
    // ex2_retire_vld is the retire heartbeat; multi-cycle producers hold it
    // high across residency (header note), acceptable for M4 (no test checks
    // exact per-DIV counts).
    output wire                     rtu_cp0_inst_retire,
    // M5 Task 8 (D7): the retiring FP op's accrued flags, and the
    // FP-instruction-retire pulse that dirties mstatus.FS in CSR.v. Both
    // EX2-registered (SECTION EX1->EX2 RETIRE REGISTER below) and donor-
    // named: aq_rtu_wb.v:268-269 (`rtu_cp0_fflags[_updt]` off the VPU's
    // writeback) and aq_rtu_rbus.v:514-516 (`rtu_cp0_fs_dirty_updt` off
    // `vpu_rtu_ex1_fp_dirty && vpu_rtu_ex1_cmplt`). rv906 has no VPU; the
    // FPU cluster's fvld/xvld completion qualifier stands in for both.
    output wire [4:0]               rtu_cp0_fflags,
    output wire                     rtu_cp0_fs_dirty_updt,

    //=========================================================================
    // RTU -> IFU : redirect target + FE-kill pulse. Reused UNCHANGED from
    // M1 -- names/widths copied verbatim from rtl/RVProc.v's current wire
    // declarations (lines 287-289), already the signal IFU.v/BPU.v consume.
    //=========================================================================
    output wire                     rtu_ifu_chgflw_vld,
    output wire [PC_WIDTH-1:0]      rtu_ifu_chgflw_pc,
    output wire                     rtu_ifu_flush_fe,

    //=========================================================================
    // RTU -> IDU : forward/commit ports (contract 8's exception-side) +
    // the flush/drain signal group (RTU note S6).
    //=========================================================================
    output wire [63:0]              rtu_idu_fwd0_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd0_reg,
    output wire                     rtu_idu_fwd0_vld,
    output wire [63:0]              rtu_idu_fwd1_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd1_reg,
    output wire                     rtu_idu_fwd1_vld,
    output wire [63:0]              rtu_idu_fwd2_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_fwd2_reg,
    output wire                     rtu_idu_fwd2_vld,
    output wire [63:0]              rtu_idu_wb0_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb0_reg,
    output wire                     rtu_idu_wb0_vld,
    output wire [63:0]              rtu_idu_wb1_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_wb1_reg,
    output wire                     rtu_idu_wb1_vld,
    // M5 Task 4b: FRF writeback, registered one cycle behind EX1 like wb0
    // (design doc S7.3). FALU is the only wbf0 producer today.
    output wire [63:0]              rtu_idu_wbf0_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_wbf0_reg,
    output wire                     rtu_idu_wbf0_vld,
    // M5 Task 4c (D8): FRF's LSU-writeback port -- a PURE combinational
    // passthrough of lsu_rtu_wb_* (gated to FRF-destined completions), same
    // shape/timing as wb1's own GPR passthrough immediately above (LSU's own
    // internal pipeline latency already produces the value at the correct
    // write-back cycle -- no extra register stage needed here).
    output wire [63:0]              rtu_idu_wbf1_data,
    output wire [GPR_IDX_WIDTH-1:0] rtu_idu_wbf1_reg,
    output wire                     rtu_idu_wbf1_vld,
    output wire                     rtu_idu_flush_fe,
    output wire                     rtu_idu_flush_stall,
    output wire                     rtu_idu_flush_wbt,
    output wire                     rtu_idu_commit,
    output wire                     rtu_idu_commit_for_bju,
    output wire                     rtu_idu_pipeline_empty
);

    //=========================================================================
    // FEEDBACK SECTION (umbrella S6.2 rule 4) -- RTU is the terminal stage
    // of the M2 pipe, so essentially every OUTPUT below is itself a
    // backward-flowing signal (redirect/flush/commit info flowing back to
    // IFU/IDU/CSR/LSU/IU against the nominal ID->EX1->EX2 flow). The only
    // genuinely-forward INPUTS are the five producers' completion/writeback
    // buses (IU x4, LSU, CP0) and `cp0_rtu_trap_pc` (a feedback INPUT from
    // CSR, consumed only by the changeflow-PC mux below).
    //=========================================================================

    //=========================================================================
    // SECTION ONE-HOT COMPLETION BUS (task 4.1, RTU note S2 -- aq_rtu_dp.v
    // :318-326) -- `{alu,mul,bju,div,lsu,cp0,vec,fpu}_cmplt_dp`, vec tied 0
    // (no VPU: D3, no vector extension), OR'd into the single un-skidded
    // EX1->EX2 retire enable.
    //=========================================================================
    wire ex1_alu_cmplt_dp = iu_rtu_ex1_alu_cmplt_dp;
    wire ex1_mul_cmplt_dp = iu_rtu_ex1_mul_cmplt_dp;
    wire ex1_bju_cmplt_dp = iu_rtu_ex1_bju_cmplt_dp;
    wire ex1_div_cmplt_dp = iu_rtu_ex1_div_cmplt_dp;
    wire ex1_lsu_cmplt_dp = lsu_rtu_ex1_cmplt_dp;
    wire ex1_cp0_cmplt_dp = cp0_rtu_ex1_cmplt_dp;
    wire ex1_vec_cmplt_dp = 1'b0;   // no VPU (D3: no vector extension)
    // M5 Task 8: the FPU's completion leg. fvld (FRF-destined result) OR
    // xvld (GPR-destined result) is exactly "an FP op completed in EX1 this
    // cycle": FPU.v SECTION 13 asserts both only on the cycle its result
    // mux actually outputs the answer, including FDSU's multi-cycle case
    // (fdsu_cmplt_now, the real completion cycle, not dispatch). No donor
    // precedent for a separate scalar-FP leg -- the donor rides FP on its
    // vec (VPU) leg (aq_rtu_dp.v:307,325); rv906 has no VPU and a separate
    // FPU cluster (D1), so this is the new 8th leg.
    wire ex1_fpu_cmplt_dp = fpu_rtu_ex1_falu_fvld || fpu_rtu_ex1_falu_xvld;

    wire [7:0] dp_cmplt_source = {ex1_alu_cmplt_dp, ex1_mul_cmplt_dp, ex1_bju_cmplt_dp,
                                   ex1_div_cmplt_dp, ex1_lsu_cmplt_dp, ex1_cp0_cmplt_dp,
                                   ex1_vec_cmplt_dp, ex1_fpu_cmplt_dp};
    wire dp_ex1_cmplt_dp = |dp_cmplt_source;

    //=========================================================================
    // SECTION TASK 7.3 -- PC-generator retire feedback to IU (closes IU.v's
    // documented "no rtu_iu_ex1_cmplt/_inst_split" scope gap). The donor
    // (aq_rtu_dp.v:344-379,537; aq_rtu_ctrl.v:148-156,240) muxes the
    // completing EU's inst_len/inst_split off this same one-hot vector and
    // ORs the completion terms for the pcgen trigger. Bit order matches the
    // concatenation above: {alu,mul,bju,div,lsu,cp0,vec,fpu} = bits {7..0}
    // (M5 Task 8 widened the vector to 8 bits; the existing seven legs keep
    // their old positions, FPU takes the new LSB).
    //=========================================================================
    localparam [7:0] CBUS_ALU_SEL = 8'b10000000;
    localparam [7:0] CBUS_MUL_SEL = 8'b01000000;
    localparam [7:0] CBUS_BJU_SEL = 8'b00100000;
    localparam [7:0] CBUS_DIV_SEL = 8'b00010000;
    localparam [7:0] CBUS_LSU_SEL = 8'b00001000;
    localparam [7:0] CBUS_CP0_SEL = 8'b00000100;
    localparam [7:0] CBUS_VEC_SEL = 8'b00000010;
    localparam [7:0] CBUS_FPU_SEL = 8'b00000001;

    // The donor's pcgen trigger is NOT the retire completion: aq_rtu_ctrl.v
    // :151-157 builds a SEPARATE `ctrl_ex1_cmplt_for_pcgen` whose LSU arm is
    // `lsu_rtu_ex1_cmplt_for_pcgen` (aq_lsu_ag.v:1675 ag_pipe_cmplt_normal)
    // instead of the real `lsu_rtu_ex1_cmplt` -- every OTHER arm (alu/mul/
    // bju/div/cp0/vec) is identical to the retire OR. The distinction is
    // about WHEN the LSU arm fires, not which ops advance the PC: a store
    // leaves the EX1 register one cycle after entering it, but its memory op
    // (this LSU's IDLE->DCS->FRZ/REPLY pipe) runs several cycles longer.
    // Driving the pcgen off the late `lsu_rtu_ex1_cmplt_dp` left it one
    // instruction (4B) behind the EX1-resident instruction, so the next
    // auipc and the next taken-branch target both computed pc+imm from the
    // PREVIOUS instruction's pc -- the M2 tohost loop (auipc; sw; auipc;
    // sw; j) self-reinforced a 4-behind steady state and tohost never read
    // the pass value. Task 9.7 fix: OR the early for-pcgen arm here.
    wire ex1_lsu_cmplt_for_pcgen = lsu_rtu_ex1_cmplt_for_pcgen;
    wire ex1_bju_cmplt_for_pcgen = iu_rtu_ex1_bju_cmplt_for_pcgen;
    // M5 Task 8: FPU has no early/late split -- fvld/xvld fire on the same
    // EX1 cycle as the result (single-cycle units; FDSU's iterative span is
    // resolved by its own fdsu_cmplt_now), so its for-pcgen arm is its `_dp`
    // flavor itself, exactly like the ALU's.
    wire dp_ex1_cmplt_for_pcgen  = ex1_alu_cmplt_dp  || ex1_mul_cmplt_dp
                                  || ex1_bju_cmplt_for_pcgen || ex1_div_cmplt_dp
                                  || ex1_lsu_cmplt_for_pcgen || ex1_cp0_cmplt_dp
                                  || ex1_vec_cmplt_dp || ex1_fpu_cmplt_dp;
    assign rtu_iu_ex1_cmplt = dp_ex1_cmplt_for_pcgen;

    // rtu_iu_ex1_inst_len = the COMPLETING instruction's length, muxed by
    // the completing EU (donor aq_rtu_dp.v:350-379). ALU/BJU/LSU/CP0 carry
    // real RVC-aware lengths; MULT/DIV/VEC are 32-bit-only in M2 (no 16-bit
    // form in the RVC decoder) and FPU is always 32-bit (the RVC extension
    // contains no F/D encodings) so they all resolve to the 1'b1 (32-bit)
    // value -- FPU via its own CBUS_FPU_SEL arm, the rest via the default.
    //
    // The select vector MUST be the same early/late flavor as the pcgen
    // TRIGGER (dp_ex1_cmplt_for_pcgen) above, not the retire-late
    // dp_cmplt_source: an LSU op's for-pcgen arm fires while its `_dp` is
    // still several cycles away (line 391-392), and a PARKED bju fires
    // for-pcgen at entry-creation but `_dp`/bju_resolves_now only at pop
    // (IU.v iu_rtu_ex1_bju_cmplt_for_pcgen note). On those cycles every
    // `_dp` bit is 0, so selecting on dp_cmplt_source fell to the
    // `default` 32-bit assumption even for a 16-bit RVC completer,
    // advancing bju_pcgen_pc by 4 instead of 2 -- a permanent pcgen/retire
    // desync (the same self-drift class as IU.v's Task 7.3 note) that
    // eventually redirects a later branch to a wrong, sometimes
    // self-referential target (found chasing the rv64ui-v-simple hang:
    // HBDBG showed the LSU fully idle while IFU relooped forever on one
    // PC, hit=1/pf=0 every fetch -- i.e. the front end, not the LSU, had
    // desynced).
    wire [7:0] pcgen_len_source = {ex1_alu_cmplt_dp, ex1_mul_cmplt_dp, ex1_bju_cmplt_for_pcgen,
                                    ex1_div_cmplt_dp, ex1_lsu_cmplt_for_pcgen, ex1_cp0_cmplt_dp,
                                    ex1_vec_cmplt_dp, ex1_fpu_cmplt_dp};
    reg rtu_iu_ex1_inst_len_r;
    always @* begin
        case (pcgen_len_source)
            CBUS_ALU_SEL: rtu_iu_ex1_inst_len_r = iu_rtu_ex1_alu_inst_len;
            CBUS_BJU_SEL: rtu_iu_ex1_inst_len_r = iu_rtu_ex1_bju_inst_len;
            CBUS_LSU_SEL: rtu_iu_ex1_inst_len_r = lsu_rtu_ex1_inst_len;
            CBUS_CP0_SEL: rtu_iu_ex1_inst_len_r = cp0_rtu_ex1_inst_len;
            CBUS_FPU_SEL: rtu_iu_ex1_inst_len_r = 1'b1;   // FPU: 32-bit (M5 Task 8)
            default:      rtu_iu_ex1_inst_len_r = 1'b1;   // MULT/DIV/VEC: 32-bit
        endcase
    end
    assign rtu_iu_ex1_inst_len = rtu_iu_ex1_inst_len_r;

    // rtu_iu_ex1_inst_split = the completing instruction's split flag (donor
    // aq_rtu_dp.v:537). No M2 producer sets inst_split (IU ties
    // alu_inst_split=0; the other EUs have no split class), so this is
    // structurally 0 and IU's pcgen advance gate (`!rtu_iu_ex1_inst_split`)
    // is vacuous -- kept for structural fidelity to the donor.
    assign rtu_iu_ex1_inst_split = 1'b0;

    // The donor's own PLAIN (non-`_dp`) completion family
    // (`iu_rtu_ex1_alu_cmplt`/`_mul_cmplt`/`_bju_cmplt`/`_div_cmplt`,
    // `lsu_rtu_ex1_cmplt` -- aq_rtu_ctrl.v:131-148's `ctrl_ex1_*_cmplt`)
    // remains on this module's frozen port list but is intentionally left
    // UNUSED by this body: every M2 producer already ties its plain
    // `_cmplt` identically to its own `_cmplt_dp` (IU.v's ALU/BJU/MUL/DIV:
    // the literal same expression assigned twice; CSR.v exposes only ONE
    // cp0 signal at all), so `dp_ex1_cmplt_dp` above already carries the
    // same information the donor's separate `ctrl_ex1_cmplt` bus would.
    // Flagged here rather than silently ignored -- a future milestone
    // whose producers legitimately diverge the two families would need to
    // build the donor's second bus for real.

    //=========================================================================
    // SECTION 4.2a -- one-hot assertion (design doc S8 risk this unit owns;
    // RTU note S2's own `// TODO add assertion here: cmplt_dp is onehot.`).
    // Classic bit trick: `v & (v-1)` is nonzero iff v has 2+ bits set (v==0
    // gives 0, v with exactly one bit gives 0, anything else is nonzero) --
    // exposed as a `verilator public` debug flag so the unit bench can poll
    // it directly, per the task's explicit instruction.
    //=========================================================================
    wire dbg_onehot_violation /* verilator public */;
    assign dbg_onehot_violation = |(dp_cmplt_source & (dp_cmplt_source - 8'd1));

    //=========================================================================
    // SECTION RBUS ARBITER (task 4.1, RTU note S3 -- aq_rtu_rbus.v) -- EX1
    // group (ALU/BJU/CP0 in M2, see header's LSU-path resolution note) >
    // DIV > MUL-EX3, by if/else-if priority (aq_rtu_rbus.v:441-461); safe
    // only because the EX1 group is one-hot by IDU's single-issue dispatch
    // and DIV/MUL report independently on their own multi-cycle completion.
    //=========================================================================

    // ---- EX1-group forward merge: ALU/BJU only (donor's own fwd0 mux,
    // aq_rtu_rbus.v:277-321, never includes CP0 either -- carried forward
    // unchanged, same "not a bug to fix" discipline as the mtval quirk
    // below: CP0's old-CSR-value write is NOT forwarded to a same-cycle-
    // dependent instruction in the real donor). LSU is absent from this
    // merge for M2 (header's LSU-path resolution: LSU writes via wb1
    // directly, never through this arbiter).
    wire [1:0] ex1_fwd_src_vld = {iu_rtu_ex1_alu_wb_dp, iu_rtu_ex1_bju_wb_dp};
    reg  [GPR_IDX_WIDTH-1:0] ex1_fwd_preg;
    reg  [63:0]              ex1_fwd_data;
    always @* begin
        case (ex1_fwd_src_vld)
            2'b01: begin ex1_fwd_preg = iu_rtu_ex1_bju_preg; ex1_fwd_data = iu_rtu_ex1_bju_data; end
            2'b10: begin ex1_fwd_preg = iu_rtu_ex1_alu_preg; ex1_fwd_data = iu_rtu_ex1_alu_data; end
            default: begin ex1_fwd_preg = {GPR_IDX_WIDTH{1'b0}}; ex1_fwd_data = 64'd0; end
        endcase
    end
    wire ex1_fwd_vld = |ex1_fwd_src_vld;

    // ---- EX1-group overall winner: ALU/BJU-merged-fwd vs CP0 vs FALU's
    // GPR-destined xvld (M5 Task 4b: fcmp/fclass results, compare/classify
    // FUNC arms per FPU.v SECTION 12) -- mutually exclusive by single-issue
    // dispatch, same discipline as the donor's 2-leg merge
    // (aq_rtu_rbus.v:372-407); the FALU leg has no donor precedent (FP is a
    // new EU not present in the donor's rbus scheme as adapted here) and is
    // provably dead pre-Task-9 (misa.F/D=0 keeps d32_illegal=1 for every
    // OP-FP arm, so ex1_eu_r[EU_FP_SEL] can never be true).
    wire [2:0] ex1_wb_src_vld = {fpu_rtu_ex1_falu_xvld, ex1_fwd_vld, cp0_rtu_ex1_wb_vld};
    reg  [GPR_IDX_WIDTH-1:0] ex1_wb_preg;
    reg  [63:0]              ex1_wb_data;
    always @* begin
        case (ex1_wb_src_vld)
            3'b001: begin ex1_wb_preg = cp0_rtu_ex1_wb_preg;      ex1_wb_data = cp0_rtu_ex1_wb_data;      end
            3'b010: begin ex1_wb_preg = ex1_fwd_preg;             ex1_wb_data = ex1_fwd_data;             end
            3'b100: begin ex1_wb_preg = fpu_rtu_ex1_falu_preg;    ex1_wb_data = fpu_rtu_ex1_falu_xdata;   end
            default: begin ex1_wb_preg = {GPR_IDX_WIDTH{1'b0}}; ex1_wb_data = 64'd0; end
        endcase
    end
    wire ex1_wb_dp  = |ex1_wb_src_vld;
    wire ex1_wb_vld = iu_rtu_ex1_alu_wb_vld || iu_rtu_ex1_bju_wb_vld || cp0_rtu_ex1_wb_vld || fpu_rtu_ex1_falu_xvld;

    // ---- Top arbiter + writeback-race grants (aq_rtu_rbus.v:467-468).
    wire div_wb_grant = !ex1_wb_dp;
    wire mul_wb_grant = !ex1_wb_dp && !iu_rtu_div_wb_dp;
    assign rtu_iu_div_wb_grant = div_wb_grant;
    assign rtu_iu_mul_wb_grant = mul_wb_grant;

    reg        rbus_wb_vld;
    reg [GPR_IDX_WIDTH-1:0] rbus_wb_preg;
    reg [63:0] rbus_wb_data;
    always @* begin
        if (ex1_wb_dp) begin
            rbus_wb_vld  = ex1_wb_vld;
            rbus_wb_preg = ex1_wb_preg;
            rbus_wb_data = ex1_wb_data;
        end else if (iu_rtu_div_wb_dp) begin
            rbus_wb_vld  = iu_rtu_div_wb_vld;
            rbus_wb_preg = iu_rtu_div_preg;
            rbus_wb_data = iu_rtu_div_data;
        end else if (iu_rtu_ex3_mul_wb_vld) begin
            // MUL has no separate wb_dp port -- wb_vld doubles as dp,
            // matching the donor exactly (rbus_mul_wb_dp = iu_rtu_ex3_mul_wb_vld).
            rbus_wb_vld  = iu_rtu_ex3_mul_wb_vld;
            rbus_wb_preg = iu_rtu_ex3_mul_preg;
            rbus_wb_data = iu_rtu_ex3_mul_data;
        end else begin
            rbus_wb_vld  = 1'b0;
            rbus_wb_preg = {GPR_IDX_WIDTH{1'b0}};
            rbus_wb_data = 64'd0;
        end
    end
    //=========================================================================
    // SECTION FORWARD PORTS (task 4.1 -- fwd0=EX1 group, fwd1=MUL-EX3,
    // fwd2=LSU-EX2). All three are purely combinational bypass ports, never
    // registered (aq_rtu_rbus.v:479-489) -- they exist so a back-to-back
    // dependent instruction can see the value one cycle before the
    // architectural write (wb0/wb1 below) actually lands.
    //=========================================================================
    assign rtu_idu_fwd0_vld  = ex1_fwd_vld;
    assign rtu_idu_fwd0_reg  = ex1_fwd_preg;
    assign rtu_idu_fwd0_data = ex1_fwd_data;

    assign rtu_idu_fwd1_vld  = iu_rtu_ex3_mul_wb_vld;
    assign rtu_idu_fwd1_reg  = iu_rtu_ex3_mul_preg;
    assign rtu_idu_fwd1_data = iu_rtu_ex3_mul_data;

    assign rtu_idu_fwd2_vld  = lsu_rtu_ex2_data_vld;
    assign rtu_idu_fwd2_reg  = lsu_rtu_ex2_dest_reg;
    assign rtu_idu_fwd2_data = lsu_rtu_ex2_data;

    //=========================================================================
    // SECTION 4.2b -- fwd0/fwd1/fwd2 destination-register collision
    // assertion (design doc S8 / IDU note S6's flagged reliance on this
    // exact invariant). x0 is a legitimate exception: two producers both
    // targeting x0 is harmless (neither write is architecturally visible)
    // and must NOT trip this flag.
    //=========================================================================
    wire fwd_reg_is_x0_0 = (ex1_fwd_preg == {GPR_IDX_WIDTH{1'b0}});
    wire fwd_reg_is_x0_1 = (iu_rtu_ex3_mul_preg == {GPR_IDX_WIDTH{1'b0}});

    wire fwd01_collision = rtu_idu_fwd0_vld && rtu_idu_fwd1_vld
                         && (ex1_fwd_preg == iu_rtu_ex3_mul_preg) && !fwd_reg_is_x0_0;
    wire fwd02_collision = rtu_idu_fwd0_vld && rtu_idu_fwd2_vld
                         && (ex1_fwd_preg == lsu_rtu_ex2_dest_reg) && !fwd_reg_is_x0_0;
    wire fwd12_collision = rtu_idu_fwd1_vld && rtu_idu_fwd2_vld
                         && (iu_rtu_ex3_mul_preg == lsu_rtu_ex2_dest_reg) && !fwd_reg_is_x0_1;

    wire dbg_fwd_collision /* verilator public */;
    assign dbg_fwd_collision = fwd01_collision || fwd02_collision || fwd12_collision;

    //=========================================================================
    // SECTION EX1->EX2 RETIRE REGISTER (task 4.1, RTU note S2 -- the single
    // un-skidded pipe register: retiring at most 1/cycle, 0/cycle on any
    // stall, no queue, no second entry). Unlike the donor's conditional-hold
    // `dp_ex2_*` register (aq_rtu_dp.v:434-454, gated on `dp_ex1_cmplt` to
    // save ASIC clock-gating power), this body uses a PLAIN unconditional
    // register for every field: functionally equivalent because every
    // consumer below ANDs in `ex2_retire_vld` (or an is-this-field's-source-
    // cmplt qualifier) before trusting the field's value -- the donor's own
    // retire_trap_vld/chgflw_vld/etc. pattern -- and RTL simulation has no
    // need to preserve an ASIC-only power optimization. `ex2_retire_vld`
    // itself (the donor's `ctrl_ex2_cmplt`/`retire_ex2_retire_vld`,
    // aq_rtu_ctrl.v:174-182) is ALREADY unconditional in the donor.
    //=========================================================================
    wire ex1_inst_chgflw = ex1_cp0_cmplt_dp && cp0_rtu_ex1_chgflw;   // mret (M2's only chgflw source)
    wire [PC_WIDTH-1:0] ex1_next_pc = ex1_inst_chgflw ? cp0_rtu_ex1_chgflw_pc : iu_rtu_ex1_next_pc;

    // Exception vec/tval mux (aq_rtu_dp.v:409-415): prefer CP0 when CP0 is
    // the cmplt source, else LSU when LSU is, else neither -- avoids
    // leaking a stale LSU tval/vec into a CP0-sourced (or no) exception.
    wire ex1_inst_expt = cp0_rtu_ex1_expt_vld || (lsu_rtu_expt_vld && ex1_lsu_cmplt_dp);
    wire [4:0] ex1_expt_vec = ex1_cp0_cmplt_dp ? cp0_rtu_ex1_expt_vec
                            : ex1_lsu_cmplt_dp ? lsu_rtu_expt_vec
                            :                     5'd0;
    // M4 Task 6: a CP0-dispatched fetch fault (vec 12 page-fault / vec 1
    // access-fault, IDU's fault-marker EU_CP0 forcing) reports tval == epc
    // == the faulting fetch PC -- iu_rtu_ex1_cur_pc, the SAME signal
    // ex2_cur_pc (below) latches for epc this exact EX1 cycle. See header.
    wire ex1_cp0_fetch_fault = ex1_cp0_cmplt_dp
                            && (cp0_rtu_ex1_expt_vec == CAUSE_FETCH_PAGE_FAULT
                             || cp0_rtu_ex1_expt_vec == CAUSE_FETCH_ACCESS);
    // Sign-extend (not zero-extend): a kernel-space fetch fault's tval must
    // equal the faulting PC's canonical VA, same defect class as IU.v's
    // iu_ifu_tar_pc/ag_rs1_live/bju_wb_data.
    wire [63:0] ex1_tval = ex1_lsu_cmplt_dp   ? lsu_rtu_tval
                          : ex1_cp0_fetch_fault ? {{(64-PC_WIDTH){iu_rtu_ex1_cur_pc[PC_WIDTH-1]}}, iu_rtu_ex1_cur_pc}
                          :                       64'd0;   // CP0 has no other tval port, see header

    reg        ex2_retire_vld;
    // Task 7.2: ex2_cur_pc is public-marked for the verisim harness
    // (rtl/verisim.h CPU_PC) -- it holds the PC of the instruction that
    // just crossed EX1->EX2, i.e. the retiring instruction's PC.
    reg [PC_WIDTH-1:0] ex2_cur_pc /* verilator public */;
    reg [PC_WIDTH-1:0] ex2_next_pc;
    reg        ex2_inst_expt;
    reg [4:0]  ex2_expt_vec;
    reg [63:0] ex2_tval;
    reg        ex2_inst_chgflw;
    // M5 Task 8: the retiring instruction's FP-ness + its accrued flags,
    // riding the SAME unconditional register stage as the rest of the
    // retire packet. `ex2_fpu_retire` (the donor's `rtu_cp0_fs_dirty_updt`)
    // is the FP-instruction-retire pulse CSR.v needs to dirty mstatus.FS;
    // `ex2_fpu_fflags` (the donor's `rtu_cp0_fflags`) is the value OR'ed
    // into fflags on that same retire cycle (D7 sticky accrual). Latching
    // both off the same EX1-cycle qualifiers keeps the two CSR-side effects
    // exactly paired, and unconditional latching matches every other field
    // here (see the section header: consumers qualify on the vld bit).
    reg        ex2_fpu_retire;
    reg [4:0]  ex2_fpu_fflags;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex2_retire_vld  <= 1'b0;
            ex2_cur_pc      <= {PC_WIDTH{1'b0}};
            ex2_next_pc     <= {PC_WIDTH{1'b0}};
            ex2_inst_expt   <= 1'b0;
            ex2_expt_vec    <= 5'd0;
            ex2_tval        <= 64'd0;
            ex2_inst_chgflw <= 1'b0;
            ex2_fpu_retire  <= 1'b0;
            ex2_fpu_fflags  <= 5'd0;
        end else begin
            ex2_retire_vld  <= dp_ex1_cmplt_dp;
            ex2_cur_pc      <= iu_rtu_ex1_cur_pc;
            ex2_next_pc     <= ex1_next_pc;
            ex2_inst_expt   <= ex1_inst_expt;
            ex2_expt_vec    <= ex1_expt_vec;
            ex2_tval        <= ex1_tval;
            ex2_inst_chgflw <= ex1_inst_chgflw;
            ex2_fpu_retire  <= ex1_fpu_cmplt_dp;
            ex2_fpu_fflags  <= fpu_rtu_ex1_falu_fflags;
        end
    end

    //=========================================================================
    // SECTION INTERRUPT PRIORITY ENCODER (RTU note S5 -- aq_rtu_int.v:52-81,
    // a faithful clone of the 15-source casez cause table). Fed by an
    // internal, permanently-0 vector -- see header's "deliberately not
    // added" note. `retire_int_inst` is therefore provably always 0
    // (the `|int_vld_raw` term forces it), matching task 4.1's "wired but
    // structurally never fire" framing exactly.
    //=========================================================================
    wire [14:0] int_vld_raw = 15'd0;

    reg [4:0] int_vec_enc;
    always @* begin
        casez (int_vld_raw)
            15'b1?????????????? : int_vec_enc = 5'd16; // mcip
            15'b01????????????? : int_vec_enc = 5'd18; // mhip
            15'b001???????????? : int_vec_enc = 5'd11; // meip
            15'b0001??????????? : int_vec_enc = 5'd3;  // msip
            15'b00001?????????? : int_vec_enc = 5'd7;  // mtip
            15'b000001????????? : int_vec_enc = 5'd9;  // seip
            15'b0000001???????? : int_vec_enc = 5'd1;  // ssip
            15'b00000001??????? : int_vec_enc = 5'd5;  // stip
            15'b000000001?????? : int_vec_enc = 5'd17; // moip
            15'b0000000001????? : int_vec_enc = 5'd16; // mcip
            15'b00000000001???? : int_vec_enc = 5'd18; // mhip
            15'b000000000001??? : int_vec_enc = 5'd9;  // seip
            15'b0000000000001?? : int_vec_enc = 5'd1;  // ssip
            15'b00000000000001? : int_vec_enc = 5'd5;  // stip
            15'b000000000000001 : int_vec_enc = 5'd17; // moip
            default              : int_vec_enc = 5'd0; // donor uses X here; 0 is lint/sim-friendlier for a dead leg
        endcase
    end

    wire dtu_int_mask_tied0  = 1'b0;   // no DTU in M2
    wire int_ex2_split_tied0 = 1'b0;   // no split-instruction concept in M2

    wire       retire_int_inst = (|int_vld_raw) && !dtu_int_mask_tied0 && !int_ex2_split_tied0;
    wire [4:0] retire_int_vec  = int_vec_enc;

    //=========================================================================
    // SECTION EXCEPTION/INTERRUPT PRIORITY CHAIN (task 4.1, RTU note S4 --
    // aq_rtu_retire.v:481-501's exact if/else-if order): pending-breakpoint
    // > interrupt > LSU async bus error > ebreak/debug breakpoint > the
    // synchronous EX1 exception CP0/LSU already decided. Legs 1
    // (pending-breakpoint) and 4 (ebreak/debug breakpoint) are permanently 0
    // -- no DTU in M2 (ebreak itself already retires via leg 5: CSR.v
    // declares it as a plain synchronous EX1 exception, vec=3, not via a
    // separate debug-trigger path). Leg 2 (interrupt) is permanently 0 per
    // the section above. Leg 3 (LSU async bus error) is REAL and wired for
    // real -- it is simply 0 today because LSU.v (Task 6) does not exist
    // yet. M2 has no debug unit and no directed interrupt test, so legs 1/2/4
    // are wired but structurally never fire, per task 4.1's own framing.
    //=========================================================================
    wire retire_pending_bkpt_expt = 1'b0;                      // leg 1: no DTU
    // retire_int_inst                                          // leg 2: see above
    wire retire_async_expt        = lsu_rtu_async_expt_vld;    // leg 3: real, LSU-sourced
    wire retire_bkpt_expt         = 1'b0;                      // leg 4: no DTU

    reg [4:0] retire_trap_vec;
    always @* begin
        if (retire_pending_bkpt_expt)
            retire_trap_vec = 5'd3;
        else if (retire_int_inst)
            retire_trap_vec = retire_int_vec;
        else if (retire_async_expt)
            retire_trap_vec = lsu_rtu_async_ld_inst ? 5'd5 : 5'd7;
        else if (retire_bkpt_expt)
            retire_trap_vec = 5'd3;
        else
            retire_trap_vec = ex2_expt_vec;                    // leg 5: CP0/LSU sync exception
    end

    // retire_mmu_trap (RTU note S4, aq_rtu_retire.v:503-505): carried
    // forward UNCHANGED, including the donor's own {1,13,15} (not
    // {12,13,15}) quirk -- design doc S8 flags this as an M4 question; this
    // is NOT a bug for Task 4 to fix. No M2 consumer exists for this signal
    // (no MMU port on RTU.v's frozen list -- MMU is M4 scope), so it is
    // computed here purely to document the carry-forward, matching the
    // donor's own logic exactly, not exposed as a port.
    wire retire_mmu_trap = (retire_trap_vec == 5'd1)
                        || (retire_trap_vec == 5'd13)
                        || (retire_trap_vec == 5'd15);

    // Sync/overall exception classification (aq_rtu_retire.v:434-464,
    // simplified: retire_pending_bkpt_expt/retire_bkpt_expt are both 0).
    wire retire_sync_expt = ex2_inst_expt;
    wire retire_expt_inst = retire_sync_expt || retire_async_expt;

    // mtval allowlist (RTU note S4, aq_rtu_retire.v:514-522): {1,2,4,5,6,7,
    // 12,13,15} -- carried forward UNCHANGED, do not "fix". Checked against
    // the SYNCHRONOUS leg's own vec (ex2_expt_vec), exactly matching the
    // donor's `retire_inst_expt_vec` (not the final, priority-resolved
    // `retire_trap_vec`) -- equivalent by construction since this term is
    // only reached once legs 1-4 have all already been ruled out.
    function automatic vec_in_tval_allowlist(input [4:0] vec);
        vec_in_tval_allowlist = (vec == 5'd1)  || (vec == 5'd2)  || (vec == 5'd4)
                             || (vec == 5'd5)  || (vec == 5'd6)  || (vec == 5'd7)
                             || (vec == 5'd12) || (vec == 5'd13) || (vec == 5'd15);
    endfunction

    // tval selection (aq_rtu_retire.v:530-550).
    wire [63:0] retire_trap_tval =
          retire_pending_bkpt_expt ? 64'd0
        : retire_int_inst          ? 64'd0
        : retire_async_expt        ? lsu_rtu_tval
        : retire_bkpt_expt         ? 64'd0
        : vec_in_tval_allowlist(ex2_expt_vec) ? ex2_tval
        :                                       64'd0;

    // epc selection (aq_rtu_retire.v:564-577, simplified: M2 has no split-
    // instruction concept, so the donor's `&& dp_retire_ex2_inst_split`
    // async-epc-uses-cur-pc term never applies here).
    wire [PC_WIDTH-1:0] retire_trap_epc = retire_sync_expt ? ex2_cur_pc : ex2_next_pc;

    // Trap-taken ack (aq_rtu_retire.v:585-589, simplified: halt_req/
    // dbg_mode_on are both permanently 0/false -- no DTU in M2).
    wire retire_trap_vld = ex2_retire_vld && (retire_expt_inst || retire_int_inst);
    wire retire_trap_int = retire_int_inst && !retire_pending_bkpt_expt;

    //=========================================================================
    // SECTION FLUSH FSM (task 4.1, RTU note S6 -- aq_rtu_retire.v:929-978),
    // 5 states: IDLE -> FE -> [WAIT] -> BE -> IDLE, WAIT inserted between FE
    // and BE only if the pipe isn't drained yet. The async-debug
    // IDLE->FE_BE->IDLE shortcut is cloned as dead-but-present (design doc's
    // "clone the FSM shape as-is" note): `halt_req_dm_async_tied0` is
    // permanently 0 (no DTU in M2), so the `else if` branch checking it is
    // structurally present but never taken.
    //=========================================================================
    localparam FLUSH_IDLE  = 3'b000;
    localparam FLUSH_FE    = 3'b001;
    localparam FLUSH_WAIT  = 3'b100;
    localparam FLUSH_BE    = 3'b010;
    localparam FLUSH_FE_BE = 3'b011;

    wire halt_req_dm_async_tied0 = 1'b0;   // no DTU / async-debug-halt request in M2

    // rv906's CSR.v (Task 2, already committed) exposes no
    // `cp0_rtu_ex1_flush`-shaped bit at all -- only `_chgflw`/`_chgflw_pc`
    // (mret) and nothing whatsoever for fence/fence.i's serializing role
    // (CSR.v's own DECODE-section note: a deliberate M2 scope decision, not
    // an oversight). `retire_flush_fe_set` below therefore ORs in
    // `ex2_inst_chgflw` (mret) directly rather than a separate
    // `ex2_inst_flush` term (which would be permanently 0 anyway, since no
    // such CSR.v port exists) -- functionally covering the donor's "xret"
    // leg of `dp_retire_ex2_inst_flush` while faithfully carrying forward
    // the also-permanently-0 fence leg per CSR.v's own scope decision. This
    // is the real donor's own intent (mret needs a front-end flush) even
    // though the specific bit path differs due to CSR.v's simpler,
    // collapsed signal shape.
    wire retire_inst_flush_fe_set = ex2_retire_vld
                                  && (retire_expt_inst || retire_int_inst || ex2_inst_chgflw);
    // vstart_updt/bkpt_expt_t1/debug_flush/halt_req/halt_req_t1 all omitted
    // -- no vector/debug unit anywhere in M2's contracts (see header).

    wire retire_bju_flush_req = iu_rtu_ex2_bju_ras_mispred || iu_rtu_depd_lsu_chgflow_vld;
    wire retire_flush_fe_set  = retire_inst_flush_fe_set || retire_bju_flush_req;

    wire retire_commit_clear         = retire_inst_flush_fe_set || retire_bju_flush_req || retire_flush_fe;
    wire retire_commit_clear_for_bju = retire_inst_flush_fe_set || iu_rtu_ex2_bju_ras_mispred || retire_flush_fe;

    // Drain gate (aq_rtu_retire.v:980-987, simplified per header's
    // documented gap: no `iu_xx_no_op`/`lsu_rtu_no_op` port anywhere in M2;
    // the donor's `!lsu_rtu_ex1_buffer_vld` STB check is omitted per
    // contract 4's own "STB drains unconditionally" resolution, not a gap).
    wire wb_no_op   = !rtu_idu_wb0_vld_int && !rtu_idu_wb1_vld_int;
    wire cpu_no_op  = !ex2_retire_vld && wb_no_op;
    wire pipeline_empty = cpu_no_op;

    reg [2:0] flush_state;
    wire [2:0] flush_next_state =
          (flush_state == FLUSH_IDLE)   ? (retire_flush_fe_set ? FLUSH_FE : FLUSH_IDLE)
        : (flush_state == FLUSH_FE)     ? (cpu_no_op ? FLUSH_BE : FLUSH_WAIT)
        : (flush_state == FLUSH_WAIT)   ? (cpu_no_op ? FLUSH_BE : FLUSH_WAIT)
        : (flush_state == FLUSH_BE)     ? FLUSH_IDLE
        : (flush_state == FLUSH_FE_BE)  ? FLUSH_IDLE
        :                                  FLUSH_IDLE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            flush_state <= FLUSH_IDLE;
        else if (halt_req_dm_async_tied0)          // dead-but-present shortcut, see header
            flush_state <= FLUSH_FE_BE;
        else
            flush_state <= flush_next_state;
    end

    wire retire_flush_wait = (flush_state == FLUSH_WAIT);
    wire retire_flush_fe   = flush_state[0];
    wire retire_flush_be   = flush_state[1];

    assign rtu_idu_commit         = !retire_commit_clear;
    assign rtu_idu_commit_for_bju = !retire_commit_clear_for_bju;

    //=========================================================================
    // SECTION CHANGEFLOW (task 4.1, RTU note S5/S6/S7): the trap-redirect
    // and mret-redirect sequencing, registered exactly as the donor's own
    // `retire_trap_chgflw_vld`/`retire_xret_vld` (aq_rtu_retire.v:993-1019).
    //=========================================================================
    reg retire_trap_chgflw_vld;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                       retire_trap_chgflw_vld <= 1'b0;
        else if (retire_trap_vld)         retire_trap_chgflw_vld <= 1'b1;
        else if (retire_flush_be)         retire_trap_chgflw_vld <= 1'b0;
    end

    wire retire_ex2_retire_normal = ex2_retire_vld && !retire_sync_expt;

    reg retire_xret_vld;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                                  retire_xret_vld <= 1'b0;
        else if (retire_ex2_retire_normal && ex2_inst_chgflw)         retire_xret_vld <= 1'b1;   // mret (no sret in M2)
        else if (retire_flush_be)                                    retire_xret_vld <= 1'b0;
    end

    // Changeflow-PC mux (aq_rtu_retire.v:1026-1039, simplified: no DTU
    // `retire_exit_debug`/`dtu_rtu_dpc` leg in M2).
    wire [PC_WIDTH-1:0] retire_chgflw_pc = retire_trap_chgflw_vld ? cp0_rtu_trap_pc : ex2_next_pc;

    // Changeflow-valid (aq_rtu_retire.v:1003-1008, simplified: no
    // `dp_retire_ex2_inst_flush`/vstart/exit_debug legs in M2 -- see the
    // flush-FSM section's note on why `ex2_inst_chgflw` alone covers this).
    wire retire_chgflw_vld = (ex2_retire_vld && !retire_trap_vld && ex2_inst_chgflw)
                           || (retire_trap_chgflw_vld && retire_flush_fe);

    //=========================================================================
    // SECTION WB0/WB1 REGISTER (task 4.1 -- 2 architectural GPR write
    // ports). wb0 = the rbus arbiter's winner, registered ONE cycle behind
    // the EX1-cycle producer signals -- the SAME single register stage as
    // the retire packet above (design doc S7.3: "retire and architectural
    // writeback are the same cycle... RTU's EX2 retire register *is* the
    // point where the GPR write ports are driven"), NOT an additional
    // second stage (aq_rtu_wb.v:160-166's "Rbus Datapath" register captures
    // rbus.v's OWN combinational output exactly once, at the same pipeline
    // depth as dp.v's dp_ex2_* latch -- confirmed by re-reading both
    // register enables side by side). wb1 = LSU's OWN dedicated late write
    // port, a PURE combinational passthrough of `lsu_rtu_wb_*` with NO
    // register at all (aq_rtu_wb.v:155,180-197 -- the donor adds no
    // register here either), per the header's LSU-writeback-path
    // resolution note.
    //=========================================================================
    reg        wb0_vld_r;
    reg [GPR_IDX_WIDTH-1:0] wb0_preg_r;
    reg [63:0] wb0_data_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb0_vld_r  <= 1'b0;
            wb0_preg_r <= {GPR_IDX_WIDTH{1'b0}};
            wb0_data_r <= 64'd0;
        end else begin
            wb0_vld_r  <= rbus_wb_vld;
            wb0_preg_r <= rbus_wb_preg;
            wb0_data_r <= rbus_wb_data;
        end
    end

    assign rtu_idu_wb0_vld  = wb0_vld_r;
    assign rtu_idu_wb0_reg  = wb0_preg_r;
    assign rtu_idu_wb0_data = wb0_data_r;

    // M5 Task 4c (D8): gated !lsu_rtu_wb_dst_frf -- an FLW/FLD completion
    // must NOT write the GPR file (its dst0_reg index was never allocated in
    // the GPR scoreboard -- IDU.v leaves dis_dst0_vld=0 for these, see
    // rv906's own FRF-destination note there). Route it to wbf1 instead.
    assign rtu_idu_wb1_vld  = lsu_rtu_wb_vld && !lsu_rtu_wb_dst_frf;
    assign rtu_idu_wb1_reg  = lsu_rtu_wb_preg;
    assign rtu_idu_wb1_data = lsu_rtu_wb_data;

    assign rtu_idu_wbf1_vld  = lsu_rtu_wb_vld && lsu_rtu_wb_dst_frf;
    assign rtu_idu_wbf1_reg  = lsu_rtu_wb_preg;
    assign rtu_idu_wbf1_data = lsu_rtu_wb_data;

    // ---- wbf0 (M5 Task 4b): FRF write port, registered ONE cycle behind
    // the EX1-cycle FALU producer -- same single register stage as wb0
    // above (design doc S7.3). No arbiter needed: FALU is the only wbf0
    // producer until FMAU/FDSU land (Tasks 5/6), at which point a merge
    // analogous to ex1_fwd_src_vld's one-hot mux belongs here. wbf1 stays
    // reserved/tied-off (future LSU-FRF path, "Task 4c").
    reg        wbf0_vld_r;
    reg [GPR_IDX_WIDTH-1:0] wbf0_preg_r;
    reg [63:0] wbf0_data_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wbf0_vld_r  <= 1'b0;
            wbf0_preg_r <= {GPR_IDX_WIDTH{1'b0}};
            wbf0_data_r <= 64'd0;
        end else begin
            wbf0_vld_r  <= fpu_rtu_ex1_falu_fvld;
            wbf0_preg_r <= fpu_rtu_ex1_falu_preg;
            wbf0_data_r <= fpu_rtu_ex1_falu_fdata;
        end
    end

    assign rtu_idu_wbf0_vld  = wbf0_vld_r;
    assign rtu_idu_wbf0_reg  = wbf0_preg_r;
    assign rtu_idu_wbf0_data = wbf0_data_r;

    // Internal aliases so the drain-gate section above (which is written
    // textually before this section) can read wb0/wb1 vld without a
    // forward-reference to the `assign`-only output wires.
    wire rtu_idu_wb0_vld_int = wb0_vld_r;
    wire rtu_idu_wb1_vld_int = lsu_rtu_wb_vld;

    //=========================================================================
    // SECTION OUTPUT -- the remaining RTU-originated signal inventory (RTU
    // note S6): broadcast to CSR, redirect to IFU, flush/commit/drain to
    // IDU, "point of no return" acks to LSU.
    //=========================================================================
    assign rtu_yy_xx_expt_vld = retire_trap_vld;
    assign rtu_yy_xx_expt_int = retire_trap_int;
    assign rtu_yy_xx_expt_vec = retire_trap_vec;
    assign rtu_yy_xx_flush_fe = retire_flush_fe;
    assign rtu_yy_xx_flush    = retire_flush_be;
    assign rtu_yy_xx_dbgon    = 1'b0;   // no DTU in M2

    assign rtu_cp0_epc  = retire_trap_epc;
    assign rtu_cp0_tval = retire_trap_tval;
    assign rtu_cp0_inst_retire = ex2_retire_vld;

    // M5 Task 8 (D7): off the EX2 retire packet above -- CSR.v OR's
    // rtu_cp0_fflags into fflags and takes rtu_cp0_fs_dirty_updt as its
    // FP-instruction-retire FS-dirty source (SECTION FP CSR STATE there).
    assign rtu_cp0_fflags        = ex2_fpu_fflags;
    assign rtu_cp0_fs_dirty_updt = ex2_fpu_retire;

    assign rtu_ifu_chgflw_vld = retire_chgflw_vld;
    assign rtu_ifu_chgflw_pc  = retire_chgflw_pc;
    assign rtu_ifu_flush_fe   = retire_flush_fe;

    assign rtu_idu_flush_fe      = retire_flush_fe;
    assign rtu_idu_flush_stall   = retire_flush_wait || retire_flush_be;
    assign rtu_idu_flush_wbt     = retire_flush_be;
    assign rtu_idu_pipeline_empty= pipeline_empty;

    assign rtu_lsu_expt_ack  = retire_trap_chgflw_vld && retire_flush_be;
    assign rtu_lsu_expt_exit = retire_xret_vld && retire_flush_be;

endmodule
