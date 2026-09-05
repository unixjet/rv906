//=============================================================================
// LSU.v - AG -> DC -> DA load/store pipe, 4-entry STB, single-outstanding
//          miss stand-in, D-side AXI master   (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 6; this file freezes
// the port list only):
//   gen_rtl/lsu/rtl/aq_lsu_top.v   (glue/instantiation order, LSU note A1)
//   gen_rtl/lsu/rtl/aq_lsu_ag.v    (address-gen: adder, misalign detect,
//                                    MMU-stub request issue)
//   gen_rtl/lsu/rtl/aq_lsu_dc.v    (tag compare / DC+DA stages, byte
//                                    rotate + sign/zero-extend)
//   gen_rtl/lsu/rtl/aq_lsu_stb.v   (+ _entry.v; 4-entry store buffer,
//                                    byte-granular store-to-load forward)
//   gen_rtl/lsu/rtl/aq_lsu_rdl.v   (writes refill/store/dirty data into the
//                                    DCache SRAMs)
// References: design doc S2.1/S2.2/S2.3.1/S2.3.3/S4.1/S4.3, contract 2
// (MMU protocol), contract 3 (misalignment: trap-only, ignores MXSTATUS.mm),
// contract 4 (STB: clone as-is, no RTU-commit gate, but DOES need the
// local flush/cancel interlock), contract 9 (DCache geometry, no alias
// bank), contract 11 (victim writeback in scope), LSU note A2/A4/A5/A6/A7.
//
// PORT-LIST AMENDMENT (Task 6, same "documented amendment, not silent"
// discipline as CSR.v/RTU.v/IDU.v's own precedent notes): RTU.v's Task 4
// body already added `lsu_rtu_ex2_dest_reg` as its OWN input ("a forward
// with data but no target register cannot be consumed by IDU's forward
// mux... added here, mirroring the donor's own `lsu_rtu_ex2_dest_reg`") --
// but this file's Task-1-frozen port list never grew the matching OUTPUT.
// Added here, closing the loop RTU.v opened. `lsu_rtu_ex2_data`/
// `_data_vld`/`_dest_reg` are driven identically to `lsu_rtu_wb_data`/
// `_vld`/`_preg` (same cycle, same value) -- RTU.v's own rbus section wires
// both families as PURE combinational passthroughs with no register of
// their own (`rtu_idu_fwd2_* = lsu_rtu_ex2_*` and `rtu_idu_wb1_* =
// lsu_rtu_wb_*`, RTU.v's rbus section), so the "fwd2 is one cycle earlier
// than wb1" framing (RTU note S3) is actually about WHEN in LSU's own
// pipeline these assert relative to the instruction's issue, not a
// relative offset between the two ports themselves: this file asserts them
// together, in the SAME cycle, exactly the cycle a load/store completes
// (REPLY) -- `wb1` becomes the architectural GPR write (IDU's `gpr_r`
// registers commit off it at the next edge) while `fwd2` is the SAME-CYCLE
// combinational bypass a dependent instruction sitting in ID/DIS this same
// cycle can read immediately (IDU note S6's fwd_data mux).
//
// SEAM NOTES:
//  * `idu_lsu_ex1_dp_sel` and `idu_lsu_ex1_sel` are BOTH real, distinct
//    donor signals, confirmed from the producer side (`aq_idu_id_ctrl.v:
//    634` vs. `:646`) and the consumer side (`aq_lsu_ag.v:655-656`:
//    `ag_dp_sel = idu_lsu_ex1_dp_sel`, `ag_inst_vld = idu_lsu_ex1_sel`):
//    `_dp_sel` is the ungated early select AG's speculative operand-mux
//    datapath uses; `_sel` is the same term additionally gated on
//    `rtu_idu_commit` -- the true architectural go-ahead. Both ports are
//    frozen here (see IDU.v's header for the full derivation); Task 5/6
//    wire the two different gating conditions for real.
//  * `lsu_idu_full` (this module's single point-to-point stall signal to
//    IDU, contract 8) is CONFIRMED directly from `aq_idu_id_ctrl.v:634`'s
//    consumer-side reference, not guessed.
//  * `lsu_rtu_*`/`lsu_mmu_*`/`mmu_lsu_*` names/widths match RTU.v's/
//    MMU.v's own port groups exactly (this task's own skeletons) -- see
//    RTU.v's header for the flagged open item on whether the donor's 3
//    distinct LSU->RTU writeback paths collapse to the 2 this skeleton
//    pins, or need a 3rd port later.
//  * DCache.v is instantiated INSIDE this module (not at RVProc.v's top
//    level, see DCache.v's header) -- but Task 1 does not require actually
//    instantiating it yet (the task text: "freezing the seven new
//    modules' port lists does not require instantiating them anywhere
//    yet"), so this skeleton's body does not reference DCache.v at all.
//  * The AXI D-channel port group below is copied verbatim from
//    rtl/RVProc.v's existing outer port list (lines 90-127) -- the same
//    channel FetchSink.v currently drives with its tohost-only write FSM;
//    Task 7 moves the connection, not the shape.
//=============================================================================

import rvproc_pkg::*;

module LSU #(
    parameter DATA_WIDTH = 512,
    parameter ADDR_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IDU -> LSU : EX1 dispatch, LSU's slice of id_ex1_t (matches IDU.v's
    // output group exactly).
    //=========================================================================
    input  wire                     idu_lsu_ex1_dp_sel,
    input  wire                     idu_lsu_ex1_sel,
    // M4 Task 5 fix: `idu_lsu_ex1_sel` minus its own `!lsu_idu_full` gate --
    // see `ag_raw_ready` (SECTION AG WAIT-STATE) for why the entry-cycle
    // detection needs this ungated signal instead of `ag_valid`.
    input  wire                     idu_lsu_ex1_raw_vld,
    input  wire [FUNC_WIDTH-1:0]    idu_lsu_ex1_func,
    input  wire [63:0]              idu_lsu_ex1_src0_data, // base addr reg
    input  wire                     idu_lsu_ex1_src0_ready,
    input  wire [63:0]              idu_lsu_ex1_src1_data, // offset imm
    input  wire                     idu_lsu_ex1_src1_ready,
    input  wire [63:0]              idu_lsu_ex1_src2_data, // store data
    input  wire                     idu_lsu_ex1_src2_ready,
    input  wire [GPR_IDX_WIDTH-1:0] idu_lsu_ex1_dst0_reg,  // load dest
    // Task 7.3: the EX1 instruction's length (1=32b,0=16b RVC) for the LSU
    // slice -- latched with the in-flight instruction, reported back to the
    // RTU's pcgen inst_len mux (aq_rtu_dp.v:367 lsu arm).
    input  wire                     idu_lsu_ex1_inst_len,
    // M3b Task D: the EX1 instruction's PC[15:0] from the IU, latched into
    // the DC stage as the PFB trainer's PC tag (donor iu_lsu_ex1_cur_pc,
    // aq_lsu_ag.v:196/685 -> ag_pipe_pc -> dc_ld_pc, aq_lsu_dc.v:1459).
    input  wire [15:0]              iu_lsu_ex1_cur_pc,

    //=========================================================================
    // LSU -> IDU : the single EX1 issue-gate stall signal (contract 8;
    // confirmed name, aq_idu_id_ctrl.v:634).
    //=========================================================================
    output wire                     lsu_idu_full,
    // LSU -> CSR : store-buffer/pipe quiescence (Task 10.1, fence.i): high
    // when no store/miss is in flight AND the STB is empty -- FENCE/FENCE.I
    // wait for this before completing.
    output wire                     lsu_cp0_stb_empty,
    // CSR -> LSU / LSU -> CSR : FENCE.I D-cache clean walk (donor
    // aq_cp0_fence_inst.v FNC_CDCA stage): CSR holds `cp0_lsu_dcache_clean`
    // from LSU-quiescence until `lsu_cp0_clean_done` pulses; LSU walks every
    // set, writes back each valid+dirty line (reusing the FRZ AXI-write
    // sub-sequence) and invalidates it via dc_inv_* (frozen ports, driven
    // for real starting now). Required so store-hit bytes reach the backing
    // memory the ICache refills from, before the I-side invalidate.
    input  wire                     cp0_lsu_dcache_clean,
    output wire                     lsu_cp0_clean_done,

    //=========================================================================
    // LSU -> RTU : lsu_rtu_t (design doc S4.2) -- matches RTU.v's input
    // group exactly.
    //=========================================================================
    output wire                     lsu_rtu_ex1_cmplt,
    output wire                     lsu_rtu_ex1_cmplt_dp,
    // Task 9.7 (class-B clone fix, donor aq_lsu_ag.v:1675): the EARLY
    // "for pcgen" completion -- fires the cycle AG accepts the instruction
    // (the store's address-gen is done and the DC FSM is ready to issue),
    // NOT when the memory op's REPLY lands. The donor drives the IU pcgen
    // off exactly this signal (`lsu_rtu_ex1_cmplt_for_pcgen =
    // ag_pipe_cmplt_normal`), keeping the pcgen in lockstep with the EX1
    // register: a store leaves EX1 one cycle after entering it, but its
    // memory op (this module's IDLE->DCS->FRZ/REPLY pipe) runs several
    // cycles longer. Advancing the pcgen on the late `lsu_rtu_ex1_cmplt_dp`
    // instead leaves it one instruction (4B) behind the EX1-resident
    // instruction, so the next auipc/branch computes pc+imm from the
    // PREVIOUS instruction's pc. See the matching RTU.v note for the
    // retire-vs-pcgen separation (donor aq_rtu_ctrl.v:151-157).
    output wire                     lsu_rtu_ex1_cmplt_for_pcgen,
    // Task 7.3: the COMPLETING LSU instruction's length, for the RTU pcgen
    // inst_len mux (donor aq_lsu_top.v:405 / aq_rtu_dp.v:367).
    output wire                     lsu_rtu_ex1_inst_len,
    output wire [63:0]              lsu_rtu_wb_data,
    output wire [GPR_IDX_WIDTH-1:0] lsu_rtu_wb_preg,
    output wire                     lsu_rtu_wb_vld,
    output wire [63:0]              lsu_rtu_ex2_data,
    output wire                     lsu_rtu_ex2_data_vld,
    output wire [GPR_IDX_WIDTH-1:0] lsu_rtu_ex2_dest_reg,   // Task 6 amendment, see header
    output wire                     lsu_rtu_expt_vld,
    output wire [4:0]               lsu_rtu_expt_vec,
    output wire [63:0]              lsu_rtu_tval,
    output wire                     lsu_rtu_async_expt_vld,
    output wire                     lsu_rtu_async_ld_inst,
    // M3 Task 1: LR.W / SC.W foundation outputs
    output wire                     lsu_rtu_lr_vld,
    output wire [4:0]               lsu_rtu_sc_res,   // 0=success(commit), 1=fail

    //=========================================================================
    // RTU -> LSU : "point of no return" acks (RTU note S6) -- also the
    // signals that gate the STB-create-vs-flush interlock contract 4
    // requires (verified cycle-by-cycle in Task 6, not assumed here).
    //=========================================================================
    input  wire                     rtu_lsu_expt_ack,
    input  wire                     rtu_lsu_expt_exit,
    // M4 Task 5 (D1): RTU's front-end flush broadcast -- clears ag_wait_r
    // (SECTION AG below) so a parked DTLB-miss op that turns out to be on
    // the wrong path is dropped rather than replayed once the walk lands.
    // Matches CSR.v's/IDU.v's own consumption of this same RTU.v broadcast
    // (`assign rtu_yy_xx_flush_fe = retire_flush_fe;`, RTU.v:903).
    input  wire                     rtu_yy_xx_flush_fe,

    //=========================================================================
    // LSU -> MMU / MMU -> LSU : the DTLB request/response (contract 2),
    // matches MMU.v's DTLB port group exactly.
    //=========================================================================
    output wire [MMU_VA_WIDTH-1:0]  lsu_mmu_va,
    output wire                     lsu_mmu_va_vld,
    output wire [1:0]               lsu_mmu_priv_mode,
    output wire                     lsu_mmu_st_inst,
    input  wire [MMU_PA_WIDTH-1:0]  mmu_lsu_pa,
    input  wire                     mmu_lsu_pa_vld,
    input  wire                     mmu_lsu_ca,
    input  wire                     mmu_lsu_so,
    input  wire                     mmu_lsu_buf,
    input  wire                     mmu_lsu_sec,
    input  wire                     mmu_lsu_sh,
    input  wire                     mmu_lsu_page_fault,
    input  wire                     mmu_lsu_access_fault,
    // M4 Task 5 (D1): the walk-abort pin for THIS port (donor's
    // lsu_mmu_abort0/1 collapsed to one -- rv906's single-outstanding DTLB
    // request). Driven straight off the same flush broadcast: any in-flight
    // walk this port started is for a VA that is about to be squashed.
    output wire                     lsu_mmu_abort,

    //=========================================================================
    // M4 Task 5 (S4/D3): the PTW memory-read servant (donor aq_lsu_mcic.v,
    // rv12 P3 precedent) -- MMU.v's group-5 six-wire channel. See SECTION
    // PTW SERVANT below for the full contract.
    //=========================================================================
    input  wire                     mmu_lsu_data_req,
    input  wire [PC_WIDTH-1:0]      mmu_lsu_data_req_addr,
    input  wire                     mmu_lsu_data_req_size,
    output wire [63:0]              lsu_mmu_data,
    output wire                     lsu_mmu_data_vld,
    output wire                     lsu_mmu_bus_error,

    //=========================================================================
    // CSR -> LSU : MHCR.de/wa + MXSTATUS.mm (matches CSR.v's output group
    // exactly).
    //=========================================================================
    input  wire                     cp0_lsu_dcache_en,
    input  wire                     cp0_lsu_mm,
    input  wire                     cp0_lsu_wa,
    // M3b Task D: MHINT D-cache prefetch controls (donor aq_cp0_ext_csr.v
    // :861,:868) feeding the PFB stride prefetcher below.
    input  wire                     cp0_lsu_dcache_pref_en,
    input  wire [1:0]               cp0_lsu_dcache_pref_dist,
    // M3b Task E: MHINT.amr (ext_csr.v:881) enabling the AMR streaming-store
    // write-allocate disabler below (00 = off, the reset default).
    input  wire [1:0]               cp0_lsu_amr,
    // M4 Task 1: privilege/MPRV controls (CSR.v storage; the effective
    // lsu_mmu_priv_mode resolution moves here at Task 5).
    input  wire                     cp0_lsu_mprv,
    input  wire [1:0]               cp0_lsu_mpp,
    input  wire [1:0]               cp0_yy_priv_mode,

    //=========================================================================
    // AXI Master Interface - DCache (ch[1]) -- copied verbatim from
    // rtl/RVProc.v's existing outer port list (lines 90-127); moves here
    // from FetchSink.v's old tohost-only write FSM (design doc S2.1).
    //=========================================================================
    output wire                     axi_d_awvalid,
    input  wire                     axi_d_awready,
    output wire [ADDR_WIDTH-1:0]    axi_d_awaddr,
    output wire [7:0]               axi_d_awlen,
    output wire [2:0]               axi_d_awsize,
    output wire [1:0]               axi_d_awburst,
    output wire [3:0]               axi_d_awcache,
    output wire [2:0]               axi_d_awprot,

    output wire                     axi_d_wvalid,
    input  wire                     axi_d_wready,
    output wire [DATA_WIDTH-1:0]    axi_d_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_d_wstrb,
    output wire                     axi_d_wlast,

    input  wire                     axi_d_bvalid,
    output wire                     axi_d_bready,
    input  wire [1:0]               axi_d_bresp,

    output wire                     axi_d_arvalid,
    input  wire                     axi_d_arready,
    output wire [ADDR_WIDTH-1:0]    axi_d_araddr,
    output wire [7:0]               axi_d_arlen,
    output wire [2:0]               axi_d_arsize,
    output wire [1:0]               axi_d_arburst,
    output wire [3:0]               axi_d_arcache,
    output wire [2:0]               axi_d_arprot,

    input  wire                     axi_d_rvalid,
    output wire                     axi_d_rready,
    input  wire [DATA_WIDTH-1:0]    axi_d_rdata,
    input  wire [1:0]               axi_d_rresp,
    input  wire                     axi_d_rlast
);

    //=========================================================================
    // TASK 6 REAL BODY.
    //
    // Pipeline: AG (combinational) -> a 4-state DC control FSM (IDLE/DCS/
    // FRZ/REPLY, this module's OWN copy, one level up from DCache.v's
    // identically-named-and-shaped array-level FSM -- see DCache.v's header
    // for why the donor's FRZ (LFB/STB/VB wakeup wait, per a Task 6 research
    // pass reading aq_lsu_dc.v directly) is fundamentally an LSU-level
    // concept once those three structures live in LSU.v, not DCache.v) ->
    // DA (combinational, inside REPLY).
    //
    //   IDLE  -- AG's cycle: compute address/misalign/MMU request this
    //            cycle (LSU note A2: "AG... issues the MMU-stub request and
    //            the DCache tag/data SRAM read the same cycle"); latch every
    //            field the rest of the pipe needs; issue DCache.v's request
    //            (only if this access actually touches the array -- see
    //            below) or, absent an in-flight instruction, opportunistically
    //            issue an STB-drain instead (contract 4: "STB drains
    //            unconditionally once created" -- this is where that drain
    //            actually happens, at LSU's own idle cycles, so it never
    //            competes with a new instruction's own issue).
    //   DCS   -- DCache.v's response (fixed 1-cycle latency, see DCache.v's
    //            header) is valid THIS cycle for any access that touched the
    //            array; latch hit/way-info/data. Branches to REPLY (a
    //            misaligned access, or a cache hit, or a drain that touched
    //            the array) or FRZ (a genuine miss needing victim-check+
    //            refill, or anything bypassing the array entirely: an
    //            uncached access, a write-allocate-disabled store miss
    //            (contract 6, MHCR.wa=0 default), or a direct STB drain).
    //   FRZ   -- the single-outstanding-miss stand-in (design doc S2.2) and
    //            the minimal single-line victim-writeback path (contract
    //            11): on a cacheable miss, picks a replacement way (prefer
    //            an invalid way, else a small round-robin counter -- "the
    //            mechanism, not the donor's full per-set FIFO-pointer
    //            generality", contract 11's own framing), peeks that way's
    //            current tag+data via DCache.v's way-select mechanism,
    //            writes it back first if dirty (contract 11's own ordering,
    //            confirmed by a Task 6 research pass against aq_lsu_vb.v:
    //            "a dirty victim being written back first"), then issues
    //            one 512-bit single-beat AXI read for the new line (LSU
    //            note A1's single-outstanding-miss framing; trivially
    //            satisfies contract 17's 16-beat cap) and commits it into
    //            DCache.v. For anything bypassing the array (uncached load/
    //            store, a wa=0 store miss, a direct STB drain), runs one
    //            generic AXI read-or-write sub-sequence straight to memory
    //            instead -- the D-side AXI master moved here from
    //            FetchSink.v's old tohost-only write FSM (design doc S2.1),
    //            now a real, load-bearing bus path serving every one of
    //            these cases, not a fixed-address write FSM.
    //   REPLY -- DA: byte-rotate + sign/zero-extend (LSU note A5's inline
    //            `case({sign_ext,size})` shape) merged against a byte-
    //            granular STB forward (LSU note A4); asserts the completion/
    //            writeback/exception bus to RTU; creates (or merges into) an
    //            STB entry for a completing store.
    //
    // Every stage after IDLE is reached ONLY if `idu_lsu_ex1_sel` gated this
    // instruction's entry into AG in the first place -- this is the whole
    // of contract 4's STB-create-vs-flush interlock, and it is NOT assumed:
    // `idu_lsu_ex1_sel` (IDU.v, already committed) is
    // `ex1_eu_r[EU_LSU_SEL] && !ctrl_ex1_internal_stall && rtu_idu_commit &&
    // !lsu_idu_full`, where `rtu_idu_commit = !retire_commit_clear` and
    // `retire_commit_clear` already includes `retire_inst_flush_fe_set`/
    // `retire_bju_flush_req` -- BOTH combinational, same-cycle functions of
    // the flush-triggering condition itself (RTU.v, already committed), not
    // registered one cycle later. So a flush arriving the SAME cycle an
    // STB-create would otherwise fire drops `rtu_idu_commit` (hence
    // `idu_lsu_ex1_sel`) THAT SAME cycle, before AG ever computes an
    // address or DCache.v ever sees a request -- there is nothing left to
    // suppress downstream, because nothing downstream ever started. This
    // file's own AG stage is gated EXCLUSIVELY on `idu_lsu_ex1_sel` --
    // `idu_lsu_ex1_dp_sel` (the ungated, IDU-note-documented "speculative
    // operand-mux datapath" early select) is received but deliberately
    // wired to NOTHING with a side effect, for exactly this reason: using
    // it for anything that could create STB/DCache/AXI state would reopen
    // the interlock contract 4 exists to close. lsu_tb.cpp's own dedicated
    // interlock test drives `_dp_sel=1`/`_sel=0` (the exact shape a same-
    // cycle flush produces) and confirms zero side effects result --
    // verified cycle-by-cycle, not assumed, per this task's own instruction.
    // `rtu_lsu_expt_ack`/`_expt_exit` (RTU's later, FLUSH_BE-cycle "point of
    // no return" pulses) need no PIPELINE-state consumer here: by the time
    // either could fire for THIS instruction's own exception, LSU has already
    // fully resolved it (single-issue, single-outstanding -- there is no
    // younger, still-speculative LSU state left to roll back), and
    // `rtu_idu_flush_stall`'s own stall of IDU's ID/DIS stage (not a port on
    // this file) already prevents any wrong-path instruction from ever
    // reaching `idu_lsu_ex1_sel` during the drain window. They DO have one
    // live consumer since the M3 audit: the LR/SC reservation clear (donor
    // aq_lsu_lm.v:135 kills the reservation on expt_ack | expt_exit).
    // Flagged here, not silently guessed.
    //=========================================================================

    localparam WAYS = DCACHE_WAYS;

    //-------------------------------------------------------------------------
    // SECTION AG (LSU note A2) -- one 64-bit adder, combinational misalign
    // detect, the MMU-stub request, and the byte-position/mask math shared
    // by loads, stores, and STB forwarding.
    //-------------------------------------------------------------------------
    wire        ag_valid    = idu_lsu_ex1_sel;
    wire        ag_dp_unused = idu_lsu_ex1_dp_sel;   // received, not wired to a
                                                       // side effect -- see header

    // M4 Task 5 fix (D1 corrected): IDU does NOT hold EX1 across an AG wait
    // (adv is gated on ctrl_ex1_eu_full, which reads lsu_idu_full's OLD value
    // on the very cycle ag_wait_r sets -- one cycle too late to block the
    // reload; verified against IDU.v's ctrl_ex1_eu_full/adv chain). So the
    // live idu_lsu_ex1_* wires drift to the NEXT decoded op while a walk is
    // outstanding. Mirror the donor's own fix instead of IDU's: donor
    // aq_lsu_ag.v:323 `ag_req_buffer` (valid-gated at :1713
    // `lsu_full_stall = ag_req_buffer_vld`) latches the parked request in
    // LSU, not in IDU. `ag_wait_*_r` below is that buffer; `ag_*_eff` is the
    // buffer/live mux every downstream consumer of the AG-stage fields reads
    // through (transparently the live wire when ag_wait_r=0, so the M2/M3
    // same-cycle-hit path is untouched).
    wire [FUNC_WIDTH-1:0] ag_func_eff = ag_wait_r ? ag_wait_func_r : idu_lsu_ex1_func;

    wire        ag_is_store = ag_func_eff[0];
    wire        ag_sign_ext = ag_func_eff[1];
    wire [1:0]  ag_size     = ag_func_eff[3:2];   // 00=B,01=H,10=W,11=D
    // M3b: plain-load detector (not store / AMO / LR / SC). Only a plain
    // cacheable load miss is deferred into the LFB for a non-blocking refill;
    // store/AMO/LR/SC keep the blocking FRZ path this cut.
    wire        ag_is_plain_ld = !ag_is_store
                      && (ag_func_eff[19:12] != 8'h01)               // not AMO
                      && (ag_func_eff != LSU_FUNC_LR_W) && (ag_func_eff != LSU_FUNC_LR_D)
                      && (ag_func_eff != LSU_FUNC_SC_W) && (ag_func_eff != LSU_FUNC_SC_D);

    wire [63:0] ag_addr_live = idu_lsu_ex1_src0_data + idu_lsu_ex1_src1_data;
    wire [63:0] ag_addr      = ag_wait_r ? ag_wait_addr_r : ag_addr_live;
    wire [63:0] ag_src2_data_eff = ag_wait_r ? ag_wait_src2_data_r : idu_lsu_ex1_src2_data;
    wire [GPR_IDX_WIDTH-1:0] ag_dst0_reg_eff = ag_wait_r ? ag_wait_dst0_reg_r : idu_lsu_ex1_dst0_reg;
    wire        ag_inst_len_eff = ag_wait_r ? ag_wait_inst_len_r : idu_lsu_ex1_inst_len;

    wire ag_misalign = (ag_size == 2'b01 && ag_addr[0])
                     || (ag_size == 2'b10 && (|ag_addr[1:0]))
                     || (ag_size == 2'b11 && (|ag_addr[2:0]));
    // M4 trap-at-issue needs the store/AMO/SC-vs-load class at AG, before the
    // DC-stage dc_is_store_r/sc_addr_set/amo_active are latched. AMO = the
    // funct7 opcode byte 0x01 (ag_is_plain_ld's own test), SC = its two FUNC
    // encodings (ag_is_plain_ld's exclusions). LR is a load (vec 4).
    wire ag_is_amo_c = (ag_func_eff[19:12] == 8'h01);
    wire ag_is_sc_c  = (ag_func_eff == LSU_FUNC_SC_W)
                    || (ag_func_eff == LSU_FUNC_SC_D);
    // contract 3: `cp0_lsu_mm` is NEVER consulted here -- always trap.
    wire _cp0_lsu_mm_unused = cp0_lsu_mm;

    // The MMU request carries the PAGE NUMBER, not the byte address --
    // exactly the donor's own split (aq_lsu_ag.v:1566: `lsu_mmu_va[51:0] =
    // ag_pipe_addr[63:12]`), the SAME convention the I-side uses (ICache
    // drives `icache_rd_addr[63:12]`). The MMU answers with the 28-bit
    // physical page number (aq_lsu_ag.v:201: `input [27:0] mmu_lsu_pa`)
    // and THIS module reassembles the full PA as {page number,
    // addr[11:0]} (aq_lsu_ag.v:1446: `ag_pipe_pa = {mmu_pa,
    // ag_pipe_addr[11:0]}`). An earlier draft drove the byte VA here and
    // made the MMU do the >>12 -- observably equivalent but not
    // clone-faithful; corrected against the donor 2026-08-23.
    assign lsu_mmu_va        = ag_addr[12 +: MMU_VA_WIDTH];      // = ag_addr[63:12]
    // M4 Task 5 (D1): stays level-valid across a DTLB-miss wait too --
    // `ag_wait_r` below (declared ahead of use, matching this file's own
    // forward-reference style throughout) -- so the walk sees a stable
    // request for its whole duration. `ag_addr` is itself the buffer/live
    // mux (SECTION AG above) so this stays the SAME va for the whole wait.
    // M4 Task 5 fix: uses `ag_raw_ready` (declared just below), NOT
    // `ag_valid` -- `ag_valid = idu_lsu_ex1_sel` is itself gated by
    // `!lsu_idu_full`, and `lsu_idu_full` needs to read 1 on the very entry
    // cycle of a fresh DTLB miss (see `ag_raw_ready`'s own comment) --
    // using `ag_valid` here would go transiently, incorrectly low for one
    // cycle on every miss entry (`ag_valid ==
    // ag_raw_ready && !ag_wait_r`, and `ag_wait_r` is still 0 on that exact
    // cycle), dropping the MMU request the same cycle it is first raised.
    assign lsu_mmu_va_vld    = ag_raw_ready || ag_wait_r;
    // M4 Task 1: effective data-access privilege (donor aq_lsu_ag.v:674-675:
    // MPRV redirects loads/stores to MPP's privilege). While MPRV=0 this is
    // the current mode (M-mode = 2'b11 for the existing battery).
    assign lsu_mmu_priv_mode = cp0_lsu_mprv ? cp0_lsu_mpp : cp0_yy_priv_mode;
    // M4 Task 5: AMO and SC are write accesses too -- they must gate the
    // MMU's write-permission/D-bit checks (MMU.v's ptw_is_store and
    // dtlb_hit_perm_pf) exactly like a plain store, matching this same
    // file's own `mmu_fault_is_store` classification below (SECTION
    // TRAP-AT-ISSUE: `ag_is_store || ag_is_amo_c || ag_is_sc_c`). Without
    // this, an AMO to a D=0 page never raises the required StoreAMOPageFault
    // (D-M4-1): ag_is_store alone is 0 for AMO/SC, so ptw_is_store latches 0
    // and `(ptw_is_store && !pte_d)` never fires.
    assign lsu_mmu_st_inst   = ag_is_store || ag_is_amo_c || ag_is_sc_c;

    //-------------------------------------------------------------------------
    // SECTION AG WAIT-STATE (M4 Task 5, D1) -- a DTLB miss (mmu_lsu_pa_vld=0
    // the cycle this op would otherwise issue) cannot enter the DC pipe: the
    // PA that touches_array/ag_dc_tag/ag_dc_index depend on isn't ready yet.
    // `ag_wait_r` parks the op AT ST_IDLE (never touching the array/STB) for
    // as many cycles as the walk takes; `mmu_lsu_pa_vld` finally landing
    // (SAME va, held live above) lets `issue_real` (below) proceed exactly
    // as it would have same-cycle on the M2/M3 stub's always-same-cycle
    // answer -- the OFF path (satp Bare/M-mode, MMU.v's own mach_path) is
    // untouched: `mmu_lsu_pa_vld` is combinationally 1 there every cycle,
    // so `ag_mmu_wait_start` below never fires and `ag_wait_r` never sets.
    //-------------------------------------------------------------------------
    reg ag_wait_r;
    // M4 Task 5 fix (closes the entry-cycle gap): the RAW "an LSU op sits
    // in EX1 right now" test, with `ag_valid`'s full non-circular term set
    // applied to `idu_lsu_ex1_raw_vld` INSTEAD of `idu_lsu_ex1_sel` --
    // `ag_valid = idu_lsu_ex1_sel = idu_lsu_ex1_raw_vld && !lsu_idu_full`,
    // and every term of `lsu_idu_full` other than its own `ag_wait_r`
    // OR-terms is repeated here (state/clean_active/stb_full+needs_slot/
    // rf_port_busy/lfb_cmplt_fire/ptw_sv_busy -- all forward-referenced,
    // same style as `ag_issue_ready` below). Using `idu_lsu_ex1_raw_vld`
    // directly (instead of gating through `lsu_idu_full`) breaks what
    // would otherwise be a comb loop: folding the entry-detection term
    // straight into `lsu_idu_full` (see that assign) would make
    // `lsu_idu_full` depend on `ag_mmu_wait_start` depend on
    // `ag_issue_ready` depend on `ag_valid` depend on `lsu_idu_full`.
    // `ag_raw_ready` has no such dependency: none of its terms touch
    // `lsu_idu_full`/`ag_valid`.
    wire ag_raw_ready = idu_lsu_ex1_raw_vld && (state == ST_IDLE)
                       && !clean_active && !(stb_full && ag_needs_slot_c)
                       && !rf_port_busy && !lfb_cmplt_fire && !ptw_sv_busy;
    // "would issue but for the MMU" -- every OTHER admission gate issue_real
    // already applies (state/clean/rf_port/lfb_cmplt/ptw_sv, the last two
    // declared ahead in SECTION PTW SERVANT, same forward-reference style),
    // OR'ing in ag_wait_r itself so a parked op keeps re-checking every cycle.
    // Uses `ag_raw_ready`, not `ag_valid` -- see that wire's comment: this
    // makes `ag_mmu_wait_start` below fully acyclic (no term touches
    // `lsu_idu_full`), which is what lets `lsu_idu_full` fold in the
    // entry-detection term below without a comb loop. Behavior-identical
    // to the old `(ag_valid || ag_wait_r)` form: `ag_valid` is exactly
    // `ag_raw_ready && !ag_wait_r`, so `(ag_valid || ag_wait_r) ==
    // (ag_raw_ready || ag_wait_r)`.
    wire ag_issue_ready = (state == ST_IDLE) && (ag_raw_ready || ag_wait_r)
                         && !clean_active && !rf_port_busy && !lfb_cmplt_fire
                         && !ptw_sv_busy;
    wire ag_mmu_wait_start = ag_issue_ready && !mmu_lsu_pa_vld;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ag_wait_r <= 1'b0;
        else if (rtu_yy_xx_flush_fe)
            ag_wait_r <= 1'b0;   // D1: the parked op is squashed outright --
                                 // it never touched the array/STB (still
                                 // ST_IDLE), so there is nothing to unwind.
        else if (ag_mmu_wait_start)
            ag_wait_r <= 1'b1;
        else if (ag_issue_ready && mmu_lsu_pa_vld)
            ag_wait_r <= 1'b0;   // issuing (or trapping-at-issue) NOW
    end

    // The actual buffer (donor aq_lsu_ag.v:323 `ag_req_buffer`): IDU does
    // NOT hold EX1 across the wait (see SECTION AG's `ag_*_eff` comment), so
    // the live idu_lsu_ex1_* wires drift to a DIFFERENT instruction while
    // `ag_wait_r` is set. Snapshot every field this port's own issue/DC-latch
    // logic still needs once the walk resolves -- ONLY on the entry edge
    // (`ag_mmu_wait_start` while `ag_wait_r` is still 0 this cycle): since
    // `ag_wait_r` is itself one of `ag_issue_ready`'s OR-terms,
    // `ag_mmu_wait_start` stays asserted on EVERY cycle of a multi-cycle
    // wait, not just the transition into it -- gating the capture on the
    // bare level (as opposed to the edge) re-snapshots the live (and by
    // then drifted) idu_lsu_ex1_* wires every one of those cycles instead
    // of latching once, corrupting the buffered VA/func mid-walk. No reset
    // needed: dead whenever ag_wait_r=0.
    reg [63:0]              ag_wait_addr_r;
    reg [FUNC_WIDTH-1:0]    ag_wait_func_r;
    reg [63:0]              ag_wait_src2_data_r;
    reg [GPR_IDX_WIDTH-1:0] ag_wait_dst0_reg_r;
    reg                     ag_wait_inst_len_r;
    always @(posedge clk) begin
        if (ag_mmu_wait_start && !ag_wait_r) begin
            ag_wait_addr_r      <= ag_addr_live;
            ag_wait_func_r      <= idu_lsu_ex1_func;
            ag_wait_src2_data_r <= idu_lsu_ex1_src2_data;
            ag_wait_dst0_reg_r  <= idu_lsu_ex1_dst0_reg;
            ag_wait_inst_len_r  <= idu_lsu_ex1_inst_len;
        end
    end

    // Cancel this port's in-flight walk on the same broadcast that clears
    // ag_wait_r -- MMU.v's SECTION 6.7 VPN+type-match fault-delivery gate
    // already drops a stale fault/refill answer once lsu_mmu_va_vld falls,
    // so `lsu_mmu_abort` only needs to say "this port is being flushed",
    // not track the walk's own state.
    assign lsu_mmu_abort = rtu_yy_xx_flush_fe;

    // Full 40-bit PA: MMU's translated page number + AG's own (untranslated)
    // page offset (contract 2; identity-mapped in M2, but this module does
    // not special-case that -- M4's real MMU slots in unchanged).
    wire [PC_WIDTH-1:0] ag_pa = {mmu_lsu_pa[MMU_PA_WIDTH-1:0], ag_addr[11:0]};
    wire [DCACHE_TAG_WIDTH-1:0] ag_dc_tag   = ag_pa[39:13];
    wire [DCACHE_INDEX_W-1:0]   ag_dc_index = ag_pa[12:6];
    wire [2:0]                  ag_dw_off   = ag_pa[5:3];
    wire [2:0]                  ag_byte_off = ag_pa[2:0];

    reg [7:0] ag_byte_mask_raw;
    always @* begin
        case (ag_size)
            2'b00:   ag_byte_mask_raw = 8'b0000_0001;
            2'b01:   ag_byte_mask_raw = 8'b0000_0011;
            2'b10:   ag_byte_mask_raw = 8'b0000_1111;
            default: ag_byte_mask_raw = 8'b1111_1111;
        endcase
    end
    wire [7:0]  ag_byte_mask = ag_byte_mask_raw << ag_byte_off;
    wire [63:0] ag_store_data_positioned = ag_src2_data_eff << ({61'b0, ag_byte_off} * 8);

    //-------------------------------------------------------------------------
    // BYTE-MASK EXPANSION (LSU note A4/A5 -- shared by store-into-STB merge
    // and STB-forward-into-load merge).
    //-------------------------------------------------------------------------
    function automatic [63:0] expand_byte_mask(input [7:0] bm);
        expand_byte_mask = {{8{bm[7]}}, {8{bm[6]}}, {8{bm[5]}}, {8{bm[4]}},
                             {8{bm[3]}}, {8{bm[2]}}, {8{bm[1]}}, {8{bm[0]}}};
    endfunction

    //-------------------------------------------------------------------------
    // SECTION DC control FSM (state) -- see header.
    //-------------------------------------------------------------------------
    localparam [1:0] ST_IDLE  = 2'b00;
    localparam [1:0] ST_DCS   = 2'b01;
    localparam [1:0] ST_FRZ   = 2'b10;
    localparam [1:0] ST_REPLY = 2'b11;

    reg [1:0] state /* verilator public */;

    // AG-latch: captured every time IDLE accepts something (a real
    // instruction or an STB drain), read uniformly by DCS/FRZ/REPLY.
    reg        dc_is_store_r /* verilator public */, dc_sign_ext_r;
    reg        dc_plain_ld_r;   // M3b: this op is a plain load (LFB-deferrable)
    reg [1:0]  dc_size_r;
    reg [63:0] dc_addr_r /* verilator public */;
    reg [2:0]  dc_dw_off_r, dc_byte_off_r;
    reg [7:0]  dc_byte_mask_r;
    reg [63:0] dc_store_data_r;
    reg [DCACHE_INDEX_W-1:0]   dc_index_r /* verilator public */;
    reg [DCACHE_TAG_WIDTH-1:0] dc_tag_r /* verilator public */;
    reg        dc_ca_r;             // "this transaction touches the array"
    reg        dc_misalign_r;
    reg [GPR_IDX_WIDTH-1:0] dc_dst0_reg_r;
    // Task 7.3: length of the in-flight LSU instruction, latched on issue
    // (1=32b,0=16b RVC); reported as lsu_rtu_ex1_inst_len at completion.
    reg                     dc_inst_len_r;
    reg  [15:0]             dc_pc_r;        // M3b Task D: PFB trainer PC tag
    reg        dc_is_drain_r /* verilator public */;
    reg [1:0]  dc_drain_idx_r;
    reg        dc_wa_r;
    reg        dc_touched_array_r;   // did the IDLE cycle that latched this
                                      // transaction actually issue a DCache.v
                                      // request (vs. misalign/uncached/direct-drain,
                                      // which issue nothing and have no response
                                      // to wait for in DCS)

    // DCS-latch: captured at the DCS->{FRZ|REPLY} transition.
    reg [WAYS-1:0] dc_hit_way_r;
    reg            dc_hit_r /* verilator public */;
    reg [WAYS-1:0] dc_way_vld_r, dc_way_dirty_r;
    reg [511:0]    dc_rdata_r;
    reg            frz_is_direct_r /* verilator public */;
    // M3b: set when a SECOND op misses ST_DCS while the LFB already holds a
    // deferred load (single-entry LFB -- the slot is occupied). DCache.v's
    // dc_resp_vld is a single unscoped pulse, not tagged per-requester
    // (DCache.v:115-144), so this op's hit/miss decision must NOT be
    // re-derived from a later u_dc_resp_vld pulse (that pulse could belong
    // to the background refill's own vpeek/commit traffic instead). Park
    // here using the already-latched dc_hit_r/dc_way_vld_r/dc_way_dirty_r
    // and retry once the background refill's miss_state frees up.
    reg            dc_wait_lfb_r;

    //-------------------------------------------------------------------------
    // SECTION LR/SC (M3 Task 1) -- 1-entry load-reserved buffer for LR.W / SC.W
    //-------------------------------------------------------------------------
    reg [55:0] lr_addr_r;         // last LR physical address (PA[55:0])
    reg [1:0]  lr_size_r;         // reservation access size (donor lm_size,
                                  // aq_lsu_lm.v:159-161 -- SC must match it)
    reg        lr_valid_r;        // set when LR completes, cleared by SC or intervening access
    reg        dc_is_lr_r;        // in-flight transaction is an LR (issue latch)

    // Exclusion detection on any store/load while lr_valid_r is held.
    // (lr_valid_r itself has exactly ONE writer: the LR-buffer always block
    // below -- a second clear-only block here raced it and was removed.)
    //
    // lr_txn_event qualifies to a REAL transaction event: in ST_IDLE the
    // DCache response bus HOLDS the previous transaction's hit-way, so an
    // unqualified dc_hit_c/dc_is_store_r reads stale and cleared a fresh
    // reservation on any idle cycle (rv64ua-p-lrsc hung in its retry loop).
    // Cached transactions fire on their DCS response; UNCACHED transactions
    // never get one (fire on the DCS pass itself). M3 audit coverage: an
    // AMO is a store to the reservation even though load-like on this pipe
    // (the donor clears its lock monitor on SC *and AMO*, aq_lsu_dc.v:1620)
    // -- a MISSED AMO to the reserved line and uncached stores/AMOs used to
    // slip through; and any completing LOAD can evict the reserved line via
    // refill, so hit-or-miss both clear (conservative, spec-legal).
    //
    // An LR is EXCEPTED: LR-over-LR re-keys the reservation (donor lm_set
    // overwrites addr/size in EXCL state, aq_lsu_lm.v:145-157). Without the
    // exception, LR#2's own DCS response cleared lr_addr_set before the
    // completion set-term could fire, leaving NO reservation (LR;LR;SC
    // failed here; succeeds on the donor).
    wire lr_txn_event        = (state == ST_DCS) && !dc_misalign_r
                             && (u_dc_resp_vld || !dc_touched_array_r);
    wire lr_exclude_on_store = lr_valid_r && lr_txn_event
                               && (dc_is_store_r || amo_active);
    wire lr_exclude_on_load  = lr_valid_r && lr_txn_event
                               && !dc_is_store_r && !amo_active && !dc_is_lr_r;

    //-------------------------------------------------------------------------
    // SECTION STB (LSU note A4) -- 4 entries, one per distinct 8-byte-
    // aligned doubleword (contract 3's trap-on-misalign guarantee means an
    // aligned access of size <=8B never straddles a doubleword, so a single
    // per-doubleword entry with an 8-bit byte-valid mask is exact, not an
    // approximation). A second store to an ALREADY-resident doubleword
    // MERGES into that entry (byte-lane mux, confirmed against
    // aq_lsu_stb_entry.v's own merge formula by a Task 6 research pass)
    // instead of allocating a second entry -- this is why load-forward
    // never needs a cross-entry age-ordered merge: at most one entry ever
    // exists per doubleword.
    //-------------------------------------------------------------------------
    reg        stb_vld      [0:3] /* verilator public */;
    reg [63:0] stb_addr     [0:3] /* verilator public */;
    reg [DCACHE_INDEX_W-1:0]   stb_index [0:3];
    reg [DCACHE_TAG_WIDTH-1:0] stb_tag   [0:3];
    reg [2:0]  stb_dw_off   [0:3];
    reg [7:0]  stb_byte_vld [0:3] /* verilator public */;
    reg [63:0] stb_data     [0:3] /* verilator public */;
    reg [WAYS-1:0] stb_way  [0:3];
    reg        stb_was_hit  [0:3] /* verilator public */;
    // Store size (0=B,1=H,2=W,3=D) per STB entry. Needed so a drained entry
    // advertises the correct AXI awsize for its direct write-back. The donor
    // C906 STB (aq_lsu_stb.v:895-901) carries stb_entryN_size per entry and
    // drives stb_awsize from it; our `issue_drain` path does not relatch
    // dc_size_r, so the store's own size would be STALE by drain time.
    reg [2:0]  stb_size     [0:3] /* verilator public */;

    wire [60:0] ag_dword       = ag_addr[63:3];
    wire        stb_m0_ag = stb_vld[0] && (stb_addr[0][63:3] == ag_dword);
    wire        stb_m1_ag = stb_vld[1] && (stb_addr[1][63:3] == ag_dword);
    wire        stb_m2_ag = stb_vld[2] && (stb_addr[2][63:3] == ag_dword);
    wire        stb_m3_ag = stb_vld[3] && (stb_addr[3][63:3] == ag_dword);

    // KNOWN, DOCUMENTED RISK (flagged, not silently modeled as fully
    // accurate -- matches this project's own established discipline for
    // carried-forward gaps): an STB entry records the WAY it hit at
    // creation time; if a LATER, different-address miss's victim-select
    // picks that SAME way before this entry drains, the pending store data
    // would be overwritten in the array without ever being written back.
    // The real donor defends against exactly this with `vb_dc_hit_idx`-
    // style index-level hazard checks (LSU note cross-cutting #6) that this
    // minimal M2 clone does not build. Mitigated in practice (not
    // eliminated) by draining STB opportunistically on every LSU-idle
    // cycle (below) rather than deferring it, keeping the exposure window
    // small; a genuine correctness gap for a future milestone's higher-
    // throughput/back-to-back-miss test programs to re-audit.
    //-------------------------------------------------------------------------
    // SECTION drain-vs-issue arbitration (from IDLE only).
    //-------------------------------------------------------------------------
    wire any_stb_vld = stb_vld[0] || stb_vld[1] || stb_vld[2] || stb_vld[3];
    // M3b/M4 audit fix -- an STB entry whose dword matches a still-in-flight
    // LFB entry (lfb_state != E_IDLE, i.e. not yet retired via
    // lfb_cmplt_fire) must not opportunistically drain here. That deferred
    // load's STB-forward merge (lfb_stb_m0..3, SECTION LFB) is evaluated
    // live against the CURRENT stb_vld/stb_addr every cycle but only
    // SAMPLED once, at lfb_cmplt_fire; the refill data it merges against
    // (lfb_rdata_r) was captured earlier, at the refill's own completion.
    // If this entry drains away in between, the forward is silently lost:
    // the load completes from the stale pre-store refill snapshot with no
    // STB entry left to correct it (mmu_pmpstore.elf TESTNUM 2: a store
    // immediately followed by a load to the same freshly-mapped, still-
    // uncached line -- the load misses, defers into the LFB, and the STB
    // entry drains out from under it before the refill lands). lfb_addr[]
    // and stb_addr[] are both VA (dc_addr_r <= ag_addr for both a real
    // issue and a drain), so the dword compare here matches the one the
    // LFB's own forward check already uses. lfb_state/LFB_DEPTH/E_IDLE are
    // forward-referenced (SECTION LFB, below), matching this file's existing
    // forward-reference style (rf_port_busy/lfb_cmplt_fire/ptw_sv_busy, etc).
    integer stb_lfb_chk_i;
    reg [3:0] stb_lfb_block_c;
    always @* begin
        for (stb_lfb_chk_i = 0; stb_lfb_chk_i < 4; stb_lfb_chk_i = stb_lfb_chk_i + 1) begin
            stb_lfb_block_c[stb_lfb_chk_i] = 1'b0;
            for (lfb_i = 0; lfb_i < LFB_DEPTH; lfb_i = lfb_i + 1) begin
                if ((lfb_state[lfb_i] != E_IDLE) && stb_vld[stb_lfb_chk_i]
                    && (stb_addr[stb_lfb_chk_i][63:3] == lfb_addr[lfb_i][63:3]))
                    stb_lfb_block_c[stb_lfb_chk_i] = 1'b1;
            end
        end
    end
    wire [3:0] stb_drainable = {stb_vld[3] && !stb_lfb_block_c[3],
                                stb_vld[2] && !stb_lfb_block_c[2],
                                stb_vld[1] && !stb_lfb_block_c[1],
                                stb_vld[0] && !stb_lfb_block_c[0]};
    wire any_stb_drainable = |stb_drainable;
    wire [1:0] drain_pick = stb_drainable[0] ? 2'd0 : stb_drainable[1] ? 2'd1
                            : stb_drainable[2] ? 2'd2 : 2'd3;
    // M3 audit fix -- STB-full admission control. Store/SC/AMO entries are
    // created at REPLY; if the STB is full and the completion cannot merge,
    // REPLY would stall waiting for a free slot -- but drains start only
    // from ST_IDLE, so the FSM would never reach IDLE again: DEADLOCK (a
    // 5th consecutive distinct-dword store hung the machine; the donor
    // retries in DCS on stb-full instead, aq_lsu_dc.v:1697-1708 /
    // aq_lsu_stb.v:538-549).
    //
    // Mechanism: `lsu_idu_full` (below) gains a combinational term
    // `stb_full && ag_needs_slot_c`. It is built ONLY from registers
    // (ex1_func/data reach this module as plain wires off IDU's EX1 flops;
    // stb_vld/stb_addr are flops here), so there is no loop through
    // idu_lsu_ex1_sel even though that sel is gated by lsu_idu_full. When
    // the term asserts: idu_lsu_ex1_sel drops (ag_valid=0), so issue_real
    // is 0 and drain_want's !ag_valid term is 1 -- drains get the FSM and
    // run until a slot frees, at which point the term drops, sel returns,
    // and the held op issues. The IDU side holds EX1 exactly as it does
    // for any lsu_idu_full cycle (ctrl_ex1_eu_full keys off ex1_eu_r[LSU]
    // && lsu_idu_full), and a mid-hold branch cancel is harmless: ex1_eu_r
    // clears, masking the full at the IDU, and the term dies with stb_full.
    //
    // Single-FSM guarantee: between admission and REPLY no other
    // transaction can CREATE an entry and no drain runs, so the slot/match
    // observed at admission still holds at REPLY (reply_can_complete
    // remains as defense-in-depth). ag_needs_slot_c deliberately reads the
    // ungated EX1 fields: when EX1 holds no LSU op they may be stale, but
    // then issue_real is 0 anyway (ag_valid=0) and the IDU masks the full.
    wire stb_full          = !stb_any_free;
    wire ag_stb_match_c    = stb_m0_ag || stb_m1_ag || stb_m2_ag || stb_m3_ag;
    wire ag_needs_slot_c   = (ag_is_store || amo_is_amo
                              || idu_lsu_ex1_func == LSU_FUNC_SC_W
                              || idu_lsu_ex1_func == LSU_FUNC_SC_D)
                             && !ag_misalign && !ag_stb_match_c;
    // M4 LFB/STB-race fix: gate on any_stb_drainable (not any_stb_vld) --
    // an entry whose dword collides with a still-in-flight LFB entry
    // (stb_lfb_block_c, SECTION drain-vs-issue arbitration above) must not
    // be opportunistically drained, so drain_want itself must not fire on
    // the strength of ONLY such entries being valid: any_stb_vld=1 with
    // any_stb_drainable=0 would still set issue_drain (line ~1010) and let
    // drain_pick's ternary fall through to its 2'd3 default, draining a
    // slot that may not even be stb_vld[3].
    wire drain_want = !ag_valid && any_stb_drainable && (state == ST_IDLE);

    //-------------------------------------------------------------------------
    // SECTION DCache.v instance -- see DCache.v's own header for why it is
    // instantiated here rather than at RVProc.v's top level.
    //-------------------------------------------------------------------------
    wire                        u_dc_req_vld;
    wire [DCACHE_INDEX_W-1:0]   u_dc_req_index;
    wire [DCACHE_TAG_WIDTH-1:0] u_dc_req_tag;
    wire                        u_dc_req_wr;
    wire [WAYS-1:0]             u_dc_req_way_sel;
    wire [511:0]                u_dc_req_wdata;
    wire [63:0]                 u_dc_req_wstrb;
    wire                        u_dc_req_dirty_set;
    wire                        u_dc_req_alloc;

    wire                        u_dc_resp_vld;
    wire [WAYS-1:0]             u_dc_resp_hit_way;
    wire [511:0]                u_dc_resp_rdata;
    wire [WAYS-1:0]             u_dc_resp_way_vld;
    wire [WAYS-1:0]             u_dc_resp_way_dirty;
    wire [DCACHE_TAG_WIDTH-1:0] u_dc_resp_victim_tag;
    wire                        u_dc_inv_done;

    // dc_inv_* is driven by the FENCE.I clean walk (SECTION CLEAN below);
    // dcache_tb.cpp exercises DCache.v's own invalidate mechanism directly,
    // standalone, per plan 6.4.

    DCache u_dcache (
        .clk(clk), .rst_n(rst_n),
        .dc_req_vld(u_dc_req_vld), .dc_req_index(u_dc_req_index), .dc_req_tag(u_dc_req_tag),
        .dc_req_wr(u_dc_req_wr), .dc_req_way_sel(u_dc_req_way_sel), .dc_req_wdata(u_dc_req_wdata),
        .dc_req_wstrb(u_dc_req_wstrb), .dc_req_dirty_set(u_dc_req_dirty_set), .dc_req_alloc(u_dc_req_alloc),
        .dc_resp_vld(u_dc_resp_vld), .dc_resp_hit_way(u_dc_resp_hit_way), .dc_resp_rdata(u_dc_resp_rdata),
        .dc_resp_way_vld(u_dc_resp_way_vld), .dc_resp_way_dirty(u_dc_resp_way_dirty),
        .dc_resp_victim_tag(u_dc_resp_victim_tag),
        .dc_inv_vld(clean_inv_fire), .dc_inv_index(clean_set), .dc_inv_way_sel(clean_way_oh),
        .dc_inv_done(u_dc_inv_done)
    );

    //-------------------------------------------------------------------------
    // SECTION PTW SERVANT (M4 Task 5, D3, S4/S12) -- the MMU has no memory
    // port of its own; every hardware page-table read arrives on group 5's
    // frozen six-wire channel (mmu_lsu_data_req/_addr/_size) and leaves on
    // its answer side (lsu_mmu_data/_vld/_bus_error). Donor aq_lsu_mcic.v is
    // a 310-line array/bus server with its own gated clock and a T-Head
    // diagnostic path this project already dropped (D-M4-5); this section
    // keeps the PROPERTY the design doc calls load-bearing (D3), not the
    // module:
    //
    //   *** PROBE THE D-CACHE FIRST, AND THAT IS A CORRECTNESS RULE. ***
    //   Page tables are written by CACHEABLE stores (the v-env's software
    //   A/D update, rv64si-p-dirty's own fault handler does `sw` to set
    //   A/D then sfence.vma + retry), and this D-cache is WRITE-BACK
    //   (contract 6) -- a PTE line can be DIRTY in the array at walk time.
    //   A walker that only reads the bus reads a STALE PTE and never sees
    //   the just-set A/D bit, looping the fault forever. Probe first, fall
    //   through to a direct AXI read only on an array miss.
    //
    //   *** THE SERVANT NEVER RUNS CONCURRENTLY WITH ANYTHING ELSE THAT
    //   TOUCHES THE ARRAY OR THE AXI READ CHANNEL -- BY CONSTRUCTION. ***
    //   `ptw_sv_idle_free` gates the ONE cycle a new probe may start on
    //   every condition the main issue mux (SECTION IDLE-cycle issue mux,
    //   below) already uses to know the array/AXI-read port is free:
    //   state==ST_IDLE (excludes ST_FRZ's blocking-miss use), !any_stb_vld
    //   (a drained STB makes the array-only probe coherent without STB
    //   forwarding -- D3's own text), !clean_active (FENCE.I's walk),
    //   !rf_port_busy (a background LFB refill's array-port phases), and
    //   !lfb_any_vld (stronger than rf_port_busy alone: excludes a
    //   background refill's OWN AXI-read phase too, MS_REFILL_READ/
    //   MS_DIRECT_READ, which rf_port_busy deliberately does NOT cover --
    //   hit-under-miss lets a new array lookup proceed there, but this
    //   servant also needs the AXI read channel on a probe miss, so it
    //   must wait out the WHOLE background transaction, not just its
    //   array-port phases). The reverse direction is closed two ways: (1)
    //   `issue_real`/`issue_drain` (below) both gate on `!ptw_sv_busy`, so
    //   a genuinely new instruction cannot steal the array/AXI channel out
    //   from under an in-flight walk (the walk just holds `state`==ST_IDLE
    //   throughout, since it never touches the main FSM's own `state`);
    //   (2) the FENCE.I clean walk's own CL_IDLE arm (SECTION CLEAN,
    //   below) additionally requires `ptw_sv_state == PTW_SV_IDLE` before
    //   starting, closing the one window `clean_active` alone could not --
    //   clean_active only gates ptw_sv at the MOMENT of a NEW probe, not
    //   for the rest of an already-in-flight walk's own multi-cycle
    //   ANSW/BUS span, during which `state` is still ST_IDLE.
    //
    //   *** SINGLE OUTSTANDING, ENFORCED HERE. *** The walker itself issues
    //   one PT read at a time (MMU.v SECTION 6's own "single walk at a
    //   time"), but this servant does not trust that: a new probe is only
    //   accepted from PTW_SV_IDLE, so a second `mmu_lsu_data_req` raised
    //   mid-walk changes nothing.
    //
    //   *** RETURN TO IDLE IS AN ACKNOWLEDGE, NOT A COUNT. *** MMU.v holds
    //   `mmu_lsu_data_req` up for the whole of its own *_DATA state and
    //   drops it the cycle AFTER `lsu_mmu_data_vld` pulses (MMU.v's own
    //   `ptw_data_req`); PTW_SV_DONE waits for that fall before re-arming,
    //   so a level held one cycle longer cannot make this servant answer
    //   the same request twice.
    //-------------------------------------------------------------------------
    localparam [1:0] PTW_SV_IDLE = 2'd0,   // no walk request accepted
                     PTW_SV_ANSW  = 2'd1,   // the array is answering THIS cycle
                     PTW_SV_BUS   = 2'd2,   // array missed -- direct AXI read
                     PTW_SV_DONE  = 2'd3;   // answered -- waiting for the req to fall
    reg [1:0]           ptw_sv_state;
    reg [PC_WIDTH-1:0]  ptw_sv_addr_r;
    reg                 ptw_sv_ar_sent;

    wire ptw_sv_busy = (ptw_sv_state != PTW_SV_IDLE);

    // any_stb_vld/clean_active/rf_port_busy/lfb_any_vld/issue_real/
    // issue_drain are all declared further down this file -- forward
    // reference, same style this file already uses throughout (e.g.
    // ag_needs_slot_c in lsu_idu_full's own definition).
    wire ptw_sv_idle_free = (state == ST_IDLE) && !any_stb_vld && !clean_active
                           && !rf_port_busy && !lfb_any_vld
                           && !issue_real && !issue_drain;
    wire ptw_sv_probe_fire = (ptw_sv_state == PTW_SV_IDLE) && mmu_lsu_data_req
                            && ptw_sv_idle_free;

    // The array probe itself reads mmu_lsu_data_req_addr directly (not yet
    // latched into ptw_sv_addr_r -- that latch happens on the SAME edge);
    // way_sel=0 is DCache.v's own "ordinary hit-check" read mode (its
    // header's "way-select dual purpose" contract).
    wire [DCACHE_TAG_WIDTH-1:0] ptw_sv_req_tag   = mmu_lsu_data_req_addr[39:13];
    wire [DCACHE_INDEX_W-1:0]   ptw_sv_req_index = mmu_lsu_data_req_addr[12:6];
    // Once latched, the same fields read off ptw_sv_addr_r for the (unused
    // this cycle, but kept symmetric) tag/index and for the doubleword
    // select below.
    wire [2:0] ptw_sv_dwoff = ptw_sv_addr_r[5:3];
    // mmu_lsu_data_req_size (donor: constant 1, "every PT read is 8B",
    // MMU.v:618) carries no information THIS servant needs -- every read
    // always fetches the full 64B line and picks the doubleword out by
    // address (ptw_sv_dwoff), the same "consumer picks out the bytes it
    // needs" simplification every other direct-read use in this file
    // already takes. Received, not silently dropped.
    wire _mmu_lsu_data_req_size_unused = mmu_lsu_data_req_size;

    wire ptw_sv_axi_active = (ptw_sv_state == PTW_SV_BUS);
    // Direct AXI reads always fetch the full 64B LINE (arsize=6, matching
    // every other read use in this file -- "the consumer picks out the
    // doubleword it actually needs"), aligned DOWN from the (8B-aligned,
    // not necessarily 64B-aligned) PTE address MMU.v's walker built.
    wire [ADDR_WIDTH-1:0] ptw_sv_line_addr =
        {{(ADDR_WIDTH-PC_WIDTH){1'b0}}, ptw_sv_addr_r[PC_WIDTH-1:6], 6'b0};

    wire ptw_sv_hit_answer = (ptw_sv_state == PTW_SV_ANSW) && u_dc_resp_vld
                            && (|u_dc_resp_hit_way);
    wire ptw_sv_miss_c     = (ptw_sv_state == PTW_SV_ANSW) && u_dc_resp_vld
                            && !(|u_dc_resp_hit_way);
    wire ptw_sv_bus_answer = ptw_sv_axi_active && axi_d_rvalid && axi_d_rready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ptw_sv_state   <= PTW_SV_IDLE;
            ptw_sv_addr_r  <= {PC_WIDTH{1'b0}};
            ptw_sv_ar_sent <= 1'b0;
        end else begin
            case (ptw_sv_state)
                PTW_SV_IDLE: if (ptw_sv_probe_fire) begin
                    ptw_sv_addr_r <= mmu_lsu_data_req_addr;
                    ptw_sv_state  <= PTW_SV_ANSW;
                end
                PTW_SV_ANSW: begin
                    if (ptw_sv_hit_answer)
                        ptw_sv_state <= PTW_SV_DONE;
                    else if (ptw_sv_miss_c) begin
                        ptw_sv_ar_sent <= 1'b0;
                        ptw_sv_state   <= PTW_SV_BUS;
                    end
                end
                PTW_SV_BUS: begin
                    if (axi_d_arvalid && axi_d_arready && ptw_sv_axi_active)
                        ptw_sv_ar_sent <= 1'b1;
                    if (ptw_sv_bus_answer) ptw_sv_state <= PTW_SV_DONE;
                end
                PTW_SV_DONE: if (!mmu_lsu_data_req) ptw_sv_state <= PTW_SV_IDLE;
                default: ptw_sv_state <= PTW_SV_IDLE;
            endcase
        end
    end

    // The answer, read combinationally off whichever source resolved it --
    // ONE-CYCLE pulses (not held through PTW_SV_DONE), matching MMU.v's own
    // consumption (`if (lsu_mmu_data_vld) pte_flop <= lsu_mmu_data`, once).
    wire [63:0] ptw_sv_hit_dword = u_dc_resp_rdata[{ptw_sv_dwoff, 6'b0} +: 64];
    wire [63:0] ptw_sv_bus_dword = axi_d_rdata[{ptw_sv_dwoff, 6'b0} +: 64];

    assign lsu_mmu_data_vld  = ptw_sv_hit_answer || ptw_sv_bus_answer;
    assign lsu_mmu_data      = ptw_sv_hit_answer ? ptw_sv_hit_dword : ptw_sv_bus_dword;
    assign lsu_mmu_bus_error = ptw_sv_bus_answer && (axi_d_rresp != 2'b00);

    //-------------------------------------------------------------------------
    // SECTION IDLE-cycle issue mux: a real instruction (ag_valid) takes
    // priority over an opportunistic drain (drain_want), matching this
    // task's own "STB drains unconditionally... but must not starve the
    // main pipe" framing.
    //-------------------------------------------------------------------------
    // M3b: a new op must not collide with a background LFB refill's D-cache
    // port phase (rf_port_busy), and the deferred-load completion
    // (lfb_cmplt_fire) takes the IDLE slot before a fresh issue. A drain
    // yields only while the background refill OWNS the D-cache port
    // (rf_port_busy: vpeek/commit); it must NOT be blocked for the whole time
    // a load is deferred, else the STB can never drain and a full STB
    // deadlocks (rv64ui-p-ld_st). Cacheable drains use the state FSM
    // (ST_DCS->ST_REPLY), not the FRZ sub-FSM, so they interleave safely
    // with the background refill's AXI phases.
    // M4 Task 5 (D1): issue_real now additionally requires mmu_lsu_pa_vld --
    // `ag_issue_ready` (SECTION AG WAIT-STATE above) is every OTHER gate
    // this wire used to spell out directly PLUS the `ag_wait_r` OR-term, so
    // a parked op keeps re-evaluating every cycle. On the OFF path
    // (mach_path, MMU.v SECTION 7) mmu_lsu_pa_vld is combinationally 1
    // every cycle, so issue_real is bit-identical to its pre-Task-5 form
    // there -- ag_wait_r never sets, this AND-term is always satisfied the
    // same cycle ag_issue_ready is.
    wire issue_real  = ag_issue_ready && mmu_lsu_pa_vld;
    wire issue_drain = (state == ST_IDLE) && !ag_valid && drain_want && !clean_active
                       && !rf_port_busy && !lfb_cmplt_fire && !ptw_sv_busy;
    // M4 misalign fix (contract 3 + donor aq_lsu_ag.v, which raises misalign
    // at AG, NOT at the reply): a misaligned access must trap at its ISSUE
    // cycle. Raising it late at ST_REPLY let the front end keep dispatching
    // the following instructions before the trap landed (they executed and
    // the handler returned into an already-advanced stream -> the
    // rv64mi-p-{ld,lh}-misaligned / rv64ui-p-ma_data hang). Trap-at-issue
    // means the flush kills every younger instruction. misalign_issue is the
    // AG-stage exception pulse; the access never enters the DC FSM.
    wire misalign_issue = issue_real && ag_misalign;

    // M4 Task 5 (S12): a DTLB PAGE_FAULT/ACCESS_FAULT rides the SAME cycle
    // pa_vld=1 arrives (MMU.v's own contract: "Faults arrive WITH pa_vld").
    // Trap-at-issue, exactly like misalign_issue: the access never enters
    // the DC FSM, so no array/STB state is ever created for it.
    wire mmu_fault_issue = issue_real && !ag_misalign
                          && (mmu_lsu_page_fault || mmu_lsu_access_fault);

    wire touches_array = issue_real  ? (mmu_lsu_ca && !ag_misalign && !mmu_fault_issue)
                        : issue_drain ? stb_was_hit[drain_pick]
                        : 1'b0;

    //-------------------------------------------------------------------------
    // SECTION main FSM sequencing + AG/DCS latches.
    //-------------------------------------------------------------------------
    wire dc_hit_c        = |u_dc_resp_hit_way;
    wire store_wa_miss_c = dc_is_store_r && !dc_hit_c && !dc_wa_eff;   // contract 6: wa=0 default;
    // M3b Task E: dc_wa_eff folds in the AMR's live write-allocate disable.
    // M3b dc_wait_lfb_r retry: same decision, but off the LATCHED dc_hit_r
    // (this op's own, already-resolved response) instead of the live
    // dc_hit_c wire, which by retry time may reflect unrelated background-
    // refill traffic on the shared (unscoped) DCache.v response.
    wire store_wa_miss_r = dc_is_store_r && !dc_hit_r && !dc_wa_eff;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            dc_is_store_r   <= 1'b0;
            dc_plain_ld_r   <= 1'b0;
            dc_sign_ext_r   <= 1'b0;
            dc_size_r       <= 2'b0;
            dc_addr_r       <= 64'd0;
            dc_dw_off_r     <= 3'd0;
            dc_byte_off_r   <= 3'd0;
            dc_byte_mask_r  <= 8'd0;
            dc_store_data_r <= 64'd0;
            dc_index_r      <= {DCACHE_INDEX_W{1'b0}};
            dc_tag_r        <= {DCACHE_TAG_WIDTH{1'b0}};
            dc_ca_r         <= 1'b0;
            dc_misalign_r   <= 1'b0;
            dc_dst0_reg_r   <= {GPR_IDX_WIDTH{1'b0}};
            dc_inst_len_r   <= 1'b0;
            dc_pc_r         <= 16'd0;
            dc_is_drain_r   <= 1'b0;
            dc_drain_idx_r  <= 2'd0;
            dc_wa_r         <= 1'b0;
            dc_hit_way_r    <= {WAYS{1'b0}};
            dc_hit_r        <= 1'b0;
            dc_way_vld_r    <= {WAYS{1'b0}};
            dc_way_dirty_r  <= {WAYS{1'b0}};
            dc_rdata_r      <= 512'd0;
            frz_is_direct_r <= 1'b0;
            dc_touched_array_r <= 1'b0;
            dc_wait_lfb_r   <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    // M4 Task 5: a DTLB fault (mmu_fault_issue) traps at
                    // issue exactly like a misaligned access -- excluded
                    // here so it never enters ST_DCS/touches the array.
                    if (issue_real && !ag_misalign && !mmu_fault_issue) begin
                        dc_is_store_r   <= ag_is_store;
                        dc_plain_ld_r   <= ag_is_plain_ld;
                        dc_sign_ext_r   <= ag_sign_ext;
                        dc_size_r       <= ag_size;
                        dc_addr_r       <= ag_addr;
                        dc_dw_off_r     <= ag_dw_off;
                        dc_byte_off_r   <= ag_byte_off;
                        dc_byte_mask_r  <= ag_byte_mask;
                        dc_store_data_r <= ag_store_data_positioned;
                        dc_index_r      <= ag_dc_index;
                        dc_tag_r        <= ag_dc_tag;
                        dc_ca_r         <= mmu_lsu_ca;
                        dc_misalign_r   <= ag_misalign;
                        dc_dst0_reg_r   <= ag_dst0_reg_eff;
                        dc_inst_len_r   <= ag_inst_len_eff;
                        dc_pc_r         <= iu_lsu_ex1_cur_pc;   // M3b Task D: PFB PC tag
                        dc_is_drain_r   <= 1'b0;
                        dc_wa_r         <= cp0_lsu_wa;
                        dc_touched_array_r <= touches_array;
                        state <= ST_DCS;
                    end else if (issue_drain) begin
                        dc_is_store_r   <= 1'b1;
                        dc_addr_r       <= stb_addr[drain_pick];
                        dc_index_r      <= stb_index[drain_pick];
                        dc_tag_r        <= stb_tag[drain_pick];
                        dc_ca_r         <= stb_was_hit[drain_pick];
                        dc_misalign_r   <= 1'b0;
                        dc_is_drain_r   <= 1'b1;
                        dc_drain_idx_r  <= drain_pick;
                        dc_touched_array_r <= touches_array;
                        state <= ST_DCS;
                    end
                end
                ST_DCS: begin
                    // M3b: this op already missed once (dc_hit_r latched a
                    // miss) and parked here. On retry, a plain cacheable
                    // load re-tries the DEFER path (lfb_defer_fire's
                    // lfb_retry_defer term, below) since a slot may have
                    // freed or the in-flight line it collided with may have
                    // drained; any other blocking miss kind just proceeds to
                    // ST_FRZ once the LFB is fully drained (!lfb_any_vld) --
                    // matching pre-M3b (single-outstanding) behavior for
                    // that rarer case.
                    if (dc_wait_lfb_r) begin
                        if (dc_plain_ld_r && dc_ca_r) begin
                            if (!lfb_full && !lfb_addr_hit) begin
                                dc_wait_lfb_r <= 1'b0;
                                frz_is_direct_r <= 1'b0;
                                state <= ST_IDLE;   // lfb_defer_fire's lfb_retry_defer fires this cycle
                            end
                            // else: still full or still line-colliding -- stay parked, re-check next cycle
                        end else begin
                            if (!lfb_any_vld) begin
                                dc_wait_lfb_r <= 1'b0;
                                frz_is_direct_r <= !dc_ca_r || store_wa_miss_r;
                                state <= ST_FRZ;
                            end
                        end
                    // Wait for DCache.v's own response (its IDLE->DCS->REPLY
                    // protocol takes a fixed 2 cycles from request to
                    // dc_resp_vld -- this state simply waits however long
                    // that takes, rather than assuming a specific count) --
                    // UNLESS nothing was ever requested (a misaligned
                    // access, an uncached real access, or a direct drain),
                    // in which case there is nothing to wait for at all.
                    end else if (!dc_touched_array_r || u_dc_resp_vld) begin
                        if (dc_touched_array_r) begin
                            dc_hit_way_r   <= u_dc_resp_hit_way;
                            dc_hit_r       <= dc_hit_c;
                            dc_way_vld_r   <= u_dc_resp_way_vld;
                            dc_way_dirty_r <= u_dc_resp_way_dirty;
                            dc_rdata_r     <= u_dc_resp_rdata;
                        end else begin
                            // nothing was requested this transaction
                            // (misalign / uncached / direct-drain) --
                            // explicitly clear rather than let dc_hit_r
                            // retain a STALE value from some earlier,
                            // unrelated transaction that DID touch the
                            // array (a real bug found while writing
                            // lsu_tb.cpp's STB/wa=1-refill tests).
                            dc_hit_way_r   <= {WAYS{1'b0}};
                            dc_hit_r       <= 1'b0;
                        end
                        if (dc_is_drain_r) begin
                            frz_is_direct_r <= !dc_ca_r;
                            state <= dc_ca_r ? ST_REPLY : ST_FRZ;
                        end else if (dc_misalign_r) begin
                            state <= ST_REPLY;
                        end else if (!dc_ca_r && !lfb_any_vld) begin
                            frz_is_direct_r <= 1'b1;
                            state <= ST_FRZ;
                        end else if (store_wa_miss_c && !lfb_any_vld) begin
                            frz_is_direct_r <= 1'b1;
                            state <= ST_FRZ;
                        end else if (dc_hit_c) begin
                            state <= ST_REPLY;
                        end else if (dc_plain_ld_r && dc_ca_r && !lfb_full && !lfb_addr_hit) begin
                            // M3b Task A: defer a cacheable plain-load miss into
                            // the LFB. The refill runs in the background (FRZ
                            // sub-FSM gated by lfb_engine_active) while the main
                            // FSM returns to ST_IDLE, so cache-HIT ops complete
                            // while this miss is in flight (hit-under-miss).
                            frz_is_direct_r <= 1'b0;
                            state <= ST_IDLE;
                        end else if (!lfb_any_vld) begin
                            frz_is_direct_r <= 1'b0;
                            state <= ST_FRZ;
                        end else begin
                            // a miss, but this can't defer right now (LFB full,
                            // or its target line already in flight under
                            // another entry) and the LFB isn't fully drained
                            // either, so a blocking FRZ can't proceed cleanly --
                            // park here (dc_hit_r/dc_way_vld_r/dc_way_dirty_r are
                            // already latched above) and retry via the
                            // dc_wait_lfb_r branch above once the picture changes.
                            dc_wait_lfb_r <= 1'b1;
                        end
                    end
                end
                ST_FRZ: begin
                    if (miss_done) begin
                        dc_rdata_r <= frz_rdata_r;   // single-writer fix: FRZ's
                                                       // own always block never
                                                       // touches dc_rdata_r directly
                        state <= ST_REPLY;
                    end
                end
                ST_REPLY: begin
                    if (reply_can_complete) state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
    // M3 Task 1: LR.W / SC.W foundation
    // Latch the address at issue time since idu_lsu_ex1_func gets cleared after
    reg        lr_addr_set;

    always @(posedge clk) begin
        if (!rst_n) begin
            lr_valid_r <= 1'b0;
            lr_addr_set <= 1'b0;
            lr_size_r <= 2'b0;
            dc_is_lr_r <= 1'b0;
        end else begin
            // Latch the LR address/size on issue_real (IDLE->DCS transition).
            // Use ag_addr (virtual address) for LR/SC comparison. A
            // MISALIGNED LR traps and must install NO reservation (donor
            // gates lm_set on !expt_ack/!expt_exit, aq_lsu_lm.v:129). M4
            // Task 5: a DTLB-faulting LR traps at issue too (mmu_fault_issue)
            // and must install no reservation for the same reason. Must read
            // ag_func_eff, not idu_lsu_ex1_func: issue_real can fire on a
            // later cycle than a waited LR's own EX1 cycle, by which point
            // the live wire has drifted to whatever op IDU has since decoded
            // (same bug class as amo_is_amo above).
            if (issue_real && (ag_func_eff == LSU_FUNC_LR_W
                               || ag_func_eff == LSU_FUNC_LR_D)) begin
                if (!ag_misalign && !mmu_fault_issue) begin
                    lr_addr_r <= ag_addr[55:0];
                    lr_size_r <= ag_size;
                    lr_addr_set <= 1'b1;
                end
                dc_is_lr_r <= !ag_misalign && !mmu_fault_issue;
            end else if (lsu_rtu_ex1_cmplt_dp || issue_real) begin
                // tag tracks the in-flight transaction (drains never set it)
                dc_is_lr_r <= 1'b0;
            end

            // Fire cmplt_dp for this LR op, set valid
            if (lsu_rtu_ex1_cmplt_dp && lr_addr_set && !lr_valid_r && dc_is_lr_r) begin
                lr_valid_r <= 1'b1;
            end

            // Clear on exclusion
            if (lr_exclude_on_store || lr_exclude_on_load) begin
                lr_valid_r <= 1'b0;
                lr_addr_set <= 1'b0;
            end

            // Any completed SC consumes the reservation, success or failure
            // (rv64ua-p-lrsc test 6: sc-after-successful-sc AND sc-after-
            // failed-sc must both fail). The exclude terms above already
            // clear it for a HIT SC (load-like access); this term covers a
            // MISALIGNED or MISSED SC.
            if (reply_fire && sc_addr_set) begin
                lr_valid_r <= 1'b0;
                lr_addr_set <= 1'b0;
            end

            // Exception ack/exit kills the reservation (donor aq_lsu_lm.v:135
            // clears on expt_ack | expt_exit). An exception between LR and SC
            // -- including interrupts once M6 lands -- must not leave a live
            // reservation behind.
            if (rtu_lsu_expt_ack || rtu_lsu_expt_exit) begin
                lr_valid_r <= 1'b0;
                lr_addr_set <= 1'b0;
            end
        end
    end

    // lr_vld pulses on the completion of an LR (and only an LR: dc_is_lr_r).
    assign lsu_rtu_lr_vld = lsu_rtu_ex1_cmplt_dp && lr_addr_set && dc_is_lr_r;

    //-------------------------------------------------------------------------
    // SECTION AMO (M3 Task 5) -- read-modify-write flow. Detect AMO at
    // issue_real, latch operands; the read phase reuses the load path; at
    // ST_REPLY write OLD to the register file AND create the STB entry
    // holding NEW in that SAME cycle (reply_is_amo_commit in the REPLY
    // section), so the write is visible to fence/STB-empty the moment the
    // AMO completes. The donor creates the AMO's STB entry at its DC stage
    // (aq_lsu_stb.v:654); same-cycle creation here gives the same
    // guarantee, and unlike a deferred next-cycle flag it cannot be dropped
    // on a busy STB (full-STB backpressure comes from the admission control
    // at SECTION drain-vs-issue + reply_can_complete).
    //-------------------------------------------------------------------------
    reg        amo_active;         // AMO in flight (read phase)
    reg [63:0] amo_src0_r;         // register operand (latched at issue)
    reg [4:0]  amo_op_r;           // AMO funct5 (latched at issue)
    reg        amo_is_dw_r;        // 1=D-width, 0=W-width

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            amo_active   <= 1'b0;
            amo_src0_r   <= 64'd0;
            amo_op_r     <= 5'd0;
            amo_is_dw_r  <= 1'b0;
        end else begin
            // Latch AMO operands at issue_real. M4 Task 5 amendment: guard
            // on !ag_misalign && !mmu_fault_issue too -- either trap fires
            // at ISSUE (misalign_issue/mmu_fault_issue) and this AMO never
            // reaches ST_REPLY, so the existing "clear at REPLY" arm below
            // would never fire and amo_active would stay stuck at 1,
            // corrupting the NEXT completing LSU op's writeback mux (the
            // exact "stuck flag" failure mode the M3 audit comment below
            // already describes for the FRZ-miss case). A misaligned AMO
            // could already hit this before Task 5 introduced mmu_fault -
            // fixed here rather than left to be found by the new fault path.
            if (issue_real && amo_is_amo && !ag_misalign && !mmu_fault_issue) begin
                amo_active  <= 1'b1;
                amo_src0_r  <= ag_src2_data_eff;
                amo_op_r    <= amo_op;
                amo_is_dw_r <= amo_dw;
            end
            // Clear at the AMO's OWN REPLY -- INCLUDING a misaligned AMO:
            // the trap must not leave amo_active asserted. A stuck flag
            // made the NEXT completing LSU op be mistaken for the AMO read
            // completion: its writeback was corrupted (W-width sign-extend
            // mux path) and a bogus ALU store was issued to its address.
            if (amo_active && (state == ST_REPLY) && reply_can_complete
                && !dc_is_drain_r)
                amo_active <= 1'b0;
        end
    end

    // NEW value, computed combinationally at REPLY from the live read data
    // (da_final) and the latched register operand. Donor operand mapping
    // (aq_lsu_amo_alu.v:128-129): src0 = memory/OLD, src1 = register/rs1.
    // W-width OLD is sign-extended first (A spec: W AMOs return OLD sign-
    // extended; the ALU's min/max compares also key off bit 31 for W).
    wire [63:0] amo_old_c = amo_is_dw_r ? da_final
                                        : {{32{da_final[31]}}, da_final[31:0]};
    wire [63:0] amo_new_c = amo_alu_compute(amo_old_c, amo_src0_r, amo_op_r, amo_is_dw_r);

    assign lsu_idu_full = (state != ST_IDLE) || clean_active
                          || (ag_wait_r && !issue_real) // M4 Task 5 fix: a parked DTLB-miss
                                             // op holds ST_IDLE (by design, D1 --
                                             // it never touches the array/STB), so
                                             // without this term lsu_idu_full reads
                                             // 0 while the walk is outstanding and
                                             // IDU advances EX1 past it -- the
                                             // comment at line ~412 ("ag_addr itself
                                             // does not move while waiting... IDU
                                             // holds the EX1 register") was never
                                             // actually enforced. idu_lsu_ex1_func/
                                             // src0_data/src1_data (hence ag_addr,
                                             // ag_is_store) drift to whatever LATER
                                             // instruction IDU next dispatches, so
                                             // the eventual mmu_fault_issue for the
                                             // ORIGINAL op reports the wrong PC
                                             // (bju_pcgen_pc has also raced ahead)
                                             // and the wrong load/store vec class
                                             // (rv64si-p-dirty TESTNUM 3: expected
                                             // STORE_PAGE_FAULT at 0x1e0, got
                                             // LOAD_PAGE_FAULT at 0x1f4 -- the PC/
                                             // class of an unrelated later load).
                                             // Donor's analogous "buffered op"
                                             // flag (aq_lsu_ag.v:323 ag_req_buffer,
                                             // valid-gated at :1713 lsu_full_stall
                                             // = ag_req_buffer_vld) always blocks
                                             // new dispatch while a park is live;
                                             // ag_wait_r is rv906's single-flop
                                             // equivalent of that buffer-valid bit.
                                             //
                                             // That term alone only covers cycles
                                             // 2+ of the wait: `ag_wait_r` itself
                                             // doesn't set until the cycle AFTER
                                             // the miss is detected, so on the
                                             // TRUE entry/detection cycle
                                             // (ag_mmu_wait_start && !ag_wait_r)
                                             // this OR-chain still read 0 and EX1
                                             // advanced one cycle too early --
                                             // the SAME root cause, just the one
                                             // remaining uncovered cycle. Folding
                                             // in that term directly (below) needs
                                             // `ag_mmu_wait_start` to be acyclic
                                             // w.r.t. `lsu_idu_full`/`ag_valid`,
                                             // which is why `ag_issue_ready` above
                                             // uses `ag_raw_ready` instead of
                                             // `ag_valid`.
                                             //
                                             // The `&& !issue_real` above is the
                                             // EXIT-side twin of the entry-side fix
                                             // below: bare `ag_wait_r` stays 1
                                             // through the resolution cycle itself
                                             // (the flop's own clear is registered,
                                             // taking effect only next cycle -- see
                                             // the always block a few lines up), so
                                             // on that exact cycle this OR-chain
                                             // would read 1 from `ag_wait_r` AND
                                             // (next cycle) 1 from `state != ST_IDLE`
                                             // -- lsu_idu_full never has the single
                                             // free cycle an ORDINARY (non-wait)
                                             // dispatch gets for free (its issue
                                             // cycle is still ST_IDLE combinationally,
                                             // so `state != ST_IDLE` reads 0 THAT
                                             // cycle and IDU's EX1 advances to the
                                             // next instruction in parallel with the
                                             // op going busy). Denied that same
                                             // cycle, EX1 sits pinned on the AMO for
                                             // its entire busy window and, when
                                             // lsu_idu_full finally clears at
                                             // completion, idu_lsu_ex1_sel fires on
                                             // the STALE (never-advanced) EX1
                                             // content -- a spurious re-dispatch of
                                             // the SAME AMO, double-executing its
                                             // read-modify-write (traced directly:
                                             // mmu_amo TESTNUM 2, cyc=755 resolves
                                             // the wait with lsu_idu_full still 1,
                                             // vs. an ordinary op's issue cycle
                                             // e.g. cyc=648 where lsu_idu_full=0).
                                             // Gating on `issue_real` (defined
                                             // above, acyclic w.r.t. this wire --
                                             // see the ag_raw_ready note) excludes
                                             // exactly the resolving cycle, giving
                                             // the wait-driven op the same one free
                                             // advance cycle an ordinary op already
                                             // gets.
                          || (ag_mmu_wait_start && !ag_wait_r) // M4 Task 5 fix:
                                             // the entry/detection cycle itself
                                             // (see the long comment above).
                          || (stb_full && ag_needs_slot_c)
                          || rf_port_busy   // M3b: don't accept a new op while a
                                             // background refill owns the D-cache
                                             // port (vpeek/commit), else the IDU
                                             // would hand off an op the LSU can't
                                             // take this cycle.
                          || lfb_cmplt_fire  // M3b: the deferred-load completion
                                             // takes the ST_IDLE issue slot this
                                             // cycle (see issue_real/issue_drain's
                                             // own !lfb_cmplt_fire term) -- without
                                             // this term IDU sees "not full" and
                                             // advances EX1 past an op LSU is
                                             // about to silently refuse, dropping
                                             // it forever (rv64ui-p-ld_st hang).
                          || ptw_sv_busy;    // M4 Task 5 (D3): the array/AXI-read
                                             // port is committed to a PTE fetch
                                             // this cycle (this port's own walk OR
                                             // an ITLB walk sharing the same
                                             // servant) -- see SECTION PTW SERVANT.
    // Quiescent = pipe idle AND store buffer empty AND no clean walk in
    // flight AND no deferred load outstanding. state==ST_IDLE implies no
    // AG-issued op is in flight (issue_real leaves IDLE the cycle it fires)
    // and no drain/miss/writeback transaction is pending (all of those live
    // in non-IDLE states). any_stb_vld covers the created-but-undrained
    // entries whose eventual writes must be globally visible before a fence.
    // M3b: a deferred load (any LFB entry occupied) also means the LSU is
    // not quiescent -- fence/fence.i must wait for its refill to land
    // (rv64ui-p-fence_i).
    // M3b Task B/C: the VB can still hold an undrained dirty writeback after
    // its owning LFB entry has already drained (vb_vld is decoupled from
    // lfb_empty by design -- see the VB section) -- a fence must wait for
    // that writeback to become globally visible too, else fence/fence.i
    // could complete before the old line's data actually reaches memory.
    assign lsu_cp0_stb_empty = (state == ST_IDLE) && !any_stb_vld && !clean_active
                               && lfb_empty && !vb_vld;

    //-------------------------------------------------------------------------
    // SECTION FRZ -- victim-select/writeback/refill-commit (cacheable miss)
    // or a generic direct AXI read-or-write (uncached / wa=0 store-miss /
    // direct STB drain). See header for the overall shape.
    //-------------------------------------------------------------------------
    localparam [3:0] MS_IDLE          = 4'd0;
    localparam [3:0] MS_VPEEK_ISSUE   = 4'd1;
    localparam [3:0] MS_VPEEK_WAIT    = 4'd2;
    // 4'd3 retired (M3b Task B/C): the old inline MS_VB_WRITE state used to
    // block the refill on the victim writeback; the victim now hands off to
    // the VB (below) and MS_VPEEK_WAIT goes straight to MS_REFILL_READ.
    localparam [3:0] MS_REFILL_READ   = 4'd4;
    localparam [3:0] MS_COMMIT_ISSUE  = 4'd5;
    localparam [3:0] MS_COMMIT_WAIT   = 4'd6;
    localparam [3:0] MS_DIRECT_WRITE  = 4'd7;
    localparam [3:0] MS_DIRECT_READ   = 4'd8;
    localparam [3:0] MS_DONE          = 4'd9;
    // M3b Task A rework: activation-time way_vld/way_dirty re-read for an
    // LFB entry transitioning E_PENDING->E_ACTIVE (see the LFB section
    // above) -- a fresh D-cache metadata read for the activating entry's
    // frz_eff_tag/frz_eff_index, before MS_IDLE's victim-select runs.
    localparam [3:0] MS_LFB_PEEK_ISSUE = 4'd10;
    localparam [3:0] MS_LFB_PEEK_WAIT  = 4'd11;

    reg [3:0]  miss_state;
    reg [1:0]  victim_idx_r;
    reg [WAYS-1:0] victim_way_r;
    reg        victim_dirty_r;
    reg [1:0]  rr_ctr;
    // M3b Task B/C: single-entry victim buffer (donor aq_lsu_vb.v, ONE entry
    // :135-148) decouples the dirty-victim writeback from the refill
    // critical path (donor aq_lsu_rdl.v CHECK->WVB before evicting another
    // dirty line). MS_VPEEK_WAIT hands the peeked victim off here and moves
    // straight to MS_REFILL_READ; the VB then drains independently whenever
    // the shared axi_w_* write-channel registers are free. Adaptation: since
    // rv906's AXI slave is single-outstanding for ANY transaction (read or
    // write, AXICrossbar.v sr_active), true bus concurrency between the
    // refill read and the victim write isn't achievable -- but ordering the
    // refill ahead of the writeback still removes the writeback from the
    // demand-miss's own latency, matching the donor's decoupling intent.
    reg        vb_vld;
    reg [DCACHE_TAG_WIDTH-1:0] vb_tag_r;
    reg [DCACHE_INDEX_W-1:0]   vb_index_r;
    reg [511:0]                vb_data_r;
    reg [511:0] frz_rdata_r;   // FRZ-local capture (refill or direct-read data);
                                 // copied into dc_rdata_r by the MAIN fsm's own
                                 // always block at the FRZ->REPLY transition so
                                 // dc_rdata_r keeps exactly one writer.

    wire miss_done = (miss_state == MS_DONE);

    //-------------------------------------------------------------------------
    // M3b Task A rework -- LFB (Load-Fill Buffer), DEPTH=8 per donor
    // aq_lsu_lfb.v:666. rv906's AXI is genuinely single-outstanding (no
    // arid/rid anywhere), so an entry can only become "active" (own the
    // shared refill engine) in the same order entries were created --
    // completion order is forced to equal creation order. This collapses
    // the donor's three independent round-robin pointers plus its separate
    // in-order pop pointer into a single circular FIFO: lfb_tail_ptr (next
    // allocation slot) and lfb_head_ptr (the entry owning the shared engine,
    // or awaiting drain). lfb_full is the same wrap-bit circular-FIFO test
    // as the donor (aq_lsu_lfb.v:718-719). A cacheable LOAD miss allocates
    // an entry here and its refill runs in the background (the FRZ sub-FSM
    // below is gated to run for `lfb_engine_active` as well as
    // `state==ST_FRZ`), so the main FSM returns to ST_IDLE and can complete
    // cache-HIT ops while the refill is in flight. When the refill commits,
    // the deferred load completes straight from the refill data
    // (frz_rdata_r) merged with any STB store forward -- no second cache
    // probe, so no eviction race.
    //-------------------------------------------------------------------------
    localparam LFB_DEPTH = 8;
    localparam [1:0] E_IDLE = 2'd0, E_PENDING = 2'd1, E_ACTIVE = 2'd2, E_DONE = 2'd3;

    reg [1:0]  lfb_state    [0:LFB_DEPTH-1];
    reg [63:0] lfb_addr     [0:LFB_DEPTH-1];   // deferred load byte address
    reg [GPR_IDX_WIDTH-1:0] lfb_dst [0:LFB_DEPTH-1];
    reg [1:0]  lfb_size     [0:LFB_DEPTH-1];
    reg        lfb_sign_ext [0:LFB_DEPTH-1];
    reg [2:0]  lfb_byte_off [0:LFB_DEPTH-1];
    reg [2:0]  lfb_dw_off   [0:LFB_DEPTH-1];
    reg [DCACHE_TAG_WIDTH-1:0] lfb_tag   [0:LFB_DEPTH-1];
    reg [DCACHE_INDEX_W-1:0]   lfb_index [0:LFB_DEPTH-1];
    // M3b Task D: prefetch-allocated entries (donor lfb.v's lfb_pf per-entry
    // attribute, carried on the create bus from pfb_top.v:414). A prefetch
    // entry refills exactly like a demand miss but drains SILENTLY (no RTU
    // completion/writeback), and if its activation-time D-cache re-read finds
    // the line already resident it completes without any refill at all
    // (donor rdl.v:627 pf_cache_hit -> rdl_lfb_cmplt at CHECK).
    reg        lfb_pf      [0:LFB_DEPTH-1];
    reg [4:0]  lfb_pfb_id  [0:LFB_DEPTH-1];   // one-hot PFB entry id, for the
                                              // cache_hit/miss fill feedback

    reg [3:0] lfb_head_ptr;   // [2:0] entry index, [3] wrap bit
    reg [3:0] lfb_tail_ptr;

    // Way-valid/dirty for the currently-ACTIVE entry only: NOT arrayized.
    // Re-read fresh at E_PENDING->E_ACTIVE activation (MS_LFB_PEEK_* below),
    // not snapshotted at allocation time -- with up to 8 entries queued, an
    // entry may sit E_PENDING for many cycles behind others, and an
    // allocation-time snapshot could go stale before it activates (matches
    // the donor's own RDL-time, not LFB-creation-time, victim lookup).
    reg [WAYS-1:0] lfb_way_vld_r;
    reg [WAYS-1:0] lfb_way_dirty_r;
    // M3b Task D: activation-time D-cache residency for the active entry's
    // line -- the activation peek already requests the entry's own tag/index,
    // so |u_dc_resp_hit_way| on its response is the donor's RDL-time prefetch
    // tag compare (rdl.v:609-627) for free. A PREFETCH entry that hits here
    // completes without any refill (rdl_lfb_cmplt at CHECK on pf_cache_hit).
    reg            lfb_dc_hit_r;
    // Dedicated capture of the active entry's refill dword, latched the
    // cycle its background refill commits (lfb_rf_done_set). frz_rdata_r is
    // shared with any later op's own miss/refill (same miss_state FSM);
    // without this separate copy, a second op missing after this refill
    // completes but before lfb_cmplt_fire drains it (main FSM busy servicing
    // that op) would clobber frz_rdata_r and corrupt this load's writeback
    // data. Single shared register, not arrayized: only the ACTIVE entry
    // ever has a refill in flight, and lfb_head_ptr only advances past a
    // drained entry, so there is no lifetime conflict.
    reg [63:0] lfb_rdata_r;

    integer lfb_i;

    wire [2:0] lfb_head_idx = lfb_head_ptr[2:0];
    wire [2:0] lfb_tail_idx = lfb_tail_ptr[2:0];
    wire [1:0] lfb_head_state = lfb_state[lfb_head_idx];

    wire lfb_full  = (lfb_head_ptr[2:0] == lfb_tail_ptr[2:0])
                     && (lfb_head_ptr[3] != lfb_tail_ptr[3]);
    wire lfb_empty = (lfb_head_ptr == lfb_tail_ptr);
    wire lfb_any_vld = !lfb_empty;

    // Hit-under-miss / secondary-access replay: OR-reduce this op's line
    // (dc_tag_r/dc_index_r, stable throughout its ST_DCS dwell) against
    // every occupied entry's captured line address (mirrors the donor's
    // dc_hit_lfb_idx/dc_hit_lfb_addr OR-tree, aq_lsu_lfb.v:722-726 -- the
    // donor is confirmed replay-not-merge, no secondary-miss queue).
    reg lfb_addr_hit_c;
    always @* begin
        lfb_addr_hit_c = 1'b0;
        for (lfb_i = 0; lfb_i < LFB_DEPTH; lfb_i = lfb_i + 1) begin
            if ((lfb_state[lfb_i] != E_IDLE)
                && (lfb_tag[lfb_i] == dc_tag_r) && (lfb_index[lfb_i] == dc_index_r))
                lfb_addr_hit_c = 1'b1;
        end
    end
    wire lfb_addr_hit = lfb_addr_hit_c;

    wire lfb_engine_active = (lfb_head_state == E_ACTIVE);
    wire lfb_head_done     = (lfb_head_state == E_DONE);

    // Effective line address for the FRZ refill: background refill uses the
    // active entry's captured tag/index; the blocking FRZ keeps dc_* as before.
    wire [DCACHE_TAG_WIDTH-1:0] frz_eff_tag   = lfb_engine_active ? lfb_tag[lfb_head_idx]   : dc_tag_r;
    wire [DCACHE_INDEX_W-1:0]   frz_eff_index = lfb_engine_active ? lfb_index[lfb_head_idx] : dc_index_r;

    // Defer: a cacheable plain-load miss allocates a new entry at
    // lfb_tail_ptr -- on first arrival (a fresh miss at ST_DCS) or on a
    // retry once a park (dc_wait_lfb_r) clears (same latched dc_*_r fields
    // throughout, since this op never leaves ST_DCS while parked).
    wire lfb_first_miss_defer = (state == ST_DCS) && u_dc_resp_vld && !dc_is_drain_r
                                && !dc_misalign_r && dc_ca_r && dc_plain_ld_r && !dc_hit_c
                                && !lfb_addr_hit && !lfb_full;
    wire lfb_retry_defer      = (state == ST_DCS) && dc_wait_lfb_r && dc_plain_ld_r && dc_ca_r
                                && !lfb_addr_hit && !lfb_full;
    wire lfb_defer_fire = lfb_first_miss_defer || lfb_retry_defer;

    // Activate: the head entry moves E_PENDING->E_ACTIVE once the shared
    // engine is free and no live hit-lookup is using the D-cache response
    // bus this cycle (same anti-collision gate as frz_issue_vpeek/commit).
    wire lfb_head_activatable = (lfb_head_state == E_PENDING)
                                && (state == ST_IDLE || dc_wait_lfb_r);

    // Refill committed: the background refill's sub-FSM reaching MS_DONE
    // for the active entry (single-writer of lfb_state[] is the LFB always
    // block below).
    wire lfb_rf_done_set = (miss_state == MS_DONE) && lfb_engine_active;
    // Complete: once the head entry's refill has committed (E_DONE), the
    // deferred load completes straight from the refill data. Gated on
    // state==ST_IDLE||dc_wait_lfb_r (same anti-collision condition as
    // lfb_head_activatable, above): a second op that missed the SAME line
    // as the now-DONE head entry parks here (dc_wait_lfb_r) with
    // lfb_addr_hit=1 against that entry, and can only unpark once the entry
    // actually drains to E_IDLE. Gating this on ST_IDLE alone deadlocks --
    // the main FSM can never reach ST_IDLE while that op holds ST_DCS
    // parked, so the entry could never drain, so the op could never unpark.
    // dc_wait_lfb_r means the D-cache response bus and RTU writeback port
    // are both free this cycle (the parked op isn't using them), so it's
    // safe to drain here exactly as it is at ST_IDLE.
    wire lfb_cmplt_fire = lfb_any_vld && lfb_head_done && (state == ST_IDLE || dc_wait_lfb_r) && !clean_active;

    // lfb_dw_off[head] is latched at allocation, well before lfb_rf_done_set
    // can fire, so this offset is already valid the cycle the refill commits.
    wire [8:0] lfb_dword_bitoff = {6'b0, lfb_dw_off[lfb_head_idx]} << 6;

    //-------------------------------------------------------------------------
    // SECTION PFB -- M3b Task D: PC-indexed stride prefetcher (donor
    // aq_lsu_pfb_top.v + aq_lsu_pfb.v, see rtl/PFB.v for the port and its
    // adaptation notes). Trained by cacheable load MISSES at ST_DCS (donor
    // dc_pfb_*, aq_lsu_dc.v:1881-1899); a confirmed stride issues prefetch
    // requests that allocate LFB entries here -- demand always wins
    // (lfb.v:673,730), and a request whose line is already covered in the
    // LFB/STB/VB or is the in-flight demand line is suppressed at grant time
    // (pfb_top.v:395-396). The authoritative "line already resident" check
    // happens at activation time (MS_LFB_PEEK_*, mirroring the donor's
    // RDL-time prefetch tag compare, rdl.v:609-627), completing such an
    // entry without a refill and reporting cache_hit feedback.
    //-------------------------------------------------------------------------
    // Training event: one cacheable plain-load DC-stage lookup (hit OR miss
    // -- donor trains entries on every load; only CREATION needs a miss).
    // !dc_wait_lfb_r scopes the pulse to this op's OWN probe response: while
    // an op is parked here, the shared (unscoped) D-cache response bus also
    // carries background-refill traffic, which must not be counted as loads
    // (same hazard the latched-dc_hit_r retry note above documents).
    wire pfb_ld_vld_c
         = (state == ST_DCS) && u_dc_resp_vld && !dc_wait_lfb_r
           && dc_touched_array_r && dc_plain_ld_r && dc_ca_r && !dc_is_drain_r;
    wire pfb_ld_miss_c = !dc_hit_c;

    // Granted request out of the PFB (already suppression-checked inside).
    wire        pfb_req;
    wire [39:0] pfb_req_pa;
    wire [4:0]  pfb_req_id;

    wire [DCACHE_TAG_WIDTH-1:0] pfb_req_tag   = pfb_req_pa[39:13];
    wire [DCACHE_INDEX_W-1:0]   pfb_req_index = pfb_req_pa[12:6];

    // Suppression compares (donor pfb_top.v:395-396 hit_idx inputs):
    //  lfb: pfb_hit_lfb_idx OR-tree (lfb.v:729)
    //  stb: pfb_hit_stb_idx       (stb.v:615)
    //  vb : line == vb_addr & vb_vld (vb.v:490)
    //  dc : line == the demand line currently occupying the pipe (dc.v:1901)
    reg pfb_lfb_hit_c, pfb_stb_hit_c;
    always @* begin
        pfb_lfb_hit_c = 1'b0;
        pfb_stb_hit_c = 1'b0;
        for (lfb_i = 0; lfb_i < LFB_DEPTH; lfb_i = lfb_i + 1) begin
            if ((lfb_state[lfb_i] != E_IDLE)
                && (lfb_tag[lfb_i] == pfb_req_tag) && (lfb_index[lfb_i] == pfb_req_index))
                pfb_lfb_hit_c = 1'b1;
        end
        for (lfb_i = 0; lfb_i < 4; lfb_i = lfb_i + 1) begin
            if (stb_vld[lfb_i]
                && (stb_tag[lfb_i] == pfb_req_tag) && (stb_index[lfb_i] == pfb_req_index))
                pfb_stb_hit_c = 1'b1;
        end
    end
    wire pfb_vb_hit = vb_vld && (vb_tag_r == pfb_req_tag) && (vb_index_r == pfb_req_index);
    wire pfb_dc_hit = (state != ST_IDLE) && dc_ca_r
                      && (dc_tag_r == pfb_req_tag) && (dc_index_r == pfb_req_index);

    // Raw grant (donor lfb_pfb_grant = !lfb_full & !dc_sel, lfb.v:730):
    // a slot is free and no demand miss is creating one this cycle.
    wire pfb_grant_c = !lfb_full && !lfb_defer_fire && !clean_active;

    // Fill feedback (donor lfb_pfb_cache_hit/miss, lfb.v:842-843): one-hot
    // id pulses when a prefetch entry completes -- cache_hit if the line was
    // already resident (activation-time drop, below), cache_miss if it
    // genuinely refilled.
    wire lfb_pf_drop_c = (miss_state == MS_IDLE) && lfb_engine_active
                         && lfb_pf[lfb_head_idx] && lfb_dc_hit_r;
    reg  lfb_pf_drop_r;   // held through the drop's MS_DONE cycle so the
                          // completion reports cache_HIT, not cache_miss
    wire [4:0] pfb_fb_hit  = lfb_pf_drop_c            ? lfb_pfb_id[lfb_head_idx] : 5'b0;
    wire [4:0] pfb_fb_miss  = (lfb_rf_done_set && lfb_pf[lfb_head_idx] && !lfb_pf_drop_r)
                              ? lfb_pfb_id[lfb_head_idx] : 5'b0;

    PFB u_pfb (
        .clk(clk), .rst_n(rst_n),
        .cp0_lsu_dcache_en(cp0_lsu_dcache_en),
        .cp0_lsu_dcache_pref_en(cp0_lsu_dcache_pref_en),
        .cp0_lsu_dcache_pref_dist(cp0_lsu_dcache_pref_dist),
        .clean_active(clean_active),
        .pfb_ld_vld(pfb_ld_vld_c), .pfb_ld_miss(pfb_ld_miss_c),
        .pfb_ld_pc(dc_pc_r),
        // M4: PFB wants the physical address (it feeds real prefetch-read
        // addresses, PFB.v:315,512) -- dc_addr_r is VA (see the AXI
        // direct-write fix above); rebuild the PA from dc_tag_r/dc_index_r.
        .pfb_ld_chk_pa({dc_tag_r, dc_index_r, dc_addr_r[5:0]}),
        .lfb_grant(pfb_grant_c),
        .lfb_hit_idx(pfb_lfb_hit_c), .stb_hit_idx(pfb_stb_hit_c),
        .vb_hit_idx(pfb_vb_hit), .dc_hit_idx(pfb_dc_hit),
        .lfb_fb_hit(pfb_fb_hit), .lfb_fb_miss(pfb_fb_miss),
        .pfb_req(pfb_req), .pfb_req_pa(pfb_req_pa), .pfb_req_id(pfb_req_id)
    );

    // Prefetch create: mutually exclusive with the demand create above by
    // pfb_grant_c's !lfb_defer_fire term (donor create mux, lfb.v:673).
    wire pfb_create_fire = pfb_req && !lfb_full;

    //-------------------------------------------------------------------------
    // SECTION AMR -- M3b Task E: streaming-store write-allocate disabler
    // (donor aq_lsu_amr.v, see rtl/AMR.v for the port and adaptation notes).
    // Trained by cacheable stores at ST_DCS (donor dc_amr_*, aq_lsu_dc.v:
    // 1906-1926); once a store-size-stride stream is confirmed it raises
    // amr_dc_wa_dis, gating write-allocate off for subsequent store misses.
    //-------------------------------------------------------------------------
    // Training event: one cacheable store DC-stage lookup (store version of
    // pfb_ld_vld_c; dc_is_store_r is func[0], so AMO/LR/SC are excluded --
    // the donor's !dc_lock_trans -- and drains are internal, not stores).
    wire dc_amr_st_req_c = (state == ST_DCS) && u_dc_resp_vld && !dc_wait_lfb_r
                           && dc_touched_array_r && dc_is_store_r && dc_ca_r
                           && !dc_is_drain_r;
    // Cancel: a NON-cacheable store passing through resets the detector
    // (donor dc_amr_cancel, aq_lsu_dc.v:1912). Non-cacheable stores do not
    // touch the array, so the decision strobe is the immediate branch.
    wire dc_amr_cancel_c = (state == ST_DCS) && !dc_wait_lfb_r && dc_is_store_r
                           && !dc_is_drain_r && !dc_ca_r
                           && (!dc_touched_array_r || u_dc_resp_vld);
    wire        dc_amr_st_miss_c = !dc_hit_c;
    wire [4:0]  dc_amr_st_size_c = 5'd1 << dc_size_r;   // bytes (donor hint_size)

    wire amr_dc_wa_dis;
    AMR u_amr (
        .clk(clk), .rst_n(rst_n),
        .cp0_lsu_amr(cp0_lsu_amr),
        .cp0_lsu_dcache_en(cp0_lsu_dcache_en),
        .cp0_lsu_sync_req(clean_active),
        .dc_amr_st_req(dc_amr_st_req_c), .dc_amr_cancel(dc_amr_cancel_c),
        .dc_amr_st_miss(dc_amr_st_miss_c), .dc_amr_st_size(dc_amr_st_size_c),
        .dc_amr_st_addr(dc_addr_r[39:0]),
        .amr_dc_wa_dis(amr_dc_wa_dis)
    );

    // Effective write-allocate (donor dcache_wa = cp0_wa & !amr_dc_wa_dis).
    wire dc_wa_eff = dc_wa_r & ~amr_dc_wa_dis;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfb_head_ptr <= 4'd0;
            lfb_tail_ptr <= 4'd0;
            lfb_rdata_r  <= 64'd0;
            lfb_pf_drop_r <= 1'b0;
            for (lfb_i = 0; lfb_i < LFB_DEPTH; lfb_i = lfb_i + 1) begin
                lfb_state[lfb_i]    <= E_IDLE;
                lfb_addr[lfb_i]     <= 64'd0;
                lfb_dst[lfb_i]      <= {GPR_IDX_WIDTH{1'b0}};
                lfb_size[lfb_i]     <= 2'd0;
                lfb_sign_ext[lfb_i] <= 1'b0;
                lfb_byte_off[lfb_i] <= 3'd0;
                lfb_dw_off[lfb_i]   <= 3'd0;
                lfb_tag[lfb_i]      <= {DCACHE_TAG_WIDTH{1'b0}};
                lfb_index[lfb_i]    <= {DCACHE_INDEX_W{1'b0}};
                lfb_pf[lfb_i]       <= 1'b0;
                lfb_pfb_id[lfb_i]   <= 5'b0;
            end
        end else begin
            // Tail side: allocate a new entry. Independent of the head-side
            // activity below -- a create and a drain can happen the same
            // cycle (a true FIFO), so this is a separate `if`, not chained.
            if (lfb_defer_fire) begin
                lfb_addr[lfb_tail_idx]     <= dc_addr_r;
                lfb_dst[lfb_tail_idx]      <= dc_dst0_reg_r;
                lfb_size[lfb_tail_idx]     <= dc_size_r;
                lfb_sign_ext[lfb_tail_idx] <= dc_sign_ext_r;
                lfb_byte_off[lfb_tail_idx] <= dc_byte_off_r;
                lfb_dw_off[lfb_tail_idx]   <= dc_dw_off_r;
                lfb_tag[lfb_tail_idx]      <= dc_tag_r;
                lfb_index[lfb_tail_idx]    <= dc_index_r;
                lfb_pf[lfb_tail_idx]       <= 1'b0;
                lfb_pfb_id[lfb_tail_idx]   <= 5'b0;
                lfb_state[lfb_tail_idx]    <= E_PENDING;
                lfb_tail_ptr <= lfb_tail_ptr + 4'd1;
            end else if (pfb_create_fire) begin
                // M3b Task D: prefetch allocation (donor lfb.v:673's create
                // mux, pfb side). Line-based address, no writeback context
                // (dst/size/offsets unused -- a prefetch drains silently).
                lfb_addr[lfb_tail_idx]     <= {24'b0, pfb_req_pa};
                lfb_dst[lfb_tail_idx]      <= {GPR_IDX_WIDTH{1'b0}};
                lfb_size[lfb_tail_idx]     <= 2'd3;
                lfb_sign_ext[lfb_tail_idx] <= 1'b0;
                lfb_byte_off[lfb_tail_idx] <= 3'd0;
                lfb_dw_off[lfb_tail_idx]   <= 3'd0;
                lfb_tag[lfb_tail_idx]      <= pfb_req_tag;
                lfb_index[lfb_tail_idx]    <= pfb_req_index;
                lfb_pf[lfb_tail_idx]       <= 1'b1;
                lfb_pfb_id[lfb_tail_idx]   <= pfb_req_id;
                lfb_state[lfb_tail_idx]    <= E_PENDING;
                lfb_tail_ptr <= lfb_tail_ptr + 4'd1;
            end

            // Head side: activate / refill-done / drain -- mutually
            // exclusive, since they are different states of the SAME entry
            // (lfb_head_ptr). The E_PENDING->E_ACTIVE kickoff is also
            // consumed by the miss_state always block below (same
            // lfb_head_activatable condition) to launch MS_LFB_PEEK_ISSUE.
            if (lfb_head_activatable) begin
                lfb_state[lfb_head_idx] <= E_ACTIVE;
            end else if (lfb_rf_done_set) begin
                lfb_state[lfb_head_idx] <= E_DONE;
                lfb_rdata_r <= frz_rdata_r[lfb_dword_bitoff +: 64];
            end else if (lfb_cmplt_fire) begin
                lfb_state[lfb_head_idx] <= E_IDLE;
                lfb_head_ptr <= lfb_head_ptr + 4'd1;
            end
        end
    end

    // Replacement policy (LSU-owned, per DCache.v's header decision):
    // prefer an invalid way, else the round-robin counter. The source of the
    // set's way-valid/dirty is muxed: a background (LFB) refill reads the
    // freshly re-read active-entry snapshot, the blocking FRZ reads the dc_*
    // latches.
    wire [WAYS-1:0] vic_src_vld   = lfb_engine_active ? lfb_way_vld_r   : dc_way_vld_r;
    wire [WAYS-1:0] vic_src_dirty = lfb_engine_active ? lfb_way_dirty_r : dc_way_dirty_r;
    wire [1:0] victim_idx_c = !vic_src_vld[0] ? 2'd0 :
                              !vic_src_vld[1] ? 2'd1 :
                              !vic_src_vld[2] ? 2'd2 :
                              !vic_src_vld[3] ? 2'd3 : rr_ctr;
    wire [WAYS-1:0] victim_way_c = 4'b0001 << victim_idx_c;
    wire victim_dirty_c = (victim_idx_c == 2'd0) ? vic_src_dirty[0] :
                          (victim_idx_c == 2'd1) ? vic_src_dirty[1] :
                          (victim_idx_c == 2'd2) ? vic_src_dirty[2] : vic_src_dirty[3];

    //---- generic AXI write sub-sequence (victim writeback / uncached store
    // / direct STB drain) -- FetchSink.v's proven D-side write FSM
    // (concurrent AW+W, BVALID watched every cycle), generalized from its
    // fixed tohost address/data to a real, parametrized one.
    reg         axi_w_active;
    reg         axi_w_aw_sent, axi_w_w_sent;
    reg [63:0]  axi_w_addr_r;
    reg [511:0] axi_w_data_r;
    reg [63:0]  axi_w_strb_r;
    reg [2:0]   axi_w_awsize_r;

    wire axi_w_aw_hs = axi_d_awvalid && axi_d_awready;
    wire axi_w_w_hs  = axi_d_wvalid  && axi_d_wready;
    wire axi_w_done  = axi_w_active && axi_d_bvalid && (axi_w_aw_sent || axi_w_aw_hs) && (axi_w_w_sent || axi_w_w_hs);

    //---- generic AXI read sub-sequence (refill / uncached load).
    reg         axi_r_active;
    reg         axi_r_ar_sent;
    reg [63:0]  axi_r_addr_r;
    wire axi_r_ar_hs   = axi_d_arvalid && axi_d_arready;
    wire axi_r_data_hs = axi_r_active && axi_d_rvalid && axi_d_rready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            miss_state     <= MS_IDLE;
            victim_idx_r   <= 2'd0;
            victim_way_r   <= {WAYS{1'b0}};
            victim_dirty_r <= 1'b0;
            vb_vld         <= 1'b0;
            vb_tag_r       <= {DCACHE_TAG_WIDTH{1'b0}};
            vb_index_r     <= {DCACHE_INDEX_W{1'b0}};
            vb_data_r      <= 512'd0;
            rr_ctr         <= 2'd0;
            axi_w_active   <= 1'b0;
            axi_w_aw_sent  <= 1'b0;
            axi_w_w_sent   <= 1'b0;
            axi_w_addr_r   <= 64'd0;
            axi_w_data_r   <= 512'd0;
            axi_w_strb_r   <= 64'd0;
            axi_w_awsize_r <= 3'd6;
            axi_r_active   <= 1'b0;
            axi_r_ar_sent  <= 1'b0;
            axi_r_addr_r   <= 64'd0;
            frz_rdata_r    <= 512'd0;
            lfb_way_vld_r   <= {WAYS{1'b0}};
            lfb_way_dirty_r <= {WAYS{1'b0}};
            lfb_dc_hit_r    <= 1'b0;
        end else begin
            // one-cycle-late AW/W accept bookkeeping (shared by every write use)
            if (axi_w_active) begin
                if (axi_w_aw_hs) axi_w_aw_sent <= 1'b1;
                if (axi_w_w_hs)  axi_w_w_sent  <= 1'b1;
            end
            if (axi_r_active && axi_r_ar_hs) axi_r_ar_sent <= 1'b1;

            // M3b: the miss sub-FSM runs both for the blocking FRZ
            // (state==ST_FRZ) and for a background LFB refill
            // (lfb_engine_active, main FSM back at ST_IDLE servicing hits).
            // When the shared engine is free and the head entry is waiting
            // its turn (E_PENDING), kick off the activation-time
            // way_vld/way_dirty re-read (MS_LFB_PEEK_ISSUE/WAIT) before
            // MS_IDLE's victim-select runs.
            if ((state == ST_FRZ) || lfb_engine_active) begin
                case (miss_state)
                    MS_LFB_PEEK_ISSUE: if (frz_issue_lfb_peek) miss_state <= MS_LFB_PEEK_WAIT;
                    MS_LFB_PEEK_WAIT: begin
                        if (u_dc_resp_vld) begin
                            lfb_way_vld_r   <= u_dc_resp_way_vld;
                            lfb_way_dirty_r <= u_dc_resp_way_dirty;
                            lfb_dc_hit_r    <= |u_dc_resp_hit_way;
                            miss_state <= MS_IDLE;
                        end
                    end
                    MS_IDLE: begin
                        if (lfb_engine_active && lfb_pf[lfb_head_idx] && lfb_dc_hit_r) begin
                            // M3b Task D: prefetch target is ALREADY resident
                            // (found by the activation re-read) -- complete
                            // with no refill, exactly the donor's RDL
                            // pf_cache_hit early completion (rdl.v:627,634).
                            // lfb_pf_drop_r (set here, cleared at the next
                            // activation) makes the completion pulse report
                            // cache_HIT feedback to the PFB entry.
                            lfb_pf_drop_r <= 1'b1;
                            miss_state <= MS_DONE;
                        end else if (frz_is_direct_r && !lfb_engine_active) begin
                            if (dc_is_store_r) begin
                                if (vb_vld) begin
                                    // M3b Task B/C: wait for the single-entry
                                    // VB to free before claiming the shared
                                    // axi_w_* write-channel registers for
                                    // this direct store. Retry at MS_IDLE.
                                    miss_state <= MS_IDLE;
                                end else begin
                                    axi_w_active  <= 1'b1;
                                    axi_w_aw_sent <= 1'b0;
                                    axi_w_w_sent  <= 1'b0;
                                    // AW address is the STORE's physical
                                    // address (donor C906 STB uses stb_entry_pa,
                                    // aq_lsu_stb.v:895), not the line base: the
                                    // sub-word beat is positioned at (addr[5:0])
                                    // inside the 64B line and the AXI memory
                                    // model sizes the write from addr[5:0].
                                    // M4: dc_addr_r itself is VA (see the
                                    // SECTION drain-vs-issue comment above,
                                    // ~line 781); dc_tag_r/dc_index_r are the
                                    // MMU-translated PA (latched from ag_pa
                                    // at issue, or from stb_tag/stb_index at
                                    // drain). Rebuild the PA the same way the
                                    // read-miss path below does (dc_addr_r[5:0]
                                    // is safe to reuse -- Sv39's page offset
                                    // is untranslated).
                                    axi_w_addr_r  <= {{(ADDR_WIDTH - PC_WIDTH){1'b0}}, dc_tag_r, dc_index_r, dc_addr_r[5:0]};
                                    axi_w_data_r  <= ({448'b0, (dc_is_drain_r ? stb_data[dc_drain_idx_r] : dc_store_data_r)})
                                                       << ({58'b0, (dc_is_drain_r ? stb_dw_off[dc_drain_idx_r] : dc_dw_off_r)} * 64);
                                    axi_w_strb_r  <= ({56'b0, (dc_is_drain_r ? stb_byte_vld[dc_drain_idx_r] : dc_byte_mask_r)})
                                                       << ({58'b0, (dc_is_drain_r ? stb_dw_off[dc_drain_idx_r] : dc_dw_off_r)} * 8);
                                    // AW size = store size (sb->0..sd->3); a
                                    // drained entry uses its tracked size (the
                                    // store's own dc_size_r is stale by drain
                                    // time). Donor: stb_awsize = stb_entry_size
                                    // (aq_lsu_stb.v:898).
                                    axi_w_awsize_r <= dc_is_drain_r ? stb_size[dc_drain_idx_r] : {1'b0, dc_size_r};
                                    miss_state <= MS_DIRECT_WRITE;
                                end
                            end else begin
                                axi_r_active  <= 1'b1;
                                axi_r_ar_sent <= 1'b0;
                                axi_r_addr_r  <= {{(ADDR_WIDTH - PC_WIDTH){1'b0}}, dc_tag_r, dc_index_r, 6'b0};
                                miss_state <= MS_DIRECT_READ;
                            end
                        end else begin
                            victim_idx_r   <= victim_idx_c;
                            victim_way_r   <= victim_way_c;
                            victim_dirty_r <= victim_dirty_c;
                            if (victim_dirty_c && vb_vld) begin
                                // M3b Task B/C RDL WVB: donor waits for
                                // the single-entry VB to free before
                                // evicting another dirty line (donor
                                // aq_lsu_rdl.v CHECK->WVB, aq_lsu_vb.v
                                // ONE entry :135-148). Retry at MS_IDLE.
                                miss_state <= MS_IDLE;
                            end else begin
                                miss_state <= victim_dirty_c ? MS_VPEEK_ISSUE : MS_REFILL_READ;
                            end
                        end
                        end
                        MS_VPEEK_ISSUE: if (frz_issue_vpeek) miss_state <= MS_VPEEK_WAIT;
                        MS_VPEEK_WAIT: begin
                            // M3b Task B/C: hand the peeked victim off to the
                            // single-entry VB (donor aq_lsu_rdl.v CHECK->WVB,
                            // aq_lsu_vb.v :135-148) and go straight to the
                            // refill read instead of blocking on the
                            // writeback -- the VB drains independently below.
                            if (u_dc_resp_vld) begin
                                vb_vld       <= 1'b1;
                                vb_tag_r     <= u_dc_resp_victim_tag;
                                vb_index_r   <= frz_eff_index;
                                vb_data_r    <= u_dc_resp_rdata;
                                axi_r_active  <= 1'b1;
                                axi_r_ar_sent <= 1'b0;
                                axi_r_addr_r  <= {{(ADDR_WIDTH - PC_WIDTH){1'b0}}, frz_eff_tag, frz_eff_index, 6'b0};
                                miss_state    <= MS_REFILL_READ;
                            end
                        end
                        MS_REFILL_READ: begin
                            if (miss_state == MS_REFILL_READ && axi_r_active == 1'b0) begin
                                axi_r_active  <= 1'b1;
                                axi_r_ar_sent <= 1'b0;
                                axi_r_addr_r  <= {{(ADDR_WIDTH - PC_WIDTH){1'b0}}, frz_eff_tag, frz_eff_index, 6'b0};
                            end else if (axi_r_data_hs) begin
                                axi_r_active <= 1'b0;
                                frz_rdata_r   <= axi_d_rdata;
                                rr_ctr        <= rr_ctr + 2'd1;
                                miss_state    <= MS_COMMIT_ISSUE;
                            end
                        end
                        MS_COMMIT_ISSUE: if (frz_issue_commit) miss_state <= MS_COMMIT_WAIT;
                        MS_COMMIT_WAIT: begin
                            if (u_dc_resp_vld) miss_state <= MS_DONE;
                        end
                        MS_DIRECT_WRITE: begin
                            if (axi_w_done) begin
                                axi_w_active  <= 1'b0;
                                axi_w_aw_sent <= 1'b0;
                                axi_w_w_sent  <= 1'b0;
                                miss_state    <= MS_DONE;
                            end
                        end
                        MS_DIRECT_READ: begin
                            if (axi_r_data_hs) begin
                                axi_r_active <= 1'b0;
                                frz_rdata_r   <= axi_d_rdata;
                                miss_state    <= MS_DONE;
                            end
                        end
                        MS_DONE: ;   // M3b: lfb_rf_done_set (combinational, below)
                                     // samples this state for a background refill;
                                     // the blocking FRZ's MS_DONE is held one cycle
                                     // by the outer FSM's own transition as before.
                        default: miss_state <= MS_IDLE;
                    endcase
                end else if (lfb_head_activatable) begin
                    // Kickoff: the LFB always block (above) moves
                    // lfb_state[head] E_PENDING->E_ACTIVE this same edge;
                    // miss_state starts the activation-time way_vld/dirty
                    // re-read next.
                    lfb_pf_drop_r <= 1'b0;   // fresh activation: no drop pending
                    miss_state <= MS_LFB_PEEK_ISSUE;
                end else begin
                    if (miss_state != MS_IDLE) miss_state <= MS_IDLE;
                end

            // FENCE.I D-cache clean walk's AXI writeback control: on the
            // peek-response cycle, launch a full-line write of the dirty way
            // just peeked; retire the write sub-sequence once the B response
            // lands. Mutually exclusive with every FRZ/IDLE use above (clean
            // only runs while state==ST_IDLE and issue_real/issue_drain are
            // gated off) AND with the VB drain below: clean can only launch
            // once lsu_cp0_stb_empty proved !vb_vld, and vb_vld cannot rise
            // during the walk (no new miss engine activity while
            // clean_active gates issue), so the CL_WB axi_w_done here is
            // unambiguously the clean walk's own write.
            if (clean_state == CL_PEEK_WAIT && u_dc_resp_vld) begin
                axi_w_active   <= 1'b1;
                axi_w_aw_sent  <= 1'b0;
                axi_w_w_sent   <= 1'b0;
                axi_w_addr_r   <= {{(ADDR_WIDTH - PC_WIDTH){1'b0}}, u_dc_resp_victim_tag, clean_set, 6'b0};
                axi_w_data_r   <= u_dc_resp_rdata;
                axi_w_strb_r   <= 64'hFFFF_FFFF_FFFF_FFFF;
                axi_w_awsize_r <= 3'd6;
            end
            if (clean_state == CL_WB && axi_w_done) begin
                axi_w_active  <= 1'b0;
                axi_w_aw_sent <= 1'b0;
                axi_w_w_sent  <= 1'b0;
            end

            // M3b Task B/C: VB drain -- the single-entry victim buffer's
            // writeback to memory (donor aq_lsu_vb.v FSM IDLE->DATA_0..3->
            // BUS_REQ/WFC with its dedicated AXI writeback, :267-354,:462-477;
            // rv906 adaptation: the shared axi_w_* registers stand in for the
            // donor's dedicated channel since the crossbar serializes all
            // transactions to the memory slave anyway). Claims the write
            // channel whenever the VB is occupied and the channel is free;
            // retires on the B response. Mutual exclusion with the other two
            // axi_w claimants: the FRZ direct-store path only claims when
            // !vb_vld (see MS_IDLE above) and the clean walk only runs with
            // vb_vld==0 (see comment above), so while vb_vld==1 the VB owns
            // any axi_w_active transaction and axi_w_done here is its own.
            if (vb_vld && !axi_w_active) begin
                axi_w_active   <= 1'b1;
                axi_w_aw_sent  <= 1'b0;
                axi_w_w_sent   <= 1'b0;
                axi_w_addr_r   <= {{(ADDR_WIDTH - PC_WIDTH){1'b0}}, vb_tag_r, vb_index_r, 6'b0};
                axi_w_data_r   <= vb_data_r;
                axi_w_strb_r   <= 64'hFFFF_FFFF_FFFF_FFFF;
                axi_w_awsize_r <= 3'd6;
            end
            if (vb_vld && axi_w_active && axi_w_done) begin
                axi_w_active  <= 1'b0;
                axi_w_aw_sent <= 1'b0;
                axi_w_w_sent  <= 1'b0;
                vb_vld        <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // SECTION CLEAN -- FENCE.I D-cache clean walk (donor aq_cp0_fence_inst.v
    // FNC_CDCA stage, Task 10.1). CSR launches it (cp0_lsu_dcache_clean held
    // high) only after fence_wait proved state==ST_IDLE && STB empty, and
    // IDU dispatch stays stalled (cp0_idu_fencei_full) for the whole walk, so
    // no ordinary access competes. Per set: one read returns way_vld/way_dirty;
    // each valid+dirty way is then peeked (way-select read), written back via
    // the shared AXI write sub-sequence (full 64B, awsize 6), and invalidated
    // through dc_inv_* before moving on. lsu_cp0_clean_done pulses on the
    // final invalidate's done.
    //-------------------------------------------------------------------------
    localparam [2:0] CL_IDLE       = 3'd0;
    localparam [2:0] CL_SET_READ   = 3'd1;
    localparam [2:0] CL_SET_WAIT   = 3'd2;
    localparam [2:0] CL_PEEK_ISSUE = 3'd3;
    localparam [2:0] CL_PEEK_WAIT  = 3'd4;
    localparam [2:0] CL_WB         = 3'd5;
    localparam [2:0] CL_INV        = 3'd6;
    localparam [2:0] CL_INV_WAIT   = 3'd7;

    reg  [2:0] clean_state;
    reg  [DCACHE_INDEX_W-1:0] clean_set;
    reg  [WAYS-1:0] clean_todo;      // ways of clean_set still to write back

    wire clean_active   = (clean_state != CL_IDLE);
    wire [1:0] clean_way_idx = clean_todo[0] ? 2'd0 : clean_todo[1] ? 2'd1
                             : clean_todo[2] ? 2'd2 : 2'd3;
    wire [WAYS-1:0] clean_way_oh = 4'b0001 << clean_way_idx;
    wire clean_last_way = (clean_todo == clean_way_oh);

    wire clean_issue_setrd = (clean_state == CL_SET_READ);
    wire clean_issue_peek  = (clean_state == CL_PEEK_ISSUE);
    wire clean_req         = clean_issue_setrd || clean_issue_peek;
    wire clean_inv_fire    = (clean_state == CL_INV);

    // Completion pulse: the walk ends on the LAST set via EITHER exit -- an
    // empty set (CL_SET_WAIT -> CL_IDLE) or the last dirty way's invalidate
    // (CL_INV_WAIT -> CL_IDLE). Both must pulse lsu_cp0_clean_done, else a
    // final set with no dirty ways would never notify CSR (fence.i wedge).
    wire clean_finish_empty_set = (clean_state == CL_SET_WAIT)
                                && ((u_dc_resp_way_vld & u_dc_resp_way_dirty) == {WAYS{1'b0}})
                                && (&clean_set);
    wire clean_finish_last_inv  = (clean_state == CL_INV_WAIT) && u_dc_inv_done
                                && clean_last_way && (&clean_set);
    assign lsu_cp0_clean_done = clean_finish_empty_set || clean_finish_last_inv;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clean_state <= CL_IDLE;
            clean_set   <= {DCACHE_INDEX_W{1'b0}};
            clean_todo  <= {WAYS{1'b0}};
        end else case (clean_state)
            CL_IDLE: begin
                // M4 Task 5: also wait for the PTW servant to be idle -- it
                // occupies the array/AXI-read port for a multi-cycle walk
                // while `state` stays ST_IDLE throughout (SECTION PTW
                // SERVANT's own gating only stops IT from STARTING during
                // an active clean walk; this is the other direction).
                if (cp0_lsu_dcache_clean && (state == ST_IDLE) && !any_stb_vld
                    && (ptw_sv_state == PTW_SV_IDLE)) begin
                    clean_state <= CL_SET_READ;
                    clean_set   <= {DCACHE_INDEX_W{1'b0}};
                end
            end
            CL_SET_READ: clean_state <= CL_SET_WAIT;
            CL_SET_WAIT: begin
                if ((u_dc_resp_way_vld & u_dc_resp_way_dirty) == {WAYS{1'b0}}) begin
                    if (&clean_set) clean_state <= CL_IDLE;
                    else begin
                        clean_set   <= clean_set + {{(DCACHE_INDEX_W-1){1'b0}}, 1'b1};
                        clean_state <= CL_SET_READ;
                    end
                end else begin
                    clean_todo  <= u_dc_resp_way_vld & u_dc_resp_way_dirty;
                    clean_state <= CL_PEEK_ISSUE;
                end
            end
            CL_PEEK_ISSUE: clean_state <= CL_PEEK_WAIT;
            CL_PEEK_WAIT:  if (u_dc_resp_vld) clean_state <= CL_WB;
            CL_WB:         if (axi_w_done)    clean_state <= CL_INV;
            CL_INV:        clean_state <= CL_INV_WAIT;
            CL_INV_WAIT: begin
                if (u_dc_inv_done) begin
                    clean_todo <= clean_todo & ~clean_way_oh;
                    if (!clean_last_way)
                        clean_state <= CL_PEEK_ISSUE;
                    else if (&clean_set)
                        clean_state <= CL_IDLE;
                    else begin
                        clean_set   <= clean_set + {{(DCACHE_INDEX_W-1){1'b0}}, 1'b1};
                        clean_state <= CL_SET_READ;
                    end
                end
            end
            default: clean_state <= CL_IDLE;
        endcase
    end

    // FRZ's own DCache.v traffic (victim-peek read, refill-commit write) --
    // combinational, mutually exclusive with the IDLE-cycle issue mux above
    // (this module is single-outstanding: the main FSM is in FRZ, not
    // IDLE, whenever any of this fires).
    // M3b: the background LFB refill (main FSM back at ST_IDLE) drives the
    // same vpeek/commit port phases as the blocking FRZ. The blocking FRZ
    // (state==ST_FRZ) is already serialized by the FSM; the BACKGROUND refill
    // must wait until the main FSM is at ST_IDLE (no hit-lookup in flight)
    // -- OR the main FSM is ST_DCS but PARKED in dc_wait_lfb_r, which reads
    // no live u_dc_resp_*/dc_hit_c wire at all (see the ST_DCS case) and so
    // cannot collide with this traffic -- before driving the port, else its
    // vpeek/commit would collide with a concurrent hit-lookup still using
    // the shared response bus. rf_port_busy blocks NEW issues while the
    // background refill owns the port.
    wire frz_issue_vpeek  = (miss_state == MS_VPEEK_ISSUE)
                            && ((state == ST_FRZ)
                                || (lfb_engine_active && (state == ST_IDLE || dc_wait_lfb_r)));
    wire frz_issue_commit = (miss_state == MS_COMMIT_ISSUE)
                            && ((state == ST_FRZ)
                                || (lfb_engine_active && (state == ST_IDLE || dc_wait_lfb_r)));
    // Same anti-collision gate for the activation-time way_vld/way_dirty
    // re-read (MS_LFB_PEEK_ISSUE) -- lfb_engine_active is already guaranteed
    // true whenever miss_state reaches this value (see the LFB always
    // block). rf_port_busy (below) reserves this same state==ST_IDLE cycle
    // via lfb_head_activatable one cycle earlier, so state is guaranteed to
    // still be ST_IDLE (or dc_wait_lfb_r) the cycle miss_state actually
    // becomes MS_LFB_PEEK_ISSUE -- no new op can have raced in and stolen it.
    wire frz_issue_lfb_peek = (miss_state == MS_LFB_PEEK_ISSUE)
                              && (state == ST_IDLE || dc_wait_lfb_r);
    // While a background refill is using the D-cache port (activation peek,
    // vpeek/commit issue+response), a new op's lookup must not collide with it.
    // lfb_head_activatable is included directly (not ANDed with
    // lfb_engine_active, which is still false the cycle activation fires):
    // without it, a new op's issue_real races the E_PENDING->E_ACTIVE
    // kickoff on the very same ST_IDLE cycle (both gated on state==ST_IDLE),
    // stealing ST_IDLE away before frz_issue_lfb_peek -- which only samples
    // state==ST_IDLE||dc_wait_lfb_r one cycle later, once miss_state has
    // already become MS_LFB_PEEK_ISSUE -- ever gets a chance to fire,
    // wedging the engine at MS_LFB_PEEK_ISSUE forever.
    wire rf_port_busy = lfb_head_activatable
                        || (lfb_engine_active && (miss_state == MS_LFB_PEEK_ISSUE
                        || miss_state == MS_LFB_PEEK_WAIT || miss_state == MS_VPEEK_ISSUE
                        || miss_state == MS_VPEEK_WAIT || miss_state == MS_COMMIT_ISSUE
                        || miss_state == MS_COMMIT_WAIT));

    // M4 Task 5: ptw_sv_probe_fire adds a fourth array requester (SECTION
    // PTW SERVANT, above) -- a plain read (way_sel=0, wr=0, alloc=0), so it
    // needs an explicit arm only where the existing fallback (ag_is_store,
    // driven by whatever is live in IDU's EX1 register regardless of
    // whether THIS cycle's array use is even a real instruction's) would
    // otherwise leak a stale write. ptw_sv_probe_fire is mutually exclusive
    // with frz_issue_*/clean_req/issue_drain by construction (its own
    // ptw_sv_idle_free gate), so it only needs to beat the ag_is_store
    // fallback, not the other arms.
    assign u_dc_req_vld       = touches_array || frz_issue_lfb_peek || frz_issue_vpeek || frz_issue_commit || clean_req || ptw_sv_probe_fire;
    assign u_dc_req_way_sel   = frz_issue_vpeek ? victim_way_r : (frz_issue_commit ? victim_way_r
                                : clean_issue_peek ? clean_way_oh
                                : (issue_drain ? stb_way[drain_pick] : {WAYS{1'b0}}));
    assign u_dc_req_wr        = frz_issue_commit ? 1'b1 : ((frz_issue_lfb_peek || frz_issue_vpeek) ? 1'b0
                                : (ptw_sv_probe_fire ? 1'b0
                                : (issue_drain ? 1'b1 : ag_is_store)));
    assign u_dc_req_alloc     = frz_issue_commit;
    assign u_dc_req_wdata      = frz_issue_commit ? frz_rdata_r
                                : (issue_drain ? ({448'b0, stb_data[drain_pick]} << ({58'b0, stb_dw_off[drain_pick]} * 64))
                                : {448'b0, ag_store_data_positioned} << ({58'b0, ag_dw_off} * 64));
    assign u_dc_req_wstrb      = frz_issue_commit ? 64'hFFFF_FFFF_FFFF_FFFF
                                : (issue_drain ? ({56'b0, stb_byte_vld[drain_pick]} << ({58'b0, stb_dw_off[drain_pick]} * 8))
                                : ({56'b0, ag_byte_mask} << ({58'b0, ag_dw_off} * 8)));
    assign u_dc_req_dirty_set  = frz_issue_commit ? 1'b0 : (ptw_sv_probe_fire ? 1'b0
                                : (issue_drain ? 1'b1 : ag_is_store));
    assign u_dc_req_index      = frz_issue_lfb_peek || frz_issue_vpeek || frz_issue_commit ? frz_eff_index
                                : clean_req ? clean_set
                                : ptw_sv_probe_fire ? ptw_sv_req_index
                                : (issue_drain ? stb_index[drain_pick] : ag_dc_index);
    assign u_dc_req_tag        = frz_issue_lfb_peek || frz_issue_vpeek || frz_issue_commit ? frz_eff_tag
                                : ptw_sv_probe_fire ? ptw_sv_req_tag
                                : (issue_drain ? stb_tag[drain_pick] : ag_dc_tag);

    //-------------------------------------------------------------------------
    // SECTION AXI D-side master (design doc S2.1) -- a single shared write
    // sub-sequence and a single shared read sub-sequence, reused for every
    // one of: victim writeback, refill read, uncached load/store, wa=0
    // store-miss direct write, direct STB drain. Always one 64B-aligned
    // single beat (contract 17), byte-selected via wstrb on writes; reads
    // always fetch the full aligned beat and the consumer (DA) picks out
    // the doubleword/bytes it actually needs -- a deliberate simplification
    // (this test harness's memory model has no read side effects to
    // protect against) documented, not accidental.
    //-------------------------------------------------------------------------
    assign axi_d_awvalid = axi_w_active && !axi_w_aw_sent;
    assign axi_d_awaddr  = axi_w_addr_r;
    assign axi_d_awlen   = 8'd0;
    // AW size is per-transaction: a store-miss direct write / STB drain must
    // advertise the STORE size (sb->0, sh->1, sw->2, sd->3) so the AXI memory
    // model writes exactly the stored bytes; a victim writeback is a full
    // 64-byte line (awsize 6). Hardcoding 6 here (the old code) made a sub-word
    // store-miss write the whole 64-byte beat, clobbering neighbours.
    assign axi_d_awsize  = axi_w_awsize_r;
    assign axi_d_awburst = 2'b01;
    assign axi_d_awcache = 4'd0;
    assign axi_d_awprot  = 3'd0;
    assign axi_d_wvalid  = axi_w_active && !axi_w_w_sent;
    assign axi_d_wdata   = axi_w_data_r;
    assign axi_d_wstrb   = axi_w_strb_r;
    assign axi_d_wlast   = axi_d_wvalid;
    assign axi_d_bready  = 1'b1;

    // M4 Task 5: ptw_sv_axi_active (SECTION PTW SERVANT, above) shares this
    // single physical AR/R channel with the FRZ/LFB refill's own axi_r_*
    // read sub-sequence. The two are mutually exclusive BY CONSTRUCTION,
    // never arbitrated: ptw_sv only starts a walk while state==ST_IDLE &&
    // !lfb_any_vld (ptw_sv_idle_free, above), and lfb_any_vld=1 for the
    // WHOLE of any FRZ/LFB refill's own AXI-read phase (an LFB entry is
    // occupied from activation through completion) -- so axi_r_active and
    // ptw_sv_axi_active can never both be 1 the same cycle. A plain OR-mux
    // is therefore exact, not an arbitration choice.
    assign axi_d_arvalid = (axi_r_active && !axi_r_ar_sent)
                          || (ptw_sv_axi_active && !ptw_sv_ar_sent);
    assign axi_d_araddr  = axi_r_active ? axi_r_addr_r : ptw_sv_line_addr;
    assign axi_d_arlen   = 8'd0;
    assign axi_d_arsize  = 3'd6;
    assign axi_d_arburst = 2'b01;
    assign axi_d_arcache = 4'd0;
    assign axi_d_arprot  = 3'd0;
    assign axi_d_rready  = 1'b1;

    //-------------------------------------------------------------------------
    // SECTION DA (LSU note A5) -- byte rotate + sign/zero-extend, merged
    // against a byte-granular STB forward (LSU note A4). Combinational,
    // read only during REPLY.
    //-------------------------------------------------------------------------
    wire [60:0] rp_dword = dc_addr_r[63:3];
    wire stb_m0_rp = stb_vld[0] && (stb_addr[0][63:3] == rp_dword);
    wire stb_m1_rp = stb_vld[1] && (stb_addr[1][63:3] == rp_dword);
    wire stb_m2_rp = stb_vld[2] && (stb_addr[2][63:3] == rp_dword);
    wire stb_m3_rp = stb_vld[3] && (stb_addr[3][63:3] == rp_dword);

    wire [63:0] stb_fwd_data = stb_m0_rp ? stb_data[0] : stb_m1_rp ? stb_data[1]
                             : stb_m2_rp ? stb_data[2] : stb_m3_rp ? stb_data[3] : 64'd0;
    wire [7:0]  stb_fwd_mask = stb_m0_rp ? stb_byte_vld[0] : stb_m1_rp ? stb_byte_vld[1]
                             : stb_m2_rp ? stb_byte_vld[2] : stb_m3_rp ? stb_byte_vld[3] : 8'd0;
    wire [63:0] stb_fwd_bits  = expand_byte_mask(stb_fwd_mask);
    // Task 7.3 width-hygiene: dword offset -> bit offset, computed at
    // exactly 9 bits (the width this part-select's base requires; max
    // 7*64 = 448) instead of the original formally-122-bit product.
    //
    // Source-select on the line data: at ST_DCS (the DC stage) the line
    // is still in the DCache's combinational response (u_dc_resp_rdata) --
    // dc_rdata_r is only latched at the END of ST_DCS, so it is not yet
    // this transaction's data during ST_DCS. The donor forwards the load
    // data to the IDU at the DC stage (aq_lsu_dc.v: "LSU int data forward
    // to IDU in DC stage"), so da_final must be computed from the live
    // response here; at ST_REPLY (RT) the latched dc_rdata_r is used for
    // the register writeback. Both carry the same value (dc_rdata_r is
    // latched from u_dc_resp_rdata at the ST_DCS->ST_REPLY edge).
    wire [511:0] dc_rdata_src = (state == ST_DCS) ? u_dc_resp_rdata : dc_rdata_r;
    wire [8:0]  dc_dword_bitoff = {6'b0, dc_dw_off_r} << 6;
    wire [63:0] raw_dword       = dc_rdata_src[dc_dword_bitoff +: 64];
    wire [63:0] merged_dword  = (stb_fwd_bits & stb_fwd_data) | (~stb_fwd_bits & raw_dword);

    // Rotate-by-byte via the double-width-shift idiom; the explicit [63:0]
    // is the same slice the 128->64 assignment took implicitly (Task 7.3).
    wire [127:0] merged2     = {merged_dword, merged_dword};
    wire [127:0] merged2_rsh = merged2 >> ({61'b0, dc_byte_off_r} * 8);
    wire [63:0] rotated     = merged2_rsh[63:0];

    reg [63:0] da_final;
    always @* begin
        case ({dc_sign_ext_r, dc_size_r})
            3'b0_00: da_final = {56'b0, rotated[7:0]};
            3'b1_00: da_final = {{56{rotated[7]}}, rotated[7:0]};
            3'b0_01: da_final = {48'b0, rotated[15:0]};
            3'b1_01: da_final = {{48{rotated[15]}}, rotated[15:0]};
            3'b0_10: da_final = {32'b0, rotated[31:0]};
            3'b1_10: da_final = {{32{rotated[31]}}, rotated[31:0]};
            default: da_final = rotated;
        endcase
    end

    //-------------------------------------------------------------------------
    // M3b Task A -- deferred-load completion data. A deferred load completes
    // from the refill line (frz_rdata_r, captured at MS_REFILL_READ) merged
    // with any STB store forward for its address -- no second cache probe, so
    // no eviction race. Mirrors the DA byte-rotate + sign/zero-extend above
    // but keyed on the LFB's captured attributes.
    //-------------------------------------------------------------------------
    wire [60:0] lfb_rp_dword = lfb_addr[lfb_head_idx][63:3];
    wire lfb_stb_m0 = stb_vld[0] && (stb_addr[0][63:3] == lfb_rp_dword);
    wire lfb_stb_m1 = stb_vld[1] && (stb_addr[1][63:3] == lfb_rp_dword);
    wire lfb_stb_m2 = stb_vld[2] && (stb_addr[2][63:3] == lfb_rp_dword);
    wire lfb_stb_m3 = stb_vld[3] && (stb_addr[3][63:3] == lfb_rp_dword);
    wire [63:0] lfb_stb_fwd_data = lfb_stb_m0 ? stb_data[0] : lfb_stb_m1 ? stb_data[1]
                                 : lfb_stb_m2 ? stb_data[2] : lfb_stb_m3 ? stb_data[3] : 64'd0;
    wire [7:0]  lfb_stb_fwd_mask = lfb_stb_m0 ? stb_byte_vld[0] : lfb_stb_m1 ? stb_byte_vld[1]
                                 : lfb_stb_m2 ? stb_byte_vld[2] : lfb_stb_m3 ? stb_byte_vld[3] : 8'd0;
    wire [63:0] lfb_stb_fwd_bits = expand_byte_mask(lfb_stb_fwd_mask);

    // lfb_dword_bitoff is declared above (needed by the LFB always block to
    // capture lfb_rdata_r at lfb_rf_done_set time, ahead of this point).
    wire [63:0] lfb_raw_dword    = lfb_rdata_r;
    wire [63:0] lfb_merged_dword = (lfb_stb_fwd_bits & lfb_stb_fwd_data)
                                 | (~lfb_stb_fwd_bits & lfb_raw_dword);
    wire [127:0] lfb_merged2     = {lfb_merged_dword, lfb_merged_dword};
    wire [127:0] lfb_merged2_rsh = lfb_merged2 >> ({61'b0, lfb_byte_off[lfb_head_idx]} * 8);
    wire [63:0]  lfb_rotated     = lfb_merged2_rsh[63:0];

    reg [63:0] lfb_wb_data;
    always @* begin
        case ({lfb_sign_ext[lfb_head_idx], lfb_size[lfb_head_idx]})
            3'b0_00: lfb_wb_data = {56'b0, lfb_rotated[7:0]};
            3'b1_00: lfb_wb_data = {{56{lfb_rotated[7]}}, lfb_rotated[7:0]};
            3'b0_01: lfb_wb_data = {48'b0, lfb_rotated[15:0]};
            3'b1_01: lfb_wb_data = {{48{lfb_rotated[15]}}, lfb_rotated[15:0]};
            3'b0_10: lfb_wb_data = {32'b0, lfb_rotated[31:0]};
            3'b1_10: lfb_wb_data = {{32{lfb_rotated[31]}}, lfb_rotated[31:0]};
            default: lfb_wb_data = lfb_rotated;
        endcase
    end
    //-------------------------------------------------------------------------
    // SECTION AMO ALU (M3 Task 4) -- combinational read-modify-write compute,
    // cloned from donor aq_lsu_amo_alu.v. src0 = register operand (the value
    // to combine), src1 = memory operand (the value read from memory). The
    // result is the NEW value to store back; the OLD value (src1) is what
    // gets written to the destination register.
    //-------------------------------------------------------------------------
    // Decode AMO op from the latched func (bits[8:4] carry the funct5). These
    // feed `issue_real && amo_is_amo` below, and issue_real fires on the
    // (possibly much later) cycle a DTLB-miss wait resolves -- by then the
    // live idu_lsu_ex1_func has drifted to whatever op IDU has since decoded
    // (IDU does not hold EX1 across the wait, see SECTION AG's ag_func_eff
    // comment). Must read the same buffer/live mux ag_is_amo_c already uses,
    // or a waited AMO silently loses its amo_active commit (mmu_amo.elf
    // TESTNUM 2: the RMW write-back never fires, ld reads back the
    // unmodified word).
    wire [4:0] amo_op   = ag_func_eff[8:4];
    // Width lives in func[3:2] (same field ag_size uses for the read-phase
    // access size): 10=W, 11=D.
    wire       amo_wd   = (ag_func_eff[3:2] == 2'b10);  // .W
    wire       amo_dw   = (ag_func_eff[3:2] == 2'b11);  // .D
    wire       amo_is_amo = (ag_func_eff[19:12] == 8'h01);  // AMO func prefix

    wire amo_add  = (amo_op == 5'b00000);
    wire amo_swap = (amo_op == 5'b00001);
    wire amo_xor  = (amo_op == 5'b00100);
    wire amo_and  = (amo_op == 5'b01100);
    wire amo_or   = (amo_op == 5'b01000);
    wire amo_min  = (amo_op == 5'b10000);
    wire amo_minu = (amo_op == 5'b11000);
    wire amo_max  = (amo_op == 5'b10100);
    wire amo_maxu = (amo_op == 5'b11100);

    // AMO ALU compute (donor aq_lsu_amo_alu.v semantics):
    // src0 = MEMORY operand (OLD value read from memory), src1 = REGISTER
    // operand (rs1, the value to combine). The result is the NEW value to
    // store back; the OLD value (src0) is what gets written to the dest reg.
    function automatic [63:0] amo_alu_compute(
        input [63:0] src0,      // memory operand (OLD value)
        input [63:0] src1,      // register operand (rs1)
        input [4:0]  op,
        input        is_dw      // 1=D-width, 0=W-width
    );
        reg signed [64:0] s0_ext, s1_ext;
        reg        adder_cin;
        reg [63:0] add_rst, logic_rst, sel_rst;
        reg        use_add, use_logic, use_sel;
        reg        src0_sel;
        reg        is_unsigned;
        begin
            // min/max are SIGNED compares; minu/maxu are UNSIGNED (donor's
            // unsign_ext selects zero- vs sign-extension the same way).
            is_unsigned = (op[4:0] == 5'b11000 || op[4:0] == 5'b11100);
            if (is_dw) begin
                s0_ext = is_unsigned ? {1'b0, src0[63:0]} : {src0[63], src0[63:0]};
                s1_ext = is_unsigned ? {1'b0, src1[63:0]} : {src1[63], src1[63:0]};
            end else begin
                s0_ext = is_unsigned ? {33'b0, src0[31:0]}
                                     : {{33{src0[31]}}, src0[31:0]};
                s1_ext = is_unsigned ? {33'b0, src1[31:0]}
                                     : {{33{src1[31]}}, src1[31:0]};
            end
            // Adder: used for add
            add_rst = src0 + src1;
            // Compare for min/max: adder_cin = (src0 < src1). With this
            // polarity the shared select below yields: min->smaller, max->larger.
            adder_cin = (s0_ext < s1_ext);
            src0_sel  = ((op[4:0] == 5'b10100 || op[4:0] == 5'b11100) ^ adder_cin)
                        && (op[4:0] != 5'b00001);  // max/maxu select, not swap
            sel_rst   = src0_sel ? src0 : src1;
            // Logic ops
            case (op[4:0])
                5'b01100: logic_rst = src0 & src1;   // and
                5'b00100: logic_rst = src0 ^ src1;   // xor
                5'b01000: logic_rst = src0 | src1;   // or
                default:  logic_rst = 64'd0;
            endcase
            use_add    = (op[4:0] == 5'b00000);                          // add
            use_logic  = (op[4:0] == 5'b01100 || op[4:0] == 5'b00100 || op[4:0] == 5'b01000);
            use_sel    = (op[4:0] == 5'b00001 || op[4:0] == 5'b10000 ||
                          op[4:0] == 5'b11000 || op[4:0] == 5'b10100 || op[4:0] == 5'b11100);
            if (use_add)         amo_alu_compute = add_rst;
            else if (use_logic)  amo_alu_compute = logic_rst;
            else if (use_sel)    amo_alu_compute = sel_rst;
            else                 amo_alu_compute = 64'd0;
        end
    endfunction

    //-------------------------------------------------------------------------
    // SECTION REPLY completion -- STB create-or-merge for a completing
    // store; the RTU completion/writeback/exception bus.
    //-------------------------------------------------------------------------
    wire stb_match_here = stb_m0_rp || stb_m1_rp || stb_m2_rp || stb_m3_rp;
    wire [1:0] stb_match_idx = stb_m0_rp ? 2'd0 : stb_m1_rp ? 2'd1 : stb_m2_rp ? 2'd2 : 2'd3;
    wire stb_any_free = !stb_vld[0] || !stb_vld[1] || !stb_vld[2] || !stb_vld[3];
    wire [1:0] stb_free_idx = !stb_vld[0] ? 2'd0 : !stb_vld[1] ? 2'd1 : !stb_vld[2] ? 2'd2 : 2'd3;

    wire reply_is_completing_store = (state == ST_REPLY) && !dc_is_drain_r && dc_is_store_r && !dc_misalign_r;
    wire reply_store_needs_new_slot = reply_is_completing_store && !stb_match_here;
    // M3: a SUCCESSFUL SC commits its store data through the STB exactly
    // like an ordinary store completion. SC is load-like (func[0]=0) so
    // reply_is_store never sees it; give it a parallel term here (and in
    // the STB create-or-merge below). Needs-a-slot backpressure included:
    // a full STB holds the SC in REPLY until a drain frees a slot, same
    // contract-4 behavior as a store.
    wire reply_is_completing_sc = (state == ST_REPLY) && !dc_is_drain_r
                                  && sc_addr_set && sc_match_r && !dc_misalign_r;
    wire reply_sc_needs_new_slot = reply_is_completing_sc && !stb_match_here;
    // M3 audit: same backpressure for the AMO writeback (now also created
    // at REPLY, see reply_is_amo_commit below). With the STB-full admission
    // control at SECTION drain-vs-issue a slot-needing op is held in EX1
    // until a slot (or merge target) exists, so these REPLY holds are
    // defense-in-depth -- but they keep the guarantee local.
    wire reply_is_completing_amo = (state == ST_REPLY) && !dc_is_drain_r
                                   && amo_active && !dc_misalign_r;
    wire reply_amo_needs_new_slot = reply_is_completing_amo && !stb_match_here;
    // A genuinely full STB (no match, no free slot) stalls the completing
    // store/SC/AMO in REPLY until a drain frees a slot -- contract 4's
    // DEPTH=4 backpressure, not a cancellation.
    wire reply_can_complete = !((reply_store_needs_new_slot || reply_sc_needs_new_slot
                                 || reply_amo_needs_new_slot) && !stb_any_free);

    wire [WAYS-1:0] final_way = dc_hit_r ? dc_hit_way_r : victim_way_r;

    // Whether the line THIS store targets is resident in the array by the
    // time REPLY fires -- true for an ordinary hit, ALSO true for a
    // cacheable wa=1 miss (it just got refilled+committed via FRZ before
    // REPLY), false only for an uncached access or a wa=0 store-miss
    // (direct-AXI bypass, never allocated). Deliberately NOT the same as
    // `dc_hit_r` (which reflects only the ORIGINAL, pre-refill tag
    // compare) -- using raw `dc_hit_r` here was a real bug found while
    // writing lsu_tb.cpp: a wa=1 store-miss would otherwise mark its own
    // STB entry `was_hit=0` and drain via the direct-AXI bypass instead of
    // writing into the way that was JUST allocated for it.
    // M3b Task E: derive residency from the LATCHED FRZ decision
    // (frz_is_direct_r=1 for a non-allocating store miss) instead of
    // recomputing dc_wa_eff here: the AMR's FUNC state could change between
    // this store's DC decision and its REPLY, and stb_was_hit MUST agree
    // with the path the store actually took (refill-allocated vs direct
    // write), else a later STB drain would write the cache for a line that
    // was never allocated (or skip a line that was).
    wire store_line_resident = dc_hit_r || (dc_ca_r && !frz_is_direct_r);

    wire reply_fire        = (state == ST_REPLY) && reply_can_complete && !dc_is_drain_r;
    wire reply_is_load      = reply_fire && !dc_is_store_r;
    wire reply_is_store      = reply_fire && dc_is_store_r;
    wire reply_is_misalign  = reply_fire && dc_misalign_r;
    // M3: successful SC at REPLY -- gates the STB store-commit below.
    // reply_fire carries reply_can_complete, so a full STB holds the SC
    // here (no entry is created until a drain frees a slot).
    wire reply_is_sc_commit = reply_fire && sc_addr_set && sc_match_r && !dc_misalign_r;
    // M3 audit: the AMO writeback commits at REPLY too (previously a
    // deferred next-cycle flag, which a full STB silently dropped).
    wire reply_is_amo_commit = reply_fire && amo_active && !dc_misalign_r;
    // Commit payload mux (store / SC use the positioned store-data path;
    // the AMO brings its computed NEW value, positioned here -- the AMO ALU
    // works in the extracted value domain, the STB holds positioned data):
    wire [63:0] commit_data = reply_is_amo_commit
                              ? (amo_new_c << ({61'b0, dc_byte_off_r} * 8))
                              : dc_store_data_r;
    wire [7:0]  commit_mask = reply_is_amo_commit
                              ? (amo_is_dw_r ? 8'hFF : (8'h0F << dc_byte_off_r))
                              : dc_byte_mask_r;
    wire [2:0]  commit_size = reply_is_amo_commit
                              ? (amo_is_dw_r ? 3'd3 : 3'd2)
                              : {1'b0, dc_size_r};
    // M3: SC detection -- latch the reservation match at ST_DCS entry
    // (dc_addr_r stable). A successful SC commits its store via the STB
    // (REPLY section below); rd gets 0=committed / 1=failed.
    reg        sc_addr_set;
    reg [FUNC_WIDTH-1:0] sc_func_r;  // latched SC opcode
    reg        sc_match_r;           // SC success latched at ST_DCS

    wire sc_func_is_sc = (sc_func_r == LSU_FUNC_SC_W) || (sc_func_r == LSU_FUNC_SC_D);
    // ag_func_eff, not idu_lsu_ex1_func: issue_real's cycle can trail a
    // waited SC's own EX1 cycle, by which point the live wire has drifted
    // (same bug class as amo_is_amo/the LR check above).
    wire sc_issue      = issue_real && ((ag_func_eff == LSU_FUNC_SC_W)
                                        || (ag_func_eff == LSU_FUNC_SC_D));
    // Combinational match at ST_DCS -- built from exactly the inputs the
    // latch samples that same cycle (sc_addr_set itself is an NBA, so it
    // is still 0 during the first DCS cycle). The DC-stage forward below
    // needs the result THIS cycle, one cycle before sc_match_r exists.
    // Donor aq_lsu_lm.v:159-161 matches on address AND access size
    // (lm_size == lm_req_size), so a LR.W;SC.D to one address fails.
    wire sc_match_c    = lr_valid_r && (dc_addr_r == lr_addr_r)
                         && (dc_size_r == lr_size_r);

    always @(posedge clk) begin
        if (!rst_n) begin
            sc_addr_set <= 1'b0;
            sc_func_r <= 20'd0;
            sc_match_r <= 1'b0;
        end else begin
            // Latch opcode at issue_real -- ag_func_eff (see sc_issue above).
            if (sc_issue) begin
                sc_func_r <= ag_func_eff;
            end
            // Latch the reservation match at ST_DCS entry. SC succeeds only
            // if there is a VALID reservation (lr_valid_r) AND the SC address
            // matches the reserved address. Without a reservation (lr_valid_r=0)
            // the SC must FAIL (return 1) -- rv64ua-p-lrsc test 2.
            if (state == ST_DCS && sc_func_is_sc && !sc_addr_set) begin
                sc_addr_set <= 1'b1;
                sc_match_r  <= sc_match_c;
            end
            // Clear after completion
            if (lsu_rtu_ex1_cmplt_dp && sc_addr_set) begin
                sc_addr_set <= 1'b0;
                sc_func_r <= 20'd0;
                sc_match_r <= 1'b0;
            end
        end
    end

    // SC result: 0=success(commit), 1=fail - latched at ST_DCS (sc_match_r).
    assign lsu_rtu_sc_res     = sc_addr_set ? (sc_match_r ? 5'd0 : 5'd1) : 5'd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stb_vld[0] <= 1'b0; stb_vld[1] <= 1'b0; stb_vld[2] <= 1'b0; stb_vld[3] <= 1'b0;
        end else begin
            // drain retirement
            if ((state == ST_REPLY) && dc_is_drain_r && (!frz_is_direct_r || miss_done_latched))
                stb_vld[dc_drain_idx_r] <= 1'b0;
            // store completion: merge into an existing entry, or allocate.
            // M3: a successful SC (reply_is_sc_commit) and an AMO
            // (reply_is_amo_commit) commit here too -- both are load-like
            // on the pipe, so without these terms their store data would
            // never reach memory (rv64ua-p-lrsc test 5). Commit payload is
            // muxed per op (commit_data/commit_mask/commit_size above).
            //
            // MERGE is required for all three (not just stores): the STB's
            // invariant is at-most-one-entry-per-doubleword. M3 audit: an
            // allocate-only AMO create broke it -- an older pending store
            // for the same dword at a higher index then drained AFTER the
            // AMO entry (drain_pick is lowest-first) and overwrote the AMO
            // result; while two entries shared the dword, the DA forward
            // also merged only the lowest-index entry's mask (stale reads).
            if ((reply_is_store && !reply_is_misalign) || reply_is_sc_commit
                || reply_is_amo_commit) begin
                if (stb_match_here) begin
                    stb_data[stb_match_idx]     <= (expand_byte_mask(commit_mask) & commit_data)
                                                  | (~expand_byte_mask(commit_mask) & stb_data[stb_match_idx]);
                    stb_byte_vld[stb_match_idx] <= stb_byte_vld[stb_match_idx] | commit_mask;
                    stb_way[stb_match_idx]      <= final_way;
                    // SC/AMO are load-like: a cacheable SC/AMO miss REFILLS
                    // (line resident afterwards, in victim_way_r), unlike a
                    // wa=0 store-miss which bypasses. dc_ca_r covers
                    // hit+refill exactly.
                    stb_was_hit[stb_match_idx]  <= (reply_is_sc_commit || reply_is_amo_commit)
                                                   ? dc_ca_r : store_line_resident;
                    // Merged entries keep the larger of the two store sizes
                    // (single-store entries -- the common case -- keep their
                    // exact size). A merged drain is an edge case; the data
                    // was already written by each store's own direct write.
                    stb_size[stb_match_idx]     <= (stb_size[stb_match_idx] > commit_size) ? stb_size[stb_match_idx] : commit_size;
                end else if (stb_any_free) begin
                    stb_vld[stb_free_idx]      <= 1'b1;
                    stb_addr[stb_free_idx]      <= dc_addr_r;
                    stb_index[stb_free_idx]     <= dc_index_r;
                    stb_tag[stb_free_idx]       <= dc_tag_r;
                    stb_dw_off[stb_free_idx]    <= dc_dw_off_r;
                    stb_data[stb_free_idx]      <= commit_data & expand_byte_mask(commit_mask);
                    stb_byte_vld[stb_free_idx]  <= commit_mask;
                    stb_way[stb_free_idx]       <= final_way;
                    stb_was_hit[stb_free_idx]   <= (reply_is_sc_commit || reply_is_amo_commit)
                                                   ? dc_ca_r : store_line_resident;
                    stb_size[stb_free_idx]      <= commit_size;
                end
            end
        end
    end

    // The commit-write's own dirty_set/alloc used dc_rdata_r as the fresh
    // line; whether THIS transaction was itself a store (wa=1 miss) is
    // handled uniformly: the refill always commits clean (dirty_set=0
    // above), and the subsequent STB-create (right above, in this SAME
    // REPLY cycle once miss handling has produced dc_hit_r via `final_way`)
    // is what marks the line dirty on drain -- exactly the same path an
    // ordinary store-hit takes.
    wire miss_done_latched = 1'b1;   // FRZ always fully drains before REPLY is entered

    assign lsu_rtu_ex1_cmplt_dp   = ((state == ST_REPLY) && reply_can_complete && !dc_is_drain_r)
                                    || (lfb_cmplt_fire && !lfb_pf[lfb_head_idx])   // M3b: deferred-load
                                    || misalign_issue   // M4: misalign traps at AG-issue
                                    || mmu_fault_issue;  // M4 Task 5: DTLB fault, same shape
                                    // completion; a PREFETCH entry drains silently (no instruction)
    assign lsu_rtu_ex1_cmplt      = lsu_rtu_ex1_cmplt_dp;
    // Task 9.7: the EARLY "for pcgen" completion (donor aq_lsu_ag.v:1675
    // ag_pipe_cmplt_normal ~ ag_pipe_inst_vld && dt_fsm_idle). In this
    // module that is exactly `issue_real`: a real (non-drain) instruction is
    // in AG (ag_valid) and the DC FSM is idle and about to accept it. Drains
    // are internal, not instructions, so they never advance the pcgen.
    //
    // M4 Task 5 (D1): a DTLB-miss wait needs NO special-case pulse here.
    // `issue_real = ag_issue_ready && mmu_lsu_pa_vld` is false throughout
    // the wait (that IS the miss) and fires exactly once, on the cycle the
    // walk resolves (successful translation or a page fault -- both are
    // `issue_real`; `mmu_fault_issue` is a subset of it) -- the SAME single
    // pulse IDU's own EX1 advance now lines up with, now that `lsu_idu_full`
    // correctly holds EX1 from the TRUE entry cycle onward (see the
    // `ag_raw_ready`/`lsu_idu_full` fix above). An earlier version of this
    // fix ALSO OR'd in an early `(ag_mmu_wait_start && !ag_wait_r)` pulse
    // here to compensate for EX1 drifting one cycle early -- that
    // compensating pulse is removed now that the root cause (the
    // entry-cycle gap in `lsu_idu_full`) is fixed directly: EX1 no longer
    // drifts, so pulsing early here would now be the bug (a spurious
    // advance of IU's bju_pcgen_pc one cycle before EX1 actually holds
    // still, double-counting against the later genuine `issue_real` pulse).
    assign lsu_rtu_ex1_cmplt_for_pcgen = issue_real;
    // Task 7.3: the completing LSU instruction's length (drains excluded by
    // the same !dc_is_drain_r the cmplt above already applies).
    // Donor aq_lsu_ag.v:1678 drives lsu_rtu_ex1_inst_len from the LIVE
    // AG-stage length (ag_pipe_inst_len), NOT a latched dc-stage copy:
    // lsu_rtu_ex1_cmplt_for_pcgen (= issue_real) fires on the SAME cycle
    // dc_inst_len_r would be latched, so reading the latch returns the
    // PREVIOUS transaction's length -- every load/store then advanced the
    // IU pcgen tracker by the wrong amount (rv64uc-p-rvc pcgen drift,
    // scrambled branch PCs from test 18 onward).
    assign lsu_rtu_ex1_inst_len   = ag_inst_len_eff;

    assign lsu_rtu_wb_vld  = (reply_is_load && !reply_is_misalign)
                             || (lfb_cmplt_fire && !lfb_pf[lfb_head_idx]);   // prefetch drains silently
    // W-width AMOs write back the OLD value sign-extended (da_final arrives
    // zero-extended since the AMO func leaves the sign bit clear); amo_active
    // is still asserted this ST_REPLY cycle (cleared by NBA at the edge).
    // For SC.W: the "load" result is actually the SC result (0=success, 1=fail).
    // Use sc_addr_set (not idu_lsu_ex1_func which may have moved on by ST_REPLY).
    // M3b: a deferred load returns lfb_wb_data (refill line + STB forward).
    assign lsu_rtu_wb_data = lfb_cmplt_fire ? lfb_wb_data
                                 : (sc_addr_set ? lsu_rtu_sc_res[4:0]
                                    : (amo_active
                                       ? (amo_is_dw_r ? da_final
                                                      : {{32{da_final[31]}}, da_final[31:0]})
                                       : da_final));
    assign lsu_rtu_wb_preg = lfb_cmplt_fire ? lfb_dst[lfb_head_idx] : dc_dst0_reg_r;

    // DC-stage forward for a cache hit: the line is available combinationally
    // at ST_DCS, so forward it there (one cycle before the ST_REPLY wb).
    wire lsu_fwd2_dc_fire = (state == ST_DCS) && u_dc_resp_vld && dc_touched_array_r
                            && dc_hit_c && !dc_is_store_r && !dc_misalign_r;

    // Donor aq_lsu_dc.v:2197-2210 (comment: "LSU int data forward to IDU
    // in DC stage"): the load data is forwarded to the IDU at the DC stage
    // (data_vld), ONE CYCLE BEFORE the RT register writeback. This is what
    // resolves the load->condbr RAW hazard the donor handles WITHOUT a stall
    // (aq_idu_id_ctrl.v RAW-except term 2, producer LSU + consumer condbr +
    // cnt in {0,1}): the consumer reads its operand at the same cycle the
    // producer is at DC, and picks up the data via this fwd2.
    //
    // A cache HIT presents its line combinationally at ST_DCS (u_dc_resp_rdata),
    // so the fwd fires there (lsu_fwd2_dc_fire). A cache MISS takes the FRZ
    // path and only has its data at ST_REPLY (dc_rdata_r, latched after the
    // refill); for that case the consumer is held by the pipeline (the LSU
    // stays busy across FRZ) and the fwd2 rides the ST_REPLY writeback
    // (lsu_rtu_wb_vld). OR-ing the two covers hit (DC-stage fwd) + miss
    // (RT-stage fwd) exactly as the donor's data_vld does across DC->REPLY.
    // M3 Task 7: the ex2 forward must carry the SAME sign-extended value as
    // the writeback for W-width AMOs (the consumer condbr reads via this
    // forward, not the GPR, when it dispatches the cycle the AMO writes back).
    // For SC the forward carries the SC result (0/1), not the memory read.
    // At the SC's FIRST ST_DCS cycle sc_addr_set is still 0 (NBA), but the
    // result is already resolvable combinationally (sc_match_c) -- forward it
    // here, otherwise a consumer dispatching this cycle would take da_final,
    // the stale memory value (rv64ua-p-lrsc test 2).
    wire sc_dcs_fire = (state == ST_DCS) && sc_func_is_sc && u_dc_resp_vld
                       && dc_touched_array_r && dc_hit_c && !dc_misalign_r;
    assign lsu_rtu_ex2_data      = lfb_cmplt_fire ? lfb_wb_data
                                 : sc_addr_set ? lsu_rtu_sc_res[4:0]
                                 : sc_dcs_fire ? (sc_match_c ? 5'd0 : 5'd1)
                                 : (amo_active
                                    ? (amo_is_dw_r ? da_final
                                                   : {{32{da_final[31]}}, da_final[31:0]})
                                    : da_final);
    assign lsu_rtu_ex2_data_vld  = lsu_fwd2_dc_fire || lsu_rtu_wb_vld;
    assign lsu_rtu_ex2_dest_reg  = lfb_cmplt_fire ? lfb_dst[lfb_head_idx] : dc_dst0_reg_r;

    // M3 Task 1: LR.W / SC.W foundation outputs
    // (lsu_rtu_sc_res is assigned near the SC-match latch above)

    assign lsu_rtu_expt_vld = misalign_issue || mmu_fault_issue
                             || (reply_fire && dc_misalign_r);
    // Misaligned SC/AMO/store take the STORE-misalign vector (cause 6 is
    // "Store/AMO address misaligned"); loads/LR take cause 4. At AG-issue the
    // DC-stage dc_is_store_r/sc_addr_set/amo_active are not yet latched, so
    // the trap-at-issue leg classifies off the live AG func (ag_is_amo_c /
    // ag_is_sc_c above); the reply leg (now unreachable for misalign but kept
    // structurally) uses the latched class.
    //
    // M4 Task 5 (S12, design doc S4.2): the MMU-fault vec ladder, same
    // AG-stage store/AMO/SC classification. Page fault outranks access
    // fault the same way the donor's own ladder orders them (donor
    // aq_lsu_ag.v:1381-1415: the page-fault arm is checked before the
    // access-fault arm) -- moot here in practice since MMU.v only ever
    // raises one of mmu_lsu_page_fault/_access_fault per access, but stated
    // rather than left to be discovered.
    wire mmu_fault_is_store = ag_is_store || ag_is_amo_c || ag_is_sc_c;
    wire [4:0] mmu_fault_vec = mmu_lsu_page_fault
                              ? (mmu_fault_is_store ? 5'd15 : 5'd13)
                              : (mmu_fault_is_store ? 5'd7  : 5'd5);
    assign lsu_rtu_expt_vec = misalign_issue
                            ? ((ag_is_store || ag_is_amo_c || ag_is_sc_c) ? 5'd6 : 5'd4)
                            : mmu_fault_issue
                            ? mmu_fault_vec
                            : ((dc_is_store_r || sc_addr_set || amo_active) ? 5'd6 : 5'd4);
    // tval = the faulting VA for both trap-at-issue exceptions (design doc
    // S4.2: "LSU encodes data causes... tval = faulting VA").
    assign lsu_rtu_tval     = (misalign_issue || mmu_fault_issue) ? ag_addr : dc_addr_r;

    // No async bus-error path is modeled for M2 (the behavioral AXI slave
    // in this test harness never returns a non-OKAY response) -- wired but
    // structurally never fires, same "landing pad" discipline RTU.v's own
    // header already established for its own dead legs.
    assign lsu_rtu_async_expt_vld = 1'b0;
    assign lsu_rtu_async_ld_inst  = 1'b0;

    // rtu_lsu_expt_ack/_expt_exit are consumed by the LR/SC reservation
    // clear (SECTION LR buffer, donor aq_lsu_lm.v:135).

endmodule
