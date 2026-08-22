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
    input  wire [FUNC_WIDTH-1:0]    idu_lsu_ex1_func,
    input  wire [63:0]              idu_lsu_ex1_src0_data, // base addr reg
    input  wire                     idu_lsu_ex1_src0_ready,
    input  wire [63:0]              idu_lsu_ex1_src1_data, // offset imm
    input  wire                     idu_lsu_ex1_src1_ready,
    input  wire [63:0]              idu_lsu_ex1_src2_data, // store data
    input  wire                     idu_lsu_ex1_src2_ready,
    input  wire [GPR_IDX_WIDTH-1:0] idu_lsu_ex1_dst0_reg,  // load dest

    //=========================================================================
    // LSU -> IDU : the single EX1 issue-gate stall signal (contract 8;
    // confirmed name, aq_idu_id_ctrl.v:634).
    //=========================================================================
    output wire                     lsu_idu_full,

    //=========================================================================
    // LSU -> RTU : lsu_rtu_t (design doc S4.2) -- matches RTU.v's input
    // group exactly.
    //=========================================================================
    output wire                     lsu_rtu_ex1_cmplt,
    output wire                     lsu_rtu_ex1_cmplt_dp,
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

    //=========================================================================
    // RTU -> LSU : "point of no return" acks (RTU note S6) -- also the
    // signals that gate the STB-create-vs-flush interlock contract 4
    // requires (verified cycle-by-cycle in Task 6, not assumed here).
    //=========================================================================
    input  wire                     rtu_lsu_expt_ack,
    input  wire                     rtu_lsu_expt_exit,

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

    //=========================================================================
    // CSR -> LSU : MHCR.de/wa + MXSTATUS.mm (matches CSR.v's output group
    // exactly).
    //=========================================================================
    input  wire                     cp0_lsu_dcache_en,
    input  wire                     cp0_lsu_mm,
    input  wire                     cp0_lsu_wa,

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
    // no return" pulses) are received but likewise have no live consumer
    // here: by the time either could fire for THIS instruction's own
    // exception, LSU has already fully resolved it (single-issue, single-
    // outstanding -- there is no younger, still-speculative LSU state left
    // to roll back), and `rtu_idu_flush_stall`'s own stall of IDU's ID/DIS
    // stage (not a port on this file) already prevents any wrong-path
    // instruction from ever reaching `idu_lsu_ex1_sel` during the drain
    // window. Flagged here, not silently guessed.
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
    wire        ag_is_store = idu_lsu_ex1_func[0];
    wire        ag_sign_ext = idu_lsu_ex1_func[1];
    wire [1:0]  ag_size     = idu_lsu_ex1_func[3:2];   // 00=B,01=H,10=W,11=D

    wire [63:0] ag_addr = idu_lsu_ex1_src0_data + idu_lsu_ex1_src1_data;

    wire ag_misalign = (ag_size == 2'b01 && ag_addr[0])
                     || (ag_size == 2'b10 && (|ag_addr[1:0]))
                     || (ag_size == 2'b11 && (|ag_addr[2:0]));
    // contract 3: `cp0_lsu_mm` is NEVER consulted here -- always trap.
    wire _cp0_lsu_mm_unused = cp0_lsu_mm;

    assign lsu_mmu_va        = ag_addr[MMU_VA_WIDTH-1:0];
    assign lsu_mmu_va_vld    = ag_valid;
    assign lsu_mmu_priv_mode = 2'b11;          // M-mode always (M2 has no other level)
    assign lsu_mmu_st_inst   = ag_is_store;

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
    wire [63:0] ag_store_data_positioned = idu_lsu_ex1_src2_data << ({61'b0, ag_byte_off} * 8);

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
    wire [1:0] drain_pick = stb_vld[0] ? 2'd0 : stb_vld[1] ? 2'd1 : stb_vld[2] ? 2'd2 : 2'd3;
    wire drain_want = !ag_valid && any_stb_vld && (state == ST_IDLE);

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

    // No D$-maintenance op is decoded anywhere in M2 (contract 10) -- this
    // module never drives dc_inv_vld. dcache_tb.cpp exercises DCache.v's
    // own invalidate mechanism directly, standalone, per plan 6.4.

    DCache u_dcache (
        .clk(clk), .rst_n(rst_n),
        .dc_req_vld(u_dc_req_vld), .dc_req_index(u_dc_req_index), .dc_req_tag(u_dc_req_tag),
        .dc_req_wr(u_dc_req_wr), .dc_req_way_sel(u_dc_req_way_sel), .dc_req_wdata(u_dc_req_wdata),
        .dc_req_wstrb(u_dc_req_wstrb), .dc_req_dirty_set(u_dc_req_dirty_set), .dc_req_alloc(u_dc_req_alloc),
        .dc_resp_vld(u_dc_resp_vld), .dc_resp_hit_way(u_dc_resp_hit_way), .dc_resp_rdata(u_dc_resp_rdata),
        .dc_resp_way_vld(u_dc_resp_way_vld), .dc_resp_way_dirty(u_dc_resp_way_dirty),
        .dc_resp_victim_tag(u_dc_resp_victim_tag),
        .dc_inv_vld(1'b0), .dc_inv_index({DCACHE_INDEX_W{1'b0}}), .dc_inv_way_sel({WAYS{1'b0}}),
        .dc_inv_done(u_dc_inv_done)
    );

    //-------------------------------------------------------------------------
    // SECTION IDLE-cycle issue mux: a real instruction (ag_valid) takes
    // priority over an opportunistic drain (drain_want), matching this
    // task's own "STB drains unconditionally... but must not starve the
    // main pipe" framing.
    //-------------------------------------------------------------------------
    wire issue_real  = (state == ST_IDLE) && ag_valid;   // misaligned accesses still
                                                           // enter the pipe (see below) --
                                                           // they just never touch the array.
    wire issue_drain = (state == ST_IDLE) && !ag_valid && drain_want;

    wire touches_array = issue_real  ? (mmu_lsu_ca && !ag_misalign)
                        : issue_drain ? stb_was_hit[drain_pick]
                        : 1'b0;

    //-------------------------------------------------------------------------
    // SECTION main FSM sequencing + AG/DCS latches.
    //-------------------------------------------------------------------------
    wire dc_hit_c        = |u_dc_resp_hit_way;
    wire store_wa_miss_c = dc_is_store_r && !dc_hit_c && !dc_wa_r;   // contract 6: wa=0 default

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            dc_is_store_r   <= 1'b0;
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
        end else begin
            case (state)
                ST_IDLE: begin
                    if (issue_real) begin
                        dc_is_store_r   <= ag_is_store;
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
                        dc_dst0_reg_r   <= idu_lsu_ex1_dst0_reg;
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
                    // Wait for DCache.v's own response (its IDLE->DCS->REPLY
                    // protocol takes a fixed 2 cycles from request to
                    // dc_resp_vld -- this state simply waits however long
                    // that takes, rather than assuming a specific count) --
                    // UNLESS nothing was ever requested (a misaligned
                    // access, an uncached real access, or a direct drain),
                    // in which case there is nothing to wait for at all.
                    if (!dc_touched_array_r || u_dc_resp_vld) begin
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
                        end else if (!dc_ca_r) begin
                            frz_is_direct_r <= 1'b1;
                            state <= ST_FRZ;
                        end else if (store_wa_miss_c) begin
                            frz_is_direct_r <= 1'b1;
                            state <= ST_FRZ;
                        end else if (dc_hit_c) begin
                            state <= ST_REPLY;
                        end else begin
                            frz_is_direct_r <= 1'b0;
                            state <= ST_FRZ;
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

    assign lsu_idu_full = (state != ST_IDLE);

    //-------------------------------------------------------------------------
    // SECTION FRZ -- victim-select/writeback/refill-commit (cacheable miss)
    // or a generic direct AXI read-or-write (uncached / wa=0 store-miss /
    // direct STB drain). See header for the overall shape.
    //-------------------------------------------------------------------------
    localparam [3:0] MS_IDLE          = 4'd0;
    localparam [3:0] MS_VPEEK_ISSUE   = 4'd1;
    localparam [3:0] MS_VPEEK_WAIT    = 4'd2;
    localparam [3:0] MS_VB_WRITE      = 4'd3;
    localparam [3:0] MS_REFILL_READ   = 4'd4;
    localparam [3:0] MS_COMMIT_ISSUE  = 4'd5;
    localparam [3:0] MS_COMMIT_WAIT   = 4'd6;
    localparam [3:0] MS_DIRECT_WRITE  = 4'd7;
    localparam [3:0] MS_DIRECT_READ   = 4'd8;
    localparam [3:0] MS_DONE          = 4'd9;

    reg [3:0]  miss_state;
    reg [1:0]  victim_idx_r;
    reg [WAYS-1:0] victim_way_r;
    reg        victim_dirty_r;
    reg [DCACHE_TAG_WIDTH-1:0] victim_tag_r;
    reg [511:0] victim_data_r;
    reg [1:0]  rr_ctr;
    reg [511:0] frz_rdata_r;   // FRZ-local capture (refill or direct-read data);
                                 // copied into dc_rdata_r by the MAIN fsm's own
                                 // always block at the FRZ->REPLY transition so
                                 // dc_rdata_r keeps exactly one writer.

    wire miss_done = (miss_state == MS_DONE);

    // Replacement policy (LSU-owned, per DCache.v's header decision):
    // prefer an invalid way, else the round-robin counter.
    wire [1:0] victim_idx_c = !dc_way_vld_r[0] ? 2'd0 :
                              !dc_way_vld_r[1] ? 2'd1 :
                              !dc_way_vld_r[2] ? 2'd2 :
                              !dc_way_vld_r[3] ? 2'd3 : rr_ctr;
    wire [WAYS-1:0] victim_way_c = 4'b0001 << victim_idx_c;
    wire victim_dirty_c = (victim_idx_c == 2'd0) ? dc_way_dirty_r[0] :
                          (victim_idx_c == 2'd1) ? dc_way_dirty_r[1] :
                          (victim_idx_c == 2'd2) ? dc_way_dirty_r[2] : dc_way_dirty_r[3];

    //---- generic AXI write sub-sequence (victim writeback / uncached store
    // / direct STB drain) -- FetchSink.v's proven D-side write FSM
    // (concurrent AW+W, BVALID watched every cycle), generalized from its
    // fixed tohost address/data to a real, parametrized one.
    reg         axi_w_active;
    reg         axi_w_aw_sent, axi_w_w_sent;
    reg [63:0]  axi_w_addr_r;
    reg [511:0] axi_w_data_r;
    reg [63:0]  axi_w_strb_r;

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
            victim_tag_r   <= {DCACHE_TAG_WIDTH{1'b0}};
            victim_data_r  <= 512'd0;
            rr_ctr         <= 2'd0;
            axi_w_active   <= 1'b0;
            axi_w_aw_sent  <= 1'b0;
            axi_w_w_sent   <= 1'b0;
            axi_w_addr_r   <= 64'd0;
            axi_w_data_r   <= 512'd0;
            axi_w_strb_r   <= 64'd0;
            axi_r_active   <= 1'b0;
            axi_r_ar_sent  <= 1'b0;
            axi_r_addr_r   <= 64'd0;
            frz_rdata_r    <= 512'd0;
        end else begin
            // one-cycle-late AW/W accept bookkeeping (shared by every write use)
            if (axi_w_active) begin
                if (axi_w_aw_hs) axi_w_aw_sent <= 1'b1;
                if (axi_w_w_hs)  axi_w_w_sent  <= 1'b1;
            end
            if (axi_r_active && axi_r_ar_hs) axi_r_ar_sent <= 1'b1;

            case (state)
                ST_FRZ: begin
                    case (miss_state)
                        MS_IDLE: begin
                            if (frz_is_direct_r) begin
                                if (dc_is_store_r) begin
                                    axi_w_active  <= 1'b1;
                                    axi_w_aw_sent <= 1'b0;
                                    axi_w_w_sent  <= 1'b0;
                                    axi_w_addr_r  <= {dc_tag_r, dc_index_r, 6'b0};
                                    axi_w_data_r  <= ({448'b0, (dc_is_drain_r ? stb_data[dc_drain_idx_r] : dc_store_data_r)})
                                                       << ({58'b0, (dc_is_drain_r ? stb_dw_off[dc_drain_idx_r] : dc_dw_off_r)} * 64);
                                    axi_w_strb_r  <= ({56'b0, (dc_is_drain_r ? stb_byte_vld[dc_drain_idx_r] : dc_byte_mask_r)})
                                                       << ({58'b0, (dc_is_drain_r ? stb_dw_off[dc_drain_idx_r] : dc_dw_off_r)} * 8);
                                    miss_state <= MS_DIRECT_WRITE;
                                end else begin
                                    axi_r_active  <= 1'b1;
                                    axi_r_ar_sent <= 1'b0;
                                    axi_r_addr_r  <= {dc_tag_r, dc_index_r, 6'b0};
                                    miss_state <= MS_DIRECT_READ;
                                end
                            end else begin
                                victim_idx_r   <= victim_idx_c;
                                victim_way_r   <= victim_way_c;
                                victim_dirty_r <= victim_dirty_c;
                                miss_state <= victim_dirty_c ? MS_VPEEK_ISSUE : MS_REFILL_READ;
                            end
                        end
                        MS_VPEEK_ISSUE: miss_state <= MS_VPEEK_WAIT;
                        MS_VPEEK_WAIT: begin
                            if (u_dc_resp_vld) begin
                                victim_tag_r  <= u_dc_resp_victim_tag;
                                victim_data_r <= u_dc_resp_rdata;
                                axi_w_active  <= 1'b1;
                                axi_w_aw_sent <= 1'b0;
                                axi_w_w_sent  <= 1'b0;
                                axi_w_addr_r  <= {u_dc_resp_victim_tag, dc_index_r, 6'b0};
                                axi_w_data_r  <= u_dc_resp_rdata;
                                axi_w_strb_r  <= 64'hFFFF_FFFF_FFFF_FFFF;
                                miss_state <= MS_VB_WRITE;
                            end
                        end
                        MS_VB_WRITE: begin
                            if (axi_w_done) begin
                                axi_w_active  <= 1'b0;
                                axi_w_aw_sent <= 1'b0;
                                axi_w_w_sent  <= 1'b0;
                                axi_r_active  <= 1'b1;
                                axi_r_ar_sent <= 1'b0;
                                axi_r_addr_r  <= {dc_tag_r, dc_index_r, 6'b0};
                                miss_state <= MS_REFILL_READ;
                            end
                        end
                        MS_REFILL_READ: begin
                            if (miss_state == MS_REFILL_READ && axi_r_active == 1'b0) begin
                                axi_r_active  <= 1'b1;
                                axi_r_ar_sent <= 1'b0;
                                axi_r_addr_r  <= {dc_tag_r, dc_index_r, 6'b0};
                            end else if (axi_r_data_hs) begin
                                axi_r_active <= 1'b0;
                                frz_rdata_r   <= axi_d_rdata;
                                rr_ctr        <= rr_ctr + 2'd1;
                                miss_state    <= MS_COMMIT_ISSUE;
                            end
                        end
                        MS_COMMIT_ISSUE: miss_state <= MS_COMMIT_WAIT;
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
                        MS_DONE: ;   // held for exactly one cycle by the outer FSM's own transition
                        default: miss_state <= MS_IDLE;
                    endcase
                end
                default: begin
                    if (miss_state != MS_IDLE) miss_state <= MS_IDLE;
                end
            endcase
        end
    end

    // FRZ's own DCache.v traffic (victim-peek read, refill-commit write) --
    // combinational, mutually exclusive with the IDLE-cycle issue mux above
    // (this module is single-outstanding: the main FSM is in FRZ, not
    // IDLE, whenever any of this fires).
    wire frz_issue_vpeek  = (state == ST_FRZ) && (miss_state == MS_VPEEK_ISSUE);
    wire frz_issue_commit = (state == ST_FRZ) && (miss_state == MS_COMMIT_ISSUE);

    assign u_dc_req_vld       = touches_array || frz_issue_vpeek || frz_issue_commit;
    assign u_dc_req_way_sel   = frz_issue_vpeek ? victim_way_r : (frz_issue_commit ? victim_way_r
                                : (issue_drain ? stb_way[drain_pick] : {WAYS{1'b0}}));
    assign u_dc_req_wr        = frz_issue_commit ? 1'b1 : (frz_issue_vpeek ? 1'b0
                                : (issue_drain ? 1'b1 : ag_is_store));
    assign u_dc_req_alloc     = frz_issue_commit;
    assign u_dc_req_wdata      = frz_issue_commit ? frz_rdata_r
                                : (issue_drain ? ({448'b0, stb_data[drain_pick]} << ({58'b0, stb_dw_off[drain_pick]} * 64))
                                : {448'b0, ag_store_data_positioned} << ({58'b0, ag_dw_off} * 64));
    assign u_dc_req_wstrb      = frz_issue_commit ? 64'hFFFF_FFFF_FFFF_FFFF
                                : (issue_drain ? ({56'b0, stb_byte_vld[drain_pick]} << ({58'b0, stb_dw_off[drain_pick]} * 8))
                                : ({56'b0, ag_byte_mask} << ({58'b0, ag_dw_off} * 8)));
    assign u_dc_req_dirty_set  = frz_issue_commit ? 1'b0 : (issue_drain ? 1'b1 : ag_is_store);
    assign u_dc_req_index      = frz_issue_vpeek || frz_issue_commit ? dc_index_r
                                : (issue_drain ? stb_index[drain_pick] : ag_dc_index);
    assign u_dc_req_tag        = frz_issue_vpeek || frz_issue_commit ? dc_tag_r
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
    assign axi_d_awsize  = 3'd6;
    assign axi_d_awburst = 2'b01;
    assign axi_d_awcache = 4'd0;
    assign axi_d_awprot  = 3'd0;
    assign axi_d_wvalid  = axi_w_active && !axi_w_w_sent;
    assign axi_d_wdata   = axi_w_data_r;
    assign axi_d_wstrb   = axi_w_strb_r;
    assign axi_d_wlast   = axi_d_wvalid;
    assign axi_d_bready  = 1'b1;

    assign axi_d_arvalid = axi_r_active && !axi_r_ar_sent;
    assign axi_d_araddr  = axi_r_addr_r;
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
    wire [63:0] raw_dword     = dc_rdata_r[({58'b0,dc_dw_off_r} * 64) +: 64];
    wire [63:0] merged_dword  = (stb_fwd_bits & stb_fwd_data) | (~stb_fwd_bits & raw_dword);

    wire [63:0] rotated = ({merged_dword, merged_dword} >> ({61'b0, dc_byte_off_r} * 8));

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
    // SECTION REPLY completion -- STB create-or-merge for a completing
    // store; the RTU completion/writeback/exception bus.
    //-------------------------------------------------------------------------
    wire stb_match_here = stb_m0_rp || stb_m1_rp || stb_m2_rp || stb_m3_rp;
    wire [1:0] stb_match_idx = stb_m0_rp ? 2'd0 : stb_m1_rp ? 2'd1 : stb_m2_rp ? 2'd2 : 2'd3;
    wire stb_any_free = !stb_vld[0] || !stb_vld[1] || !stb_vld[2] || !stb_vld[3];
    wire [1:0] stb_free_idx = !stb_vld[0] ? 2'd0 : !stb_vld[1] ? 2'd1 : !stb_vld[2] ? 2'd2 : 2'd3;

    wire reply_is_completing_store = (state == ST_REPLY) && !dc_is_drain_r && dc_is_store_r && !dc_misalign_r;
    wire reply_store_needs_new_slot = reply_is_completing_store && !stb_match_here;
    // A genuinely full STB (no match, no free slot) stalls the completing
    // store in REPLY until a drain frees a slot -- contract 4's DEPTH=4
    // backpressure, not a cancellation.
    wire reply_can_complete = !(reply_store_needs_new_slot && !stb_any_free);

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
    wire store_line_resident = dc_hit_r || (dc_ca_r && dc_wa_r);

    wire reply_fire        = (state == ST_REPLY) && reply_can_complete && !dc_is_drain_r;
    wire reply_is_load      = reply_fire && !dc_is_store_r;
    wire reply_is_store      = reply_fire && dc_is_store_r;
    wire reply_is_misalign  = reply_fire && dc_misalign_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stb_vld[0] <= 1'b0; stb_vld[1] <= 1'b0; stb_vld[2] <= 1'b0; stb_vld[3] <= 1'b0;
        end else begin
            // drain retirement
            if ((state == ST_REPLY) && dc_is_drain_r && (!frz_is_direct_r || miss_done_latched))
                stb_vld[dc_drain_idx_r] <= 1'b0;
            // store completion: merge into an existing entry, or allocate.
            if (reply_is_store && !reply_is_misalign) begin
                if (stb_match_here) begin
                    stb_data[stb_match_idx]     <= (expand_byte_mask(dc_byte_mask_r) & dc_store_data_r)
                                                  | (~expand_byte_mask(dc_byte_mask_r) & stb_data[stb_match_idx]);
                    stb_byte_vld[stb_match_idx] <= stb_byte_vld[stb_match_idx] | dc_byte_mask_r;
                    stb_way[stb_match_idx]      <= final_way;
                    stb_was_hit[stb_match_idx]  <= store_line_resident;
                end else if (stb_any_free) begin
                    stb_vld[stb_free_idx]      <= 1'b1;
                    stb_addr[stb_free_idx]      <= dc_addr_r;
                    stb_index[stb_free_idx]     <= dc_index_r;
                    stb_tag[stb_free_idx]       <= dc_tag_r;
                    stb_dw_off[stb_free_idx]    <= dc_dw_off_r;
                    stb_data[stb_free_idx]      <= dc_store_data_r & expand_byte_mask(dc_byte_mask_r);
                    stb_byte_vld[stb_free_idx]  <= dc_byte_mask_r;
                    stb_way[stb_free_idx]       <= final_way;
                    stb_was_hit[stb_free_idx]   <= store_line_resident;
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

    assign lsu_rtu_ex1_cmplt_dp   = (state == ST_REPLY) && reply_can_complete && !dc_is_drain_r;
    assign lsu_rtu_ex1_cmplt      = lsu_rtu_ex1_cmplt_dp;

    assign lsu_rtu_wb_vld  = reply_is_load && !reply_is_misalign;
    assign lsu_rtu_wb_data = da_final;
    assign lsu_rtu_wb_preg = dc_dst0_reg_r;

    assign lsu_rtu_ex2_data      = lsu_rtu_wb_data;
    assign lsu_rtu_ex2_data_vld  = lsu_rtu_wb_vld;
    assign lsu_rtu_ex2_dest_reg  = lsu_rtu_wb_preg;

    assign lsu_rtu_expt_vld = reply_fire && dc_misalign_r;
    assign lsu_rtu_expt_vec = dc_is_store_r ? 5'd6 : 5'd4;   // store/load misalign
    assign lsu_rtu_tval     = dc_addr_r;

    // No async bus-error path is modeled for M2 (the behavioral AXI slave
    // in this test harness never returns a non-OKAY response) -- wired but
    // structurally never fires, same "landing pad" discipline RTU.v's own
    // header already established for its own dead legs.
    assign lsu_rtu_async_expt_vld = 1'b0;
    assign lsu_rtu_async_ld_inst  = 1'b0;

    // rtu_lsu_expt_ack/_expt_exit: received, no live consumer -- see header.
    wire _rtu_ack_unused  = rtu_lsu_expt_ack;
    wire _rtu_exit_unused = rtu_lsu_expt_exit;

endmodule
