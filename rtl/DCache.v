//=============================================================================
// DCache.v - 32 KB 4-way L1 data cache: tag/data/dirty SRAM arrays
//                                                (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 6; this file freezes
// the port list only):
//   gen_rtl/lsu/rtl/aq_dcache_top.v (+ tag/data/dirty array wrappers --
//                                     real donor is TWO 4-way tag banks
//                                     selected by VA[12] for VIPT-alias
//                                     detection; M2 omits the second bank
//                                     entirely, contract 9)
// References: design doc S2.2/S4.1/S4.3 (DCache is its own file per
// umbrella S6.2 rule 7, matching ICache.v's M1 precedent for file-
// worthiness; sits behind LSU's DC stage), contract 9 (single 128-set x
// 4-way group, PA[39:12] 28b tag, no alias bank, budget tag-row bits for a
// future valid/tag pair per way), contract 11 (dirty-line victim writeback
// in scope for M2), LSU note A3 (real geometry)/A1 (module graph).
//
// SEAM NOTES (rv906 decomposition, differs from ICache.v's M1 placement):
//  * Unlike ICache.v (a PEER of IFU.v, instantiated directly in
//    rtl/RVProc.v since M1), DCache.v is instantiated INSIDE LSU.v, not at
//    RVProc.v's top level -- plan Task 7.1's rewire list names only
//    `IFU -> IDU -> IU -> LSU -> RTU, plus CSR.v and MMU.v` as RVProc.v's
//    top-level instances; DCache.v never appears there. "Mirrors ICache.v's
//    M1 precedent" (the DCache.v plan bullet's own phrase) therefore means
//    file-worthiness (umbrella S6.2 rule 7: independent state + a clean
//    interface earns its own file) and single-consumer isolation (no
//    direct RTU/IDU/CSR ports, plan 1.2's DCache.v bullet), NOT
//    instantiation placement.
//  * No AXI ports on this module at all -- the D-side AXI master (refill
//    on miss, victim writeback on dirty eviction) is explicitly LSU.v's
//    job per this task's own LSU.v bullet ("the AXI D-channel port group
//    ... moved here from FetchSink.v's tohost-only write FSM"). DCache.v
//    exposes only a plain, synchronous read/write/invalidate/victim-
//    writeback request/response interface to LSU.v; LSU.v's own refill/
//    victim FSM (Task 6) drives AXI and then writes the result in through
//    this interface, rather than DCache.v doing its own AXI transaction
//    the way ICache.v does.
//  * The exact request/response protocol (timing, FSM states) is Task 6's
//    job; this task only pins the port LIST. The interface below is sized
//    to contract 9's confirmed geometry (128 sets, 4 ways, 64B = 512b
//    line, 28b tag) and explicitly budgets a per-way valid+dirty pair
//    (contract 9's "budget tag-row bits for a future valid/tag pair per
//    way" forward-compat requirement for M4's second bank).
//=============================================================================

import rvproc_pkg::*;

module DCache (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // LSU -> DCache : tag/data read-or-write request.
    //=========================================================================
    input  wire                            dc_req_vld,
    input  wire [DCACHE_INDEX_W-1:0]       dc_req_index,     // PA[12:6] (Task 6
                                                              // fix, see
                                                              // rvproc_pkg.sv)
    input  wire [DCACHE_TAG_WIDTH-1:0]     dc_req_tag,       // PA[39:13] (Task 6
                                                              // fix, see
                                                              // rvproc_pkg.sv)
    input  wire                            dc_req_wr,        // 0=read(load/
                                                              // tag-compare),
                                                              // 1=write
                                                              // (store-hit/
                                                              // refill)
    input  wire [DCACHE_WAYS-1:0]          dc_req_way_sel,   // one-hot,
                                                              // write/refill/
                                                              // victim-select
    input  wire [DCACHE_LINE_BYTES*8-1:0]  dc_req_wdata,     // one 64B line
    input  wire [DCACHE_LINE_BYTES-1:0]    dc_req_wstrb,     // byte-granular
    input  wire                            dc_req_dirty_set, // mark written
                                                              // way dirty
    input  wire                            dc_req_alloc,     // allocate a
                                                              // new tag on a
                                                              // write (write-
                                                              // allocate/
                                                              // refill vs. a
                                                              // plain store-
                                                              // hit data
                                                              // update)

    //=========================================================================
    // DCache -> LSU : tag-compare/read response, one cycle later.
    //=========================================================================
    output wire                            dc_resp_vld,
    output wire [DCACHE_WAYS-1:0]          dc_resp_hit_way,   // one-hot
    output wire [DCACHE_LINE_BYTES*8-1:0]  dc_resp_rdata,     // hit way's line
    output wire [DCACHE_WAYS-1:0]          dc_resp_way_vld,   // per-way valid
    output wire [DCACHE_WAYS-1:0]          dc_resp_way_dirty, // per-way dirty
    output wire [DCACHE_TAG_WIDTH-1:0]     dc_resp_victim_tag,// evict
                                                               // candidate's
                                                               // tag (contract
                                                               // 11, victim
                                                               // writeback)

    // The DCache's own FSM is in its accept state (ST_IDLE): it will take a
    // dc_req_vld / dc_inv_vld presented THIS cycle. Exposed so LSU.v's
    // donor-faithful per-cycle STB data-port drain grant (the direct
    // cacheable-drain path) can key off "the data array is free" exactly as
    // the donor's arb_stb_grant keys off no-other-data-requester (donor
    // aq_lsu_arb.v:380) -- in rv906's single-ported DCache that is precisely
    // this FSM's accept state.
    output wire                            dc_idle,

    //=========================================================================
    // LSU -> DCache : invalidate (fence.i-adjacent D-side coherence /
    // explicit line invalidate).
    //=========================================================================
    input  wire                     dc_inv_vld,
    input  wire [DCACHE_INDEX_W-1:0] dc_inv_index,
    input  wire [DCACHE_WAYS-1:0]   dc_inv_way_sel,
    output wire                     dc_inv_done
);

    //=========================================================================
    // TASK 6 REAL BODY.
    //
    // GEOMETRY (contract 9, and the Task 6 rvproc_pkg.sv fix documented
    // there): single 128-set x 4-way group, tag=PA[39:13] (27b),
    // index=PA[12:6] (7b), line-offset=PA[5:0] (6b) -- 27+7+6=40, exact.
    //
    // PROTOCOL (this task's own design, since DCache.v's frozen header says
    // "the exact request/response protocol... is Task 6's job"): a single
    // outstanding request at a time, 4 named states mirroring
    // aq_lsu_dc.v's IDLE/DCS/FRZ/REPLY shape (confirmed via direct read of
    // aq_lsu_dc.v:1349-1403 by a Task 6 research pass):
    //   IDLE  -- accept a new dc_req_vld (or service dc_inv_vld) this cycle.
    //            Fields are ALWAYS latched here regardless of which path is
    //            taken, so DCS/REPLY can read a single, uniform register
    //            set. If dc_req_vld and dc_inv_vld arrive the SAME cycle,
    //            invalidate wins the (shared, single-port) tag/dirty SRAM
    //            this cycle and the normal request's own SRAM issue is
    //            deferred by exactly one cycle -- this IS the FRZ trigger,
    //            and it is the genuine, only possible same-cycle port
    //            conflict in a design where LSU.v (the sole real requester)
    //            is itself single-outstanding (never issues a second
    //            dc_req_vld before the first's dc_resp_vld arrives). The
    //            donor's own FRZ (research: aq_lsu_dc.v's `dc_reply`/
    //            `dc_wakeup`, gated on LFB/STB/VB event signals, NOT a
    //            literal data-array port conflict) is a strictly LSU-level
    //            concept in the donor (miss-buffer/store-buffer/victim-
    //            buffer coordination) that has no DCache-array-level
    //            equivalent once those three structures move to LSU.v (per
    //            this file's own header: "DCache.v exposes only a plain...
    //            interface... LSU.v's own refill/victim FSM... drives AXI").
    //            LSU.v's OWN pipeline (Task 6.3) models THAT wait as its
    //            own miss-handling sequence; THIS module's FRZ is the one
    //            real array-level port hazard that exists at this layer --
    //            invalidate vs. a normal access -- deliberately exercised
    //            standalone by dcache_tb (LSU.v never drives dc_inv_vld in
    //            M2 at all, contract 10: no D$-maintenance op is decoded).
    //   DCS   -- the cycle after IDLE (or after FRZ's deferred issue): the
    //            SRAM's registered read output is now valid (SRAM.v: address
    //            applied one cycle, Q readable the next); tag compare
    //            resolves here and the result is LATCHED into the resp_*
    //            registers this transition (matching the donor's "DC:
    //            tag/data SRAM outputs are compared" and note A2's "tag
    //            compare resolves hit/miss in the DCS cycle").
    //   FRZ   -- (only entered on the invalidate-collision above) issues
    //            the deferred SRAM access now that the port is free; always
    //            exactly 1 cycle, then -> DCS.
    //   REPLY -- presents the DCS-latched answer via dc_resp_vld=1 (matches
    //            note A2's "DA: ... asserts lsu_rtu_wb_vld" cycle, one-to-
    //            one with LSU's own DA stage since REPLY IS the cycle LSU
    //            consumes the response) -- FRZ is a pure array-port-
    //            arbitration stall, not part of nominal hit latency (LSU
    //            note A2), exactly as the plan text states: a clean
    //            transaction is IDLE->DCS->REPLY (2 cycles, matching AG-
    //            issues/DC-resolves/DA-presents when LSU's own AG cycle IS
    //            this module's IDLE-accept cycle -- 3 total cycles end to
    //            end); a collision adds exactly the one FRZ cycle.
    //
    // WAY-SELECT DUAL PURPOSE (`dc_req_way_sel`, per this file's own header
    // comment "one-hot, write/refill/victim-select"): for a WRITE
    // (`dc_req_wr=1`) it is REQUIRED one-hot -- the way being updated. For a
    // READ (`dc_req_wr=0`) it is normally all-zero (an ordinary hit-check,
    // where the hit way isn't known yet) but MAY be driven one-hot to
    // EXPLICITLY peek a specific way's raw tag+data regardless of the tag-
    // compare outcome -- LSU.v's victim-writeback path uses this to read out
    // a dirty eviction candidate's exact tag+data before overwriting it.
    // `dc_resp_hit_way` always reflects the REAL tag-compare result (zero on
    // a genuine miss); `dc_resp_rdata`/`dc_resp_victim_tag` reflect whichever
    // way `dc_req_way_sel` names, falling back to the hit way when
    // `dc_req_way_sel` is all-zero.
    //
    // REPLACEMENT POLICY IS NOT OWNED BY THIS MODULE -- LSU.v runs it,
    // exactly as the donor does (the pointer-advance logic lives in
    // aq_lsu_rdl.v/aq_lsu_lfb.v, outside the D-cache arrays). This module
    // exposes exactly what a requester needs to run ANY replacement policy
    // itself (`dc_resp_way_vld`/`dc_resp_way_dirty` at the index, plus the
    // way-peek mechanism above); LSU.v is where the donor's per-set FIFO
    // replacement pointer now lives (fifo_ptr[], the 2-bit-count equivalent
    // of the donor's 4-bit one-hot stored in the dirty row's upper nibble):
    // victim = the pointer's way, advanced p -> p+1 mod 4 on every same-set
    // refill commit, reset to way 0 on invalidate-all (see LSU.v's
    // victim-select section for the full donor cites).
    //=========================================================================

    localparam WAYS       = DCACHE_WAYS;                 // 4
    localparam TAGW        = DCACHE_TAG_WIDTH;             // 27
    localparam LINEBITS    = DCACHE_LINE_BYTES*8;          // 512
    localparam TAG_SLOT_W  = TAGW + 1;                     // {valid,tag} = 28
    localparam TAG_ROW_W   = WAYS*TAG_SLOT_W;              // 112

    localparam [1:0] ST_IDLE  = 2'b00;
    localparam [1:0] ST_DCS   = 2'b01;
    localparam [1:0] ST_FRZ   = 2'b10;
    localparam [1:0] ST_REPLY = 2'b11;

    reg [1:0] state;

    assign dc_idle = (state == ST_IDLE);

    //-------------------------------------------------------------------------
    // Latched request fields (captured every time IDLE accepts a request,
    // whether serviced immediately (-> DCS next) or deferred by an
    // invalidate collision (-> FRZ next, still using these same fields)).
    //-------------------------------------------------------------------------
    reg [DCACHE_INDEX_W-1:0]      req_index_r;
    reg [DCACHE_TAG_WIDTH-1:0]    req_tag_r;
    reg                            req_wr_r;
    reg [WAYS-1:0]                 req_way_sel_r;
    reg [LINEBITS-1:0]             req_wdata_r;
    reg [DCACHE_LINE_BYTES-1:0]    req_wstrb_r;
    reg                             req_alloc_r;
    reg                             req_dirty_set_r;

    wire do_inv_now = dc_inv_vld && (state == ST_IDLE);
    wire issue_cyc  = (state == ST_IDLE && dc_req_vld && !dc_inv_vld) || (state == ST_FRZ);

    // Effective fields for THIS cycle's array access: live dc_req_* ports
    // while accepting from IDLE, the latched copies while replaying from FRZ.
    wire [DCACHE_INDEX_W-1:0]   eff_index    = (state == ST_FRZ) ? req_index_r    : dc_req_index;
    wire [DCACHE_TAG_WIDTH-1:0] eff_tag      = (state == ST_FRZ) ? req_tag_r      : dc_req_tag;
    wire                        eff_wr       = (state == ST_FRZ) ? req_wr_r       : dc_req_wr;
    wire [WAYS-1:0]             eff_way_sel  = (state == ST_FRZ) ? req_way_sel_r  : dc_req_way_sel;
    wire [LINEBITS-1:0]         eff_wdata    = (state == ST_FRZ) ? req_wdata_r    : dc_req_wdata;
    wire [DCACHE_LINE_BYTES-1:0] eff_wstrb   = (state == ST_FRZ) ? req_wstrb_r    : dc_req_wstrb;
    wire                        eff_alloc    = (state == ST_FRZ) ? req_alloc_r    : dc_req_alloc;
    wire                        eff_dirty_set= (state == ST_FRZ) ? req_dirty_set_r: dc_req_dirty_set;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            req_index_r      <= {DCACHE_INDEX_W{1'b0}};
            req_tag_r        <= {DCACHE_TAG_WIDTH{1'b0}};
            req_wr_r         <= 1'b0;
            req_way_sel_r    <= {WAYS{1'b0}};
            req_wdata_r      <= {LINEBITS{1'b0}};
            req_wstrb_r      <= {DCACHE_LINE_BYTES{1'b0}};
            req_alloc_r      <= 1'b0;
            req_dirty_set_r  <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (dc_req_vld) begin
                        req_index_r     <= dc_req_index;
                        req_tag_r       <= dc_req_tag;
                        req_wr_r        <= dc_req_wr;
                        req_way_sel_r   <= dc_req_way_sel;
                        req_wdata_r     <= dc_req_wdata;
                        req_wstrb_r     <= dc_req_wstrb;
                        req_alloc_r     <= dc_req_alloc;
                        req_dirty_set_r <= dc_req_dirty_set;
                        state <= dc_inv_vld ? ST_FRZ : ST_DCS;
                    end
                end
                ST_FRZ:   state <= ST_DCS;
                ST_DCS:   state <= ST_IDLE;   // response is combinational at DCS; no REPLY
                default:  state <= ST_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // TAG ARRAY -- one combined row per set: WAYS x {valid(1),tag(TAGW)}.
    //-------------------------------------------------------------------------
    wire [TAG_ROW_W-1:0] tag_q;
    wire [TAG_ROW_W-1:0] tag_wdata_full;
    wire [TAG_ROW_W-1:0] tag_wen_n_full;
    wire [TAG_ROW_W-1:0] inv_tag_din;
    wire [TAG_ROW_W-1:0] inv_tag_wen_n;

    wire tag_issue_wr = issue_cyc && eff_wr && eff_alloc;   // write only on refill/alloc

    genvar gs;
    generate
        for (gs = 0; gs < WAYS; gs = gs + 1) begin : g_tagslot
            wire issue_hit_this_way = tag_issue_wr && eff_way_sel[gs];
            wire inv_hit_this_way   = do_inv_now && dc_inv_way_sel[gs];

            assign tag_wen_n_full[gs*TAG_SLOT_W +: TAG_SLOT_W] =
                issue_hit_this_way ? {TAG_SLOT_W{1'b0}} : {TAG_SLOT_W{1'b1}};
            assign tag_wdata_full[gs*TAG_SLOT_W +: TAG_SLOT_W] =
                issue_hit_this_way ? {1'b1, eff_tag} : {TAG_SLOT_W{1'b0}};

            assign inv_tag_wen_n[gs*TAG_SLOT_W +: TAG_SLOT_W] =
                inv_hit_this_way ? {TAG_SLOT_W{1'b0}} : {TAG_SLOT_W{1'b1}};
            assign inv_tag_din[gs*TAG_SLOT_W +: TAG_SLOT_W] = {TAG_SLOT_W{1'b0}};   // clears valid+tag
        end
    endgenerate

    wire tag_cen_n  = !(issue_cyc || do_inv_now);
    wire tag_gwen_n = !(tag_issue_wr || do_inv_now);
    wire [DCACHE_INDEX_W-1:0] tag_addr = do_inv_now ? dc_inv_index : eff_index;
    wire [TAG_ROW_W-1:0]      tag_d    = do_inv_now ? inv_tag_din   : tag_wdata_full;
    wire [TAG_ROW_W-1:0]      tag_wenn = do_inv_now ? inv_tag_wen_n : tag_wen_n_full;

    SRAM #(.WIDTH(TAG_ROW_W), .DEPTH(DCACHE_SETS)) u_tag_array (
        .clk(clk), .cen_n(tag_cen_n), .gwen_n(tag_gwen_n), .wen_n(tag_wenn),
        .addr(tag_addr), .d(tag_d), .q(tag_q)
    );

    wire [WAYS-1:0] way_valid_c;
    wire [DCACHE_TAG_WIDTH-1:0] way_tag_c [0:WAYS-1];
    generate
        for (gs = 0; gs < WAYS; gs = gs + 1) begin : g_tagread
            assign way_valid_c[gs] = tag_q[gs*TAG_SLOT_W + DCACHE_TAG_WIDTH];
            assign way_tag_c[gs]   = tag_q[gs*TAG_SLOT_W +: DCACHE_TAG_WIDTH];
        end
    endgenerate

    //-------------------------------------------------------------------------
    // DIRTY ARRAY -- one combined row per set: WAYS dirty bits.
    //-------------------------------------------------------------------------
    wire [WAYS-1:0] dirty_q;
    wire issue_write = issue_cyc && eff_wr;

    wire [WAYS-1:0] dirty_issue_wen_n = issue_write ? ~eff_way_sel      : {WAYS{1'b1}};
    wire [WAYS-1:0] dirty_issue_din   = {WAYS{eff_dirty_set}};
    wire [WAYS-1:0] dirty_inv_wen_n   = do_inv_now  ? ~dc_inv_way_sel  : {WAYS{1'b1}};
    wire [WAYS-1:0] dirty_inv_din     = {WAYS{1'b0}};

    wire dirty_cen_n  = !(issue_cyc || do_inv_now);
    wire dirty_gwen_n = !(issue_write || do_inv_now);
    wire [DCACHE_INDEX_W-1:0] dirty_addr = do_inv_now ? dc_inv_index : eff_index;
    wire [WAYS-1:0] dirty_d    = do_inv_now ? dirty_inv_din   : dirty_issue_din;
    wire [WAYS-1:0] dirty_wenn = do_inv_now ? dirty_inv_wen_n : dirty_issue_wen_n;

    SRAM #(.WIDTH(WAYS), .DEPTH(DCACHE_SETS)) u_dirty_array (
        .clk(clk), .cen_n(dirty_cen_n), .gwen_n(dirty_gwen_n), .wen_n(dirty_wenn),
        .addr(dirty_addr), .d(dirty_d), .q(dirty_q)
    );

    //-------------------------------------------------------------------------
    // DATA ARRAYS -- WAYS separate SRAM instances, each DCACHE_SETS x
    // LINEBITS (one 64B line per row). A read cycle enables ALL ways
    // (speculative -- the hit way isn't known until the tag compare
    // resolves); a write cycle enables only the ONE targeted way.
    //-------------------------------------------------------------------------
    wire [LINEBITS-1:0] wstrb_bitmask;
    generate
        for (gs = 0; gs < DCACHE_LINE_BYTES; gs = gs + 1) begin : g_wstrb
            assign wstrb_bitmask[gs*8 +: 8] = {8{eff_wstrb[gs]}};
        end
    endgenerate

    wire [LINEBITS-1:0] data_q [0:WAYS-1];
    genvar gw;
    generate
        for (gw = 0; gw < WAYS; gw = gw + 1) begin : g_dataarr
            wire way_write_active = issue_write && eff_way_sel[gw];
            wire way_read_active  = issue_cyc && !eff_wr;
            wire way_cen_n  = !(way_write_active || way_read_active);
            wire way_gwen_n = !way_write_active;
            wire [LINEBITS-1:0] way_wen_n = way_write_active ? ~wstrb_bitmask : {LINEBITS{1'b1}};

            SRAM #(.WIDTH(LINEBITS), .DEPTH(DCACHE_SETS)) u_data_array (
                .clk(clk), .cen_n(way_cen_n), .gwen_n(way_gwen_n), .wen_n(way_wen_n),
                .addr(eff_index), .d(eff_wdata), .q(data_q[gw])
            );
        end
    endgenerate

    //-------------------------------------------------------------------------
    // HIT COMPARE + WAY-SELECT-OR-HIT-WAY RESPONSE MUX (combinational,
    // computed continuously; LATCHED into resp_* only at the DCS->REPLY
    // transition below).
    //-------------------------------------------------------------------------
    wire [WAYS-1:0] hit_way_c;
    generate
        for (gs = 0; gs < WAYS; gs = gs + 1) begin : g_hit
            assign hit_way_c[gs] = way_valid_c[gs] && (way_tag_c[gs] == req_tag_r);
        end
    endgenerate

    wire [WAYS-1:0] resp_way_for_data_c = (req_way_sel_r != {WAYS{1'b0}}) ? req_way_sel_r : hit_way_c;

    wire [LINEBITS-1:0] rdata_term0 = resp_way_for_data_c[0] ? data_q[0] : {LINEBITS{1'b0}};
    wire [LINEBITS-1:0] rdata_term1 = resp_way_for_data_c[1] ? data_q[1] : {LINEBITS{1'b0}};
    wire [LINEBITS-1:0] rdata_term2 = resp_way_for_data_c[2] ? data_q[2] : {LINEBITS{1'b0}};
    wire [LINEBITS-1:0] rdata_term3 = resp_way_for_data_c[3] ? data_q[3] : {LINEBITS{1'b0}};
    wire [LINEBITS-1:0] rdata_mux_c = rdata_term0 | rdata_term1 | rdata_term2 | rdata_term3;

    wire [DCACHE_TAG_WIDTH-1:0] vtag_term0 = resp_way_for_data_c[0] ? way_tag_c[0] : {DCACHE_TAG_WIDTH{1'b0}};
    wire [DCACHE_TAG_WIDTH-1:0] vtag_term1 = resp_way_for_data_c[1] ? way_tag_c[1] : {DCACHE_TAG_WIDTH{1'b0}};
    wire [DCACHE_TAG_WIDTH-1:0] vtag_term2 = resp_way_for_data_c[2] ? way_tag_c[2] : {DCACHE_TAG_WIDTH{1'b0}};
    wire [DCACHE_TAG_WIDTH-1:0] vtag_term3 = resp_way_for_data_c[3] ? way_tag_c[3] : {DCACHE_TAG_WIDTH{1'b0}};
    wire [DCACHE_TAG_WIDTH-1:0] victim_tag_mux_c = vtag_term0 | vtag_term1 | vtag_term2 | vtag_term3;

    reg [WAYS-1:0]              resp_hit_way_r;
    reg [LINEBITS-1:0]           resp_rdata_r;
    reg [WAYS-1:0]               resp_way_vld_r;
    reg [WAYS-1:0]               resp_way_dirty_r;
    reg [DCACHE_TAG_WIDTH-1:0]   resp_victim_tag_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            resp_hit_way_r     <= {WAYS{1'b0}};
            resp_rdata_r        <= {LINEBITS{1'b0}};
            resp_way_vld_r      <= {WAYS{1'b0}};
            resp_way_dirty_r    <= {WAYS{1'b0}};
            resp_victim_tag_r   <= {DCACHE_TAG_WIDTH{1'b0}};
        end else if (state == ST_DCS) begin
            resp_hit_way_r     <= hit_way_c;
            resp_rdata_r        <= rdata_mux_c;
            resp_way_vld_r      <= way_valid_c;
            resp_way_dirty_r    <= dirty_q;
            resp_victim_tag_r   <= victim_tag_mux_c;
        end
    end

    assign dc_resp_vld        = (state == ST_DCS);
    assign dc_resp_hit_way    = hit_way_c;
    assign dc_resp_rdata      = rdata_mux_c;
    assign dc_resp_way_vld    = way_valid_c;
    assign dc_resp_way_dirty  = dirty_q;
    assign dc_resp_victim_tag = victim_tag_mux_c;

    //-------------------------------------------------------------------------
    // INVALIDATE ack -- one-cycle pulse the cycle after the invalidate write
    // committed (mirrors ICache.v's own `ifu_cp0_icache_inv_done` pattern).
    //-------------------------------------------------------------------------
    reg inv_done_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) inv_done_r <= 1'b0;
        else        inv_done_r <= do_inv_now;
    end
    assign dc_inv_done = inv_done_r;

endmodule
