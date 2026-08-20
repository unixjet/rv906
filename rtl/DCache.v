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
    input  wire [DCACHE_INDEX_W-1:0]       dc_req_index,     // PA[11:6]
    input  wire [DCACHE_TAG_WIDTH-1:0]     dc_req_tag,       // PA[39:12]
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
    // SKELETON BODY (plan Task 6 replaces it): every read/tag-compare
    // reports a permanent miss, every way permanently invalid/clean.
    //=========================================================================
    assign dc_resp_vld        = 1'b0;
    assign dc_resp_hit_way    = {DCACHE_WAYS{1'b0}};
    assign dc_resp_rdata      = {DCACHE_LINE_BYTES*8{1'b0}};
    assign dc_resp_way_vld    = {DCACHE_WAYS{1'b0}};
    assign dc_resp_way_dirty  = {DCACHE_WAYS{1'b0}};
    assign dc_resp_victim_tag = {DCACHE_TAG_WIDTH{1'b0}};

    assign dc_inv_done = 1'b0;

endmodule
