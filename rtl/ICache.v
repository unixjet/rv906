//=============================================================================
// ICache.v - 32 KB 2-way VIPT L1 instruction cache  (M1 SKELETON: ports frozen)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_ifu_icache.v            (monolithic: tag/data SRAM
//                                                wrappers, hit judge, refill
//                                                FSM, prefetch FSM, CP0
//                                                invalidate/diag-read FSM,
//                                                AXI (BIU) master)
//   gen_rtl/ifu/rtl/aq_ifu_icache_tag_array.v  (1x aq_spsram_256x59)
//   gen_rtl/ifu/rtl/aq_ifu_icache_data_array.v (4x aq_spsram_2048x32)
// References: IFU pipeline extraction notes S1 (SRAM inventory), S3 (full
// geometry, tag row, refill, invalidate), S9 (read-directly spans). Body
// arrives in plan Task 2; this file freezes the port list only. Port names
// and widths below are the REAL aq_ifu_icache.v port list (read directly,
// icache.v:16-81 for the module header, :94-155 for widths) -- not a
// C910-by-analogy guess.
//
// SEAM NOTE (rv906 decomposition, differs from C910/rv12's split):
//  * C906's ICache is architecturally ONE monolithic module with its own
//    internal 2-cycle request/hit-check pipeline (IFU notes S0, S2 item 3) --
//    there is no separate IF/IP module pair to split the tag COMPARE across,
//    the way rv12 did for C910. rv906 keeps the whole tag/data/hit-check/
//    refill/invalidate/AXI-master function inside this ONE module, matching
//    the donor's own module boundary exactly (a smaller seam decision than
//    rv12's, not a bigger one).
//  * The MMU-facing port group (`ifu_mmu_*`/`mmu_ifu_*`) is a port of THIS
//    module, not of IFU.v -- confirmed from aq_ifu_icache.v's own port list
//    (icache.v:74-81,113-116,153-155: `mmu_ifu_pa` is used directly at
//    icache.v:729, `icache_pa[39:0] = {mmu_ifu_pa[27:0],
//    icache_rd_addr[11:0]}`). This differs from rv12's C910 seam, where
//    translation lived in the IFU (ifdp) because C910 splits tag-compare
//    across IF/IP stage modules; C906 has no such split, so the boundary
//    that was arbitrary for C910 is a confirmed structural fact here.
//  * Dropped entirely (per umbrella spec S6.3's no-clock-gating-cells rule,
//    and the parent design's HAD/debug and performance-tuning fencing):
//    `cpurst_b` (redundant with `rst_n`), `forever_cpuclk`/`cp0_ifu_icg_en`/
//    `pad_yy_icg_scan_en` (gated-clock/scan infrastructure), every `_gate`
//    companion signal (`icache_pcgen_grant_gate` etc. -- the low-power
//    clock-gating variant of an already-present enable), `hpcp_ifu_cnt_en`/
//    `ifu_hpcp_icache_access`/`ifu_hpcp_icache_miss` (perf counters, M8's
//    job), `cp0_ifu_icache_read_*`/`ifu_cp0_icache_read_data*`/
//    `icache_top_*` (the CP0 diagnostic cache-line-read path -- HAD-adjacent
//    debug infrastructure, deferred with the rest of HAD to M7, same
//    precedent rv12 set for C910's equivalent), `cp0_ifu_lpmd_req` (low-power
//    mode, not modeled), `ifu_yy_xx_no_op`, `icache_btb_grant`/
//    `icache_pred_inst_vld` (BTB/pred read their own PC-driven enable
//    straight from PCGEN inside IFU.v per the BPU notes' arbitration
//    description -- not gated through ICache; see IFU.v's header).
//=============================================================================

import rvproc_pkg::*;

module ICache #(
    parameter DATA_WIDTH = 512,             // rv906 SoC bus width (spec dev. 1)
    parameter ADDR_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // CP0 chicken bits + invalidate handshake (direct CP0 fan-out, per the
    // real RTL -- NOT routed through IFU's ctrl hub, IFU notes S5.1). In M1
    // these are driven by FetchSink's config bank (plan "Global contracts").
    //=========================================================================
    input  wire                     cp0_ifu_icache_en,      // icache.v port
    input  wire                     cp0_ifu_iwpe,            // way pred/bypass; 0 in M1
    input  wire                     cp0_ifu_icache_pref_en,  // next-line prefetch enable
    input  wire [63:0]              cp0_ifu_icache_inv_addr,
    input  wire                     cp0_ifu_icache_inv_req,  // fence.i / icache.iall
    input  wire [1:0]               cp0_ifu_icache_inv_type,
    output wire                     ifu_cp0_icache_inv_done,

    //=========================================================================
    // IFU -> ICache : fetch request, PCGEN-timed (IFU notes S2)
    //=========================================================================
    // pcgen_icache_va[63:0] = pcgen_fetch_pc[63:0] (icache.v:120); only bits
    // [39:0] are architecturally meaningful (PC_WIDTH). pcgen_icache_seq_tag
    // = pcgen_ifpc[39:6], the line tag (pcgen.v:311).
    input  wire [63:0]              pcgen_icache_va,
    input  wire [33:0]              pcgen_icache_seq_tag,
    input  wire                     pcgen_icache_chgflw_vld,
    input  wire                     ctrl_icache_req_vld,    // = ctrl_inst_fetch
    input  wire                     ctrl_icache_abort,      // = ctrl_if_cancel

    //=========================================================================
    // ICache -> IFU : grant / feedback to PCGEN
    //=========================================================================
    output wire                     icache_pcgen_grant,
    output wire [39:0]              icache_pcgen_addr,
    output wire                     icache_pcgen_inst_vld,
    output wire                     icache_ctrl_stall,      // -> ctrl_if_stall term

    //=========================================================================
    // ICache -> IFU : data output to IPACK (icache_ipack_inst is the ENTIRE
    // per-cycle bus -- 32 bits/2 halfwords, IFU notes S3/S4.1; no separate
    // predecode array, RVC boundaries are computed live downstream in IPACK)
    //=========================================================================
    output wire [31:0]              icache_ipack_inst,
    output wire                     icache_ipack_inst_vld,
    output wire                     icache_ipack_acc_err,
    output wire                     icache_ipack_pgflt,
    output wire                     icache_ipack_unalign,

    //=========================================================================
    // MMU/ITLB stub interface (icache.v's own ports; see SEAM NOTE above).
    // M1 drives a zero-latency bare-physical-mapping stub from RVProc.v
    // (design doc S2.1/S3); M4 swaps the implementation without touching
    // this port list.
    //=========================================================================
    output wire                     ifu_mmu_abort,
    output wire [MMU_VA_WIDTH-1:0]  ifu_mmu_va,
    output wire                     ifu_mmu_va_vld,
    input  wire                     mmu_ifu_access_fault,
    input  wire [MMU_PA_WIDTH-1:0]  mmu_ifu_pa,
    input  wire                     mmu_ifu_pa_vld,
    input  wire [MMU_PROT_WIDTH-1:0] mmu_ifu_prot,

    //=========================================================================
    // AXI read master (SoC I-side, ch[0]); channel names/widths copied from
    // TestMaster.v's axi_i_* group so RVProc.v passes them straight through.
    // Read-only master: the write channels of the I-side port stay in
    // RVProc.v (ICache never writes memory). Spec deviation 1: ONE 512-bit
    // single-beat read per 64B line, sliced internally (plan Task 2.1).
    //=========================================================================
    output wire                     axi_i_arvalid,
    input  wire                     axi_i_arready,
    output wire [ADDR_WIDTH-1:0]    axi_i_araddr,
    output wire [7:0]               axi_i_arlen,
    output wire [2:0]               axi_i_arsize,
    output wire [1:0]               axi_i_arburst,
    output wire [3:0]               axi_i_arcache,
    output wire [2:0]               axi_i_arprot,

    input  wire                     axi_i_rvalid,
    output wire                     axi_i_rready,
    input  wire [DATA_WIDTH-1:0]    axi_i_rdata,
    input  wire [1:0]               axi_i_rresp,
    input  wire                     axi_i_rlast
);

    //=========================================================================
    // TASK 2.1 BODY. Section map (dataflow order):
    //   GEOMETRY    local widths, all real numbers still come from the pkg
    //   IWPE        the cp0_ifu_iwpe resolution (open item, see below)
    //   INVALIDATE  fence.i / icache.iall INV_ALL walk, 256 sets
    //   REQUEST     cycle-A/cycle-B request pipe (mirrors icache.v:514-536,
    //               749-776 -- one flop stage between address-issue and
    //               tag-compare, same as the donor's "one module, one
    //               internal flop" 2-cycle access, IFU notes S2 item 3)
    //   MMU         ITLB request/response (zero-latency M1 stub, RVProc.v)
    //   ARRAYS      tag SRAM (1x256x59) + data SRAM (4x2048x32)
    //   HIT         tag compare, way select, miss detect
    //   REFILL      miss FSM (deviation 1: one 512b single-beat AXI read/
    //               line, sliced into 4 FILL cycles -- rv12 ICache.v:225-
    //               232,320-447 pattern) + AXI master + bypass
    //   OUTPUT      pcgen/ipack renames
    //
    // CP0_IFU_IWPE RESOLUTION (plan Task 2.1 open item; extraction note S10,
    // design doc S2.2/S2.3/S6, risk list): read icache.v:566-664 in full.
    // `cp0_ifu_iwpe` gates a SINGLE-ENTRY "tag hit buffer" (`tag_hit_vld`/
    // `buf_hit_tag`/`buf_hit_way`) that remembers the (line-tag, way) of the
    // MOST RECENT hit or refill. On the next request, if the incoming
    // `pcgen_icache_seq_tag` (the requested line's tag, computed by PCGEN
    // one cycle ahead of the actual access -- icache.v:659 `addr_equal`)
    // equals the buffered tag AND no invalidate/refill/chgflw happened since
    // (`cen_mask_vld`, icache.v:662), the module sets `direct_sel` and (a)
    // skips the tag SRAM read/compare entirely (`icache_hit = ... ||
    // direct_sel`, icache.v:786-787) and (b) enables only the ALREADY-KNOWN
    // hit way's data bank (`data_cen_masked`, icache.v:664,679) instead of
    // both ways. Both effects are pure power-saving array-enable gating: the
    // buffered (tag, way) pair is exactly what a real tag compare would have
    // produced (the buffer is invalidated on every refill/inv/chgflw event
    // that could make it stale, `buf_clr_en`, icache.v:598-602), so
    // `direct_sel` changes ZERO fetched bits and zero hit/miss outcomes --
    // it only removes SRAM reads that would have re-derived the same answer.
    // FINDING: this is NOT way prediction (nothing is guessed; the "way" is
    // a proven-correct memoized fact, not a speculative one) and it is not
    // even the C910-style `iwpe` speculative-way-read rv12 modeled -- it is
    // a low-power tag-hit-buffer/bypass, exactly as the extraction note's
    // open item suspected. Per the design doc's deferred-items posture
    // (S2.2 table, "any way-prediction beyond what's structurally present")
    // and because this is a pure power optimization with NO behavioral
    // effect when disabled, rv906 does NOT build the buffer at all for M1 --
    // `cp0_ifu_iwpe` stays a frozen port (tied to `ICACHE_IWPE_DEFAULT`=0 by
    // whichever caller drives it) and this module always performs the full
    // tag compare and reads both ways every cycle (see ARRAYS/HIT below).
    // Structurally: there is no storage to "leave disabled" here (unlike a
    // real way predictor, which would need a table); the honest clone of
    // "iwpe=0" is simply "the bypass logic does not exist in this build."
    //=========================================================================

    //-------------------------------------------------------------------------
    // GEOMETRY
    //-------------------------------------------------------------------------
    localparam SET_W       = ICACHE_TAG_IDX_W;   // 8   (256 sets)
    localparam BANKS       = ICACHE_DATA_BANKS;  // 4
    localparam BANK_ADDR_W = ICACHE_DATA_IDX_W;  // 11  (2048 rows/bank)

    // DATA ARRAY ORGANIZATION (extraction note S10/risks: "icache_data_idx
    // bit-slice reconciliation... left for Task 2 to reconcile"). The real
    // aq_ifu_icache_data_array.v interleaves banks across BOTH ways to let 4
    // physical banks answer a simultaneous 2-way read (way0 bank = {pa[3]^1,
    // pa[2]}, way1 bank = {pa[3], pa[2]}, traced from icache_data_array.v:
    // 210-241's cen_b equations) purely to avoid instantiating 8 banks for a
    // 2-way array -- an ASIC density/routing optimization with no externally
    // observable behavior difference from a simpler layout. rv906 clones the
    // CONFIRMED FACT (4 total 2048x32 banks, 32KB across both ways, IFU notes
    // S3) but resolves the bit-slice question with a clean, documented, and
    // EQUIVALENT layout instead of replicating the interleave: way0 owns
    // banks {0,1}, way1 owns banks {2,3}, selected within its pair by
    // addr[2]; the row address {set[7:0], addr[5:4], addr[3]} is common to
    // both ways' bank pair, still 11 bits (SET_W+3 = 8+3 = BANK_ADDR_W). This
    // is the SEAM-NOTE-style liberty rv12 itself took for its own array
    // reorganization (rv12 ICache.v header, "SEAM NOTE") -- documented here
    // rather than silently assumed, per the plan's explicit ask to resolve
    // this open item, not guess it.
    //-------------------------------------------------------------------------
    // REQUEST PIPE (cycle A: address issue into the SRAMs; cycle B: SRAM Q
    // valid, tag compare + MMU response, both zero-latency vs. cycle A --
    // icache.v:749-776, "one module, one internal flop" 2-cycle access)
    //-------------------------------------------------------------------------
    reg         icache_rd_vld;     // cycle-B "a compare is pending" flop
    reg  [63:0] icache_rd_addr;    // registered fetch VA (icache.v:759-776)
    reg         icache_en_reg;     // cp0_ifu_icache_en, sampled with rd_addr

    wire [63:0] icache_va = pcgen_icache_va;     // no pf/diag-read mux (dropped, see below)

    wire icache_pa_vld;            // fwd decl: driven by MMU section
    wire icache_miss_req;          // fwd decl: driven by HIT section
    wire inv_on;                   // fwd decl: driven by INVALIDATE section
    wire refill_idle;              // fwd decl: driven by REFILL section
    wire inv_block;                // fwd decl: driven by INVALIDATE section

    wire icache_stall  = icache_rd_vld && !icache_pa_vld && !ctrl_icache_abort; // icache.v:938
    wire icache_rd_req = ctrl_icache_req_vld && refill_idle && !inv_on;
    // (real icache_rd_req also ORs in inv_fsm_read/pf_chk_req -- both belong
    // to features rv906 drops for M1: the CP0 diagnostic line-read path (SEAM
    // NOTE at the top of this file) and next-line prefetch, deferred to M8
    // with the rest of performance tuning, same precedent rv12 set for
    // C910's pf FSM.)
    wire icache_rd_cen = icache_rd_req && !icache_stall;
    // ref_rdy adds "!inv_on" vs. the literal icache.v:939 formula
    // (`ref_fsm_idle&&pf_fsm_idle&&!miss&&!stall`, no inv term): the donor
    // gets away without it because `icache_rd_req` already excludes
    // `inv_fsm_idle`, so a grant with no corresponding read could only arise
    // there if a caller asserted `ctrl_icache_req_vld` while invalidating
    // against the donor's own convention. rv906 defends the grant signal
    // itself instead of relying on the caller to know that convention --
    // documented deviation, not a silent guess.
    wire ref_rdy = refill_idle && !inv_on && !icache_miss_req && !icache_stall;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            icache_rd_vld <= 1'b0;
        else if (icache_rd_cen && ref_rdy)
            icache_rd_vld <= 1'b1;
        else if (icache_pa_vld || ctrl_icache_abort)
            icache_rd_vld <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            icache_rd_addr <= 64'b0;
            icache_en_reg  <= 1'b0;
        end
        else if (icache_rd_cen && ref_rdy) begin
            icache_rd_addr <= icache_va;
            icache_en_reg  <= cp0_ifu_icache_en;
        end
    end

    //-------------------------------------------------------------------------
    // MMU/ITLB request-response (icache.v:723-731; M1 stub in RVProc.v
    // answers same-cycle, so this module does not assume any latency beyond
    // what the array's own 1-cycle SRAM read already imposes)
    //-------------------------------------------------------------------------
    assign ifu_mmu_abort  = ctrl_icache_abort;         // cp0_ifu_lpmd_req dropped, not modeled M1
    assign ifu_mmu_va     = icache_rd_addr[63:12];     // 52b VPN, MMU_VA_WIDTH
    assign ifu_mmu_va_vld = icache_rd_vld;

    assign icache_pa_vld = mmu_ifu_pa_vld;
    wire [39:0] icache_pa = {mmu_ifu_pa, icache_rd_addr[11:0]};

    // mmu_ifu_prot[4:0] bit assignment, pinned here from icache.v's own
    // consumers (icache.v:730,809,839-842; NOT confirmed from the ITLB RTL
    // itself, out of scope, but this is the exact bit usage the ICache side
    // depends on so it has to be pinned somewhere -- Task 2 pins it):
    //   [4] pgflt/deny (forces a "hit" so the fault reports instead of
    //       triggering a refill, icache.v:786-787,809)
    //   [3] supv_mode (-> AXI ARPROT)      [2] cacheable (allocate gate)
    //   [1] bufferable (-> AXI ARCACHE)    [0] secure (reserved, unused M1)
    // NOTE: RVProc.v's M1 MMU stub ties this field to all-1s as a permissive
    // placeholder (its own header flags the encoding as unconfirmed). Under
    // THIS pinned encoding that would force bit[4]=1, i.e. every fetch would
    // report a permanent page fault -- clearly not the M1 stub's intent (a
    // bare, fault-free physical mapping). Fixed at RVProc.v's stub assign
    // (an internal wire assignment, not a port -- the freeze covers ports
    // only) alongside this commit; see RVProc.v's own comment at that line.
    wire icache_deny = icache_pa_vld && mmu_ifu_access_fault;
    wire prot_pgflt  = mmu_ifu_prot[4];

    //=========================================================================
    // SECTION: INVALIDATE  (icache.v:1176-1301 IOP FSM, INV_ALL path only --
    // per-line VA/PA invalidate and the CP0 diagnostic read are out of scope,
    // SEAM NOTE at top of file / design doc deferred list)
    //=========================================================================
    localparam I_IDLE = 1'b0, I_ALL = 1'b1;

    reg                inv_state;
    reg  [SET_W-1:0]   inv_cnt;
    reg                inv_req_r;
    reg                inv_pend;

    wire inv_req_rise = cp0_ifu_icache_inv_req && !inv_req_r;
    wire inv_over     = (inv_cnt == {SET_W{1'b0}});
    wire inv_start    = !inv_on && (inv_pend || inv_req_rise) && refill_idle;

    assign inv_on    = (inv_state != I_IDLE);
    assign inv_block = inv_on || inv_pend || cp0_ifu_icache_inv_req;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inv_state <= I_IDLE;
            inv_cnt   <= {SET_W{1'b0}};
            inv_req_r <= 1'b0;
            inv_pend  <= 1'b0;
        end
        else begin
            inv_req_r <= cp0_ifu_icache_inv_req;

            if (inv_start)         inv_pend <= 1'b0;
            else if (inv_req_rise) inv_pend <= 1'b1;

            if (inv_start) begin
                inv_state <= I_ALL;
                inv_cnt   <= {SET_W{1'b1}};             // 255 -> 0, 256 sets
            end
            else if (inv_on) begin
                if (inv_over) inv_state <= I_IDLE;
                else          inv_cnt   <= inv_cnt - {{(SET_W-1){1'b0}}, 1'b1};
            end
        end
    end

    assign ifu_cp0_icache_inv_done = inv_on && inv_over;   // one-cycle pulse

    //=========================================================================
    // SECTION: ARRAYS      (tag: icache_tag_array.v; data: icache_data_array.v)
    //=========================================================================
    wire [SET_W-1:0] tag_rd_addr = icache_va[13:6];       // "32KB: tag index addr[13:6]"

    wire tag_fifo_bit;
    wire tag_way1_vld, tag_way0_vld;
    wire [ICACHE_PTAG_WIDTH-1:0] tag_way1_ptag, tag_way0_ptag;

    // refill-side signals used by the tag write mux (declared here, driven
    // by the REFILL section below -- Verilog elaboration order doesn't care)
    wire        refill_fill, fill_first, fill_last, refill_tagwr;
    wire [7:0]  miss_set;
    wire [27:0] miss_ptag;
    wire        miss_way;

    wire fifo_we  = inv_on || (refill_tagwr && fill_first);
    wire way1_we  = inv_on || (refill_tagwr && miss_way);
    wire way0_we  = inv_on || (refill_tagwr && !miss_way);

    wire [SET_W-1:0] tag_wr_addr = inv_on ? inv_cnt : miss_set;
    wire             tag_wr_active = fifo_we || way1_we || way0_we;
    wire [SET_W-1:0] tag_addr   = tag_wr_active ? tag_wr_addr : tag_rd_addr;
    wire             tag_cen_n  = !(icache_rd_cen || tag_wr_active);
    wire             tag_gwen_n = !tag_wr_active;
    wire [ICACHE_TAG_ROW_WIDTH-1:0] tag_wen_n =
        {~fifo_we, {ICACHE_TAG_WAY_WIDTH{~way1_we}}, {ICACHE_TAG_WAY_WIDTH{~way0_we}}};

    wire                          fifo_din      = inv_on ? 1'b0 : ~miss_way;
    wire                          way_valid_din = inv_on ? 1'b0 : fill_last;
    wire [ICACHE_PTAG_WIDTH-1:0]  way_ptag_din  = (inv_on || fill_first) ? {ICACHE_PTAG_WIDTH{1'b0}}
                                                                          : miss_ptag;
    wire [ICACHE_TAG_ROW_WIDTH-1:0] tag_din =
        {fifo_din, way_valid_din, way_ptag_din, way_valid_din, way_ptag_din};
    wire [ICACHE_TAG_ROW_WIDTH-1:0] tag_q;

    SRAM #(
        .WIDTH (ICACHE_TAG_ROW_WIDTH),
        .DEPTH (ICACHE_SETS)
    ) u_tag_array (
        .clk    (clk),
        .cen_n  (tag_cen_n),
        .gwen_n (tag_gwen_n),
        .wen_n  (tag_wen_n),
        .addr   (tag_addr),
        .d      (tag_din),
        .q      (tag_q)
    );

    assign tag_fifo_bit             = tag_q[ICACHE_TAG_ROW_WIDTH-1];
    assign tag_way1_vld             = tag_q[2*ICACHE_TAG_WAY_WIDTH-1];
    assign tag_way1_ptag            = tag_q[2*ICACHE_TAG_WAY_WIDTH-2 : ICACHE_TAG_WAY_WIDTH];
    assign tag_way0_vld             = tag_q[ICACHE_TAG_WAY_WIDTH-1];
    assign tag_way0_ptag            = tag_q[ICACHE_PTAG_WIDTH-1:0];

    //-------------------------------------------------------------------------
    // Data array: 4 x 2048x32, way0 -> banks{0,1}, way1 -> banks{2,3} (see
    // GEOMETRY note above for the reconciliation this resolves)
    //-------------------------------------------------------------------------
    wire alloc_r;                    // fwd decl (REFILL): allocate into arrays
    wire wr_active;                  // fwd decl (REFILL): data write strobe
    wire [1:0] fill_seq;             // fwd decl (REFILL): 0..3 beat sequence
    wire [DATA_WIDTH-1:0] rdata_r_w; // fwd decl (REFILL): latched line data

    wire [BANK_ADDR_W-1:0] common_row = {icache_va[13:6], icache_va[5:4], icache_va[3]};

    wire [ICACHE_DATA_WIDTH-1:0] bank_q [0:BANKS-1];

    genvar gb;
    generate
        for (gb = 0; gb < BANKS; gb = gb + 1) begin : g_bank
            localparam BIT1 = (gb >> 1) & 1;
            localparam BIT0 = gb & 1;

            // pa3_field: which "half" of the way's 8-row bank this beat's
            // word belongs to -- see GEOMETRY note (way-dedicated bank
            // pair). Derived from the read-side mux below (bank_way0/
            // bank_way1 = {way?pa3:~pa3, pa2}): inverted here to solve, for
            // a FIXED physical bank (BIT1) and the way being refilled, which
            // pa[3] value this bank answers for -- way=1: pa3=BIT1; way=0:
            // pa3=~BIT1.
            wire pa3_field = (BIT1 != 0) ? miss_way : ~miss_way;
            wire [BANK_ADDR_W-1:0] wr_addr = {miss_set, fill_seq, pa3_field};
            wire [3:0]  word_idx = {fill_seq, pa3_field, (BIT0 != 0)};
            wire [ICACHE_DATA_WIDTH-1:0] wr_data = rdata_r_w[{word_idx, 5'b0} +: ICACHE_DATA_WIDTH];

            wire [BANK_ADDR_W-1:0] bank_addr = wr_active ? wr_addr : common_row;
            wire                   bank_cen_n  = !(icache_rd_cen || wr_active);
            wire                   bank_gwen_n = !wr_active;

            SRAM #(
                .WIDTH (ICACHE_DATA_WIDTH),
                .DEPTH (ICACHE_DATA_ROWS)
            ) u_data_bank (
                .clk    (clk),
                .cen_n  (bank_cen_n),
                .gwen_n (bank_gwen_n),
                .wen_n  ({ICACHE_DATA_WIDTH{!wr_active}}),
                .addr   (bank_addr),
                .d      (wr_data),
                .q      (bank_q[gb])
            );
        end
    endgenerate

    //=========================================================================
    // SECTION: HIT          (icache.v:741-812 tag compare + data select)
    //=========================================================================
    wire way1_hit = tag_way1_vld && (tag_way1_ptag == icache_pa[39:12]);
    wire way0_hit = tag_way0_vld && (tag_way0_ptag == icache_pa[39:12]);

    wire icache_hit = ((way1_hit || way0_hit) && icache_en_reg) || icache_deny || prot_pgflt;
    wire icache_hit_vld = icache_hit && icache_rd_vld && icache_pa_vld;
    assign icache_miss_req = !icache_hit && icache_rd_vld && icache_pa_vld && !ctrl_icache_abort;

    wire [1:0] bank_way0 = {~icache_rd_addr[3], icache_rd_addr[2]};
    wire [1:0] bank_way1 = { icache_rd_addr[3], icache_rd_addr[2]};
    wire [ICACHE_DATA_WIDTH-1:0] way0_word = bank_q[bank_way0];
    wire [ICACHE_DATA_WIDTH-1:0] way1_word = bank_q[bank_way1];
    wire [ICACHE_DATA_WIDTH-1:0] icache_hit_inst = way1_hit ? way1_word : way0_word;

    //=========================================================================
    // SECTION: REFILL       (deviation 1: one 512b single-beat AXI read per
    // 64B line, sliced into 4 FILL cycles -- rv12 ICache.v:225-232,320-447
    // FSM shape, C906 semantics (256 sets, way-fifo replacement, no
    // predecode) driven from icache.v:844-1016.)
    //=========================================================================
    localparam R_IDLE = 2'd0, R_REQ = 2'd1, R_WFD = 2'd2, R_FILL = 2'd3;

    reg  [1:0]           refill_state;
    reg  [1:0]           fill_seq_r;
    reg  [39:0]          miss_pa_r;
    reg                  miss_way_r;
    reg                  alloc_r_r, buf_r, supv_r;
    reg  [DATA_WIDTH-1:0] rdata_reg;
    reg                  err_r;
    reg                  refill_data_abort;

    assign refill_idle = (refill_state == R_IDLE);
    wire   refill_req  = (refill_state == R_REQ);
    wire   refill_wfd  = (refill_state == R_WFD);
    assign refill_fill = (refill_state == R_FILL);

    // The IP-equivalent (here: the cycle-B hit-check itself) asks for a line;
    // an invalidate walk outranks it (icache.v:939 ref_rdy excludes a
    // concurrent inv the same way `inv_block` does here).
    wire refill_start = refill_idle && icache_miss_req && !inv_block;

    assign fill_first = refill_fill && (fill_seq_r == 2'd0);
    wire [1:0] fill_last_seq = (alloc_r_r && !err_r) ? 2'd3 : 2'd0;   // uncached/errored: 1 beat only
    assign fill_last  = refill_fill && (fill_seq_r == fill_last_seq);
    assign refill_tagwr = refill_fill && alloc_r_r && !err_r && (fill_first || fill_last);
    assign wr_active     = refill_fill && alloc_r_r && !err_r;
    assign fill_seq       = fill_seq_r;
    assign alloc_r         = alloc_r_r;
    assign rdata_r_w       = rdata_reg;
    assign miss_set  = miss_pa_r[13:6];
    assign miss_ptag = miss_pa_r[39:12];
    assign miss_way  = miss_way_r;

    wire bypass_vld = fill_first && !refill_data_abort;
    wire [3:0] bypass_word_idx = miss_pa_r[5:2];
    wire [ICACHE_DATA_WIDTH-1:0] bypass_word = rdata_reg[{bypass_word_idx, 5'b0} +: ICACHE_DATA_WIDTH];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            refill_state <= R_IDLE;
        else case (refill_state)
            R_IDLE: if (refill_start)   refill_state <= R_REQ;
            R_REQ:  if (axi_i_arready)  refill_state <= R_WFD;
            R_WFD:  if (axi_i_rvalid)   refill_state <= R_FILL;
            R_FILL: if (fill_last)      refill_state <= R_IDLE;
            default:                    refill_state <= R_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fill_seq_r <= 2'd0;
        else if (refill_fill) fill_seq_r <= fill_last ? 2'd0 : (fill_seq_r + 2'd1);
        else fill_seq_r <= 2'd0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            miss_pa_r  <= 40'b0;
            miss_way_r <= 1'b0;
            alloc_r_r  <= 1'b0;
            buf_r      <= 1'b0;
            supv_r     <= 1'b0;
        end
        else if (refill_start) begin
            miss_pa_r  <= icache_pa;
            miss_way_r <= tag_fifo_bit;
            alloc_r_r  <= mmu_ifu_prot[2] && icache_en_reg;
            buf_r      <= mmu_ifu_prot[1];
            supv_r     <= mmu_ifu_prot[3];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_reg <= {DATA_WIDTH{1'b0}};
            err_r     <= 1'b0;
        end
        else if (refill_wfd && axi_i_rvalid) begin
            rdata_reg <= axi_i_rdata;
            err_r     <= axi_i_rresp[1];      // SLVERR/DECERR -> access fault
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) refill_data_abort <= 1'b0;
        else if (ctrl_icache_abort && !refill_idle) refill_data_abort <= 1'b1;
        else if (refill_idle) refill_data_abort <= 1'b0;
    end

    //-------------------------------------------------------------------------
    // AXI read master: ONE 512b single-beat read per 64B line, always at the
    // line-aligned address regardless of cacheable (the SoC bus works in
    // 512b/64B granules; "uncached" differs only in `alloc_r_r`=0, i.e. no
    // array write happens -- data still reaches IPACK via the bypass path,
    // exactly like a cacheable refill's critical-beat bypass, icache.v:1324).
    //-------------------------------------------------------------------------
    assign axi_i_arvalid = refill_req;
    assign axi_i_araddr  = {{(ADDR_WIDTH-40){1'b0}}, miss_pa_r[39:6], 6'b0};
    assign axi_i_arlen   = 8'd0;             // single beat (deviation 1)
    assign axi_i_arsize  = 3'd6;             // 64 bytes
    assign axi_i_arburst = 2'b01;            // INCR
    assign axi_i_arcache = {alloc_r_r, alloc_r_r, 1'b1, buf_r};   // icache.v:1366
    assign axi_i_arprot  = {1'b1, 1'b1, supv_r};                  // icache.v:1367
    assign axi_i_rready  = 1'b1;

    //=========================================================================
    // SECTION: OUTPUT       (icache.v "Rename for Output")
    //=========================================================================
    assign icache_pcgen_grant    = ref_rdy && ctrl_icache_req_vld;
    assign icache_pcgen_addr     = icache_rd_addr[39:0];
    assign icache_pcgen_inst_vld = bypass_vld || icache_hit_vld;
    assign icache_ctrl_stall     = !icache_pa_vld;

    assign icache_ipack_inst_vld = bypass_vld || icache_hit_vld;
    assign icache_ipack_inst     = bypass_vld ? bypass_word : icache_hit_inst;
    assign icache_ipack_acc_err  = bypass_vld ? err_r : icache_deny;
    assign icache_ipack_pgflt    = prot_pgflt;
    assign icache_ipack_unalign  = bypass_vld ? miss_pa_r[1] : icache_pa[1];

    //-------------------------------------------------------------------------
    // Ports intentionally unused for M1 (documented, not accidental):
    //   pcgen_icache_chgflw_vld / pcgen_icache_seq_tag -- iwpe tag-hit-buffer
    //     inputs only (icache.v:602,659,662); no buffer exists (see IWPE
    //     resolution above), so nothing here consumes them.
    //   cp0_ifu_icache_pref_en -- next-line prefetch FSM, deferred to M8 with
    //     the rest of performance tuning (rv12 set the same precedent for
    //     C910's pf FSM).
    //   cp0_ifu_icache_inv_addr / cp0_ifu_icache_inv_type -- per-line VA/PA
    //     invalidate is out of scope for M1 (SEAM NOTE at top of file); every
    //     cp0_ifu_icache_inv_req pulse is treated as INV_ALL.
    //-------------------------------------------------------------------------
    wire _unused_ok = &{1'b0, pcgen_icache_chgflw_vld, pcgen_icache_seq_tag,
                        cp0_ifu_icache_pref_en, cp0_ifu_icache_inv_addr,
                        cp0_ifu_icache_inv_type};

endmodule
