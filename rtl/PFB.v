//=============================================================================
// PFB.v -- D-cache stride prefetch buffer (M3b Task D).
//
// Faithful port of donor aq_lsu_pfb_top.v + aq_lsu_pfb.v (+ the aq_prio.v
// round-robin matrix arbiter): 5 tracker entries = 1 GLOBAL (pc_mode=0,
// hit_cnt_max=15) + 4 PC-INDEXED (pc_mode=1, hit_cnt_max=3)
// (aq_lsu_pfb_top.v:221,472-672). Entries are created by cacheable LOAD
// MISSES at the DC stage (pfb_top.v:299,304), train a stride
// (new_load_PA - last_PA, |stride| < 1KB, pfb.v:386-393), confirm it after
// hit_cnt_max repeats (pfb_top.v:494,535), then prefetch addr+stride on
// each LFB grant up to lookahead = stride << pref_dist, pausing (SUSP)
// while running ahead and self-evicting on confidence/timeout decay.
//
// rv906 adaptations (documented per clone discipline):
//  - No ICG (rv906 is plain-clocked): the donor's *_clk gated clocks and
//    the `_dp` clock-enable shadow paths collapse to plain flops enabled
//    by the functional condition.
//  - No ifu_lsu_warm_up (donor scan/test initialization hook).
//  - No virt_idx / priv_mode / wb attribute bits in the request bus:
//    rv906 is PIPT (no alias, M3b Adaptation Decision #3) with PA-based
//    refill and M-mode only until M4; the donor carries those for its
//    VIPT alias check and fill-time permission walk. Prefetches never
//    cross the training 4K page (the cross-4k guards below, pfb.v:487-489),
//    so a PA refill inherits the training load's already-checked access.
//  - pfb_ld_mask (donor dc_pf_amr_mask, aq_lsu_dc.v:1891) is tied 0: AMR
//    is unimplemented until M3b Task E (default off even in the donor).
//  - xx_flush adds clean_active (the fence.i D-cache walk) in place of the
//    donor's cp0_lsu_sync_req / dc_pfb_dca_vld / icc_xx_sync_req, which
//    rv906 folds into that one serialization (pfb_top.v:363-367).
//=============================================================================

//=============================================================================
// One tracker entry (donor aq_lsu_pfb.v, FSM at :191-200).
//=============================================================================
module PFB_ENTRY #(
    parameter        PC_MODE     = 1'b1,   // 1 = PC-indexed, 0 = global
    parameter [3:0]  HIT_CNT_MAX = 4'h3    // confirm after 3 (PC) / 15 (global)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  cp0_lsu_dcache_pref_dist,
    // flush sources
    input  wire        xx_flush,             // global flush (top)
    input  wire        pfb_flush_by_gpfb,    // global entry is prefetching (PC entries only)
    // create / training interface (top-merged load event)
    input  wire        pfb_create_en,        // pulse: entry is (re)created
    input  wire        xx_create_ld_req,     // load-event strobe (= pfb_ld_req_vld)
    input  wire        xx_create_ld_miss,    // the training load missed
    input  wire [14:0] xx_create_pc,         // load PC[15:1]
    input  wire [39:0] xx_create_chk_pa,     // load PA
    // LFB-side grant + fill feedback
    input  wire        pfb_grant,            // this entry's request was granted
    input  wire        lfb_cache_hit,        // pulse: prefetched line was already resident
    input  wire        lfb_cache_miss,       // pulse: prefetch needed a genuine bus fill
    // status back to top
    output wire        pfb_entry_vld,
    output wire        pfb_entry_evict,
    output wire        pfb_entry_in_pf,
    output wire        pfb_entry_req,
    output wire        pfb_dc_hit_pc,
    output wire [33:0] pfb_entry_arbus       // {PA[39:12], index[11:6]}
);

    //==========================================================================
    // FSM (donor aq_lsu_pfb.v:191-315)
    //==========================================================================
    localparam [3:0] PF_IDLE  = 4'b0000;
    localparam [3:0] PF_CALS  = 4'b0011;
    localparam [3:0] PF_CHKS  = 4'b0101;
    localparam [3:0] PF_INIT  = 4'b1000;
    localparam [3:0] PF_REQ   = 4'b1001;
    localparam [3:0] PF_SUSP  = 4'b1010;
    localparam [3:0] PF_EVICT = 4'b1011;

    reg  [3:0] pfb_cur_state, pfb_next_state;
    reg  [39:0] pfb_ld_addr;
    reg  [14:0] pfb_ld_pc;
    reg  [5:0]  time_out_cnt;
    reg  [10:0] stride_val;
    reg  [3:0]  hit_cnt;
    reg  [2:0]  confidence;
    reg         pf_cache_hit;
    reg  [4:0]  pfb_req_stride;
    reg  [27:0] pfb_req_ppn;
    reg  [5:0]  pfb_req_addr_11to6;

    wire pf_start    = pfb_create_en;
    wire dc_ld_pc_hit = (pfb_ld_pc == xx_create_pc);
    // Training strobe seen by THIS entry: every load event for the global
    // entry; only PC-matching loads for a PC entry (donor pfb.v:218).
    wire dc_ld_vld   = xx_create_ld_req & (!PC_MODE | dc_ld_pc_hit);

    assign pfb_dc_hit_pc   = dc_ld_pc_hit & (pfb_cur_state != PF_IDLE);
    assign pfb_entry_vld   = (pfb_cur_state != PF_IDLE);
    assign pfb_entry_evict = (pfb_cur_state == PF_EVICT);
    assign pfb_entry_in_pf = pfb_cur_state[3] & (pfb_cur_state != PF_EVICT);

    // Timeout: 64 load events without a PC match (donor pfb.v:358-368).
    // PC entries in a training state flush to IDLE; prefetching (state[3])
    // ones degrade to EVICT candidates instead (pfb.v:202-214).
    wire time_out0 = (time_out_cnt == 6'd63);

    wire pfb_flush = xx_flush | pfb_flush_by_gpfb |
                     (PC_MODE & (pfb_cur_state != PF_IDLE) & !pfb_cur_state[3] & time_out0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pfb_cur_state <= PF_IDLE;
        else if (pfb_flush)
            pfb_cur_state <= PF_IDLE;
        else if (PC_MODE & pfb_cur_state[3] & time_out0)
            pfb_cur_state <= PF_EVICT;
        else
            pfb_cur_state <= pfb_next_state;
    end

    wire pfb_cur_idle  = (pfb_cur_state == PF_IDLE);
    wire pfb_cur_cals  = (pfb_cur_state == PF_CALS);
    wire pfb_cur_chks  = (pfb_cur_state == PF_CHKS);
    wire pfb_cur_init  = (pfb_cur_state == PF_INIT);
    wire pfb_cur_req   = (pfb_cur_state == PF_REQ);
    wire pfb_cur_susp  = (pfb_cur_state == PF_SUSP);
    wire pfb_next_init = (pfb_next_state == PF_INIT);
    wire pfb_next_req  = (pfb_next_state == PF_REQ);
    wire pfb_next_chks = (pfb_next_state == PF_CHKS);

    //==========================================================================
    // stride calculation (donor pfb.v:381-399)
    //==========================================================================
    wire [39:0] dc_ld_pa     = pfb_cur_idle ? 40'd0 : xx_create_chk_pa;
    wire [39:0] stride_cal   = dc_ld_pa - pfb_ld_addr;
    // signed |stride| < 1KB (pfb.v:387-388)
    wire stride_less_than_1k = ((stride_cal[39:10] == 30'b0) & |stride_cal[9:0]) |
                               (stride_cal[39:10] == {30{1'b1}});
    wire [10:0] stride_new   = {stride_cal[39], stride_cal[9:0]};
    wire stride_hit          = stride_less_than_1k & (stride_val == stride_new);

    //==========================================================================
    // request address / distance (donor pfb.v:449-506)
    //==========================================================================
    // stride in LINES, min +/-1 (pfb.v:452-455)
    wire [4:0] stride_line    = stride_new[10] ? 5'b11111 : 5'b00001;
    wire       stride_less_line = stride_new[10] ? (&stride_new[9:6]) : (~|stride_new[9:6]);
    wire [4:0] req_stride_c   = stride_less_line ? stride_line : stride_new[10:6];

    wire [6:0] pf_init_addr   = {1'b0, xx_create_chk_pa[11:6]}
                                + {{2{req_stride_c[4]}}, req_stride_c[4:0]};
    wire train_addr_cross_4k  = pf_init_addr[6];

    wire [6:0] pfb_req_addr_next = {1'b0, pfb_req_addr_11to6}
                                   + {{2{pfb_req_stride[4]}}, pfb_req_stride[4:0]};
    wire pf_addr_cross_4k     = pfb_req_addr_next[6];

    wire [33:0] pfb_req_addr  = {pfb_req_ppn, pfb_req_addr_11to6};

    // lookahead distance = stride << pref_dist (pfb.v:494-499)
    wire [3:0] distance_sel   = 4'b1 << cp0_lsu_dcache_pref_dist;
    wire [7:0] stride_ext     = {{3{pfb_req_stride[4]}}, pfb_req_stride};
    wire [8:0] stride_ext_dist = {9{distance_sel[0]}} & {stride_ext[7:0], 1'b0} |
                                 {9{distance_sel[1]}} & {stride_ext[6:0], 2'b0} |
                                 {9{distance_sel[2]}} & {stride_ext[5:0], 3'b0} |
                                 {9{distance_sel[3]}} & {stride_ext[4:0], 4'b0};

    wire [5:0] pfb_req_distance = pfb_req_addr[5:0] - pfb_ld_addr[11:6];
    wire pf_reach_max_distance  = pfb_req_stride[4]
                                  ? ({{3{pfb_req_distance[5]}}, pfb_req_distance} <= stride_ext_dist)
                                  : ({{3{pfb_req_distance[5]}}, pfb_req_distance} >= stride_ext_dist);

    wire distance_cross_page = (pfb_req_addr[33:6] != pfb_ld_addr[39:12]);

    //==========================================================================
    // confidence (donor pfb.v:413-431)
    //==========================================================================
    wire [2:0] confidence_inc = (confidence == 3'b111) ? 3'b111 : confidence + 3'b1;
    wire [2:0] confidence_dec = confidence - 3'b1;
    wire confidence_close = (confidence == 3'b0);
    // two consecutive "already resident" fill feedbacks pop the entry
    wire pf_cache_hit_pop = pf_cache_hit & lfb_cache_hit;
    wire confidence_pop   = (dc_ld_vld & confidence_close) | pf_cache_hit_pop;

    //==========================================================================
    // next-state (donor pfb.v:221-300, verbatim)
    //==========================================================================
    always @* begin
        case (pfb_cur_state)
        PF_IDLE: begin
            if (pf_start)
                pfb_next_state = PF_CALS;
            else
                pfb_next_state = PF_IDLE;
        end
        PF_CALS: begin
            if (dc_ld_vld & stride_less_than_1k)
                pfb_next_state = PF_CHKS;
            else if (dc_ld_vld)
                pfb_next_state = PF_IDLE;
            else
                pfb_next_state = PF_CALS;
        end
        PF_CHKS: begin
            if (dc_ld_vld) begin
                if (stride_hit & (hit_cnt == HIT_CNT_MAX))
                    pfb_next_state = PF_INIT;
                else if (!stride_hit)
                    pfb_next_state = PF_IDLE;
                else
                    pfb_next_state = PF_CHKS;
            end
            else
                pfb_next_state = PF_CHKS;
        end
        PF_INIT: begin
            if (dc_ld_vld & stride_hit & !train_addr_cross_4k & xx_create_ld_miss)
                pfb_next_state = PF_REQ;
            else if (confidence_pop)
                pfb_next_state = PF_IDLE;
            else
                pfb_next_state = PF_INIT;
        end
        PF_REQ: begin
            if (pf_addr_cross_4k & pfb_grant)
                pfb_next_state = PF_INIT;
            else if (pf_reach_max_distance)
                pfb_next_state = PF_SUSP;
            else if (confidence_pop)
                pfb_next_state = PF_IDLE;
            else
                pfb_next_state = PF_REQ;
        end
        PF_SUSP: begin
            if (!pf_reach_max_distance)
                pfb_next_state = PF_REQ;
            else if (confidence_pop | distance_cross_page)
                pfb_next_state = PF_IDLE;
            else
                pfb_next_state = PF_SUSP;
        end
        PF_EVICT: begin
            if (dc_ld_vld)
                pfb_next_state = PF_INIT;
            else if (pf_start)
                pfb_next_state = PF_CALS;
            else
                pfb_next_state = PF_EVICT;
        end
        default: pfb_next_state = PF_IDLE;
        endcase
    end

    //==========================================================================
    // datapath registers (donor pfb.v:317-483; the donor's ICG `_dp` shadow
    // enables collapse onto the functional enable in plain-clocked rv906)
    //==========================================================================
    wire ld_addr_update_en = (pfb_cur_cals & pfb_next_chks) |
                             ((pfb_cur_chks | pfb_cur_state[3]) & dc_ld_vld);

    always @(posedge clk) begin
        if (pfb_create_en | ld_addr_update_en)
            pfb_ld_addr <= xx_create_chk_pa;
    end

    always @(posedge clk) begin
        if (pfb_create_en & PC_MODE)
            pfb_ld_pc <= xx_create_pc;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            time_out_cnt <= 6'd0;
        else if (pfb_create_en & PC_MODE)
            time_out_cnt <= 6'd0;
        else if (xx_create_ld_req & !pfb_cur_idle & PC_MODE)
            time_out_cnt <= dc_ld_pc_hit ? 6'd0 : time_out_cnt + 6'd1;
    end

    always @(posedge clk) begin
        if (pfb_cur_cals & dc_ld_vld & stride_less_than_1k)
            stride_val <= stride_new;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            hit_cnt <= 4'd0;
        else if (pfb_create_en)
            hit_cnt <= 4'd0;
        else if (pfb_cur_chks & dc_ld_vld & stride_hit)
            hit_cnt <= hit_cnt + 4'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            confidence <= 3'b111;
        else if (pfb_cur_chks & pfb_next_init)
            confidence <= 3'b111;
        else if ((pfb_cur_init | pfb_cur_req | pfb_cur_susp) & dc_ld_vld & !confidence_close)
            confidence <= stride_hit ? confidence_inc : confidence_dec;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pf_cache_hit <= 1'b0;
        else if ((pfb_cur_chks & pfb_next_init) | lfb_cache_miss)
            pf_cache_hit <= 1'b0;
        else if (lfb_cache_hit)
            pf_cache_hit <= 1'b1;
    end

    always @(posedge clk) begin
        if (pfb_cur_init & pfb_next_req)
            pfb_req_stride <= req_stride_c;
    end

    always @(posedge clk) begin
        if (pfb_cur_init & pfb_next_req)
            pfb_req_ppn <= xx_create_chk_pa[39:12];
    end

    always @(posedge clk) begin
        if (pfb_cur_init & pfb_next_req)
            pfb_req_addr_11to6 <= pf_init_addr[5:0];
        else if (pfb_cur_req & pfb_grant)
            pfb_req_addr_11to6 <= pfb_req_addr_next[5:0];
    end

    //==========================================================================
    // outputs
    //==========================================================================
    assign pfb_entry_req   = pfb_cur_req;
    assign pfb_entry_arbus = pfb_req_addr;

endmodule

//=============================================================================
// PFB top (donor aq_lsu_pfb_top.v): create steering, flush, the 5-entry
// aq_prio round-robin, grant-time suppression and the request mux out.
//=============================================================================
module PFB (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cp0_lsu_dcache_en,
    input  wire        cp0_lsu_dcache_pref_en,
    input  wire [1:0]  cp0_lsu_dcache_pref_dist,
    input  wire        clean_active,        // rv906 xx_flush source (see header)
    // training interface from the LSU DC stage (one load event per pulse)
    input  wire        pfb_ld_vld,
    input  wire        pfb_ld_miss,
    input  wire [15:0] pfb_ld_pc,
    input  wire [39:0] pfb_ld_chk_pa,
    // grant + suppression from the LSU (donor pfb_top.v:395-396)
    input  wire        lfb_grant,           // !lfb_full && no demand create
    input  wire        lfb_hit_idx,         // requested line already in LFB
    input  wire        stb_hit_idx,         // requested line held in STB
    input  wire        vb_hit_idx,          // requested line in the VB
    input  wire        dc_hit_idx,          // requested line is the in-flight demand line
    // LFB fill feedback, one-hot entry-id pulses (donor lfb.v:842-843)
    input  wire [4:0]  lfb_fb_hit,          // prefetch found the line resident
    input  wire [4:0]  lfb_fb_miss,         // prefetch genuinely refilled
    // granted request out (LSU creates an LFB entry on this)
    output wire        pfb_req,
    output wire [39:0] pfb_req_pa,
    output wire [4:0]  pfb_req_id
);

    localparam DEPTH = 4;   // PC-indexed entries (donor pfb_top.v:221)

    //==========================================================================
    // flush (donor pfb_top.v:363-367, rv906 sources per header)
    //==========================================================================
    wire xx_flush = !cp0_lsu_dcache_pref_en | !cp0_lsu_dcache_en | clean_active;

    //==========================================================================
    // create control (donor pfb_top.v:250-306). pfb_ld_mask is tied 0: the
    // donor's dc_pf_amr_mask (aq_lsu_dc.v:1891) comes from AMR, unimplemented
    // until Task E -- with it 0, pfb_ld_miss_ff can never set and the fast
    // init path collapses to pfb_ld_req = pfb_ld_vld, but the full logic
    // stays for the day AMR lands.
    //==========================================================================
    wire pfb_ld_mask = 1'b0;

    reg  pfb_ld_miss_ff;
    reg  pfb_ld_req_mask;

    wire pfb_ld_miss_save = pfb_ld_vld & pfb_ld_miss & cp0_lsu_dcache_pref_en & pfb_ld_mask;
    wire pfb_ld_req_vld   = pfb_ld_vld & !pfb_ld_req_mask & !pfb_ld_mask & cp0_lsu_dcache_pref_en;
    wire pfb_ld_miss_clr  = pfb_ld_req_vld;
    wire pfb_ld_req_miss  = pfb_ld_miss | pfb_ld_miss_ff;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pfb_ld_miss_ff <= 1'b0;
        else if (pfb_ld_miss_save)
            pfb_ld_miss_ff <= 1'b1;
        else if (pfb_ld_miss_clr | xx_flush)
            pfb_ld_miss_ff <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pfb_ld_req_mask <= 1'b0;
        else if (pfb_ld_vld && cp0_lsu_dcache_pref_en)
            pfb_ld_req_mask <= 1'b0;
        else if (pfb_ld_req_vld)
            pfb_ld_req_mask <= 1'b1;
    end

    //==========================================================================
    // entry create steering (donor pfb_top.v:289-359)
    //==========================================================================
    wire [4:0] pfb_entry_vld, pfb_entry_evict, pfb_entry_in_pf, pfb_entry_req;
    wire [4:0] dc_hit_pfb_pc;
    wire [4:0] pfb_create_en;
    wire [33:0] pfb_entry_arbus [0:4];

    wire       gpfb_entry_in_pf = pfb_entry_in_pf[0];
    wire [3:0] ppfb_entry_in_pf = pfb_entry_in_pf[DEPTH:1];
    wire [3:0] dc_hit_ppfb_pc   = dc_hit_pfb_pc[DEPTH:1];
    wire [3:0] ppfb_entry_vld   = pfb_entry_vld[DEPTH:1];
    wire [3:0] ppfb_entry_evict = pfb_entry_evict[DEPTH:1];

    wire ppfb_hit_pc = |dc_hit_ppfb_pc;

    // PC entries: a missed load whose PC no live PC entry is training gets
    // one; not while the global entry is actively prefetching.
    wire ppfb_req_mask   = gpfb_entry_in_pf;
    wire ppfb_create_vld = pfb_ld_req_vld & pfb_ld_req_miss & !ppfb_req_mask & !ppfb_hit_pc;

    // Global entry: any missed load, unless a matching PC entry is itself
    // actively prefetching.
    wire gpfb_req_mask  = |(dc_hit_ppfb_pc & ppfb_entry_in_pf);
    wire gpfb_create_en = pfb_ld_req_vld & pfb_ld_req_miss & !gpfb_req_mask;

    // first-free, else round-evict pointer encoders (donor pfb_top.v:308-336)
    reg [3:0] ppfb_create_ptr, ppfb_evict_ptr;
    always @* begin
        casez (ppfb_entry_vld)
        4'b???0 : ppfb_create_ptr = 4'b0001;
        4'b??01 : ppfb_create_ptr = 4'b0010;
        4'b?011 : ppfb_create_ptr = 4'b0100;
        4'b0111 : ppfb_create_ptr = 4'b1000;
        default : ppfb_create_ptr = 4'b0000;
        endcase
    end
    always @* begin
        casez (ppfb_entry_evict)
        4'b???1 : ppfb_evict_ptr = 4'b0001;
        4'b??10 : ppfb_evict_ptr = 4'b0010;
        4'b?100 : ppfb_evict_ptr = 4'b0100;
        4'b1000 : ppfb_evict_ptr = 4'b1000;
        default : ppfb_evict_ptr = 4'b0000;
        endcase
    end

    wire [3:0] ppfb_create_sel = (&ppfb_entry_vld) ? ppfb_evict_ptr : ppfb_create_ptr;
    wire [3:0] ppfb_create_en  = {DEPTH{ppfb_create_vld}} & ppfb_create_sel;

    assign pfb_create_en = {ppfb_create_en, gpfb_create_en};

    //==========================================================================
    // request arbitration: donor aq_prio.v NUM=5 priority MATRIX round-robin
    // (winner drops to lowest priority on grant). Reset priority makes entry
    // 0 (global) highest, exactly like the donor's reset encoding
    // {prio[i],unused} = 10'b0000011111 << i.
    //==========================================================================
    wire [4:0] arb_valid = pfb_entry_req;
    reg  [4:0] arb_sel;
    wire [4:0] clr_bus   = {5{lfb_grant}} & arb_sel;

    reg [4:0] prio [0:4];
    reg [9:0] prio_rst;              // donor's {prio[i],unused} 10-bit form
    integer pi, si;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (pi = 0; pi < 5; pi = pi + 1) begin
                prio_rst = 10'b0000011111 << pi;
                prio[pi] <= prio_rst[9:5];
            end
        end else if (|clr_bus) begin
            for (pi = 0; pi < 5; pi = pi + 1) begin
                if (clr_bus == (5'd1 << pi))
                    prio[pi] <= ~clr_bus;
                else
                    prio[pi] <= prio[pi] & ~clr_bus;
            end
        end
    end

    always @* begin
        for (si = 0; si < 5; si = si + 1)
            arb_sel[si] = arb_valid[si] && !(|(arb_valid & prio[si]));
    end

    //==========================================================================
    // grant + suppression + request out (donor pfb_top.v:395-424)
    //==========================================================================
    wire pfb_req_grant = lfb_grant &
                         !lfb_hit_idx & !stb_hit_idx & !dc_hit_idx & !vb_hit_idx;

    wire [4:0] pfb_grant = {5{pfb_req_grant}} & arb_sel;

    // selected entry's address bus
    reg [33:0] pfb_arbus;
    integer mi;
    always @* begin
        pfb_arbus = 34'd0;
        for (mi = 0; mi < 5; mi = mi + 1)
            if (arb_sel[mi])
                pfb_arbus = pfb_entry_arbus[mi];
    end

    assign pfb_req    = (|(arb_sel & pfb_entry_req)) & pfb_req_grant & !xx_flush;
    assign pfb_req_pa = {pfb_arbus, 6'b0};
    assign pfb_req_id = arb_sel;

    //==========================================================================
    // the 5 entries: [0] global, [4:1] PC-indexed (donor pfb_top.v:472-668)
    //==========================================================================
    PFB_ENTRY #(.PC_MODE(1'b0), .HIT_CNT_MAX(4'hf)) u_gpfb (
        .clk(clk), .rst_n(rst_n),
        .cp0_lsu_dcache_pref_dist(cp0_lsu_dcache_pref_dist),
        .xx_flush(xx_flush), .pfb_flush_by_gpfb(1'b0),
        .pfb_create_en(pfb_create_en[0]),
        .xx_create_ld_req(pfb_ld_req_vld), .xx_create_ld_miss(pfb_ld_req_miss),
        .xx_create_pc(pfb_ld_pc[15:1]), .xx_create_chk_pa(pfb_ld_chk_pa),
        .pfb_grant(pfb_grant[0]),
        .lfb_cache_hit(lfb_fb_hit[0]), .lfb_cache_miss(lfb_fb_miss[0]),
        .pfb_entry_vld(pfb_entry_vld[0]), .pfb_entry_evict(pfb_entry_evict[0]),
        .pfb_entry_in_pf(pfb_entry_in_pf[0]), .pfb_entry_req(pfb_entry_req[0]),
        .pfb_dc_hit_pc(dc_hit_pfb_pc[0]), .pfb_entry_arbus(pfb_entry_arbus[0])
    );

    genvar gi;
    generate
        for (gi = 1; gi <= DEPTH; gi = gi + 1) begin : PPFB
            PFB_ENTRY #(.PC_MODE(1'b1), .HIT_CNT_MAX(4'h3)) u_ppfb (
                .clk(clk), .rst_n(rst_n),
                .cp0_lsu_dcache_pref_dist(cp0_lsu_dcache_pref_dist),
                .xx_flush(xx_flush), .pfb_flush_by_gpfb(gpfb_entry_in_pf),
                .pfb_create_en(pfb_create_en[gi]),
                .xx_create_ld_req(pfb_ld_req_vld), .xx_create_ld_miss(pfb_ld_req_miss),
                .xx_create_pc(pfb_ld_pc[15:1]), .xx_create_chk_pa(pfb_ld_chk_pa),
                .pfb_grant(pfb_grant[gi]),
                .lfb_cache_hit(lfb_fb_hit[gi]), .lfb_cache_miss(lfb_fb_miss[gi]),
                .pfb_entry_vld(pfb_entry_vld[gi]), .pfb_entry_evict(pfb_entry_evict[gi]),
                .pfb_entry_in_pf(pfb_entry_in_pf[gi]), .pfb_entry_req(pfb_entry_req[gi]),
                .pfb_dc_hit_pc(dc_hit_pfb_pc[gi]), .pfb_entry_arbus(pfb_entry_arbus[gi])
            );
        end
    endgenerate

endmodule
