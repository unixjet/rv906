//=============================================================================
// AMR.v -- Adaptive Memory-allocation Restrictor (M3b Task E, OPTIONAL).
//
// Faithful port of donor aq_lsu_amr.v: a streaming-store detector that
// disables write-allocate once it recognizes a program is scanning through
// memory with store-size strides (a "memset/memcpy"-style stream). When the
// detector reaches its FUNC state it raises amr_dc_wa_dis, which the LSU
// applies as `dcache_wa = cp0_wa & !amr_dc_wa_dis` -- subsequent store
// misses are written straight through instead of allocating a line that
// would be evicted moments later (donor aq_lsu_dc.v wa gating).
//
// FSM (donor aq_lsu_amr.v:124-128):
//   IDLE -> (store MISS with amr_en) -> CALS      [latch addr/size]
//   CALS -> (next store, stride==size) -> CHCK    [latch stride]
//   CHCK -> (repeated stride, line_cnt_done) -> FUNC
//   FUNC -> (stride breaks, confidence 0) -> IDLE
// The extra MISS_WAIT state handles a masked first store; rv906 ties the
// mask to 0 (no masked / unit-stride vector stores), so MISS_WAIT is
// retained for structural fidelity but is unreachable.
//
// rv906 adaptations (documented per clone discipline):
//  - No ICG (plain clocked), no ifu_lsu_warm_up scan hook.
//  - dc_amr_st_mask tied 0: the donor's mask covers vector / unalign-split
//    stores that rv906 does not implement.
//  - cp0_lsu_sync_req maps to clean_active (fence.i D-cache walk), the same
//    serialization the PFB uses.
//  - dc_hint_size (donor, bytes) is supplied by the LSU as (1 << dc_size_r).
//=============================================================================
module AMR (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  cp0_lsu_amr,          // MHINT.amr (00=off)
    input  wire        cp0_lsu_dcache_en,
    input  wire        cp0_lsu_sync_req,     // rv906: clean_active
    // DC-stage store training event (one pulse per cacheable store lookup)
    input  wire        dc_amr_st_req,
    input  wire        dc_amr_cancel,        // non-cacheable store seen
    input  wire        dc_amr_st_miss,
    input  wire [4:0]  dc_amr_st_size,       // store size in bytes
    input  wire [39:0] dc_amr_st_addr,
    output wire        amr_dc_wa_dis
);

    parameter PADDR = 40;
    parameter BYTE  = 6'b000001;
    parameter HALF  = 6'b000010;
    parameter WORD  = 6'b000100;
    parameter DWORD = 6'b001000;
    parameter QWORD = 6'b010000;
    parameter EWORD = 6'b100000;

    reg [39:0] amr_addr;
    reg [2:0]  amr_cur_state, amr_next_state;
    reg [5:0]  amr_size;
    reg [5:0]  byte_cnt;
    reg [5:0]  byte_cnt_mask;
    reg [1:0]  confidence;
    reg [5:0]  line_cnt;
    reg [5:0]  line_cnt_mask;
    reg [4:0]  stride_reg;

    //==========================================================================
    // FSM (donor aq_lsu_amr.v:124-203)
    //==========================================================================
    localparam [2:0] AMR_IDLE      = 3'b000;
    localparam [2:0] AMR_MISS_WAIT = 3'b001;
    localparam [2:0] AMR_CALS      = 3'b101;
    localparam [2:0] AMR_CHCK      = 3'b110;
    localparam [2:0] AMR_FUNC      = 3'b111;

    // rv906 ties the store mask to 0 (no vector / unalign-split stores).
    wire dc_amr_st_mask = 1'b0;

    wire amr_flush = !amr_en | !cp0_lsu_dcache_en | cp0_lsu_sync_req | dc_amr_cancel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            amr_cur_state <= AMR_IDLE;
        else if (amr_flush)
            amr_cur_state <= AMR_IDLE;
        else
            amr_cur_state <= amr_next_state;
    end

    wire amr_en        = |cp0_lsu_amr;
    wire amr_start     = dc_amr_st_req & amr_en & dc_amr_st_miss;
    wire amr_start_chk = amr_start & !dc_amr_st_mask;
    wire dc_st_req     = dc_amr_st_req & !dc_amr_st_mask;

    always @* begin
        case (amr_cur_state)
        AMR_IDLE: begin
            if (amr_start_chk)
                amr_next_state = AMR_CALS;
            else if (amr_start)
                amr_next_state = AMR_MISS_WAIT;
            else
                amr_next_state = AMR_IDLE;
        end
        AMR_MISS_WAIT: begin
            if (dc_st_req)
                amr_next_state = AMR_CALS;
            else
                amr_next_state = AMR_MISS_WAIT;
        end
        AMR_CALS: begin
            if (dc_st_req & stride_equal_size)
                amr_next_state = AMR_CHCK;
            else if (dc_st_req)
                amr_next_state = AMR_IDLE;
            else
                amr_next_state = AMR_CALS;
        end
        AMR_CHCK: begin
            if (dc_st_req & stride_hit)
                amr_next_state = line_cnt_done ? AMR_FUNC : AMR_CHCK;
            else if (dc_st_req)
                amr_next_state = AMR_IDLE;
            else
                amr_next_state = AMR_CHCK;
        end
        AMR_FUNC: begin
            if (amr_exit)
                amr_next_state = AMR_IDLE;
            else
                amr_next_state = AMR_FUNC;
        end
        default: amr_next_state = AMR_IDLE;
        endcase
    end

    wire amr_cur_idle = (amr_cur_state == AMR_IDLE);
    wire amr_cur_cals = (amr_cur_state == AMR_CALS);
    wire amr_cur_chk  = (amr_cur_state == AMR_CHCK);
    wire amr_cur_func = (amr_cur_state == AMR_FUNC);

    //==========================================================================
    // datapath (donor aq_lsu_amr.v:205-301)
    //==========================================================================
    wire amr_create_en = !amr_cur_state[2] & dc_st_req & amr_en;

    always @(posedge clk) begin
        if (amr_create_en)
            amr_addr <= dc_amr_st_addr;
        else if (dc_st_req & !amr_cur_idle)
            amr_addr <= dc_amr_st_addr;
    end

    // for masked store insts the donor accumulates size; with mask tied 0
    // only the plain-update branch ever fires.
    always @(posedge clk) begin
        if (dc_st_req)
            amr_size <= {1'b0, dc_amr_st_size};
        else if (dc_amr_st_req & dc_amr_st_mask)
            amr_size <= amr_size + {1'b0, dc_amr_st_size};
    end

    wire [39:0] stride_cal       = dc_amr_st_addr - amr_addr;
    wire [6:0]  stride_raw       = {stride_cal[PADDR-1], stride_cal[5:0]};
    wire [5:0]  stride_neg       = ~stride_raw[5:0] + 6'b1;
    wire [5:0]  stride_abs       = stride_raw[6] ? stride_neg : stride_raw[5:0];
    wire        stride_equal_size = (stride_abs == amr_size);

    always @(posedge clk) begin
        if (amr_cur_cals & dc_st_req)
            stride_reg <= stride_raw[4:0];
    end

    wire stride_hit = (stride_reg == stride_raw[4:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            byte_cnt <= 6'b0;
        else if (amr_cur_cals & dc_st_req)
            byte_cnt <= 6'b0;
        else if (amr_cur_chk & dc_st_req & stride_hit)
            byte_cnt <= byte_cnt + 6'b1;
    end

    always @* begin
        byte_cnt_mask = 6'b0;
        case (amr_size)
        BYTE:    byte_cnt_mask = {6{1'b1}};
        HALF:    byte_cnt_mask = {5{1'b1}};
        WORD:    byte_cnt_mask = {4{1'b1}};
        DWORD:   byte_cnt_mask = {3{1'b1}};
        QWORD:   byte_cnt_mask = {2{1'b1}};
        EWORD:   byte_cnt_mask = 1'b1;
        default: byte_cnt_mask = 6'b0;
        endcase
    end

    wire byte_cnt_done = (byte_cnt & byte_cnt_mask) == ({6{1'b1}} & byte_cnt_mask);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            line_cnt <= 6'b0;
        else if (amr_cur_cals & dc_st_req)
            line_cnt <= 6'b0;
        else if (amr_cur_chk & dc_st_req & byte_cnt_done)
            line_cnt <= line_cnt + 6'b1;
    end

    always @* begin
        line_cnt_mask = 6'b0;
        case (cp0_lsu_amr)
        2'b01:   line_cnt_mask[1:0] = {2{1'b1}};   // 4 lines
        2'b10:   line_cnt_mask[3:0] = {4{1'b1}};   // 16 lines
        2'b11:   line_cnt_mask[5:0] = {6{1'b1}};   // 64 lines
        default: line_cnt_mask     = 6'b0;
        endcase
    end

    wire line_cnt_done = (line_cnt & line_cnt_mask) == ({6{1'b1}} & line_cnt_mask);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            confidence <= 2'b00;
        else if (amr_cur_chk & dc_st_req & line_cnt_done)
            confidence <= 2'b11;
        else if (amr_cur_func & dc_st_req & !stride_hit)
            confidence <= confidence - 2'b1;
    end

    wire amr_exit = dc_st_req & (confidence == 2'b00);

    assign amr_dc_wa_dis = amr_cur_func;

endmodule
