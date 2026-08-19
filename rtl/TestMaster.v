//=============================================================================
// TestMaster.v - rv906 M0 SCAFFOLDING ONLY (deleted when the real core lands
// in M2). Recovered from rv12's git history (commit 5a7a4e7^, before rv12's
// own M1 replaced it) -- the AXI protocol behavior is core-agnostic.
//=============================================================================
// Drop-in replacement for the CPU core inside RVProcAXI: exercises the AXI
// crossbar, MEMCTL/ExtMem path and CLINT, then writes the result to the
// tohost line at 0x90001000 (kept in sync with test/smoke/smoke.ld).
//=============================================================================

module TestMaster #(
    parameter XLEN = 64,
    parameter ILEN = 32,
    parameter RESET_VECTOR = 64'h80000000,
    parameter DATA_WIDTH = 512,
    parameter ADDR_WIDTH = 64
)(
    input  wire                 clk,
    input  wire                 rst_n,

    //=========================================================================
    // AXI Master Interface - ICache (ch[0])
    //=========================================================================
    // Write Address Channel (not used by ICache)
    output wire                     axi_i_awvalid,
    input  wire                     axi_i_awready,
    output wire [ADDR_WIDTH-1:0]    axi_i_awaddr,
    output wire [7:0]               axi_i_awlen,
    output wire [2:0]               axi_i_awsize,
    output wire [1:0]               axi_i_awburst,
    output wire [3:0]               axi_i_awcache,
    output wire [2:0]               axi_i_awprot,

    // Write Data Channel
    output wire                     axi_i_wvalid,
    input  wire                     axi_i_wready,
    output wire [DATA_WIDTH-1:0]    axi_i_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_i_wstrb,
    output wire                     axi_i_wlast,

    // Write Response Channel
    input  wire                     axi_i_bvalid,
    output wire                     axi_i_bready,
    input  wire [1:0]               axi_i_bresp,

    // Read Address Channel
    output wire                     axi_i_arvalid,
    input  wire                     axi_i_arready,
    output wire [ADDR_WIDTH-1:0]    axi_i_araddr,
    output wire [7:0]               axi_i_arlen,
    output wire [2:0]               axi_i_arsize,
    output wire [1:0]               axi_i_arburst,
    output wire [3:0]               axi_i_arcache,
    output wire [2:0]               axi_i_arprot,

    // Read Data Channel
    input  wire                     axi_i_rvalid,
    output wire                     axi_i_rready,
    input  wire [DATA_WIDTH-1:0]    axi_i_rdata,
    input  wire [1:0]               axi_i_rresp,
    input  wire                     axi_i_rlast,

    //=========================================================================
    // AXI Master Interface - DCache (ch[1])
    //=========================================================================
    // Write Address Channel
    output wire                     axi_d_awvalid,
    input  wire                     axi_d_awready,
    output wire [ADDR_WIDTH-1:0]    axi_d_awaddr,
    output wire [7:0]               axi_d_awlen,
    output wire [2:0]               axi_d_awsize,
    output wire [1:0]               axi_d_awburst,
    output wire [3:0]               axi_d_awcache,
    output wire [2:0]               axi_d_awprot,

    // Write Data Channel
    output wire                     axi_d_wvalid,
    input  wire                     axi_d_wready,
    output wire [DATA_WIDTH-1:0]    axi_d_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_d_wstrb,
    output wire                     axi_d_wlast,

    // Write Response Channel
    input  wire                     axi_d_bvalid,
    output wire                     axi_d_bready,
    input  wire [1:0]               axi_d_bresp,

    // Read Address Channel
    output wire                     axi_d_arvalid,
    input  wire                     axi_d_arready,
    output wire [ADDR_WIDTH-1:0]    axi_d_araddr,
    output wire [7:0]               axi_d_arlen,
    output wire [2:0]               axi_d_arsize,
    output wire [1:0]               axi_d_arburst,
    output wire [3:0]               axi_d_arcache,
    output wire [2:0]               axi_d_arprot,

    // Read Data Channel
    input  wire                     axi_d_rvalid,
    output wire                     axi_d_rready,
    input  wire [DATA_WIDTH-1:0]    axi_d_rdata,
    input  wire [1:0]               axi_d_rresp,
    input  wire                     axi_d_rlast,

    //=========================================================================
    // Interrupt Inputs (from CLINT and PLIC)
    //=========================================================================
    input  wire                 mtip,               // Machine Timer Interrupt Pending
    input  wire                 msip,               // Machine Software Interrupt Pending
    input  wire                 meip,               // Machine External Interrupt Pending

    //=========================================================================
    // Control/Status
    //=========================================================================
    output wire                 quitted
);

    // I-side unused: tie all outputs inactive
    assign axi_i_awvalid = 1'b0;  assign axi_i_wvalid = 1'b0;
    assign axi_i_arvalid = 1'b0;  assign axi_i_bready = 1'b1;
    assign axi_i_rready  = 1'b1;  assign axi_i_wlast  = 1'b0;
    assign axi_i_awaddr = {ADDR_WIDTH{1'b0}};  assign axi_i_araddr = {ADDR_WIDTH{1'b0}};
    assign axi_i_awlen = 8'd0;   assign axi_i_arlen = 8'd0;
    assign axi_i_awsize = 3'd6;  assign axi_i_arsize = 3'd6;
    assign axi_i_awburst = 2'b01; assign axi_i_arburst = 2'b01;
    assign axi_i_awcache = 4'd0; assign axi_i_arcache = 4'd0;
    assign axi_i_awprot = 3'd0;  assign axi_i_arprot = 3'd0;
    assign axi_i_wdata = {DATA_WIDTH{1'b0}};
    assign axi_i_wstrb = {DATA_WIDTH/8{1'b0}};

    assign quitted = 1'b0;   // completion is reported via tohost, like the donor

    localparam [63:0] PATTERN     = 64'hC906_5AFE_A5A5_0000; // lane 0 sentinel
    localparam [63:0] PATTERN7    = 64'hDEAD_BEEF_0000_0007; // lane 7 sentinel
    localparam [63:0] PATTERN_ADDR= 64'h0000_0000_9000_2000;
    localparam [63:0] TOHOST_ADDR = 64'h0000_0000_9000_1000; // = smoke.ld tohost
    localparam [63:0] CLINT_MTIME = 64'h0000_0000_0200_BFF8;

    localparam [2:0] S_WRITE  = 3'd0,
                     S_R_ADDR = 3'd1, S_R_DATA = 3'd2,
                     S_C_ADDR = 3'd3, S_C_DATA = 3'd4,
                     S_TOHOST = 3'd5, S_DONE   = 3'd6;

    reg [2:0]  state;
    reg        fail;
    reg        aw_sent, w_sent;   // per-channel AW/W handshake trackers
    reg [12:0] watchdog;          // converts a fabric stall into a reported FAIL
    reg [63:0] waddr_q, araddr_q, wlane0_q, wlane7_q;

    // The AXICrossbar (AXICrossbar.v:223) only grants a write when AWVALID and
    // WVALID are high in the SAME cycle, and MEMCTL asserts BVALID in the same
    // cycle as WREADY -- so AW and W are driven concurrently (each dropping on
    // its own handshake) and BVALID is watched every cycle of the write state.
    // Modeled on the donor's proven AXIMaster.v write channel.
    wire in_write = (state == S_WRITE) || (state == S_TOHOST);
    wire aw_hs    = axi_d_awvalid && axi_d_awready;
    wire w_hs     = axi_d_wvalid  && axi_d_wready;

    assign axi_d_awvalid = in_write && !aw_sent;
    assign axi_d_awaddr  = waddr_q;
    assign axi_d_awlen   = 8'd0;
    assign axi_d_awsize  = 3'd6;          // one full 64-byte beat
    assign axi_d_awburst = 2'b01;
    assign axi_d_awcache = 4'd0;
    assign axi_d_awprot  = 3'd0;

    assign axi_d_wvalid  = in_write && !w_sent;
    assign axi_d_wdata   = {wlane7_q, {(DATA_WIDTH-128){1'b0}}, wlane0_q};
    assign axi_d_wstrb   = {DATA_WIDTH/8{1'b1}};
    assign axi_d_wlast   = axi_d_wvalid;
    assign axi_d_bready  = 1'b1;

    assign axi_d_arvalid = (state == S_R_ADDR) || (state == S_C_ADDR);
    assign axi_d_araddr  = araddr_q;
    assign axi_d_arlen   = 8'd0;
    // The 512->64 AXIWidthAdapter ignores arsize (the 64-bit result is
    // replicated across all lanes); 3'd3 documents the 8-byte intent only.
    assign axi_d_arsize  = (state == S_C_ADDR) ? 3'd3 : 3'd6;
    assign axi_d_arburst = 2'b01;
    assign axi_d_arcache = 4'd0;
    assign axi_d_arprot  = 3'd0;
    assign axi_d_rready  = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_WRITE;
            fail     <= 1'b0;
            aw_sent  <= 1'b0;
            w_sent   <= 1'b0;
            watchdog <= 13'd0;
            waddr_q  <= PATTERN_ADDR;
            araddr_q <= PATTERN_ADDR;
            wlane0_q <= PATTERN;
            wlane7_q <= PATTERN7;
        end else begin
            case (state)
            S_WRITE, S_TOHOST: begin
                if (aw_hs) aw_sent <= 1'b1;
                if (w_hs)  w_sent  <= 1'b1;
                if (axi_d_bvalid && (aw_sent || aw_hs) && (w_sent || w_hs)) begin
                    // bresp matters only for the pattern write; the tohost
                    // write's response has nothing left to report into
                    if (axi_d_bresp != 2'b00) fail <= 1'b1;
                    aw_sent <= 1'b0;
                    w_sent  <= 1'b0;
                    state   <= (state == S_WRITE) ? S_R_ADDR : S_DONE;
                end
            end
            S_R_ADDR: if (axi_d_arready) state <= S_R_DATA;
            S_R_DATA: if (axi_d_rvalid) begin
                if (axi_d_rresp != 2'b00
                    || axi_d_rdata[63:0] != PATTERN
                    || axi_d_rdata[DATA_WIDTH-1 -: 64] != PATTERN7)
                    fail <= 1'b1;
                araddr_q <= CLINT_MTIME;
                state <= S_C_ADDR;
            end
            S_C_ADDR: if (axi_d_arready) state <= S_C_DATA;
            S_C_DATA: if (axi_d_rvalid) begin
                if (axi_d_rresp != 2'b00) fail <= 1'b1;
                // tohost value must be final BEFORE entering the write state:
                // WDATA is driven from the first S_TOHOST cycle on
                wlane0_q <= (fail || (axi_d_rresp != 2'b00)) ? 64'd3 : 64'd1;
                wlane7_q <= 64'd0;
                waddr_q  <= TOHOST_ADDR;
                state    <= S_TOHOST;
            end
            S_DONE:   state <= S_DONE;
            default:  state <= S_DONE;
            endcase

            // Watchdog: if the fabric stalls, still try to report FAIL via
            // tohost instead of dying silently in the testbench timeout.
            if (state != S_TOHOST && state != S_DONE) begin
                watchdog <= watchdog + 13'd1;
                if (watchdog == 13'h1FFF) begin
                    fail     <= 1'b1;
                    aw_sent  <= 1'b0;
                    w_sent   <= 1'b0;
                    wlane0_q <= 64'd3;
                    wlane7_q <= 64'd0;
                    waddr_q  <= TOHOST_ADDR;
                    state    <= S_TOHOST;
                end
            end
        end
    end

endmodule
