//=============================================================================
// AXIWidthAdapter.v - AXI Data Width Converter (Wide to Narrow)
//=============================================================================
// Converts AXI data width from wide (e.g., 512-bit) to narrow (e.g., 64-bit).
// Used for connecting CLINT/PLIC (64-bit AXI4-Lite) to crossbar (512-bit).
//
// Features:
//   - Write: Extracts correct NARROW_WIDTH bits based on address alignment
//   - Read: Replicates narrow data to fill wide bus
//   - Strobe: Extracts correct strobe bits based on address alignment
//   - Single-beat only (no burst support for MMIO)
//
// Address-based selection (512->64 bit, 8 segments):
//   addr[5:3]=0: use wdata[63:0],   wstrb[7:0]
//   addr[5:3]=1: use wdata[127:64], wstrb[15:8]
//   addr[5:3]=2: use wdata[191:128],wstrb[23:16]
//   ...
//=============================================================================

module AXIWidthAdapter #(
    parameter WIDE_DATA_WIDTH  = 512,
    parameter NARROW_DATA_WIDTH = 64,
    parameter ADDR_WIDTH       = 64
)(
    input  wire                              clk,
    input  wire                              rst_n,

    //=========================================================================
    // Wide Side (from Crossbar)
    //=========================================================================
    // Write Address Channel
    input  wire                              w_awvalid,
    output wire                              w_awready,
    input  wire [ADDR_WIDTH-1:0]             w_awaddr,
    input  wire [7:0]                        w_awlen,
    input  wire [2:0]                        w_awsize,
    input  wire [1:0]                        w_awburst,
    input  wire [2:0]                        w_awprot,

    // Write Data Channel
    input  wire                              w_wvalid,
    output wire                              w_wready,
    input  wire [WIDE_DATA_WIDTH-1:0]        w_wdata,
    input  wire [WIDE_DATA_WIDTH/8-1:0]      w_wstrb,
    input  wire                              w_wlast,

    // Write Response Channel
    output wire                              w_bvalid,
    input  wire                              w_bready,
    output wire [1:0]                        w_bresp,

    // Read Address Channel
    input  wire                              w_arvalid,
    output wire                              w_arready,
    input  wire [ADDR_WIDTH-1:0]             w_araddr,
    input  wire [7:0]                        w_arlen,
    input  wire [2:0]                        w_arsize,
    input  wire [1:0]                        w_arburst,
    input  wire [2:0]                        w_arprot,

    // Read Data Channel
    output wire                              w_rvalid,
    input  wire                              w_rready,
    output wire [WIDE_DATA_WIDTH-1:0]        w_rdata,
    output wire [1:0]                        w_rresp,
    output wire                              w_rlast,

    //=========================================================================
    // Narrow Side (to CLINT/PLIC)
    //=========================================================================
    // Write Address Channel
    output wire                              n_awvalid,
    input  wire                              n_awready,
    output wire [31:0]                       n_awaddr,  // CLINT/PLIC use 32-bit addr

    // Write Data Channel
    output wire                              n_wvalid,
    input  wire                              n_wready,
    output wire [NARROW_DATA_WIDTH-1:0]      n_wdata,
    output wire [NARROW_DATA_WIDTH/8-1:0]    n_wstrb,

    // Write Response Channel
    input  wire                              n_bvalid,
    output wire                              n_bready,
    input  wire [1:0]                        n_bresp,

    // Read Address Channel
    output wire                              n_arvalid,
    input  wire                              n_arready,
    output wire [31:0]                       n_araddr,

    // Read Data Channel
    input  wire                              n_rvalid,
    output wire                              n_rready,
    input  wire [NARROW_DATA_WIDTH-1:0]      n_rdata,
    input  wire [1:0]                        n_rresp
);

    //=========================================================================
    // Write Path: Extract correct slice based on address
    //=========================================================================
    // For 512->64 bit conversion, addr[5:3] selects which 64-bit segment
    localparam RATIO = WIDE_DATA_WIDTH / NARROW_DATA_WIDTH;  // 8 for 512/64
    localparam SEL_BITS = $clog2(RATIO);                     // 3 bits

    // AW and W can arrive in the same cycle, so use current AW address if valid
    // Otherwise use latched address from previous AW
    reg [SEL_BITS-1:0] w_addr_sel_r;
    reg                aw_received;  // AW received but B not sent yet

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_addr_sel_r <= {SEL_BITS{1'b0}};
            aw_received <= 1'b0;
        end else begin
            if (w_awvalid && w_awready) begin
                w_addr_sel_r <= w_awaddr[SEL_BITS+2:3];  // [5:3] for 512->64
                aw_received <= 1'b1;
            end
            if (n_bvalid && n_bready) begin
                aw_received <= 1'b0;
            end
        end
    end

    // Use current AW address if AW is valid, else use latched
    wire [SEL_BITS-1:0] sel = w_awvalid ? w_awaddr[SEL_BITS+2:3] : w_addr_sel_r;

    // Extract correct slice from wide data using indexed part-select
    // sel selects which NARROW_DATA_WIDTH segment to use
    wire [NARROW_DATA_WIDTH-1:0] n_wdata_mux = w_wdata[sel * NARROW_DATA_WIDTH +: NARROW_DATA_WIDTH];
    wire [NARROW_DATA_WIDTH/8-1:0] n_wstrb_mux = w_wstrb[sel * (NARROW_DATA_WIDTH/8) +: (NARROW_DATA_WIDTH/8)];

    assign n_awvalid = w_awvalid;
    assign w_awready = n_awready;
    assign n_awaddr  = w_awaddr[31:0];

    assign n_wvalid  = w_wvalid;
    assign w_wready  = n_wready;
    assign n_wdata   = n_wdata_mux;
    assign n_wstrb   = n_wstrb_mux;

    assign w_bvalid  = n_bvalid;
    assign n_bready  = w_bready;
    assign w_bresp   = n_bresp;

    //=========================================================================
    // Read Path: Zero-extend (or replicate) narrow data to wide
    //=========================================================================
    assign n_arvalid = w_arvalid;
    assign w_arready = n_arready;
    assign n_araddr  = w_araddr[31:0];

    assign w_rvalid  = n_rvalid;
    assign n_rready  = w_rready;
    // Replicate 64-bit data to fill 512-bit (8x replication)
    assign w_rdata   = {8{n_rdata}};
    assign w_rresp   = n_rresp;
    assign w_rlast   = 1'b1;  // Single-beat for MMIO

endmodule
