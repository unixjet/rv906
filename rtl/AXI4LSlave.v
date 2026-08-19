//=============================================================================
// AXI4LSlave.v - Generic AXI4-Lite Slave Interface
//=============================================================================
// Handles AXI4-Lite protocol, provides simple device interface.
// Based on C2RTL AXI4L::TSlaveFSM design.
//
// Usage:
//   - Connect AXI4-Lite bus to axi_* ports
//   - Device implements combinational read (dev_rdata based on dev_raddr)
//   - Device captures write on dev_write posedge clk
//
// Device Interface:
//   dev_raddr - Read address (directly from AXI AR channel)
//   dev_waddr - Write address (valid when dev_write)
//   dev_wdata - Write data (valid when dev_write)
//   dev_wstrb - Write strobe (valid when dev_write)
//   dev_read  - Read request pulse (active for 1 cycle)
//   dev_write - Write request pulse (active for 1 cycle)
//   dev_rdata - Read data from device (combinational based on dev_raddr)
//   dev_ready - Device ready (tie to 1 if always ready)
//=============================================================================

module AXI4LSlave #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // AXI4-Lite Slave Interface
    //=========================================================================
    // Write Address Channel
    input  wire                     axi_awvalid,
    output wire                     axi_awready,
    input  wire [ADDR_WIDTH-1:0]    axi_awaddr,

    // Write Data Channel
    input  wire                     axi_wvalid,
    output wire                     axi_wready,
    input  wire [DATA_WIDTH-1:0]    axi_wdata,
    input  wire [DATA_WIDTH/8-1:0]  axi_wstrb,

    // Write Response Channel
    output wire                     axi_bvalid,
    input  wire                     axi_bready,
    output wire [1:0]               axi_bresp,

    // Read Address Channel
    input  wire                     axi_arvalid,
    output wire                     axi_arready,
    input  wire [ADDR_WIDTH-1:0]    axi_araddr,

    // Read Data Channel
    output wire                     axi_rvalid,
    input  wire                     axi_rready,
    output wire [DATA_WIDTH-1:0]    axi_rdata,
    output wire [1:0]               axi_rresp,

    //=========================================================================
    // Device Interface (separate read/write addresses like TSlaveFSM)
    //=========================================================================
    output wire [ADDR_WIDTH-1:0]    dev_raddr,  // Read address
    output wire [ADDR_WIDTH-1:0]    dev_waddr,  // Write address
    output wire [DATA_WIDTH-1:0]    dev_wdata,
    output wire [DATA_WIDTH/8-1:0]  dev_wstrb,
    output wire                     dev_read,   // Read strobe (1 cycle)
    output wire                     dev_write,  // Write strobe (1 cycle)
    input  wire [DATA_WIDTH-1:0]    dev_rdata,  // Combinational read data
    input  wire                     dev_ready   // Device ready (tie to 1 if always ready)
);

    //=========================================================================
    // Write Channel State (Independent AW/W handling)
    //=========================================================================
    reg                     aw_pending;     // AW received, waiting for W
    reg                     w_pending;      // W received, waiting for AW
    reg                     b_pending;      // Both received, waiting for B handshake
    reg [ADDR_WIDTH-1:0]    req_awaddr;     // Latched write address
    reg [DATA_WIDTH-1:0]    req_wdata;      // Latched write data
    reg [DATA_WIDTH/8-1:0]  req_wstrb;      // Latched write strobe

    // Write handling - accept AW and W independently
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_pending <= 1'b0;
            w_pending  <= 1'b0;
            b_pending  <= 1'b0;
            req_awaddr <= {ADDR_WIDTH{1'b0}};
            req_wdata  <= {DATA_WIDTH{1'b0}};
            req_wstrb  <= {(DATA_WIDTH/8){1'b0}};
        end else begin
            // Accept AW channel
            if (axi_awvalid && axi_awready) begin
                aw_pending <= 1'b1;
                req_awaddr <= axi_awaddr;
            end

            // Accept W channel
            if (axi_wvalid && axi_wready) begin
                w_pending <= 1'b1;
                req_wdata <= axi_wdata;
                req_wstrb <= axi_wstrb;
            end

            // Both received -> move to b_pending
            if ((aw_pending || (axi_awvalid && axi_awready)) &&
                (w_pending || (axi_wvalid && axi_wready)) &&
                !b_pending) begin
                b_pending  <= 1'b1;
                aw_pending <= 1'b0;
                w_pending  <= 1'b0;
            end

            // B handshake complete
            if (axi_bvalid && axi_bready) begin
                b_pending <= 1'b0;
            end
        end
    end

    // Write trigger: both channels ready (either latched or arriving now)
    wire do_write = ((aw_pending || (axi_awvalid && axi_awready)) &&
                     (w_pending || (axi_wvalid && axi_wready)) &&
                     !b_pending);

    // Write data selection (latched or current)
    wire [ADDR_WIDTH-1:0]   wr_addr  = aw_pending ? req_awaddr : axi_awaddr;
    wire [DATA_WIDTH-1:0]   wr_wdata = w_pending  ? req_wdata  : axi_wdata;
    wire [DATA_WIDTH/8-1:0] wr_wstrb = w_pending  ? req_wstrb  : axi_wstrb;

    // AXI Write signals - accept AW/W independently until both received
    assign axi_awready = !aw_pending && !b_pending && dev_ready;
    assign axi_wready  = !w_pending && !b_pending && dev_ready;
    assign axi_bvalid  = b_pending;
    assign axi_bresp   = 2'b00;  // OKAY

    //=========================================================================
    // Read Channel State
    //=========================================================================
    reg                     rd_pending;
    reg [DATA_WIDTH-1:0]    rd_data_reg;

    // Read handling
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_pending  <= 1'b0;
            rd_data_reg <= {DATA_WIDTH{1'b0}};
        end else begin
            if (axi_arvalid && axi_arready) begin
                rd_pending  <= 1'b1;
                rd_data_reg <= dev_rdata;  // Capture combinational read data
            end else if (axi_rvalid && axi_rready) begin
                rd_pending <= 1'b0;
            end
        end
    end

    // Read trigger: AR handshake
    wire do_read = axi_arvalid && axi_arready;

    // AXI Read signals
    assign axi_arready = !rd_pending && dev_ready;
    assign axi_rvalid  = rd_pending;
    assign axi_rdata   = rd_data_reg;
    assign axi_rresp   = 2'b00;  // OKAY

    //=========================================================================
    // Device Interface (separate addresses for read and write)
    //=========================================================================
    assign dev_raddr = axi_araddr;  // Read address directly from AR channel
    assign dev_waddr = wr_addr;     // Write address (latched or current)
    assign dev_wdata = wr_wdata;
    assign dev_wstrb = wr_wstrb;
    assign dev_read  = do_read;
    assign dev_write = do_write;

endmodule
