//=============================================================================
// PLIC.v - Platform-Level Interrupt Controller (using AXI4LSlave interface)
//=============================================================================
// Simplified PLIC for single-core M-mode only:
//   - 8 interrupt sources (configurable)
//   - 1 context (M-mode only)
//   - 3-bit prio (0-7, 0 = disabled)
//
// Memory Map (base = 0x0C000000):
//   0x000004: source 1 prio (4 bytes, only bits[2:0] used)
//   0x000008: source 2 prio
//   ...
//   0x00001C: source 7 prio
//   0x001000: pending bits (bits[7:1] = sources 1-7, bit 0 reserved)
//   0x002000: enable bits context 0 (bits[7:1] = sources 1-7)
//   0x200000: prio threshold context 0 (bits[2:0])
//   0x200004: claim/complete context 0
//
// Interrupt flow:
//   1. External interrupt asserted -> gateway latches -> pending set
//   2. Software reads claim -> returns highest prio pending & enabled
//   3. Interrupt delivered to CPU (meip)
//   4. Software writes complete -> clears pending
//
// Reference: RISC-V PLIC Spec 1.0.0
//=============================================================================

module PLIC #(
    parameter XLEN = 64,
    parameter N_SOURCE = 8,         // Number of interrupt sources (including reserved 0)
    parameter N_PRIORITY = 8,       // Priority levels (3 bits: 0-7)
    parameter BASE_ADDR = 32'h0C000000
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // External interrupt inputs (active high, directly from devices)
    // int_src[0] is reserved (always 0)
    input  wire [N_SOURCE-1:0]  int_src,

    // Interrupt output to CPU
    output wire                 meip,       // Machine External Interrupt Pending

    //=========================================================================
    // AXI4-Lite Slave Interface
    //=========================================================================
    input  wire                 axi_awvalid,
    output wire                 axi_awready,
    input  wire [31:0]          axi_awaddr,

    input  wire                 axi_wvalid,
    output wire                 axi_wready,
    input  wire [63:0]          axi_wdata,
    input  wire [7:0]           axi_wstrb,

    output wire                 axi_bvalid,
    input  wire                 axi_bready,
    output wire [1:0]           axi_bresp,

    input  wire                 axi_arvalid,
    output wire                 axi_arready,
    input  wire [31:0]          axi_araddr,

    output wire                 axi_rvalid,
    input  wire                 axi_rready,
    output wire [63:0]          axi_rdata,
    output wire [1:0]           axi_rresp
);

    //=========================================================================
    // Parameters
    //=========================================================================
    localparam PRIO_BITS = 3;  // log2(N_PRIORITY)

    //=========================================================================
    // AXI4-Lite Slave Interface Instance
    //=========================================================================
    wire [31:0] dev_raddr;
    wire [31:0] dev_waddr;
    wire [63:0] dev_wdata;
    wire [7:0]  dev_wstrb;
    wire        dev_read;
    wire        dev_write;
    reg  [63:0] dev_rdata;

    AXI4LSlave #(
        .ADDR_WIDTH(32),
        .DATA_WIDTH(64)
    ) u_axi_slave (
        .clk        (clk),
        .rst_n      (rst_n),
        // AXI4-Lite interface
        .axi_awvalid(axi_awvalid),
        .axi_awready(axi_awready),
        .axi_awaddr (axi_awaddr),
        .axi_wvalid (axi_wvalid),
        .axi_wready (axi_wready),
        .axi_wdata  (axi_wdata),
        .axi_wstrb  (axi_wstrb),
        .axi_bvalid (axi_bvalid),
        .axi_bready (axi_bready),
        .axi_bresp  (axi_bresp),
        .axi_arvalid(axi_arvalid),
        .axi_arready(axi_arready),
        .axi_araddr (axi_araddr),
        .axi_rvalid (axi_rvalid),
        .axi_rready (axi_rready),
        .axi_rdata  (axi_rdata),
        .axi_rresp  (axi_rresp),
        // Device interface
        .dev_raddr  (dev_raddr),
        .dev_waddr  (dev_waddr),
        .dev_wdata  (dev_wdata),
        .dev_wstrb  (dev_wstrb),
        .dev_read   (dev_read),
        .dev_write  (dev_write),
        .dev_rdata  (dev_rdata),
        .dev_ready  (1'b1)      // Always ready
    );

    //=========================================================================
    // Registers
    //=========================================================================
    reg [PRIO_BITS-1:0] prio [1:N_SOURCE-1];  // priority (keyword reserved)

    // Pending bits (level-sensitive gateway)
    reg [N_SOURCE-1:0] pending;

    // Enable bits for context 0 (M-mode)
    reg [N_SOURCE-1:0] enable;

    // Priority threshold for context 0
    reg [PRIO_BITS-1:0] threshold;

    // Claimed interrupt (for tracking)
    reg [N_SOURCE-1:0] claimed;

    //=========================================================================
    // Gateway Logic (Level-Sensitive)
    //=========================================================================
    // For level-sensitive interrupts:
    // - pending is set when interrupt is asserted and not claimed
    // - pending is cleared when claimed
    // - interrupt stays pending if still asserted after complete

    wire [N_SOURCE-1:0] gateway_pending;
    genvar gi;
    generate
        for (gi = 1; gi < N_SOURCE; gi = gi + 1) begin : gen_gateway
            assign gateway_pending[gi] = int_src[gi] & ~claimed[gi];
        end
    endgenerate
    assign gateway_pending[0] = 1'b0;  // Source 0 reserved

    //=========================================================================
    // Priority Comparison - Find Highest Priority Pending & Enabled
    //=========================================================================
    reg [PRIO_BITS-1:0] max_prio;
    reg [3:0]           max_id;  // ID of highest prio interrupt (0 = none)

    integer i;
    always @(*) begin
        max_prio = 0;
        max_id = 0;
        for (i = 1; i < N_SOURCE; i = i + 1) begin
            if (pending[i] && enable[i]) begin
                if (prio[i] > max_prio) begin
                    max_prio = prio[i];
                    max_id = i[3:0];
                end
            end
        end
    end

    //=========================================================================
    // Interrupt Output
    //=========================================================================
    // meip is asserted when highest prio > threshold
    assign meip = (max_prio > threshold) && (max_id != 0);

    //=========================================================================
    // Read Address Decode
    //=========================================================================
    wire [23:0] rd_offset = dev_raddr[23:0];
    wire rd_prio      = (rd_offset >= 24'h000004) && (rd_offset <= 24'h00001C);
    wire rd_pending   = (rd_offset == 24'h001000);
    wire rd_enable    = (rd_offset == 24'h002000);
    wire rd_threshold = (rd_offset == 24'h200000);
    wire rd_claim     = (rd_offset == 24'h200004);
    wire [2:0] rd_prio_idx = (rd_offset[4:2] - 1);

    //=========================================================================
    // Write Decode (address + strobe)
    //=========================================================================
    wire [23:0] wr_offset = dev_waddr[23:0];
    wire wr_prio      = dev_write & (wr_offset >= 24'h000004) & (wr_offset <= 24'h00001C);
    wire wr_enable    = dev_write & (wr_offset == 24'h002000);
    wire wr_threshold = dev_write & (wr_offset == 24'h200000);
    wire wr_complete  = dev_write & (wr_offset == 24'h200004);
    wire [2:0] wr_prio_idx = (wr_offset[4:2] - 1);

    //=========================================================================
    // Write Data Alignment (32-bit registers)
    //=========================================================================
    wire [31:0] wdata = dev_wdata[31:0];
    wire [3:0]  wstrb = dev_waddr[2] ? dev_wstrb[7:4] : dev_wstrb[3:0];
    wire [3:0]  complete_id = wdata[3:0];

    //=========================================================================
    // Combinational Read Logic
    //=========================================================================
    // For 32-bit aligned accesses:
    //   addr[2]=0: data returned in bits [31:0]
    //   addr[2]=1: data returned in bits [63:32]
    reg [31:0] rdata;

    always @(*) begin
        if (rd_prio && {{29{1'b0}}, rd_prio_idx} < N_SOURCE-1)
            rdata = {29'b0, prio[rd_prio_idx + 1]};
        else if (rd_pending)
            rdata = {24'b0, pending};
        else if (rd_enable)
            rdata = {24'b0, enable};
        else if (rd_threshold)
            rdata = {29'b0, threshold};
        else if (rd_claim)
            rdata = {28'b0, max_id};
        else
            rdata = 32'b0;
    end

    // Align read data based on address
    always @(*) begin
        if (dev_raddr[2])
            dev_rdata = {rdata, 32'b0};
        else
            dev_rdata = {32'b0, rdata};
    end

    //=========================================================================
    // Claim Logic
    //=========================================================================
    // When claim register is read, mark the interrupt as claimed
    wire do_claim = dev_read && rd_claim && (max_id != 0);

    //=========================================================================
    // Complete Logic
    //=========================================================================
    // When complete register is written, clear the claimed bit
    wire do_complete = wr_complete;

    //=========================================================================
    // Register Updates
    //=========================================================================

    // Priority registers
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (j = 1; j < N_SOURCE; j = j + 1) begin
                prio[j] <= 0;
            end
        end else if (wr_prio) begin
            if ({{29{1'b0}}, wr_prio_idx} < N_SOURCE-1 && wstrb[0]) begin
                prio[wr_prio_idx + 1] <= wdata[PRIO_BITS-1:0];
            end
        end
    end

    // Pending bits (gateway + claim/complete)
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending <= 0;
            claimed <= 0;
        end else begin
            // Update pending from gateway
            for (k = 1; k < N_SOURCE; k = k + 1) begin
                if (gateway_pending[k] && !claimed[k])
                    pending[k] <= 1'b1;
            end

            // Claim: mark as claimed, clear pending
            if (do_claim) begin
                claimed[max_id[2:0]] <= 1'b1;
                pending[max_id[2:0]] <= 1'b0;
            end

            // Complete: clear claimed, re-check pending
            if (do_complete && complete_id != 0 && complete_id < N_SOURCE) begin
                claimed[complete_id[2:0]] <= 1'b0;
                // If interrupt still asserted, set pending again
                if (int_src[complete_id[2:0]])
                    pending[complete_id[2:0]] <= 1'b1;
            end
        end
    end

    // Enable bits
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable <= 0;
        end else if (wr_enable && wstrb[0]) begin
            enable <= {wdata[N_SOURCE-1:1], 1'b0};  // Bit 0 always 0
        end
    end

    // Threshold
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            threshold <= 0;
        end else if (wr_threshold && wstrb[0]) begin
            threshold <= wdata[PRIO_BITS-1:0];
        end
    end

endmodule
