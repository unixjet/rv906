//=============================================================================
// CLINT.v - Core Local Interruptor (using AXI4LSlave interface)
//=============================================================================
// RISC-V CLINT provides:
//   - mtime:    64-bit timer counter (increments every rtc_tick)
//   - mtimecmp: 64-bit timer compare (generates MTIP when mtime >= mtimecmp)
//   - msip:     Software interrupt pending bit
//
// Memory Map (base = 0x02000000):
//   0x0000: msip[0]     - Machine Software Interrupt Pending (1 bit)
//   0x4000: mtimecmp[0] - Machine Timer Compare (64 bits)
//   0xBFF8: mtime       - Machine Timer (64 bits)
//
// Reference: RISC-V Privileged Spec, SiFive CLINT
//=============================================================================

module CLINT #(
    parameter XLEN = 64,
    parameter BASE_ADDR = 32'h02000000
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // RTC tick input (typically 1MHz or 10MHz)
    input  wire                 rtc_tick,

    // Interrupt outputs to CPU
    output wire                 mtip,       // Machine Timer Interrupt Pending
    output wire                 msip,       // Machine Software Interrupt Pending
    output wire [63:0]          mtime_out,  // M6 Task 3: live mtime mirror for the `time` CSR (0xC01)

    //=========================================================================
    // AXI4-Lite Slave Interface
    //=========================================================================
    // Write Address Channel
    input  wire                 axi_awvalid,
    output wire                 axi_awready,
    input  wire [31:0]          axi_awaddr,

    // Write Data Channel
    input  wire                 axi_wvalid,
    output wire                 axi_wready,
    input  wire [63:0]          axi_wdata,
    input  wire [7:0]           axi_wstrb,

    // Write Response Channel
    output wire                 axi_bvalid,
    input  wire                 axi_bready,
    output wire [1:0]           axi_bresp,

    // Read Address Channel
    input  wire                 axi_arvalid,
    output wire                 axi_arready,
    input  wire [31:0]          axi_araddr,

    // Read Data Channel
    output wire                 axi_rvalid,
    input  wire                 axi_rready,
    output wire [63:0]          axi_rdata,
    output wire [1:0]           axi_rresp
);

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
    reg [63:0] mtime;           // Timer counter
    reg [63:0] mtimecmp;        // Timer compare value
    reg        msip_reg;        // Software interrupt pending

    //=========================================================================
    // Read Address Decode
    //=========================================================================
    wire [15:0] rd_offset = dev_raddr[15:0];
    wire rd_msip     = (rd_offset == 16'h0000);
    wire rd_mtimecmp = (rd_offset == 16'h4000);
    wire rd_mtime    = (rd_offset == 16'hBFF8);

    //=========================================================================
    // Write Decode (address + strobe)
    //=========================================================================
    wire [15:0] wr_offset = dev_waddr[15:0];
    wire wr_msip     = dev_write & (wr_offset == 16'h0000);
    wire wr_mtimecmp = dev_write & (wr_offset == 16'h4000);
    wire wr_mtime    = dev_write & (wr_offset == 16'hBFF8);

    //=========================================================================
    // Combinational Read Logic
    //=========================================================================
    always @(*) begin
        if (rd_msip)
            dev_rdata = {63'b0, msip_reg};
        else if (rd_mtimecmp)
            dev_rdata = mtimecmp;
        else if (rd_mtime)
            dev_rdata = mtime;
        else
            dev_rdata = 64'b0;
    end

    //=========================================================================
    // Register Updates
    //=========================================================================

    // mtime: increment on rtc_tick, or software write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtime <= 64'b0;
        end else if (wr_mtime) begin
            // Software write to mtime (byte-wise)
            mtime[7:0]   <= dev_wstrb[0] ? dev_wdata[7:0]   : mtime[7:0];
            mtime[15:8]  <= dev_wstrb[1] ? dev_wdata[15:8]  : mtime[15:8];
            mtime[23:16] <= dev_wstrb[2] ? dev_wdata[23:16] : mtime[23:16];
            mtime[31:24] <= dev_wstrb[3] ? dev_wdata[31:24] : mtime[31:24];
            mtime[39:32] <= dev_wstrb[4] ? dev_wdata[39:32] : mtime[39:32];
            mtime[47:40] <= dev_wstrb[5] ? dev_wdata[47:40] : mtime[47:40];
            mtime[55:48] <= dev_wstrb[6] ? dev_wdata[55:48] : mtime[55:48];
            mtime[63:56] <= dev_wstrb[7] ? dev_wdata[63:56] : mtime[63:56];
        end else if (rtc_tick) begin
            mtime <= mtime + 64'b1;
        end
    end

    // mtimecmp: software write only
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtimecmp <= 64'hFFFFFFFF_FFFFFFFF;  // Max value (no interrupt initially)
        end else if (wr_mtimecmp) begin
            mtimecmp[7:0]   <= dev_wstrb[0] ? dev_wdata[7:0]   : mtimecmp[7:0];
            mtimecmp[15:8]  <= dev_wstrb[1] ? dev_wdata[15:8]  : mtimecmp[15:8];
            mtimecmp[23:16] <= dev_wstrb[2] ? dev_wdata[23:16] : mtimecmp[23:16];
            mtimecmp[31:24] <= dev_wstrb[3] ? dev_wdata[31:24] : mtimecmp[31:24];
            mtimecmp[39:32] <= dev_wstrb[4] ? dev_wdata[39:32] : mtimecmp[39:32];
            mtimecmp[47:40] <= dev_wstrb[5] ? dev_wdata[47:40] : mtimecmp[47:40];
            mtimecmp[55:48] <= dev_wstrb[6] ? dev_wdata[55:48] : mtimecmp[55:48];
            mtimecmp[63:56] <= dev_wstrb[7] ? dev_wdata[63:56] : mtimecmp[63:56];
        end
    end

    // msip: software write only (bit 0)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msip_reg <= 1'b0;
        end else if (wr_msip && dev_wstrb[0]) begin
            msip_reg <= dev_wdata[0];
        end
    end

    //=========================================================================
    // Interrupt Outputs
    //=========================================================================
    assign mtip = (mtime >= mtimecmp);
    assign msip = msip_reg;
    // M6 Task 3: mirror the free-running mtime to the CPU's `time` CSR (0xC01).
    // Combinational mirror of the internal register -- same value the AXI read
    // at offset 0xBFF8 returns, so the CSR and the MMIO view never diverge.
    assign mtime_out = mtime;

endmodule
