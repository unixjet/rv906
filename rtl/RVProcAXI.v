//=============================================================================
// RVProcAXI.v - rv906 SoC Wrapper (Top-Level System Integration)
//=============================================================================
// Adapted from the donor rocket-chip project's rocketM wrapper
// ($RC/rtl/rocketM/RVProcAXI.RTL.v). Top-level wrapper that provides G_
// prefix interface for testbench compatibility.
//
// Internally instantiates:
//   - u_core: the rv906 core shell (RVProc.v, M1: IFU+ICache+BPU+FetchSink).
//     M0's TestMaster placeholder core occupied this same instance name and
//     AXI port list; plan Task 4.2 swapped it for RVProc with a one-word
//     change, per the port-identical freeze from Task 1.3.
//   - AXICrossbar (2 masters x 4 slaves)
//   - MEMCTL_AXI4L_step (C2RTL generated memory controller)
//   - CLINT and PLIC (interrupt controllers)
//
// Architecture:
//   u_core (axi_i_*, axi_d_*)
//      └─► AXICrossbar ─┬─► s[0] → MEMCTL → mpin
//                       ├─► s[1] → AXIWidthAdapter → CLINT
//                       ├─► s[2] → AXIWidthAdapter → PLIC
//                       └─► s[3] → UART (external)
//=============================================================================

module RVProcAXI (
    /* global inputs */
    clk, rst_n,

    /* Memory controller inputs */
    G_io_pins_mpin_dout_data_0, G_io_pins_mpin_dout_data_1, G_io_pins_mpin_dout_data_2,
    G_io_pins_mpin_dout_data_3, G_io_pins_mpin_dout_data_4, G_io_pins_mpin_dout_data_5,
    G_io_pins_mpin_dout_data_6, G_io_pins_mpin_dout_data_7,

    /* AXI slave ch_2 inputs (UART) */
    G_axi_bus_s_ch_2_raddr_s_ready, G_axi_bus_s_ch_2_waddr_s_ready,
    G_axi_bus_s_ch_2_rdat_s_data_data_0, G_axi_bus_s_ch_2_rdat_s_data_data_1,
    G_axi_bus_s_ch_2_rdat_s_data_data_2, G_axi_bus_s_ch_2_rdat_s_data_data_3,
    G_axi_bus_s_ch_2_rdat_s_data_data_4, G_axi_bus_s_ch_2_rdat_s_data_data_5,
    G_axi_bus_s_ch_2_rdat_s_data_data_6, G_axi_bus_s_ch_2_rdat_s_data_data_7,
    G_axi_bus_s_ch_2_rdat_s_resp, G_axi_bus_s_ch_2_rdat_s_valid, G_axi_bus_s_ch_2_rdat_s_last,
    G_axi_bus_s_ch_2_wdat_s_ready, G_axi_bus_s_ch_2_wres_s_resp, G_axi_bus_s_ch_2_wres_s_valid,

    /* M6 Task 6: UART interrupt level (C++ device model) -> PLIC source 7 */
    G_io_pins_uart_irq,

    /* M7 Task 5: JTAG debug pads (D-M7-2: 4 pins, no trst_n) +
       DM reset (D-M7-3, mirrors donor ciu_rst_b) */
    jtag_tck, jtag_tms, jtag_tdi, tdt_rst_n,

    /* Memory controller outputs */
    G_io_pins_mpin_addr, G_io_pins_mpin_din_data_0, G_io_pins_mpin_din_data_1,
    G_io_pins_mpin_din_data_2, G_io_pins_mpin_din_data_3, G_io_pins_mpin_din_data_4,
    G_io_pins_mpin_din_data_5, G_io_pins_mpin_din_data_6, G_io_pins_mpin_din_data_7,
    G_io_pins_mpin_size, G_io_pins_mpin_cs, G_io_pins_mpin_we, G_io_pins_mpin_ras, G_io_pins_mpin_cas,

    /* AXI ch_2 outputs (UART) */
    G_axi_bus_s_ch_2_raddr_m_addr, G_axi_bus_s_ch_2_raddr_m_size, G_axi_bus_s_ch_2_raddr_m_valid,
    G_axi_bus_s_ch_2_raddr_m_len, G_axi_bus_s_ch_2_raddr_m_prot, G_axi_bus_s_ch_2_raddr_m_burst,
    G_axi_bus_s_ch_2_waddr_m_addr, G_axi_bus_s_ch_2_waddr_m_size, G_axi_bus_s_ch_2_waddr_m_valid,
    G_axi_bus_s_ch_2_waddr_m_len, G_axi_bus_s_ch_2_waddr_m_prot, G_axi_bus_s_ch_2_waddr_m_burst,
    G_axi_bus_s_ch_2_rdat_m_ready,
    G_axi_bus_s_ch_2_wdat_m_data_data_0, G_axi_bus_s_ch_2_wdat_m_data_data_1,
    G_axi_bus_s_ch_2_wdat_m_data_data_2, G_axi_bus_s_ch_2_wdat_m_data_data_3,
    G_axi_bus_s_ch_2_wdat_m_data_data_4, G_axi_bus_s_ch_2_wdat_m_data_data_5,
    G_axi_bus_s_ch_2_wdat_m_data_data_6, G_axi_bus_s_ch_2_wdat_m_data_data_7,
    G_axi_bus_s_ch_2_wdat_m_strobe, G_axi_bus_s_ch_2_wdat_m_valid, G_axi_bus_s_ch_2_wdat_m_last,
    G_axi_bus_s_ch_2_wres_m_ready,

    /* M7 Task 5: JTAG TDO + DM chip-level reset outputs (chip outputs
       in the donor; dangling in the rv906 harness, documented) */
    jtag_tdo, ndmreset_n, hartreset_n,

    /* Return value */
    G_RVProcAXI_OUT
);

    parameter M_ID = 0;

    //=========================================================================
    // Input Ports
    //=========================================================================
    input        clk;
    input        rst_n;
    input [63:0] G_io_pins_mpin_dout_data_0;
    input [63:0] G_io_pins_mpin_dout_data_1;
    input [63:0] G_io_pins_mpin_dout_data_2;
    input [63:0] G_io_pins_mpin_dout_data_3;
    input [63:0] G_io_pins_mpin_dout_data_4;
    input [63:0] G_io_pins_mpin_dout_data_5;
    input [63:0] G_io_pins_mpin_dout_data_6;
    input [63:0] G_io_pins_mpin_dout_data_7;
    input        G_axi_bus_s_ch_2_raddr_s_ready;
    input        G_axi_bus_s_ch_2_waddr_s_ready;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_0;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_1;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_2;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_3;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_4;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_5;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_6;
    input [63:0] G_axi_bus_s_ch_2_rdat_s_data_data_7;
    input [1:0]  G_axi_bus_s_ch_2_rdat_s_resp;
    input        G_axi_bus_s_ch_2_rdat_s_valid;
    input        G_axi_bus_s_ch_2_rdat_s_last;
    input        G_axi_bus_s_ch_2_wdat_s_ready;
    input [1:0]  G_axi_bus_s_ch_2_wres_s_resp;
    input        G_axi_bus_s_ch_2_wres_s_valid;

    // M6 Task 6: UART interrupt level (from the C++ device model), routed to
    // the PLIC as external source 7 (see u_plic below).
    input        G_io_pins_uart_irq;

    // M7 Task 5: JTAG pads (D-M7-2) + DM reset (D-M7-3). tdt_rst_n is the
    // DM's async reset, independent of the core rst_n (donor ciu_rst_b):
    // the DM survives a core reset (attach-after-crash / ndmreset flow).
    input        jtag_tck;
    input        jtag_tms;
    input        jtag_tdi;
    input        tdt_rst_n;

    //=========================================================================
    // Output Ports
    //=========================================================================
    output [53:0] G_io_pins_mpin_addr;
    output [63:0] G_io_pins_mpin_din_data_0;
    output [63:0] G_io_pins_mpin_din_data_1;
    output [63:0] G_io_pins_mpin_din_data_2;
    output [63:0] G_io_pins_mpin_din_data_3;
    output [63:0] G_io_pins_mpin_din_data_4;
    output [63:0] G_io_pins_mpin_din_data_5;
    output [63:0] G_io_pins_mpin_din_data_6;
    output [63:0] G_io_pins_mpin_din_data_7;
    output [2:0]  G_io_pins_mpin_size;
    output [1:0]  G_io_pins_mpin_cs;
    output        G_io_pins_mpin_we;
    output        G_io_pins_mpin_ras;
    output        G_io_pins_mpin_cas;
    output [53:0] G_axi_bus_s_ch_2_raddr_m_addr;
    output [2:0]  G_axi_bus_s_ch_2_raddr_m_size;
    output        G_axi_bus_s_ch_2_raddr_m_valid;
    output [3:0]  G_axi_bus_s_ch_2_raddr_m_len;
    output [2:0]  G_axi_bus_s_ch_2_raddr_m_prot;
    output [1:0]  G_axi_bus_s_ch_2_raddr_m_burst;
    output [53:0] G_axi_bus_s_ch_2_waddr_m_addr;
    output [2:0]  G_axi_bus_s_ch_2_waddr_m_size;
    output        G_axi_bus_s_ch_2_waddr_m_valid;
    output [3:0]  G_axi_bus_s_ch_2_waddr_m_len;
    output [2:0]  G_axi_bus_s_ch_2_waddr_m_prot;
    output [1:0]  G_axi_bus_s_ch_2_waddr_m_burst;
    output        G_axi_bus_s_ch_2_rdat_m_ready;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_0;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_1;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_2;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_3;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_4;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_5;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_6;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_data_data_7;
    output [63:0] G_axi_bus_s_ch_2_wdat_m_strobe;
    output        G_axi_bus_s_ch_2_wdat_m_valid;
    output        G_axi_bus_s_ch_2_wdat_m_last;
    output        G_axi_bus_s_ch_2_wres_m_ready;

    // M7 Task 5: JTAG TDO + DM chip-level reset outputs (donor
    // tdt_dm_pad_ndmreset_n / tdt_dm_pad_hartreset_n, tdt_top.v:74-75;
    // dangling in the rv906 harness -- the testbench reads them, nothing
    // consumes them yet).
    output       jtag_tdo;
    output       ndmreset_n;
    output       hartreset_n;

    output        G_RVProcAXI_OUT;

    //=========================================================================
    // Address Mapping Parameters
    //=========================================================================
    localparam [31:0] MEM_BASE   = 32'h80000000;
    localparam [31:0] MEM_MASK   = 32'h80000000;  // Upper 2GB
    localparam [31:0] CLINT_BASE = 32'h02000000;
    localparam [31:0] CLINT_MASK = 32'hFFFF0000;  // 64KB
    localparam [31:0] PLIC_BASE  = 32'h0C000000;
    localparam [31:0] PLIC_MASK  = 32'hFF000000;  // 16MB
    localparam [31:0] UART_BASE  = 32'h10000000;
    localparam [31:0] UART_MASK  = 32'hFFFF0000;  // 64KB

    // Slave indices
    localparam SI_MEM   = 0;
    localparam SI_CLINT = 1;
    localparam SI_PLIC  = 2;
    localparam SI_UART  = 3;

    //=========================================================================
    // Internal Wires - Core AXI Interface (I-side - m[0])
    //=========================================================================
    wire        axi_i_awvalid;
    wire        axi_i_awready;
    wire [63:0] axi_i_awaddr;
    wire [ 7:0] axi_i_awlen;
    wire [ 2:0] axi_i_awsize;
    wire [ 1:0] axi_i_awburst;
    wire [ 3:0] axi_i_awcache;
    wire [ 2:0] axi_i_awprot;
    wire        axi_i_wvalid;
    wire        axi_i_wready;
    wire [511:0] axi_i_wdata;
    wire [63:0] axi_i_wstrb;
    wire        axi_i_wlast;
    wire        axi_i_bvalid;
    wire        axi_i_bready;
    wire [ 1:0] axi_i_bresp;
    wire        axi_i_arvalid;
    wire        axi_i_arready;
    wire [63:0] axi_i_araddr;
    wire [ 7:0] axi_i_arlen;
    wire [ 2:0] axi_i_arsize;
    wire [ 1:0] axi_i_arburst;
    wire [ 3:0] axi_i_arcache;
    wire [ 2:0] axi_i_arprot;
    wire        axi_i_rvalid;
    wire        axi_i_rready;
    wire [511:0] axi_i_rdata;
    wire [ 1:0] axi_i_rresp;
    wire        axi_i_rlast;

    //=========================================================================
    // Internal Wires - Core AXI Interface (D-side - m[1])
    //=========================================================================
    wire        axi_d_awvalid;
    wire        axi_d_awready;
    wire [63:0] axi_d_awaddr;
    wire [ 7:0] axi_d_awlen;
    wire [ 2:0] axi_d_awsize;
    wire [ 1:0] axi_d_awburst;
    wire [ 3:0] axi_d_awcache;
    wire [ 2:0] axi_d_awprot;
    wire        axi_d_wvalid;
    wire        axi_d_wready;
    wire [511:0] axi_d_wdata;
    wire [63:0] axi_d_wstrb;
    wire        axi_d_wlast;
    wire        axi_d_bvalid;
    wire        axi_d_bready;
    wire [ 1:0] axi_d_bresp;
    wire        axi_d_arvalid;
    wire        axi_d_arready;
    wire [63:0] axi_d_araddr;
    wire [ 7:0] axi_d_arlen;
    wire [ 2:0] axi_d_arsize;
    wire [ 1:0] axi_d_arburst;
    wire [ 3:0] axi_d_arcache;
    wire [ 2:0] axi_d_arprot;
    wire        axi_d_rvalid;
    wire        axi_d_rready;
    wire [511:0] axi_d_rdata;
    wire [ 1:0] axi_d_rresp;
    wire        axi_d_rlast;

    //=========================================================================
    // Crossbar Master Ports (2D packed arrays)
    //=========================================================================
    // M7 Task 5: N_MASTERS 2->3 -- m[0]=ICache, m[1]=DCache,
    // m[2]=SBA (via SBA_AxiUp, D-M7-4). AXICrossbar.v is parameterized
    // over N_MASTERS (MASTER_ID_BITS = $clog2(N_MASTERS) = 2 for 3).
    localparam N_MASTERS = 3;
    localparam M_ADDR_WIDTH = 64;
    localparam M_DATA_WIDTH = 512;

    wire [N_MASTERS-1:0]                    m_awvalid;
    wire [N_MASTERS-1:0]                    m_awready;
    wire [N_MASTERS-1:0][M_ADDR_WIDTH-1:0]  m_awaddr;
    wire [N_MASTERS-1:0][7:0]               m_awlen;
    wire [N_MASTERS-1:0][2:0]               m_awsize;
    wire [N_MASTERS-1:0][1:0]               m_awburst;
    wire [N_MASTERS-1:0][2:0]               m_awprot;

    wire [N_MASTERS-1:0]                    m_wvalid;
    wire [N_MASTERS-1:0]                    m_wready;
    wire [N_MASTERS-1:0][M_DATA_WIDTH-1:0]  m_wdata;
    wire [N_MASTERS-1:0][M_DATA_WIDTH/8-1:0] m_wstrb;
    wire [N_MASTERS-1:0]                    m_wlast;

    wire [N_MASTERS-1:0]                    m_bvalid;
    wire [N_MASTERS-1:0]                    m_bready;
    wire [N_MASTERS-1:0][1:0]               m_bresp;

    wire [N_MASTERS-1:0]                    m_arvalid;
    wire [N_MASTERS-1:0]                    m_arready;
    wire [N_MASTERS-1:0][M_ADDR_WIDTH-1:0]  m_araddr;
    wire [N_MASTERS-1:0][7:0]               m_arlen;
    wire [N_MASTERS-1:0][2:0]               m_arsize;
    wire [N_MASTERS-1:0][1:0]               m_arburst;
    wire [N_MASTERS-1:0][2:0]               m_arprot;

    wire [N_MASTERS-1:0]                    m_rvalid;
    wire [N_MASTERS-1:0]                    m_rready;
    wire [N_MASTERS-1:0][M_DATA_WIDTH-1:0]  m_rdata;
    wire [N_MASTERS-1:0][1:0]               m_rresp;
    wire [N_MASTERS-1:0]                    m_rlast;

    // Connect ICache to m[0]
    assign m_awvalid[0] = axi_i_awvalid;
    assign axi_i_awready = m_awready[0];
    assign m_awaddr[0]  = axi_i_awaddr;
    assign m_awlen[0]   = axi_i_awlen;
    assign m_awsize[0]  = axi_i_awsize;
    assign m_awburst[0] = axi_i_awburst;
    assign m_awprot[0]  = axi_i_awprot;
    assign m_wvalid[0]  = axi_i_wvalid;
    assign axi_i_wready = m_wready[0];
    assign m_wdata[0]   = axi_i_wdata;
    assign m_wstrb[0]   = axi_i_wstrb;
    assign m_wlast[0]   = axi_i_wlast;
    assign axi_i_bvalid = m_bvalid[0];
    assign m_bready[0]  = axi_i_bready;
    assign axi_i_bresp  = m_bresp[0];
    assign m_arvalid[0] = axi_i_arvalid;
    assign axi_i_arready = m_arready[0];
    assign m_araddr[0]  = axi_i_araddr;
    assign m_arlen[0]   = axi_i_arlen;
    assign m_arsize[0]  = axi_i_arsize;
    assign m_arburst[0] = axi_i_arburst;
    assign m_arprot[0]  = axi_i_arprot;
    assign axi_i_rvalid = m_rvalid[0];
    assign m_rready[0]  = axi_i_rready;
    assign axi_i_rdata  = m_rdata[0];
    assign axi_i_rresp  = m_rresp[0];
    assign axi_i_rlast  = m_rlast[0];

    // Connect DCache to m[1]
    assign m_awvalid[1] = axi_d_awvalid;
    assign axi_d_awready = m_awready[1];
    assign m_awaddr[1]  = axi_d_awaddr;
    assign m_awlen[1]   = axi_d_awlen;
    assign m_awsize[1]  = axi_d_awsize;
    assign m_awburst[1] = axi_d_awburst;
    assign m_awprot[1]  = axi_d_awprot;
    assign m_wvalid[1]  = axi_d_wvalid;
    assign axi_d_wready = m_wready[1];
    assign m_wdata[1]   = axi_d_wdata;
    assign m_wstrb[1]   = axi_d_wstrb;
    assign m_wlast[1]   = axi_d_wlast;
    assign axi_d_bvalid = m_bvalid[1];
    assign m_bready[1]  = axi_d_bready;
    assign axi_d_bresp  = m_bresp[1];
    assign m_arvalid[1] = axi_d_arvalid;
    assign axi_d_arready = m_arready[1];
    assign m_araddr[1]  = axi_d_araddr;
    assign m_arlen[1]   = axi_d_arlen;
    assign m_arsize[1]  = axi_d_arsize;
    assign m_arburst[1] = axi_d_arburst;
    assign m_arprot[1]  = axi_d_arprot;
    assign axi_d_rvalid = m_rvalid[1];
    assign m_rready[1]  = axi_d_rready;
    assign axi_d_rdata  = m_rdata[1];
    assign axi_d_rresp  = m_rresp[1];
    assign axi_d_rlast  = m_rlast[1];

    //=========================================================================
    // M7 Task 5: Debug unit (TDT_DTM + TDT_DM + SBA)
    //
    // TDT_DTM (tck domain): 4 JTAG pads + tdt_rst_n (D-M7-2/3); its DMI
    // -> APB bridge drives the TDT_DM APB slave (donor tdt_top.v wiring:
    // the DMI APB bus lands on the DM, tdt_top.v:179-186).
    // TDT_DM (clk domain): the Debug Module; its core interface crosses
    // into the RVProc instance as new module ports (M7 Task 5); its
    // SBA 128-bit AXI master goes through SBA_AxiUp to crossbar m[2]
    // (D-M7-4). ndmreset_n/hartreset_n are chip-level outputs (donor
    // tdt_dm_pad_* pads, tdt_top.v:74-75), dangling in the harness.
    //
    // OFF-path identity: with JTAG idle (tck=0 -> TAP in
    // Test-Logic-Reset, no scans, DTM APB master idle) and dmactive=0
    // (DM reset value) every DM core-side output sits at its tdt_rst_n
    // reset constant and the SBA master drives no AXI requests, so all
    // existing behavior is bit-identical (design doc OFF-path argument).
    //=========================================================================
    // TDT_DTM <-> TDT_DM APB (donor tdt_dmi_* bus)
    wire        dtm_dm_psel;
    wire        dtm_dm_penable;
    wire        dtm_dm_pwrite;
    wire [11:0] dtm_dm_paddr;
    wire [31:0] dtm_dm_pwdata;
    wire        dtm_dm_pready;
    wire [31:0] dtm_dm_prdata;
    wire        dtm_dm_pslverr;

    // TDT_DM <-> core (DTU) -- crosses into the RVProc instance
    wire        dm_core_halt_req;
    wire        dm_core_resume_req;
    wire        dm_core_halt_on_reset;
    wire        dm_core_ack_havereset;
    wire [31:0] dm_core_itr;
    wire        dm_core_itr_vld;
    wire        dm_core_wr_vld;
    wire [1:0]  dm_core_wr_flg;
    wire [63:0] dm_core_wdata;
    wire        core_dm_halted;
    wire        core_dm_havereset;
    wire        core_dm_itr_done;
    wire        core_dm_retire_debug_expt;
    wire        core_dm_wr_ready;
    wire [63:0] core_dm_rx_data;

    // TDT_DM SBA AXI4 master (128-bit) -> SBA_AxiUp -> crossbar m[2]
    wire [39:0] sba_awaddr;
    wire [3:0]  sba_awlen;
    wire [2:0]  sba_awsize;
    wire [1:0]  sba_awburst;
    wire [2:0]  sba_awprot;
    wire        sba_awvalid;
    wire        sba_awready;
    wire [127:0] sba_wdata;
    wire [15:0] sba_wstrb;
    wire        sba_wvalid;
    wire        sba_wlast;
    wire        sba_wready;
    wire        sba_bready;
    wire [1:0]  sba_bresp;
    wire        sba_bvalid;
    wire [39:0] sba_araddr;
    wire [3:0]  sba_arlen;
    wire [2:0]  sba_arsize;
    wire [1:0]  sba_arburst;
    wire [2:0]  sba_arprot;
    wire        sba_arvalid;
    wire        sba_arready;
    wire [127:0] sba_rdata;
    wire        sba_rvalid;
    wire        sba_rlast;
    wire [1:0]  sba_rresp;
    wire        sba_rready;

    TDT_DTM u_tdt_dtm (
        // JTAG pads (D-M7-2: 4 pins, no trst_n)
        .tck          (jtag_tck),
        .tms          (jtag_tms),
        .tdi          (jtag_tdi),
        .tdo          (jtag_tdo),
        // pclk domain (single-clock rv906: pclk = clk, D-M7-1)
        .pclk         (clk),
        .preset_n     (tdt_rst_n),
        // APB master to the Debug Module (donor DMI APB bus)
        .apbm_psel    (dtm_dm_psel),
        .apbm_penable (dtm_dm_penable),
        .apbm_pwrite  (dtm_dm_pwrite),
        .apbm_paddr   (dtm_dm_paddr),
        .apbm_pwdata  (dtm_dm_pwdata),
        .apbm_pready  (dtm_dm_pready),
        .apbm_prdata  (dtm_dm_prdata),
        .apbm_pslverr (dtm_dm_pslverr)
    );

    TDT_DM u_tdt_dm (
        .clk                  (clk),
        .tdt_rst_n            (tdt_rst_n),
        // APB slave (from the TDT_DTM DMI bridge)
        .dm_paddr             (dtm_dm_paddr),
        .dm_pwrite            (dtm_dm_pwrite),
        .dm_psel              (dtm_dm_psel),
        .dm_penable           (dtm_dm_penable),
        .dm_pwdata            (dtm_dm_pwdata),
        .dm_prdata            (dtm_dm_prdata),
        .dm_pready            (dtm_dm_pready),
        .dm_pslverr           (dtm_dm_pslverr),
        // Core interface (to the RVProc instance below)
        .dm_core_halt_req_o   (dm_core_halt_req),
        .dm_core_resume_req_o (dm_core_resume_req),
        .dm_core_halt_on_reset_o (dm_core_halt_on_reset),
        .dm_core_ack_havereset_o (dm_core_ack_havereset),
        .dm_core_itr_o        (dm_core_itr),
        .dm_core_itr_vld_o    (dm_core_itr_vld),
        .dm_core_wr_vld_o     (dm_core_wr_vld),
        .dm_core_wr_flg_o     (dm_core_wr_flg),
        .dm_core_wdata_o      (dm_core_wdata),
        .dm_core_rstn_o       (hartreset_n),
        .dm_core_ndmreset_n_o (ndmreset_n),
        .core_dm_halted_i     (core_dm_halted),
        .core_dm_havereset_i  (core_dm_havereset),
        .core_dm_itr_done_i   (core_dm_itr_done),
        .core_dm_retire_debug_expt_i (core_dm_retire_debug_expt),
        .core_dm_wr_ready_i   (core_dm_wr_ready),
        .core_dm_rx_data_i    (core_dm_rx_data),
        // SBA AXI4 master (128-bit) -> SBA_AxiUp. Donor-shape ID/cache/
        // lock ports have no rv906 consumer and are left unconnected
        // (the rv906 crossbar has no ID channels and ignores cache/lock);
        // bid/rid are tied 0 (the crossbar never drives them).
        .dm_pad_awid          (),
        .dm_pad_awaddr        (sba_awaddr),
        .dm_pad_awlen         (sba_awlen),
        .dm_pad_awsize        (sba_awsize),
        .dm_pad_awvalid       (sba_awvalid),
        .pad_dm_awready       (sba_awready),
        .dm_pad_wdata         (sba_wdata),
        .dm_pad_wvalid        (sba_wvalid),
        .dm_pad_wlast         (sba_wlast),
        .dm_pad_wstrb         (sba_wstrb),
        .pad_dm_wready        (sba_wready),
        .dm_pad_bready        (sba_bready),
        .pad_dm_bid           (4'b0),
        .pad_dm_bresp         (sba_bresp),
        .pad_dm_bvalid        (sba_bvalid),
        .dm_pad_arid          (),
        .dm_pad_araddr        (sba_araddr),
        .dm_pad_arlen         (sba_arlen),
        .dm_pad_arsize        (sba_arsize),
        .dm_pad_arvalid       (sba_arvalid),
        .pad_dm_arready       (sba_arready),
        .pad_dm_rid           (4'b0),
        .pad_dm_rdata         (sba_rdata),
        .pad_dm_rvalid        (sba_rvalid),
        .pad_dm_rlast         (sba_rlast),
        .pad_dm_rresp         (sba_rresp),
        .dm_pad_rready        (sba_rready),
        .dm_pad_awburst       (sba_awburst),
        .dm_pad_awcache       (),
        .dm_pad_awlock        (),
        .dm_pad_awprot        (sba_awprot),
        .dm_pad_arburst       (sba_arburst),
        .dm_pad_arcache       (),
        .dm_pad_arlock        (),
        .dm_pad_arprot        (sba_arprot)
    );

    SBA_AxiUp u_sba_axiup (
        // 128-bit side (TDT_DM SBA master)
        .m_awaddr  (sba_awaddr),
        .m_awlen   (sba_awlen),
        .m_awsize  (sba_awsize),
        .m_awburst (sba_awburst),
        .m_awprot  (sba_awprot),
        .m_awvalid (sba_awvalid),
        .m_awready (sba_awready),
        .m_wdata   (sba_wdata),
        .m_wstrb   (sba_wstrb),
        .m_wvalid  (sba_wvalid),
        .m_wlast   (sba_wlast),
        .m_wready  (sba_wready),
        .m_bready  (sba_bready),
        .m_bresp   (sba_bresp),
        .m_bvalid  (sba_bvalid),
        .m_araddr  (sba_araddr),
        .m_arlen   (sba_arlen),
        .m_arsize  (sba_arsize),
        .m_arburst (sba_arburst),
        .m_arprot  (sba_arprot),
        .m_arvalid (sba_arvalid),
        .m_arready (sba_arready),
        .m_rdata   (sba_rdata),
        .m_rvalid  (sba_rvalid),
        .m_rlast   (sba_rlast),
        .m_rresp   (sba_rresp),
        .m_rready  (sba_rready),
        // 512-bit side (crossbar master port m[2])
        .s_awaddr  (m_awaddr[2]),
        .s_awlen   (m_awlen[2]),
        .s_awsize  (m_awsize[2]),
        .s_awburst (m_awburst[2]),
        .s_awprot  (m_awprot[2]),
        .s_awvalid (m_awvalid[2]),
        .s_awready (m_awready[2]),
        .s_wdata   (m_wdata[2]),
        .s_wstrb   (m_wstrb[2]),
        .s_wvalid  (m_wvalid[2]),
        .s_wlast   (m_wlast[2]),
        .s_wready  (m_wready[2]),
        .s_bvalid  (m_bvalid[2]),
        .s_bready  (m_bready[2]),
        .s_bresp   (m_bresp[2]),
        .s_araddr  (m_araddr[2]),
        .s_arlen   (m_arlen[2]),
        .s_arsize  (m_arsize[2]),
        .s_arburst (m_arburst[2]),
        .s_arprot  (m_arprot[2]),
        .s_arvalid (m_arvalid[2]),
        .s_arready (m_arready[2]),
        .s_rdata   (m_rdata[2]),
        .s_rvalid  (m_rvalid[2]),
        .s_rlast   (m_rlast[2]),
        .s_rresp   (m_rresp[2]),
        .s_rready  (m_rready[2])
    );

    //=========================================================================
    // Crossbar Slave Ports (2D packed arrays)
    //=========================================================================
    localparam N_SLAVES = 4;
    localparam S_ADDR_WIDTH = 64;
    localparam S_DATA_WIDTH = 512;

    wire [N_SLAVES-1:0]                    s_awvalid;
    wire [N_SLAVES-1:0]                    s_awready;
    wire [N_SLAVES-1:0][S_ADDR_WIDTH-1:0]  s_awaddr;
    wire [N_SLAVES-1:0][7:0]               s_awlen;
    wire [N_SLAVES-1:0][2:0]               s_awsize;
    wire [N_SLAVES-1:0][1:0]               s_awburst;
    wire [N_SLAVES-1:0][2:0]               s_awprot;

    wire [N_SLAVES-1:0]                    s_wvalid;
    wire [N_SLAVES-1:0]                    s_wready;
    wire [N_SLAVES-1:0][S_DATA_WIDTH-1:0]  s_wdata;
    wire [N_SLAVES-1:0][S_DATA_WIDTH/8-1:0] s_wstrb;
    wire [N_SLAVES-1:0]                    s_wlast;

    wire [N_SLAVES-1:0]                    s_bvalid;
    wire [N_SLAVES-1:0]                    s_bready;
    wire [N_SLAVES-1:0][1:0]               s_bresp;

    wire [N_SLAVES-1:0]                    s_arvalid;
    wire [N_SLAVES-1:0]                    s_arready;
    wire [N_SLAVES-1:0][S_ADDR_WIDTH-1:0]  s_araddr;
    wire [N_SLAVES-1:0][7:0]               s_arlen;
    wire [N_SLAVES-1:0][2:0]               s_arsize;
    wire [N_SLAVES-1:0][1:0]               s_arburst;
    wire [N_SLAVES-1:0][2:0]               s_arprot;

    wire [N_SLAVES-1:0]                    s_rvalid;
    wire [N_SLAVES-1:0]                    s_rready;
    wire [N_SLAVES-1:0][S_DATA_WIDTH-1:0]  s_rdata;
    wire [N_SLAVES-1:0][1:0]               s_rresp;
    wire [N_SLAVES-1:0]                    s_rlast;

    //=========================================================================
    // CLINT and PLIC Interrupt Controllers
    //=========================================================================
    wire        clint_mtip;     // Machine Timer Interrupt Pending
    wire        clint_msip;     // Machine Software Interrupt Pending
    wire [63:0] clint_mtime;    // M6 Task 3: CLINT mtime mirror -> CPU `time` CSR
    wire        plic_meip;      // Machine External Interrupt Pending

    // RTC tick divider (divide clk by 100 for ~1MHz RTC from 100MHz system clock)
    reg [6:0]   rtc_div;
    reg         rtc_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rtc_div  <= 7'd0;
            rtc_tick <= 1'b0;
        end else begin
            if (rtc_div == 7'd99) begin
                rtc_div  <= 7'd0;
                rtc_tick <= 1'b1;
            end else begin
                rtc_div  <= rtc_div + 7'd1;
                rtc_tick <= 1'b0;
            end
        end
    end

    //=========================================================================
    // AXIWidthAdapter Wires for CLINT (s[1])
    //=========================================================================
    wire        clint_awvalid;
    wire        clint_awready;
    wire [31:0] clint_awaddr;
    wire        clint_wvalid;
    wire        clint_wready;
    wire [63:0] clint_wdata;
    wire [7:0]  clint_wstrb;
    wire        clint_bvalid;
    wire        clint_bready;
    wire [1:0]  clint_bresp;
    wire        clint_arvalid;
    wire        clint_arready;
    wire [31:0] clint_araddr;
    wire        clint_rvalid;
    wire        clint_rready;
    wire [63:0] clint_rdata;
    wire [1:0]  clint_rresp;

    //=========================================================================
    // AXIWidthAdapter Wires for PLIC (s[2])
    //=========================================================================
    wire        plic_awvalid;
    wire        plic_awready;
    wire [31:0] plic_awaddr;
    wire        plic_wvalid;
    wire        plic_wready;
    wire [63:0] plic_wdata;
    wire [7:0]  plic_wstrb;
    wire        plic_bvalid;
    wire        plic_bready;
    wire [1:0]  plic_bresp;
    wire        plic_arvalid;
    wire        plic_arready;
    wire [31:0] plic_araddr;
    wire        plic_rvalid;
    wire        plic_rready;
    wire [63:0] plic_rdata;
    wire [1:0]  plic_rresp;

    //=========================================================================
    // MEMCTL Internal Wires (s[0])
    //=========================================================================
    wire [53:0] mem_raddr_m_addr;
    wire [2:0]  mem_raddr_m_size;
    wire        mem_raddr_m_valid;
    wire [3:0]  mem_raddr_m_len;
    wire        mem_raddr_s_ready;
    wire [53:0] mem_waddr_m_addr;
    wire [2:0]  mem_waddr_m_size;
    wire        mem_waddr_m_valid;
    wire [3:0]  mem_waddr_m_len;
    wire        mem_waddr_s_ready;
    wire        mem_rdat_m_ready;
    wire [63:0] mem_rdat_s_data_0, mem_rdat_s_data_1, mem_rdat_s_data_2, mem_rdat_s_data_3;
    wire [63:0] mem_rdat_s_data_4, mem_rdat_s_data_5, mem_rdat_s_data_6, mem_rdat_s_data_7;
    wire        mem_rdat_s_resp_1b;
    wire [1:0]  mem_rdat_s_resp;
    wire        mem_rdat_s_valid;
    wire        mem_rdat_s_last;
    wire [63:0] mem_wdat_m_data_0, mem_wdat_m_data_1, mem_wdat_m_data_2, mem_wdat_m_data_3;
    wire [63:0] mem_wdat_m_data_4, mem_wdat_m_data_5, mem_wdat_m_data_6, mem_wdat_m_data_7;
    wire [63:0] mem_wdat_m_strobe;
    wire        mem_wdat_m_valid;
    wire        mem_wdat_s_ready;
    wire        mem_wres_m_ready;
    wire        mem_wres_s_resp_1b;
    wire [1:0]  mem_wres_s_resp;
    wire        mem_wres_s_valid;

    wire [53:0] mem_pin_addr;
    wire [63:0] mem_pin_din_0, mem_pin_din_1, mem_pin_din_2, mem_pin_din_3;
    wire [63:0] mem_pin_din_4, mem_pin_din_5, mem_pin_din_6, mem_pin_din_7;
    wire [ 2:0] mem_pin_size;
    wire [ 1:0] mem_pin_cs;
    wire        mem_pin_we, mem_pin_ras, mem_pin_cas;

    wire        quitted;

    // Zero-extend 1-bit AXI resp to 2-bit
    assign mem_rdat_s_resp = {1'b0, mem_rdat_s_resp_1b};
    assign mem_wres_s_resp = {1'b0, mem_wres_s_resp_1b};

    //=========================================================================
    // Connect MEMCTL to s[0]
    //=========================================================================
    assign mem_raddr_m_addr  = s_araddr[SI_MEM][53:0];
    assign mem_raddr_m_size  = s_arsize[SI_MEM];
    assign mem_raddr_m_valid = s_arvalid[SI_MEM];
    assign mem_raddr_m_len   = s_arlen[SI_MEM][3:0];
    assign s_arready[SI_MEM] = mem_raddr_s_ready;

    assign mem_waddr_m_addr  = s_awaddr[SI_MEM][53:0];
    assign mem_waddr_m_size  = s_awsize[SI_MEM];
    assign mem_waddr_m_valid = s_awvalid[SI_MEM];
    assign mem_waddr_m_len   = s_awlen[SI_MEM][3:0];
    assign s_awready[SI_MEM] = mem_waddr_s_ready;

    assign mem_rdat_m_ready  = s_rready[SI_MEM];
    assign s_rvalid[SI_MEM]  = mem_rdat_s_valid;
    assign s_rdata[SI_MEM]   = {mem_rdat_s_data_7, mem_rdat_s_data_6, mem_rdat_s_data_5, mem_rdat_s_data_4,
                                mem_rdat_s_data_3, mem_rdat_s_data_2, mem_rdat_s_data_1, mem_rdat_s_data_0};
    assign s_rresp[SI_MEM]   = mem_rdat_s_resp;
    assign s_rlast[SI_MEM]   = mem_rdat_s_last;

    assign mem_wdat_m_data_0 = s_wdata[SI_MEM][63:0];
    assign mem_wdat_m_data_1 = s_wdata[SI_MEM][127:64];
    assign mem_wdat_m_data_2 = s_wdata[SI_MEM][191:128];
    assign mem_wdat_m_data_3 = s_wdata[SI_MEM][255:192];
    assign mem_wdat_m_data_4 = s_wdata[SI_MEM][319:256];
    assign mem_wdat_m_data_5 = s_wdata[SI_MEM][383:320];
    assign mem_wdat_m_data_6 = s_wdata[SI_MEM][447:384];
    assign mem_wdat_m_data_7 = s_wdata[SI_MEM][511:448];
    assign mem_wdat_m_strobe = s_wstrb[SI_MEM];
    assign mem_wdat_m_valid  = s_wvalid[SI_MEM];
    assign s_wready[SI_MEM]  = mem_wdat_s_ready;

    assign mem_wres_m_ready  = s_bready[SI_MEM];
    assign s_bvalid[SI_MEM]  = mem_wres_s_valid;
    assign s_bresp[SI_MEM]   = mem_wres_s_resp;

    //=========================================================================
    // AXIWidthAdapter for CLINT (s[1])
    //=========================================================================
    AXIWidthAdapter #(
        .WIDE_DATA_WIDTH(512),
        .NARROW_DATA_WIDTH(64),
        .ADDR_WIDTH(64)
    ) u_clint_adapter (
        .clk        (clk),
        .rst_n      (rst_n),
        // Wide side (from crossbar)
        .w_awvalid  (s_awvalid[SI_CLINT]),
        .w_awready  (s_awready[SI_CLINT]),
        .w_awaddr   (s_awaddr[SI_CLINT]),
        .w_awlen    (s_awlen[SI_CLINT]),
        .w_awsize   (s_awsize[SI_CLINT]),
        .w_awburst  (s_awburst[SI_CLINT]),
        .w_awprot   (s_awprot[SI_CLINT]),
        .w_wvalid   (s_wvalid[SI_CLINT]),
        .w_wready   (s_wready[SI_CLINT]),
        .w_wdata    (s_wdata[SI_CLINT]),
        .w_wstrb    (s_wstrb[SI_CLINT]),
        .w_wlast    (s_wlast[SI_CLINT]),
        .w_bvalid   (s_bvalid[SI_CLINT]),
        .w_bready   (s_bready[SI_CLINT]),
        .w_bresp    (s_bresp[SI_CLINT]),
        .w_arvalid  (s_arvalid[SI_CLINT]),
        .w_arready  (s_arready[SI_CLINT]),
        .w_araddr   (s_araddr[SI_CLINT]),
        .w_arlen    (s_arlen[SI_CLINT]),
        .w_arsize   (s_arsize[SI_CLINT]),
        .w_arburst  (s_arburst[SI_CLINT]),
        .w_arprot   (s_arprot[SI_CLINT]),
        .w_rvalid   (s_rvalid[SI_CLINT]),
        .w_rready   (s_rready[SI_CLINT]),
        .w_rdata    (s_rdata[SI_CLINT]),
        .w_rresp    (s_rresp[SI_CLINT]),
        .w_rlast    (s_rlast[SI_CLINT]),
        // Narrow side (to CLINT)
        .n_awvalid  (clint_awvalid),
        .n_awready  (clint_awready),
        .n_awaddr   (clint_awaddr),
        .n_wvalid   (clint_wvalid),
        .n_wready   (clint_wready),
        .n_wdata    (clint_wdata),
        .n_wstrb    (clint_wstrb),
        .n_bvalid   (clint_bvalid),
        .n_bready   (clint_bready),
        .n_bresp    (clint_bresp),
        .n_arvalid  (clint_arvalid),
        .n_arready  (clint_arready),
        .n_araddr   (clint_araddr),
        .n_rvalid   (clint_rvalid),
        .n_rready   (clint_rready),
        .n_rdata    (clint_rdata),
        .n_rresp    (clint_rresp)
    );

    //=========================================================================
    // AXIWidthAdapter for PLIC (s[2])
    //=========================================================================
    AXIWidthAdapter #(
        .WIDE_DATA_WIDTH(512),
        .NARROW_DATA_WIDTH(64),
        .ADDR_WIDTH(64)
    ) u_plic_adapter (
        .clk        (clk),
        .rst_n      (rst_n),
        // Wide side (from crossbar)
        .w_awvalid  (s_awvalid[SI_PLIC]),
        .w_awready  (s_awready[SI_PLIC]),
        .w_awaddr   (s_awaddr[SI_PLIC]),
        .w_awlen    (s_awlen[SI_PLIC]),
        .w_awsize   (s_awsize[SI_PLIC]),
        .w_awburst  (s_awburst[SI_PLIC]),
        .w_awprot   (s_awprot[SI_PLIC]),
        .w_wvalid   (s_wvalid[SI_PLIC]),
        .w_wready   (s_wready[SI_PLIC]),
        .w_wdata    (s_wdata[SI_PLIC]),
        .w_wstrb    (s_wstrb[SI_PLIC]),
        .w_wlast    (s_wlast[SI_PLIC]),
        .w_bvalid   (s_bvalid[SI_PLIC]),
        .w_bready   (s_bready[SI_PLIC]),
        .w_bresp    (s_bresp[SI_PLIC]),
        .w_arvalid  (s_arvalid[SI_PLIC]),
        .w_arready  (s_arready[SI_PLIC]),
        .w_araddr   (s_araddr[SI_PLIC]),
        .w_arlen    (s_arlen[SI_PLIC]),
        .w_arsize   (s_arsize[SI_PLIC]),
        .w_arburst  (s_arburst[SI_PLIC]),
        .w_arprot   (s_arprot[SI_PLIC]),
        .w_rvalid   (s_rvalid[SI_PLIC]),
        .w_rready   (s_rready[SI_PLIC]),
        .w_rdata    (s_rdata[SI_PLIC]),
        .w_rresp    (s_rresp[SI_PLIC]),
        .w_rlast    (s_rlast[SI_PLIC]),
        // Narrow side (to PLIC)
        .n_awvalid  (plic_awvalid),
        .n_awready  (plic_awready),
        .n_awaddr   (plic_awaddr),
        .n_wvalid   (plic_wvalid),
        .n_wready   (plic_wready),
        .n_wdata    (plic_wdata),
        .n_wstrb    (plic_wstrb),
        .n_bvalid   (plic_bvalid),
        .n_bready   (plic_bready),
        .n_bresp    (plic_bresp),
        .n_arvalid  (plic_arvalid),
        .n_arready  (plic_arready),
        .n_araddr   (plic_araddr),
        .n_rvalid   (plic_rvalid),
        .n_rready   (plic_rready),
        .n_rdata    (plic_rdata),
        .n_rresp    (plic_rresp)
    );

    //=========================================================================
    // CLINT Instance
    //=========================================================================
    CLINT #(
        .XLEN(64),
        .BASE_ADDR(CLINT_BASE)
    ) u_clint (
        .clk            (clk),
        .rst_n          (rst_n),
        .rtc_tick       (rtc_tick),
        .mtip           (clint_mtip),
        .msip           (clint_msip),
        .mtime_out      (clint_mtime),
        .axi_awvalid    (clint_awvalid),
        .axi_awready    (clint_awready),
        .axi_awaddr     (clint_awaddr),
        .axi_wvalid     (clint_wvalid),
        .axi_wready     (clint_wready),
        .axi_wdata      (clint_wdata),
        .axi_wstrb      (clint_wstrb),
        .axi_bvalid     (clint_bvalid),
        .axi_bready     (clint_bready),
        .axi_bresp      (clint_bresp),
        .axi_arvalid    (clint_arvalid),
        .axi_arready    (clint_arready),
        .axi_araddr     (clint_araddr),
        .axi_rvalid     (clint_rvalid),
        .axi_rready     (clint_rready),
        .axi_rdata      (clint_rdata),
        .axi_rresp      (clint_rresp)
    );

    //=========================================================================
    // PLIC Instance
    //=========================================================================
    PLIC #(
        .XLEN(64),
        .N_SOURCE(8),
        .N_PRIORITY(8),
        .BASE_ADDR(PLIC_BASE)
    ) u_plic (
        .clk            (clk),
        .rst_n          (rst_n),
        // M6 Task 6: UART (source 7) interrupt level from the C++ device
        // model, un-tied from 8'b0. Sources 1-6 remain 0 (D-M6-3: single
        // M context, 7 sources; only the UART is wired at M6).
        .int_src        ({G_io_pins_uart_irq, 7'b0}),
        .meip           (plic_meip),
        .axi_awvalid    (plic_awvalid),
        .axi_awready    (plic_awready),
        .axi_awaddr     (plic_awaddr),
        .axi_wvalid     (plic_wvalid),
        .axi_wready     (plic_wready),
        .axi_wdata      (plic_wdata),
        .axi_wstrb      (plic_wstrb),
        .axi_bvalid     (plic_bvalid),
        .axi_bready     (plic_bready),
        .axi_bresp      (plic_bresp),
        .axi_arvalid    (plic_arvalid),
        .axi_arready    (plic_arready),
        .axi_araddr     (plic_araddr),
        .axi_rvalid     (plic_rvalid),
        .axi_rready     (plic_rready),
        .axi_rdata      (plic_rdata),
        .axi_rresp      (plic_rresp)
    );

    //=========================================================================
    // Connect UART to s[3] (external interface)
    //=========================================================================
    assign G_axi_bus_s_ch_2_raddr_m_addr  = s_araddr[SI_UART][53:0];
    assign G_axi_bus_s_ch_2_raddr_m_size  = s_arsize[SI_UART];
    assign G_axi_bus_s_ch_2_raddr_m_valid = s_arvalid[SI_UART];
    assign G_axi_bus_s_ch_2_raddr_m_len   = s_arlen[SI_UART][3:0];
    assign G_axi_bus_s_ch_2_raddr_m_prot  = s_arprot[SI_UART];
    assign G_axi_bus_s_ch_2_raddr_m_burst = s_arburst[SI_UART];
    assign s_arready[SI_UART] = G_axi_bus_s_ch_2_raddr_s_ready;

    assign G_axi_bus_s_ch_2_waddr_m_addr  = s_awaddr[SI_UART][53:0];
    assign G_axi_bus_s_ch_2_waddr_m_size  = s_awsize[SI_UART];
    assign G_axi_bus_s_ch_2_waddr_m_valid = s_awvalid[SI_UART];
    assign G_axi_bus_s_ch_2_waddr_m_len   = s_awlen[SI_UART][3:0];
    assign G_axi_bus_s_ch_2_waddr_m_prot  = s_awprot[SI_UART];
    assign G_axi_bus_s_ch_2_waddr_m_burst = s_awburst[SI_UART];
    assign s_awready[SI_UART] = G_axi_bus_s_ch_2_waddr_s_ready;

    assign G_axi_bus_s_ch_2_rdat_m_ready = s_rready[SI_UART];
    assign s_rvalid[SI_UART] = G_axi_bus_s_ch_2_rdat_s_valid;
    assign s_rdata[SI_UART]  = {G_axi_bus_s_ch_2_rdat_s_data_data_7, G_axi_bus_s_ch_2_rdat_s_data_data_6,
                                G_axi_bus_s_ch_2_rdat_s_data_data_5, G_axi_bus_s_ch_2_rdat_s_data_data_4,
                                G_axi_bus_s_ch_2_rdat_s_data_data_3, G_axi_bus_s_ch_2_rdat_s_data_data_2,
                                G_axi_bus_s_ch_2_rdat_s_data_data_1, G_axi_bus_s_ch_2_rdat_s_data_data_0};
    assign s_rresp[SI_UART]  = G_axi_bus_s_ch_2_rdat_s_resp;
    assign s_rlast[SI_UART]  = G_axi_bus_s_ch_2_rdat_s_last;

    assign G_axi_bus_s_ch_2_wdat_m_data_data_0 = s_wdata[SI_UART][63:0];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_1 = s_wdata[SI_UART][127:64];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_2 = s_wdata[SI_UART][191:128];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_3 = s_wdata[SI_UART][255:192];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_4 = s_wdata[SI_UART][319:256];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_5 = s_wdata[SI_UART][383:320];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_6 = s_wdata[SI_UART][447:384];
    assign G_axi_bus_s_ch_2_wdat_m_data_data_7 = s_wdata[SI_UART][511:448];
    assign G_axi_bus_s_ch_2_wdat_m_strobe = s_wstrb[SI_UART];
    assign G_axi_bus_s_ch_2_wdat_m_valid  = s_wvalid[SI_UART];
    assign G_axi_bus_s_ch_2_wdat_m_last   = s_wlast[SI_UART];
    assign s_wready[SI_UART] = G_axi_bus_s_ch_2_wdat_s_ready;

    assign G_axi_bus_s_ch_2_wres_m_ready = s_bready[SI_UART];
    assign s_bvalid[SI_UART] = G_axi_bus_s_ch_2_wres_s_valid;
    assign s_bresp[SI_UART]  = G_axi_bus_s_ch_2_wres_s_resp;

    //=========================================================================
    // AXI Crossbar (2 masters x 4 slaves)
    //=========================================================================
    AXICrossbar #(
        .N_MASTERS      (N_MASTERS),
        .M_ADDR_WIDTH   (M_ADDR_WIDTH),
        .M_DATA_WIDTH   (M_DATA_WIDTH),
        .N_SLAVES       (N_SLAVES),
        .S_ADDR_WIDTH   (S_ADDR_WIDTH),
        .S_DATA_WIDTH   (S_DATA_WIDTH),
        // Address mapping: s[0]=MEM, s[1]=CLINT, s[2]=PLIC, s[3]=UART
        .ADDR_BASE      ({UART_BASE, PLIC_BASE, CLINT_BASE, MEM_BASE}),
        .ADDR_MASK      ({UART_MASK, PLIC_MASK, CLINT_MASK, MEM_MASK}),
        .DEFAULT_SLAVE  (SI_MEM)
    ) u_axi_crossbar (
        .clk            (clk),
        .rst_n          (rst_n),
        // Master ports
        .m_awvalid      (m_awvalid),
        .m_awready      (m_awready),
        .m_awaddr       (m_awaddr),
        .m_awlen        (m_awlen),
        .m_awsize       (m_awsize),
        .m_awburst      (m_awburst),
        .m_awprot       (m_awprot),
        .m_wvalid       (m_wvalid),
        .m_wready       (m_wready),
        .m_wdata        (m_wdata),
        .m_wstrb        (m_wstrb),
        .m_wlast        (m_wlast),
        .m_bvalid       (m_bvalid),
        .m_bready       (m_bready),
        .m_bresp        (m_bresp),
        .m_arvalid      (m_arvalid),
        .m_arready      (m_arready),
        .m_araddr       (m_araddr),
        .m_arlen        (m_arlen),
        .m_arsize       (m_arsize),
        .m_arburst      (m_arburst),
        .m_arprot       (m_arprot),
        .m_rvalid       (m_rvalid),
        .m_rready       (m_rready),
        .m_rdata        (m_rdata),
        .m_rresp        (m_rresp),
        .m_rlast        (m_rlast),
        // Slave ports
        .s_awvalid      (s_awvalid),
        .s_awready      (s_awready),
        .s_awaddr       (s_awaddr),
        .s_awlen        (s_awlen),
        .s_awsize       (s_awsize),
        .s_awburst      (s_awburst),
        .s_awprot       (s_awprot),
        .s_wvalid       (s_wvalid),
        .s_wready       (s_wready),
        .s_wdata        (s_wdata),
        .s_wstrb        (s_wstrb),
        .s_wlast        (s_wlast),
        .s_bvalid       (s_bvalid),
        .s_bready       (s_bready),
        .s_bresp        (s_bresp),
        .s_arvalid      (s_arvalid),
        .s_arready      (s_arready),
        .s_araddr       (s_araddr),
        .s_arlen        (s_arlen),
        .s_arsize       (s_arsize),
        .s_arburst      (s_arburst),
        .s_arprot       (s_arprot),
        .s_rvalid       (s_rvalid),
        .s_rready       (s_rready),
        .s_rdata        (s_rdata),
        .s_rresp        (s_rresp),
        .s_rlast        (s_rlast)
    );

    //=========================================================================
    // rv906 core (M1: front end only -- IFU+ICache+BPU+FetchSink, see
    // RVProc.v; M0's TestMaster scaffold is retired, plan Task 4.2/spec S4.3)
    //=========================================================================
    RVProc #(
        .XLEN(64),
        .ILEN(32),
        .RESET_VECTOR(64'h80000000),
        .DATA_WIDTH(512),
        .ADDR_WIDTH(64)
    ) u_core (
        .clk                (clk),
        .rst_n              (rst_n),

        // ICache AXI (m[0])
        .axi_i_awvalid      (axi_i_awvalid),
        .axi_i_awready      (axi_i_awready),
        .axi_i_awaddr       (axi_i_awaddr),
        .axi_i_awlen        (axi_i_awlen),
        .axi_i_awsize       (axi_i_awsize),
        .axi_i_awburst      (axi_i_awburst),
        .axi_i_awcache      (axi_i_awcache),
        .axi_i_awprot       (axi_i_awprot),
        .axi_i_wvalid       (axi_i_wvalid),
        .axi_i_wready       (axi_i_wready),
        .axi_i_wdata        (axi_i_wdata),
        .axi_i_wstrb        (axi_i_wstrb),
        .axi_i_wlast        (axi_i_wlast),
        .axi_i_bvalid       (axi_i_bvalid),
        .axi_i_bready       (axi_i_bready),
        .axi_i_bresp        (axi_i_bresp),
        .axi_i_arvalid      (axi_i_arvalid),
        .axi_i_arready      (axi_i_arready),
        .axi_i_araddr       (axi_i_araddr),
        .axi_i_arlen        (axi_i_arlen),
        .axi_i_arsize       (axi_i_arsize),
        .axi_i_arburst      (axi_i_arburst),
        .axi_i_arcache      (axi_i_arcache),
        .axi_i_arprot       (axi_i_arprot),
        .axi_i_rvalid       (axi_i_rvalid),
        .axi_i_rready       (axi_i_rready),
        .axi_i_rdata        (axi_i_rdata),
        .axi_i_rresp        (axi_i_rresp),
        .axi_i_rlast        (axi_i_rlast),

        // DCache AXI (m[1])
        .axi_d_awvalid      (axi_d_awvalid),
        .axi_d_awready      (axi_d_awready),
        .axi_d_awaddr       (axi_d_awaddr),
        .axi_d_awlen        (axi_d_awlen),
        .axi_d_awsize       (axi_d_awsize),
        .axi_d_awburst      (axi_d_awburst),
        .axi_d_awcache      (axi_d_awcache),
        .axi_d_awprot       (axi_d_awprot),
        .axi_d_wvalid       (axi_d_wvalid),
        .axi_d_wready       (axi_d_wready),
        .axi_d_wdata        (axi_d_wdata),
        .axi_d_wstrb        (axi_d_wstrb),
        .axi_d_wlast        (axi_d_wlast),
        .axi_d_bvalid       (axi_d_bvalid),
        .axi_d_bready       (axi_d_bready),
        .axi_d_bresp        (axi_d_bresp),
        .axi_d_arvalid      (axi_d_arvalid),
        .axi_d_arready      (axi_d_arready),
        .axi_d_araddr       (axi_d_araddr),
        .axi_d_arlen        (axi_d_arlen),
        .axi_d_arsize       (axi_d_arsize),
        .axi_d_arburst      (axi_d_arburst),
        .axi_d_arcache      (axi_d_arcache),
        .axi_d_arprot       (axi_d_arprot),
        .axi_d_rvalid       (axi_d_rvalid),
        .axi_d_rready       (axi_d_rready),
        .axi_d_rdata        (axi_d_rdata),
        .axi_d_rresp        (axi_d_rresp),
        .axi_d_rlast        (axi_d_rlast),

        // Interrupt inputs (from CLINT and PLIC)
        .mtip               (clint_mtip),
        .msip               (clint_msip),
        .meip               (plic_meip),
        .mtime              (clint_mtime),

        // M7 Task 5: DM <-> DTU core interface (from the TDT_DM instance)
        .tdt_dm_dtu_halt_req      (dm_core_halt_req),
        .tdt_dm_dtu_resume_req    (dm_core_resume_req),
        .tdt_dm_dtu_halt_on_reset (dm_core_halt_on_reset),
        .tdt_dm_dtu_ack_havereset (dm_core_ack_havereset),
        .tdt_dm_dtu_itr           (dm_core_itr),
        .tdt_dm_dtu_itr_vld       (dm_core_itr_vld),
        .tdt_dm_dtu_wr_vld        (dm_core_wr_vld),
        .tdt_dm_dtu_wr_flg        (dm_core_wr_flg),
        .tdt_dm_dtu_wdata         (dm_core_wdata),
        .dtu_tdt_dm_halted        (core_dm_halted),
        .dtu_tdt_dm_havereset     (core_dm_havereset),
        .dtu_tdt_dm_itr_done      (core_dm_itr_done),
        .dtu_tdt_dm_retire_debug_expt_vld (core_dm_retire_debug_expt),
        .dtu_tdt_dm_wr_ready      (core_dm_wr_ready),
        .dtu_tdt_dm_rx_data       (core_dm_rx_data),

        .quitted            (quitted)
    );

    //=========================================================================
    // Memory Controller - C2RTL Generated
    //=========================================================================
    MEMCTL_AXI4L_step u_memctl (
        .clk                (clk),
        .rst_n              (rst_n),

        // AXI Slave Interface (from crossbar s[0])
        .G_axi_raddr_m_addr (mem_raddr_m_addr),
        .G_axi_raddr_m_size (mem_raddr_m_size),
        .G_axi_raddr_m_valid(mem_raddr_m_valid),
        .G_axi_raddr_m_len  (mem_raddr_m_len),
        .G_axi_raddr_s_ready(mem_raddr_s_ready),

        .G_axi_waddr_m_addr (mem_waddr_m_addr),
        .G_axi_waddr_m_size (mem_waddr_m_size),
        .G_axi_waddr_m_valid(mem_waddr_m_valid),
        .G_axi_waddr_m_len  (mem_waddr_m_len),
        .G_axi_waddr_s_ready(mem_waddr_s_ready),

        .G_axi_rdat_m_ready      (mem_rdat_m_ready),
        .G_axi_rdat_s_data_data_0(mem_rdat_s_data_0),
        .G_axi_rdat_s_data_data_1(mem_rdat_s_data_1),
        .G_axi_rdat_s_data_data_2(mem_rdat_s_data_2),
        .G_axi_rdat_s_data_data_3(mem_rdat_s_data_3),
        .G_axi_rdat_s_data_data_4(mem_rdat_s_data_4),
        .G_axi_rdat_s_data_data_5(mem_rdat_s_data_5),
        .G_axi_rdat_s_data_data_6(mem_rdat_s_data_6),
        .G_axi_rdat_s_data_data_7(mem_rdat_s_data_7),
        .G_axi_rdat_s_resp       (mem_rdat_s_resp_1b),
        .G_axi_rdat_s_valid      (mem_rdat_s_valid),
        .G_axi_rdat_s_last       (mem_rdat_s_last),

        .G_axi_wdat_m_data_data_0(mem_wdat_m_data_0),
        .G_axi_wdat_m_data_data_1(mem_wdat_m_data_1),
        .G_axi_wdat_m_data_data_2(mem_wdat_m_data_2),
        .G_axi_wdat_m_data_data_3(mem_wdat_m_data_3),
        .G_axi_wdat_m_data_data_4(mem_wdat_m_data_4),
        .G_axi_wdat_m_data_data_5(mem_wdat_m_data_5),
        .G_axi_wdat_m_data_data_6(mem_wdat_m_data_6),
        .G_axi_wdat_m_data_data_7(mem_wdat_m_data_7),
        .G_axi_wdat_m_valid      (mem_wdat_m_valid),
        .G_axi_wdat_s_ready      (mem_wdat_s_ready),

        .G_axi_wres_m_ready      (mem_wres_m_ready),
        .G_axi_wres_s_resp       (mem_wres_s_resp_1b),
        .G_axi_wres_s_valid      (mem_wres_s_valid),

        .G_axi_intr              (),

        // Memory Pin Interface (to external memory model)
        .G_mem_pin_addr     (mem_pin_addr),
        .G_mem_pin_din_data_0(mem_pin_din_0),
        .G_mem_pin_din_data_1(mem_pin_din_1),
        .G_mem_pin_din_data_2(mem_pin_din_2),
        .G_mem_pin_din_data_3(mem_pin_din_3),
        .G_mem_pin_din_data_4(mem_pin_din_4),
        .G_mem_pin_din_data_5(mem_pin_din_5),
        .G_mem_pin_din_data_6(mem_pin_din_6),
        .G_mem_pin_din_data_7(mem_pin_din_7),
        .G_mem_pin_size     (mem_pin_size),
        .G_mem_pin_cs       (mem_pin_cs),
        .G_mem_pin_we       (mem_pin_we),
        .G_mem_pin_ras      (mem_pin_ras),
        .G_mem_pin_cas      (mem_pin_cas),
        .G_mem_pin_dout_data_0(G_io_pins_mpin_dout_data_0),
        .G_mem_pin_dout_data_1(G_io_pins_mpin_dout_data_1),
        .G_mem_pin_dout_data_2(G_io_pins_mpin_dout_data_2),
        .G_mem_pin_dout_data_3(G_io_pins_mpin_dout_data_3),
        .G_mem_pin_dout_data_4(G_io_pins_mpin_dout_data_4),
        .G_mem_pin_dout_data_5(G_io_pins_mpin_dout_data_5),
        .G_mem_pin_dout_data_6(G_io_pins_mpin_dout_data_6),
        .G_mem_pin_dout_data_7(G_io_pins_mpin_dout_data_7)
    );

    //=========================================================================
    // Memory Pin Output Mapping
    //=========================================================================
    assign G_io_pins_mpin_addr = mem_pin_addr;
    assign G_io_pins_mpin_cs   = mem_pin_cs;
    assign G_io_pins_mpin_we   = mem_pin_we;
    assign G_io_pins_mpin_ras  = mem_pin_ras;
    assign G_io_pins_mpin_cas  = mem_pin_cas;
    assign G_io_pins_mpin_size = mem_pin_size;
    assign G_io_pins_mpin_din_data_0 = mem_pin_din_0;
    assign G_io_pins_mpin_din_data_1 = mem_pin_din_1;
    assign G_io_pins_mpin_din_data_2 = mem_pin_din_2;
    assign G_io_pins_mpin_din_data_3 = mem_pin_din_3;
    assign G_io_pins_mpin_din_data_4 = mem_pin_din_4;
    assign G_io_pins_mpin_din_data_5 = mem_pin_din_5;
    assign G_io_pins_mpin_din_data_6 = mem_pin_din_6;
    assign G_io_pins_mpin_din_data_7 = mem_pin_din_7;

    // Return value (quitted signal)
    assign G_RVProcAXI_OUT = quitted;

endmodule
