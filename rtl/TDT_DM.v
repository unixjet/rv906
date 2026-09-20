//=============================================================================
// TDT_DM.v - Debug Module + SBA clone  (M7 Task 3)
//=============================================================================
// Clone of the donor C906 Debug Module:
//   gen_rtl/tdt/rtl/debug/tdt_dm.v       (APB slave, DM registers, abstract
//                                          command engine, progbuf engine,
//                                          SBA register file)
//   gen_rtl/tdt/rtl/debug/tdt_sba_axi.v  (single-beat 128-bit AXI4 master)
//
// rv906 deviations from the donor (docs/superpowers/specs/2026-09-11-m7-
// debug-design.md, D-M7-1..D-M7-10):
//   D-M7-1  Single clock domain: pclk = dm_pclk = forever_cpuclk = clk.
//           The donor's tdt_dm_pulse_sync CDC pairs degenerate to direct
//           1-cycle handshakes (the SoC runs everything on clk).
//   D-M7-5  cuscs/cuscmd/cusbuf0-7 @0x70-0x79 read 0 / writes ignored.
//   D-M7-6  The CUSCMD-0 async-halt output (dm_core_async_halt_req_o) and
//           the wr_flg 10/11 rx sources (latest_pc/satp) are dropped; the
//           rx_data mux is 00=dscratch0 (via core_dm_rx_data_i), else 0.
//   D-M7-7  No custom async halt: no dm_core_async_halt_req_o port.
//   D-M7-9  DM not clock-gated: the shared clk cannot be gated by dmactive.
//           Instead dmactive=0 holds the DM in reset via the donor's own
//           sync_rst generator (tdt_dm.v:817-829); all core-side outputs
//           idle when dmactive=0 (this IS the OFF-path identity for the
//           SoC).
//   D-M7-10 Kept constants: datacount=2 (RV64), progbufsize=4 + implied
//           ebreak at index 4, sbasize=40, SBA_DW=128, sbaccess 2/3/4,
//           NEXTDM=0, version=2 (spec 0.13), sbversion=1, compid vendor
//           12'b1011_011_0111_1 / comptype 0 / compversion 1.
//
// C906 config (donor cpu_cfig.h, cited per-item): CORE_NUM=1
// (tdt_define.h:16-99 TDT_DM_SINGLE_CORE), PB_SIZE=4 (cpu_cfig.h:228-236),
// IMP_EBREAK=1 (tdt_define.h:103), RV64 (CORE_MAX_XLEN=64, datacount=2,
// tdt_dm.v:2055-2059), SBAW=40 (cpu_cfig.h:463), NSCRATCH=2
// (cpu_cfig.h:455), SBA_DW=128 (tdt_define.h:142-153), NEXTDM=0
// (tdt_dm_top.v TDT_NEXT_DM_BA=0).
//
// APB byte address = spec word offset << 2 (the DMI bridge does the shift;
// the DM decodes dm_paddr[11:2], donor tdt_dm.v:171-227, read mux
// :3413-3470).
//=============================================================================

module TDT_DM (
    input           clk,
    input           tdt_rst_n,          // async reset, mirrors donor ciu_rst_b (D-M7-3)

    // APB slave (donor tdt_dm.v:34-41)
    input  [11:0]   dm_paddr,
    input           dm_pwrite,
    input           dm_psel,
    input           dm_penable,
    input  [31:0]   dm_pwdata,
    output reg [31:0] dm_prdata,
    output reg      dm_pready,
    output          dm_pslverr,

    // Core interface (donor tdt_dm.v:47-104, WIRING MAP note 2 section 6)
    output reg      dm_core_halt_req_o,
    output reg      dm_core_resume_req_o,
    output reg      dm_core_halt_on_reset_o,
    output reg      dm_core_ack_havereset_o,
    output [31:0]   dm_core_itr_o,
    output          dm_core_itr_vld_o,
    output reg      dm_core_wr_vld_o,
    output reg [1:0] dm_core_wr_flg_o,
    output [63:0]   dm_core_wdata_o,
    output reg      dm_core_rstn_o,        // hartreset_n pad (dangling in SoC)
    output reg      dm_core_ndmreset_n_o,  // ndmreset_n pad (dangling in SoC)
    input           core_dm_halted_i,
    input           core_dm_havereset_i,
    input           core_dm_itr_done_i,
    input           core_dm_retire_debug_expt_i,
    input           core_dm_wr_ready_i,
    input  [63:0]   core_dm_rx_data_i,

    // SBA AXI4 master (donor tdt_sba_axi.v:36-67 + tdt_dm.v:3266-3273)
    output [3:0]    dm_pad_awid,
    output reg [39:0] dm_pad_awaddr,
    output [3:0]    dm_pad_awlen,
    output reg [2:0] dm_pad_awsize,
    output reg      dm_pad_awvalid,
    input           pad_dm_awready,
    output reg [127:0] dm_pad_wdata,
    output reg      dm_pad_wvalid,
    output          dm_pad_wlast,
    output reg [15:0] dm_pad_wstrb,
    input           pad_dm_wready,
    output          dm_pad_bready,
    input  [3:0]    pad_dm_bid,
    input  [1:0]    pad_dm_bresp,
    input           pad_dm_bvalid,
    output [3:0]    dm_pad_arid,
    output reg [39:0] dm_pad_araddr,
    output [3:0]    dm_pad_arlen,
    output reg [2:0] dm_pad_arsize,
    output reg      dm_pad_arvalid,
    input           pad_dm_arready,
    input  [3:0]    pad_dm_rid,
    input  [127:0]  pad_dm_rdata,
    input           pad_dm_rvalid,
    input           pad_dm_rlast,
    input  [1:0]    pad_dm_rresp,
    output          dm_pad_rready,
    output [1:0]    dm_pad_awburst,
    output [3:0]    dm_pad_awcache,
    output          dm_pad_awlock,
    output [2:0]    dm_pad_awprot,
    output [1:0]    dm_pad_arburst,
    output [3:0]    dm_pad_arcache,
    output          dm_pad_arlock,
    output [2:0]    dm_pad_arprot
);

//==========================================================
//    local parameters (donor tdt_dm.v:171-241)
//==========================================================
localparam [9:0] OFFSET_DATA0      = 10'h04;   // tdt_dm.v:171
localparam [9:0] OFFSET_DATA1      = 10'h05;   // :172
localparam [9:0] OFFSET_DMCONTROL  = 10'h10;   // :183
localparam [9:0] OFFSET_DMSTATUS   = 10'h11;   // :184
localparam [9:0] OFFSET_HARTINFO   = 10'h12;   // :185
localparam [9:0] OFFSET_HAWINDOW   = 10'h15;   // :186
localparam [9:0] OFFSET_ABSTRACTCS = 10'h16;   // :187
localparam [9:0] OFFSET_COMMAND    = 10'h17;   // :188
localparam [9:0] OFFSET_ABSTRACTAUTO = 10'h18; // :189
localparam [9:0] OFFSET_NEXTDM     = 10'h1d;   // :190
localparam [9:0] OFFSET_PB0        = 10'h20;   // :191
localparam [9:0] OFFSET_PB1        = 10'h21;
localparam [9:0] OFFSET_PB2        = 10'h22;
localparam [9:0] OFFSET_PB3        = 10'h23;   // :194
localparam [9:0] OFFSET_DMCS2      = 10'h32;   // :207
localparam [9:0] OFFSET_SBCS       = 10'h38;   // :208
localparam [9:0] OFFSET_SBADDR0    = 10'h39;   // :209
localparam [9:0] OFFSET_SBADDR1    = 10'h3a;   // :210
localparam [9:0] OFFSET_SBDATA0    = 10'h3c;   // :211
localparam [9:0] OFFSET_SBDATA1    = 10'h3d;
localparam [9:0] OFFSET_SBDATA2    = 10'h3e;
localparam [9:0] OFFSET_SBDATA3    = 10'h3f;
localparam [9:0] OFFSET_HARTSUM0   = 10'h40;   // :215
localparam [9:0] OFFSET_ITR        = 10'h1f;   // :216
localparam [9:0] OFFSET_CUSCS      = 10'h70;   // :217 (D-M7-5: reads 0)
localparam [9:0] OFFSET_CUSCMD     = 10'h71;   // :218 (D-M7-5: writes ignored)
localparam [9:0] OFFSET_COMPID     = 10'h7f;   // :227

localparam [11:0] JEP106_ID   = 12'b1011_011_0111_1; // :229
localparam [3:0]  DM_VERSION  = 4'h2;                // :230
localparam [11:0] COMP_TYPE   = 12'h0;               // :231
localparam [7:0]  COMP_VER    = 8'h1;                // :232
localparam [11:0] DSCR0_ADDR  = 12'h7b2;             // :233
localparam [11:0] DSCR1_ADDR  = 12'h7b3;             // :234
localparam [6:0]  SYS_OPCODE  = 7'h73;               // :236
localparam [2:0]  CSRRW_F3    = 3'h1;                // :237
localparam [2:0]  CSRRC_F3    = 3'h3;                // :238
localparam [4:0]  X0_GPR      = 5'h0;                // :239
localparam [31:0] EBREAK_INST  = 32'h00100073;        // :240
localparam [31:0] CEBREAK_INST = 32'h00009002;        // :241

// REGACC FSM states (donor tdt_dm.v:243-253)
localparam [3:0] REGACC_IDLE      = 4'h0;
localparam [3:0] REGACC_WDSC0     = 4'h1;
localparam [3:0] REGACC_RDSC0     = 4'h2;
localparam [3:0] REGACC_X6_2_DSC1 = 4'h3;
localparam [3:0] REGACC_DSC1_2_X6 = 4'h4;
localparam [3:0] REGACC_DSC0_2_G  = 4'h5;
localparam [3:0] REGACC_G_2_DSC0  = 4'h6;
localparam [3:0] REGACC_C_2_X6    = 4'h7;
localparam [3:0] REGACC_X6_2_C    = 4'h8;
localparam [3:0] REGACC_X6_2_DSC0 = 4'h9;
localparam [3:0] REGACC_DSC0_2_X6 = 4'ha;

localparam PB_SIZE = 4;   // cpu_cfig.h:235
localparam SBAW    = 40;  // cpu_cfig.h:463

//==========================================================
//    wires and registers
//==========================================================
wire        pb_sel, intra_sel, apb_access;
wire        dm_intra_psel, dm_intra_access, dm_intra_apbw, dm_intra_apbr;

reg  [31:0] progbuf [0:15];

reg         resumereq, hartreset, ackhavereset, hasel;
reg         setresethaltreq, clrresethaltreq, ndmreset;
wire [31:0] dmcontrol;
reg  [31:0] hawindow;
reg         dmactive, dmactive_d1;
wire        sync_rst;

reg         core_dm_halted_d;
wire        core_dm_resume_req, core_dm_running;

reg         resumeack;
wire        allhavereset, anyhavereset, allresumeack, anyresumeack;
wire        allrunning, anyrunning, allhalted, anyhalted;
wire [31:0] dmstatus;

reg  [7:0]  cmdtype;
reg         aarpostincrement, aarpostexec, transfer, write;
reg  [15:0] regno;
reg  [11:0] autoexecdata;
reg  [15:0] autoexecprogbuf;
wire [31:0] abstractauto;
wire        cmd_busy_set, haltreq_raise, haltreq_fall, aarsize_err;
reg         cmd_start, cmd_work, cmd_done, pb_work;
wire        busy;
reg  [2:0]  cmderr;
wire [31:0] abstractcs;
wire        access_gpr, access_csr, access_dsc0, access_illegal_reg;
reg  [31:0] itr;
reg         itr_vld, itr_work;
reg  [63:0] core_rdata_r;
reg         core_rdata_vld_r;
reg  [31:0] data0, data1;
wire [31:0] hartsum0;
reg  [31:0] hartinfo;
wire [31:0] compid;
reg         ebk_work;
wire [31:0] pb_mux;
reg  [3:0]  pb_idx;
reg  [3:0]  regacc_cur_state, regacc_nxt_state;
wire        fsm_x6_2_dsc1, fsm_x6_2_dsc0, fsm_x6_2_c, fsm_c_2_x6;
wire        fsm_dsc0_2_x6, fsm_dsc1_2_x6;
reg         itr_send_pb_r;
wire        itr_send_pb, itr_vld_en, dm_core_wr_vld_en;
wire        apbw_dmctrl, apbw_abscmd, apbw_abscmdauto;
wire        legal_cmd_sel, apbw_accreg_cfg, pb_work_start, dm_paddr_is_itr;
wire        apb_access_pb0, apb_access_pb1, apb_access_pb2, apb_access_pb3;
wire        apb_access_data0, apb_access_data1;
wire [15:0] apb_access_pb, apb_access_pb_real;
wire [11:0] apb_access_data_real;
wire        autoaccen;

wire [31:0] dmcs2;

reg  [4:0]  sbaddrplus;
wire [32:0] sbaddr0_plus_pre;
reg  [31:0] sbaddr0, sbaddr1;
wire [2:0]  sbversion;
reg         sbbusy, sbbusyerror;
reg         sberror_will_be_4, sberror_will_be_3;
reg  [2:0]  sberror;
wire [6:0]  sbasize;
wire [4:0]  sbaccess_info;
reg         sbreadonaddr;
reg  [2:0]  sbaccess;
reg         sbautoincrement, sbreadondata;
wire [31:0] sbcs;
reg  [31:0] sbdata0, sbdata1, sbdata2, sbdata3;
wire [127:0] sba_w_data;
reg         sba_wr_flg, sba_wr_vld;
wire [SBAW-1:0] sba_wr_addr;
wire [63:0] sba_wr_addr_pre;
wire [2:0]  sba_wr_size;
reg  [127:0] sba_rd_data;
reg         sba_wr_ready;
wire        sba_write, sba_read;
wire        sba_write_ignore_unalign, sba_read_ignore_unalign;
reg         sba_write_ignore_unalign_f, sba_read_ignore_unalign_f;
reg         sba_error;
wire        sb_noerr, dm_paddr_is_sbaddr0, sbbusyerr_set, sbaccess_unalian;
wire        apbw_sbcs;

reg  [127:0] s_wr_data;
reg          s_wr_flg;
reg  [SBAW-1:0] s_wr_addr;
reg          s_wr_vld;
reg  [2:0]   s_wr_size;
reg          axi_wr_ready_pre;
reg  [127:0] rd_data_pre;
wire [3:0]   addr_alian;
wire [15:0]  wstrb_pre;
reg  [15:0]  wstrb_pre1;
reg  [127:0] wdata_pre;
reg  [127:0] rdata_smp;
reg          sba_error_pre;

//==========================================================
//    APB mini decode (donor tdt_dm.v:598-609)
//==========================================================
assign pb_sel          = dm_paddr[11:6] == {4'b0, 2'b10};
assign intra_sel       = !pb_sel;
assign apb_access      = dm_psel & dm_penable;
assign dm_intra_psel   = dm_psel && intra_sel;
assign dm_intra_access = dm_intra_psel && dm_penable;
assign dm_intra_apbw   = dm_intra_access && dm_pwrite;
assign dm_intra_apbr   = dm_intra_access && !dm_pwrite;

//==========================================================
//    progbuf (donor :614-657, PB_SIZE=4 + implied ebreak)
//==========================================================
genvar i;
generate
    for (i=0; i<PB_SIZE; i=i+1) begin : gen_progbuf
        always @ (posedge clk or negedge tdt_rst_n) begin
            if (!tdt_rst_n)
                progbuf[i] <= 32'h0;
            else if (sync_rst)
                progbuf[i] <= 32'h0;
            else if (dm_psel && dm_penable && pb_sel && dm_pwrite
                     && dm_paddr[5:2] == i[3:0] && ~busy)
                progbuf[i] <= dm_pwdata[31:0];
        end
    end
endgenerate

always @ (*) begin
    progbuf[PB_SIZE] = EBREAK_INST;
end

generate
    for (i=PB_SIZE+1; i<16; i=i+1) begin : gen_zero_progbuf
        always @ (*) begin
            progbuf[i] = 32'h0;
        end
    end
endgenerate

//==========================================================
//    dmcontrol (donor :660-836)
//==========================================================
assign apbw_dmctrl = dm_intra_apbw && dm_paddr[11:2] == OFFSET_DMCONTROL;
assign haltreq_raise = apbw_dmctrl && dm_pwdata[31];
assign haltreq_fall  = apbw_dmctrl && ~dm_pwdata[31];

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n)
        core_dm_halted_d <= 1'b0;
    else if (sync_rst)
        core_dm_halted_d <= 1'b0;
    else
        core_dm_halted_d <= core_dm_halted_i;
end

assign core_dm_resume_req = core_dm_halted_d && ~core_dm_halted_i;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) resumereq <= 1'b0;
    else if (sync_rst) resumereq <= 1'b0;
    else if (apbw_dmctrl) resumereq <= !dm_pwdata[31] && dm_pwdata[30];
    else if (resumereq) resumereq <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) hartreset <= 1'b0;
    else if (sync_rst) hartreset <= 1'b0;
    else if (apbw_dmctrl) hartreset <= dm_pwdata[29];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) ackhavereset <= 1'b0;
    else if (sync_rst) ackhavereset <= 1'b0;
    else if (apbw_dmctrl) ackhavereset <= dm_pwdata[28];
    else if (ackhavereset) ackhavereset <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) hasel <= 1'b0;
    else if (sync_rst) hasel <= 1'b0;
    else if (apbw_dmctrl) hasel <= dm_pwdata[26];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) setresethaltreq <= 1'b0;
    else if (sync_rst) setresethaltreq <= 1'b0;
    else if (apbw_dmctrl) setresethaltreq <= dm_pwdata[3];
    else if (setresethaltreq) setresethaltreq <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) clrresethaltreq <= 1'b0;
    else if (sync_rst) clrresethaltreq <= 1'b0;
    else if (apbw_dmctrl) clrresethaltreq <= dm_pwdata[2];
    else if (clrresethaltreq) clrresethaltreq <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_halt_on_reset_o <= 1'b0;
    else if (sync_rst) dm_core_halt_on_reset_o <= 1'b0;
    else if (clrresethaltreq) dm_core_halt_on_reset_o <= 1'b0;
    else if (setresethaltreq) dm_core_halt_on_reset_o <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) ndmreset <= 1'b0;
    else if (sync_rst) ndmreset <= 1'b0;
    else if (apbw_dmctrl) ndmreset <= dm_pwdata[1];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dmactive <= 1'b0;
    else if (apbw_dmctrl) dmactive <= dm_pwdata[0];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dmactive_d1 <= 1'b0;
    else dmactive_d1 <= dmactive;
end

assign sync_rst = dmactive_d1 & ~dmactive;

assign dmcontrol[31:0] = {2'b0, hartreset, ackhavereset, 1'b0,
                    hasel, 10'b0, 10'b0, 2'b0,
                    setresethaltreq, clrresethaltreq, ndmreset, dmactive};

//==========================================================
//    hawindow (donor :841-848)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) hawindow[31:0] <= 32'b0;
    else if (sync_rst) hawindow[31:0] <= 32'b0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_HAWINDOW)
        hawindow[31:0] <= dm_pwdata[31:0];
end

//==========================================================
//    dmstatus (donor :890-969, single-core collapsed)
//==========================================================
assign allhavereset   = core_dm_havereset_i;
assign anyhavereset   = core_dm_havereset_i;
assign allresumeack   = resumeack;
assign anyresumeack   = resumeack;
assign core_dm_running = ~core_dm_halted_i;
assign allrunning      = ~core_dm_halted_i;
assign anyrunning      = ~core_dm_halted_i;
assign allhalted       = core_dm_halted_i;
assign anyhalted       = core_dm_halted_i;

assign dmstatus[31:0] = {9'h0, 1'b1 /*impebreak*/, 2'b0, allhavereset, anyhavereset,
                   allresumeack, anyresumeack, 1'b0 /*allnonexist*/, 1'b0 /*anynonexist*/,
                   1'b0 /*allunavail*/, 1'b0 /*anyunavail*/, allrunning, anyrunning,
                   allhalted, anyhalted, 1'b1 /*authenticated*/, 1'b0 /*authbusy*/,
                   1'b1 /*haresethaltreq*/, 1'b0 /*confstrptrvalid*/, DM_VERSION};

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) resumeack <= 1'b0;
    else if (sync_rst) resumeack <= 1'b0;
    else if (resumereq) resumeack <= 1'b0;
    else if (core_dm_resume_req) resumeack <= 1'b1;
end

assign dmcs2[31:0] = 32'h0;

//==========================================================
//    reset outputs (donor :1226-1288)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_rstn_o <= 1'b1;
    else if (sync_rst) dm_core_rstn_o <= 1'b1;
    else if (apbw_dmctrl) dm_core_rstn_o <= !dm_pwdata[29];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_ack_havereset_o <= 1'b0;
    else if (sync_rst) dm_core_ack_havereset_o <= 1'b0;
    else if (dm_core_ack_havereset_o) dm_core_ack_havereset_o <= 1'b0;
    else if (ackhavereset) dm_core_ack_havereset_o <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_ndmreset_n_o <= 1'b1;
    else if (sync_rst) dm_core_ndmreset_n_o <= 1'b1;
    else if (apbw_dmctrl) dm_core_ndmreset_n_o <= !dm_pwdata[1];
end

//==========================================================
//    halt and resume (donor :1486-1562, single-core)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_halt_req_o <= 1'b0;
    else if (sync_rst) dm_core_halt_req_o <= 1'b0;
    else if (core_dm_halted_i | haltreq_fall)
        dm_core_halt_req_o <= 1'b0;
    else if (haltreq_raise)
        dm_core_halt_req_o <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_resume_req_o <= 1'b0;
    else if (sync_rst) dm_core_resume_req_o <= 1'b0;
    else if (dm_core_resume_req_o) dm_core_resume_req_o <= 1'b0;
    else if (resumereq && hartsum0[0])
        dm_core_resume_req_o <= 1'b1;
end

//==========================================================
//    ABS CMD (donor :1688-2061)
//==========================================================
assign apbw_abscmd     = dm_intra_apbw && dm_paddr[11:2] == OFFSET_COMMAND;
assign apbw_abscmdauto = dm_intra_apbw && dm_paddr[11:2] == OFFSET_ABSTRACTAUTO;
assign apbw_accreg_cfg = apbw_abscmd && dm_pwdata[31:24] == 8'h0 && ~busy && cmderr[2:0] == 3'h0;
assign legal_cmd_sel   = ~busy && cmderr[2:0] == 3'h0 && hartsum0[0];

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) cmdtype[7:0] <= 8'b0;
    else if (sync_rst) cmdtype[7:0] <= 8'b0;
    else if (apbw_abscmd && ~busy && cmderr[2:0] == 3'h0)
        cmdtype[7:0] <= dm_pwdata[31:24];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) aarpostincrement <= 1'b0;
    else if (sync_rst) aarpostincrement <= 1'b0;
    else if (apbw_accreg_cfg) aarpostincrement <= dm_pwdata[19];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) aarpostexec <= 1'b0;
    else if (sync_rst) aarpostexec <= 1'b0;
    else if (apbw_accreg_cfg) aarpostexec <= dm_pwdata[18];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) transfer <= 1'b0;
    else if (sync_rst) transfer <= 1'b0;
    else if (apbw_accreg_cfg) transfer <= dm_pwdata[17];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) write <= 1'b0;
    else if (sync_rst) write <= 1'b0;
    else if (apbw_accreg_cfg) write <= dm_pwdata[16];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_wr_flg_o[1:0] <= 2'b0;
    else if (sync_rst) dm_core_wr_flg_o[1:0] <= 2'b0;
    else if (apbw_accreg_cfg) dm_core_wr_flg_o[1:0] <= {1'b0, dm_pwdata[16]};
end

assign dm_core_wdata_o[63:0] = {data1[31:0], data0[31:0]};

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) regno[15:0] <= 16'b0;
    else if (sync_rst) regno[15:0] <= 16'b0;
    else if (apbw_accreg_cfg) regno[15:0] <= dm_pwdata[15:0];
    else if (aarpostincrement && cmd_done) regno[15:0] <= regno[15:0] + 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) autoexecdata[11:0] <= 12'b0;
    else if (sync_rst) autoexecdata[11:0] <= 12'b0;
    else if (apbw_abscmdauto && ~busy) autoexecdata[11:0] <= dm_pwdata[11:0];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) autoexecprogbuf[15:0] <= 16'b0;
    else if (sync_rst) autoexecprogbuf[15:0] <= 16'b0;
    else if (apbw_abscmdauto && ~busy) autoexecprogbuf[15:0] <= dm_pwdata[31:16];
end

assign abstractauto[31:0] = {autoexecprogbuf[15:0], 4'b0, autoexecdata[11:0]};

assign apb_access_pb0   = apb_access && dm_paddr[11:2] == OFFSET_PB0;
assign apb_access_pb1   = apb_access && dm_paddr[11:2] == OFFSET_PB1;
assign apb_access_pb2   = apb_access && dm_paddr[11:2] == OFFSET_PB2;
assign apb_access_pb3   = apb_access && dm_paddr[11:2] == OFFSET_PB3;
assign apb_access_data0 = apb_access && dm_paddr[11:2] == OFFSET_DATA0;
assign apb_access_data1 = apb_access && dm_paddr[11:2] == OFFSET_DATA1;

assign apb_access_pb[15:0] = {12'b0, apb_access_pb3, apb_access_pb2,
                               apb_access_pb1, apb_access_pb0};
assign apb_access_pb_real[15:0] = {12'b0, apb_access_pb[3:0]};
assign apb_access_data_real[11:0] = {10'h0, apb_access_data1, apb_access_data0};

assign cmd_busy_set = (|apb_access_pb_real) | (|apb_access_data_real) |
                      ((dm_psel & dm_penable & dm_pwrite) &
                        (dm_paddr[11:2] == OFFSET_ABSTRACTCS ||
                         dm_paddr[11:2] == OFFSET_COMMAND ||
                         dm_paddr[11:2] == OFFSET_ITR ||
                         dm_paddr[11:2] == OFFSET_ABSTRACTAUTO));

assign autoaccen = (|(apb_access_data_real & autoexecdata)) ||
                   (|(apb_access_pb_real & autoexecprogbuf));

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) cmd_start <= 1'b0;
    else if (sync_rst) cmd_start <= 1'b0;
    else if (cmd_start) cmd_start <= 1'b0;
    else if ((transfer && autoaccen && cmdtype == 8'h0 && regno < 16'h1020) ||
             (apbw_abscmd && dm_pwdata[31:24] == 8'h0 && dm_pwdata[15:0] < 16'h1020 &&
              dm_pwdata[17] && ~aarsize_err))
        cmd_start <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) cmd_work <= 1'b0;
    else if (sync_rst) cmd_work <= 1'b0;
    else if (cmd_done) cmd_work <= 1'b0;
    else if (cmd_start && transfer && legal_cmd_sel) cmd_work <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) cmd_done <= 1'b0;
    else if (sync_rst) cmd_done <= 1'b0;
    else if (cmd_done) cmd_done <= 1'b0;
    else if (regacc_nxt_state == REGACC_IDLE && regacc_cur_state != REGACC_IDLE)
        cmd_done <= 1'b1;
end

assign pb_mux[31:0] = progbuf[pb_idx];

//==========================================================
//    progbuf engine (donor :1952-1988)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) pb_idx[3:0] <= 4'h0;
    else if (sync_rst) pb_idx[3:0] <= 4'h0;
    else if (core_dm_itr_done_i && core_dm_retire_debug_expt_i)
        pb_idx[3:0] <= 4'h0;
    else if (ebk_work && core_dm_itr_done_i)
        pb_idx[3:0] <= 4'h0;
    else if (pb_work && core_dm_itr_done_i)
        pb_idx[3:0] <= pb_idx[3:0] + 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) ebk_work <= 1'b0;
    else if (sync_rst) ebk_work <= 1'b0;
    else if (ebk_work && core_dm_itr_done_i)
        ebk_work <= 1'b0;
    else if (pb_work && itr_vld && (pb_mux == EBREAK_INST || pb_mux == CEBREAK_INST))
        ebk_work <= 1'b1;
end

assign pb_work_start = (apbw_abscmd && dm_pwdata[31:24] == 8'h0 && dm_pwdata[18] &&
                        !dm_pwdata[17] && ~busy) ||
                       (transfer && cmd_done && aarpostexec) ||
                       (autoaccen && aarpostexec && !transfer && ~busy);

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) pb_work <= 1'b0;
    else if (sync_rst) pb_work <= 1'b0;
    else if (pb_work && (ebk_work || core_dm_retire_debug_expt_i) && core_dm_itr_done_i)
        pb_work <= 1'b0;
    else if (pb_work_start && hartsum0[0] && cmderr == 3'h0)
        pb_work <= 1'b1;
end

assign busy = pb_work | cmd_work | itr_work;

assign dm_core_itr_o[31:0] = itr[31:0];

assign aarsize_err = dm_pwdata[22:20] != 3'h3 && (dm_pwdata[16] || dm_pwdata[22:20] != 3'h2);

assign dm_paddr_is_itr = dm_paddr[11:2] == OFFSET_ITR;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) cmderr[2:0] <= 3'h0;
    else if (sync_rst) cmderr[2:0] <= 3'h0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_ABSTRACTCS &&
             dm_pwdata[10:8] == 3'b111 && cmderr != 3'h0)
        cmderr[2:0] <= 3'h0;
    else if ((apbw_abscmd && ((dm_pwdata[31:24] != 8'h0) ||
              (dm_pwdata[17] && (dm_pwdata[15:0] >= 16'h1020 || aarsize_err))) && ~busy) ||
             (autoaccen && (cmdtype != 8'h0 || (transfer && regno >= 16'h1020))))
        cmderr[2:0] <= 3'h2;
    else if (core_dm_itr_done_i && core_dm_retire_debug_expt_i && busy)
        cmderr[2:0] <= 3'h3;
    else if ((((dm_intra_apbw && dm_paddr_is_itr) || cmd_start || pb_work_start) &&
             !hartsum0[0] && ~busy) || (itr_vld_en && !hartsum0[0]))
        cmderr[2:0] <= 3'h4;
    else if (busy && cmderr == 3'h0 && cmd_busy_set)
        cmderr[2:0] <= 3'h1;
end

assign abstractcs[31:0] = {3'b0, 5'd4 /*progbufsize*/, 11'b0, busy, 1'b0, cmderr,
                            4'b0, 4'h2 /*datacount*/};

//==========================================================
//    REGACC FSM (donor :2064-2181)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) regacc_cur_state <= 4'h0;
    else if (sync_rst) regacc_cur_state <= 4'h0;
    else regacc_cur_state <= regacc_nxt_state;
end

wire not_acc_dsc0;
assign not_acc_dsc0 = regno != 16'h7b2;

always @ (*) begin
    case (regacc_cur_state)
        REGACC_IDLE : if (transfer && cmd_start && ~access_illegal_reg && legal_cmd_sel) begin
                          if (write)
                              regacc_nxt_state = REGACC_WDSC0;
                          else if (access_gpr)
                              regacc_nxt_state = REGACC_G_2_DSC0;
                          else if (access_dsc0)
                              regacc_nxt_state = REGACC_RDSC0;
                          else if (access_csr)
                              regacc_nxt_state = REGACC_X6_2_DSC1;
                          else
                              regacc_nxt_state = REGACC_IDLE;
                      end else
                          regacc_nxt_state = REGACC_IDLE;
        REGACC_WDSC0 : if (core_dm_wr_ready_i) begin
                           if (access_gpr)
                              regacc_nxt_state = REGACC_DSC0_2_G;
                           else if (access_dsc0)
                              regacc_nxt_state = REGACC_IDLE;
                           else if (access_csr)
                              regacc_nxt_state = REGACC_X6_2_DSC1;
                           else
                              regacc_nxt_state = REGACC_WDSC0;
                       end else
                           regacc_nxt_state = REGACC_WDSC0;
        REGACC_G_2_DSC0 : if (core_dm_itr_done_i)
                              regacc_nxt_state = REGACC_RDSC0;
                          else
                              regacc_nxt_state = REGACC_G_2_DSC0;
        REGACC_RDSC0 : if (core_rdata_vld_r)
                           regacc_nxt_state = REGACC_IDLE;
                       else
                           regacc_nxt_state = REGACC_RDSC0;
        REGACC_X6_2_DSC1 : if (core_dm_itr_done_i) begin
                               if (access_csr && not_acc_dsc0) begin
                                   if (write)
                                       regacc_nxt_state = REGACC_DSC0_2_X6;
                                   else
                                       regacc_nxt_state = REGACC_C_2_X6;
                               end else
                                   regacc_nxt_state = REGACC_X6_2_DSC1;
                           end else
                               regacc_nxt_state = REGACC_X6_2_DSC1;
        REGACC_DSC0_2_G : if (core_dm_itr_done_i)
                              regacc_nxt_state = REGACC_IDLE;
                          else
                              regacc_nxt_state = REGACC_DSC0_2_G;
        REGACC_DSC1_2_X6 : if (core_dm_itr_done_i) begin
                               if (access_csr && not_acc_dsc0) begin
                                   if (write)
                                       regacc_nxt_state = REGACC_IDLE;
                                   else
                                       regacc_nxt_state = REGACC_RDSC0;
                               end else
                                   regacc_nxt_state = REGACC_DSC1_2_X6;
                           end else
                               regacc_nxt_state = REGACC_DSC1_2_X6;
        REGACC_DSC0_2_X6 : if (core_dm_itr_done_i)
                               regacc_nxt_state = REGACC_X6_2_C;
                           else
                               regacc_nxt_state = REGACC_DSC0_2_X6;
        REGACC_X6_2_DSC0 : if (core_dm_itr_done_i)
                               regacc_nxt_state = REGACC_DSC1_2_X6;
                           else
                               regacc_nxt_state = REGACC_X6_2_DSC0;
        REGACC_C_2_X6 : if (core_dm_itr_done_i)
                            regacc_nxt_state = REGACC_X6_2_DSC0;
                        else
                            regacc_nxt_state = REGACC_C_2_X6;
        REGACC_X6_2_C : if (core_dm_itr_done_i)
                            regacc_nxt_state = REGACC_DSC1_2_X6;
                        else
                            regacc_nxt_state = REGACC_X6_2_C;
        default : regacc_nxt_state = REGACC_IDLE;
    endcase
end

//==========================================================
//    ITR and DCC (donor :2182-2446)
//==========================================================
assign access_gpr  = cmdtype == 8'h0 && regno[15:5] == 11'h080;
assign access_csr  = cmdtype == 8'h0 && regno[15:12] == 4'h0;
assign access_dsc0 = cmdtype == 8'h0 && regno == 16'h7b2;
assign access_illegal_reg = cmdtype == 8'h0 && regno >= 16'h1020;

assign fsm_x6_2_dsc1 = regacc_nxt_state == REGACC_X6_2_DSC1 &&
    (regacc_cur_state == REGACC_IDLE || regacc_cur_state == REGACC_WDSC0);
assign fsm_x6_2_dsc0 = regacc_nxt_state == REGACC_X6_2_DSC0 &&
    regacc_cur_state == REGACC_C_2_X6;
assign fsm_dsc1_2_x6 = regacc_nxt_state == REGACC_DSC1_2_X6 &&
    (regacc_cur_state == REGACC_X6_2_DSC0 || regacc_cur_state == REGACC_X6_2_C);
assign fsm_dsc0_2_x6 = regacc_nxt_state == REGACC_DSC0_2_X6 &&
    regacc_cur_state == REGACC_X6_2_DSC1;
assign fsm_c_2_x6 = regacc_nxt_state == REGACC_C_2_X6 &&
    regacc_cur_state == REGACC_X6_2_DSC1;
assign fsm_x6_2_c = regacc_nxt_state == REGACC_X6_2_C &&
    regacc_cur_state == REGACC_DSC0_2_X6;

assign itr_send_pb = (pb_work_start && hartsum0[0]) ||
                     (pb_work && !ebk_work && core_dm_itr_done_i && !core_dm_retire_debug_expt_i);

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) itr_send_pb_r <= 1'b0;
    else if (sync_rst) itr_send_pb_r <= 1'b0;
    else if (itr_send_pb_r) itr_send_pb_r <= 1'b0;
    else if (itr_send_pb) itr_send_pb_r <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) itr[31:0] <= 32'b0;
    else if (sync_rst) itr[31:0] <= 32'b0;
    else if (dm_intra_apbw && dm_paddr_is_itr && legal_cmd_sel)
        itr[31:0] <= dm_pwdata[31:0];
    else if (pb_work)
        itr[31:0] <= pb_mux;
    else if (access_gpr) begin
        if (transfer && !write && cmd_start && legal_cmd_sel)
            // GPR read: csrrw x0, dscratch0, x{regno}  (:2280-2285)
            itr[31:0] <= {DSCR0_ADDR, regno[4:0], CSRRW_F3, X0_GPR, SYS_OPCODE};
        else if (write && core_dm_wr_ready_i)
            // GPR write: csrrc x{regno}, dscratch0, x0  (:2286-2290)
            itr[31:0] <= {DSCR0_ADDR, X0_GPR, CSRRC_F3, regno[4:0], SYS_OPCODE};
    end
    else if (access_csr && not_acc_dsc0) begin
        // CSR access: save/restore x6 through dscratch1 (:2293-2324)
        if (fsm_x6_2_dsc1)
            itr[31:0] <= {DSCR1_ADDR, 5'h6, CSRRW_F3, X0_GPR, SYS_OPCODE};
        else if (fsm_x6_2_dsc0)
            itr[31:0] <= {DSCR0_ADDR, 5'h6, CSRRW_F3, X0_GPR, SYS_OPCODE};
        else if (fsm_dsc1_2_x6)
            itr[31:0] <= {DSCR1_ADDR, X0_GPR, CSRRC_F3, 5'h6, SYS_OPCODE};
        else if (fsm_dsc0_2_x6)
            itr[31:0] <= {DSCR0_ADDR, X0_GPR, CSRRC_F3, 5'h6, SYS_OPCODE};
        else if (fsm_c_2_x6)
            itr[31:0] <= {regno[11:0], X0_GPR, CSRRC_F3, 5'h6, SYS_OPCODE};
        else if (fsm_x6_2_c)
            itr[31:0] <= {regno[11:0], 5'h6, CSRRW_F3, X0_GPR, SYS_OPCODE};
    end
end

assign itr_vld_en = (dm_intra_apbw && dm_paddr_is_itr && ~busy && hartsum0[0]) ||
                    (access_gpr && ((write && core_dm_wr_ready_i) ||
                     (!write && transfer && cmd_start && ~busy && hartsum0[0]))) ||
                    (access_csr && not_acc_dsc0 &&
                     (fsm_x6_2_c || fsm_c_2_x6 || fsm_dsc0_2_x6 || fsm_dsc1_2_x6 ||
                      fsm_x6_2_dsc1 || fsm_x6_2_dsc0)) ||
                    itr_send_pb_r;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) itr_vld <= 1'b0;
    else if (sync_rst) itr_vld <= 1'b0;
    else if (itr_vld) itr_vld <= 1'b0;
    else if (itr_vld_en) itr_vld <= 1'b1;
end

assign dm_core_itr_vld_o = itr_vld;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) itr_work <= 1'b0;
    else if (sync_rst) itr_work <= 1'b0;
    else if (core_dm_itr_done_i) itr_work <= 1'b0;
    else if (dm_intra_apbw && dm_paddr_is_itr && ~busy && hartsum0[0])
        itr_work <= 1'b1;
end

assign dm_core_wr_vld_en = transfer &&
    ((regacc_cur_state == REGACC_IDLE && regacc_nxt_state == REGACC_WDSC0) ||
     ((regacc_cur_state == REGACC_IDLE || regacc_cur_state == REGACC_G_2_DSC0 ||
       regacc_cur_state == REGACC_DSC1_2_X6) && regacc_nxt_state == REGACC_RDSC0));

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_core_wr_vld_o <= 1'b0;
    else if (sync_rst) dm_core_wr_vld_o <= 1'b0;
    else if (dm_core_wr_vld_en) dm_core_wr_vld_o <= 1'b1;
    else if (dm_core_wr_vld_o) dm_core_wr_vld_o <= 1'b0;
end

//==========================================================
//    data (donor :2448-2525)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) core_rdata_r <= 64'b0;
    else if (sync_rst) core_rdata_r <= 64'b0;
    else if (core_dm_wr_ready_i && dm_core_wr_flg_o != 2'b01)
        core_rdata_r <= core_dm_rx_data_i;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) core_rdata_vld_r <= 1'b0;
    else if (sync_rst) core_rdata_vld_r <= 1'b0;
    else if (core_dm_wr_ready_i) core_rdata_vld_r <= 1'b1;
    else if (core_rdata_vld_r) core_rdata_vld_r <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) data0 <= 32'b0;
    else if (sync_rst) data0 <= 32'b0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_DATA0 && ~busy)
        data0 <= dm_pwdata[31:0];
    else if (core_rdata_vld_r && dm_core_wr_flg_o != 2'b01)
        data0 <= core_rdata_r[31:0];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) data1 <= 32'b0;
    else if (sync_rst) data1 <= 32'b0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_DATA1 && ~busy)
        data1 <= dm_pwdata[31:0];
    else if (core_rdata_vld_r && dm_core_wr_flg_o != 2'b01)
        data1 <= core_rdata_r[63:32];
end

assign hartsum0[31:0] = {31'b0, core_dm_halted_i};

assign compid[31:0] = {JEP106_ID, COMP_TYPE, COMP_VER};

//==========================================================
//    SBA registers (donor :2741-3130)
//==========================================================
assign apbw_sbcs = dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBCS;

always @ (*) begin
    case (sbaccess)
        3'h2    : sbaddrplus = 5'd4;
        3'h3    : sbaddrplus = 5'd8;
        3'h4    : sbaddrplus = 5'd16;
        default : sbaddrplus = 5'd0;
    endcase
end

assign sbaddr0_plus_pre = {1'b0, sbaddr0} + {28'b0, sbaddrplus};
assign sb_noerr = ~sbbusy && ~sbbusyerror && sberror == 3'h0;
assign dm_paddr_is_sbaddr0 = dm_paddr[11:2] == OFFSET_SBADDR0;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbaddr0 <= 32'h0;
    else if (sync_rst) sbaddr0 <= 32'h0;
    else if (dm_intra_apbw && dm_paddr_is_sbaddr0 && sb_noerr)
        sbaddr0 <= dm_pwdata;
    else if (sbautoincrement && sba_wr_ready)
        sbaddr0 <= sbaddr0_plus_pre[31:0];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbaddr1 <= 32'h0;
    else if (sync_rst) sbaddr1 <= 32'h0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBADDR1 && sb_noerr)
        sbaddr1 <= dm_pwdata;
    else if (sbautoincrement && sba_wr_ready && sbaddr0_plus_pre[32])
        sbaddr1 <= sbaddr1 + 1'b1;
end

assign sbversion = 3'h1;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbbusy <= 1'b0;
    else if (sync_rst) sbbusy <= 1'b0;
    else if (sba_wr_ready) sbbusy <= 1'b0;
    else if (sba_wr_vld) sbbusy <= 1'b1;
end

assign sbbusyerr_set = (dm_intra_apbw   && dm_paddr[11:2] == OFFSET_SBADDR1) ||
                       (dm_intra_apbw   && dm_paddr[11:2] == OFFSET_SBADDR0) ||
                       (dm_intra_access && dm_paddr[11:2] == OFFSET_SBDATA0) ||
                       (dm_intra_access && dm_paddr[11:2] == OFFSET_SBDATA1) ||
                       (dm_intra_access && dm_paddr[11:2] == OFFSET_SBDATA2) ||
                       (dm_intra_access && dm_paddr[11:2] == OFFSET_SBDATA3);

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbbusyerror <= 1'b0;
    else if (sync_rst) sbbusyerror <= 1'b0;
    else if (sbbusyerr_set && sbbusy) sbbusyerror <= 1'b1;
    else if (apbw_sbcs && dm_pwdata[22]) sbbusyerror <= 1'b0;
end

assign sbaccess_unalian = (sbaccess == 3'h2 && |dm_pwdata[1:0]) ||
                          (sbaccess == 3'h3 && |dm_pwdata[2:0]) ||
                          (sbaccess == 3'h4 && |dm_pwdata[3:0]);

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sberror_will_be_4 <= 1'b0;
    else if (sync_rst) sberror_will_be_4 <= 1'b0;
    else if (apbw_sbcs && &dm_pwdata[14:12])
        sberror_will_be_4 <= 1'b0;
    else if (apbw_sbcs && (dm_pwdata[19:17] < 3'h2 || dm_pwdata[19:17] > 3'h4) && ~sbbusy)
        sberror_will_be_4 <= 1'b1;
    else if (apbw_sbcs && !(dm_pwdata[19:17] < 3'h2 || dm_pwdata[19:17] > 3'h4) && ~sbbusy)
        sberror_will_be_4 <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sberror_will_be_3 <= 1'b0;
    else if (sync_rst) sberror_will_be_3 <= 1'b0;
    else if (apbw_sbcs && &dm_pwdata[14:12])
        sberror_will_be_3 <= 1'b0;
    else if (dm_intra_apbw && dm_paddr_is_sbaddr0 && sbaccess_unalian && ~sbbusy)
        sberror_will_be_3 <= 1'b1;
    else if (dm_intra_apbw && dm_paddr_is_sbaddr0 && !sbaccess_unalian && ~sbbusy)
        sberror_will_be_3 <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sberror <= 3'h0;
    else if (sync_rst) sberror <= 3'h0;
    else if (apbw_sbcs && &dm_pwdata[14:12])
        sberror <= 3'h0;
    else if (sberror_will_be_4 && (sba_write_ignore_unalign_f || sba_read_ignore_unalign_f) && sb_noerr)
        sberror <= 3'h4;
    else if (sberror_will_be_3 && (sba_write_ignore_unalign_f || sba_read_ignore_unalign_f) && sb_noerr)
        sberror <= 3'h3;
    else if (sba_error && sba_wr_ready)
        sberror <= 3'h7;
end

assign sbasize = 7'd40;
assign sbaccess_info = 5'b11100;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbreadonaddr <= 1'b0;
    else if (sync_rst) sbreadonaddr <= 1'b0;
    else if (apbw_sbcs) sbreadonaddr <= dm_pwdata[20];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbaccess <= 3'h2;
    else if (sync_rst) sbaccess <= 3'h2;
    else if (apbw_sbcs) sbaccess <= dm_pwdata[19:17];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbautoincrement <= 1'b0;
    else if (sync_rst) sbautoincrement <= 1'b0;
    else if (apbw_sbcs) sbautoincrement <= dm_pwdata[16];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbreadondata <= 1'b0;
    else if (sync_rst) sbreadondata <= 1'b0;
    else if (apbw_sbcs) sbreadondata <= dm_pwdata[15];
end

assign sbcs = {sbversion, 6'h0, sbbusyerror, sbbusy, sbreadonaddr,
               sbaccess, sbautoincrement, sbreadondata, sberror,
               sbasize, sbaccess_info};

assign sba_write = dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBDATA0 &&
                   sbaddr0[1:0] == 2'b0 && !(sbreadonaddr || sbreadondata);
assign sba_read  = (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBADDR0 &&
                    dm_pwdata[1:0] == 2'b0 && sbreadonaddr) ||
                   (dm_intra_apbr && dm_paddr[11:2] == OFFSET_SBDATA0 &&
                    sbaddr0[1:0] == 2'b0 && sbreadondata);

assign sba_write_ignore_unalign = dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBDATA0 &&
                                   !(sbreadonaddr || sbreadondata);
assign sba_read_ignore_unalign  = (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBADDR0 &&
                                    sbreadonaddr) ||
                                   (dm_intra_apbr && dm_paddr[11:2] == OFFSET_SBDATA0 &&
                                    sbreadondata);

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_write_ignore_unalign_f <= 1'b0;
    else if (sba_write_ignore_unalign_f) sba_write_ignore_unalign_f <= 1'b0;
    else if (sba_write_ignore_unalign) sba_write_ignore_unalign_f <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_read_ignore_unalign_f <= 1'b0;
    else if (sba_read_ignore_unalign_f) sba_read_ignore_unalign_f <= 1'b0;
    else if (sba_read_ignore_unalign) sba_read_ignore_unalign_f <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_wr_flg <= 1'b0;
    else if (sync_rst) sba_wr_flg <= 1'b0;
    else if (sba_write) sba_wr_flg <= 1'b1;
    else if (sba_read) sba_wr_flg <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_wr_vld <= 1'b0;
    else if (sync_rst) sba_wr_vld <= 1'b0;
    else if (sba_wr_vld) sba_wr_vld <= 1'b0;
    else if ((sba_write || sba_read) && sb_noerr && !sberror_will_be_3 && !sberror_will_be_4)
        sba_wr_vld <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbdata0 <= 32'h0;
    else if (sync_rst) sbdata0 <= 32'h0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBDATA0 && sb_noerr)
        sbdata0 <= dm_pwdata;
    else if (sba_wr_ready && !sba_wr_flg)
        sbdata0 <= sba_rd_data[31:0];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbdata1 <= 32'h0;
    else if (sync_rst) sbdata1 <= 32'h0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBDATA1 && sb_noerr)
        sbdata1 <= dm_pwdata;
    else if (sba_wr_ready && !sba_wr_flg && sbaccess > 3'h2)
        sbdata1 <= sba_rd_data[63:32];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbdata2 <= 32'h0;
    else if (sync_rst) sbdata2 <= 32'h0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBDATA2 && sb_noerr)
        sbdata2 <= dm_pwdata;
    else if (sba_wr_ready && !sba_wr_flg && sbaccess > 3'h3)
        sbdata2 <= sba_rd_data[95:64];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sbdata3 <= 32'h0;
    else if (sync_rst) sbdata3 <= 32'h0;
    else if (dm_intra_apbw && dm_paddr[11:2] == OFFSET_SBDATA3 && sb_noerr)
        sbdata3 <= dm_pwdata;
    else if (sba_wr_ready && !sba_wr_flg && sbaccess > 3'h3)
        sbdata3 <= sba_rd_data[127:96];
end

assign sba_w_data = {sbdata3, sbdata2, sbdata1, sbdata0};
assign sba_wr_addr_pre = {sbaddr1, sbaddr0};
assign sba_wr_addr = sba_wr_addr_pre[SBAW-1:0];
assign sba_wr_size = sbaccess;

//==========================================================
//    SBA AXI controller (donor tdt_sba_axi.v:72-342)
//    D-M7-1: single clock, no CDC pulse syncs.
//    D-M7-9: no clock gating; gated_clk_cell dropped.
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) begin
        s_wr_data <= 128'b0;
        s_wr_flg  <= 1'b0;
        s_wr_addr <= {SBAW{1'b0}};
        s_wr_size <= 3'b0;
        s_wr_vld  <= 1'b0;
    end else begin
        if (sba_wr_vld) begin
            s_wr_data <= sba_w_data;
            s_wr_flg  <= sba_wr_flg;
            s_wr_addr <= sba_wr_addr;
            s_wr_size <= sba_wr_size;
        end
        s_wr_vld <= sba_wr_vld;
    end
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_awvalid <= 1'b0;
    else if (dm_pad_awvalid & pad_dm_awready) dm_pad_awvalid <= 1'b0;
    else if (s_wr_vld & s_wr_flg) dm_pad_awvalid <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_wvalid <= 1'b0;
    else if (dm_pad_wvalid & pad_dm_wready) dm_pad_wvalid <= 1'b0;
    else if (s_wr_vld & s_wr_flg) dm_pad_wvalid <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_awaddr <= {SBAW{1'b0}};
    else if (s_wr_vld & s_wr_flg) dm_pad_awaddr <= s_wr_addr;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_awsize <= 3'b0;
    else if (s_wr_vld & s_wr_flg) dm_pad_awsize <= s_wr_size;
end

assign wstrb_pre = (s_wr_size == 3'h4) ? 16'hffff :
                   (s_wr_size == 3'h3) ? 16'h00ff : 16'h000f;

assign addr_alian = s_wr_addr[3:0];

always @ (*) begin
    case (addr_alian[3:2])
        2'b00: wdata_pre = s_wr_data;
        2'b01: wdata_pre = {s_wr_data[95:0], 32'b0};
        2'b10: wdata_pre = {s_wr_data[63:0], 64'b0};
        2'b11: wdata_pre = {s_wr_data[31:0], 96'b0};
    endcase
end

always @ (*) begin
    case (addr_alian[3:2])
        2'b00: wstrb_pre1 = wstrb_pre;
        2'b01: wstrb_pre1 = {wstrb_pre[11:0], 4'h0};
        2'b10: wstrb_pre1 = {wstrb_pre[7:0],  8'h0};
        2'b11: wstrb_pre1 = {wstrb_pre[3:0],  12'h0};
    endcase
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_wdata <= 128'b0;
    else if (s_wr_vld & s_wr_flg) dm_pad_wdata <= wdata_pre;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_wstrb <= 16'b0;
    else if (s_wr_vld & s_wr_flg) dm_pad_wstrb <= wstrb_pre1;
end

assign dm_pad_bready  = 1'b1;
assign dm_pad_wlast   = 1'b1;
assign dm_pad_awlen   = 4'h0;
assign dm_pad_arlen   = 4'h0;
assign dm_pad_rready  = 1'b1;
assign dm_pad_awid    = 4'b0;
assign dm_pad_arid    = 4'b0;
assign dm_pad_awburst = 2'b01;
assign dm_pad_awcache = 4'h0;
assign dm_pad_awlock  = 1'b0;
assign dm_pad_awprot  = 3'b010;
assign dm_pad_arburst = 2'b01;
assign dm_pad_arcache = 4'h0;
assign dm_pad_arlock  = 1'b0;
assign dm_pad_arprot  = 3'b010;

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_arvalid <= 1'b0;
    else if (dm_pad_arvalid & pad_dm_arready) dm_pad_arvalid <= 1'b0;
    else if (s_wr_vld & !s_wr_flg) dm_pad_arvalid <= 1'b1;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_araddr <= {SBAW{1'b0}};
    else if (s_wr_vld & !s_wr_flg) dm_pad_araddr <= s_wr_addr;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pad_arsize <= 3'b0;
    else if (s_wr_vld & !s_wr_flg) dm_pad_arsize <= s_wr_size;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) rdata_smp <= 128'b0;
    else if (pad_dm_rvalid) rdata_smp <= pad_dm_rdata;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) axi_wr_ready_pre <= 1'b0;
    else if (axi_wr_ready_pre) axi_wr_ready_pre <= 1'b0;
    else if ((s_wr_flg && pad_dm_bvalid) || (!s_wr_flg && pad_dm_rvalid))
        axi_wr_ready_pre <= 1'b1;
end

always @ (*) begin
    case (addr_alian[3:2])
        2'b00 : rd_data_pre = rdata_smp;
        2'b01 : rd_data_pre = {32'h0, rdata_smp[127:32]};
        2'b10 : rd_data_pre = {64'h0, rdata_smp[127:64]};
        2'b11 : rd_data_pre = {96'h0, rdata_smp[127:96]};
    endcase
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_wr_ready <= 1'b0;
    else if (axi_wr_ready_pre) sba_wr_ready <= 1'b1;
    else if (sba_wr_ready) sba_wr_ready <= 1'b0;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_rd_data <= 128'b0;
    else if (axi_wr_ready_pre) sba_rd_data <= rd_data_pre;
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_error_pre <= 1'b0;
    else if (pad_dm_bvalid) sba_error_pre <= pad_dm_bresp[1];
    else if (pad_dm_rvalid) sba_error_pre <= pad_dm_rresp[1];
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) sba_error <= 1'b0;
    else if (axi_wr_ready_pre) sba_error <= sba_error_pre;
end

//==========================================================
//    APB pready (donor :3315-3322)
//==========================================================
always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_pready <= 1'b0;
    else if (dm_pready) dm_pready <= 1'b0;
    else if (dm_psel && !dm_penable) dm_pready <= 1'b1;
end

//==========================================================
//    nextdm / hartinfo (donor :3324-3362)
//==========================================================
always @ (*) begin
    hartinfo = {8'h0, 4'h2 /*nscratch=2*/, 20'h0};
end

//==========================================================
//    APB read mux (donor :3364-3470)
//==========================================================
reg [31:0] prdata_pre;

always @ (*) begin
    case (dm_paddr[11:2])
        OFFSET_DATA0     : prdata_pre = data0;
        OFFSET_DATA1     : prdata_pre = data1;
        OFFSET_DMCONTROL : prdata_pre = dmcontrol;
        OFFSET_DMSTATUS  : prdata_pre = dmstatus;
        OFFSET_HARTINFO  : prdata_pre = hartinfo;
        OFFSET_HAWINDOW  : prdata_pre = hawindow;
        OFFSET_ABSTRACTCS: prdata_pre = abstractcs;
        OFFSET_COMMAND   : prdata_pre = 32'h0;
        OFFSET_ABSTRACTAUTO: prdata_pre = abstractauto;
        OFFSET_NEXTDM    : prdata_pre = 32'h0;
        OFFSET_PB0       : prdata_pre = progbuf[0];
        OFFSET_PB1       : prdata_pre = progbuf[1];
        OFFSET_PB2       : prdata_pre = progbuf[2];
        OFFSET_PB3       : prdata_pre = progbuf[3];
        OFFSET_DMCS2     : prdata_pre = dmcs2;
        OFFSET_SBCS      : prdata_pre = sbcs;
        OFFSET_SBADDR0   : prdata_pre = sbaddr0;
        OFFSET_SBADDR1   : prdata_pre = sbaddr1;
        OFFSET_SBDATA0   : prdata_pre = sbdata0;
        OFFSET_SBDATA1   : prdata_pre = sbdata1;
        OFFSET_SBDATA2   : prdata_pre = sbdata2;
        OFFSET_SBDATA3   : prdata_pre = sbdata3;
        OFFSET_HARTSUM0  : prdata_pre = hartsum0;
        OFFSET_ITR       : prdata_pre = itr;
        OFFSET_CUSCS     : prdata_pre = 32'h0;   // D-M7-5
        OFFSET_CUSCMD    : prdata_pre = 32'h0;   // D-M7-5
        OFFSET_COMPID    : prdata_pre = compid;
        default          : prdata_pre = 32'h0;
    endcase
end

always @ (posedge clk or negedge tdt_rst_n) begin
    if (!tdt_rst_n) dm_prdata <= 32'b0;
    else if (sync_rst) dm_prdata <= 32'b0;
    else if (dm_psel && !dm_penable) dm_prdata <= prdata_pre;
end

assign dm_pslverr = 1'b0;

endmodule
