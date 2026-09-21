//=============================================================================
// SBA_AxiUp.v - 128->512 AXI up-converter for the SBA master
//=============================================================================
// rv906-specific glue (design doc 2026-09-11-m7-debug-design.md :126-131,
// D-M7-4, D-M7-10). The TDT_DM SBA master is a single-beat 128-bit AXI4
// master (donor tdt_sba_axi.v, cloned into TDT_DM.v); the rv906 AXI
// crossbar is 512-bit (64-byte beats). The donor SoC never validated the
// SBA path (its interconnect ties the SBA port off, tr_axi_interconnect.v:
// 861-895) so this adapter is rv906 glue: it is pure combinational (no
// state, no reset) and maps the SBA's 16-byte transfer into the 64-byte
// crossbar beat.
//
// 128-bit side (m_*): TDT_DM's SBA AXI master (TDT_DM.v:75-110).
// 512-bit side (s_*): crossbar master port m[2] (AXICrossbar.v, no ID
// channels, M_DATA_WIDTH=512).
//
// BEAT CONVENTION (rv906 "contract 17", see LSU.v:2544-2553 and the
// memory model io/ExtMem.h:249-258): the 512-bit data bus carries the 64
// bytes starting at the 64-byte-aligned base of the transaction address;
// a data byte at absolute address x sits at bus byte offset x[5:0]; the
// model does memcpy(mem + (addr & 0xfff), din + (addr & 0x3f), 1<<size),
// so the AW/AR size must be the ACTUAL access size (2/3/4), passed
// through unchanged. DCache precedent: LSU.v:2193-2195 (store data at
// byte offset pa[5:3]*8, awaddr = byte address, awsize = store size).
//
// WHAT THE SBA MASTER DRIVES (donor semantics, already inlined into
// TDT_DM.v): for an access at address a (4-byte aligned; 8/16-byte
// accesses aligned to 8/16 by the DM's sbaccess_unalian check,
// TDT_DM.v:956-958) the 128-bit wdata is the 16-byte window
// [a & ~15 .. a & ~15 + 15] with the accessed bytes 4-byte-shifted into
// window offset a[3:0] (donor tdt_sba_axi.v:171-188, clone TDT_DM.v:
// 1153-1174), and wstrb (16 bits) is the size mask 16'h000f/16'h00ff/
// 16'hffff (donor tdt_sba_axi.v:159-160) shifted the same 4*a[3:2]
// bytes.
//
// MAPPING (this adapter): the 16-byte window is placed at byte offset
// 16*a[5:4] inside the 64-byte beat (beat base = a & ~63), i.e. the
// window's own 16-byte alignment is preserved:
//
//   a[5:4]   window bytes of the beat   s_wdata / s_wstrb placement
//   2'b00    0..15     s_wdata[127:0]   = m_wdata / s_wstrb[15:0]   = m_wstrb
//   2'b01    16..31    s_wdata[255:128] = m_wdata / s_wstrb[31:16]  = m_wstrb
//   2'b10    32..47    s_wdata[383:256] = m_wdata / s_wstrb[47:32]  = m_wstrb
//   2'b11    48..63    s_wdata[511:384] = m_wdata / s_wstrb[63:48]  = m_wstrb
//
// BYTE-LANE PROOF: the SBA access covers absolute addresses
// a .. a+size-1 (size = 2^awsize = 4/8/16). Inside the beat these are
// byte offsets a[5:0] .. a[5:0]+size-1:
//   - 4-byte (a[1:0]=0): valid window lanes a[3:0]..a[3:0]+3
//     (TDT_DM.v:1153-1154,1167-1174) -> beat bytes 16*a[5:4]+a[3:0]
//     = a[5:0] .. a[5:0]+3
//   - 8-byte (a[2:0]=0): valid lanes a[3:0]..a[3:0]+7 -> a[5:0]..a[5:0]+7
//   - 16-byte (a[3:0]=0): valid lanes 0..15 -> a[5:0]..a[5:0]+15
//     (a[5:0] in {0,16,32,48}; worst case 48..63 fits the beat)
// The 16-lane window itself is 16-byte aligned so it always fits inside
// the 64-byte beat (max 48..63). No transfer straddles a beat boundary.
//
// READ SIDE: the crossbar returns the 512-bit beat (base a & ~63); the
// 16-byte window [a & ~15 .. a & ~15 + 15] sits at beat byte offset
// 16*araddr[5:4], so m_rdata = the window slice of s_rdata. TDT_DM then
// applies its own 4-byte-unit read shift (TDT_DM.v:1230-1237, donor
// tdt_sba_axi.v:286-294) to place the accessed bytes at
// sba_rd_data[32*size-1:0].
//
// ADDRESS: 40->64-bit zero extension (sbasize=40, design doc D-M7-10).
// Single-beat in and out: the SBA master drives awlen=arlen=0,
// wlast=1 always (TDT_DM.v:1186-1189); the adapter passes them through.
// prot/burst: passed through (donor values 3'b010 / 2'b01,
// TDT_DM.v:1193,1196,1197,1200).
//
// TDT_DM's ID/cache/lock outputs (dm_pad_awid/arid/awcache/awlock/
// arcache/arlock) are part of the donor port shape but the rv906
// crossbar has no ID channels and ignores cache/lock; they are left
// unconnected at the RVProcAXI instance. m_bid/m_rid are tied 0 here.
//=============================================================================

module SBA_AxiUp (
    //=========================================================================
    // 128-bit SBA master side (from TDT_DM)
    //=========================================================================
    input  wire [39:0]  m_awaddr,
    input  wire [3:0]   m_awlen,
    input  wire [2:0]   m_awsize,
    input  wire [1:0]   m_awburst,
    input  wire [2:0]   m_awprot,
    input  wire         m_awvalid,
    output wire         m_awready,

    input  wire [127:0] m_wdata,
    input  wire [15:0]  m_wstrb,
    input  wire         m_wvalid,
    input  wire         m_wlast,
    output wire         m_wready,

    input  wire         m_bready,
    output wire [1:0]   m_bresp,
    output wire         m_bvalid,
    input  wire [39:0]  m_araddr,
    input  wire [3:0]   m_arlen,
    input  wire [2:0]   m_arsize,
    input  wire [1:0]   m_arburst,
    input  wire [2:0]   m_arprot,
    input  wire         m_arvalid,
    output wire         m_arready,

    output wire [127:0] m_rdata,
    output wire         m_rvalid,
    output wire         m_rlast,
    output wire [1:0]   m_rresp,
    input  wire         m_rready,

    //=========================================================================
    // 512-bit crossbar master side (crossbar m[2])
    //=========================================================================
    output wire [63:0]  s_awaddr,
    output wire [7:0]   s_awlen,
    output wire [2:0]   s_awsize,
    output wire [1:0]   s_awburst,
    output wire [2:0]   s_awprot,
    output wire         s_awvalid,
    input  wire         s_awready,

    output wire [511:0] s_wdata,
    output wire [63:0]  s_wstrb,
    output wire         s_wvalid,
    output wire         s_wlast,
    input  wire         s_wready,

    input  wire         s_bvalid,
    output wire         s_bready,
    input  wire [1:0]   s_bresp,

    output wire [63:0]  s_araddr,
    output wire [7:0]   s_arlen,
    output wire [2:0]   s_arsize,
    output wire [1:0]   s_arburst,
    output wire [2:0]   s_arprot,
    output wire         s_arvalid,
    input  wire         s_arready,

    input  wire [511:0] s_rdata,
    input  wire         s_rvalid,
    input  wire         s_rlast,
    input  wire [1:0]   s_rresp,
    output wire         s_rready
);

    //=========================================================================
    // Window placement: 16-byte window at beat byte offset 16*a[5:4].
    // m_awaddr / m_araddr are TDT_DM-registered and stable for the whole
    // in-flight transaction (TDT_DM.v:1143-1151, 1208-1216; sbbusy gates
    // sba_wr_vld, TDT_DM.v:912,1059-1065), so the combinational placement
    // below is glitch-free across the AW/W and AR/R handshakes.
    //=========================================================================
    wire [1:0] w_win = m_awaddr[5:4];   // 0..3 -> beat bytes 0/16/32/48
    wire [1:0] r_win = m_araddr[5:4];

    assign s_awaddr  = {24'b0, m_awaddr};
    assign s_awlen   = {4'b0, m_awlen};   // 4'h0 (TDT_DM.v:1188); the
                                           // crossbar's awlen is 8 bits
    assign s_awsize  = m_awsize;         // 2/3/4: actual access size --
                                         // the ExtMem model copies
                                         // 1<<size bytes (io/ExtMem.h:254)
    assign s_awburst = m_awburst;        // 2'b01 (TDT_DM.v:1193)
    assign s_awprot  = m_awprot;         // 3'b010 (TDT_DM.v:1196)
    assign s_awvalid = m_awvalid;
    assign m_awready = s_awready;

    // 16-byte window -> 64-byte beat (write side). Placement follows the
    // MAPPING table above: in {A, B}, A is the MSB, so the window at
    // beat bytes 0..15 is {384'b0, m_wdata} (window in the LOW bits) and
    // the window at beat bytes 48..63 is {m_wdata, 384'b0} (window in
    // the HIGH bits). All four placement wires are exactly 512 / 64
    // bits wide; the 4:1 select is a 2:1 tree over the declared wires.
    wire [511:0] wdata_p0 = {384'b0, m_wdata};          // beat bytes 0..15
    wire [511:0] wdata_p1 = {256'b0, m_wdata, 128'b0};  // beat bytes 16..31
    wire [511:0] wdata_p2 = {128'b0, m_wdata, 256'b0};  // beat bytes 32..47
    wire [511:0] wdata_p3 = {m_wdata, 384'b0};          // beat bytes 48..63
    wire [63:0]  wstrb_p0 = {48'b0,  m_wstrb};          // bits 15:0
    wire [63:0]  wstrb_p1 = {32'b0,  m_wstrb, 16'b0};   // bits 31:16
    wire [63:0]  wstrb_p2 = {16'b0,  m_wstrb, 32'b0};   // bits 47:32
    wire [63:0]  wstrb_p3 = {m_wstrb, 48'b0};           // bits 63:48

    wire [511:0] wdata_p01 = w_win[0] ? wdata_p1 : wdata_p0;
    wire [511:0] wdata_p23 = w_win[0] ? wdata_p3 : wdata_p2;
    assign s_wdata  = w_win[1] ? wdata_p23 : wdata_p01;
    wire [63:0]  wstrb_p01 = w_win[0] ? wstrb_p1 : wstrb_p0;
    wire [63:0]  wstrb_p23 = w_win[0] ? wstrb_p3 : wstrb_p2;
    assign s_wstrb  = w_win[1] ? wstrb_p23 : wstrb_p01;
    assign s_wvalid = m_wvalid;
    assign s_wlast  = m_wlast;           // 1'b1 (TDT_DM.v:1187)
    assign m_wready = s_wready;

    assign s_bready = m_bready;          // 1'b1 (TDT_DM.v:1186)
    assign m_bvalid = s_bvalid;
    assign m_bresp  = s_bresp;

    assign s_araddr  = {24'b0, m_araddr};
    assign s_arlen   = {4'b0, m_arlen};   // 4'h0 (TDT_DM.v:1189)
    assign s_arsize  = m_arsize;         // 2/3/4, as on the write side
    assign s_arburst = m_arburst;        // 2'b01 (TDT_DM.v:1197)
    assign s_arprot  = m_arprot;         // 3'b010 (TDT_DM.v:1200)
    assign s_arvalid = m_arvalid;
    assign m_arready = s_arready;

    // 64-byte beat -> 16-byte window (read side). Same 2:1 tree form as
    // the write side (part-select arms, verified in generated C++).
    wire [127:0] rdata_p0 = s_rdata[127:0];
    wire [127:0] rdata_p1 = s_rdata[255:128];
    wire [127:0] rdata_p2 = s_rdata[383:256];
    wire [127:0] rdata_p3 = s_rdata[511:384];

    wire [127:0] rdata_p01 = r_win[0] ? rdata_p1 : rdata_p0;
    wire [127:0] rdata_p23 = r_win[0] ? rdata_p3 : rdata_p2;
    assign m_rdata = r_win[1] ? rdata_p23 : rdata_p01;
    assign m_rvalid = s_rvalid;
    assign m_rlast  = s_rlast;
    assign m_rresp  = s_rresp;
    assign s_rready = m_rready;          // 1'b1 (TDT_DM.v:1190)
endmodule
