//=============================================================================
// SRAM.v - behavioral single-port SRAM (the one SRAM model for rv906)
//=============================================================================
// C906 files covered:
//   gen_rtl/ifu/rtl/aq_spsram_256x59.v    (ICache tag array, 1x, 256 sets)
//   gen_rtl/ifu/rtl/aq_spsram_2048x32.v   (ICache data array, 4x banks)
//   gen_rtl/ifu/rtl/aq_spsram_1024x16.v   (BHT array, 1x)
// Reference: IFU pipeline extraction notes S1/S3, BPU extraction notes S5.
// Read directly (not assumed to match C910's ct_spsram_* convention): each
// aq_spsram_* wraps an FPGA behavioral macro (aq_f_spsram_*) with ports
// A[ADDR_WIDTH-1:0], CEN, CLK, D[DATA_WIDTH-1:0], GWEN, Q[DATA_WIDTH-1:0],
// WEN[WE_WIDTH-1:0] -- no "_n" suffix in the primitive's own port names, but
// every wrapper module that instantiates one (aq_ifu_icache_tag_array.v:95-
// 101, aq_ifu_icache_data_array.v:197-199, aq_ifu_bht_array.v:94-96) drives
// CEN/GWEN/WEN from a "_b" (bar) signal it computes itself
// (icache_tag_cen_b = !icache_tag_cen; icache_tag_gwen_b =
// !(|icache_tag_wen); icache_tag_bwen_b = ~{...}) -- i.e. the primitive's
// CEN/GWEN/WEN inputs are consumed as ACTIVE LOW, exactly the same
// convention as C910's ct_spsram_*/ct_f_spsram_*.
//
// Contract:
//   * every control is ACTIVE LOW, exactly like the donor macro;
//   * cen_n=1 (chip disabled): nothing happens, q HOLDS its previous value;
//   * cen_n=0, gwen_n=1: READ. q gets mem[addr] one cycle later.
//   * cen_n=0, gwen_n=0: WRITE. Bit i of mem[addr] is updated only where
//     wen_n[i]==0 (bitwise lane enables); q is NOT updated by a write cycle,
//     so a read issued before a write still returns the pre-write row.
// Deviation from the donor macro: the donor (aq_f_spsram_* behind
// aq_spsram_*) is a synchronous-read RAM with a registered output that is
// WRITE-THROUGH (a write cycle also loads Q with the write data on written
// lanes). No ICache/BHT consumer reads Q on a write cycle in this project,
// so rv906 holds Q across write cycles instead (no write-through) -- the two
// are equivalent for every consumer here, same deviation rv12 recorded for
// its own C910 SRAM model.
//
// One model, three instantiations (per plan Task 1.1): WIDTH=59/DEPTH=256
// for the ICache tag array, WIDTH=32/DEPTH=2048 (x4 banks) for the ICache
// data array, WIDTH=16/DEPTH=1024 for the BHT array.
//=============================================================================

module SRAM #(
    parameter WIDTH  = 32,                  // bits per row
    parameter DEPTH  = 1024,                // number of rows
    parameter ADDR_W = $clog2(DEPTH)        // derived; do not override
)(
    input  wire                 clk,
    input  wire                 cen_n,      // chip enable,        active low
    input  wire                 gwen_n,     // global write enable, active low
    input  wire [WIDTH-1:0]     wen_n,      // per-bit write enable, active low
    input  wire [ADDR_W-1:0]    addr,
    input  wire [WIDTH-1:0]     d,
    output wire [WIDTH-1:0]     q
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [WIDTH-1:0] q_r;

    wire            wr_cyc = !cen_n && !gwen_n;
    wire            rd_cyc = !cen_n &&  gwen_n;
    wire [WIDTH-1:0] bit_en = ~wen_n;       // 1 = this bit is written

    always @(posedge clk) begin
        if (wr_cyc)
            mem[addr] <= (mem[addr] & ~bit_en) | (d & bit_en);
        else if (rd_cyc)
            q_r <= mem[addr];
        // cen_n=1: array untouched, q_r holds
    end

    assign q = q_r;

endmodule
