//=============================================================================
// AXIAddrDecode.v - Address Decoder for AXI Crossbar
//=============================================================================
// Default implementation using base/mask pairs.
// Users can replace this module with their own implementation.
//
// Interface:
//   Input:  addr[ADDR_WIDTH-1:0]
//   Output: slave_id[$clog2(N_SLAVES)-1:0]
//
// Example custom implementation:
//   module AXIAddrDecode #(...) (...);
//       localparam DI_MEM  = 0;
//       localparam DI_CLINT = 1;
//       localparam DI_PLIC = 2;
//       localparam DI_UART = 3;
//       assign slave_id = (addr[31]) ? DI_MEM :
//                         ((addr[31:16] == 16'h0200)) ? DI_CLINT :
//                         ((addr[31:24] == 8'h0C)) ? DI_PLIC : DI_UART;
//   endmodule
//=============================================================================

module AXIAddrDecode #(
    parameter ADDR_WIDTH = 64,
    parameter N_SLAVES   = 4,
    parameter SLAVE_ID_BITS = $clog2(N_SLAVES) > 0 ? $clog2(N_SLAVES) : 1,

    // Address mapping: {slave[N-1], ..., slave[0]}
    parameter [N_SLAVES*32-1:0] ADDR_BASE = {32'h10010000, 32'h0C000000, 32'h02000000, 32'h80000000},
    parameter [N_SLAVES*32-1:0] ADDR_MASK = {32'hFFFF0000, 32'hFF000000, 32'hFFFF0000, 32'h80000000},

    // Default slave when no match (typically memory or error)
    parameter DEFAULT_SLAVE = 0
)(
    input  wire [ADDR_WIDTH-1:0]     addr,
    output wire [SLAVE_ID_BITS-1:0]  slave_id
);

    // Decode logic: check each slave's address range
    reg [SLAVE_ID_BITS-1:0] decoded;
    integer s;

    always @(*) begin
        decoded = DEFAULT_SLAVE[SLAVE_ID_BITS-1:0];
        for (s = 0; s < N_SLAVES; s = s + 1) begin
            if ((addr[31:0] & ADDR_MASK[s*32 +: 32]) ==
                (ADDR_BASE[s*32 +: 32] & ADDR_MASK[s*32 +: 32])) begin
                decoded = s[SLAVE_ID_BITS-1:0];
            end
        end
    end

    assign slave_id = decoded;

endmodule
