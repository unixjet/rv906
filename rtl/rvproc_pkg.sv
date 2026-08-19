//=============================================================================
// rvproc_pkg.sv - rv906 shared parameters (seed version, M0)
//=============================================================================
// Unlike RV12 (which parameterizes NUM_CORES/HAS_L2/HAS_DEBUG), rv906 clones
// one fixed openc906 configuration -- see design doc S2.3. This package has
// no configurability knobs; it only grows subsystem structure constants
// (cache geometry, jTLB/PMP sizing, predictor tables) as each milestone
// extracts them from refs/openc906, per the design's alignment contract
// ("never quoted from memory").
//=============================================================================
package rvproc_pkg;

parameter XLEN = 64;
parameter ILEN = 32;

//-----------------------------------------------------------------------------
// M0: address map shared by the RTL and the harness (from rv12's rocketM
// fabric; the FDT and TestMaster placeholder core below must stay in sync
// with these).
//-----------------------------------------------------------------------------
parameter [63:0] ADDR_MEM_BASE   = 64'h0000_0000_8000_0000;
parameter [63:0] ADDR_CLINT_BASE = 64'h0000_0000_0200_0000;
parameter [63:0] ADDR_PLIC_BASE  = 64'h0000_0000_0C00_0000;
parameter [63:0] ADDR_UART_BASE  = 64'h0000_0000_1000_1000;
parameter [63:0] ADDR_TOHOST     = 64'h0000_0000_9000_1000;

endpackage
