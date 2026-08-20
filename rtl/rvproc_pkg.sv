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

//=============================================================================
// M1 additions (IFU + ICache + BPU). Sources cited inline; every constant
// below is a fact read directly out of refs/openc906, not carried over from
// rv12's C910 clone -- see docs/superpowers/specs/notes/2026-08-20-c906-
// ifu-pipeline-extraction.md and .../2026-08-20-c906-bpu-extraction.md.
//=============================================================================

//-----------------------------------------------------------------------------
// M1: program counters (IFU notes "Global", cpu_cfig.h:142)
//-----------------------------------------------------------------------------
// C906's PC is a BYTE address, 40 bits, throughout the front end
// (cp0_xx_mrvbr[39:0], pcgen_ifpc[39:0]) -- NOT a halfword convention like
// C910's. Some internal IFU buses are nonetheless carried on a full 64-bit
// (XLEN-wide) wire even though only the low 40 bits are architecturally
// meaningful: pcgen_icache_va[63:0] = pcgen_fetch_pc[63:0] (confirmed
// aq_ifu_pcgen.v:92-93,310; aq_ifu_icache.v:120 `input [63:0]
// pcgen_icache_va`), and icache_rd_addr[63:0] inside the ICache truncates
// back down to [39:0] on the way out to pcgen (icache.v:1350
// `icache_pcgen_addr[39:0] = icache_rd_addr[39:0]`). rv906 clones this
// bus-width fact exactly (uses XLEN, not PC_WIDTH, for those specific wires)
// rather than narrowing it, since Task 2/3 will find the same widths in the
// real RTL and the freeze must not need rewidening later.
parameter PC_WIDTH = 40;                  // byte address

// MMU/ITLB interface widths -- read directly from aq_ifu_icache.v's own
// port declarations (icache.v:74-81,113-116,153-155; the IFU pipeline note
// only summarized this in passing, so this was confirmed by reading the
// cited RTL file directly per the plan's normative-documents rule):
//   output ifu_mmu_abort; output [51:0] ifu_mmu_va; output ifu_mmu_va_vld;
//   input  mmu_ifu_access_fault; input [27:0] mmu_ifu_pa;
//   input  mmu_ifu_pa_vld; input [4:0] mmu_ifu_prot;
// ifu_mmu_va[51:0] = icache_rd_addr[63:12] (a 52-bit VA page number on the
// 64-bit VA bus above); mmu_ifu_pa[27:0] is the PA page number, PA[39:12].
// Unlike C910's ifdp_l1_refill_cacheable/_bufferable/_secure split, C906's
// ICache-facing MMU port has NO separate attribute wires at all -- whatever
// cacheable/bufferable/secure information exists lives inside the 5-bit
// mmu_ifu_prot field, confirmed by icache.v's own port list having no such
// ports.
parameter MMU_VA_WIDTH   = 52;
parameter MMU_PA_WIDTH   = 28;
parameter MMU_PROT_WIDTH = 5;

//-----------------------------------------------------------------------------
// M1: L1 instruction cache geometry (`ICACHE_32K`, IFU notes S3)
//-----------------------------------------------------------------------------
// 32 KB, 2-way, 64 B line, 256 sets -- exactly half of C910's 64KB/512-set
// ICache, same 59-bit tag-row layout and FIFO-bit replacement scheme, half
// the rows. Fetch bandwidth is only 1 word (32 bits = 2 halfwords) per
// cycle -- the 4 data banks exist for REFILL bandwidth (128b/beat AXI), not
// fetch width; a materially different fetch bandwidth from C910's 16B/cycle
// (IFU notes S3, explicitly flagged there as NOT a simplification but a real
// structural difference to clone).
parameter ICACHE_SIZE       = 32768;
parameter ICACHE_WAYS       = 2;
parameter ICACHE_LINE_BYTES = 64;
parameter ICACHE_SETS       = 256;        // = 32KB / 2 ways / 64B
parameter ICACHE_TAG_IDX_W  = 8;          // bytePC[13:6] -> 256 sets

// Data array: 4 banks x 2048x32 SRAMs (aq_spsram_2048x32, confirmed
// icache_data_array.v:245,269,293,317), holding both ways' worth of one 64B
// line across the 4 banks -- way selection folds into the bank index
// (icache_data_idx_high/low), not a separate way dimension.
// OPEN ITEM (IFU notes S10, not resolved by this task): the in-RTL comment
// says "data index: addr[14:2]" (13 bits) but the SRAM depth is 2048 = 2^11;
// the exact index-bit-to-bank mapping is left for the M1 plan's ICache task
// (Task 2) to reconcile against icache_data_array.v:200-241 directly -- the
// constants below are the confirmed SRAM geometry, not a claim that the
// bit-slice question is closed.
parameter ICACHE_DATA_BANKS = 4;
parameter ICACHE_DATA_ROWS  = 2048;
parameter ICACHE_DATA_IDX_W = 11;
parameter ICACHE_DATA_WIDTH = 32;         // one bank row = 1 fetch word

// Tag row, 59 bits (icache.v:742-746), same layout as C910's, half the rows:
//   [58]    fifo         - 1 bit of FIFO replacement state per SET
//   [57]    way1_vld
//   [56:29] way1_ptag[27:0]  = PA[39:12]
//   [28]    way0_vld
//   [27:0]  way0_ptag[27:0] = PA[39:12]
// icache_tag_wen[2:0]: [2]=fifo write-enable (shared per set), [1]=way1,
// [0]=way0 (icache_tag_array.v:98-101) -- 3 bits wide even though it is a
// 2-way array; do not read this as "3-way".
parameter ICACHE_TAG_ROW_WIDTH = 59;
parameter ICACHE_TAG_WAY_WIDTH = 29;      // {valid, ptag[27:0]}
parameter ICACHE_PTAG_WIDTH    = 28;      // PA[39:12]
parameter ICACHE_TAG_WEN_WIDTH = 3;

// Way prediction / low-power tag-hit bypass (`cp0_ifu_iwpe`): IFU notes S10
// flags this as possibly a low-power tag-hit-buffer bypass rather than real
// way prediction (icache.v:566-664, `direct_sel`/`buf_hit_tag`). Default OFF
// for M1 either way -- matches the design doc's deferred-items posture, same
// as rv12 did for C910's `iwpe`. The M1 plan's ICache task (Task 2.1) is
// where the semantics question actually gets resolved and recorded.
parameter ICACHE_IWPE_DEFAULT = 1'b0;

//-----------------------------------------------------------------------------
// M1: branch predictors (BPU extraction notes S0-S6)
//-----------------------------------------------------------------------------
// BHT: single 1024x16 SRAM (aq_spsram_1024x16, confirmed
// aq_ifu_bht_array.v:102), pure global-history index -- NO PC contribution
// at all (`pred_bht_pc`/`iu_ifu_bht_cur_pc` are confirmed DEAD ports in the
// real RTL, grepped with zero hits outside the port list/synthesis-tool
// comments -- rv906 does not clone them; BPU notes S1.4). 8 lanes of 2-bit
// saturating counters per row, selected by 3 GHR bits (one bit shared with
// the row index itself, BPU notes S1.4). 14-bit GHR, two copies
// (architectural `bht_ghr` + speculative `bht_vghr`), NOT C910's
// spec/arch/checkpoint-FIFO triple.
parameter BHT_ROWS       = 1024;
parameter BHT_WIDTH      = 16;
parameter BHT_IDX_W      = 10;
parameter BHT_GHR_WIDTH  = 14;
parameter BHT_LANES      = 8;
parameter BHT_COUNTER_W  = 2;
parameter BHT_INV_CYCLES = 1024;          // invalidate sweep length (BPU S1.7)

// BTB: 16 flop-based fully-associative entries (`aq_ifu_btb_entry` x16,
// confirmed zero SRAM instantiations in either btb file) -- a CAM, NOT an
// SRAM set-associative table like C910's. Tag/target cover only PC[15:0] (a
// real 64KiB aliasing period, cloned as-is per design doc S2.3.3). Round-
// robin FIFO allocation on a miss; in-place replace on a tag hit; no
// confidence/counter field at all (direction comes entirely from BHT).
parameter BTB_ENTRIES      = 16;
parameter BTB_TAG_WIDTH    = 16;          // PC[15:0]
parameter BTB_TARGET_WIDTH = 16;          // PC[15:0]
parameter BTB_ENTRY_WIDTH  = 33;          // {valid, tag, target}

// RAS: 4 flop entries, each just a 24-bit PC -- no valid bit, no privilege
// field, no SRAM (confirmed zero SRAM instantiations). One physical content
// array shared by a speculative pointer (`ras_pop`) and a confirmed pointer
// (`ras_bju`); misprediction resync moves the POINTER only, never entry
// content (BPU notes S3.2) -- correctness is only guaranteed for <=4
// in-flight unresolved call/return predictions, a real shipped limitation
// cloned as-is (design doc S2.1/S2.3/S4.1), not an idealized deeper RAS.
parameter RAS_DEPTH    = 4;
parameter RAS_PC_WIDTH = 24;

// Predictor CP0 chicken bits and invalidate-done handshakes are ports
// directly on ICache.v/BPU.v (mirroring the real RTL's direct CP0 fan-out to
// aq_ifu_icache.v/aq_ifu_bht.v/aq_ifu_btb.v -- NOT routed through
// aq_ifu_ctrl.v, IFU notes S5.1) -- there is deliberately no single
// "predictor enable" struct/constant here.

endpackage
