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
// M2 Task 1: relocated from 64'h9000_1000 to 64'h7FFF_F000 -- inside the
// uncached aperture (<0x8000_0000, "bit31 clear = uncached" convention
// test/m1/uncached.S/common.ld already prove), 60KB past M1_UNCACHED_BASE
// (0x7FFF_0000) to clear uncached.S's own content (peaks at offset 0xBE).
// This is the resolved write-back-DCache/tohost hazard fix (M2 design doc
// S2.3.5/S8, mirroring rv12's proven C910 fix): a tohost store landing on a
// cacheable address could dirty a line without ever reaching the AXI bus
// (ExtMem), hanging the harness's poll loop forever. See also MHCR.wa's
// default-0 reset (CSR.v, Task 2) for the defense-in-depth half of this fix.
// test/m1/common.ld's `.tohost` section moves to match, same task.
parameter [63:0] ADDR_TOHOST     = 64'h0000_0000_7FFF_F000;

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

//=============================================================================
// M2 additions (IDU + IU + RTU + LSU + DCache + MMU + minimal CSR). Sources
// cited inline; see docs/superpowers/specs/notes/2026-08-20-c906-{idu,iu,
// rtu,lsu-base-cp0}-extraction.md and .../specs/2026-08-20-m2-integer-
// design.md. As with M1, every constant is a fact read directly out of
// refs/openc906 (or, for the standard RISC-V CSR addresses, out of the ISA
// spec directly, per the plan's own carve-out) -- never carried over from
// rv12's C910 clone by analogy.
//=============================================================================

//-----------------------------------------------------------------------------
// M2: register/operand-payload widths shared by IDU/IU/LSU/CSR (id_ex1_t,
// see the ID_EX1_* bit-range block below).
//-----------------------------------------------------------------------------
parameter FUNC_WIDTH    = 20;             // aq_idu_cfig.h:302 (FUNC_WIDTH)
parameter GPR_IDX_WIDTH = 6;               // aq_idu_cfig.h DIS_INT_DSTn_REG/
                                            // SRCn_REG width -- 6 bits because
                                            // a vector register number needs a
                                            // bank bit (IDU note S3.4); M2's
                                            // integer-only decode only ever
                                            // populates the low 5.

//-----------------------------------------------------------------------------
// M2: DCache geometry (contract 9 / design doc S2.2/S4.1, LSU note A3+
// cross-cutting #3). M2 omits C906's real VIPT-with-alias-detection second
// tag bank entirely (safe under an identity-map MMU where VA[12]==PA[12]
// always) -- a single 128-set x 4-way group, PA[39:12] tag. No alias-bank
// constants are declared; DCache.v's Task 6 body must still budget tag-row
// bits for a future valid/tag pair per way so M4 can add the second bank
// without a full re-derivation.
//-----------------------------------------------------------------------------
parameter DCACHE_SIZE       = 32768;       // 32KB
parameter DCACHE_WAYS       = 4;
parameter DCACHE_LINE_BYTES = 64;
parameter DCACHE_SETS       = 128;         // = 32KB / 4 ways / 64B
parameter DCACHE_TAG_WIDTH  = 28;          // PA[39:12]
parameter DCACHE_INDEX_W    = 7;           // log2(128 sets) -- PA[11:6], no
                                            // VA[12] alias bit (contract 9)

//-----------------------------------------------------------------------------
// M2: standard RISC-V M-mode CSR addresses (privileged ISA spec, not donor-
// specific -- safe to pin directly per the plan's own carve-out).
//-----------------------------------------------------------------------------
parameter [11:0] CSR_MSTATUS   = 12'h300;
parameter [11:0] CSR_MISA      = 12'h301;
parameter [11:0] CSR_MIE       = 12'h304;
parameter [11:0] CSR_MTVEC     = 12'h305;
parameter [11:0] CSR_MSCRATCH  = 12'h340;
parameter [11:0] CSR_MEPC      = 12'h341;
parameter [11:0] CSR_MCAUSE    = 12'h342;
parameter [11:0] CSR_MTVAL     = 12'h343;
parameter [11:0] CSR_MIP       = 12'h344;
parameter [11:0] CSR_MCYCLE    = 12'hB00;
parameter [11:0] CSR_MINSTRET  = 12'hB02;
parameter [11:0] CSR_MVENDORID = 12'hF11;
parameter [11:0] CSR_MARCHID   = 12'hF12;
parameter [11:0] CSR_MIMPID    = 12'hF13;
parameter [11:0] CSR_MHARTID   = 12'hF14;

//-----------------------------------------------------------------------------
// M2: custom T-Head CSR addresses -- CONFIRMED directly from
// refs/openc906/C906_RTL_FACTORY/gen_rtl/cp0/rtl/aq_cp0_regs.v:846-847
// (`parameter MXSTATUS = 12'h7C0; parameter MHCR = 12'h7C1;`), per the
// plan's explicit instruction NOT to assume rv12's C910 numbers transfer.
//-----------------------------------------------------------------------------
parameter [11:0] CSR_MXSTATUS = 12'h7C0;
parameter [11:0] CSR_MHCR     = 12'h7C1;

// MHCR bit positions -- confirmed aq_cp0_ext_csr.v:670,694,696-725: the RHS
// concat list of `mhcr_value[63:0] = {45'b0, sck[2:0], 3'b0, l0btbe, 3'b0,
// wbr, ibpe, btbe, bpe, rse, wb, wa, de, ie}` gives (LSB = last-listed term)
// ie=bit0, de=bit1, wa=bit2, wb=bit3, rse=bit4, bpe=bit5, btbe=bit6,
// ibpe=bit7, wbr=bit8. ie/de/wa/rse/bpe reset to 0 together (ext_csr.v:
// 696-704, the `mhcr_local_en` always block); btbe resets to 0 separately
// (ext_csr.v:674-680, its own always block). wb/wbr are NOT flops at all --
// hardwired 1 (`assign wb = 1'b1;` line 694, `assign wbr = 1'b1;` line 670),
// i.e. read-only.
parameter MHCR_IE_BIT   = 0;    // icache-en
parameter MHCR_DE_BIT   = 1;    // dcache-en
parameter MHCR_WA_BIT   = 2;    // write-allocate; M2 default 0 (contract 6,
                                 // defense-in-depth half of the tohost fix)
parameter MHCR_WB_BIT   = 3;    // write-back; hardwired 1, read-only
parameter MHCR_RSE_BIT  = 4;
parameter MHCR_BPE_BIT  = 5;
parameter MHCR_BTBE_BIT = 6;
parameter MHCR_WBR_BIT  = 8;    // hardwired 1, read-only

// MXSTATUS.mm bit position -- confirmed aq_cp0_ext_csr.v:643-646:
// `mxstatus_value[63:0] = {32'b0, regs_pm[1:0], 7'b0, cskyisaee, maee,
// fccee, insde, mhrd, clintee, ucme, mm, pmp4k, pmdm, 1'b0, pmds, pmdu, v,
// ve, 8'b0}` places `mm` 15 bits up from the LSB (counting the trailing
// 8'b0/ve/v/pmdu/pmds/1'b0/pmdm/pmp4k terms below it) -> bit 15. Resets to 1
// (ext_csr.v:550-556), matching design doc S2.3.3's cited reset value.
parameter MXSTATUS_MM = 15;

//-----------------------------------------------------------------------------
// M2: integer execute-unit one-hot select (IDU note S7, EU_WIDTH=10 per
// aq_idu_cfig.h:100; individual patterns/bit positions confirmed directly
// against cfig.h:105-153, not inferred).
//-----------------------------------------------------------------------------
parameter EU_WIDTH = 10;

parameter [EU_WIDTH-1:0] EU_ALU  = 10'b0000000001;   // cfig.h:105
parameter [EU_WIDTH-1:0] EU_BJU  = 10'b0000000010;   // cfig.h:106
parameter [EU_WIDTH-1:0] EU_MULT = 10'b0000000100;   // cfig.h:107
parameter [EU_WIDTH-1:0] EU_DIV  = 10'b0000001000;   // cfig.h:108
parameter [EU_WIDTH-1:0] EU_CP0  = 10'b0000010000;   // cfig.h:109
parameter [EU_WIDTH-1:0] EU_LSU  = 10'b0000100000;   // cfig.h:110

parameter EU_ALU_SEL  = 0;    // cfig.h:112
parameter EU_BJU_SEL  = 1;    // cfig.h:113
parameter EU_MULT_SEL = 2;    // cfig.h:114
parameter EU_DIV_SEL  = 3;    // cfig.h:115
parameter EU_CP0_SEL  = 4;    // cfig.h:116
parameter EU_LSU_SEL  = 5;    // cfig.h:117

// The remaining 3 EU_WIDTH bit POSITIONS the donor uses for FP/VEC group
// tagging (EU_VGROUP_SEL=7 cfig.h:137, EU_FP_SEL=8 cfig.h:122, EU_VEC_SEL=9
// cfig.h:136 -- bit 6 is not separately named in the donor's own SEL list)
// are recorded here for traceability but are DECLARED-YET-PERMANENTLY-
// UNREACHABLE in M2's decode: FP is out of scope until M5, vector decode is
// dead in the donor build itself and stays "never" per the parent design's
// milestone table, and LR/SC/AMO*/FP/vector all decode to illegal for M2
// (contract 10). No EU_FP/EU_VEC one-hot pattern constants are declared --
// the donor's own FP/VEC encodings are denser 2-MSB-tag multi-bit patterns
// this package has no consumer for.
parameter EU_VGROUP_SEL = 7;
parameter EU_FP_SEL     = 8;
parameter EU_VEC_SEL    = 9;

//-----------------------------------------------------------------------------
// M2: WBT producer-type tag (IDU note S5.1, aq_idu_cfig.h:168-172).
//-----------------------------------------------------------------------------
parameter [2:0] WB_INT_TYPE_OTHER = 3'd0;
parameter [2:0] WB_INT_TYPE_ALU   = 3'd1;
parameter [2:0] WB_INT_TYPE_BJU   = 3'd2;
parameter [2:0] WB_INT_TYPE_MULT  = 3'd3;
parameter [2:0] WB_INT_TYPE_LSU   = 3'd4;
// TODO(Task 3): whether DIV reuses WB_INT_TYPE_MULT's tag or needs its own
// 5th value is an open item (design doc S8; IDU note S5.1/S10 confirms the
// donor's WBT only enumerates OTHER/ALU/BJU/MULT/LSU, no DIV entry). IU.v's
// real body (Task 3.6) resolves this by reading aq_idu_id_wbt.v/
// aq_iu_top.v/aq_idu_id_wbt.v directly and records the finding here -- do
// not add a 5th value on a guess.

//-----------------------------------------------------------------------------
// M2 Task 2: CP0/EU_CP0 FUNC one-hot values (aq_idu_cfig.h:453-474, the "CP0
// Decoder" section). These are the exact bit patterns the donor's decoder
// produces for the CSRRW/S/C(+I)/ECALL/EBREAK/MRET/FENCE/FENCE.I encodings
// -- read directly out of the donor's IDU config header (not the CP0 files
// CSR.v's own header cites) because Task 5 (IDU's real decode) has not run
// yet and CSR.v (Task 2) needs a pinned, donor-faithful value to dispatch
// against NOW. Pinned here (rather than left for Task 5) so Task 5's real
// decode has a fixed target to hit instead of inventing its own encoding
// after the fact. Every value below is `{FUNC_WIDTH-10{1'b0}}` zero-extended
// from the donor's own 10-bit literal -- confirmed bit-exact, not
// re-derived. SRET/WFI/DRET/SFENCE/SYNC/SYNCI/CACHE/VSETVL/VSETVLI (the
// donor's other EU_CP0 sub-ops) are deliberately NOT pinned: M2's IDU never
// emits them (contracts 2.2/2.3.4/10), so they are DECLARED-YET-
// PERMANENTLY-UNREACHABLE the same way EU_VGROUP_SEL/EU_FP_SEL/EU_VEC_SEL
// are above -- no constant is declared for them at all.
//-----------------------------------------------------------------------------
parameter [FUNC_WIDTH-1:0] CP0_FUNC_ECALL   = 20'h00012;  // cfig.h:455
parameter [FUNC_WIDTH-1:0] CP0_FUNC_EBREAK  = 20'h00022;  // cfig.h:456
parameter [FUNC_WIDTH-1:0] CP0_FUNC_MRET    = 20'h00042;  // cfig.h:457
parameter [FUNC_WIDTH-1:0] CP0_FUNC_FENCE   = 20'h00028;  // cfig.h:461
parameter [FUNC_WIDTH-1:0] CP0_FUNC_FENCEI  = 20'h00024;  // cfig.h:462
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRW   = 20'h00011;  // cfig.h:469
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRS   = 20'h00021;  // cfig.h:470
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRC   = 20'h00041;  // cfig.h:471
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRWI  = 20'h00211;  // cfig.h:472
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRSI  = 20'h00221;  // cfig.h:473
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRCI  = 20'h00241;  // cfig.h:474

//-----------------------------------------------------------------------------
// M2: id_ex1_t -- IDU's single shared EX1 payload (design doc S4.2),
// field-sliced per consumer (IU/LSU/CSR each take their own view) rather
// than four separate structs routed to four destination registers (matches
// C906's own "one shared EX1 register" structure, IDU note S7). Per umbrella
// S6.1 rule 1 ("packed struct only inside a module, never on a port") /
// S6.2 rule 8 ("cross-module buses are flat concatenated vectors with
// widths in rvproc_pkg.sv... no magic slicing at use sites"), the payload
// is pinned here as a flat-vector bit range, one `define per field, rather
// than a struct type.
//
// Confirmed against the REAL donor register split (read directly, not
// deferred to Task 5 per the plan's explicit instruction):
//   - aq_idu_id_dp.v:934-1064: the 311b `ex1_int_inst_data[310:0]` operand/
//     func/dst payload register (four separately-clock-gated slices: the
//     bulk register + one gated slice per src0/1/2 for the EX1-resident
//     late-forward path, dp.v:805-891,942-997).
//   - aq_idu_cfig.h:239-264 (`DIS_INT_*` bit-position defines): chaining
//     the offsets from `DIS_INT_SRC0_DATA=63` up through
//     `DIS_INT_HINFO_MSB=DIS_INT_OPCODE+TDT_HINFO_WIDTH` and
//     `DIS_INT_WIDTH=DIS_INT_HINFO_MSB+1` gives EXACTLY 311 bits (with
//     TDT_HINFO_WIDTH=22, confirmed via dtu/rtl/aq_dtu_cfig.h + the
//     TDT_TM_MCONTROL_TRI_NUM=8/TDT_TM_OTHER_TRI_NUM=2 config selectors in
//     cpu/rtl/cpu_cfig.h:477-496) -- this is the "311-bit payload" the
//     design doc's S4.1 unit graph cites.
//   - aq_idu_id_ctrl.v:598-616: the EU one-hot select, `ex1_eu_sel
//     [EU_WIDTH-1:0]` (10b), is a SEPARATE register from the 311b bus above
//     (alongside `ex1_inst_vld`), not one of its fields.
//
// rv906's OWN id_ex1_t is NOT a byte-for-byte clone of the donor's 311b
// bus: it carries only the fields the design doc names (func, EU one-hot,
// src0/1/2 data+ready, dst0/1 reg, imm, illegal, PC). Dropped, relative to
// the donor: HINFO/raw OPCODE/VSEW/VLMUL/BHT_PRED/EXPT_ACC-PAGE-HIGH-ILLE/
// SPLIT/LENGTH -- all vector-, FP-, or debug-trigger-adjacent fields with
// no M2 consumer (contract 10; FP is M5, vector/HAD are "never"/M7).
// Added, relative to the donor: a PC field -- real C906 does NOT carry PC
// through this register at all; BJU instead tracks its own redundant PC
// copy (`bju_pcgen_pc`, IU note S4.5) fed by IFU's chgflw port + CP0's
// mrvbr. rv906 chooses to carry PC through the shared id_ex1_t payload
// instead (a legitimate from-scratch design choice, Task 5 implements the
// consequences). Per-field WIDTHS below, for every field shared with the
// donor, are the confirmed donor widths (not independently re-derived):
//   func        20b (FUNC_WIDTH)
//   eu          10b (EU_WIDTH, one-hot)
//   src0/1/2 data  64b each (XLEN; DIS_INT_SRCn_DATA)
//   src0/1/2 ready  1b each (DIS_INT_SRCn_RDY)
//   dst0/1 reg      6b each (GPR_IDX_WIDTH; DIS_INT_DSTn_REG)
//   imm            64b (rv906's own field, sized to the donor's src1_imm/
//                  src2_imm generators, IDU note S4 -- unlike the donor,
//                  which pre-merges the selected immediate into src1_data/
//                  src2_data at the ID/DIS stage before the EX1 register,
//                  rv906 carries it through as its own field so each
//                  consumer's operand-prepare mux decides src-data-vs-imm
//                  itself; Task 5 implements the mux)
//   illegal         1b (IDU_ILLEGAL)
//   pc         PC_WIDTH b (40b, M2's own addition, see above)
//-----------------------------------------------------------------------------
`define ID_EX1_WIDTH            342

`define ID_EX1_FUNC_HI          341
`define ID_EX1_FUNC_LO          322
`define ID_EX1_EU_HI            321
`define ID_EX1_EU_LO            312
`define ID_EX1_SRC0_DATA_HI     311
`define ID_EX1_SRC0_DATA_LO     248
`define ID_EX1_SRC1_DATA_HI     247
`define ID_EX1_SRC1_DATA_LO     184
`define ID_EX1_SRC2_DATA_HI     183
`define ID_EX1_SRC2_DATA_LO     120
`define ID_EX1_IMM_HI           119
`define ID_EX1_IMM_LO           56
`define ID_EX1_SRC0_RDY         55
`define ID_EX1_SRC1_RDY         54
`define ID_EX1_SRC2_RDY         53
`define ID_EX1_DST0_REG_HI      52
`define ID_EX1_DST0_REG_LO      47
`define ID_EX1_DST1_REG_HI      46
`define ID_EX1_DST1_REG_LO      41
`define ID_EX1_PC_HI            40
`define ID_EX1_PC_LO            1
`define ID_EX1_ILLEGAL          0

endpackage
