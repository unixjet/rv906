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

// M4 Task 4: TLB geometry (design doc D11 -- a single 128-entry
// fully-associative flop array replaces C906's uTLB+jTLB two-level
// structure; donor aq_mmu_jtlb.v tag/data words transcribed for the field
// shapes, entry count/replacement per D11's own wording).
parameter MMU_TLB_ENTRIES = 128;
parameter MMU_VPN_WIDTH   = 27;   // Sv39 VA[38:12]
parameter MMU_ASID_WIDTH  = 16;
parameter MMU_PGS_WIDTH   = 3;    // one-hot {1G,2M,4K}
parameter MMU_FLG_WIDTH   = 12;   // {pma[4:0],D,A,U,X,W,R,V} -- PMP is
                                   // live-rechecked per access (not cached
                                   // in the TLB entry), so donor's 4-bit
                                   // PMP slice is dropped from the cached
                                   // word (rv12 MMU_FLG_W precedent: same
                                   // 12-bit shape, same reasoning).

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
// TASK 6 DISCOVERED GAP, FIXED HERE (same "documented amendment, not silent"
// discipline as CSR.v Task 2's/RTU.v Task 4's/IDU.v Task 5's own "discovered
// gap" notes): the M2-Task-1-pinned values below (`DCACHE_TAG_WIDTH=28` with
// the comment "PA[39:12]", `DCACHE_INDEX_W=7` with the comment "PA[11:6]")
// cannot be simultaneously true -- PA[11:6] is only 6 bits, not 7, and
// 28(tag)+7(index)+6(line-offset, 64B line) = 41 bits, one more than the
// 40-bit PA (PC_WIDTH=40) this whole design uses everywhere else. This is a
// carried-over consequence of the donor's OWN 128-"set" figure actually
// counting BOTH VIPT-alias-bank halves together (LSU note A3: "index =
// {VA[12] (alias bit), PA[11:6]} (7 bits)" -- each physical bank is only
// 64 rows deep, addressed by the 6-bit PA[11:6]; VA[12] selects the bank,
// not an extra index bit within one bank). Contract 9 already resolved M2
// to a SINGLE 128-set x 4-way group with NO alias bank -- meaning M2's own
// single group genuinely needs a full 7-bit index living entirely inside
// the PA (no VA[12] bank-select bit to borrow), which only balances against
// a 40-bit PA and a 6-bit (64B) line offset if the tag is 27 bits, not 28:
// 27(tag)+7(index)+6(offset)=40, exact. Fixed here, before Task 6's DCache.v
// becomes the first real consumer of either constant (confirmed via grep:
// no other committed file referenced either constant before this commit) --
// tag = PA[39:13] (27 bits), index = PA[12:6] (7 bits), line offset =
// PA[5:0] (6 bits).
parameter DCACHE_TAG_WIDTH  = 27;          // PA[39:13] (Task 6 fix, was 28/PA[39:12])
parameter DCACHE_INDEX_W    = 7;           // PA[12:6] (Task 6 fix, was documented as
                                            // PA[11:6]) -- no VA[12] alias bit (contract 9)

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
// MHINT -- confirmed aq_cp0_regs.v:851 (`parameter MHINT = 12'h7C5;`),
// M3b Task D (PFB stride prefetch controls live here).
parameter [11:0] CSR_MHINT    = 12'h7C5;

// M4 Task 2: PMP CSR addresses (privileged ISA standard, confirmed donor
// aq_pmp_top.v:82-99). pmpcfg0 (entries 0-7 cfg bytes), pmpcfg2 (entries 8-15,
// reads 0 / writes ignored in the 8-entry config), pmpaddr0-7.
parameter [11:0] CSR_PMPCFG0   = 12'h3A0;
parameter [11:0] CSR_PMPCFG2   = 12'h3A2;
parameter [11:0] CSR_PMPADDR0  = 12'h3B0;   // pmpaddr0..7 = 0x3B0..0x3B7

//-----------------------------------------------------------------------------
// M4: privilege (M/S/U) CSR addresses -- standard RISC-V privileged ISA
// (S-mode bank + delegation + counters + satp), pinned per the M4 design doc
// §2.1 S8. Debug-trigger CSRs (T-Head aq_cp0_regs.v:826-831).
//-----------------------------------------------------------------------------
// S-mode bank
parameter [11:0] CSR_SSTATUS    = 12'h100;
parameter [11:0] CSR_SIE        = 12'h104;
parameter [11:0] CSR_STVEC      = 12'h105;
parameter [11:0] CSR_SCOUNTEREN = 12'h106;
parameter [11:0] CSR_SSCRATCH   = 12'h140;
parameter [11:0] CSR_SEPC       = 12'h141;
parameter [11:0] CSR_SCAUSE     = 12'h142;
parameter [11:0] CSR_STVAL      = 12'h143;
parameter [11:0] CSR_SIP        = 12'h144;
parameter [11:0] CSR_SATP       = 12'h180;
// M-mode delegation + counter-enable
parameter [11:0] CSR_MEDELEG    = 12'h302;
parameter [11:0] CSR_MIDELEG    = 12'h303;
parameter [11:0] CSR_MCOUNTEREN = 12'h306;
// User read-only counter aliases (Zicntr)
parameter [11:0] CSR_CYCLE      = 12'hC00;
parameter [11:0] CSR_TIME       = 12'hC01;   // storage deferred to M6 (D-M4-9)
parameter [11:0] CSR_INSTRET    = 12'hC02;
// Debug triggers (M4: zero-trigger escape hatch, D-M4-5; real triggers M7)
parameter [11:0] CSR_TSELECT    = 12'h7A0;
parameter [11:0] CSR_TDATA1     = 12'h7A1;
parameter [11:0] CSR_TDATA2     = 12'h7A2;
parameter [11:0] CSR_TDATA3     = 12'h7A3;
parameter [11:0] CSR_TCONTROL   = 12'h7A5;

// M5 Task 1: user-level floating-point CSRs (standard RISC-V unprivileged
// ISA addresses; donor confirms the same numbers, aq_cp0_regs.v FFLAGS/FRM/
// FCSR case arms).
parameter [11:0] CSR_FFLAGS     = 12'h001;
parameter [11:0] CSR_FRM        = 12'h002;
parameter [11:0] CSR_FCSR       = 12'h003;

// mstatus / sstatus bit positions (privileged ISA standard layout; donor
// aq_cp0_trap_csr.v:486-494). RV64: SD=63, MBE/SBE=37/36 (tied 0), SXL/UXL
// =35:34/33:32 (RO 2'b10), TSR=22, TW=21, TVM=20, MXR=19, SUM=18, MPRV=17,
// XS=16:15, FS=14:13, MPP=12:11, VS=10:9 (tied 0), SPP=8, MPIE=7, UBE=6
// (tied 0), SPIE=5, MIE=3, UPIE=4 (tied 0), SIE=1.
parameter MSTATUS_SD_BIT    = 63;
parameter MSTATUS_TSR_BIT   = 22;
parameter MSTATUS_TW_BIT    = 21;
parameter MSTATUS_TVM_BIT   = 20;
parameter MSTATUS_MXR_BIT   = 19;
parameter MSTATUS_SUM_BIT   = 18;
parameter MSTATUS_MPRV_BIT  = 17;
parameter MSTATUS_FS_HI     = 14;   // FS[1:0] = [14:13]
parameter MSTATUS_FS_LO     = 13;
parameter MSTATUS_MPP_HI    = 12;   // MPP[1:0] = [12:11]
parameter MSTATUS_MPP_LO    = 11;
parameter MSTATUS_SPP_BIT   = 8;
parameter MSTATUS_MPIE_BIT  = 7;
parameter MSTATUS_SPIE_BIT  = 5;
parameter MSTATUS_MIE_BIT   = 3;
parameter MSTATUS_SIE_BIT   = 1;

// Privilege levels (aq_cp0_trap_csr.v pm encoding: U=00, S=01, M=11).
parameter [1:0] PRIV_U = 2'b00;
parameter [1:0] PRIV_S = 2'b01;
parameter [1:0] PRIV_M = 2'b11;

// Exception cause codes used by the M4 trap/privilege logic.
parameter CAUSE_MISALIGNED_FETCH = 5'd0;
parameter CAUSE_FETCH_ACCESS     = 5'd1;
parameter CAUSE_ILLEGAL          = 5'd2;
parameter CAUSE_BREAKPOINT       = 5'd3;
parameter CAUSE_MISALIGNED_LOAD  = 5'd4;
parameter CAUSE_LOAD_ACCESS      = 5'd5;
parameter CAUSE_MISALIGNED_STORE = 5'd6;
parameter CAUSE_STORE_ACCESS     = 5'd7;
parameter CAUSE_USER_ECALL       = 5'd8;
parameter CAUSE_SUPERVISOR_ECALL = 5'd9;
parameter CAUSE_MACHINE_ECALL    = 5'd11;
parameter CAUSE_FETCH_PAGE_FAULT = 5'd12;
parameter CAUSE_LOAD_PAGE_FAULT  = 5'd13;
parameter CAUSE_STORE_PAGE_FAULT = 5'd15;

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
// RESOLVED (Task 3.6b): DIV does NOT reuse WB_INT_TYPE_MULT's tag, and no
// 5th value is needed. Confirmed directly in the donor's real producer-type
// mux, aq_idu_id_dp.v:566-570:
//   dp_wb_dst0_type[2:0] =
//       {3{..._EU_ALU_SEL}}  & WB_INT_TYPE_ALU
//     | {3{..._EU_BJU_SEL}}  & WB_INT_TYPE_BJU
//     | {3{..._EU_MULT_SEL}} & WB_INT_TYPE_MULT
//     | {3{..._EU_LSU_SEL}}  & WB_INT_TYPE_LSU;
// -- an OR-mux with exactly four one-hot terms (ALU/BJU/MULT/LSU); there is
// NO `EU_DIV_SEL` term anywhere in it. A DIV-dispatched instruction
// therefore produces dp_wb_dst0_type == 3'b000 == WB_INT_TYPE_OTHER, the
// same "no special type" default every non-ALU/BJU/MULT/LSU EU (CP0/CSR
// included) falls into. Cross-checked against the consumer side,
// aq_idu_id_ctrl.v:513-530: the RAW/WAW "except" clauses only special-case
// `dst0_type == WB_INT_TYPE_LSU` and `== WB_INT_TYPE_MULT` -- DIV (tagged
// OTHER) matches neither, so a DIV producer's dependent consumer gets the
// generic (non-excepted) scoreboard stall treatment, NOT MULT's exception.
// IDU's Task 5 WBT scoreboard must cite this finding rather than guessing
// DIV shares MULT's fast-path exemption.

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
parameter [FUNC_WIDTH-1:0] CP0_FUNC_SRET    = 20'h00082;  // cfig.h:458 (M4)
parameter [FUNC_WIDTH-1:0] CP0_FUNC_WFI     = 20'h00102;  // cfig.h:459 (M4)
parameter [FUNC_WIDTH-1:0] CP0_FUNC_FENCE   = 20'h00028;  // cfig.h:461
parameter [FUNC_WIDTH-1:0] CP0_FUNC_FENCEI  = 20'h00024;  // cfig.h:462
parameter [FUNC_WIDTH-1:0] CP0_FUNC_SFENCE  = 20'h00044;  // cfig.h:463 (M4)
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRW   = 20'h00011;  // cfig.h:469
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRS   = 20'h00021;  // cfig.h:470
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRC   = 20'h00041;  // cfig.h:471
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRWI  = 20'h00211;  // cfig.h:472
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRSI  = 20'h00221;  // cfig.h:473
parameter [FUNC_WIDTH-1:0] CP0_FUNC_CSRRCI  = 20'h00241;  // cfig.h:474

//-----------------------------------------------------------------------------
// M2 Task 3.6a: idu_iu_ex1_func bit-per-opcode table for ALU/BJU/MULT/DIV --
// RESOLVES the design doc S8 open item. Cross-referenced two sources, both
// read directly (never from memory, per the plan's traceability rule):
//   1. aq_idu_id_decd.v's main casez table (casez({x_inst[31:25],
//      x_inst[14:12],x_inst[6:2]}), decd.v:1541-2169) -- confirms WHICH
//      `FUNC_*` literal the donor's real decoder emits per RV64I/M opcode
//      (e.g. decd.v:1856-1858 "add" -> EU_ALU/FUNC_ADD; decd.v:1569-1571
//      "beq" -> EU_BJU/FUNC_BEQ; decd.v:1926-1928 "mul" -> EU_MULT/FUNC_MUL;
//      decd.v:1954-1956 "div" -> EU_DIV/FUNC_DIV) and the custom-opcode
//      "perf" table (decd.v:3187-3325) for the XThead ALU ops this unit
//      implements (srri/srriw/tstnbz/rev/ff0/ff1/tst/revw/mveqz/mvnez/ext/
//      extu, decd.v:3197-3324).
//   2. aq_idu_cfig.h's own `FUNC_*` literal defines (cfig.h:321-410,
//      415-422) -- the actual bit patterns, confirmed bit-exact against
//      IU's own consumer-side op-group tests (IU note S12/S13, S2/S4/S5/S6):
//      bit0=adder-group/mul-inst64/div-word selects (per-unit meaning
//      differs, since func[19:0] is a SHARED bus gated by each unit's own
//      *_sel), bit1=shifter-group, bit2=logic-group/mul-inst32, bit3=misc-
//      group/mul-inst16, bit6=BJU-conditional-branch-flag (FUNC_CONDBR_SEL
//      per cfig.h:312, confirmed matching every BEQ/BNE/BLT/BGE/BLTU/BGEU
//      value below), bit7=BJU-AUIPC-flag (FUNC_AUIPC_SEL per cfig.h:313).
// rv906's IU.v (Task 3 real body) does NOT re-derive C906's internal onehot
// operand-prepare bit games from these values (that mechanism is IU note
// S2's prose, not a bit-for-bit port requirement) -- it decodes by exact
// equality against these pinned constants, the same style CSR.v (Task 2)
// already established for CP0_FUNC_*. Immediate-vs-register instruction
// forms sharing one mnemonic (ADD/ADDI, SLL/SLLI, BEQ has no immediate
// form, etc.) share ONE constant here, exactly as the donor's own decode
// table does (e.g. decd.v:1730 "addi" and decd.v:1856 "add" both assign
// `FUNC_ADD` -- IDU's Task 5 decode picks which operand (register or
// sign-extended immediate) lands in src1_data, not a different func value).
//-----------------------------------------------------------------------------
// ALU (IU note S2; alu.v:187,287,572,594 confirm func[0]=adder,[1]=shifter,
// [2]=logic,[3]=misc as the op-group select, matched here by exact value):
parameter [FUNC_WIDTH-1:0] ALU_FUNC_LUI    = 20'h41401;  // cfig.h:321
parameter [FUNC_WIDTH-1:0] ALU_FUNC_ADD    = 20'h60401;  // cfig.h:322 (ADD/ADDI/ADDDI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_ADDW   = 20'h08441;  // cfig.h:325 (ADDW/ADDIW)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SUB    = 20'h60601;  // cfig.h:327
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SUBW   = 20'h08641;  // cfig.h:328
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SLT    = 20'h60e81;  // cfig.h:329 (SLT/SLTI, signed)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SLTU   = 20'h04a81;  // cfig.h:331 (SLTU/SLTIU, unsigned)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SLL    = 20'h10022;  // cfig.h:345 (SLL/SLLI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SLLW   = 20'h10062;  // cfig.h:347 (SLLW/SLLIW)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SRL    = 20'h18802;  // cfig.h:349 (SRL/SRLI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SRLW   = 20'h14842;  // cfig.h:351 (SRLW/SRLIW)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SRA    = 20'h08082;  // cfig.h:353 (SRA/SRAI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SRAW   = 20'h010c2;  // cfig.h:355 (SRAW/SRAIW)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SRRI   = 20'h08102;  // cfig.h:357 (XThead rotate)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_SRRIW  = 20'h02142;  // cfig.h:358
parameter [FUNC_WIDTH-1:0] ALU_FUNC_EXT    = 20'h18602;  // cfig.h:359 (XThead sign bitfield extract)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_EXTU   = 20'h18202;  // cfig.h:360 (XThead zero bitfield extract)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_AND    = 20'h00024;  // cfig.h:365 (AND/ANDI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_XOR    = 20'h00044;  // cfig.h:367 (XOR/XORI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_OR     = 20'h00084;  // cfig.h:369 (OR/ORI)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_FF0    = 20'h00608;  // cfig.h:375 (find-first-0)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_FF1    = 20'h00208;  // cfig.h:376 (find-first-1)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_REV    = 20'h00048;  // cfig.h:377 (byte-reverse, 64b)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_REVW   = 20'h00028;  // cfig.h:378 (byte-reverse, 32b)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_TST    = 20'h00108;  // cfig.h:379 (single-bit test)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_TSTNBZ = 20'h00088;  // cfig.h:380 (per-byte nonzero test)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_MVEQZ  = 20'h00c08;  // cfig.h:381 (conditional move if ==0)
parameter [FUNC_WIDTH-1:0] ALU_FUNC_MVNEZ  = 20'h00808;  // cfig.h:382 (conditional move if !=0)
// No ALU_FUNC_MAX/MIN/MAXU/MINU/ADDSL constants -- see IU.v's header for the
// MAX/MIN-vs-ADDSL nuance this task's own re-verification found (MAX/MIN
// genuinely unreachable from decode; ADDSL is decode-reachable and its
// adder-side mechanism is NOT dead in alu.v, but is still out of M2's own
// ISA target per the design doc's explicit ALU op enumeration -- not ported
// either way, for a more precise reason than "dead in this C906 build").

// BJU (IU note S4; bju.v:611-612 confirm func[6]=conditional-branch-flag,
// func[1]=jalr-within-uncond-group, func[7]=auipc, matched here by value):
parameter [FUNC_WIDTH-1:0] BJU_FUNC_BEQ    = 20'h00144;  // cfig.h:404
parameter [FUNC_WIDTH-1:0] BJU_FUNC_BNE    = 20'h0014c;  // cfig.h:405
parameter [FUNC_WIDTH-1:0] BJU_FUNC_BLT    = 20'h00152;  // cfig.h:406 (signed)
parameter [FUNC_WIDTH-1:0] BJU_FUNC_BGE    = 20'h0015a;  // cfig.h:407 (signed)
parameter [FUNC_WIDTH-1:0] BJU_FUNC_BLTU   = 20'h00142;  // cfig.h:408 (unsigned)
parameter [FUNC_WIDTH-1:0] BJU_FUNC_BGEU   = 20'h0014a;  // cfig.h:409 (unsigned)
parameter [FUNC_WIDTH-1:0] BJU_FUNC_JAL    = 20'h00921;  // cfig.h:402
parameter [FUNC_WIDTH-1:0] BJU_FUNC_JALR   = 20'h00822;  // cfig.h:403
parameter [FUNC_WIDTH-1:0] BJU_FUNC_AUIPC  = 20'h00980;  // cfig.h:410 (rides BJU's wb bus, IU note S2/S3)

// MULT (IU note S5; mul.v:227-235 confirm bit0=inst64,bit1=inst32,
// bit2=inst16 width class -- MULW's decode-table value sets bit1, matching
// "32x32" width; matched here by value, not by re-deriving the width bits):
parameter [FUNC_WIDTH-1:0] MULT_FUNC_MUL    = 20'h00021;  // cfig.h:387
parameter [FUNC_WIDTH-1:0] MULT_FUNC_MULW   = 20'h00022;  // cfig.h:388
parameter [FUNC_WIDTH-1:0] MULT_FUNC_MULH   = 20'h00121;  // cfig.h:389 (signed x signed)
parameter [FUNC_WIDTH-1:0] MULT_FUNC_MULHU  = 20'h00181;  // cfig.h:390 (unsigned x unsigned)
parameter [FUNC_WIDTH-1:0] MULT_FUNC_MULHSU = 20'h00141;  // cfig.h:391 (signed x unsigned)

// DIV (IU note S6; div.v:191-193 confirm func[0]=word,[1]=quotient-select,
// [2]=signed -- DIV/DIVU/REM/REMU share one core, div_res_sel_quotient just
// picks which result rides the bus, matched here by value):
parameter [FUNC_WIDTH-1:0] DIV_FUNC_DIV    = 20'h00006;  // cfig.h:415 (signed quotient)
parameter [FUNC_WIDTH-1:0] DIV_FUNC_DIVU   = 20'h00002;  // cfig.h:416 (unsigned quotient)
parameter [FUNC_WIDTH-1:0] DIV_FUNC_DIVW   = 20'h00007;  // cfig.h:417
parameter [FUNC_WIDTH-1:0] DIV_FUNC_DIVUW  = 20'h00003;  // cfig.h:418
parameter [FUNC_WIDTH-1:0] DIV_FUNC_REM    = 20'h00004;  // cfig.h:419 (signed remainder)
parameter [FUNC_WIDTH-1:0] DIV_FUNC_REMU   = 20'h00000;  // cfig.h:420 (unsigned remainder)
parameter [FUNC_WIDTH-1:0] DIV_FUNC_REMW   = 20'h00005;  // cfig.h:421
parameter [FUNC_WIDTH-1:0] DIV_FUNC_REMUW  = 20'h00001;  // cfig.h:422

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

//-----------------------------------------------------------------------------
// M2 Task 5: EU_LSU FUNC one-hot values (aq_idu_cfig.h:503-516, the load/
// store FUNC table) -- not pinned by Task 2/3 (CSR.v/IU.v never dispatch to
// EU_LSU), needed now because IDU's real decode (this task) is the first
// body to actually produce an idu_lsu_ex1_func value. Confirmed bit-exact
// against aq_idu_cfig.h's own 12-bit literals (zero-extended to
// FUNC_WIDTH), read directly during Task 5's decode-table extraction, same
// discipline as Task 3.6a's ALU/BJU/MULT/DIV table: bit0 = FUNC_STORE_SEL
// (cfig.h:310) -- 1 for every store, 0 for every load, confirmed below.
//-----------------------------------------------------------------------------
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LB  = 20'h00302;  // cfig.h:503
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LH  = 20'h00306;  // cfig.h:504
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LW  = 20'h0030a;  // cfig.h:505
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LD  = 20'h0030e;  // cfig.h:506
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LBU = 20'h00300;  // cfig.h:507
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LHU = 20'h00304;  // cfig.h:508
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LWU = 20'h00308;  // cfig.h:509
// M3: LR/SC opcodes. All four keep func[0]=0 (load-like) so the read phase
// rides the load path; the field layout follows the regular-load convention
// (func[1]=sign-extend, func[3:2]=size), so LR.W sign-extends exactly like
// LW. Donor cross-ref (aq_idu_cfig.h:520-523): FUNC_LR_W=12'b..0010_1010,
// FUNC_LR_D=12'b..0010_1100 -- the functionally-active low-4 bits (load/
// sign/size) match bit-for-bit; the prefix bits differ (donor 0x02x class
// tag vs this clone's 0xB0x). The donor's SC_W/D (...1001 / ...1101) are
// store-like (func[0]=1), a documented deviation: in this clone SC commits
// its store via an explicit STB-create at reply (gated on the reservation
// match) instead of through the store pipeline.
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LR_W = 20'h00b0a;  // load-like, sign-ext, W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_LR_D = 20'h00b0c;  // load-like, D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_SC_W = 20'h00b08;  // load-like, W (read value discarded)
parameter [FUNC_WIDTH-1:0] LSU_FUNC_SC_D = 20'h00b0e;  // load-like, D
// M3 Task 4: AMO opcodes (func[0]=0 -> load-like for the initial read phase).
// Encoding: upper bits select AMO op (matching donor aq_lsu_amo_alu.v funct5),
// bit[1:0] select width. These drive the AMO FSM read-modify-write sequence.
// AMO funct5 (donor aq_lsu_amo_alu.v:142-150):
//   add=00000, swap=00001, xor=00100, and=01100, or=01000,
//   min=10000, minu=11000, max=10100, maxu=11100
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOADD_W  = 20'h01008;  // AMOADD.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOSWAP_W = 20'h01018;  // AMOSWAP.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOXOR_W  = 20'h01048;  // AMOXOR.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOAND_W  = 20'h010c8;  // AMOAND.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOOR_W   = 20'h01088;  // AMOOR.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMIN_W  = 20'h01108;  // AMOMIN.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMINU_W = 20'h01188;  // AMOMINU.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMAX_W  = 20'h01148;  // AMOMAX.W
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMAXU_W = 20'h011c8;  // AMOMAXU.W
// D-width AMOs: bits[3:2]=11 selects D (matches ag_size=func[3:2] used for
// the read-phase access size), bits[1:0] stay 00.
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOADD_D  = 20'h0100c;  // AMOADD.D  (bits[3:2]=11)
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOSWAP_D = 20'h0101c;  // AMOSWAP.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOXOR_D  = 20'h0104c;  // AMOXOR.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOAND_D  = 20'h010cc;  // AMOAND.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOOR_D   = 20'h0108c;  // AMOOR.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMIN_D  = 20'h0110c;  // AMOMIN.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMINU_D = 20'h0118c;  // AMOMINU.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMAX_D  = 20'h0114c;  // AMOMAX.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_AMOMAXU_D = 20'h011cc;  // AMOMAXU.D
parameter [FUNC_WIDTH-1:0] LSU_FUNC_SB  = 20'h00301;  // cfig.h:513
parameter [FUNC_WIDTH-1:0] LSU_FUNC_SH  = 20'h00305;  // cfig.h:514
parameter [FUNC_WIDTH-1:0] LSU_FUNC_SW  = 20'h00309;  // cfig.h:515
parameter [FUNC_WIDTH-1:0] LSU_FUNC_SD  = 20'h0030d;  // cfig.h:516

endpackage
