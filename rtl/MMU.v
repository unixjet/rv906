//=============================================================================
// MMU.v - identity-map + PMA/sysmap stub, shared by IFU's ITLB port and
//          LSU's DTLB port                        (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 6; this file freezes
// the port list only -- the REAL uTLB/JTLB/PTW behind this same protocol is
// M4 scope, not read/ported here):
//   gen_rtl/mmu/rtl/aq_mmu_top.v          (real MMU top -- M4 target,
//                                           referenced only for the
//                                           request/response protocol
//                                           shape this stub must satisfy)
//   gen_rtl/mmu/rtl/aq_mmu_sysmap.v       (+ _hit.v, sysmap.h -- the
//                                           8-region fixed PA-range
//                                           cacheability table; rv906's own
//                                           PMA table, contract 5, replaces
//                                           T-Head's SoC-specific
//                                           thresholds)
// References: design doc S2.3.2 (the identity-map stub decision), S2.3.5
// (rv906's own PMA table), contract 2 (the exact request/response shape)
// and contract 5 (the PMA regions), LSU note A7/B3.
//
// SEAM NOTES:
//  * Two INDEPENDENT port groups (contract 2) -- one for IFU's ITLB
//    request, one for LSU's DTLB request. The ITLB group's names/widths
//    match `rtl/ICache.v`'s ALREADY-FROZEN-SINCE-M1 MMU-facing ports
//    exactly (icache.v header, confirmed icache.v:74-81,113-116,153-155)
//    -- this module replaces `rtl/RVProc.v`'s current inline
//    `assign mmu_ifu_pa = ifu_mmu_va[MMU_PA_WIDTH-1:0]` stub (Task 7), not
//    ICache.v's port list, which stays untouched.
//  * The DTLB group's names/widths match `rtl/LSU.v`'s `lsu_mmu_*`/
//    `mmu_lsu_*` ports exactly (this task's own LSU.v skeleton).
//  * Stub behavior for BOTH ports (contract 2, implemented for real in
//    Task 6, tied inactive here): `pa_vld=1` always, `pa[27:0]` = the
//    identity-map page number, and BOTH request inputs ARE page numbers
//    (the I-side's `ifu_mmu_va` is `icache_rd_addr[63:12]`, the D-side's
//    `lsu_mmu_va` is `ag_addr[63:12]` -- donor: aq_lsu_ag.v:1566), so the
//    identity map is the same bit-slice on both sides: `pa =
//    <port>_mmu_va[27:0]`. (An earlier draft of this file misread the
//    D-side input as a byte VA and extracted `va[39:12]` here; a donor
//    check found the page-number shift belongs in the requester, LSU.v --
//    see LSU.v's own comment at its `lsu_mmu_va` assign. 2026-08-23.)
//    `page_fault=access_fault=0` always; `ca`/`so`/`buf`/`sec`/`sh` come
//    from the PMA/sysmap lookup (contract 5), independent of the
//    identity-map logic. The ITLB side's M1 protocol packs
//    cacheable/bufferable/secure into one 5-bit `prot` field instead of
//    separate wires (ICache.v's own header note) -- this module must
//    produce that same packed encoding for the ITLB port while producing
//    the DTLB port's separate `ca`/`so`/`buf`/`sec`/`sh` wires, per each
//    port's own already-established shape.
//  * No RTU/IDU/CSR ports at all -- this is a combinational lookup table
//    with exactly two clients, matching contract 2's shape (mirrors the
//    project's "a submodule earns its own file" rule, umbrella S6.2 rule
//    7, without being an RTU-visible unit itself).
//=============================================================================

import rvproc_pkg::*;

module MMU (
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IFU's ITLB request/response -- names/widths match ICache.v's already-
    // frozen-since-M1 MMU-facing ports verbatim.
    //=========================================================================
    input  wire                     ifu_mmu_abort,
    input  wire [MMU_VA_WIDTH-1:0]  ifu_mmu_va,
    input  wire                     ifu_mmu_va_vld,
    output wire                     mmu_ifu_access_fault,
    output wire [MMU_PA_WIDTH-1:0]  mmu_ifu_pa,
    output wire                     mmu_ifu_pa_vld,
    output wire [MMU_PROT_WIDTH-1:0] mmu_ifu_prot,

    //=========================================================================
    // LSU's DTLB request/response -- contract 2's generic shape, matching
    // LSU.v's `lsu_mmu_*`/`mmu_lsu_*` ports verbatim.
    //=========================================================================
    input  wire [MMU_VA_WIDTH-1:0]  lsu_mmu_va,
    input  wire                     lsu_mmu_va_vld,
    input  wire [1:0]               lsu_mmu_priv_mode,
    input  wire                     lsu_mmu_st_inst,
    output wire [MMU_PA_WIDTH-1:0]  mmu_lsu_pa,
    output wire                     mmu_lsu_pa_vld,
    output wire                     mmu_lsu_ca,
    output wire                     mmu_lsu_so,
    output wire                     mmu_lsu_buf,
    output wire                     mmu_lsu_sec,
    output wire                     mmu_lsu_sh,
    output wire                     mmu_lsu_page_fault,
    output wire                     mmu_lsu_access_fault
);

    //=========================================================================
    // TASK 6 REAL BODY: combinational identity-map + PMA/sysmap stub
    // (contract 2). Both ports respond the SAME cycle a request arrives --
    // no registers anywhere in this module.
    //
    // PMA TABLE (contract 5, rv906's own SoC map, NOT T-Head's sysmap
    // thresholds -- design doc S2.3.5): keyed on the physical (==virtual,
    // identity-mapped) BYTE address. Region membership is checked on the
    // page-aligned reconstruction of the 28-bit PPN (`{pa,12'b0}`) against
    // the literal byte-address boundaries below -- safe because every
    // region in the table is at least page-granular (the CLINT/PLIC/UART
    // upper bounds are not page-aligned themselves, but the region's SIZE
    // is an exact multiple of 4KB starting from a page-aligned base, so
    // comparing the page-aligned address against the literal upper bound
    // still resolves membership correctly one page at a time).
    //   DRAM  0x8000_0000-0xFFFF_FFFF : cacheable, bufferable
    //   CLINT 0x0200_0000-0x0200_FFFF : uncached, strongly ordered
    //   PLIC  0x0C00_0000-0x0CFF_FFFF : uncached, strongly ordered
    //   UART  0x1000_0000-0x1000_FFFF : uncached, strongly ordered
    //   else  0x0000_0000-0x7FFF_FFFF (not otherwise covered) : uncached/
    //         reserved -- deliberate divergence from AXICrossbar's
    //         DEFAULT_SLAVE=SI_MEM convenience (contract 5).
    // Only two attribute bits are meaningful for M2 (contract 5): `so` is
    // simply `!ca` for every row this table actually contains (no row pairs
    // "cacheable + strongly-ordered" or "uncached + weakly-ordered"); `buf`
    // mirrors `ca` (the one cacheable region is also the one bufferable
    // region); `sec`/`sh` are permanently 0 (M4/never territory, design doc
    // S2.3.5).
    //=========================================================================
    function automatic pma_cacheable(input [MMU_PA_WIDTH-1:0] page_num);
        reg [39:0] pa_full;
        begin
            pa_full = {page_num, 12'b0};   // page-aligned reconstruction, PC_WIDTH=40
            pma_cacheable = (pa_full >= 40'h8000_0000) && (pa_full <= 40'hFFFF_FFFF);
        end
    endfunction

    // ---- ITLB port (IFU) ----
    wire _ifu_mmu_abort_unused = ifu_mmu_abort;   // comb stub: nothing to cancel

    wire ifu_ca = pma_cacheable(ifu_mmu_va[MMU_PA_WIDTH-1:0]);

    assign mmu_ifu_access_fault = 1'b0;
    assign mmu_ifu_pa           = ifu_mmu_va[MMU_PA_WIDTH-1:0];
    assign mmu_ifu_pa_vld       = 1'b1;                          // contract 2: never a miss
    // {pgflt, supv, ca, ba, sec} -- pinned by ICache.v's own header from
    // icache.v's actual consumers; pgflt/sec permanently 0 (no fault-capable
    // MMU exists until M4), supv permissively 1 (M2 is M-mode-only, matching
    // RVProc.v's M1 inline stub convention this module replaces).
    assign mmu_ifu_prot = {1'b0, 1'b1, ifu_ca, ifu_ca, 1'b0};

    // ---- DTLB port (LSU) ----
    wire _lsu_priv_unused = lsu_mmu_priv_mode[0] ^ lsu_mmu_priv_mode[1];
    wire _lsu_st_unused   = lsu_mmu_st_inst;   // no permission checks in M2 (bare M-mode)

    // `lsu_mmu_va` is the PAGE NUMBER, not the byte address -- the SAME
    // convention as the ITLB port above (icache.v drives
    // `icache_rd_addr[63:12]`). Donor proof: aq_lsu_ag.v:1566 `assign
    // lsu_mmu_va[51:0] = ag_pipe_addr[63:12]`, response aq_lsu_ag.v:201
    // `input [27:0] mmu_lsu_pa`, PA reassembled by the requester itself,
    // aq_lsu_ag.v:1446 `ag_pipe_pa = {mmu_pa, ag_pipe_addr[11:0]}`. (An
    // earlier revision of this file misread contract 2's "va[51:0]" as a
    // byte VA and did the >>12 HERE; a donor check found LSU.v was the
    // module that deviated, so the shift lives in LSU.v -- 2026-08-23.)
    wire lsu_ca = pma_cacheable(lsu_mmu_va[MMU_PA_WIDTH-1:0]);

    assign mmu_lsu_pa           = lsu_mmu_va[MMU_PA_WIDTH-1:0];   // identity map: page in, page out
    assign mmu_lsu_pa_vld       = 1'b1;                            // contract 2: never a miss
    assign mmu_lsu_ca           = lsu_ca;
    assign mmu_lsu_so           = !lsu_ca;
    assign mmu_lsu_buf          = lsu_ca;
    assign mmu_lsu_sec          = 1'b0;
    assign mmu_lsu_sh           = 1'b0;
    assign mmu_lsu_page_fault   = 1'b0;
    assign mmu_lsu_access_fault = 1'b0;

endmodule
