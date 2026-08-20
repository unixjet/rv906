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
//    Task 6, tied inactive here): `pa_vld=1` always, `pa[27:0]=va[27:0]`
//    (identity map), `page_fault=access_fault=0` always; `ca`/`so`/`buf`/
//    `sec`/`sh` come from the PMA/sysmap lookup (contract 5), independent
//    of the identity-map logic. The ITLB side's M1 protocol packs
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
    // SKELETON BODY (plan Task 6 replaces it): every response inactive/0 --
    // NOT yet the contract-2 identity-map behavior (pa_vld=1 always etc.);
    // that is real logic, deferred to Task 6 like every other unit here.
    //=========================================================================
    assign mmu_ifu_access_fault = 1'b0;
    assign mmu_ifu_pa           = {MMU_PA_WIDTH{1'b0}};
    assign mmu_ifu_pa_vld       = 1'b0;
    assign mmu_ifu_prot         = {MMU_PROT_WIDTH{1'b0}};

    assign mmu_lsu_pa           = {MMU_PA_WIDTH{1'b0}};
    assign mmu_lsu_pa_vld       = 1'b0;
    assign mmu_lsu_ca           = 1'b0;
    assign mmu_lsu_so           = 1'b0;
    assign mmu_lsu_buf          = 1'b0;
    assign mmu_lsu_sec          = 1'b0;
    assign mmu_lsu_sh           = 1'b0;
    assign mmu_lsu_page_fault   = 1'b0;
    assign mmu_lsu_access_fault = 1'b0;

endmodule
