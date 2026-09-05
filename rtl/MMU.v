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
    output wire                     mmu_lsu_access_fault,

    //=========================================================================
    // CP0 -> MMU : M4 Task 1 privilege/translation controls (storage in
    // CSR.v; given meaning at Tasks 3-5). satp + MXR/SUM + current priv.
    //=========================================================================
    input  wire [63:0]              cp0_mmu_satp_data,
    input  wire                     cp0_mmu_satp_wen,
    input  wire                     cp0_mmu_mxr,
    input  wire                     cp0_mmu_sum,
    input  wire [1:0]               cp0_yy_priv_mode,
    // CSR -> MMU / MMU -> CSR : sfence.vma whole-TLB invalidate handshake
    // (Task 7). Mirrors CSR.v's cp0_mmu_sfence_vld/mmu_cp0_sfence_done --
    // see the sfence_apply/sfence_pend site near tlb_inv_all below for the
    // mid-walk-hazard resolution (wait-for-PTW-idle instead of the donor's
    // abort-drain, aq_mmu_ptw.v's PTW_ABT/ABT_DATA).
    input  wire                     cp0_mmu_sfence_vld,
    output wire                     mmu_cp0_sfence_done,

    //=========================================================================
    // M4 Task 4: PTW memory-read servant channel -- contract shape mirrors
    // donor aq_mmu_ptw.v's mmu_lsu_data_req/_addr/_size + lsu_mmu_data/_vld/
    // _bus_error (group 5's frozen six-wire channel, rv12 P3 precedent).
    // LSU.v's Task 5 servant answers these for real; until then RVProc.v
    // ties the inputs inactive and leaves the outputs unconsumed (satp
    // stays Mode=0 through Task 8's swap, so this path is unreachable in
    // the full-core sim before then -- the G1 OFF-path gate holds by
    // construction, not by inspection).
    //=========================================================================
    output wire                     mmu_lsu_data_req,
    output wire [PC_WIDTH-1:0]      mmu_lsu_data_req_addr,
    output wire                     mmu_lsu_data_req_size,  // donor hardwires 1 (8B PTE)
    input  wire [63:0]              lsu_mmu_data,
    input  wire                     lsu_mmu_data_vld,
    input  wire                     lsu_mmu_bus_error,

    //=========================================================================
    // M4 Task 4: walk abort on a flushed requester. ifu_mmu_abort already
    // exists (M2 port); lsu_mmu_abort is new (donor's lsu_mmu_abort0/1
    // collapsed to one -- rv906's single-outstanding DTLB request, D1).
    //=========================================================================
    input  wire                     lsu_mmu_abort,

    //=========================================================================
    // M4 Task 4: PMP check channels. ONE channel per I/D side, time-shared
    // by MMU.v among three uses that never coincide in the same cycle: the
    // mach/bare identity path (D14), the TLB-hit live re-check (rv906
    // doesn't cache PMP flags in the TLB entry -- rv12 MMU_FLG_W precedent,
    // see rvproc_pkg.sv), and the walker's per-level PT-page check (donor's
    // P14 rule: a PT read is PMP-checked with the ORIGINAL access's own
    // R/W/X requirement, not "this is a read"). Task 5/6 wire these to the
    // real PMP.v instance in RVProc.v (currently dead-tied per Task 2's own
    // header note); this task's mmu_tb pokes the deny inputs directly.
    //=========================================================================
    output wire [PC_WIDTH-1:0]      mmu_pmp_fetch_pa,
    output wire                     mmu_pmp_fetch_vld,
    input  wire                     pmp_mmu_fetch_deny,
    output wire [PC_WIDTH-1:0]      mmu_pmp_data_pa,
    output wire                     mmu_pmp_load,
    output wire                     mmu_pmp_store,
    output wire                     mmu_pmp_data_vld,
    // MPRV-resolved privilege for the data channel's M-mode-bypass check
    // (donor aq_pmp_acc.v:118-119, commented-out
    // "cp0_priv_mode = pmp_mprv_status ? cp0_pmp_mpp : cur_priv_mode" --
    // the donor's caller resolves this per access type before presenting
    // it to the single shared aq_pmp_acc channel; rv906's split fetch/data
    // channels need the resolution only on the data side, since MPRV never
    // affects fetches). Time-shared the same way as mmu_pmp_data_pa above:
    // the walker's per-level PT-page check uses the priv captured at walk
    // start (ptw_priv_mode, itself sampled from lsu_mmu_priv_mode -- SECTION
    // 6.5), everything else uses the live lsu_mmu_priv_mode (mirrors
    // lsu_supv/lsu_user's use of it for the DTLB-hit permission check,
    // SECTION 7 below).
    output wire [1:0]               mmu_pmp_data_priv_mode,
    input  wire                     pmp_mmu_data_deny
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
    function automatic [4:0] sysmap_attr(input [MMU_PA_WIDTH-1:0] page_num);
        reg [39:0] pa_full;
        begin
            pa_full = {page_num, 12'b0};   // page-aligned reconstruction, PC_WIDTH=40
            if ((pa_full >= 40'h8000_0000) && (pa_full <= 40'hFFFF_FFFF))
                sysmap_attr = 5'b01100;    // DRAM: {so,ca,buf,sh,sec}
            else
                sysmap_attr = 5'b10000;    // uncached: so=1, rest 0
        end
    endfunction

    //=========================================================================
    // M4 TASK 3 -- satp decode + translation enable (donor aq_mmu_regs.v;
    // aq_cp0_prtc_csr.v:139-140). CSR.v holds the satp CSR itself (S-bank,
    // WARL: only mode bit63 writable -> Mode in {0=Bare, 8=Sv39}; ASID[59:44],
    // PPN[27:0] stored) and presents it here as cp0_mmu_satp_data. The MMU
    // extracts the fields and forms the per-port translation enable:
    //   sv39_en   = satp[63]                  (donor regs_mmu_en)
    //   xx_mmu_en = Sv39 && priv != M         (donor aq_mmu_regs.v)
    // M-mode and bare accesses (xx_mmu_en=0) take the mach/identity path
    // (D14) -- still PMP-checked once PMP joins (Tasks 5/6). Until the
    // TLB/PTW storage exists (Task 4) BOTH ports stay on the identity+PMA
    // path, so the OFF template (satp=0 reset, or any M-mode access) is
    // bit-exact with the M2/M3 stub (G5 off-template equivalence). MXR/SUM
    // (cp0_mmu_mxr/_sum) feed the PTE permission predicate at Task 4.
    //=========================================================================
    wire        sv39_en   = cp0_mmu_satp_data[63];
    wire [15:0] satp_asid = cp0_mmu_satp_data[59:44];
    wire [MMU_PA_WIDTH-1:0] satp_ppn = cp0_mmu_satp_data[MMU_PA_WIDTH-1:0];

    wire ifu_mmu_en = sv39_en && (cp0_yy_priv_mode  != PRIV_M);
    wire lsu_mmu_en = sv39_en && (lsu_mmu_priv_mode != PRIV_M);

    // satp-write flush (donor aq_cp0_prtc_csr.v): any accepted satp write
    // pulses cp0_mmu_satp_wen; it invalidates the TLB at Task 4.
    wire satp_write_flush = cp0_mmu_satp_wen;

    // ---- the mach/bare identity+PMA path (D14) -- UNCHANGED from the
    // M2/M3 stub; still reached whenever ifu_mmu_en/lsu_mmu_en is 0, so the
    // OFF template stays bit-exact (G1/G5) regardless of everything below.
    wire [4:0] ifu_mach_attr = sysmap_attr(ifu_mmu_va[MMU_PA_WIDTH-1:0]);
    wire       ifu_mach_ca   = ifu_mach_attr[3];
    wire [4:0] lsu_mach_attr = sysmap_attr(lsu_mmu_va[MMU_PA_WIDTH-1:0]);
    wire       lsu_mach_ca   = lsu_mach_attr[3];

    wire ifu_mach_path = !ifu_mmu_en;
    wire lsu_mach_path = !lsu_mmu_en;

    //=========================================================================
    // SECTION 4: THE TLB (M4 Task 4). Single 128-entry fully-associative
    // flop array (D11) shared by BOTH ports -- donor's uTLB+jTLB two-level
    // structure collapses to one lookup per port per cycle, matching
    // rv906's single-issue in-order pipe. Entry shape transcribes donor
    // aq_mmu_jtlb.v's tag/data words (tag: vld+vpn+asid+pgs+g; data:
    // ppn+flg) minus the tag's per-size FIFO age bits (replacement is one
    // round-robin pointer, D11's own wording) and minus the data word's
    // 4-bit PMP slice: rv906 does NOT cache PMP permission in the TLB
    // entry -- every access (mach, TLB-hit, and the walker's own per-level
    // PT-page reads) re-checks PMP LIVE against PMP.v (SECTION 7), so PMP
    // writes take effect without requiring a TLB flush. This matches rv12's
    // OWN MMU_FLG_W shape (no PMP bits either), not a deviation from either
    // sibling project -- and it is behaviorally identical to the donor's
    // cache-at-refill scheme under the one PMP configuration every test in
    // this suite ever uses (a single all-of-memory NAPOT RWX region), so it
    // needs no ledger entry.
    //=========================================================================
    localparam TLB_N     = MMU_TLB_ENTRIES;
    localparam TLB_IDX_W = 7;              // clog2(128)

    reg                        tlb_vld  [0:TLB_N-1];
    reg  [MMU_VPN_WIDTH-1:0]   tlb_vpn  [0:TLB_N-1];
    reg  [MMU_ASID_WIDTH-1:0]  tlb_asid [0:TLB_N-1];
    reg  [MMU_PGS_WIDTH-1:0]   tlb_pgs  [0:TLB_N-1];   // one-hot {1g,2m,4k}
    reg                        tlb_g    [0:TLB_N-1];
    reg  [MMU_PA_WIDTH-1:0]    tlb_ppn  [0:TLB_N-1];
    reg  [MMU_FLG_WIDTH-1:0]   tlb_flg  [0:TLB_N-1];   // {pma[4:0],D,A,U,X,W,R,V}

    // Invalidate-all trigger: a satp write flushes the whole TLB (donor
    // "satp write flushes uTLBs", per-port aq_mmu_regs finding). Task 7's
    // sfence.vma sequencer ORs its own pulse into this SAME trigger
    // (D-M4-6: every sfence flavor over-invalidates the whole array, so no
    // separate ASID/VA-scoped invalidate machinery is ever needed) via
    // sfence_apply, defined below near the PTW state reg it gates on.
    wire tlb_inv_all = satp_write_flush || sfence_apply;

    reg [TLB_IDX_W-1:0] tlb_rr_ptr;   // round-robin replacement pointer (D11)

    // VPN match at the entry's OWN page size (donor aq_mmu_jtlb.v's
    // size-selected compare).
    function automatic vpn_match(input [MMU_PGS_WIDTH-1:0] pgs,
                                  input [MMU_VPN_WIDTH-1:0] tvpn,
                                  input [MMU_VPN_WIDTH-1:0] rvpn);
        begin
            if (pgs[2])      vpn_match = (tvpn[26:18] == rvpn[26:18]);  // 1G
            else if (pgs[1]) vpn_match = (tvpn[26:9]  == rvpn[26:9]);   // 2M
            else             vpn_match = (tvpn[26:0]  == rvpn[26:0]);   // 4K
        end
    endfunction

    //-------------------------------------------------------------------------
    // The high-VA legality check (donor aq_mmu_iutlb/dutlb's own arm,
    // Agent-3/rv12 precedent): VA[63:39] must sign-extend VA[38] (tag
    // width is only 27 bits -- Sv39's own VPN -- so this is checked from
    // the raw port bits, independent of any TLB tag, and folds into PAGE
    // fault, never access fault, spec S3.1). Only meaningful while
    // translation is active; the mach/bare path has no such notion.
    //-------------------------------------------------------------------------
    wire ifu_va_illegal =
        (ifu_mmu_va[MMU_VPN_WIDTH-1]  && !(&ifu_mmu_va[MMU_VA_WIDTH-1:MMU_VPN_WIDTH]))
     || (!ifu_mmu_va[MMU_VPN_WIDTH-1] &&  (|ifu_mmu_va[MMU_VA_WIDTH-1:MMU_VPN_WIDTH]));
    wire lsu_va_illegal =
        (lsu_mmu_va[MMU_VPN_WIDTH-1]  && !(&lsu_mmu_va[MMU_VA_WIDTH-1:MMU_VPN_WIDTH]))
     || (!lsu_mmu_va[MMU_VPN_WIDTH-1] &&  (|lsu_mmu_va[MMU_VA_WIDTH-1:MMU_VPN_WIDTH]));
    wire ifu_va_illegal_active = ifu_mmu_en && ifu_va_illegal;
    wire lsu_va_illegal_active = lsu_mmu_en && lsu_va_illegal;

    //-------------------------------------------------------------------------
    // ITLB lookup (combinational scan; highest-index match wins on the
    // pathological multi-hit case -- unreachable under correct software,
    // which sfences the whole TLB, D-M4-6, before any page-size change).
    //-------------------------------------------------------------------------
    wire [MMU_VPN_WIDTH-1:0] ifu_req_vpn = ifu_mmu_va[MMU_VPN_WIDTH-1:0];
    reg                      itlb_hit_r;
    reg  [MMU_PA_WIDTH-1:0]  itlb_ppn_r;
    reg  [MMU_FLG_WIDTH-1:0] itlb_flg_r;
    reg  [MMU_PGS_WIDTH-1:0] itlb_pgs_r;
    integer ii;
    always @* begin
        itlb_hit_r = 1'b0;
        itlb_ppn_r = {MMU_PA_WIDTH{1'b0}};
        itlb_flg_r = {MMU_FLG_WIDTH{1'b0}};
        itlb_pgs_r = {MMU_PGS_WIDTH{1'b0}};
        for (ii = 0; ii < TLB_N; ii = ii + 1)
            if (tlb_vld[ii] && (tlb_g[ii] || tlb_asid[ii] == satp_asid)
                            && vpn_match(tlb_pgs[ii], tlb_vpn[ii], ifu_req_vpn)) begin
                itlb_hit_r = 1'b1;
                itlb_ppn_r = tlb_ppn[ii];
                itlb_flg_r = tlb_flg[ii];
                itlb_pgs_r = tlb_pgs[ii];
            end
    end
    // Superpage splice: a cached entry's PPN is the FRAME base (alignment-
    // checked zero in the low bits at refill, SECTION 6); the actual access
    // supplies the sub-frame offset from its own VPN (donor's PA-mux
    // splice, ct_mmu_iutlb.v/dutlb.v precedent).
    wire [MMU_PA_WIDTH-1:0] itlb_ppn_spliced =
          ({MMU_PA_WIDTH{itlb_pgs_r[2]}} & {itlb_ppn_r[MMU_PA_WIDTH-1:18], ifu_req_vpn[17:0]})
        | ({MMU_PA_WIDTH{itlb_pgs_r[1]}} & {itlb_ppn_r[MMU_PA_WIDTH-1:9],  ifu_req_vpn[8:0]})
        | ({MMU_PA_WIDTH{itlb_pgs_r[0]}} &  itlb_ppn_r);

    //-------------------------------------------------------------------------
    // DTLB lookup (same shape).
    //-------------------------------------------------------------------------
    wire [MMU_VPN_WIDTH-1:0] lsu_req_vpn = lsu_mmu_va[MMU_VPN_WIDTH-1:0];
    reg                      dtlb_hit_r;
    reg  [MMU_PA_WIDTH-1:0]  dtlb_ppn_r;
    reg  [MMU_FLG_WIDTH-1:0] dtlb_flg_r;
    reg  [MMU_PGS_WIDTH-1:0] dtlb_pgs_r;
    integer di;
    always @* begin
        dtlb_hit_r = 1'b0;
        dtlb_ppn_r = {MMU_PA_WIDTH{1'b0}};
        dtlb_flg_r = {MMU_FLG_WIDTH{1'b0}};
        dtlb_pgs_r = {MMU_PGS_WIDTH{1'b0}};
        for (di = 0; di < TLB_N; di = di + 1)
            if (tlb_vld[di] && (tlb_g[di] || tlb_asid[di] == satp_asid)
                            && vpn_match(tlb_pgs[di], tlb_vpn[di], lsu_req_vpn)) begin
                dtlb_hit_r = 1'b1;
                dtlb_ppn_r = tlb_ppn[di];
                dtlb_flg_r = tlb_flg[di];
                dtlb_pgs_r = tlb_pgs[di];
            end
    end
    wire [MMU_PA_WIDTH-1:0] dtlb_ppn_spliced =
          ({MMU_PA_WIDTH{dtlb_pgs_r[2]}} & {dtlb_ppn_r[MMU_PA_WIDTH-1:18], lsu_req_vpn[17:0]})
        | ({MMU_PA_WIDTH{dtlb_pgs_r[1]}} & {dtlb_ppn_r[MMU_PA_WIDTH-1:9],  lsu_req_vpn[8:0]})
        | ({MMU_PA_WIDTH{dtlb_pgs_r[0]}} &  dtlb_ppn_r);

    //-------------------------------------------------------------------------
    // The hit-time permission predicate: the walker's page-fault predicate
    // (SECTION 6) minus the raw-PTE-only arms (invalid/write-only/leaf-
    // alignment/level-3-no-R-X) which cannot apply to an already-installed
    // LEAF entry -- only the per-access arms that must be re-checked on
    // EVERY use of a cached mapping. D-M4-1 keeps the D-bit check live
    // (donor comments it out, aq_mmu_ptw.v:699-700, needed by
    // rv64si-p-dirty + the v-env). D-M4-10 (below, SECTION 6) drops the
    // SUM excuse for fetches; the fetch predicate here carries no SUM/MXR
    // term at all, matching that same amendment.
    //-------------------------------------------------------------------------
    function automatic dtlb_perm_flt(input [MMU_FLG_WIDTH-1:0] flg,
                                      input ld, input st,
                                      input supv, input usr,
                                      input mxr, input sum);
        reg r, w, x, u, a, d;
        begin
            r = flg[1]; w = flg[2]; x = flg[3]; u = flg[4]; a = flg[5]; d = flg[6];
            dtlb_perm_flt = (ld && !r && !(mxr && x))
                         || (st && !w)
                         || (u && supv && !sum)
                         || (!u && usr)
                         || !a
                         || (st && !d);
        end
    endfunction

    function automatic itlb_perm_flt(input [MMU_FLG_WIDTH-1:0] flg,
                                      input supv, input usr);
        reg x, u, a;
        begin
            x = flg[3]; u = flg[4]; a = flg[5];
            itlb_perm_flt = (!x) || (u && supv) || (!u && usr) || !a;
        end
    endfunction

    //=========================================================================
    // SECTION 5: sfence write conflict guard -- NONE needed. tlb_inv_all
    // (satp_write_flush) and a PTW refill write can never coincide: a satp
    // write is only accepted while mmu_en observes the OLD satp (the
    // decode-to-writeback pipeline already serializes CSR writes against
    // in-flight memory ops in this single-issue core), and any walk started
    // under the OLD satp is for an address space the flush is about to
    // discard anyway -- the flushed entry simply gets overwritten again by
    // whatever re-walks it next. No ledger entry: this is the natural
    // consequence of D-M4-6's own whole-TLB-invalidate choice, not a new
    // simplification.
    //=========================================================================

    //=========================================================================
    // SECTION 6: THE PTW (M4 Task 4). ct_mmu_ptw.v transcribed, simplified
    // for rv906's own architecture:
    //   * NO uTLB refill target -- the walker writes THIS TLB directly
    //     (SECTION 4's write port), so there is no separate "arb" stage and
    //     no arb_ptw_grant wait in PTW_DATA_VLD (D3/D4: the TLB write is
    //     immediate, not shared with another requester).
    //   * NO PMP-cross ("CRS") states -- donor's PTW_1G_PMP1/PMP2/2M_PMP1/
    //     PMP2 exist ONLY to determine what PMP flags to CACHE for a huge
    //     page that might span multiple PMP regions. rv906 doesn't cache
    //     PMP flags at all (SECTION 4's own note) -- every access is
    //     re-checked live against the EXACT final PA, so PMP granularity
    //     is handled correctly with no walker-time cross detection (D9's
    //     own argument: "rv906's page-granular PMP makes this
    //     unnecessary").
    //   * NO abort-drain states (donor's PTW_ABT/PTW_ABT_DATA, which exist
    //     to restart a walk cleanly after a mid-flight sfence). A flushed
    //     requester's va_vld simply drops, so its fault/refill answer is
    //     never delivered (SECTION 6.4's vpn+type-matched delivery gate);
    //     the walk itself is NOT aborted early -- a documented
    //     simplification, since a stale walk free-runs to completion and
    //     either installs a harmless TLB entry or is silently dropped, at
    //     most costing a few bounded stall cycles (<=9, one 3-level walk)
    //     on the NEXT translation request. The sfence-mid-walk hazard
    //     (Task 7) is closed WITHOUT an abort-drain path: sfence_apply
    //     (above, next to ptw_st's declaration) only fires once ptw_st has
    //     returned to PTW_IDLE, i.e. any stale walk is left to finish (its
    //     refill lands, if at all, strictly before that idle cycle) and
    //     the very next tlb_inv_all wipes whatever it installed.
    //   * Single arbiter between the two ports feeding one walker (D3/D4:
    //     "one outstanding 3-level PTW served through the LSU") -- D-side
    //     wins ties (a load/store is further along the pipe than the NEXT
    //     fetch it's blocking).
    //=========================================================================
    localparam [3:0] PTW_IDLE     = 4'd0,
                     PTW_FST_PMP  = 4'd1,
                     PTW_FST_DATA = 4'd2,
                     PTW_FST_CHK  = 4'd3,
                     PTW_SCD_PMP  = 4'd4,
                     PTW_SCD_DATA = 4'd5,
                     PTW_SCD_CHK  = 4'd6,
                     PTW_THD_PMP  = 4'd7,
                     PTW_THD_DATA = 4'd8,
                     PTW_THD_CHK  = 4'd9,
                     PTW_ACC_FLT  = 4'd10,
                     PTW_PGE_FLT  = 4'd11,
                     PTW_DATA_VLD = 4'd12;

    reg [3:0] ptw_st;

    // M4 Task 7: sfence.vma mid-walk hazard resolution (closes MMU.v
    // SECTION 5's own flagged gap; see CSR.v's sfence sequencer comment
    // for the full wait-then-wipe vs. the donor's abort-drain writeup).
    // cp0_mmu_sfence_vld is a single-cycle launch pulse; if the PTW isn't
    // idle that cycle, latch it in sfence_pend_r and keep re-checking
    // every cycle until ptw_st returns to PTW_IDLE (i.e. any in-flight
    // walk -- which may have read pre-invalidate PTE data -- has finished
    // and, if it refilled a now-stale entry, that entry is wiped by THIS
    // SAME tlb_inv_all pulse: refill writes only happen at PTW_DATA_VLD,
    // strictly one state before PTW_IDLE, so a stale write can never land
    // on the same or a later cycle than the invalidate that follows it).
    reg  sfence_pend_r;
    wire sfence_apply = (cp0_mmu_sfence_vld || sfence_pend_r)
                      && (ptw_st == PTW_IDLE);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                    sfence_pend_r <= 1'b0;
        else if (cp0_mmu_sfence_vld && ptw_st != PTW_IDLE) sfence_pend_r <= 1'b1;
        else if (sfence_apply)                          sfence_pend_r <= 1'b0;
    end
    assign mmu_cp0_sfence_done = sfence_apply;

    //-------------------------------------------------------------------------
    // 6.1  Request/arbitration and the accepted request's latched fields
    //-------------------------------------------------------------------------
    wire itlb_req = ifu_mmu_va_vld && !ifu_mmu_abort && ifu_mmu_en
                 && !itlb_hit_r && !ifu_va_illegal;
    wire dtlb_req = lsu_mmu_va_vld && !lsu_mmu_abort && lsu_mmu_en
                 && !dtlb_hit_r && !lsu_va_illegal;

    wire ptw_accept_d = (ptw_st == PTW_IDLE) && dtlb_req;
    wire ptw_accept_i = (ptw_st == PTW_IDLE) && !dtlb_req && itlb_req;
    wire ptw_accept   = ptw_accept_d || ptw_accept_i;

    reg                       ptw_is_fetch, ptw_is_load, ptw_is_store;
    reg [MMU_VPN_WIDTH-1:0]   ptw_vpn;
    reg [1:0]                 ptw_priv_mode;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ptw_is_fetch  <= 1'b0;
            ptw_is_load   <= 1'b0;
            ptw_is_store  <= 1'b0;
            ptw_vpn       <= {MMU_VPN_WIDTH{1'b0}};
            ptw_priv_mode <= PRIV_M;
        end else if (ptw_accept_d) begin
            ptw_is_fetch  <= 1'b0;
            ptw_is_load   <= !lsu_mmu_st_inst;
            ptw_is_store  <= lsu_mmu_st_inst;
            ptw_vpn       <= lsu_req_vpn;
            ptw_priv_mode <= lsu_mmu_priv_mode;
        end else if (ptw_accept_i) begin
            ptw_is_fetch  <= 1'b1;
            ptw_is_load   <= 1'b0;
            ptw_is_store  <= 1'b0;
            ptw_vpn       <= ifu_req_vpn;
            ptw_priv_mode <= cp0_yy_priv_mode;
        end
    end

    wire ptw_supv = (ptw_priv_mode == PRIV_S);
    wire ptw_user = (ptw_priv_mode == PRIV_U);

    //-------------------------------------------------------------------------
    // 6.2  The PTE flop and the walk addresses (ct_mmu_ptw.v:552-636).
    //      Level-advance addresses use the LIVE lsu_mmu_data (not the
    //      flopped PTE), exactly as the donor does -- forming the NEXT
    //      level's address one cycle earlier than a flop round-trip would.
    //-------------------------------------------------------------------------
    reg [63:0] pte_flop;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pte_flop <= 64'b0;
        else if (lsu_mmu_data_vld) pte_flop <= lsu_mmu_data;
    end

    wire pte_v = pte_flop[0], pte_r = pte_flop[1], pte_w = pte_flop[2],
         pte_x = pte_flop[3], pte_u = pte_flop[4], pte_g_bit = pte_flop[5],
         pte_a = pte_flop[6], pte_d = pte_flop[7];
    wire [MMU_PA_WIDTH-1:0] pte_ppn = pte_flop[10 +: MMU_PA_WIDTH];

    wire [MMU_PA_WIDTH-1:0] live_ppn = lsu_mmu_data[10 +: MMU_PA_WIDTH];

    wire ptw_hit_1g   = (ptw_st == PTW_FST_CHK) && pte_v && (pte_r || pte_x);
    wire ptw_hit_2m   = (ptw_st == PTW_SCD_CHK) && pte_v && (pte_r || pte_x);
    wire ptw_leaf_vld = ptw_hit_1g || ptw_hit_2m || (ptw_st == PTW_THD_CHK);

    wire addr_fst = ptw_accept;
    wire addr_scd = (ptw_st == PTW_FST_DATA) && lsu_mmu_data_vld;
    wire addr_thd = (ptw_st == PTW_SCD_DATA) && lsu_mmu_data_vld;

    wire [MMU_VPN_WIDTH-1:0] accept_vpn = ptw_accept_d ? lsu_req_vpn : ifu_req_vpn;
    wire [PC_WIDTH-1:0] fst_addr = {satp_ppn, accept_vpn[26:18], 3'b0};
    wire [PC_WIDTH-1:0] scd_addr = {live_ppn, ptw_vpn[17:9], 3'b0};
    wire [PC_WIDTH-1:0] thd_addr = {live_ppn, ptw_vpn[8:0], 3'b0};

    reg [PC_WIDTH-1:0] ptw_req_addr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ptw_req_addr <= {PC_WIDTH{1'b0}};
        else if (addr_fst) ptw_req_addr <= fst_addr;
        else if (addr_scd) ptw_req_addr <= scd_addr;
        else if (addr_thd) ptw_req_addr <= thd_addr;
    end

    //-------------------------------------------------------------------------
    // 6.3  The page-fault predicate (ct_mmu_ptw.v:655-687), arm for arm.
    //      D-M4-1: the D-bit arm is LIVE (donor comments it out). D-M4-10
    //      (new deviation, more-correct-than-donor like D-M4-1/3/8/9):
    //      donor's S->U arm is `pte_u && ptw_supv && !cp0_mmu_sum`
    //      unconditionally -- under SUM=1 that wrongly lets a SUPERVISOR
    //      FETCH execute from a U page too (donor aq_mmu_ptw.v:678, and
    //      its I-side uTLB hit arm agrees, ct_mmu_iutlb.v:599). The
    //      privileged spec is explicit SUM governs LOADS AND STORES only,
    //      never execution. rv906 qualifies the excuse to data accesses
    //      (`!(sum && !fetch)`), matching rv12's identical, already-shipped
    //      fix (rv12 MMU.v SECTION 5.4) applied at both this walker and the
    //      hit-time predicates above (SECTION 4) -- fetch permission
    //      resolves identically at a TLB hit and mid-walk.
    //-------------------------------------------------------------------------
    wire ptw_1g_misalign = ptw_hit_1g && (pte_ppn[17:0] != 18'b0);
    wire ptw_2m_misalign = ptw_hit_2m && (pte_ppn[8:0]  != 9'b0);

    wire ptw_pf_invalid    = !pte_v;
    wire ptw_pf_write_only = pte_w && !(pte_r || (cp0_mmu_mxr && pte_x));
    wire ptw_pf_type_priv  =
           (ptw_is_load  && !pte_r && !(cp0_mmu_mxr && pte_x))
        || (ptw_is_store && !pte_w)
        || (ptw_is_fetch && !pte_x)
        || (pte_u && ptw_supv && !(cp0_mmu_sum && !ptw_is_fetch))  // D-M4-10
        || (!pte_u && ptw_user)
        || (!pte_a)
        || (ptw_is_store && !pte_d)                                // D-M4-1
        || ptw_1g_misalign
        || ptw_2m_misalign;
    wire ptw_pf_thd_norwx = (ptw_st == PTW_THD_CHK) && !pte_r && !pte_x;

    wire ptw_page_flt = ptw_pf_invalid
                      || ptw_pf_write_only
                      || (ptw_pf_type_priv && ptw_leaf_vld)
                      || ptw_pf_thd_norwx;

    //-------------------------------------------------------------------------
    // 6.4  The PMP check (P14's rule: a PT read is checked with the
    //      ORIGINAL access's own R/W/X requirement, not "this is a read" --
    //      SECTION 7 wires this to the real PMP.v channels).
    //-------------------------------------------------------------------------
    wire ptw_pmp_beat = (ptw_st == PTW_FST_PMP) || (ptw_st == PTW_SCD_PMP)
                      || (ptw_st == PTW_THD_PMP);
    wire ptw_pmp_deny = ptw_is_fetch ? pmp_mmu_fetch_deny : pmp_mmu_data_deny;

    //-------------------------------------------------------------------------
    // 6.5  The FSM (ct_mmu_ptw.v:307-548, minus the dropped states above).
    //-------------------------------------------------------------------------
    reg [3:0] ptw_nxt_st;
    always @* begin
        case (ptw_st)
            PTW_IDLE:     ptw_nxt_st = ptw_accept ? PTW_FST_PMP : PTW_IDLE;
            PTW_FST_PMP:  ptw_nxt_st = ptw_pmp_deny ? PTW_ACC_FLT : PTW_FST_DATA;
            PTW_FST_DATA: ptw_nxt_st = lsu_mmu_bus_error ? PTW_ACC_FLT
                                      : lsu_mmu_data_vld  ? PTW_FST_CHK
                                      :                     PTW_FST_DATA;
            PTW_FST_CHK:  ptw_nxt_st = ptw_page_flt ? PTW_PGE_FLT
                                      : ptw_hit_1g   ? PTW_DATA_VLD
                                      :                PTW_SCD_PMP;
            PTW_SCD_PMP:  ptw_nxt_st = ptw_pmp_deny ? PTW_ACC_FLT : PTW_SCD_DATA;
            PTW_SCD_DATA: ptw_nxt_st = lsu_mmu_bus_error ? PTW_ACC_FLT
                                      : lsu_mmu_data_vld  ? PTW_SCD_CHK
                                      :                     PTW_SCD_DATA;
            PTW_SCD_CHK:  ptw_nxt_st = ptw_page_flt ? PTW_PGE_FLT
                                      : ptw_hit_2m   ? PTW_DATA_VLD
                                      :                PTW_THD_PMP;
            PTW_THD_PMP:  ptw_nxt_st = ptw_pmp_deny ? PTW_ACC_FLT : PTW_THD_DATA;
            PTW_THD_DATA: ptw_nxt_st = lsu_mmu_bus_error ? PTW_ACC_FLT
                                      : lsu_mmu_data_vld  ? PTW_THD_CHK
                                      :                     PTW_THD_DATA;
            PTW_THD_CHK:  ptw_nxt_st = ptw_page_flt ? PTW_PGE_FLT : PTW_DATA_VLD;
            PTW_ACC_FLT:  ptw_nxt_st = PTW_IDLE;
            PTW_PGE_FLT:  ptw_nxt_st = PTW_IDLE;
            PTW_DATA_VLD: ptw_nxt_st = PTW_IDLE;
            default:      ptw_nxt_st = PTW_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ptw_st <= PTW_IDLE;
        else        ptw_st <= ptw_nxt_st;
    end
    wire ptw_data_req = (ptw_st == PTW_FST_DATA) || (ptw_st == PTW_SCD_DATA)
                      || (ptw_st == PTW_THD_DATA);
    assign mmu_lsu_data_req      = ptw_data_req;
    assign mmu_lsu_data_req_addr = ptw_req_addr;
    assign mmu_lsu_data_req_size = 1'b1;   // donor hardwires 1 (every PT read is 8B)

    //-------------------------------------------------------------------------
    // 6.6  The refill word and the TLB write (SECTION 4's array).
    //-------------------------------------------------------------------------
    reg [MMU_PGS_WIDTH-1:0] ref_pgs;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ref_pgs <= {MMU_PGS_WIDTH{1'b0}};
        else if (ptw_leaf_vld)
            ref_pgs <= {ptw_hit_1g, ptw_hit_2m, (ptw_st == PTW_THD_CHK)};
    end

    wire [MMU_VPN_WIDTH-1:0] ptw_ref_vpn =
          ({MMU_VPN_WIDTH{ref_pgs[2]}} & {ptw_vpn[26:18], 18'b0})
        | ({MMU_VPN_WIDTH{ref_pgs[1]}} & {ptw_vpn[26:9],  9'b0})
        | ({MMU_VPN_WIDTH{ref_pgs[0]}} &  ptw_vpn);

    // rv906 PMA always comes from its own sysmap table (no MAEE/PTE-encoded
    // PMA, D-M4-2) -- looked up on the leaf's own frame base.
    wire [4:0] ptw_ref_pma = sysmap_attr(pte_ppn);
    wire [MMU_FLG_WIDTH-1:0] ptw_ref_flg =
        {ptw_ref_pma, pte_d, pte_a, pte_u, pte_x, pte_w, pte_r, pte_v};

    wire ptw_refill_wr = (ptw_st == PTW_DATA_VLD);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)               tlb_rr_ptr <= {TLB_IDX_W{1'b0}};
        else if (tlb_inv_all)     tlb_rr_ptr <= {TLB_IDX_W{1'b0}};
        else if (ptw_refill_wr)   tlb_rr_ptr <= tlb_rr_ptr + 1'b1;
    end

    // Per-entry write (a genvar-indexed generate, not a for-loop over the
    // array with non-blocking assigns -- Verilator BLKLOOPINIT doesn't
    // support that shape; PMP.v's own register file uses the same
    // per-entry generate pattern).
    genvar te;
    generate
        for (te = 0; te < TLB_N; te = te + 1) begin : g_tlb
            wire tlb_wr_sel = ptw_refill_wr && (tlb_rr_ptr == te[TLB_IDX_W-1:0]);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    tlb_vld[te] <= 1'b0;
                else if (tlb_inv_all)
                    tlb_vld[te] <= 1'b0;
                else if (tlb_wr_sel) begin
                    tlb_vld [te] <= 1'b1;
                    tlb_vpn [te] <= ptw_ref_vpn;
                    tlb_asid[te] <= satp_asid;
                    tlb_pgs [te] <= ref_pgs;
                    tlb_g   [te] <= pte_g_bit;
                    tlb_ppn [te] <= pte_ppn;
                    tlb_flg [te] <= ptw_ref_flg;
                end
            end
        end
    endgenerate

    //-------------------------------------------------------------------------
    // 6.7  Fault-pulse delivery, gated to the SAME live request the walk
    //      was launched for (VPN+type match). rv906's single-outstanding-
    //      per-port pipe needs no fuller instruction-id park mechanism than
    //      this (unlike rv12's iid-tracked fault park, P13) -- see the
    //      SECTION-6 header note on why an early abort isn't needed either.
    //-------------------------------------------------------------------------
    wire ptw_acc_flt_pulse = (ptw_st == PTW_ACC_FLT);
    wire ptw_pge_flt_pulse = (ptw_st == PTW_PGE_FLT);

    wire itlb_ptw_flt   = (ptw_acc_flt_pulse || ptw_pge_flt_pulse) && ptw_is_fetch
                        && ifu_mmu_va_vld && !ifu_mmu_abort && (ifu_req_vpn == ptw_vpn);
    wire dtlb_ptw_flt   = (ptw_acc_flt_pulse || ptw_pge_flt_pulse) && !ptw_is_fetch
                        && lsu_mmu_va_vld && !lsu_mmu_abort && (lsu_req_vpn == ptw_vpn);
    wire itlb_ptw_pgflt = itlb_ptw_flt && ptw_pge_flt_pulse;
    wire dtlb_ptw_pgflt = dtlb_ptw_flt && ptw_pge_flt_pulse;
    wire itlb_ptw_accflt = itlb_ptw_flt && ptw_acc_flt_pulse;
    wire dtlb_ptw_accflt = dtlb_ptw_flt && ptw_acc_flt_pulse;

    //=========================================================================
    // SECTION 7: THE ANSWER MUXES + THE PMP CHANNELS.
    //=========================================================================
    wire itlb_supv = (cp0_yy_priv_mode == PRIV_S);
    wire itlb_user = (cp0_yy_priv_mode == PRIV_U);
    wire itlb_hit_perm_pf = itlb_perm_flt(itlb_flg_r, itlb_supv, itlb_user);

    wire lsu_supv = (lsu_mmu_priv_mode == PRIV_S);
    wire lsu_user = (lsu_mmu_priv_mode == PRIV_U);
    wire dtlb_hit_perm_pf = dtlb_perm_flt(dtlb_flg_r, !lsu_mmu_st_inst, lsu_mmu_st_inst,
                                          lsu_supv, lsu_user, cp0_mmu_mxr, cp0_mmu_sum);

    // ---- PMP channels: ONE per I/D side, time-shared among mach-path,
    // TLB-hit live re-check, and the walker's per-level PT-page check
    // (mutually exclusive every cycle, SECTION 4's own note).
    //-------------------------------------------------------------------------
    wire [PC_WIDTH-1:0] ifu_mach_pa_full = {ifu_mmu_va[MMU_PA_WIDTH-1:0], 12'b0};
    wire [PC_WIDTH-1:0] itlb_hit_pa_full = {itlb_ppn_spliced, 12'b0};
    wire [PC_WIDTH-1:0] lsu_mach_pa_full = {lsu_mmu_va[MMU_PA_WIDTH-1:0], 12'b0};
    wire [PC_WIDTH-1:0] dtlb_hit_pa_full = {dtlb_ppn_spliced, 12'b0};

    assign mmu_pmp_fetch_pa = (ptw_pmp_beat && ptw_is_fetch) ? ptw_req_addr
                             : ifu_mach_path                 ? ifu_mach_pa_full
                             : itlb_hit_r                    ? itlb_hit_pa_full
                             :                                 {PC_WIDTH{1'b0}};
    assign mmu_pmp_fetch_vld = (ptw_pmp_beat && ptw_is_fetch)
                             || (ifu_mmu_va_vld && (ifu_mach_path || itlb_hit_r));

    assign mmu_pmp_data_pa = (ptw_pmp_beat && !ptw_is_fetch) ? ptw_req_addr
                            : lsu_mach_path                  ? lsu_mach_pa_full
                            : dtlb_hit_r                     ? dtlb_hit_pa_full
                            :                                  {PC_WIDTH{1'b0}};
    wire dtlb_load_now  = (ptw_pmp_beat && !ptw_is_fetch) ? ptw_is_load  : !lsu_mmu_st_inst;
    wire dtlb_store_now = (ptw_pmp_beat && !ptw_is_fetch) ? ptw_is_store : lsu_mmu_st_inst;
    assign mmu_pmp_load  = dtlb_load_now;
    assign mmu_pmp_store = dtlb_store_now;
    assign mmu_pmp_data_vld = (ptw_pmp_beat && !ptw_is_fetch)
                             || (lsu_mmu_va_vld && (lsu_mach_path || dtlb_hit_r));
    assign mmu_pmp_data_priv_mode = (ptw_pmp_beat && !ptw_is_fetch) ? ptw_priv_mode
                                                                     : lsu_mmu_priv_mode;

    wire itlb_pmp_deny_now = pmp_mmu_fetch_deny && (ifu_mach_path || itlb_hit_r);
    wire dtlb_pmp_deny_now = pmp_mmu_data_deny  && (lsu_mach_path || dtlb_hit_r);

    // ---- ITLB port (IFU) ----
    wire itlb_answer_now = ifu_mach_path || ifu_va_illegal_active
                          || itlb_hit_r || itlb_ptw_flt;

    assign mmu_ifu_pa_vld       = itlb_answer_now;
    assign mmu_ifu_pa           = ifu_mach_path ? ifu_mmu_va[MMU_PA_WIDTH-1:0]
                                                : itlb_ppn_spliced;
    assign mmu_ifu_access_fault = itlb_pmp_deny_now || itlb_ptw_accflt;

    wire ifu_pgflt_out = ifu_va_illegal_active
                       || (itlb_hit_r && itlb_hit_perm_pf)
                       || itlb_ptw_pgflt;

    // {pgflt, supv, ca, ba, sec} -- the mach-path branch is bit-exact with
    // the M2/M3 stub (supv permissively 1); the translated branch reports
    // the true current privilege and the cached entry's PMA.
    // itlb_flg_r[11:7] = pma[4:0] = {so,ca,buf,sh,sec} (SECTION 6.6's
    // ptw_ref_pma concatenation) -- ca=flg[10], buf(ba)=flg[9].
    assign mmu_ifu_prot = ifu_mach_path
        ? {1'b0, 1'b1, ifu_mach_ca, ifu_mach_attr[2], 1'b0}
        : {ifu_pgflt_out, !itlb_user, itlb_flg_r[10], itlb_flg_r[9], 1'b0};

    // ---- DTLB port (LSU) ----
    wire dtlb_answer_now = lsu_mach_path || lsu_va_illegal_active
                          || dtlb_hit_r || dtlb_ptw_flt;

    assign mmu_lsu_pa_vld = dtlb_answer_now;
    assign mmu_lsu_pa     = lsu_mach_path ? lsu_mmu_va[MMU_PA_WIDTH-1:0]
                                          : dtlb_ppn_spliced;
    assign mmu_lsu_page_fault   = lsu_va_illegal_active
                                 || (dtlb_hit_r && dtlb_hit_perm_pf)
                                 || dtlb_ptw_pgflt;
    assign mmu_lsu_access_fault = dtlb_pmp_deny_now || dtlb_ptw_accflt;

    assign mmu_lsu_ca  = lsu_mach_path ? lsu_mach_ca        : dtlb_flg_r[10];
    assign mmu_lsu_so  = lsu_mach_path ? lsu_mach_attr[4]   : dtlb_flg_r[11];
    assign mmu_lsu_buf = lsu_mach_path ? lsu_mach_attr[2]   : dtlb_flg_r[9];
    assign mmu_lsu_sec = lsu_mach_path ? lsu_mach_attr[0]   : dtlb_flg_r[7];
    assign mmu_lsu_sh  = lsu_mach_path ? lsu_mach_attr[1]   : dtlb_flg_r[8];

endmodule
