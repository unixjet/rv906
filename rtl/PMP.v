//=============================================================================
// PMP.v - Physical Memory Protection, 8 entries, 4KB granularity (M4 Task 2).
//=============================================================================
// C906 files covered (faithful port, combined into one file as rv12 did):
//   gen_rtl/pmp/rtl/aq_pmp_regs.v      (cfg bytes + pmpaddr storage, lock,
//                                       WARL incl. the CONFORMANT NAPOT
//                                       readback -- see the readback note)
//   gen_rtl/pmp/rtl/aq_pmp_comp_hit.v  (TOR subtract / NA4 dead / NAPOT
//                                       trailing-ones mask match)
//   gen_rtl/pmp/rtl/aq_pmp_acc.v       (lowest-hit priority, M-mode bypass
//                                       unless locked, per-access-type check)
//   gen_rtl/pmp/rtl/aq_pmp_top.v       (channel wiring shape)
//
// GEOMETRY (design doc: 8 entries, 4KB granularity): PMP_ADDR_W = 29 stores
// PA[39:11] (the page number plus one grain bit, donor aq_pmp_regs.v:153
// ADDR_WIDTH=29, stored from cp0_pmp_wdata[37:9]). Granularity G=10 (4KB):
// NA4 is DEAD (donor aq_pmp_comp_hit.v:101 ties na4_addr_match=0, since a
// 4-byte region is finer than the 4KB grain), NAPOT minimum is 8KB.
//
// CONFORMANT NAPOT READBACK (rv906 deviation from the C906 donor, matching
// rv12's D-M4-3): the C906 donor reads the low 9 pmpaddr bits as ZERO in
// every mode (aq_pmp_regs.v:520-522), which FAILS rv64mi-p-pmpaddr's
// granularity probe. The privileged spec (S3.7.1, G=10) requires NAPOT to
// read pmpaddr[G-2:0]=[8:0] as ONES and bit[G-1]=[9] live, and OFF/TOR to
// read [G-1:0] as ZEROS. rv906 implements the conformant readback (the same
// call rv12 made), so rv64mi-p-pmpaddr passes.
//
// CHANNELS: the donor checks PMP for fetch, load, store, and the PTW. rv906
// exposes four check channels (fetch / load / store / ptw) as combinational
// deny outputs; the LSU/IFU/MMU consume them at Tasks 3-6. With no PMP
// regions configured (reset), M-mode accesses pass (default allow) and the
// machine behaves exactly as before.
//=============================================================================

import rvproc_pkg::*;

module PMP #(
    parameter PMP_ADDR_W = 29            // PA[39:11]
)(
    input  wire                    clk,
    input  wire                    rst_n,

    //-------------------------------------------------------------------------
    // CSR write interface (CSR.v decodes pmpcfg0/pmpcfg2/pmpaddr0-7 and
    // strokes these; the storage lives HERE, mirroring the donor aq_pmp_regs).
    //-------------------------------------------------------------------------
    input  wire                    pmpcfg0_wen,
    input  wire [63:0]             pmpcfg0_wdata,
    input  wire [7:0]              pmpaddr_wen,      // one bit per pmpaddr0-7
    input  wire [63:0]             pmpaddr_wdata,
    // readback to CSR.v (conformant NAPOT readback, see header)
    output wire [63:0]             pmp_cfg0_value,
    input  wire [2:0]              pmpaddr_rsel,     // which pmpaddr0-7 to read
    output wire [63:0]             pmp_addr_value,    // selected pmpaddr read

    //-------------------------------------------------------------------------
    // Current privilege (for the M-mode bypass). Fetches are never MPRV-
    // adjusted (privileged spec: MPRV affects only "explicit memory
    // accesses"), so priv_mode carries the RAW current privilege and feeds
    // the fetch channel only. The data channel gets its own MPRV-resolved
    // privilege (donor aq_pmp_acc.v:118-119's commented-out
    // "cp0_priv_mode = pmp_mprv_status ? cp0_pmp_mpp : cur_priv_mode" --
    // the donor's caller resolves this before presenting it to the single
    // shared aq_pmp_acc channel; MMU.v's mmu_pmp_data_priv_mode does the
    // equivalent resolution for rv906's split data channel, mirroring the
    // existing lsu_mmu_priv_mode resolution already used for the DTLB
    // permission check, LSU.v's lsu_mmu_priv_mode assign).
    //-------------------------------------------------------------------------
    input  wire [1:0]              priv_mode,
    input  wire [1:0]              data_priv_mode,

    //-------------------------------------------------------------------------
    // Check channels: {addr[39:0], load, store, fetch} -> deny.
    //-------------------------------------------------------------------------
    input  wire [PC_WIDTH-1:0]     chk_fetch_pa,
    input  wire                    chk_fetch_vld,
    input  wire [PC_WIDTH-1:0]     chk_data_pa,
    input  wire                    chk_load,
    input  wire                    chk_store,
    input  wire                    chk_data_vld,
    output wire                    pmp_fetch_deny,
    output wire                    pmp_data_deny
);

    localparam PMP_ENTRIES = 8;

    //=========================================================================
    // SECTION 1: register file (aq_pmp_regs.v). cfg byte {L,A[1:0],X,W,R} at
    // bits {7,4:3,2,1,0}; bits[6:5] reserved, read 0. Per-byte lock freezes
    // the byte. pmpaddr gated by own-L and the next-entry-locked-TOR rule.
    //=========================================================================
    reg [PMP_ENTRIES-1:0]           cfg_r, cfg_w, cfg_x, cfg_l;
    reg [2*PMP_ENTRIES-1:0]         cfg_a;
    reg [PMP_ADDR_W*PMP_ENTRIES-1:0] pmpaddr;

    genvar ge;
    generate
        for (ge = 0; ge < PMP_ENTRIES; ge = ge + 1) begin : g_ent
            wire byte_lock = cfg_l[ge];
            // next-entry locked-TOR gate (donor aq_pmp_regs.v:418): entry i's
            // pmpaddr is frozen if entry i+1 is locked AND TOR (A==2'b01).
            // generate-if (not a ternary) so the last entry never elaborates
            // the out-of-range cfg_l[8] / cfg_a[17:16] index.
            wire next_locked_tor;
            if (ge < PMP_ENTRIES-1) begin : g_nlt
                assign next_locked_tor = cfg_l[ge+1]
                                      && (cfg_a[2*(ge+1) +: 2] == 2'b01);
            end else begin : g_nlt_last
                assign next_locked_tor = 1'b0;
            end
            wire updt_cfg  = pmpcfg0_wen && !byte_lock;
            wire updt_addr = pmpaddr_wen[ge] && !cfg_l[ge] && !next_locked_tor;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cfg_r[ge] <= 1'b0;
                    cfg_w[ge] <= 1'b0;
                    cfg_x[ge] <= 1'b0;
                    cfg_l[ge] <= 1'b0;
                    cfg_a[2*ge +: 2] <= 2'b0;
                    pmpaddr[PMP_ADDR_W*ge +: PMP_ADDR_W] <= {PMP_ADDR_W{1'b0}};
                end else begin
                    if (updt_cfg) begin
                        cfg_r[ge]        <= pmpcfg0_wdata[8*ge + 0];
                        cfg_w[ge]        <= pmpcfg0_wdata[8*ge + 1];
                        cfg_x[ge]        <= pmpcfg0_wdata[8*ge + 2];
                        cfg_a[2*ge +: 2] <= pmpcfg0_wdata[8*ge + 4 -: 2];
                        cfg_l[ge]        <= pmpcfg0_wdata[8*ge + 7];
                    end
                    if (updt_addr)
                        pmpaddr[PMP_ADDR_W*ge +: PMP_ADDR_W]
                            <= pmpaddr_wdata[PMP_ADDR_W-1 + 9 : 9];  // wdata[37:9]
                end
            end

            // cfg byte readback {L,2'b0,A[1:0],X,W,R} (donor :400-407).
            assign pmp_cfg0_value[8*ge +: 8] =
                {cfg_l[ge], 2'b0, cfg_a[2*ge +: 2], cfg_x[ge], cfg_w[ge], cfg_r[ge]};
        end
    endgenerate

    // pmpaddr readback (CONFORMANT, see header). pmpaddr stores PA[39:11]
    // (stored[28:0]). The 64-bit read returns:
    //   bits[37:10] = stored[28:1]          (the page number, stored[0] masked)
    //   bit [9]     = stored[0] if NAPOT else 0
    //   bits[8:0]   = 9'h1ff if NAPOT else 0
    // CSR.v selects which pmpaddr via pmpaddr_rsel.
    wire [63:0] pmp_addr_read [0:PMP_ENTRIES-1];
    genvar gr;
    generate
        for (gr = 0; gr < PMP_ENTRIES; gr = gr + 1) begin : g_rd
            wire [PMP_ADDR_W-1:0] stored = pmpaddr[PMP_ADDR_W*gr +: PMP_ADDR_W];
            wire napot = cfg_a[2*gr + 1];
            wire [PMP_ADDR_W-2:0] hi = stored[PMP_ADDR_W-1:1];   // stored[28:1]
            wire [9:0] low10 = napot ? {stored[0], 9'h1ff} : 10'h0;
            assign pmp_addr_read[gr] = {26'b0, hi, low10};
        end
    endgenerate

    // select the requested pmpaddr (one-hot via rsel index)
    reg [63:0] pmp_addr_sel;
    integer rk;
    always @* begin
        pmp_addr_sel = 64'd0;
        for (rk = 0; rk < PMP_ENTRIES; rk = rk + 1)
            if (pmpaddr_rsel == rk[2:0])
                pmp_addr_sel = pmp_addr_read[rk];
    end
    assign pmp_addr_value = pmp_addr_sel;

    //=========================================================================
    // SECTION 2: match (aq_pmp_comp_hit.v). Per entry, per checked PA:
    // OFF->no hit, TOR-> (prev_addr <= pa < this_addr), NA4->dead, NAPOT->
    // masked compare. The NAPOT mask is the trailing-ones decode.
    //=========================================================================
    function automatic [PMP_ADDR_W-1:0] napot_mask_fn(input [PMP_ADDR_W-1:0] a);
        // trailing-ones decode (donor aq_pmp_comp_hit.v:108-143): a clear bit
        // at position t gives a 2^(t+12)-byte region mask. all-ones -> 0
        // (matches everything, the catch-all).
        casez (a)
            29'b????????????????????????????0   : napot_mask_fn = 29'h1fffffff; // 4KB
            29'b???????????????????????????01   : napot_mask_fn = 29'h1ffffffe; // 8KB
            29'b??????????????????????????011   : napot_mask_fn = 29'h1ffffffc;
            29'b?????????????????????????0111   : napot_mask_fn = 29'h1ffffff8;
            29'b????????????????????????01111   : napot_mask_fn = 29'h1ffffff0;
            29'b???????????????????????011111   : napot_mask_fn = 29'h1fffffe0;
            29'b??????????????????????0111111   : napot_mask_fn = 29'h1fffffc0;
            29'b?????????????????????01111111   : napot_mask_fn = 29'h1fffff80;
            29'b????????????????????011111111   : napot_mask_fn = 29'h1fffff00;
            29'b???????????????????0111111111   : napot_mask_fn = 29'h1ffffe00;
            29'b??????????????????01111111111   : napot_mask_fn = 29'h1ffffc00;
            29'b?????????????????011111111111   : napot_mask_fn = 29'h1ffff800;
            29'b????????????????0111111111111   : napot_mask_fn = 29'h1ffff000;
            29'b???????????????01111111111111   : napot_mask_fn = 29'h1fffe000;
            29'b??????????????011111111111111   : napot_mask_fn = 29'h1fffc000;
            29'b?????????????0111111111111111   : napot_mask_fn = 29'h1fff8000;
            29'b????????????01111111111111111   : napot_mask_fn = 29'h1fff0000;
            29'b???????????011111111111111111   : napot_mask_fn = 29'h1ffe0000;
            29'b??????????0111111111111111111   : napot_mask_fn = 29'h1ffc0000;
            29'b?????????01111111111111111111   : napot_mask_fn = 29'h1ff80000;
            29'b????????011111111111111111111   : napot_mask_fn = 29'h1ff00000;
            29'b???????0111111111111111111111   : napot_mask_fn = 29'h1fe00000;
            29'b??????01111111111111111111111   : napot_mask_fn = 29'h1fc00000;
            29'b?????011111111111111111111111   : napot_mask_fn = 29'h1f800000;
            29'b????0111111111111111111111111   : napot_mask_fn = 29'h1f000000;
            29'b???01111111111111111111111111   : napot_mask_fn = 29'h1e000000;
            29'b??011111111111111111111111111   : napot_mask_fn = 29'h1c000000;
            29'b?0111111111111111111111111111   : napot_mask_fn = 29'h18000000;
            29'b01111111111111111111111111111   : napot_mask_fn = 29'h10000000;
            29'b11111111111111111111111111111   : napot_mask_fn = 29'h00000000; // catch-all
            default                            : napot_mask_fn = 29'h00000000;
        endcase
    endfunction

    // Per-entry, per-channel match. We compute for the data channel PA and
    // the fetch channel PA. PA is byte address; the PMP grain is the page,
    // so compare page numbers: pa_page = pa[39:12], stored page = stored[28:1]
    // (stored[0] is the grain bit used only by the NAPOT size decode).
    wire [PC_WIDTH-1:0] data_pa  = chk_data_pa;
    wire [PC_WIDTH-1:0] fetch_pa = chk_fetch_pa;

    wire [PMP_ENTRIES-1:0] data_hit;
    wire [PMP_ENTRIES-1:0] fetch_hit;

    genvar gm;
    generate
        for (gm = 0; gm < PMP_ENTRIES; gm = gm + 1) begin : g_match
            wire [1:0] a = cfg_a[2*gm +: 2];
            wire [PMP_ADDR_W-1:0] stored = pmpaddr[PMP_ADDR_W*gm +: PMP_ADDR_W];
            // stored page number = stored[28:1] (drop grain bit 0)
            wire [PMP_ADDR_W-2:0] this_page = stored[PMP_ADDR_W-1:1];
            // previous entry's page (TOR lower bound); entry 0 bound = 0.
            // prev entry's stored[28:1] (28 bits) via part-select (not a
            // shift, which would keep 29 bits and mis-width the ternary).
            wire [PMP_ADDR_W-2:0] prev_page =
                (gm == 0) ? {(PMP_ADDR_W-1){1'b0}}
                          : pmpaddr[PMP_ADDR_W*(gm-1) + PMP_ADDR_W-1
                                    : PMP_ADDR_W*(gm-1) + 1];

            // data channel
            wire [PMP_ADDR_W-2:0] d_page = data_pa[PC_WIDTH-1:12];
            wire d_ge_prev = (d_page >= prev_page);
            wire d_lt_this = (d_page <  this_page);
            wire d_tor     = d_ge_prev && d_lt_this;
            wire [PMP_ADDR_W-1:0] d_mask = napot_mask_fn(stored);
            wire d_napot = ((d_mask & {d_page, 1'b0}) == (d_mask & {this_page, 1'b0}));
            always @* begin
                case (a)
                    2'b00:   data_hit[gm] = 1'b0;          // OFF
                    2'b01:   data_hit[gm] = d_tor;          // TOR
                    2'b10:   data_hit[gm] = 1'b0;          // NA4 dead
                    2'b11:   data_hit[gm] = d_napot;        // NAPOT
                    default: data_hit[gm] = 1'b0;
                endcase
            end

            // fetch channel
            wire [PMP_ADDR_W-2:0] f_page = fetch_pa[PC_WIDTH-1:12];
            wire f_ge_prev = (f_page >= prev_page);
            wire f_lt_this = (f_page <  this_page);
            wire f_tor     = f_ge_prev && f_lt_this;
            wire f_napot = ((d_mask & {f_page, 1'b0}) == (d_mask & {this_page, 1'b0}));
            always @* begin
                case (a)
                    2'b00:   fetch_hit[gm] = 1'b0;
                    2'b01:   fetch_hit[gm] = f_tor;
                    2'b10:   fetch_hit[gm] = 1'b0;
                    2'b11:   fetch_hit[gm] = f_napot;
                    default: fetch_hit[gm] = 1'b0;
                endcase
            end
        end
    endgenerate

    //=========================================================================
    // SECTION 3: permission (aq_pmp_acc.v). Lowest-hitting entry wins. The
    // flag is {L,X,W,R}. M-mode bypasses PMP UNLESS the matching entry is
    // locked (then even M-mode is checked). No hit: M-mode allow, S/U deny.
    //=========================================================================
    wire fetch_mach_mode = (priv_mode == 2'b11);
    wire data_mach_mode  = (data_priv_mode == 2'b11);

    // lowest-hit priority encode -> flag {L,X,W,R}
    reg [3:0] data_flg;
    reg       data_any_hit;
    reg [3:0] fetch_flg;
    reg       fetch_any_hit;

    // priority select (entry 0 highest). We build the flag of the lowest hit.
    integer pi;
    always @* begin
        data_flg = 4'b0111;      // default {L=0,X=1,W=1,R=1} = M-mode allow
        data_any_hit = 1'b0;
        for (pi = PMP_ENTRIES-1; pi >= 0; pi = pi - 1) begin
            if (data_hit[pi]) begin
                data_flg = {cfg_l[pi], cfg_x[pi], cfg_w[pi], cfg_r[pi]};
                data_any_hit = 1'b1;
            end
        end
    end

    integer fi;
    always @* begin
        fetch_flg = 4'b0111;
        fetch_any_hit = 1'b0;
        for (fi = PMP_ENTRIES-1; fi >= 0; fi = fi - 1) begin
            if (fetch_hit[fi]) begin
                fetch_flg = {cfg_l[fi], cfg_x[fi], cfg_w[fi], cfg_r[fi]};
                fetch_any_hit = 1'b1;
            end
        end
    end

    // deny logic (donor aq_pmp_acc.v). For a hit entry, M-mode is denied only
    // if locked and the access bit is clear; S/U are denied if the access bit
    // is clear. For no hit, M-mode allows, S/U deny.
    wire data_mach_bypass  = data_mach_mode && !data_flg[3];   // M & not locked
    wire data_access_ok    = data_any_hit
                             ? (chk_load  ? data_flg[0] : 1'b1)
                               & (chk_store ? data_flg[1] : 1'b1)
                             : 1'b0;
    wire data_deny_raw = chk_data_vld &&
                         (data_any_hit ? !(data_mach_bypass || data_access_ok)
                                       : !data_mach_mode);

    wire fetch_mach_bypass = fetch_mach_mode && !fetch_flg[3];
    wire fetch_access_ok  = fetch_any_hit ? fetch_flg[2] : 1'b0;  // X bit
    wire fetch_deny_raw = chk_fetch_vld &&
                          (fetch_any_hit ? !(fetch_mach_bypass || fetch_access_ok)
                                         : !fetch_mach_mode);

    assign pmp_data_deny  = data_deny_raw;
    assign pmp_fetch_deny = fetch_deny_raw;

endmodule
