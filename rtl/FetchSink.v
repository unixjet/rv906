//=============================================================================
// FetchSink.v - M1 SCAFFOLDING: fake BJU + fake RTU + CP0 stand-in +
//               harness config bank
//=============================================================================
// C906 files covered (the consumer side this fake stands in for):
//   gen_rtl/idu/rtl/aq_idu_top.v (ports)   (the single-instruction IFU->IDU
//                                           interface, IFU notes S2 item 6)
//   gen_rtl/iu/rtl/aq_iu_bju.v             (resolve: chgflw / bht / RAS
//                                           update signals -- read directly
//                                           in Task 4.1, see FINDINGS below)
//   gen_rtl/idu/rtl/aq_idu_id_decd.v,      (the FUNC_* opcode encoding table
//   gen_rtl/idu/rtl/aq_idu_cfig.h           that lets aq_iu_bju.v's classify-
//                                           ing bits be read at all -- see
//                                           FINDINGS below)
//   gen_rtl/rtu/rtl/aq_rtu_*.v              (flush; C906 has no per-slot
//                                           retire bus to model here since
//                                           only one instruction commits per
//                                           cycle -- simpler than C910's
//                                           3-slot retire, design doc S4.1)
//   rtl/TestMaster.v                       (M0's proven D-side AXI write
//                                           FSM, lifted verbatim below)
// References: design doc S4.1 (the FULL behavioural contract, implemented
// verbatim below), plan "Global contracts" (JR_TARGET, direction rule,
// shadow call stack, harness config mechanism, RAS-faithful grading path).
//
// This module is DELETED at M2: the IDU/IU/RTU take over its interfaces
// unchanged. Nothing here models C906 microarchitecture -- it is an oracle.
//
// CP0 STAND-IN: in the real RTL, `cp0_ifu_*` chicken bits and invalidate
// requests are driven by CP0 directly to ICache.v/BPU.v (IFU notes S5.1;
// BPU notes S8) -- NOT through IDU/IU/RTU. Since there is no CP0 module in
// M1, FetchSink hosts that config bank too (plan "Global contracts": harness
// config mechanism) and RVProc.v fans its outputs straight to ICache.v/
// BPU.v/IFU.v, mirroring the real direct-fan-out topology.
//
//=============================================================================
// TASK 4.1 FINDINGS (read this before touching the resolve logic below)
//=============================================================================
// 1. SIGNAL NAMES: aq_iu_bju.v's own IFU-facing output ports are, verbatim:
//      iu_ifu_tar_pc_vld, iu_ifu_tar_pc[63:0], iu_ifu_pc_mispred,
//      iu_ifu_bht_mispred, iu_ifu_br_vld, iu_ifu_bht_taken,
//      iu_ifu_bht_pred[1:0], iu_ifu_link_vld, iu_ifu_ret_vld
//    and its IFU-facing INPUT ports are, verbatim:
//      ifu_iu_chgflw_vld, ifu_iu_chgflw_pc[39:0]
//    Every one of these EXACTLY MATCHES the name already sitting on
//    FetchSink's frozen port list (Task 1's placeholder guess). NO MISMATCH
//    was found -- unlike the plan's cautionary note ("do not assume C910's
//    iu_ifu_bht_check_vld/iu_ifu_chgflw_* naming"), C906's real names happen
//    to already be what Task 1 guessed. Nothing to rename.
//
// 2. pcall/preturn/ind_br CLASSIFICATION (confirmed from aq_iu_bju.v:637-667,
//    cross-checked against the FUNC_* encoding table in
//    idu/rtl/aq_idu_cfig.h:402-450 -- the classification LOGIC itself lives
//    in the IU/BJU file, not in aq_idu_id_decd.v's 6662-line opcode-to-FUNC
//    table body; aq_idu_id_decd.v's only contribution is emitting the FUNC_*
//    code that aq_iu_bju.v later tests bit[11] of):
//      bju_link_vld_raw (=~pcall) = dst_preg==x1 && uncond_sel && func[11]
//      bju_ret_vld_raw  (=~preturn) = src0_reg==x1 && inst_jalr &&
//                                     !(src0_reg==dst_preg && func[11])
//      bju_pc_reg_mispred (forces a redirect, ~ind_br) = inst_jalr &&
//                                     src0_reg != x1
//    CONFIRMED, FLAGGED PROMINENTLY: C906 checks dst_preg/src0_reg against
//    x1 EXACTLY -- there is NO x5 alternate link register the way some
//    conventions (and rv12's own C910 clone, `(x[11:7]==5'd1)||(x[11:7]==
//    5'd5)`) allow. This is a real, confirmed divergence from C910's
//    register-set convention, exactly the kind of thing the plan's "verify
//    it during Task 5.1" note anticipated -- found here in Task 4.1 instead
//    since FetchSink's own classification needed it first. `jal x5, ...` or
//    `jalr x0, x5, 0` do NOT push/pop the shadow stack on rv906/C906; only
//    x1 does. func[11] (aq_idu_cfig.h) is 1 for JAL/JALR/C.JALR and 0 for
//    C.J/C.JR (FUNC_JAL=12'b100100100001, FUNC_JALR=FUNC_C_JALR=
//    12'b100000100010, FUNC_C_J=12'b000100100001, FUNC_C_JR=
//    12'b000000100010 -- bit 11 is the only bit that differs between the
//    two pairs), i.e. "this uncond jump is eligible to write a link" --
//    always true for JAL/JALR/C.JALR and always false for C.J/C.JR (whose
//    rd is hardwired x0 by the encoding anyway, so the dst_preg==x1 check
//    alone would already exclude them; func[11] is a second, redundant gate
//    in the real RTL, reproduced here for traceability).
//    The "jalr x1, x1, 0" dual case (rd==rs1==x1) is explicitly EXCLUDED
//    from preturn by the `!(src0_reg==dst_preg && func[11])` term -- real
//    hardware treats it as a plain call (RAS push only), never a
//    push-then-pop. Because rv906 has only ONE link register (not C910's
//    x1/x5 pair), this is the ONLY way a single jalr can appear to satisfy
//    both push and pop conditions at once, and the real RTL's own exclusion
//    term already prevents it -- FetchSink's shadow-stack update below
//    therefore never needs a genuine "push and pop the same cycle" case
//    (see the SHADOW STACK section for where rv12's C910 clone needed one
//    and why rv906 does not).
//    ACTUAL TARGET for jalr-family instructions: real hardware always
//    computes this from the ALU (rs1 + imm), which FetchSink cannot do (no
//    register file exists in M1). For preturn, the shadow stack pop is the
//    obvious deterministic substitute. For every OTHER jalr-family case --
//    genuine indirect jumps (rs1 != x1) AND the rare "indirect call through
//    a register" case (rd==x1 but rs1 != x1, or the excluded dual case) --
//    there is no register file to consult either, so this module uses the
//    same JR_TARGET(pc) formula for all of them. This is a documented Task
//    4.1 design choice, not something aq_iu_bju.v itself specifies (real
//    hardware would compute an exact address in every case); confirm during
//    Task 6 bring-up that the directed test suite's "indirect call through a
//    register" scenarios (if any) actually land at JR_TARGET(pc) too.
//
// 3. SQUASH WINDOW: IFU.v's flush network (`ibuf_flush_en`, `ipack_buf_
//    flush`, `ctrl_if_cancel`) is wired directly and combinationally off
//    `iu_ifu_tar_pc_vld` (IFU.v SECTION IBUF/IPACK/CTRL), so in principle one
//    cycle of already-buffered (wrong-path) IBUF content can still be
//    presented on `ifu_idu_id_inst_vld` in the SAME cycle the redirect first
//    becomes visible to IFU (a one-cycle "stale delivery" window, the same
//    structural fact rv12's C910 clone documented for its own, differently-
//    shaped IBUF). FetchSink below uses a conservative 2-cycle squash window
//    (discard, don't commit, anything delivered for 2 cycles after a
//    redirect fires) to cover this plus a margin for ICache/IPACK pipeline
//    residue this module cannot see from its own ports. This constant is a
//    Task 4 placeholder, not a proven bound -- Task 6's integration bring-up
//    ("this is where pipeline bugs die") is where it gets validated or
//    retuned against the real, wired-together pipeline; Task 4.3's unit
//    bench below only proves FetchSink honors ITS OWN stated squash-window
//    contract in isolation, not that the window's length is exactly right
//    for IFU.v's real timing.
//=============================================================================

import rvproc_pkg::*;

module FetchSink #(
    parameter DATA_WIDTH  = 512,
    parameter ADDR_WIDTH  = 64,
    // tohost line; must agree with test/m1/common.ld and the testbench's ELF
    // symbol lookup (docs/08-verification.md M1 section, Task 5).
    parameter [63:0] TOHOST_ADDR  = ADDR_TOHOST,
    parameter [63:0] RESET_VECTOR = 64'h0000_0000_8000_0000
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IDU side: consume the IFU's single-instruction delivery (IFU notes
    // S2 item 6, S5.2)
    //=========================================================================
    input  wire [31:0]              ifu_idu_id_inst,
    input  wire                     ifu_idu_id_inst_vld,
    input  wire [1:0]               ifu_idu_id_bht_pred,
    output wire                     idu_ifu_id_stall,

    //=========================================================================
    // Fake BJU (design doc S4.1): registered resolve, direction rule
    // ^pc[7:4], decoded B/CB/J/CJ immediates, shadow call stack, JR_TARGET
    // formula. `iu_ifu_br_vld` fires for EVERY resolved conditional branch;
    // the mispredict-only signals gate PCGEN's redirect and RAS's pointer
    // snap-back.
    //=========================================================================
    output wire                     iu_ifu_tar_pc_vld,
    output wire [63:0]              iu_ifu_tar_pc,
    output wire                     iu_ifu_pc_mispred,
    output wire                     iu_ifu_bht_mispred,
    output wire                     iu_ifu_br_vld,
    output wire                     iu_ifu_bht_taken,
    output wire [1:0]               iu_ifu_bht_pred,
    output wire                     iu_ifu_link_vld,
    output wire                     iu_ifu_ret_vld,
    input  wire                     ifu_iu_chgflw_vld,      // IFU forwards RTU's redirect
    input  wire [PC_WIDTH-1:0]      ifu_iu_chgflw_pc,

    //=========================================================================
    // Fake RTU (design doc S4.1): front-end flush only -- C906 delivers one
    // instruction/cycle, so there is no per-slot retire bus to drive here,
    // unlike C910's 3-slot rtughr/RAS-mirror retire fan-out.
    //=========================================================================
    output wire                     rtu_ifu_chgflw_vld,
    output wire [PC_WIDTH-1:0]      rtu_ifu_chgflw_pc,
    output wire                     rtu_ifu_flush_fe,

    //=========================================================================
    // CP0 stand-in: ICache-facing chicken bits + invalidate (see module
    // header). Ports mirror ICache.v's cp0_ifu_* group exactly.
    //=========================================================================
    output wire                     cp0_ifu_icache_en,
    output wire                     cp0_ifu_iwpe,
    output wire                     cp0_ifu_icache_pref_en,
    output wire [63:0]              cp0_ifu_icache_inv_addr,
    output wire                     cp0_ifu_icache_inv_req,
    output wire [1:0]               cp0_ifu_icache_inv_type,
    input  wire                     ifu_cp0_icache_inv_done,

    //=========================================================================
    // CP0 stand-in: BPU-facing chicken bits + invalidate. Ports mirror
    // BPU.v's cp0_ifu_* group exactly. rung bits (--m1-rung=<1..4>, plan
    // "Global contracts") select which of these three are asserted.
    //=========================================================================
    output wire                     cp0_ifu_bht_en,
    output wire                     cp0_ifu_btb_en,
    output wire                     cp0_ifu_ras_en,
    output wire                     cp0_ifu_bht_inv,
    output wire                     cp0_ifu_btb_clr,
    input  wire                     bht_cp0_inv_done,       // PLACEHOLDER, see BPU.v header

    //=========================================================================
    // CP0 stand-in: boot / reset vector (IFU.v's only CP0-shaped input)
    //=========================================================================
    output wire [PC_WIDTH-1:0]      cp0_xx_mrvbr,

    //=========================================================================
    // D-side AXI write master (ch[1]) - tohost reporting only.
    // Port group copied from TestMaster.v's axi_d_* list; plan Task 4.1
    // lifts its write FSM verbatim (concurrent AW+W, BVALID watched every
    // cycle of the write state).
    //=========================================================================
    output wire                     axi_d_awvalid,
    input  wire                     axi_d_awready,
    output wire [ADDR_WIDTH-1:0]    axi_d_awaddr,
    output wire [7:0]               axi_d_awlen,
    output wire [2:0]               axi_d_awsize,
    output wire [1:0]               axi_d_awburst,
    output wire [3:0]               axi_d_awcache,
    output wire [2:0]               axi_d_awprot,

    output wire                     axi_d_wvalid,
    input  wire                     axi_d_wready,
    output wire [DATA_WIDTH-1:0]    axi_d_wdata,
    output wire [DATA_WIDTH/8-1:0]  axi_d_wstrb,
    output wire                     axi_d_wlast,

    input  wire                     axi_d_bvalid,
    output wire                     axi_d_bready,
    input  wire [1:0]               axi_d_bresp,

    output wire                     axi_d_arvalid,
    input  wire                     axi_d_arready,
    output wire [ADDR_WIDTH-1:0]    axi_d_araddr,
    output wire [7:0]               axi_d_arlen,
    output wire [2:0]               axi_d_arsize,
    output wire [1:0]               axi_d_arburst,
    output wire [3:0]               axi_d_arcache,
    output wire [2:0]               axi_d_arprot,

    input  wire                     axi_d_rvalid,
    output wire                     axi_d_rready,
    input  wire [DATA_WIDTH-1:0]    axi_d_rdata,
    input  wire [1:0]               axi_d_rresp,
    input  wire                     axi_d_rlast
);

    //=========================================================================
    // SECTION: CONFIG BANK  (plan "Global contracts": harness config bank;
    // frozen from Task 1, unchanged here)
    //=========================================================================
    // THESE REGISTERS HAVE NO RTL DRIVER ON PURPOSE. RVProcTest.cpp pokes
    // them through the verisim.h signal paths after dut.init() and before
    // the first step (rung selection, stall mode, instruction budget), and
    // PULSES the cfg_*_inv/cfg_btb_clr bits mid-test for --inv-test.
    // The simulator zero-initializes them, so an un-poked run is rung 1
    // (every predictor off, ICache off) -- matching the ladder's floor.
    // Marked verilator public so the flattened model keeps them addressable.
    reg        cfg_icache_en       /* verilator public */;
    reg        cfg_iwpe            /* verilator public */;  // stays 0 in M1
    reg        cfg_icache_pref_en  /* verilator public */;
    reg        cfg_bht_en          /* verilator public */;  // ladder rung 4
    reg        cfg_btb_en          /* verilator public */;  // ladder rung 3
    reg        cfg_ras_en          /* verilator public */;  // ladder rung 2
    reg        cfg_icache_inv      /* verilator public */;  // --inv-test pulses
    reg        cfg_bht_inv         /* verilator public */;
    reg        cfg_btb_clr         /* verilator public */;
    reg        cfg_sink_stall      /* verilator public */;  // --sink-stall
    reg [31:0] cfg_max_insts       /* verilator public */;  // --max-insts

    assign cp0_ifu_icache_en      = cfg_icache_en;
    assign cp0_ifu_iwpe           = cfg_iwpe;
    assign cp0_ifu_icache_pref_en = cfg_icache_pref_en;
    assign cp0_ifu_icache_inv_addr= 64'd0;
    assign cp0_ifu_icache_inv_req = cfg_icache_inv;
    assign cp0_ifu_icache_inv_type= 2'd0;   // INV_ALL only in M1

    assign cp0_ifu_bht_en = cfg_bht_en;
    assign cp0_ifu_btb_en = cfg_btb_en;
    assign cp0_ifu_ras_en = cfg_ras_en;
    assign cp0_ifu_bht_inv= cfg_bht_inv;
    assign cp0_ifu_btb_clr= cfg_btb_clr;

    assign cp0_xx_mrvbr = RESET_VECTOR[PC_WIDTH-1:0];

    // The invalidate-done replies and IU/RTU's own forwarded-chgflw inputs
    // are read by nobody in M1: the harness polls invalidate completion
    // through verisim.h directly (rv12 precedent, see that project's
    // verisim.h header), and rtu_ifu_chgflw_vld is tied 0 below so
    // ifu_iu_chgflw_vld/_pc (which only ever mirror an RTU-sourced redirect
    // back out, IFU.v pcgen.v:325-326) can never carry anything but the
    // reset value. Genuinely unused, not an oversight.
    wire _cp0_unused_ok = ifu_cp0_icache_inv_done | bht_cp0_inv_done;

    //=========================================================================
    // SECTION: HELPERS  (plan Task 4.1)
    //=========================================================================
    localparam [31:0] SENTINEL = 32'h0000_006F;   // jal x0, 0

    // JR_TARGET (plan "Global contracts"): a 64-byte-aligned landing pad
    // 64..256 bytes ahead of the indirect jump. pc is the BYTE address; the
    // .S generator and the C++ ISS place/replay pads at exactly these
    // addresses.
    //   JR_TARGET(pc) = (pc & ~0x3F) + 0x40 + (((pc >> 6) & 3) << 6)
    function automatic [PC_WIDTH-1:0] jr_target;
        input [PC_WIDTH-1:0] pc;
        reg [2:0] blocks;                 // 1..4, i.e. 64..256 bytes ahead
        begin
            blocks    = {1'b0, pc[7:6]} + 3'd1;
            jr_target = {pc[PC_WIDTH-1:6], 6'b0} +
                        {{(PC_WIDTH-9){1'b0}}, blocks, 6'b0};
        end
    endfunction

    // Instruction classification. is32/con_br/ab_br/jalr_fam and the B/CB/
    // J/CJ immediate decode below are plain RV64GC ISA facts (opcode/funct3/
    // compressed-quadrant bit patterns), identical across C906 and C910 --
    // reused verbatim from rv12's own FetchSink.v decode (style reference).
    // pcall/preturn/pc_reg_mis are NOT reused from rv12: they encode the
    // x1-only classification confirmed in aq_iu_bju.v (TASK 4.1 FINDINGS
    // item 2 above), not C910's x1-or-x5 convention.
    typedef struct packed {
        logic                is32;
        logic                con_br;     // B-type | c.beqz | c.bnez
        logic                ab_br;      // jal | c.j          ("always taken")
        logic                jalr_fam;   // jalr | c.jr | c.jalr
        logic                pcall;      // RAS push (bju_link_vld_raw)
        logic                preturn;    // RAS pop  (bju_ret_vld_raw)
        logic                pc_reg_mis; // bju_pc_reg_mispred, literal (jalr-family, rs1 != x1)
        logic                sentinel;   // jal x0, 0
        logic [PC_WIDTH-1:0] imm;        // sign-extended BYTE offset (B/CB/J/CJ)
    } fsdec_t;

    function automatic fsdec_t fs_decode;
        input [31:0] x;
        reg jal32, jalr32, cjr, cjalr, cjump, cbz, bxx;
        reg [4:0] dst_preg, src0_reg;
        reg func11, jalr_fam, src_dst_eq;
        begin
            jal32  =  (x[6:0] == 7'b1101111);
            jalr32 = ({x[14:12], x[6:0]} == 10'b000_1100111);
            cjr    = ({x[15:12], x[6:0]} == 11'b1000_0000010) && (x[11:7] != 5'b0);
            cjalr  = ({x[15:12], x[6:0]} == 11'b1001_0000010) && (x[11:7] != 5'b0);

            // ct_ifu_precode-style "br" bit union, ISA-standard patterns.
            bxx    = (x[6:0] == 7'b1100011) && (x[14:13] != 2'b01);
            cbz    = ({x[15:14], x[1:0]} == 4'b1101);
            cjump  = ({x[15:13], x[1:0]} == 5'b10101);

            fs_decode.is32     = (x[1:0] == 2'b11);
            fs_decode.con_br   = bxx || cbz;
            fs_decode.ab_br    = jal32 || cjump;
            jalr_fam           = jalr32 || cjr || cjalr;
            fs_decode.jalr_fam = jalr_fam;

            // Register fields, standard RISC-V positions. For c.jr/c.jalr,
            // x[11:7] is rs1, NOT rd (rd is implicit x0/x1 respectively) --
            // reproduced per aq_iu_bju.v's own treatment (FINDINGS item 2).
            dst_preg = jal32  ? x[11:7] :
                       jalr32 ? x[11:7] :
                       cjalr  ? 5'd1    : 5'd0;
            src0_reg = jalr32          ? x[19:15] :
                       (cjr || cjalr)  ? x[11:7]  : 5'd0;

            // aq_idu_cfig.h FUNC_* table bit 11 ("link-eligible"): 1 for
            // JAL/JALR/C.JALR, 0 for C.J/C.JR (FINDINGS item 2).
            func11 = jal32 || jalr32 || cjalr;

            // x1-ONLY (CONFIRMED, see FINDINGS item 2 -- do not add a x5
            // alternate the way C910's convention would).
            fs_decode.pcall      = func11 && (dst_preg == 5'd1);
            src_dst_eq           = (src0_reg == dst_preg) && func11;
            fs_decode.preturn    = jalr_fam && (src0_reg == 5'd1) && !src_dst_eq;
            fs_decode.pc_reg_mis = jalr_fam && (src0_reg != 5'd1);

            fs_decode.sentinel = (x == SENTINEL);

            // Sign-extended BYTE offsets (RV64GC standard B/J/CB/CJ formats).
            if (jal32)
                fs_decode.imm = {{(PC_WIDTH-21){x[31]}},
                                 x[31], x[19:12], x[20], x[30:21], 1'b0};
            else if (bxx)
                fs_decode.imm = {{(PC_WIDTH-13){x[31]}},
                                 x[31], x[7], x[30:25], x[11:8], 1'b0};
            else if (cjump)
                fs_decode.imm = {{(PC_WIDTH-12){x[12]}},
                                 x[12], x[8], x[10:9], x[6], x[7], x[2],
                                 x[11], x[5:3], 1'b0};
            else if (cbz)
                fs_decode.imm = {{(PC_WIDTH-9){x[12]}},
                                 x[12], x[6:5], x[2], x[11:10], x[4:3], 1'b0};
            else
                fs_decode.imm = {PC_WIDTH{1'b0}};
        end
    endfunction

    //=========================================================================
    // SECTION: SHADOW CALL STACK  (plan "Global contracts": 16 entries,
    // deeper than the real 4-entry RAS on purpose; push fall-through on a
    // committed pcall, pop = actual target on a committed preturn, pop on
    // empty OR past depth 16 returns the fall-through -- both this stack and
    // the C++ ISS (Task 5.1) implement the exact same rule, so they agree
    // deterministically even past the array's own capacity).
    //
    // UNLIKE rv12's C910 clone, this design never needs a same-cycle
    // "push-and-pop" case: C910's x1/x5 dual-link convention lets a single
    // jalr appear to satisfy both push and pop at once (c.jalr x5), but
    // C906's x1-only convention's own dual case ("jalr x1,x1,0") is
    // EXCLUDED from preturn by aq_iu_bju.v's own src_dst_reg_equal term
    // (FINDINGS item 2) -- it is a push-only call in real hardware, full
    // stop. do_push and do_pop below are therefore mutually exclusive by
    // construction, not by a runtime check.
    //
    // RAS-FAITHFUL GRADING HOOK (plan "RAS depth-limitation contract";
    // design doc S2.1/S4.1; plan Task 7.1): the real aq_ifu_ras.v is 4 flop
    // entries with POINTER-ONLY misprediction resync (no entry-content
    // resync) -- a materially different, shallower object than this
    // general-purpose 16-entry oracle. FILLED IN below (SECTION RAS-FAITHFUL
    // GRADING MODEL) now that BPU.v's real RAS exists to mirror -- this
    // 16-entry stack remains the general/rung-agnostic CORRECTNESS oracle
    // (unchanged); the grading model is a separate, diagnostic-only
    // cross-check, not a replacement.
    //=========================================================================
    localparam integer SST_N = 16;
    reg [PC_WIDTH-1:0] sst_mem [0:SST_N-1];
    reg [4:0]          sst_sp;             // tracks true depth past SST_N too

    wire sst_has_entry = (sst_sp != 5'd0) && (sst_sp <= SST_N[4:0]);

    //=========================================================================
    // SECTION: RAS-FAITHFUL GRADING MODEL (plan Task 7.1's hook, filled in
    // now that BPU.v's real RAS exists to mirror -- see this file's header
    // "RAS-FAITHFUL GRADING HOOK" note above, and BPU.v's own RAS section
    // for the aq_ifu_ras.v pointer-resync findings this model reproduces).
    //
    // DIAGNOSTIC ONLY, does not gate pass/fail: the online checker's
    // committed-stream comparison is already invariant to RAS prediction
    // accuracy by design (design doc S4.1's "predictors change WHEN an
    // instruction is fetched, never WHICH instructions commit") -- a real
    // RAS misprediction near depth 4 costs BPU.v some wasted speculative
    // fetch cycles that FetchSink's own resolve (Task 7.2, unconditional
    // for every jalr-family instruction) always corrects before commit.
    // This model exists so a trace can CONFIRM the real 4-entry/pointer-
    // only-resync limitation is actually being exercised the way
    // callret.S's Phase 2 (6 unreturned calls stacked, 2 past the 4-entry
    // limit) intends, not to re-decide correctness.
    //
    // Mirrors aq_ifu_ras.v's algorithm exactly: ONE one-hot 4-bit pointer
    // (push rotates -1 mod 4, i.e. right-rotate; pop rotates +1 mod 4, i.e.
    // left-rotate), one physical 4-entry content array, NO content resync
    // on misprediction, NO empty-stack special case (real hardware has no
    // valid bit either -- an under-flowed pop just reads whatever is
    // physically in the pointed-to entry, reset-zero if never written).
    // Driven by FetchSink's own COMMITTED push/pop events (do_push/do_pop
    // below), NOT BPU's speculative ID-stage stream -- the two views
    // coincide exactly whenever the wrong-path window between a call's
    // fetch and its resolve never itself contains another call/return,
    // which is true of this directed suite (a divergence would show up as
    // a committed-stream mismatch in Task 6/7's own bring-up, since BPU.v's
    // classification runs on the SAME x1-only rule independently). A model
    // built on the confirmed-only view structurally cannot see genuine
    // speculative/wrong-path RAS corruption -- BPU.v's own trace is the
    // only ground truth for that; this is a best-effort cross-check, not a
    // substitute.
    //=========================================================================
    reg [PC_WIDTH-1:0] rasf_entry0, rasf_entry1, rasf_entry2, rasf_entry3;
    reg [3:0]          rasf_pop;

    wire [3:0] rasf_push_next = {rasf_pop[0], rasf_pop[3:1]};   // right-rotate (push)
    wire [3:0] rasf_pop_next  = {rasf_pop[2:0], rasf_pop[3]};   // left-rotate  (pop)

    reg [PC_WIDTH-1:0] rasf_read;
    always @* begin
        case (rasf_pop)
            4'b0001: rasf_read = rasf_entry0;
            4'b0010: rasf_read = rasf_entry1;
            4'b0100: rasf_read = rasf_entry2;
            4'b1000: rasf_read = rasf_entry3;
            default: rasf_read = {PC_WIDTH{1'b0}};
        endcase
    end

    //=========================================================================
    // SECTION: RESOLVE  (combinational decode of THIS cycle's delivered
    // instruction against FetchSink's own tracked arch_pc -- design doc
    // S4.1: "Resolve is REGISTERED, never combinational from delivered
    // data", so this block only computes the decision; every port below is
    // driven from a flop latched at the end of this section, never directly
    // from these wires).
    //
    // C906's real ifu_idu_id_inst/_vld/_bht_pred carry NO PC field at all
    // (confirmed: IFU.v's frozen ports, ibuf.v:1354-1358) -- unlike C910,
    // where rv12's FetchSink cross-checks a PC riding in the payload against
    // its own tracked arch_pc. There is nothing to cross-check here: this
    // module's arch_pc IS the only record of "what PC does this opcode
    // belong to", exactly like the independent C++ fetch-ISS (Task 5.1) will
    // also have to track its own PC forward with no ground truth to compare
    // against except the ISS's own decode. A wrong ICache fetch (wrong
    // opcode bytes at the right PC) is caught downstream, by the online
    // checker comparing FetchSink's exported (pc, opcode) stream against the
    // ISS's independently-computed one -- not by anything inside this
    // module itself.
    //=========================================================================
    reg [PC_WIDTH-1:0] arch_pc;
    reg [31:0]         cmt_total;

    localparam [1:0] ST_RUN    = 2'd0,
                     ST_SQUASH = 2'd1,
                     ST_TOHOST = 2'd2,
                     ST_DONE   = 2'd3;
    reg [1:0] state;
    reg [1:0] squash_cnt;
    reg       r_stall;

    // Accept exactly what IFU presents THIS cycle, but only if FetchSink
    // itself was not stalling last cycle -- ifu_idu_id_inst_vld is a level
    // (repeat-until-accepted) signal, not a one-shot pulse (IFU.v's
    // pop_entry_vld does not gate on idu_ifu_id_stall at all; only the
    // QUEUE ADVANCE does), so re-sampling it while r_stall is asserted would
    // double-commit the same instruction.
    wire accept    = ifu_idu_id_inst_vld && !r_stall;
    wire commit_en = accept && (state == ST_RUN);

    fsdec_t d;
    always @* d = fs_decode(ifu_idu_id_inst);

    wire [PC_WIDTH-1:0] pc   = arch_pc;
    wire [PC_WIDTH-1:0] len  = d.is32 ? {{(PC_WIDTH-3){1'b0}}, 3'd4}
                                       : {{(PC_WIDTH-3){1'b0}}, 3'd2};
    wire [PC_WIDTH-1:0] fall = pc + len;
    wire                taken = ^pc[7:4];                        // direction rule

    wire [PC_WIDTH-1:0] cond_tgt = taken ? (pc + d.imm) : fall;
    wire [PC_WIDTH-1:0] jal_tgt  = pc + d.imm;
    // sst_has_entry already bounds sst_sp to [1..SST_N], so (sst_sp-1) fits
    // a 4-bit array index (SST_N=16) -- the explicit slice below is just
    // making that bound visible to the tool, not a new constraint.
    wire [4:0]          sst_rd_idx5 = sst_sp - 5'd1;
    wire [PC_WIDTH-1:0] ret_pop  = sst_has_entry ? sst_mem[sst_rd_idx5[3:0]] : fall;
    wire [PC_WIDTH-1:0] jalr_tgt = d.preturn ? ret_pop : jr_target(pc);

    // Actual next PC. Target computation is keyed purely on instruction
    // CLASS (con_br/ab_br/jalr_fam), never on pcall/preturn directly -- a
    // jal that also happens to be a call still gets its target from the
    // immediate, exactly as real hardware's ALU-computed target does not
    // care about the RAS side effect riding alongside it.
    wire [PC_WIDTH-1:0] actual_next = d.con_br    ? cond_tgt :
                                       d.ab_br     ? jal_tgt  :
                                       d.jalr_fam  ? jalr_tgt : fall;

    // PREDICTOR-INDEPENDENT redirect rule (design doc S4.1): assert a
    // correction whenever the actual outcome differs from what IFU's own
    // proactive machinery would otherwise deliver. In M1 (BPU.v still an
    // inert skeleton through Task 4 -- Tasks 7-9 build the real predictors),
    // NOTHING upstream of this module can classify or redirect any control
    // transfer on its own, so this fires for every taken conditional branch
    // and every jal/c.j/jalr-family instruction, unconditionally.
    //
    // TASK 9 FIX (found bringing up dense_br.S's Region G -- BHT training --
    // at rung 4, not reachable before a real BHT existed): the original
    // formula only asserted a redirect for a TAKEN conditional branch
    // (`d.con_br ? taken : ...`), on the reasoning "if not taken, IFU's own
    // default/sequential fetch already delivers the correct fall-through,
    // no correction needed." That reasoning silently assumed NOTHING
    // upstream could have ALREADY diverted the fetch stream away from
    // fall-through for a branch that turns out not-taken -- true through
    // Task 8 (no real predictor could ever predict a CONDITIONAL branch
    // taken: `pred_inst0_taken` reduced to exactly `pred_jmp_vld0`, jal/c.j
    // only, until Task 9's BHT term existed), but FALSE now that a real,
    // pure-GHR-indexed BHT (zero PC disambiguation, BPU.v SECTION BHT) can
    // and does alias a NOT-taken branch onto a table entry trained "taken"
    // by unrelated history, driving `pred_chgflw` to speculatively redirect
    // PCGEN to the branch's OWN taken-target -- a genuine misprediction in
    // the OPPOSITE direction from anything Tasks 1-8 could produce. With the
    // old formula, `redirect_needed=0` for this not-taken-but-speculatively-
    // redirected branch, so FetchSink never corrected it: the wrong-path
    // bytes fetched from the mispredicted target were accepted as if they
    // were the true fall-through, corrupting the committed stream (confirmed
    // via a temporary C++ probe, bring-up only, not left in the final RTL --
    // FetchSink kept reporting the correct fall-through PC while delivering
    // bytes from several instructions further down the wrong-path target).
    //
    // THE FIX, made surgical rather than unconditional: OR in
    // `ifu_idu_id_bht_pred[1]` (the captured BHT prediction already riding
    // with this exact instruction, IFU.v's SECTION IBUF `ibuf_tag[]`
    // plumbing) -- redirect whenever the ACTUAL outcome is taken (the
    // original case) OR the PREDICTOR claimed taken (catches a taken-
    // mispredicted-not-taken branch too), but NOT for the overwhelmingly
    // common "predicted not-taken, actually not-taken" case. At rungs 1-3
    // (BHT chicken-bit off) `ifu_idu_id_bht_pred` is provably 0 for every
    // con_br (BPU.v's BHT SRAM is never enabled, its output never departs
    // from reset), so this reduces to EXACTLY the original formula there --
    // zero behavioral change below rung 4. An unconditional
    // `d.con_br ? 1'b1 : ...` (redirecting on literally every conditional
    // branch, matching `d.ab_br`/`d.jalr_fam`) was considered and rejected:
    // `iu_ifu_tar_pc_vld` firing on EVERY not-taken branch would ALSO
    // unconditionally flush IPACK/IBUF every single cycle a branch commits
    // (`ipack_buf_flush`/`ibuf_flush_en`, IFU.v SECTION IPACK/IBUF, both
    // gate directly on this signal) -- correctness-preserving but a
    // needless, drastic throughput regression with no real-hardware analog
    // (real silicon only flushes on an actual misprediction). The
    // BHT-prediction OR-term targets exactly the dangerous case.
    wire redirect_needed = d.con_br   ? (taken || ifu_idu_id_bht_pred[1]) :
                            d.ab_br    ? 1'b1 :
                            d.jalr_fam ? 1'b1 : 1'b0;

    wire bht_mispred = d.con_br && (taken != ifu_idu_id_bht_pred[1]);

    wire do_push = d.pcall;
    wire do_pop  = d.preturn;

    wire sentinel_hit = commit_en && d.sentinel;
    wire [31:0] cmt_total_n = cmt_total + (commit_en ? 32'd1 : 32'd0);
    wire budget_hit = commit_en && !d.sentinel && (cmt_total_n >= cfg_max_insts);

    // resolve_kind (verisim.h export): 0 none, 1 cond-branch redirect,
    // 2 jal/c.j, 3 preturn (shadow-stack pop), 4 other jalr-family
    // (indirect jump or an indirect call, both via JR_TARGET).
    wire [2:0] kind = !redirect_needed ? 3'd0 :
                      d.con_br         ? 3'd1 :
                      d.ab_br          ? 3'd2 :
                      d.preturn        ? 3'd3 : 3'd4;

    //=========================================================================
    // SECTION: BJU / export registers (driven by the SEQUENTIAL block below)
    //=========================================================================
    reg r_tar_pc_vld;
    reg [PC_WIDTH-1:0] r_tar_pc;
    reg r_pc_mispred;
    reg r_bht_mispred;
    reg r_br_vld;
    reg r_bht_taken;
    reg [1:0] r_bht_pred;
    reg r_link_vld;
    reg r_ret_vld;

    // Committed-stream export + resolve event/kind, read by the C++ online
    // checker via verisim.h (Task 4.2/5.2). ONE slot, not three -- C906
    // delivers a single instruction/cycle (plan "Global contracts").
    reg        cmt_valid     /* verilator public */;
    reg [63:0] cmt_pc        /* verilator public */;
    reg [31:0] cmt_opcode    /* verilator public */;
    reg [31:0] cmt_count     /* verilator public */;
    reg        resolve_event /* verilator public */;
    reg [2:0]  resolve_kind  /* verilator public */;
    // perr_code: 0 none, 1 --max-insts budget exhausted without a sentinel.
    // C906's thinner IFU->IDU interface (no PC/expt fields riding along)
    // structurally cannot support most of rv12's PERR_* checks (payload-PC
    // mismatch, slot-valid gaps, checkpoint FIFO bookkeeping) -- there is
    // simply nothing on this interface to check those against. This is a
    // documented consequence of item 1's finding (thinner interface), not a
    // gap left by oversight.
    reg [3:0]  perr_code     /* verilator public */;

    // RAS-faithful grading model export (diagnostic only, see SECTION
    // RAS-FAITHFUL GRADING MODEL above -- does not gate pass/fail).
    // rasf_mispredict pulses for one cycle whenever a committed preturn's
    // ACTUAL target (this module's own ground truth) differs from what the
    // mirrored 4-entry/pointer-only-resync model would have predicted --
    // i.e. exactly the cycles where the real RAS's depth-4 limitation would
    // have produced a genuine wrong speculative redirect.
    reg        rasf_mispredict /* verilator public */;
    reg [63:0] rasf_pred_pc    /* verilator public */;

    reg [63:0] report_val;          // drives the tohost write
    reg [15:0] lfsr;
    wire       lfsr_fb = lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10];

    reg        aw_sent, w_sent;
    reg [12:0] wdog;

    wire in_write   = (state == ST_TOHOST);
    wire aw_hs      = axi_d_awvalid && axi_d_awready;
    wire w_hs       = axi_d_wvalid  && axi_d_wready;
    wire write_done = axi_d_bvalid && (aw_sent || aw_hs) && (w_sent || w_hs);

    //=========================================================================
    // SECTION: SEQUENTIAL  (one update block for the whole datapath)
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arch_pc    <= RESET_VECTOR[PC_WIDTH-1:0];
            cmt_total  <= 32'd0;
            state      <= ST_RUN;
            squash_cnt <= 2'd0;
            r_stall    <= 1'b0;
            lfsr       <= 16'hACE1;
            sst_sp     <= 5'd0;
            rasf_pop     <= 4'b0001;
            rasf_entry0  <= {PC_WIDTH{1'b0}};
            rasf_entry1  <= {PC_WIDTH{1'b0}};
            rasf_entry2  <= {PC_WIDTH{1'b0}};
            rasf_entry3  <= {PC_WIDTH{1'b0}};
            rasf_mispredict <= 1'b0;
            rasf_pred_pc    <= 64'd0;

            r_tar_pc_vld  <= 1'b0;
            r_tar_pc      <= {PC_WIDTH{1'b0}};
            r_pc_mispred  <= 1'b0;
            r_bht_mispred <= 1'b0;
            r_br_vld      <= 1'b0;
            r_bht_taken   <= 1'b0;
            r_bht_pred    <= 2'd0;
            r_link_vld    <= 1'b0;
            r_ret_vld     <= 1'b0;

            cmt_valid     <= 1'b0;
            cmt_pc        <= 64'd0;
            cmt_opcode    <= 32'd0;
            cmt_count     <= 32'd0;
            resolve_event <= 1'b0;
            resolve_kind  <= 3'd0;
            perr_code     <= 4'd0;
            report_val    <= 64'd0;

            aw_sent <= 1'b0;
            w_sent  <= 1'b0;
            wdog    <= 13'd0;
        end
        else begin
            lfsr <= {lfsr[14:0], lfsr_fb};

            //-----------------------------------------------------------
            // Shadow call stack (at most one push OR one pop per commit --
            // see SECTION SHADOW CALL STACK header for why this design
            // never needs a same-cycle push-and-pop case).
            //-----------------------------------------------------------
            if (commit_en && do_push) begin
                if (sst_sp < SST_N[4:0]) sst_mem[sst_sp[3:0]] <= fall;
                if (sst_sp != 5'h1F)     sst_sp <= sst_sp + 5'd1;
            end
            else if (commit_en && do_pop) begin
                if (sst_sp != 5'd0) sst_sp <= sst_sp - 5'd1;
            end

            //-----------------------------------------------------------
            // RAS-faithful grading model (diagnostic only, SECTION
            // RAS-FAITHFUL GRADING MODEL above): driven by the SAME
            // commit-time do_push/do_pop pulses as the shadow call stack,
            // but through the real RAS's own 4-entry/pointer-only-resync
            // algorithm instead of an unbounded array. `rasf_read` (the
            // model's prediction) is sampled BEFORE the pointer rotates,
            // matching aq_ifu_ras.v's own read-then-rotate ordering.
            //-----------------------------------------------------------
            if (commit_en && do_push) begin
                if (rasf_push_next[0]) rasf_entry0 <= fall;
                if (rasf_push_next[1]) rasf_entry1 <= fall;
                if (rasf_push_next[2]) rasf_entry2 <= fall;
                if (rasf_push_next[3]) rasf_entry3 <= fall;
                rasf_pop <= rasf_push_next;
            end
            else if (commit_en && do_pop) begin
                rasf_pop <= rasf_pop_next;
            end
            rasf_mispredict <= commit_en && do_pop && (rasf_read != actual_next);
            rasf_pred_pc    <= {{(64-PC_WIDTH){1'b0}}, rasf_read};

            //-----------------------------------------------------------
            // Architectural PC and the commit counter
            //-----------------------------------------------------------
            if (commit_en) begin
                arch_pc   <= actual_next;
                cmt_total <= cmt_total_n;
                cmt_count <= cmt_total_n;
            end

            //-----------------------------------------------------------
            // Control state: RUN -> SQUASH (post-redirect discard window,
            // TASK 4.1 FINDINGS item 3) -> RUN, or RUN -> TOHOST -> DONE.
            //-----------------------------------------------------------
            case (state)
                ST_RUN: begin
                    if (sentinel_hit) begin
                        state      <= ST_TOHOST;
                        report_val <= 64'd1;
                    end
                    else if (budget_hit) begin
                        state      <= ST_TOHOST;
                        report_val <= 64'd3;
                        perr_code  <= 4'd1;
                    end
                    else if (commit_en && redirect_needed) begin
                        state      <= ST_SQUASH;
                        squash_cnt <= 2'd2;
                    end
                end
                ST_SQUASH: begin
                    squash_cnt <= (squash_cnt == 2'd0) ? 2'd0 : (squash_cnt - 2'd1);
                    if (squash_cnt <= 2'd1) state <= ST_RUN;
                end
                ST_TOHOST: begin
                    if (write_done) state <= ST_DONE;
                end
                default: ;   // ST_DONE holds forever
            endcase

            //-----------------------------------------------------------
            // idu_ifu_id_stall (registered; --sink-stall pseudo-random
            // mode, plan "Global contracts": harness config mechanism).
            // Deliberately NOT asserted during ST_SQUASH: IBUF needs to be
            // free to drain/refill during the discard window, and the
            // squash is enforced by commit_en's own state check above, not
            // by backpressure.
            //-----------------------------------------------------------
            r_stall <= cfg_sink_stall && (lfsr[2:0] == 3'b000);

            //-----------------------------------------------------------
            // BJU outputs -- the registered resolve (design doc S4.1: never
            // combinational from delivered data). Suppressed on the
            // sentinel commit itself: the sim is ending, and firing one
            // last spurious redirect right before tohost buys nothing.
            //-----------------------------------------------------------
            r_tar_pc_vld  <= commit_en && !d.sentinel && redirect_needed;
            r_tar_pc      <= actual_next;
            r_pc_mispred  <= commit_en && d.pc_reg_mis;
            r_bht_mispred <= commit_en && bht_mispred;
            r_br_vld      <= commit_en && d.con_br;
            r_bht_taken   <= taken;
            r_bht_pred    <= ifu_idu_id_bht_pred;
            r_link_vld    <= commit_en && d.pcall;
            r_ret_vld     <= commit_en && d.preturn;

            //-----------------------------------------------------------
            // Committed-stream export
            //-----------------------------------------------------------
            cmt_valid     <= commit_en;
            cmt_pc        <= {{(64-PC_WIDTH){1'b0}}, pc};
            cmt_opcode    <= ifu_idu_id_inst;
            resolve_event <= commit_en && !d.sentinel && redirect_needed;
            resolve_kind  <= kind;

            //-----------------------------------------------------------
            // REPORT: tohost write handshake (TestMaster.v:184-247, lifted
            // verbatim -- concurrent AW+W, BVALID watched every cycle of
            // the write state).
            //-----------------------------------------------------------
            if (in_write) begin
                if (aw_hs) aw_sent <= 1'b1;
                if (w_hs)  w_sent  <= 1'b1;
                wdog <= wdog + 13'd1;
                // If the fabric never answers, re-arm so a lost handshake
                // shows up as a repeated write rather than a silent hang.
                if (wdog == 13'h1FFF) begin
                    aw_sent <= 1'b0;
                    w_sent  <= 1'b0;
                end
            end
            else begin
                aw_sent <= 1'b0;
                w_sent  <= 1'b0;
                wdog    <= 13'd0;
            end
        end
    end

    //=========================================================================
    // SECTION: BJU / RTU output assigns
    //=========================================================================
    // idu_ifu_id_stall (registered; --sink-stall pseudo-random mode, see
    // SECTION SEQUENTIAL above where r_stall is computed) -- BUG FIX during
    // review: this port was declared and r_stall was computed but never
    // actually driven out, so IFU.v's `ctrl_ibuf_pop_en = !idu_ifu_id_stall`
    // (IFU.v:340) saw a permanently-floating/optimized-to-0 net regardless of
    // cfg_sink_stall, meaning IFU would advance its IBUF pop pointer every
    // cycle even while FetchSink itself was internally refusing to accept
    // (accept = ifu_idu_id_inst_vld && !r_stall) -- a silent dropped-
    // instruction bug whenever --sink-stall is exercised (Task 6 requires
    // rung-1 bring-up "and with --sink-stall"). Caught by this bench's own
    // T14 (test_sink_stall_mode), which reads this exact port.
    assign idu_ifu_id_stall   = r_stall;

    assign iu_ifu_tar_pc_vld  = r_tar_pc_vld;
    assign iu_ifu_tar_pc      = {{(64-PC_WIDTH){1'b0}}, r_tar_pc};
    assign iu_ifu_pc_mispred  = r_pc_mispred;
    assign iu_ifu_bht_mispred = r_bht_mispred;
    assign iu_ifu_br_vld      = r_br_vld;
    assign iu_ifu_bht_taken   = r_bht_taken;
    assign iu_ifu_bht_pred    = r_bht_pred;
    assign iu_ifu_link_vld    = r_link_vld;
    assign iu_ifu_ret_vld     = r_ret_vld;

    // Front-end flush belongs to exceptions/interrupts, which M1 has no
    // machinery for yet (no CSR file, no trap path exists before M2) -- the
    // ports stay quiet until the real RTU lands, same posture as rv12's
    // analogous tie-off.
    assign rtu_ifu_chgflw_vld = 1'b0;
    assign rtu_ifu_chgflw_pc  = {PC_WIDTH{1'b0}};
    assign rtu_ifu_flush_fe   = 1'b0;

    // ifu_iu_chgflw_vld/_pc (IFU forwarding an RTU-sourced redirect back to
    // IU, pcgen.v:325-326) can never carry anything but the reset value
    // while rtu_ifu_chgflw_vld is tied 0 above -- genuinely unused here, not
    // an oversight.
    wire _iu_unused_ok = ifu_iu_chgflw_vld | (|ifu_iu_chgflw_pc);

    //=========================================================================
    // SECTION: REPORT -- tohost write channel (TestMaster.v's proven D-side
    // AXI write FSM, lifted verbatim: concurrent AW+W, BVALID every cycle).
    //=========================================================================
    assign axi_d_awvalid = in_write && !aw_sent;
    assign axi_d_awaddr  = TOHOST_ADDR[ADDR_WIDTH-1:0];
    assign axi_d_awlen   = 8'd0;
    assign axi_d_awsize  = 3'd6;            // one full 64-byte beat
    assign axi_d_awburst = 2'b01;
    assign axi_d_awcache = 4'd0;
    assign axi_d_awprot  = 3'd0;

    assign axi_d_wvalid  = in_write && !w_sent;
    assign axi_d_wdata   = {{(DATA_WIDTH-64){1'b0}}, report_val};
    assign axi_d_wstrb   = {DATA_WIDTH/8{1'b1}};
    assign axi_d_wlast   = axi_d_wvalid;
    assign axi_d_bready  = 1'b1;

    // The sink never reads on the D side.
    assign axi_d_arvalid = 1'b0;
    assign axi_d_araddr  = {ADDR_WIDTH{1'b0}};
    assign axi_d_arlen   = 8'd0;
    assign axi_d_arsize  = 3'd6;
    assign axi_d_arburst = 2'b01;
    assign axi_d_arcache = 4'd0;
    assign axi_d_arprot  = 3'd0;
    assign axi_d_rready  = 1'b1;

    wire _axi_r_unused_ok = axi_d_rvalid | axi_d_rlast | (|axi_d_rresp) |
                            (|axi_d_rdata) | (|axi_d_bresp);

endmodule
