//=============================================================================
// FPU.v - scalar FPU cluster, FALU sub-block only  (M5 Task 3: new file)
//=============================================================================
// C906 files covered (the FALU control-word decode, confirmed by direct read
// this task -- see rvproc_pkg.sv's FUNC_* comment block for the full citation
// of how these span the donor's shared 20-bit `vpu_group_1_xx_ex1_func`):
//   gen_rtl/vfalu/rtl/aq_fadd_scalar_dp.v:542-566  (add/sub/cmp/max/min group)
//   gen_rtl/vfalu/rtl/aq_fspu_top.v:195-227        (sgnj/mv/class group)
//   gen_rtl/vfalu/rtl/aq_fcnvt_scalar_dp.v:449-513 (widen/narrow group)
//   gen_rtl/vfalu/rtl/aq_falu_top.v, aq_falu_ctrl.v (cluster-level EU-select
//                                     gating; pure pipeline control, no
//                                     func-bit decode of its own)
// Algorithmic porting source: ../rv12/rtl/FPUAlu.v (rv12's own C910-derived,
// donor-cited FALU implementation -- the arithmetic/rounding/classify logic
// below is a direct transcription of that file's SECTIONS 2-11, flattened
// per D1). References: docs/superpowers/specs/2026-09-05-m5-fpu-design.md
// D1 (single-issue, no dual-pipe/forwarding, EX-stage shape mirrors IU.v).
//
// DECISIONS AND RECORDED DEVIATIONS
//=============================================================================
// D1  *** SINGLE-CYCLE COMBINATIONAL, NOT rv12's THREE-FLOP EX1/EX2/EX3
//     CHAIN. ***  rv12's FPUAlu.v (itself a flattening of C910's donor
//     pipe) still keeps three pipeline stages per sub-unit (its own SECTION
//     1's `ex1_pipedown`/`ex2_pipedown`/`ex3_pipedown`). The M5 design doc's
//     D1 pins rv906's FPU cluster as single-issue in-order with no dual-pipe
//     or cross-pipe forwarding, mirroring `IU.v`'s ALU section (`rtl/IU.v`,
//     the "one-hot OR-mux across sub-blocks", fully combinational, no
//     completion latency). This file therefore takes rv12's per-path
//     algorithms (SECTIONS 2-11) but drops the flop chain entirely: every
//     rv12 `e2_*`/`ex2_*` name below is a same-cycle EX1 `wire` ALIAS, not a
//     registered value -- kept under its rv12 name only so a reader
//     diffing this file against ../rv12/rtl/FPUAlu.v can align section by
//     section. `clk`/`rst_n` are frozen into the port list now (this module
//     grows FMAU/FDSU in later M5 tasks, which DO need a clock for the
//     iterative divider's busy/stall handshake, design doc D1) but are
//     unused by this task's purely-combinational FALU body.
//
// D-TASK3-1  *** FSPU RESTRICTED TO SIGN-INJECT + fclass. ***  fmv.x.w /
//     fmv.w.x / fmv.x.d / fmv.d.x (rv12's `spu_op_mv_fx`/`spu_op_mv_xf` and
//     everything they read/produce) are DROPPED from this task and deferred
//     to Task 7 (int<->FP data-move family), per the M5 plan's Task 3 scope
//     statement. No `FUNC_SPU_MV*` constants exist in rvproc_pkg.sv yet --
//     Task 7 is that constant family's first writer.
//
// D-TASK3-2  *** FCNVT RESTRICTED TO FLOAT-TO-FLOAT. ***  fcvt.s.d/fcvt.d.s
//     only; every int<->float arm (rv12 SECTION 11a's float->int saturation
//     table and SECTION 11b's int->float normalise, plus the `cvt_src_si`/
//     `cvt_dest_si`/`cvt_i_*` machinery SECTION 10 uses to select them) is
//     DROPPED and deferred to Task 7. Because this task's only live FCNVT
//     shape is float-to-float, `cvt_src_flt`/`cvt_dest_flt` are hardwired
//     1'b1 here rather than read from `FUNC_CVT_SRC_FLT`/`_DEST_FLT` (which,
//     like the MV family above, do not exist in rvproc_pkg.sv yet -- Task 7
//     is their first writer too).
//
// D-TASK3-3  *** `FUNC_SCALAR` DROPPED, PER rvproc_pkg.sv's OWN FUNC_* BLOCK
//     COMMENT. ***  rv12's `box_check_en = f_scalar && f_single` collapses
//     to `f_single` alone here: rv906 has no vector/SIMD FP context at all
//     (design doc D1), so every instruction this file ever sees is
//     "scalar" by construction and the bit does not exist.
//=============================================================================

import rvproc_pkg::*;

module FPU (
    input  wire                        clk,
    input  wire                        rst_n,

    //=========================================================================
    // IDU -> FPU : EX1 dispatch. Task 3: `idu_fpu_ex1_fsrc0/1_data` are the
    // real IDU FRF outputs (M5 Task 2, IDU.v:1580-1582); the three `_sel`
    // signals and `_func`/`_rm` are new ports this task adds -- IDU decode
    // does not drive them yet (RVProc.v ties them to inert constants until
    // Task 4, per the plan's own Task 3 scope statement).
    //=========================================================================
    input  wire                        idu_fpu_ex1_fadd_sel,
    input  wire                        idu_fpu_ex1_fspu_sel,
    input  wire                        idu_fpu_ex1_fcnvt_sel,
    // M5 Task 5: FMAU dispatch select + the third FRF source (the addend
    // for fmadd/fmsub/fnmadd/fnmsub; don't-care, per rv12 D6's class-terms
    // gating, for a plain fmul). IDU.v already exports this data port
    // (M5 Task 2); the _sel wire is new this task.
    input  wire                        idu_fpu_ex1_fmau_sel,
    // M5 Task 6: FDSU (fdiv.{s,d}/fsqrt.{s,d}) dispatch select. Mirrors
    // IU.v's idu_iu_ex1_div_sel exactly (IDU.v gates it with
    // `!fpu_idu_fdsu_full` the same way DIV is gated with
    // `!iu_idu_div_full`) -- see `fpu_idu_fdsu_full` below.
    input  wire                        idu_fpu_ex1_fdsu_sel,
    input  wire [FUNC_WIDTH-1:0]       idu_fpu_ex1_func,
    input  wire [2:0]                  idu_fpu_ex1_rm,
    // M5 Task 11 BUG 2 (Category A, spec): frm CSR for dynamic rounding.
    // rm=111 (DYN) is resolved to frm below (rv12/rtl/FPU.v:297/:662's own
    // `cp0_fpu_frm` pattern) -- the port name follows CSR.v's
    // cp0_<consumer>_<signal> convention.
    input  wire [2:0]                  cp0_fpu_frm,
    input  wire [XLEN-1:0]             idu_fpu_ex1_fsrc0_data,
    input  wire [XLEN-1:0]             idu_fpu_ex1_fsrc1_data,
    input  wire [XLEN-1:0]             idu_fpu_ex1_fsrc2_data,
    // M5 Task 4b: destination register tag, pure pass-through (IU.v's
    // `iu_rtu_ex1_alu_preg = idu_iu_ex1_dst0_reg` / CSR.v's
    // `cp0_rtu_ex1_wb_preg = idu_cp0_ex1_dst0_reg` precedent) -- RTU uses
    // this to route the FRF writeback (rtu_idu_wbf0_reg).
    input  wire [GPR_IDX_WIDTH-1:0]    idu_fpu_ex1_dst0_reg,

    //=========================================================================
    // FPU -> RTU : the FALU's two answer shapes (rv12 FPUAlu.v's `ex1_mfvr`/
    // `ex3_freg` split, S3.1) -- `_fdata`/`_fflags`/`_fvld` for an FP-
    // register-destination result (add/sub/sgnj*/fcvt.s.d/fcvt.d.s),
    // `_xdata`/`_xvld` for an integer-register-destination result (compare,
    // fclass). Both leave at EX1 (D1: no completion latency).
    //=========================================================================
    output wire [XLEN-1:0]             fpu_rtu_ex1_falu_fdata,
    output wire [XLEN-1:0]             fpu_rtu_ex1_falu_xdata,
    output wire [4:0]                  fpu_rtu_ex1_falu_fflags,
    output wire                        fpu_rtu_ex1_falu_fvld,
    output wire                        fpu_rtu_ex1_falu_xvld,
    output wire [GPR_IDX_WIDTH-1:0]    fpu_rtu_ex1_falu_preg,

    //=========================================================================
    // FPU -> IDU : FDSU busy/full. Mirrors IU.v's `iu_idu_div_full` (IU.v:
    // 1331-1332) but WITHOUT the `!wb_grant` term -- RTU.v:961-963/974
    // confirms FPU's wbf0 writeback has no arbiter ("FALU is the only wbf0
    // producer today"; `wbf0_vld_r <= fpu_rtu_ex1_falu_fvld` unconditional),
    // so there is no writeback-grant-wait state to fold in here.
    //=========================================================================
    output wire                        fpu_idu_fdsu_full
);

    //=========================================================================
    // SECTION 2: SHARED PRIMITIVES (../rv12/rtl/FPUAlu.v SECTION 2, verbatim)
    //=========================================================================
    localparam integer PW      = 57;   // the pre-pack field width
    localparam [10:0]  BIAS_D  = 11'd1023;
    localparam [10:0]  BIAS_S  = 11'd127;

    // Leading-zero count over the 57-bit pre-pack field. Returns 57 for zero.
    function [7:0] lzc57;
        input [PW-1:0] v;
        integer i;
        reg    found;
        begin
            lzc57 = 8'd57;
            found = 1'b0;
            for (i = PW-1; i >= 0; i = i - 1)
                if (v[i] && !found) begin
                    found = 1'b1;
                    lzc57 = 8'd56 - i[7:0];
                end
        end
    endfunction

    // Leading-zero count over a 64-bit integer magnitude (M5 Task 7 i2f).
    // Returns 64 for zero (../rv12/rtl/FPUAlu.v:355-378, verbatim shape).
    function [7:0] lzc64;
        input [63:0] v;
        integer i;
        reg    found;
        begin
            lzc64 = 8'd64;
            found = 1'b0;
            for (i = 63; i >= 0; i = i - 1)
                if (v[i] && !found) begin
                    found = 1'b1;
                    lzc64 = 8'd63 - i[7:0];
                end
        end
    endfunction

    // THE ROUND DECISION. Five modes, RISC-V's own encoding (rm = 000 RNE,
    // 001 RTZ, 010 RDN, 011 RUP, 100 RMM). 101/110 reserved (IDU refuses
    // them); 111 DYN is resolved before it reaches this module.
    function round_up;
        input       sign;
        input       lsb;
        input       g;
        input       s;
        input [2:0] rm;
        begin
            case (rm)
                3'b000:  round_up = g && (s || lsb);      // RNE
                3'b001:  round_up = 1'b0;                 // RTZ
                3'b010:  round_up = sign  && (g || s);    // RDN, away from zero
                3'b011:  round_up = !sign && (g || s);    // RUP, away from zero
                3'b100:  round_up = g;                    // RMM, ties away
                default: round_up = 1'b0;
            endcase
        end
    endfunction

    // The overflow result select: on overflow the result is the LARGEST
    // FINITE number when the mode rounds toward zero for this sign, and
    // infinity otherwise.
    function lfn_sel;
        input       sign;
        input [2:0] rm;
        begin
            lfn_sel = (rm == 3'b001) ||               // rtz
                      ((rm == 3'b011) &&  sign) ||    // rup and negative
                      ((rm == 3'b010) && !sign);      // rdn and positive
        end
    endfunction

    // THE PACKER (D4 in rv12's ledger): normalise -> denormalise-if-tiny ->
    // round -> encode, with the five flags. Result is already NaN-boxed for
    // a single-precision destination. Returns {result[63:0], fflags[4:0]}.
    localparam integer PACK_W = 69;

    function [PACK_W-1:0] fp_pack;
        input               sign;
        input signed [12:0] e_in;      // biased FIELD exponent, leading bit at P[55]
        input [PW-1:0]      p_in;
        input               st_in;     // sticky already accumulated below P[0]
        input [2:0]         rm;
        input               is_dbl;
        begin: pack_body
            reg [PW-1:0]      p;
            reg signed [12:0] e;
            reg               st;
            reg [7:0]         lz;
            reg [12:0]        rsh;
            reg [PW-1:0]      lowmask;
            reg [PW-1:0]      ulp;
            reg               lsb, g, s, inc, is_sub, ovf, tiny;
            reg [PW-1:0]      p2;
            reg [10:0]        efield;
            reg [51:0]        frac;
            reg [63:0]        res;
            reg [4:0]         fl;

            p  = p_in;
            e  = e_in;
            st = st_in;

            if (p == {PW{1'b0}}) begin
                // An exact zero. The sign is the caller's; no flags.
                res = is_dbl ? {sign, 63'b0}
                             : {32'hffffffff, sign, 31'b0};
                fl  = 5'b0;
            end else begin
                // 1. normalise so that the leading one sits at P[55]
                lz = lzc57(p);
                if (lz == 8'd0) begin
                    // the adder carried into P[56]
                    st = st | p[0];
                    p  = p >> 1;
                    e  = e + 13'sd1;
                end else if (lz > 8'd1) begin
                    p = p << (lz - 8'd1);
                    e = e - $signed({5'b0, (lz - 8'd1)});
                end

                // 1a. the tininess probe, at the NORMAL round point (IEEE's
                // "tininess after rounding" as-if-unbounded-exponent rule --
                // NOT the same as rounding on the subnormal grid).
                if (is_dbl) begin
                    lowmask = {{(PW-3){1'b0}}, 3'b111};
                    ulp     = {{(PW-4){1'b0}}, 4'b1000};
                    lsb     = p[3];
                    g       = p[2];
                    s       = st | (|p[1:0]);
                end else begin
                    lowmask = {{(PW-32){1'b0}}, {32{1'b1}}};
                    ulp     = {{(PW-33){1'b0}}, 1'b1, {32{1'b0}}};
                    lsb     = p[32];
                    g       = p[31];
                    s       = st | (|p[30:0]);
                end
                inc  = round_up(sign, lsb, g, s, rm);
                p2   = (p & ~lowmask) + (inc ? ulp : {PW{1'b0}});
                tiny = (p2[PW-1] ? (e + 13'sd1) : e) < 13'sd1;

                // 2. gradual underflow -- no FTZ anywhere, ever.
                if (e < 13'sd1) begin
                    rsh = $unsigned(13'sd1 - e);
                    if (rsh >= 13'd57) begin
                        st = st | (|p);
                        p  = {PW{1'b0}};
                    end else begin
                        st = st | (|(p & ~({PW{1'b1}} << rsh)));
                        p  = p >> rsh;
                    end
                    e = 13'sd1;
                end

                // 3. round at the format's own round point
                if (is_dbl) begin
                    lowmask = {{(PW-3){1'b0}}, 3'b111};
                    ulp     = {{(PW-4){1'b0}}, 4'b1000};
                    lsb     = p[3];
                    g       = p[2];
                    s       = st | (|p[1:0]);
                end else begin
                    lowmask = {{(PW-32){1'b0}}, {32{1'b1}}};
                    ulp     = {{(PW-33){1'b0}}, 1'b1, {32{1'b0}}};
                    lsb     = p[32];
                    g       = p[31];
                    s       = st | (|p[30:0]);
                end
                inc = round_up(sign, lsb, g, s, rm);
                p2  = (p & ~lowmask) + (inc ? ulp : {PW{1'b0}});
                if (p2[PW-1]) begin
                    // the increment carried out of P[55]
                    p2 = p2 >> 1;
                    e  = e + 13'sd1;
                end

                // 4. encode
                is_sub = !p2[55];
                ovf    = is_dbl ? (e >= 13'sd2047) : (e >= 13'sd255);
                frac   = is_dbl ? p2[54:3] : {p2[54:32], 29'b0};
                efield = is_sub ? 11'b0 : e[10:0];

                if (ovf) begin
                    if (lfn_sel(sign, rm))
                        res = is_dbl ? {sign, 11'h7fe, {52{1'b1}}}
                                     : {32'hffffffff, sign, 8'hfe, {23{1'b1}}};
                    else
                        res = is_dbl ? {sign, 11'h7ff, 52'b0}
                                     : {32'hffffffff, sign, 8'hff, 23'b0};
                end else begin
                    res = is_dbl ? {sign, efield, frac}
                                 : {32'hffffffff, sign, efield[7:0], frac[51:29]};
                end

                // NV and DZ are never raised here; the caller ORs its own in.
                // UF is `tiny` AND inexact. `is_sub` is the DELIVERED result's
                // shape and deliberately NOT the flag (they disagree at the
                // subnormal/normal midpoint case).
                fl = {1'b0, 1'b0, ovf, tiny && (g || s), (g || s) || ovf};
            end
            fp_pack = {res, fl};
        end
    endfunction

    //=========================================================================
    // SECTION 3: THE NaN-BOX CONSUMER CHECK AND OPERAND CLASSIFY
    // (../rv12/rtl/FPUAlu.v SECTION 3, verbatim except D-TASK3-3's
    // box_check_en collapse)
    //=========================================================================
    wire        f_double = idu_fpu_ex1_func[FUNC_DOUBLE];
    wire        f_single = idu_fpu_ex1_func[FUNC_B_SINGLE];

    // M5 Task 11 BUG 2 (Category A, spec): THE single rm resolution point
    // (rv12/rtl/FPU.v:662's own `p6_ex1_rm` shape). RISC-V: rm=111 (DYN)
    // resolves to the frm CSR; 101/110 stay IDU-refused (dead encodings).
    // Every internal rm consumer below reads rm_eff -- the idu_fpu_ex1_rm
    // PORT itself stays untouched (frozen IDU interface). Note FMA has no rm
    // field: its funct3=111 latched through dis_rm used to hit round_up's
    // default arm (RTZ-like) and silently mis-round every fmadd/fmsub/
    // fnmadd/fnmsub.
    wire [2:0]   rm_eff  = (idu_fpu_ex1_rm == 3'b111) ? cp0_fpu_frm
                                                          : idu_fpu_ex1_rm;

    // D-TASK3-3: rv12's `f_scalar && f_single` collapses to `f_single` alone.
    wire        box_check_en = f_single;

    // -- source 0 -----------------------------------------------------------
    wire        a_cnan  = box_check_en && !(&idu_fpu_ex1_fsrc0_data[63:32]);
    wire        a_s     = f_double ? idu_fpu_ex1_fsrc0_data[63] : idu_fpu_ex1_fsrc0_data[31];
    wire [10:0] a_ef    = f_double ? idu_fpu_ex1_fsrc0_data[62:52] : {3'b0, idu_fpu_ex1_fsrc0_data[30:23]};
    wire [51:0] a_frac  = f_double ? idu_fpu_ex1_fsrc0_data[51:0] : {idu_fpu_ex1_fsrc0_data[22:0], 29'b0};
    wire        a_e_max = f_double ? (&idu_fpu_ex1_fsrc0_data[62:52]) : (&idu_fpu_ex1_fsrc0_data[30:23]);
    wire        a_e_z   = ~|a_ef;
    wire        a_f_z   = ~|a_frac;
    wire        a_f_msb = f_double ? idu_fpu_ex1_fsrc0_data[51] : idu_fpu_ex1_fsrc0_data[22];

    wire        a_is_snan = a_e_max && !a_f_msb && !a_f_z && !a_cnan;
    wire        a_is_qnan = (a_e_max &&  a_f_msb) || a_cnan;
    wire        a_is_inf  = a_e_max && a_f_z && !a_cnan;
    wire        a_is_zero = a_e_z   && a_f_z && !a_cnan;

    // -- source 1 -----------------------------------------------------------
    wire        b_cnan  = box_check_en && !(&idu_fpu_ex1_fsrc1_data[63:32]);
    wire        b_s     = f_double ? idu_fpu_ex1_fsrc1_data[63] : idu_fpu_ex1_fsrc1_data[31];
    wire [10:0] b_ef    = f_double ? idu_fpu_ex1_fsrc1_data[62:52] : {3'b0, idu_fpu_ex1_fsrc1_data[30:23]};
    wire [51:0] b_frac  = f_double ? idu_fpu_ex1_fsrc1_data[51:0] : {idu_fpu_ex1_fsrc1_data[22:0], 29'b0};
    wire        b_e_max = f_double ? (&idu_fpu_ex1_fsrc1_data[62:52]) : (&idu_fpu_ex1_fsrc1_data[30:23]);
    wire        b_e_z   = ~|b_ef;
    wire        b_f_z   = ~|b_frac;
    wire        b_f_msb = f_double ? idu_fpu_ex1_fsrc1_data[51] : idu_fpu_ex1_fsrc1_data[22];

    wire        b_is_snan = b_e_max && !b_f_msb && !b_f_z && !b_cnan;
    wire        b_is_qnan = (b_e_max &&  b_f_msb) || b_cnan;
    wire        b_is_inf  = b_e_max && b_f_z && !b_cnan;
    wire        b_is_zero = b_e_z   && b_f_z && !b_cnan;

    // The internal number form (SECTION 2's doc comment): a cnan source is
    // REPLACED by the canonical qNaN here, so the datapath never sees it.
    wire [52:0] a_sig53 = a_cnan ? 53'b0 : {~a_e_z, a_frac};
    wire [52:0] b_sig53 = b_cnan ? 53'b0 : {~b_e_z, b_frac};
    wire [11:0] a_eeff  = a_e_z ? 12'd1 : {1'b0, a_ef};
    wire [11:0] b_eeff  = b_e_z ? 12'd1 : {1'b0, b_ef};

    //=========================================================================
    // SECTION 4: FADD EX1 -- OPERAND EXCHANGE, THE THREE PATH SELECTS,
    // COMPARES (../rv12/rtl/FPUAlu.v SECTION 4, verbatim)
    //=========================================================================
    wire op_add = idu_fpu_ex1_func[FUNC_ADD];
    wire op_sub = idu_fpu_ex1_func[FUNC_SUB];
    wire op_cmp = idu_fpu_ex1_func[FUNC_CMP];
    wire op_max = idu_fpu_ex1_func[FUNC_MAX];
    wire op_min = idu_fpu_ex1_func[FUNC_MIN];
    wire op_sel_mm = op_max || op_min;

    wire cmp_feq  = op_cmp && idu_fpu_ex1_func[FUNC_CMP_FEQ];
    wire cmp_flt  = op_cmp && idu_fpu_ex1_func[FUNC_CMP_LT];
    wire cmp_fle  = op_cmp && idu_fpu_ex1_func[FUNC_CMP_LE];
    wire cmp_ford = op_cmp && idu_fpu_ex1_func[FUNC_CMP_FORD];
    wire cmp_fne  = op_cmp && idu_fpu_ex1_func[FUNC_CMP_FNE];

    // The donor's own effective-operation table: a compare or min/max is a
    // subtraction for the purpose of picking the path (that is how the
    // donor gets its compare answer out of the adder).
    wire cmp_sub  = op_sub || op_cmp || op_sel_mm;
    wire act_add  = (op_add  && (a_s ~^ b_s)) || (cmp_sub && (a_s ^ b_s));
    wire act_sub  = (op_add  && (a_s ^  b_s)) || (cmp_sub && (a_s ~^ b_s));

    //-- the exchange: the operand with the larger effective exponent is src0 --
    wire        add_es    = (a_eeff < b_eeff);
    wire [11:0] add_ed    = add_es ? (b_eeff - a_eeff) : (a_eeff - b_eeff);

    wire        s0_s      = add_es ? b_s      : a_s;
    wire [11:0] s0_e      = add_es ? b_eeff   : a_eeff;
    wire [52:0] s0_sig    = add_es ? b_sig53  : a_sig53;
    wire [52:0] s1_sig    = add_es ? a_sig53  : b_sig53;
    wire        s1_is_zero= add_es ? a_is_zero: b_is_zero;

    // `a - b` with |b| > |a| comes out with the sign of -b, which is why the
    // inversion is on s0 and not on s1.
    wire        act_s     = (add_es && cmp_sub) ? ~s0_s : s0_s;

    //-- the three path selects --------------------------------------------
    // BYPASS: the smaller operand lies entirely below half an ulp of the
    //   larger; SECTION 2's max(E,1) form gives a single-valued threshold.
    wire add_bypass_sel = f_double ? (add_ed > 12'd54) : (add_ed > 12'd25);
    // CLOSE: an effective subtraction whose operands are within a factor of
    //   two -- massive cancellation is possible, no alignment shift beyond
    //   one bit is needed.
    wire add_close_sel  = act_sub && (add_ed <= 12'd1);
    // FAR: everything else.
    wire add_far_sel    = !add_bypass_sel && !add_close_sel;

    //-- the compares (RISC-V's predicates, IEEE total-order magnitude form) --
    wire [62:0] a_mag = f_double ? idu_fpu_ex1_fsrc0_data[62:0] : {32'b0, idu_fpu_ex1_fsrc0_data[30:0]};
    wire [62:0] b_mag = f_double ? idu_fpu_ex1_fsrc1_data[62:0] : {32'b0, idu_fpu_ex1_fsrc1_data[30:0]};
    wire        both_zero = a_is_zero && b_is_zero;

    wire cmp_eq_mag = (a_mag == b_mag) && (a_s == b_s);
    wire cmp_a_eq_b = cmp_eq_mag || both_zero;
    wire cmp_a_lt_b = (a_s && !b_s) ? !both_zero
                    : (!a_s && b_s) ? 1'b0
                    : a_s           ? (a_mag > b_mag)
                                    : (a_mag < b_mag);

    wire cmp_any_nan = a_is_nan_i || b_is_nan_i;
    wire a_is_nan_i  = a_is_snan || a_is_qnan;
    wire b_is_nan_i  = b_is_snan || b_is_qnan;
    wire cmp_res     = (cmp_feq  && !cmp_any_nan &&  cmp_a_eq_b)
                    || (cmp_fne  && !(!cmp_any_nan && cmp_a_eq_b))
                    || (cmp_flt  && !cmp_any_nan &&  cmp_a_lt_b)
                    || (cmp_fle  && !cmp_any_nan && (cmp_a_lt_b || cmp_a_eq_b))
                    || (cmp_ford && !cmp_any_nan);

    // The compare answer leaves at EX1 on the mfvr (integer-destination) bus.
    wire [63:0] fadd_mfvr_data = {63'b0, cmp_res};

    //-- FLATTENED EX1->EX2 (D1): these are same-cycle wire ALIASES, not
    // registered EX2 values -- named `e2_*` only so SECTIONS 5-8 below stay
    // a verbatim transcription of ../rv12/rtl/FPUAlu.v and remain diffable
    // against it section by section.
    wire        e2_double     = f_double;
    wire        e2_op_add     = op_add;
    wire        e2_op_sub     = op_sub;
    wire        e2_op_cmp     = op_cmp;
    wire        e2_op_max     = op_max;
    wire        e2_op_min     = op_min;
    wire        e2_cmp_flt    = cmp_flt;
    wire        e2_cmp_fle    = cmp_fle;
    wire [2:0]  e2_rm         = rm_eff;
    wire        e2_act_add    = act_add;
    wire        e2_act_sub    = act_sub;
    wire        e2_act_s      = act_s;
    wire        e2_bypass_sel = add_bypass_sel;
    wire        e2_close_sel  = add_close_sel;
    wire        e2_far_sel    = add_far_sel;
    wire [11:0] e2_ed         = add_ed;
    wire [11:0] e2_s0_e       = s0_e;
    wire [52:0] e2_s0_sig     = s0_sig;
    wire [52:0] e2_s1_sig     = s1_sig;
    wire        e2_s1_is_zero = s1_is_zero;
    wire        e2_a_snan     = a_is_snan;
    wire        e2_a_qnan     = a_is_qnan;
    wire        e2_a_inf      = a_is_inf;
    wire        e2_a_zero     = a_is_zero;
    wire        e2_a_s        = a_s;
    wire        e2_b_snan     = b_is_snan;
    wire        e2_b_qnan     = b_is_qnan;
    wire        e2_b_inf      = b_is_inf;
    wire        e2_b_zero     = b_is_zero;
    wire        e2_b_s        = b_s;
    wire [63:0] e2_a_raw      = a_cnan ? {32'hffffffff, 32'h7fc00000} : idu_fpu_ex1_fsrc0_data;
    wire [63:0] e2_b_raw      = b_cnan ? {32'hffffffff, 32'h7fc00000} : idu_fpu_ex1_fsrc1_data;
    wire        e2_cmp_res    = cmp_res;

    //=========================================================================
    // SECTION 5: FADD EX2 -- THE BYPASS PATH (verbatim)
    //=========================================================================
    wire [PW-1:0] byp_p_base = {1'b0, e2_s0_sig, 3'b000};
    wire [PW-1:0] byp_p      = e2_s1_is_zero ? byp_p_base
                             : e2_act_add    ? (byp_p_base + {{(PW-1){1'b0}}, 1'b1})
                                             : (byp_p_base - {{(PW-1){1'b0}}, 1'b1});

    wire [PACK_W-1:0] byp_pack = fp_pack(e2_act_s, $signed({1'b0, e2_s0_e}),
                                         byp_p, 1'b0, e2_rm, e2_double);
    wire [63:0]  byp_result = byp_pack[PACK_W-1:5];
    wire [4:0]   byp_flags  = byp_pack[4:0];
    // "bypass is inexact unless the far operand was a true zero"
    wire         byp_nx     = !e2_s1_is_zero;

    //=========================================================================
    // SECTION 6: FADD EX2 -- THE CLOSE PATH (verbatim)
    //=========================================================================
    wire [53:0] cls_a54 = {e2_s0_sig, 1'b0};
    wire [53:0] cls_b54 = (e2_ed == 12'd1) ? {1'b0, e2_s1_sig} : {e2_s1_sig, 1'b0};
    wire        cls_b_gt_a = cls_b54 > cls_a54;
    wire [53:0] cls_diff = cls_b_gt_a ? (cls_b54 - cls_a54) : (cls_a54 - cls_b54);
    // when the smaller-exponent operand turns out to have the larger
    // significand the result's sign flips.
    wire        cls_sign = cls_b_gt_a ? ~e2_act_s : e2_act_s;
    wire [PW-1:0] cls_p  = {1'b0, cls_diff, 2'b00};

    wire [PACK_W-1:0] cls_pack = fp_pack(cls_sign, $signed({1'b0, e2_s0_e}),
                                         cls_p, 1'b0, e2_rm, e2_double);
    wire [63:0]  cls_result = cls_pack[PACK_W-1:5];
    wire [4:0]   cls_flags  = cls_pack[4:0];
    // an exact cancellation to zero. The sign is not the path's; it is the
    // IEEE zero-sign rule in SECTION 8.
    wire         cls_r_is_0 = (cls_diff == 54'b0);

    //=========================================================================
    // SECTION 7: FADD EX2 -- THE FAR PATH (verbatim)
    //=========================================================================
    wire [55:0] far_a56 = {e2_s0_sig, 3'b000};
    wire [55:0] far_b56_full = {e2_s1_sig, 3'b000};
    wire [5:0]  far_sh  = (e2_ed > 12'd55) ? 6'd55 : e2_ed[5:0];
    wire [55:0] far_b56 = far_b56_full >> far_sh;
    wire        far_st  = |(far_b56_full & ~({56{1'b1}} << far_sh));
    wire [56:0] far_sum = e2_act_add
                        ? ({1'b0, far_a56} + {1'b0, far_b56})
                        : ({1'b0, far_a56} - {1'b0, far_b56}
                           - {56'b0, far_st});

    wire [PW-1:0] far_p = far_sum;

    wire [PACK_W-1:0] far_pack = fp_pack(e2_act_s, $signed({1'b0, e2_s0_e}),
                                         far_p, far_st, e2_rm, e2_double);
    wire [63:0]  far_result = far_pack[PACK_W-1:5];
    wire [4:0]   far_flags  = far_pack[4:0];

    //=========================================================================
    // SECTION 8: FADD EX2 -- SPECIAL RESULTS, MIN/MAX, THE RESULT MUX, FLAGS
    // (verbatim)
    //=========================================================================
    wire [63:0] canon_qnan = e2_double ? {1'b0, 11'h7ff, 1'b1, 51'b0}
                                       : {32'hffffffff, 1'b0, 8'hff, 1'b1, 22'b0};

    // Any NaN in, or inf - inf, gives a NaN out; always the CANONICAL one,
    // never a payload (dqnan pinned 0, no FXCR).
    wire add_r_is_qnan = e2_a_snan || e2_b_snan || e2_a_qnan || e2_b_qnan
                      || (e2_a_inf && e2_b_inf && e2_act_sub);
    wire add_r_is_inf  = (e2_a_inf || e2_b_inf) && !add_r_is_qnan;
    // The SECOND operand's contribution is negated on an `fsub`, so an
    // infinity arriving on src1 comes out with the OPPOSITE sign.
    wire add_inf_sign  = e2_a_inf ? e2_a_s : (e2_b_s ^ e2_op_sub);
    // Both operands zero, or an exact cancellation in the close path. The
    // sign rule reads the ORIGINAL operand order (an exchange cannot happen
    // when the magnitudes are equal, the only way either arm fires).
    wire add_r_is_0    = ((e2_a_zero && e2_b_zero) || (e2_close_sel && cls_r_is_0))
                       && !add_r_is_qnan;
    wire add_0_sign    = (e2_op_add && (( e2_a_s &&  e2_b_s)
                                     || (( e2_a_s ||  e2_b_s) && (e2_rm == 3'b010))))
                      || (e2_op_sub && (( e2_a_s && !e2_b_s)
                                     || (( e2_a_s || !e2_b_s) && (e2_rm == 3'b010))));

    //-- min / max ------------------------------------------------------------
    wire mm_both_nan = (e2_a_snan || e2_a_qnan) && (e2_b_snan || e2_b_qnan);
    wire mm_a_nan    = e2_a_snan || e2_a_qnan;
    wire mm_b_nan    = e2_b_snan || e2_b_qnan;
    wire mm_both_0   = e2_a_zero && e2_b_zero;
    wire mm_pick_neg = e2_op_max ? (e2_a_s && e2_b_s) : (e2_a_s || e2_b_s);
    wire [62:0] mm_a_mag = e2_double ? e2_a_raw[62:0] : {32'b0, e2_a_raw[30:0]};
    wire [62:0] mm_b_mag = e2_double ? e2_b_raw[62:0] : {32'b0, e2_b_raw[30:0]};
    wire        mm_a_lt_b = (e2_a_s && !e2_b_s) ? !mm_both_0
                          : (!e2_a_s && e2_b_s) ? 1'b0
                          : e2_a_s              ? (mm_a_mag > mm_b_mag)
                                                : (mm_a_mag < mm_b_mag);
    // min takes a when a < b; max takes a when a is NOT less than b.
    wire mm_take_b   = e2_op_max ? mm_a_lt_b : !mm_a_lt_b;

    reg [63:0] mm_result;
    always @* begin
        if (mm_both_nan)
            mm_result = canon_qnan;
        else if (mm_a_nan)
            mm_result = e2_b_raw;
        else if (mm_b_nan)
            mm_result = e2_a_raw;
        else if (mm_both_0)
            mm_result = e2_double ? {mm_pick_neg, 63'b0}
                                  : {32'hffffffff, mm_pick_neg, 31'b0};
        else
            mm_result = mm_take_b ? e2_b_raw : e2_a_raw;
    end

    //-- the operation-select result ------------------------------------------
    wire        add_op_sel = e2_op_max || e2_op_min || e2_op_cmp;
    wire [63:0] add_sel_result = e2_op_cmp ? {63'b0, e2_cmp_res} : mm_result;

    //-- the special-result merge ---------------------------------------------
    wire        add_spe_sel = add_r_is_qnan || add_r_is_inf || add_r_is_0;
    reg  [63:0] add_spe_result;
    always @* begin
        if (add_r_is_qnan)
            add_spe_result = canon_qnan;
        else if (add_r_is_inf)
            add_spe_result = e2_double ? {add_inf_sign, 11'h7ff, 52'b0}
                                       : {32'hffffffff, add_inf_sign, 8'hff, 23'b0};
        else
            add_spe_result = e2_double ? {add_0_sign, 63'b0}
                                       : {32'hffffffff, add_0_sign, 31'b0};
    end

    //-- the three-path normal-result mux -------------------------------------
    reg  [63:0] add_nor_result;
    reg  [4:0]  add_nor_flags;
    always @* begin
        case ({e2_bypass_sel, e2_far_sel, e2_close_sel})
            3'b100:  begin add_nor_result = byp_result; add_nor_flags = byp_flags; end
            3'b010:  begin add_nor_result = far_result; add_nor_flags = far_flags; end
            3'b001:  begin add_nor_result = cls_result; add_nor_flags = cls_flags; end
            default: begin add_nor_result = 64'b0;      add_nor_flags = 5'b0;      end
        endcase
    end

    wire [63:0] fadd_ex2_result = add_op_sel  ? add_sel_result
                                : add_spe_sel ? add_spe_result
                                              : add_nor_result;

    //-- the flags -------------------------------------------------------------
    // NV: an sNaN on any operation; inf minus inf; a qNaN on `flt`/`fle` and
    // NOT on `feq` (RISC-V's polarity).
    wire add_nv = e2_a_snan || e2_b_snan
               || ((e2_op_add || e2_op_sub) && e2_a_inf && e2_b_inf && e2_act_sub)
               || ((e2_cmp_flt || e2_cmp_fle) && (e2_a_qnan || e2_b_qnan));
    // NX/OF/UF come from whichever path ran, suppressed on a special result
    // and on every compare and min/max.
    wire        add_path_nx = e2_bypass_sel ? byp_nx : add_nor_flags[0];
    wire [4:0]  fadd_ex2_flags =
        {add_nv, 1'b0,
         (!add_spe_sel && !add_op_sel) ? add_nor_flags[2] : 1'b0,
         (!add_spe_sel && !add_op_sel) ? add_nor_flags[1] : 1'b0,
         (!add_spe_sel && !add_op_sel) ? add_path_nx      : 1'b0};

    //=========================================================================
    // SECTION 9: FSPU -- SIGN INJECTION, fclass, AND fmv.{x.w,w.x,x.d,d.x}
    // (M5 Task 7 adds the fmv.* pair below; ../rv12/rtl/FPUAlu.v:1019-1114
    // is the donor, adapted from its two-bit {MV_FX,MV_XF} encoding to
    // rv906's single FUNC_SPU_MV trigger + FUNC_SPU_MV_XF direction bit)
    //=========================================================================
    wire spu_op_sgnjx = idu_fpu_ex1_func[FUNC_SPU_SGN] && idu_fpu_ex1_func[FUNC_SPU_SGN_X];
    wire spu_op_sgnjn = idu_fpu_ex1_func[FUNC_SPU_SGN] && idu_fpu_ex1_func[FUNC_SPU_SGN_N];
    wire spu_op_sgnj  = idu_fpu_ex1_func[FUNC_SPU_SGN] && idu_fpu_ex1_func[FUNC_SPU_SGN_J];
    wire spu_op_class = idu_fpu_ex1_func[FUNC_CLASS];
    wire spu_op_mv_fx = idu_fpu_ex1_func[FUNC_SPU_MV] && !idu_fpu_ex1_func[FUNC_SPU_MV_XF]; // fmv.w.x/d.x
    wire spu_op_mv_xf = idu_fpu_ex1_func[FUNC_SPU_MV] &&  idu_fpu_ex1_func[FUNC_SPU_MV_XF]; // fmv.x.w/x.d

    // The unboxed single views: fsgnj*.s and fclass.s are defined on
    // single-precision VALUES and therefore unbox.
    wire [31:0] spu_a_s32 = a_cnan ? 32'h7fc00000 : idu_fpu_ex1_fsrc0_data[31:0];
    wire [31:0] spu_b_s32 = b_cnan ? 32'h7fc00000 : idu_fpu_ex1_fsrc1_data[31:0];

    // Sign injection operates on the BOXED 64-bit value so the box survives.
    wire [63:0] spu_sgnj_d  = {idu_fpu_ex1_fsrc1_data[63],  idu_fpu_ex1_fsrc0_data[62:0]};
    wire [63:0] spu_sgnjn_d = {~idu_fpu_ex1_fsrc1_data[63], idu_fpu_ex1_fsrc0_data[62:0]};
    wire [63:0] spu_sgnjx_d = {idu_fpu_ex1_fsrc0_data[63] ^ idu_fpu_ex1_fsrc1_data[63], idu_fpu_ex1_fsrc0_data[62:0]};
    wire [63:0] spu_sgnj_s  = {32'hffffffff, spu_b_s32[31],  spu_a_s32[30:0]};
    wire [63:0] spu_sgnjn_s = {32'hffffffff, ~spu_b_s32[31], spu_a_s32[30:0]};
    wire [63:0] spu_sgnjx_s = {32'hffffffff, spu_a_s32[31] ^ spu_b_s32[31],
                               spu_a_s32[30:0]};

    // fclass: the flat ten-bit decode, zero-extended per format. The eight
    // non-NaN bits carry `&& !a_cnan`; the two NaN bits do not -- that is the
    // box check reaching fclass (a broken box classifies as the canonical
    // qNaN and as NOTHING ELSE).
    wire [63:0] spu_class = { 54'b0,
                              a_is_qnan,
                              a_is_snan,
                              !a_s && a_e_max && a_f_z   && !a_cnan, // +inf
                              !a_s && !a_e_max && !a_e_z && !a_cnan, // +normal
                              !a_s && a_e_z && !a_f_z    && !a_cnan, // +subnormal
                              !a_s && a_e_z && a_f_z     && !a_cnan, // +0
                               a_s && a_e_z && a_f_z     && !a_cnan, // -0
                               a_s && a_e_z && !a_f_z    && !a_cnan, // -subnormal
                               a_s && !a_e_max && !a_e_z && !a_cnan, // -normal
                               a_s && a_e_max && a_f_z   && !a_cnan }; // -inf

    // fmv.w.x/fmv.d.x -- int->fp passthrough. Source is
    // idu_fpu_ex1_fsrc0_data, GPR-valued for this op via IDU.v's
    // dis_gpr_fsrc0 mux (M5 Task 7). fmv.w.x boxes into NaN-boxed single;
    // fmv.d.x is the 64-bit identity (../rv12/rtl/FPUAlu.v:1019-1114).
    wire [63:0] spu_mtvr   = idu_fpu_ex1_fsrc0_data;
    wire [63:0] spu_mtvr_s = {32'hffffffff, spu_mtvr[31:0]};
    wire [63:0] spu_mtvr_d = spu_mtvr;

    // fmv.x.w/fmv.x.d -- fp->int passthrough, reading the RAW (boxed or
    // not) src0 bits -- no NaN-box canonicalization, unlike spu_a_s32.
    // fmv.x.w sign-extends the raw 32-bit view; fmv.x.d is identity.
    wire [63:0] spu_mfvr_s = {{32{idu_fpu_ex1_fsrc0_data[31]}}, idu_fpu_ex1_fsrc0_data[31:0]};
    wire [63:0] spu_mfvr_d = idu_fpu_ex1_fsrc0_data;

    // The freg-destination result -- leaves at EX1, D1 (no completion latch).
    wire [63:0] fspu_ex1_result =
          ({64{spu_op_sgnj  &&  f_double}} & spu_sgnj_d)
        | ({64{spu_op_sgnjn &&  f_double}} & spu_sgnjn_d)
        | ({64{spu_op_sgnjx &&  f_double}} & spu_sgnjx_d)
        | ({64{spu_op_sgnj  && !f_double}} & spu_sgnj_s)
        | ({64{spu_op_sgnjn && !f_double}} & spu_sgnjn_s)
        | ({64{spu_op_sgnjx && !f_double}} & spu_sgnjx_s)
        | ({64{spu_op_mv_fx &&  f_double}} & spu_mtvr_d)
        | ({64{spu_op_mv_fx && !f_double}} & spu_mtvr_s);

    // ... and the mfvr (integer-destination) answer: fclass or fmv.x.w/x.d.
    wire [63:0] fspu_mfvr_data = ({64{spu_op_class}} & spu_class)
                                | ({64{spu_op_mv_xf &&  f_double}} & spu_mfvr_d)
                                | ({64{spu_op_mv_xf && !f_double}} & spu_mfvr_s);

    //=========================================================================
    // SECTION 10: FCNVT EX1 -- FORMAT DECODE AND SOURCE PREPARE (M5 Task 7
    // adds int<->float below the pre-existing f2f-only decode; donor
    // ../rv12/rtl/FPUAlu.v:1116-1315 SECTION 10/11a/11b, adapted from its
    // 4-bit {SRC_FLT,SRC_SI,DEST_FLT,DEST_SI} scheme to rv906's
    // FUNC_CVT_INT/FUNC_CVT_F2I trigger+direction pair)
    //=========================================================================
    wire cvt_widden = idu_fpu_ex1_func[FUNC_CVT_WIDDEN] && !idu_fpu_ex1_func[FUNC_CVT_NARROW];
    wire cvt_narrow = !idu_fpu_ex1_func[FUNC_CVT_WIDDEN] && idu_fpu_ex1_func[FUNC_CVT_NARROW];
    wire cvt_equal  = !idu_fpu_ex1_func[FUNC_CVT_WIDDEN] && !idu_fpu_ex1_func[FUNC_CVT_NARROW];

    // int<->float trigger/direction/width bits (int-convert ops always
    // carry cvt_widden=cvt_narrow=0, i.e. cvt_equal=1 -- neither WIDDEN nor
    // NARROW is ever set by IDU.v's fcvt.*.{w,wu,l,lu} decode arms).
    wire cvt_is_int   = idu_fpu_ex1_func[FUNC_CVT_INT];
    wire cvt_f2i      = idu_fpu_ex1_func[FUNC_CVT_F2I];      // 1=fp->int, 0=int->fp
    wire cvt_unsigned = idu_fpu_ex1_func[FUNC_CVT_UNSIGNED];
    wire cvt_wide64   = idu_fpu_ex1_func[FUNC_CVT_WIDE64];   // int-side width

    wire cvt_src_l64  = idu_fpu_ex1_func[FUNC_DOUBLE] || (idu_fpu_ex1_func[FUNC_B_SINGLE] && cvt_narrow);
    // cvt_src_l32 additionally fires for a single-precision f2i source --
    // FUNC_B_SINGLE is f2f-only plumbing (never set by the int-convert
    // decode arms) so the box-check below would otherwise never trigger
    // for fcvt.w/wu/l/lu.s.
    wire cvt_src_l32  = (idu_fpu_ex1_func[FUNC_B_SINGLE] && !cvt_narrow)
                      || (cvt_is_int && cvt_f2i && !idu_fpu_ex1_func[FUNC_DOUBLE]);
    wire cvt_dest_l64 = (cvt_src_l64 && cvt_equal) || (cvt_src_l32 && cvt_widden);

    wire cvt_dest_dbl = cvt_dest_l64;   // cvt_dest_flt hardwired 1 (D-TASK3-2)

    //-- the FLOAT source, classified in its own width -- reused UNCHANGED
    //-- for f2i: cvt_src_l64/cvt_src_l32 above already collapse to
    //-- f_double/!f_double for every int-convert op, so this block needs
    //-- no int-convert-specific logic of its own. ----------------------
    wire        cvt_s_dbl  = cvt_src_l64;
    wire        cvt_f_cnan = cvt_src_l32 && !(&idu_fpu_ex1_fsrc0_data[63:32]);
    wire        cvt_f_s    = cvt_s_dbl ? idu_fpu_ex1_fsrc0_data[63] : idu_fpu_ex1_fsrc0_data[31];
    wire [10:0] cvt_f_ef   = cvt_s_dbl ? idu_fpu_ex1_fsrc0_data[62:52] : {3'b0, idu_fpu_ex1_fsrc0_data[30:23]};
    wire [51:0] cvt_f_frac = cvt_s_dbl ? idu_fpu_ex1_fsrc0_data[51:0] : {idu_fpu_ex1_fsrc0_data[22:0], 29'b0};
    wire        cvt_f_emax = cvt_s_dbl ? (&idu_fpu_ex1_fsrc0_data[62:52]) : (&idu_fpu_ex1_fsrc0_data[30:23]);
    wire        cvt_f_ez   = ~|cvt_f_ef;
    wire        cvt_f_fz   = ~|cvt_f_frac;
    wire        cvt_f_msb  = cvt_s_dbl ? idu_fpu_ex1_fsrc0_data[51] : idu_fpu_ex1_fsrc0_data[22];
    wire        cvt_f_snan = cvt_f_emax && !cvt_f_msb && !cvt_f_fz && !cvt_f_cnan;
    wire        cvt_f_qnan = (cvt_f_emax && cvt_f_msb) || cvt_f_cnan;
    wire        cvt_f_inf  = cvt_f_emax && cvt_f_fz && !cvt_f_cnan;
    wire        cvt_f_zero = cvt_f_ez && cvt_f_fz && !cvt_f_cnan;
    wire [52:0] cvt_f_sig  = cvt_f_cnan ? 53'b0 : {~cvt_f_ez, cvt_f_frac};
    // The true exponent, SECTION 2's max(E,1) less the source bias.
    wire signed [12:0] cvt_f_eunb =
        $signed({2'b0, (cvt_f_ez ? 11'd1 : cvt_f_ef)})
      - $signed({2'b0, (cvt_s_dbl ? BIAS_D : BIAS_S)});

    //-- the INTEGER source, prepared for i2f (../rv12/rtl/FPUAlu.v ~1200,
    //-- adapted: cvt_src_si -> cvt_int_signed, cvt_src_l64 -> cvt_wide64).
    //-- idu_fpu_ex1_fsrc0_data is GPR-valued here via IDU.v's dis_gpr_fsrc0
    //-- mux. Meaningless (and unused) when !cvt_is_int or cvt_f2i.
    wire cvt_int_signed = !cvt_unsigned;   // shared: i2f src-signed / f2i dst-signed
    wire cvt_i_neg = cvt_is_int && !cvt_f2i && cvt_int_signed
                   && (cvt_wide64 ? idu_fpu_ex1_fsrc0_data[63] : idu_fpu_ex1_fsrc0_data[31]);
    wire [63:0] cvt_i_ext = cvt_wide64 ? idu_fpu_ex1_fsrc0_data
                          : {{32{cvt_int_signed & idu_fpu_ex1_fsrc0_data[31]}}, idu_fpu_ex1_fsrc0_data[31:0]};
    wire [63:0] cvt_i_mag = cvt_i_neg ? (~cvt_i_ext + 64'd1) : cvt_i_ext;

    //-- FLATTENED EX1->EX2 (D1) -- see SECTION 4's note; f2f-only subset.
    wire        e2_cvt_dest_dbl = cvt_dest_dbl;
    wire        e2_cvt_f_s      = cvt_f_s;
    wire        e2_cvt_f_snan   = cvt_f_snan;
    wire        e2_cvt_f_qnan   = cvt_f_qnan;
    wire        e2_cvt_f_inf    = cvt_f_inf;
    wire        e2_cvt_f_zero   = cvt_f_zero;
    wire [52:0] e2_cvt_f_sig    = cvt_f_sig;
    wire signed [12:0] e2_cvt_f_eunb = cvt_f_eunb;
    wire [2:0]  e2_cvt_rm       = rm_eff;

    //=========================================================================
    // SECTION 11: FCNVT -- FLOAT->FLOAT
    //=========================================================================
    // The source's exponent is re-biased to the destination's format and the
    // packer does the rest -- including fcvt.s.d's rounding/overflow/
    // gradual-underflow. fcvt.d.s is exact and rides the same path.
    wire signed [12:0] f2f_e = e2_cvt_f_eunb
                             + $signed({2'b0, (e2_cvt_dest_dbl ? BIAS_D : BIAS_S)});
    wire [PW-1:0] f2f_p = {1'b0, e2_cvt_f_sig, 3'b000};

    wire [PACK_W-1:0] f2f_pack = fp_pack(e2_cvt_f_s, f2f_e, f2f_p, 1'b0,
                                         e2_cvt_rm, e2_cvt_dest_dbl);
    wire [63:0] f2f_canon = e2_cvt_dest_dbl ? {1'b0, 11'h7ff, 1'b1, 51'b0}
                                            : {32'hffffffff, 1'b0, 8'hff, 1'b1, 22'b0};
    wire [63:0] f2f_inf   = e2_cvt_dest_dbl ? {e2_cvt_f_s, 11'h7ff, 52'b0}
                                            : {32'hffffffff, e2_cvt_f_s, 8'hff, 23'b0};
    wire [63:0] f2f_zero  = e2_cvt_dest_dbl ? {e2_cvt_f_s, 63'b0}
                                            : {32'hffffffff, e2_cvt_f_s, 31'b0};
    wire [63:0] f2f_result = (e2_cvt_f_snan || e2_cvt_f_qnan) ? f2f_canon
                            : e2_cvt_f_inf                     ? f2f_inf
                            : e2_cvt_f_zero                    ? f2f_zero
                                                                : f2f_pack[PACK_W-1:5];
    wire [4:0]  f2f_flags  = (e2_cvt_f_snan)                    ? 5'b10000
                           : (e2_cvt_f_qnan || e2_cvt_f_inf || e2_cvt_f_zero) ? 5'b0
                                                                : f2f_pack[4:0];

    //=========================================================================
    // SECTION 11a: FCNVT -- FLOAT->INTEGER (M5 Task 7; donor verbatim,
    // ../rv12/rtl/FPUAlu.v ~1220-1290. e2_cvt_dest_si/l64 -> cvt_int_signed/
    // cvt_wide64; the NaN->MAXIMUM-only saturation asymmetry is DELIBERATE
    // -- do not "tidy" it into a shared max/min term, see donor comment and
    // ct_fcnvt_double_dp.v:1212-1236.)
    //=========================================================================
    wire f2i_huge  = (cvt_f_eunb >= 13'sd64) || cvt_f_inf || cvt_f_snan || cvt_f_qnan;
    wire f2i_tiny  = (cvt_f_eunb <  -13'sd1);
    wire signed [12:0] f2i_rsh_s = 13'sd52 - cvt_f_eunb;
    wire signed [12:0] f2i_lsh_s = cvt_f_eunb - 13'sd52;
    wire [6:0]  f2i_rsh   = f2i_rsh_s[6:0];
    wire [6:0]  f2i_lsh   = f2i_lsh_s[6:0];

    // Donor verbatim (../rv12/rtl/FPUAlu.v:1239-1256; e2_cvt_f_sig ->
    // cvt_f_sig, e2_cvt_f_eunb -> cvt_f_eunb): the magnitude is the 53-bit
    // significand zero-extended to 65 and shifted into place; the guard/
    // sticky terms mask the RAW 53-bit significand (no pre-widening, no
    // shift-amount guards); the huge branch parks f2i_mag all-ones (the
    // f2i_rmag below re-asserts them independently of f2i_mag).
    reg  [64:0] f2i_mag;
    reg         f2i_g, f2i_s;
    always @* begin
        f2i_mag = 65'b0;
        f2i_g   = 1'b0;
        f2i_s   = 1'b0;
        if (f2i_huge) begin
            f2i_mag = {65{1'b1}};
        end else if (f2i_tiny) begin
            f2i_s   = |cvt_f_sig;
        end else if (cvt_f_eunb >= 13'sd52) begin
            f2i_mag = {12'b0, cvt_f_sig} << f2i_lsh;
        end else begin
            f2i_mag = {12'b0, cvt_f_sig} >> f2i_rsh;
            f2i_g   = |(cvt_f_sig & ({53{1'b1}} & (53'd1 << (f2i_rsh - 7'd1))));
            f2i_s   = |(cvt_f_sig & ~({53{1'b1}} << (f2i_rsh - 7'd1)));
        end
    end

    wire f2i_inc  = round_up(cvt_f_s, f2i_mag[0], f2i_g, f2i_s, rm_eff);
    wire [64:0] f2i_rmag = f2i_huge ? {65{1'b1}} : (f2i_mag + {64'b0, f2i_inc});

    wire f2i_nan = cvt_f_snan || cvt_f_qnan;

    // Donor verbatim (../rv12/rtl/FPUAlu.v:1264-1271): the range terms
    // compare the ROUNDED 65-bit magnitude against the destination's
    // max/min constants. (The bit-test forms that stood here before
    // mis-placed sat_si64/sat_ui64 one bit early -- 2**62/2**63 instead
    // of 2**63/2**64 -- and dropped min_si32's "above 2**31 with the low
    // 31 bits clear" arm, e.g. exactly -2**32.)
    wire f2i_sat_si64 = !cvt_f_s && (f2i_rmag >  {2'b0, {63{1'b1}}});
    wire f2i_min_si64 =  cvt_f_s && (f2i_rmag >  {1'b0, 1'b1, 63'b0});
    wire f2i_sat_ui64 = !cvt_f_s && (f2i_rmag >  {1'b0, {64{1'b1}}});
    wire f2i_min_ui64 =  cvt_f_s && (|f2i_rmag);
    wire f2i_sat_si32 = !cvt_f_s && (f2i_rmag >  {34'b0, {31{1'b1}}});
    wire f2i_min_si32 =  cvt_f_s && (f2i_rmag >  {33'b0, 1'b1, 31'b0});
    wire f2i_sat_ui32 = !cvt_f_s && (f2i_rmag >  {33'b0, {32{1'b1}}});
    wire f2i_min_ui32 =  cvt_f_s && (|f2i_rmag);

    wire f2i_of = cvt_wide64 ? (cvt_int_signed ? f2i_sat_si64 : f2i_sat_ui64) : (cvt_int_signed ? f2i_sat_si32 : f2i_sat_ui32);
    wire f2i_uf = cvt_wide64 ? (cvt_int_signed ? f2i_min_si64 : f2i_min_ui64) : (cvt_int_signed ? f2i_min_si32 : f2i_min_ui32);

    // *** SATURATION TABLE ASYMMETRY IS DELIBERATE: a NaN converts to the
    // MAXIMUM representable value only, never the minimum (RISC-V spec). ***
    wire f2i_dst_max_s =  cvt_int_signed && (f2i_of || f2i_nan);
    wire f2i_dst_min_s =  cvt_int_signed &&  f2i_uf;
    wire f2i_dst_max_u = !cvt_int_signed && (f2i_of || f2i_nan);
    wire f2i_dst_min_u = !cvt_int_signed &&  f2i_uf;

    wire [63:0] f2i_raw64 = cvt_f_s ? (~f2i_rmag[63:0] + 64'd1) : f2i_rmag[63:0];
    wire [63:0] f2i_s64 = f2i_dst_max_s ? {1'b0,{63{1'b1}}} : f2i_dst_min_s ? {1'b1,63'b0} : f2i_raw64;
    wire [63:0] f2i_u64 = f2i_dst_max_u ? {64{1'b1}}        : f2i_dst_min_u ? 64'b0        : f2i_raw64;
    wire [31:0] f2i_s32 = f2i_dst_max_s ? {1'b0,{31{1'b1}}} : f2i_dst_min_s ? {1'b1,31'b0} : f2i_raw64[31:0];
    wire [31:0] f2i_u32 = f2i_dst_max_u ? {32{1'b1}}        : f2i_dst_min_u ? 32'b0        : f2i_raw64[31:0];

    wire [63:0] f2i_l64_result = cvt_int_signed ? f2i_s64 : f2i_u64;
    // Zero-extends by design -- the RV64 w/wu sign-extension is composed
    // below (f2i_xdata) and consumed at SECTION 13's xdata mux, matching
    // FMV.X.W's own boxing convention (SECTION 9's spu_mfvr_s).
    wire [63:0] f2i_l32_result = cvt_int_signed ? {32'b0, f2i_s32} : {32'b0, f2i_u32};
    wire [63:0] f2i_result = cvt_wide64 ? f2i_l64_result : f2i_l32_result;

    // RV64 GPR writeback boxing for the 32-bit answers -- fmv.x.w's own
    // sign-extension (SECTION 9's spu_mfvr_s shape), selected on the
    // integer-side width: l/lu pass through, w/wu sign-extend bit 31.
    // Matches the donor's crack, whose uop-1 is fmv.x.w for the whole
    // 32-bit family and fmv.x.d for l/lu (C910 ct_idu_id_split_short.v
    // :644-649/:677-682, ported in ../rv12's mk_fp_uop1).
    wire [63:0] f2i_xdata = cvt_wide64 ? f2i_result
                                        : {{32{f2i_result[31]}}, f2i_result[31:0]};

    wire f2i_nv = f2i_of || f2i_uf || f2i_nan;
    wire [4:0] f2i_flags = {f2i_nv, 1'b0, 1'b0, 1'b0, !f2i_nv && (f2i_g || f2i_s)};

    //=========================================================================
    // SECTION 11b: FCNVT -- INTEGER->FLOAT (M5 Task 7; donor verbatim,
    // ../rv12/rtl/FPUAlu.v ~1290-1315. e2_cvt_i_neg/_mag/_rm/_dest_dbl ->
    // cvt_i_neg/cvt_i_mag/idu_fpu_ex1_rm/f_double. rv12 special-cases
    // cvt_i_mag==0 explicitly rather than relying on fp_pack's own
    // internal zero-path -- preserved here, NOT redundant to drop.)
    //=========================================================================
    wire [7:0]  i2f_lz  = lzc64(cvt_i_mag);
    wire [7:0]  i2f_rsh = (i2f_lz < 8'd8) ? (8'd8 - i2f_lz) : 8'd0;
    wire [7:0]  i2f_lsh = (i2f_lz < 8'd8) ? 8'd0 : (i2f_lz - 8'd8);
    wire [63:0] i2f_sh  = (i2f_lz < 8'd8) ? (cvt_i_mag >> i2f_rsh) : (cvt_i_mag << i2f_lsh);
    wire [PW-1:0] i2f_p = i2f_sh[PW-1:0];
    wire        i2f_st  = (i2f_lz < 8'd8) && (|(cvt_i_mag & ~({64{1'b1}} << i2f_rsh)));
    wire signed [12:0] i2f_e = $signed({5'b0, (8'd63 - i2f_lz)}) + $signed({2'b0, (f_double ? BIAS_D : BIAS_S)});
    wire [PACK_W-1:0] i2f_pack = fp_pack(cvt_i_neg, i2f_e, i2f_p, i2f_st, rm_eff, f_double);
    wire [63:0] i2f_result = (cvt_i_mag == 64'b0) ? (f_double ? 64'b0 : {32'hffffffff, 32'b0}) : i2f_pack[PACK_W-1:5];
    wire [4:0]  i2f_flags  = (cvt_i_mag == 64'b0) ? 5'b0 : i2f_pack[4:0];

    //=========================================================================
    // SECTION 11c: FCNVT -- RESULT MERGE. f2f when neither WIDDEN/NARROW nor
    // FUNC_CVT_INT selects an int-convert path (mutually exclusive by
    // construction: IDU.v never sets both groups of bits on one dispatch).
    //=========================================================================
    wire [63:0] fcnvt_ex1_result = !cvt_is_int ? f2f_result
                                  : cvt_f2i     ? f2i_result
                                                : i2f_result;
    wire [4:0]  fcnvt_ex1_flags  = !cvt_is_int ? f2f_flags
                                  : cvt_f2i     ? f2i_flags
                                                : i2f_flags;

    //=========================================================================
    // SECTION 12: FMAU -- FUSED MULTIPLY-ADD (fmul.{s,d}, and the fmadd/
    // fmsub/fnmadd/fnmsub.{s,d} family). Algorithmic porting source:
    // ../rv12/rtl/FPUMul.v SECTIONS 3-9 (that file's own C910-derived FMA
    // datapath), FLATTENED per D1 exactly as SECTIONS 4-8 above flatten
    // FPUAlu.v -- every `fm_e2_*` wire below is a same-cycle EX1 alias, not
    // a registered EX2 value; a distinct `fm_e2_` prefix (rather than
    // colliding with FADD's own `e2_*` names above) is used only so a
    // reader can still align this section against ../rv12/rtl/FPUMul.v's
    // SECTION 7 register cut-line. rv12's own SECTION 10 result-tap
    // registers (EX3/EX4/EX5) are DROPPED ENTIRELY: there is only one
    // cycle here, so `fmau_ex1_result`/`fmau_ex1_flags` feed the result mux
    // in SECTION 13 directly, same shape as FALU/FCNVT above.
    //
    // {NEG,SUB,FUSED} is rv12 FPUMul.v's own SECTION 3 field name for the
    // three-bit fused-op sub-select (../rv12/rtl/FPUMul.v:59); rv906 does
    // not preserve the LOW-THREE-BITS placement (rvproc_pkg.sv's
    // FUNC_MAU_FUSED/_SUB/_NEG comment explains why: those bit positions
    // are already occupied by FADD/FSPU sub-group bits under this file's
    // established bit-reuse convention) but keeps the donor's three
    // independent flags and their meaning verbatim:
    //     fmadd = {NEG,SUB,FUSED} = 001   fmsub  = 011
    //     fnmsub                 = 111   fnmadd = 101
    //     plain fmul: FUSED = 0 (NEG/SUB don't-care)
    //=========================================================================
    wire op_fused   = idu_fpu_ex1_func[FUNC_MAU_FUSED];
    wire op_mau_sub = idu_fpu_ex1_func[FUNC_MAU_SUB];
    wire op_mau_neg = idu_fpu_ex1_func[FUNC_MAU_NEG];

    // -- source 2 / addend classify, gated on op_fused (rv12 D6: a plain
    // fmul forces the addend out at the CLASS TERMS, not the operand mux)
    wire        c_cnan  = op_fused && box_check_en && !(&idu_fpu_ex1_fsrc2_data[63:32]);
    wire        c_s_raw = f_double ? idu_fpu_ex1_fsrc2_data[63] : idu_fpu_ex1_fsrc2_data[31];
    wire [10:0] c_ef    = f_double ? idu_fpu_ex1_fsrc2_data[62:52] : {3'b0, idu_fpu_ex1_fsrc2_data[30:23]};
    wire [51:0] c_frac  = f_double ? idu_fpu_ex1_fsrc2_data[51:0] : {idu_fpu_ex1_fsrc2_data[22:0], 29'b0};
    wire        c_e_max = f_double ? (&idu_fpu_ex1_fsrc2_data[62:52]) : (&idu_fpu_ex1_fsrc2_data[30:23]);
    wire        c_e_z   = ~|c_ef;
    wire        c_f_z   = ~|c_frac;
    wire        c_f_msb = f_double ? idu_fpu_ex1_fsrc2_data[51] : idu_fpu_ex1_fsrc2_data[22];
    wire        c_is_snan = op_fused && c_e_max && !c_f_msb && !c_f_z && !c_cnan;
    wire        c_is_qnan = op_fused && ((c_e_max && c_f_msb) || c_cnan);
    wire        c_is_inf  = op_fused && c_e_max && c_f_z && !c_cnan;
    wire        c_is_zero = !op_fused || (c_e_z && c_f_z && !c_cnan);
    wire [52:0] c_sig53   = (!op_fused || c_cnan) ? 53'b0 : {~c_e_z, c_frac};
    wire [11:0] c_eeff    = c_e_z ? 12'd1 : {1'b0, c_ef};

    // Only FMAU needs "is this operand a finite normal/subnormal" (SECTION
    // 6's prod_is_inf term below); FALU has no equivalent use, so these
    // extend SECTION 3's classify set rather than living there.
    wire a_is_norm = !a_is_zero && !a_e_max && !a_cnan;
    wire b_is_norm = !b_is_zero && !b_e_max && !b_cnan;

    // -- SECTION 4 (FPUMul.v): effective signs/exponents ---------------------
    wire         prod_sign = a_s ^ b_s ^ op_mau_neg;
    wire         add_sign  = op_fused ? (c_s_raw ^ op_mau_sub ^ op_mau_neg) : prod_sign;
    wire         sub_vld   = op_fused && (prod_sign ^ add_sign);
    wire [10:0]  mau_bias  = f_double ? BIAS_D : BIAS_S;
    wire signed [13:0] e_a = $signed({2'b0, a_eeff}) - $signed({3'b0, mau_bias});
    wire signed [13:0] e_b = $signed({2'b0, b_eeff}) - $signed({3'b0, mau_bias});
    wire signed [13:0] e_c = $signed({2'b0, c_eeff}) - $signed({3'b0, mau_bias});
    wire signed [13:0] top_p = e_a + e_b + 14'sd1;
    wire signed [13:0] top_c = e_c;

    // -- SECTION 5 (FPUMul.v): exact product + alignment geometry -----------
    // D2: ONE Verilog `*` for the exact 106-bit product -- no Booth/Wallace
    // array (rv12's own decision; behavioral is correct here).
    wire [105:0] prod106  = a_sig53 * b_sig53;
    wire         top_p_vld = !(a_is_zero || b_is_zero);
    wire         top_c_vld = !c_is_zero;
    wire signed [13:0] mau_top = (top_p_vld && top_c_vld) ? ((top_p > top_c) ? top_p : top_c)
                                : top_p_vld ? top_p : top_c_vld ? top_c : top_p;
    wire signed [13:0] d_p_raw = top_p_vld ? (mau_top - top_p) : 14'sd0;
    wire signed [13:0] d_c_raw = top_c_vld ? (mau_top - top_c) : 14'sd0;
    wire [7:0] d_p = (d_p_raw >= 14'sd164) ? 8'd164 : d_p_raw[7:0];
    wire [7:0] d_c = (d_c_raw >= 14'sd164) ? 8'd164 : d_c_raw[7:0];

    // -- SECTION 6 (FPUMul.v): special results/NV ----------------------------
    wire prod_is_inf = (a_is_inf && b_is_norm) || (b_is_inf && a_is_norm) || (a_is_inf && b_is_inf);
    wire mau_nv = a_is_snan || b_is_snan || c_is_snan
               || (a_is_zero && b_is_inf) || (b_is_zero && a_is_inf)
               || (prod_is_inf && c_is_inf && sub_vld);
    wire mau_res_qnan = a_is_qnan || b_is_qnan || c_is_qnan || mau_nv;
    wire mau_res_inf  = !mau_res_qnan && (a_is_inf || b_is_inf || c_is_inf);
    wire mau_inf_sign = c_is_inf ? add_sign : prod_sign;
    wire mau_res_special = mau_res_qnan || mau_res_inf;
    wire [63:0] mau_special_data = mau_res_qnan
        ? (f_double ? {1'b0, 11'h7ff, 1'b1, 51'b0} : {32'hffffffff, 1'b0, 8'hff, 1'b1, 22'b0})
        : (f_double ? {mau_inf_sign, 11'h7ff, 52'b0} : {32'hffffffff, mau_inf_sign, 8'hff, 23'b0});
    wire [4:0]  mau_special_flags = {mau_nv, 4'b0};
    wire        mau_zero_sign = (prod_sign == add_sign) ? prod_sign : (rm_eff == 3'b010);

    //-- FLATTENED EX1->EX2 (D1): fm_e2_* wire ALIASES, see the section
    // banner above -- not registered.
    wire         fm_e2_double        = f_double;
    wire [105:0] fm_e2_prod106       = prod106;
    wire [52:0]  fm_e2_c_sig53       = c_sig53;
    wire [7:0]   fm_e2_d_p           = d_p;
    wire [7:0]   fm_e2_d_c           = d_c;
    wire         fm_e2_prod_sign     = prod_sign;
    wire         fm_e2_add_sign      = add_sign;
    wire signed [13:0] fm_e2_top     = mau_top;
    wire [10:0]  fm_e2_bias          = mau_bias;
    wire [2:0]   fm_e2_rm            = rm_eff;
    wire         fm_e2_special       = mau_res_special;
    wire [63:0]  fm_e2_special_data  = mau_special_data;
    wire [4:0]   fm_e2_special_flags = mau_special_flags;
    wire         fm_e2_zero_sign     = mau_zero_sign;

    // Leading-zero count over the 164-bit alignment field (rv12 FPUMul.v's
    // own `lzc164`; D4: a plain count taken AFTER the add, not a parallel
    // anticipator). Returns 164 for zero.
    localparam integer WF = 164;

    function [7:0] lzc164;
        input [WF-1:0] v;
        integer i;
        reg    found;
        begin
            lzc164 = 8'd164;
            found = 1'b0;
            for (i = WF-1; i >= 0; i = i - 1)
                if (v[i] && !found) begin
                    found = 1'b1;
                    lzc164 = 8'd163 - i[7:0];
                end
        end
    endfunction

    // -- SECTION 8 (FPUMul.v): 164-bit alignment + exact sum -----------------
    wire [WF-1:0] pf_base = {1'b0, fm_e2_prod106, 57'b0};
    wire [WF-1:0] cf_base = {1'b0, fm_e2_c_sig53, 110'b0};
    wire [WF-1:0] pf = pf_base >> fm_e2_d_p;
    wire [WF-1:0] cf = cf_base >> fm_e2_d_c;
    wire drop_p = |(pf_base & ~({WF{1'b1}} << fm_e2_d_p));
    wire drop_c = |(cf_base & ~({WF{1'b1}} << fm_e2_d_c));
    wire mau_same_sign = (fm_e2_prod_sign == fm_e2_add_sign);
    wire p_bigger = (pf > cf) || ((pf == cf) && drop_p);

    reg [WF-1:0] fsum;
    reg          fst;
    reg          fsign;
    always @* begin
        if (mau_same_sign) begin
            fsum  = pf + cf;
            fst   = drop_p || drop_c;
            fsign = fm_e2_prod_sign;
        end else if (p_bigger) begin
            fsum  = pf - cf - {{(WF-1){1'b0}}, drop_c};
            fst   = drop_p || drop_c;
            fsign = fm_e2_prod_sign;
        end else begin
            fsum  = cf - pf - {{(WF-1){1'b0}}, drop_p};
            fst   = drop_p || drop_c;
            fsign = fm_e2_add_sign;
        end
    end

    // -- SECTION 9 (FPUMul.v): normalize/single-pack/special-merge ----------
    wire [7:0] fsum_lz      = lzc164(fsum);
    wire [7:0] fsum_top_idx = 8'd163 - fsum_lz;
    wire       need_rsh     = (fsum_top_idx >= 8'd55);
    wire [7:0] map_rsh      = fsum_top_idx - 8'd55;
    wire [7:0] map_lsh      = 8'd55 - fsum_top_idx;
    wire [WF-1:0] fsum_rsh  = fsum >> map_rsh;
    wire [WF-1:0] fsum_lsh  = fsum << map_lsh;
    wire       map_st       = need_rsh ? |(fsum & ~({WF{1'b1}} << map_rsh)) : 1'b0;
    wire       fsum_zero    = (fsum == {WF{1'b0}});
    wire [PW-1:0] pack_p    = fsum_zero ? (fst ? {{(PW-1){1'b0}}, 1'b1} : {PW{1'b0}})
                            : need_rsh  ? fsum_rsh[PW-1:0]
                                        : fsum_lsh[PW-1:0];
    wire signed [13:0] pack_e14 = fsum_zero
        ? (fm_e2_top - 14'sd107 + $signed({3'b0, fm_e2_bias}))
        : (fm_e2_top + 14'sd1 - $signed({6'b0, fsum_lz}) + $signed({3'b0, fm_e2_bias}));
    wire signed [12:0] pack_e = pack_e14[12:0];

    wire [PACK_W-1:0] mau_packed = fp_pack(fsign, pack_e, pack_p, fst | map_st,
                                           fm_e2_rm, fm_e2_double);
    wire        mau_zero_taken = fsum_zero && !fst;
    wire [63:0] mau_zero_data  = fm_e2_double ? {fm_e2_zero_sign, 63'b0}
                                              : {32'hffffffff, fm_e2_zero_sign, 31'b0};

    wire [63:0] fmau_ex1_result = fm_e2_special  ? fm_e2_special_data
                                : mau_zero_taken  ? mau_zero_data
                                                  : mau_packed[PACK_W-1:5];
    wire [4:0]  fmau_ex1_flags  = fm_e2_special  ? fm_e2_special_flags
                                : mau_zero_taken  ? 5'b0
                                                  : mau_packed[4:0];

    //=========================================================================
    // SECTION 14: FDSU -- DIVIDE/SQRT (fdiv.{s,d}, fsqrt.{s,d}). Structural
    // template: IU.v's SECTION DIV (IU.v:1084-1332), simplified per M5
    // design doc D1 (single-issue busy/stall FSM) minus the DIV_WFWB
    // grant-wait state -- RTU.v:961-963/974 confirms FPU's wbf0 writeback
    // has no arbiter ("FALU is the only wbf0 producer today";
    // `wbf0_vld_r <= fpu_rtu_ex1_falu_fvld` unconditional), so CMPLT always
    // returns straight to IDLE.
    //
    // Round count is a FIXED constant per format (donor aq_fdsu_scalar_
    // ctrl.v:417-423: double=29 rounds; single is CORRECTED to 14 here vs.
    // the donor comment's stale "15" -- verified against the donor's own
    // iteration-count arithmetic, not just the comment, in an earlier task).
    //=========================================================================
    localparam FDSU_IDLE = 2'b00, FDSU_BUSY = 2'b01, FDSU_CMPLT = 2'b10;

    wire fds_op_div  = idu_fpu_ex1_func[FUNC_FDSU_DIV];
    wire fds_op_sqrt = idu_fpu_ex1_func[FUNC_FDSU_SQRT];

    // -- FDIV special cases (IEEE 754), off the LIVE SECTION 3 classify
    // wires -- resolves same cycle as dispatch, exactly like DIV's own
    // abnormal fast path.
    wire fdiv_is_nan         = a_is_snan || a_is_qnan || b_is_snan || b_is_qnan
                              || (a_is_inf && b_is_inf) || (a_is_zero && b_is_zero);
    wire fdiv_nv             = a_is_snan || b_is_snan || (a_is_inf && b_is_inf)
                              || (a_is_zero && b_is_zero);
    wire fdiv_is_inf_result  = !fdiv_is_nan && (a_is_inf || b_is_zero);
    wire fdiv_dz             = !fdiv_is_nan && !a_is_inf && b_is_zero;
    wire fdiv_is_zero_result = !fdiv_is_nan && !fdiv_is_inf_result && (a_is_zero || b_is_inf);
    wire fdiv_sign           = a_s ^ b_s;
    wire fdiv_abnormal       = fdiv_is_nan || fdiv_is_inf_result || fdiv_is_zero_result;

    // -- FSQRT special cases. rs2/fsrc1 is unused (RISC-V spec); sqrt reads
    // only fsrc0/a.
    wire fsqrt_is_nan            = a_is_snan || a_is_qnan;
    wire fsqrt_nv                = a_is_snan || (a_s && !a_is_zero);
    wire fsqrt_result_is_a       = a_is_zero;
    wire fsqrt_result_is_neg_nan = a_s && !a_is_zero && !a_is_snan && !a_is_qnan;
    wire fsqrt_result_is_inf     = !a_s && a_is_inf;
    wire fsqrt_abnormal          = fsqrt_is_nan || fsqrt_result_is_a
                                  || fsqrt_result_is_neg_nan || fsqrt_result_is_inf;

    wire fds_abnormal_res_vld = (fds_op_div && fdiv_abnormal) || (fds_op_sqrt && fsqrt_abnormal);
    wire fds_nv = (fds_op_div && fdiv_nv) || (fds_op_sqrt && fsqrt_nv);
    wire fds_dz = fds_op_div && fdiv_dz;

    wire [63:0] fds_qnan_data = f_double ? {1'b0, 11'h7ff, 1'b1, 51'b0}
                                          : {32'hffffffff, 1'b0, 8'hff, 1'b1, 22'b0};
    wire        fds_inf_sign  = fds_op_div ? fdiv_sign : 1'b0;
    wire [63:0] fds_inf_data  = f_double ? {fds_inf_sign, 11'h7ff, 52'b0}
                                          : {32'hffffffff, fds_inf_sign, 8'hff, 23'b0};
    wire [63:0] fds_zero_data = f_double ? {fdiv_sign, 63'b0} : {32'hffffffff, fdiv_sign, 31'b0};

    wire [63:0] fds_abnormal_data =
          (fds_op_div && fdiv_is_nan)                                ? fds_qnan_data
        : (fds_op_div && fdiv_is_inf_result)                         ? fds_inf_data
        : (fds_op_div && fdiv_is_zero_result)                        ? fds_zero_data
        : (fds_op_sqrt && (fsqrt_is_nan || fsqrt_result_is_neg_nan)) ? fds_qnan_data
        : (fds_op_sqrt && fsqrt_result_is_a)                         ? idu_fpu_ex1_fsrc0_data
        : (fds_op_sqrt && fsqrt_result_is_inf)                       ? fds_inf_data
        :                                                              64'b0;
    wire [4:0] fds_abnormal_flags = {fds_nv, fds_dz, 3'b000};

    wire [5:0] fds_round_count = f_double ? 6'd29 : 6'd14;

    // -- FSM --
    reg [1:0] fdsu_state;
    reg [5:0] fdsu_iter_left;

    wire fdsu_new_dispatch = idu_fpu_ex1_fdsu_sel && (fdsu_state == FDSU_IDLE);
    wire fdsu_iter_start   = fdsu_new_dispatch && !fds_abnormal_res_vld;
    wire fdsu_ex1_res_vld  = fdsu_new_dispatch && fds_abnormal_res_vld;

    wire [1:0] fdsu_next_state =
          (fdsu_state == FDSU_IDLE) ? (fdsu_iter_start ? FDSU_BUSY : FDSU_IDLE)
        : (fdsu_state == FDSU_BUSY) ? ((fdsu_iter_left <= 6'd1) ? FDSU_CMPLT : FDSU_BUSY)
        :                             FDSU_IDLE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fdsu_state <= FDSU_IDLE;
        else
            fdsu_state <= fdsu_next_state;
    end

    wire fdsu_cmplt_now = (fdsu_state == FDSU_CMPLT) || (fdsu_state == FDSU_IDLE && fdsu_ex1_res_vld);

    // -- Dispatch-time operand/control latch (mirrors IU.v's div_dividend_
    // flop/div_preg_reg discipline exactly -- the live idu_fpu_ex1_* bus is
    // not guaranteed stable for the whole multi-cycle busy period).
    reg [52:0] fdsu_a_sig_flop, fdsu_b_sig_flop;
    reg [11:0] fdsu_a_eeff_flop, fdsu_b_eeff_flop;
    reg        fdsu_a_s_flop, fdsu_b_s_flop;
    reg        fdsu_double_flop;
    reg        fdsu_op_div_flop;
    reg [2:0]  fdsu_rm_flop;
    reg [GPR_IDX_WIDTH-1:0] fdsu_preg_flop;

    always @(posedge clk) begin
        if (fdsu_iter_start) begin
            fdsu_a_sig_flop   <= a_sig53;
            fdsu_b_sig_flop   <= b_sig53;
            fdsu_a_eeff_flop  <= a_eeff;
            fdsu_b_eeff_flop  <= b_eeff;
            fdsu_a_s_flop     <= a_s;
            fdsu_b_s_flop     <= b_s;
            fdsu_double_flop  <= f_double;
            fdsu_op_div_flop  <= fds_op_div;
            // M5 Task 11 BUG 2: latch the RESOLVED rm (rm_eff), so a DYN
            // fdiv/fsqrt holds the frm-resolved value across its multi-cycle
            // busy period rather than re-reading the live (post-advance) bus.
            fdsu_rm_flop      <= rm_eff;
            fdsu_iter_left    <= fds_round_count;
        end
        else if (fdsu_state == FDSU_BUSY)
            fdsu_iter_left <= fdsu_iter_left - 6'd1;
    end

    always @(posedge clk) begin
        if (fdsu_new_dispatch)
            fdsu_preg_flop <= idu_fpu_ex1_dst0_reg;
    end

    // -- Real-compute helper functions --
    // 53-bit leading-zero count. The real-compute path is only entered for
    // finite nonzero operands, and SECTION 3's a_sig53/b_sig53 construction
    // forces bit52 set for any NORMAL operand, so a nonzero LZC only
    // actually occurs for a SUBNORMAL operand.
    function [7:0] lzc53;
        input [52:0] v;
        integer i;
        reg    found;
        begin
            lzc53 = 8'd53;
            found = 1'b0;
            for (i = 52; i >= 0; i = i - 1)
                if (v[i] && !found) begin
                    found = 1'b1;
                    lzc53 = 8'd52 - i[7:0];
                end
        end
    endfunction

    // Bit-by-bit restoring integer square root, 120-bit input -> 60-bit
    // output. Behavioral only -- mirrors this codebase's existing precedent
    // of plain `/`/`%` for IU.v's DIV (no digit-recurrence array ported).
    function [59:0] fdsu_isqrt120;
        input [119:0] x;
        integer i;
        reg [119:0] rem, root_acc, try;
        begin
            rem      = x;
            root_acc = 120'b0;
            for (i = 59; i >= 0; i = i - 1) begin
                try = root_acc | ({119'b0, 1'b1} << i);
                if ((try * try) <= rem)
                    root_acc = try;
            end
            fdsu_isqrt120 = root_acc[59:0];
        end
    endfunction

    // -- FDIV real-compute datapath (both operands finite nonzero) --
    wire [7:0]  fdsu_a_lza = lzc53(fdsu_a_sig_flop);
    wire [7:0]  fdsu_b_lza = lzc53(fdsu_b_sig_flop);
    wire [52:0] fdsu_a_norm = fdsu_a_sig_flop << fdsu_a_lza;
    wire [52:0] fdsu_b_norm = fdsu_b_sig_flop << fdsu_b_lza;
    wire [10:0] fdsu_bias   = fdsu_double_flop ? BIAS_D : BIAS_S;
    wire signed [13:0] fdsu_e_a = $signed({2'b0, fdsu_a_eeff_flop})
                                 - $signed({6'b0, fdsu_a_lza}) - $signed({3'b0, fdsu_bias});
    wire signed [13:0] fdsu_e_b = $signed({2'b0, fdsu_b_eeff_flop})
                                 - $signed({6'b0, fdsu_b_lza}) - $signed({3'b0, fdsu_bias});

    // Wide division: both operands normalized to [2^52,2^53), so the ratio
    // lies in (0.5,2) -- at most a 1-bit leading-position ambiguity.
    wire [111:0] fdiv_wide_dividend  = {fdsu_a_norm, 59'b0};
    wire [111:0] fdiv_wide_divisor   = {59'b0, fdsu_b_norm};
    wire [111:0] fdiv_wide_quotient  = fdiv_wide_dividend / fdiv_wide_divisor;
    wire [111:0] fdiv_wide_remainder = fdiv_wide_dividend % fdiv_wide_divisor;
    wire         fdiv_lead59 = fdiv_wide_quotient[59];

    // Two FIXED-width constant slices selected by a runtime mux (Verilog
    // disallows a variable-width bit-select) -- both align the true
    // leading bit to LOCAL position 55 of a PW=57 field.
    wire [PW-1:0] fdiv_pack_p = fdiv_lead59 ? {1'b0, fdiv_wide_quotient[59:4]}
                                            : {1'b0, fdiv_wide_quotient[58:3]};
    wire fdiv_dropped_st = fdiv_lead59 ? (|fdiv_wide_quotient[3:0]) : (|fdiv_wide_quotient[2:0]);
    wire fdiv_sticky     = fdiv_dropped_st || (fdiv_wide_remainder != 112'b0);

    // value = wide_quotient * 2^(e_a-e_b-59) exactly; normalizing to LOCAL
    // bit55 needs shift_amt=4 (lead59) or 3 (lead58) -- the lead58 case
    // needs an extra -1 on the exponent fed to fp_pack to compensate
    // (verified algebraically against fp_pack's own "leading bit at
    // P[55], e_in is the biased FIELD exponent as if it were already
    // there" convention).
    wire signed [13:0] fdiv_pack_e14 = $signed({3'b0, fdsu_bias}) + fdsu_e_a - fdsu_e_b
                                      - (fdiv_lead59 ? 14'sd0 : 14'sd1);
    wire fdiv_pack_sign = fdsu_a_s_flop ^ fdsu_b_s_flop;

    // -- FSQRT real-compute datapath (operand finite, positive, nonzero) --
    wire fsqrt_ea_odd = fdsu_e_a[0];
    wire [53:0] fsqrt_adj_mant = fsqrt_ea_odd ? {fdsu_a_norm, 1'b0} : {1'b0, fdsu_a_norm};
    wire signed [13:0] fsqrt_adj_ea  = fsqrt_ea_odd ? (fdsu_e_a - 14'sd1) : fdsu_e_a;
    wire signed [13:0] fsqrt_half_ea = fsqrt_adj_ea >>> 1;
    wire [119:0] fsqrt_scaled  = {fsqrt_adj_mant, 66'b0};
    wire [59:0]  fsqrt_root    = fdsu_isqrt120(fsqrt_scaled);
    wire [119:0] fsqrt_root_sq = fsqrt_root * fsqrt_root;
    wire         fsqrt_exact   = (fsqrt_scaled == fsqrt_root_sq);
    wire [PW-1:0] fsqrt_pack_p = {1'b0, fsqrt_root[59:4]};
    wire fsqrt_sticky = (|fsqrt_root[3:0]) || !fsqrt_exact;
    wire signed [13:0] fsqrt_pack_e14 = $signed({3'b0, fdsu_bias}) + fsqrt_half_ea;
    wire fsqrt_pack_sign = 1'b0;

    // -- Combine per op, pack, mux with the abnormal-path result --
    wire        fdsu_real_sign   = fdsu_op_div_flop ? fdiv_pack_sign : fsqrt_pack_sign;
    wire signed [12:0] fdsu_real_pack_e = fdsu_op_div_flop ? fdiv_pack_e14[12:0] : fsqrt_pack_e14[12:0];
    wire [PW-1:0] fdsu_real_pack_p = fdsu_op_div_flop ? fdiv_pack_p : fsqrt_pack_p;
    wire        fdsu_real_pack_st  = fdsu_op_div_flop ? fdiv_sticky : fsqrt_sticky;

    wire [PACK_W-1:0] fdsu_packed = fp_pack(fdsu_real_sign, fdsu_real_pack_e, fdsu_real_pack_p,
                                            fdsu_real_pack_st, fdsu_rm_flop, fdsu_double_flop);
    wire [63:0] fdsu_real_data  = fdsu_packed[PACK_W-1:5];
    wire [4:0]  fdsu_real_flags = fdsu_packed[4:0];

    wire [63:0] fdsu_ex1_result = (fdsu_state == FDSU_IDLE) ? fds_abnormal_data : fdsu_real_data;
    wire [4:0]  fdsu_ex1_flags  = (fdsu_state == FDSU_IDLE) ? fds_abnormal_flags : fdsu_real_flags;

    assign fpu_idu_fdsu_full = (fdsu_state == FDSU_BUSY);

    //=========================================================================
    // SECTION 13: THE EU RESULT MUX (D1: one-hot OR-mux, IU.v ALU-section
    // style -- no EX3 register, the four selects are mutually exclusive by
    // construction since IDU issues at most one EU per cycle)
    //=========================================================================
    assign fpu_rtu_ex1_falu_fdata  = idu_fpu_ex1_fadd_sel  ? fadd_ex2_result
                                    : idu_fpu_ex1_fspu_sel  ? fspu_ex1_result
                                    : idu_fpu_ex1_fcnvt_sel ? fcnvt_ex1_result
                                    : idu_fpu_ex1_fmau_sel  ? fmau_ex1_result
                                    : fdsu_cmplt_now        ? fdsu_ex1_result
                                                             : 64'b0;

    assign fpu_rtu_ex1_falu_xdata  = idu_fpu_ex1_fadd_sel  ? fadd_mfvr_data
                                    : idu_fpu_ex1_fspu_sel  ? fspu_mfvr_data
                                    : idu_fpu_ex1_fcnvt_sel && cvt_is_int && cvt_f2i ? f2i_xdata
                                                                                     : 64'b0;

    assign fpu_rtu_ex1_falu_fflags = idu_fpu_ex1_fadd_sel  ? fadd_ex2_flags
                                    : idu_fpu_ex1_fcnvt_sel ? fcnvt_ex1_flags
                                    : idu_fpu_ex1_fmau_sel  ? fmau_ex1_flags
                                    : fdsu_cmplt_now        ? fdsu_ex1_flags
                                                             : 5'b0;

    // fvld: an FP-register-destination result -- add/sub/min/max/sgnj*/f2f/
    // i2f/fma/fdiv/fsqrt, but NOT a compare (that's xvld), NOT fclass
    // (also xvld), NOT fmv.x.w/x.d (xvld) and NOT f2i (xvld -- the float->
    // int convert writes the GPR, not the FRF). FMAU/FDSU never target an
    // integer destination (RISC-V spec), so they only ever contribute to
    // fvld/fdata, never xvld/xdata. FDSU's term is gated on
    // `fdsu_cmplt_now` (FSM state), NOT on `idu_fpu_ex1_fdsu_sel` (the
    // dispatch-select signal), because that signal may still legitimately
    // be asserted on the CMPLT cycle itself (full is deasserted then) for
    // a reason unrelated to gating this writeback -- exactly like DIV's own
    // independence of iu_rtu_div_wb_vld from idu_iu_ex1_div_sel.
    assign fpu_rtu_ex1_falu_fvld   = (idu_fpu_ex1_fadd_sel  && !op_cmp)
                                    || (idu_fpu_ex1_fspu_sel  && !spu_op_class && !spu_op_mv_xf)
                                    || (idu_fpu_ex1_fcnvt_sel && !(cvt_is_int && cvt_f2i))
                                    || idu_fpu_ex1_fmau_sel
                                    || fdsu_cmplt_now;

    // xvld: an integer-register-destination result -- compare, fclass,
    // fmv.x.w/x.d (the mfvr moves), and f2i (fcvt.{w,wu,l,lu}.{s,d}).
    assign fpu_rtu_ex1_falu_xvld   = (idu_fpu_ex1_fadd_sel && op_cmp)
                                    || (idu_fpu_ex1_fspu_sel && (spu_op_class || spu_op_mv_xf))
                                    || (idu_fpu_ex1_fcnvt_sel && cvt_is_int && cvt_f2i);

    // M5 Task 4b: pure pass-through, shared by both the fdata (FRF) and
    // xdata (GPR) answer shapes -- RTU picks which regfile to write from
    // fvld/xvld, not from this tag. FDSU's iter path holds the pipe busy
    // for several cycles after dispatch, so at FDSU_CMPLT the live
    // idu_fpu_ex1_dst0_reg may already name a later instruction -- use the
    // dispatch-time latch there (mirrors IU.v's div_preg_reg fix). The
    // fast (abnormal) path resolves in the same cycle as dispatch, where
    // the live bus is still this op's own, so it falls through untouched.
    assign fpu_rtu_ex1_falu_preg   = (fdsu_cmplt_now && fdsu_state != FDSU_IDLE)
                                    ? fdsu_preg_flop : idu_fpu_ex1_dst0_reg;

endmodule
