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
    input  wire [FUNC_WIDTH-1:0]       idu_fpu_ex1_func,
    input  wire [2:0]                  idu_fpu_ex1_rm,
    input  wire [XLEN-1:0]             idu_fpu_ex1_fsrc0_data,
    input  wire [XLEN-1:0]             idu_fpu_ex1_fsrc1_data,
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
    output wire [GPR_IDX_WIDTH-1:0]    fpu_rtu_ex1_falu_preg
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
    wire [2:0]  e2_rm         = idu_fpu_ex1_rm;
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
    // SECTION 9: FSPU -- SIGN INJECTION AND fclass ONLY (D-TASK3-1: fmv.*
    // dropped, deferred to Task 7)
    //=========================================================================
    wire spu_op_sgnjx = idu_fpu_ex1_func[FUNC_SPU_SGN] && idu_fpu_ex1_func[FUNC_SPU_SGN_X];
    wire spu_op_sgnjn = idu_fpu_ex1_func[FUNC_SPU_SGN] && idu_fpu_ex1_func[FUNC_SPU_SGN_N];
    wire spu_op_sgnj  = idu_fpu_ex1_func[FUNC_SPU_SGN] && idu_fpu_ex1_func[FUNC_SPU_SGN_J];
    wire spu_op_class = idu_fpu_ex1_func[FUNC_CLASS];

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

    // The freg-destination result -- leaves at EX1, D1 (no completion latch).
    wire [63:0] fspu_ex1_result =
          ({64{spu_op_sgnj  &&  f_double}} & spu_sgnj_d)
        | ({64{spu_op_sgnjn &&  f_double}} & spu_sgnjn_d)
        | ({64{spu_op_sgnjx &&  f_double}} & spu_sgnjx_d)
        | ({64{spu_op_sgnj  && !f_double}} & spu_sgnj_s)
        | ({64{spu_op_sgnjn && !f_double}} & spu_sgnjn_s)
        | ({64{spu_op_sgnjx && !f_double}} & spu_sgnjx_s);

    // ... and the mfvr (integer-destination) answer, fclass only in this task.
    wire [63:0] fspu_mfvr_data = ({64{spu_op_class}} & spu_class);

    //=========================================================================
    // SECTION 10: FCNVT EX1 -- FORMAT DECODE AND SOURCE PREPARE, FLOAT-ONLY
    // (D-TASK3-2: int<->float dropped, deferred to Task 7 -- cvt_src_flt/
    // cvt_dest_flt hardwired 1'b1 rather than read from FUNC_CVT_SRC_FLT/
    // _DEST_FLT, which don't exist yet)
    //=========================================================================
    wire cvt_widden = idu_fpu_ex1_func[FUNC_CVT_WIDDEN] && !idu_fpu_ex1_func[FUNC_CVT_NARROW];
    wire cvt_narrow = !idu_fpu_ex1_func[FUNC_CVT_WIDDEN] && idu_fpu_ex1_func[FUNC_CVT_NARROW];
    wire cvt_equal  = !idu_fpu_ex1_func[FUNC_CVT_WIDDEN] && !idu_fpu_ex1_func[FUNC_CVT_NARROW];

    wire cvt_src_l64  = idu_fpu_ex1_func[FUNC_DOUBLE] || (idu_fpu_ex1_func[FUNC_B_SINGLE] && cvt_narrow);
    wire cvt_src_l32  = idu_fpu_ex1_func[FUNC_B_SINGLE] && !cvt_narrow;
    wire cvt_dest_l64 = (cvt_src_l64 && cvt_equal) || (cvt_src_l32 && cvt_widden);

    wire cvt_dest_dbl = cvt_dest_l64;   // cvt_dest_flt hardwired 1 (D-TASK3-2)

    //-- the FLOAT source, classified in its own width -----------------------
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

    //-- FLATTENED EX1->EX2 (D1) -- see SECTION 4's note; f2f-only subset.
    wire        e2_cvt_dest_dbl = cvt_dest_dbl;
    wire        e2_cvt_f_s      = cvt_f_s;
    wire        e2_cvt_f_snan   = cvt_f_snan;
    wire        e2_cvt_f_qnan   = cvt_f_qnan;
    wire        e2_cvt_f_inf    = cvt_f_inf;
    wire        e2_cvt_f_zero   = cvt_f_zero;
    wire [52:0] e2_cvt_f_sig    = cvt_f_sig;
    wire signed [12:0] e2_cvt_f_eunb = cvt_f_eunb;
    wire [2:0]  e2_cvt_rm       = idu_fpu_ex1_rm;

    //=========================================================================
    // SECTION 11: FCNVT -- FLOAT->FLOAT ONLY (D-TASK3-2: SECTIONS 11a/11b's
    // float<->int machinery dropped, deferred to Task 7)
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
    wire [63:0] fcnvt_ex1_result = (e2_cvt_f_snan || e2_cvt_f_qnan) ? f2f_canon
                                 : e2_cvt_f_inf                     ? f2f_inf
                                 : e2_cvt_f_zero                    ? f2f_zero
                                                                    : f2f_pack[PACK_W-1:5];
    wire [4:0]  fcnvt_ex1_flags = (e2_cvt_f_snan)                    ? 5'b10000
                                : (e2_cvt_f_qnan || e2_cvt_f_inf || e2_cvt_f_zero) ? 5'b0
                                                                     : f2f_pack[4:0];

    //=========================================================================
    // SECTION 12: THE EU RESULT MUX (D1: one-hot OR-mux, IU.v ALU-section
    // style -- no EX3 register, the three selects are mutually exclusive by
    // construction since IDU issues at most one EU per cycle)
    //=========================================================================
    assign fpu_rtu_ex1_falu_fdata  = idu_fpu_ex1_fadd_sel  ? fadd_ex2_result
                                    : idu_fpu_ex1_fspu_sel  ? fspu_ex1_result
                                    : idu_fpu_ex1_fcnvt_sel ? fcnvt_ex1_result
                                                             : 64'b0;

    assign fpu_rtu_ex1_falu_xdata  = idu_fpu_ex1_fadd_sel  ? fadd_mfvr_data
                                    : idu_fpu_ex1_fspu_sel  ? fspu_mfvr_data
                                                             : 64'b0;

    assign fpu_rtu_ex1_falu_fflags = idu_fpu_ex1_fadd_sel  ? fadd_ex2_flags
                                    : idu_fpu_ex1_fcnvt_sel ? fcnvt_ex1_flags
                                                             : 5'b0;

    // fvld: an FP-register-destination result -- add/sub/min/max/sgnj*/f2f,
    // but NOT a compare (that's xvld) and NOT fclass (also xvld).
    assign fpu_rtu_ex1_falu_fvld   = (idu_fpu_ex1_fadd_sel  && !op_cmp)
                                    || (idu_fpu_ex1_fspu_sel  && !spu_op_class)
                                    || idu_fpu_ex1_fcnvt_sel;

    // xvld: an integer-register-destination result -- compare or fclass.
    assign fpu_rtu_ex1_falu_xvld   = (idu_fpu_ex1_fadd_sel && op_cmp)
                                    || (idu_fpu_ex1_fspu_sel && spu_op_class);

    // M5 Task 4b: pure pass-through, shared by both the fdata (FRF) and
    // xdata (GPR) answer shapes -- RTU picks which regfile to write from
    // fvld/xvld, not from this tag.
    assign fpu_rtu_ex1_falu_preg   = idu_fpu_ex1_dst0_reg;

    // D1: clk/rst_n are frozen into the port list for FMAU/FDSU (later M5
    // tasks) but this task's FALU body is purely combinational.
    wire _unused_ok = &{1'b0, clk, rst_n};

endmodule
