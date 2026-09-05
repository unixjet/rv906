//=============================================================================
// fpu_tb.cpp -- standalone unit bench for rtl/FPU.v (M5 Task 3)
//=============================================================================
// Verilates FPU.v + rvproc_pkg.sv alone (no IDU/RTU) and drives the frozen
// idu_fpu_ex1_*/rm/fsrc0/fsrc1 ports directly, same tick()-based clocking and
// check()/test_result() bookkeeping pattern as test/m2/unit/iu_tb.cpp.
//
// Scope: Task 3's restricted FALU sub-block only -- FADD (add/sub/compare/
// min-max), FSPU (sign-inject + fclass only, no fmv.*), FCNVT (float-to-
// float only, no int<->float). This is a white-box bench of FPU.v's own
// documented contract (see FPU.v's header D1/D-TASK3-1/2/3), not a RISC-V
// compliance suite -- rv64uf/ud coverage lands once IDU decode routes to
// this module (Task 4) and the ISA swap flips misa.F/D (Task 9).
//
// ORACLE: the host x86-64 FPU's native `double`/`float` arithmetic IS an
// IEEE-754 binary64/binary32 implementation defaulting to round-to-nearest-
// even -- bit-identical to RISC-V's RNE (rm=000), the only mode exercised
// here except one directed RTZ check via fesetround(). Result VALUES are
// checked against plain native arithmetic. The NX (inexact) flag is checked
// via a widening trick: redo the same operation in a wider type (`long
// double` for double operands, `double` for float operands -- comfortably
// exact for the small, hand-picked operand pairs used below) and see
// whether narrowing that exact-for-these-operands result changes it. No
// SoftFloat oracle is vendored for this directed first pass.
//
// Build/run: make -C test/m2/unit fpu && bin/unit/fpu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VFPU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

//-----------------------------------------------------------------------------
// FUNC_* bit positions (rvproc_pkg.sv, M5 Task 3 block).
//-----------------------------------------------------------------------------
static const unsigned FUNC_DOUBLE     = 16;
static const unsigned FUNC_B_SINGLE   = 15;
static const unsigned FUNC_CLASS      = 18;
static const unsigned FUNC_ADD        = 12;
static const unsigned FUNC_SUB        = 11;
static const unsigned FUNC_CMP        = 10;
static const unsigned FUNC_MAX        = 9;
static const unsigned FUNC_MIN        = 8;
static const unsigned FUNC_CMP_FNE    = 4;
static const unsigned FUNC_CMP_FORD   = 3;
static const unsigned FUNC_CMP_LE     = 2;
static const unsigned FUNC_CMP_LT     = 1;
static const unsigned FUNC_CMP_FEQ    = 0;
static const unsigned FUNC_SPU_SGN    = 6;
static const unsigned FUNC_SPU_SGN_X  = 2;
static const unsigned FUNC_SPU_SGN_N  = 1;
static const unsigned FUNC_SPU_SGN_J  = 0;
static const unsigned FUNC_CVT_WIDDEN = 14;
static const unsigned FUNC_CVT_NARROW = 13;

static inline uint32_t bitv(unsigned n) { return 1u << n; }

// RISC-V rounding-mode encoding (the rm field on FP instructions).
static const unsigned RM_RNE = 0;
static const unsigned RM_RTZ = 1;

//-----------------------------------------------------------------------------
// Bit <-> float/double reinterpretation, and single-precision NaN-boxing.
//-----------------------------------------------------------------------------
static uint64_t d2b(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static double   b2d(uint64_t b) { double d; memcpy(&d, &b, 8); return d; }
static uint32_t f2b(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static float    b2f(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
static uint64_t box(uint32_t f32) { return 0xFFFFFFFF00000000ULL | (uint64_t)f32; }

// Special double-precision bit patterns.
static const uint64_t QNAN_D  = 0x7FF8000000000000ULL;
static const uint64_t SNAN_D  = 0x7FF0000000000001ULL;
static const uint64_t PINF_D  = 0x7FF0000000000000ULL;
static const uint64_t NINF_D  = 0xFFF0000000000000ULL;
static const uint64_t PZERO_D = 0x0000000000000000ULL;
static const uint64_t NZERO_D = 0x8000000000000000ULL;

// Special single-precision (unboxed 32-bit) patterns -- box() before driving.
static const uint32_t QNAN_S  = 0x7FC00000u;
static const uint32_t SNAN_S  = 0x7F800001u;
static const uint32_t PINF_S  = 0x7F800000u;
static const uint32_t NINF_S  = 0xFF800000u;
static const uint32_t PZERO_S = 0x00000000u;
static const uint32_t NZERO_S = 0x80000000u;

//-----------------------------------------------------------------------------
// DUT plumbing (mirrors iu_tb.cpp's tick()/reset_dut()/check()/test_result())
//-----------------------------------------------------------------------------
static VFPU *dut = nullptr;
static uint64_t g_cycles = 0;

static void tie_idle_inputs(void) {
    dut->idu_fpu_ex1_fadd_sel   = 0;
    dut->idu_fpu_ex1_fspu_sel   = 0;
    dut->idu_fpu_ex1_fcnvt_sel  = 0;
    dut->idu_fpu_ex1_func       = 0;
    dut->idu_fpu_ex1_rm         = 0;
    dut->idu_fpu_ex1_fsrc0_data = 0;
    dut->idu_fpu_ex1_fsrc1_data = 0;
}

static void tick(void) {
    dut->eval();
    dut->clk = 1;
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
}

static void reset_dut(void) {
    dut->clk   = 0;
    dut->rst_n = 0;
    tie_idle_inputs();
    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0) {
    if (!cond) {
        g_local++;
        if (g_fail < 60)
            printf("    FAIL %-64s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name) {
    printf("[fpu_tb] %-64s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// Exactness oracle: is a+b (or a-b) exactly representable in the operand
// format, for the SPECIFIC operands used below? See file header note.
//-----------------------------------------------------------------------------
static bool add_exact_d(double a, double b, bool sub) {
    long double e = sub ? ((long double)a - (long double)b) : ((long double)a + (long double)b);
    return (long double)(double)e == e;
}
static bool add_exact_s(float a, float b, bool sub) {
    double e = sub ? ((double)a - (double)b) : ((double)a + (double)b);
    return (double)(float)e == e;
}

//-----------------------------------------------------------------------------
// Dispatch helpers -- purely combinational, EX1-only (FPU.v D1): set inputs,
// eval(), read the result the same cycle, then tick() once.
//-----------------------------------------------------------------------------
struct FpuResult {
    uint64_t fdata, xdata;
    unsigned fflags;
    bool     fvld, xvld;
};

static FpuResult read_result(void) {
    FpuResult r;
    r.fdata  = dut->fpu_rtu_ex1_falu_fdata;
    r.xdata  = dut->fpu_rtu_ex1_falu_xdata;
    r.fflags = dut->fpu_rtu_ex1_falu_fflags;
    r.fvld   = dut->fpu_rtu_ex1_falu_fvld != 0;
    r.xvld   = dut->fpu_rtu_ex1_falu_xvld != 0;
    return r;
}

static FpuResult fadd_op(uint32_t func, unsigned rm, uint64_t fsrc0, uint64_t fsrc1) {
    tie_idle_inputs();
    dut->idu_fpu_ex1_fadd_sel   = 1;
    dut->idu_fpu_ex1_func       = func;
    dut->idu_fpu_ex1_rm         = rm;
    dut->idu_fpu_ex1_fsrc0_data = fsrc0;
    dut->idu_fpu_ex1_fsrc1_data = fsrc1;
    dut->eval();
    FpuResult r = read_result();
    tick();
    return r;
}

static FpuResult fspu_op(uint32_t func, uint64_t fsrc0, uint64_t fsrc1) {
    tie_idle_inputs();
    dut->idu_fpu_ex1_fspu_sel   = 1;
    dut->idu_fpu_ex1_func       = func;
    dut->idu_fpu_ex1_fsrc0_data = fsrc0;
    dut->idu_fpu_ex1_fsrc1_data = fsrc1;
    dut->eval();
    FpuResult r = read_result();
    tick();
    return r;
}

static FpuResult fcnvt_op(uint32_t func, unsigned rm, uint64_t fsrc0) {
    tie_idle_inputs();
    dut->idu_fpu_ex1_fcnvt_sel  = 1;
    dut->idu_fpu_ex1_func       = func;
    dut->idu_fpu_ex1_rm         = rm;
    dut->idu_fpu_ex1_fsrc0_data = fsrc0;
    dut->eval();
    FpuResult r = read_result();
    tick();
    return r;
}

//-----------------------------------------------------------------------------
// T1: FADD.D add/sub -- value + NX flag, exact and bypass paths.
//-----------------------------------------------------------------------------
static void test_fadd_d_basic(void) {
    uint32_t f_add = bitv(FUNC_ADD) | bitv(FUNC_DOUBLE);
    uint32_t f_sub = bitv(FUNC_SUB) | bitv(FUNC_DOUBLE);

    double a = 1.5, b = 2.25;
    FpuResult r = fadd_op(f_add, RM_RNE, d2b(a), d2b(b));
    check(r.fdata == d2b(a + b), "FADD.D 1.5+2.25 value", r.fdata, d2b(a + b));
    check(r.fflags == 0, "FADD.D 1.5+2.25 exact, flags clean", r.fflags, 0);
    check(r.fvld && !r.xvld, "FADD.D add: fvld/xvld routing");

    a = 5.0; b = 5.0;
    r = fadd_op(f_sub, RM_RNE, d2b(a), d2b(b));
    check(r.fdata == d2b(0.0), "FADD.D 5.0-5.0 == +0", r.fdata, d2b(0.0));
    check(r.fflags == 0, "FADD.D 5.0-5.0 flags clean", r.fflags, 0);

    a = 1.0; b = ldexp(1.0, -60);
    r = fadd_op(f_add, RM_RNE, d2b(a), d2b(b));
    double native = a + b;
    check(r.fdata == d2b(native), "FADD.D 1.0+2^-60 value (bypass path)", r.fdata, d2b(native));
    check((r.fflags & 0x1) == (add_exact_d(a, b, false) ? 0u : 1u),
          "FADD.D 1.0+2^-60 sets NX", r.fflags, 1);

    a = -3.5; b = 1.25;
    r = fadd_op(f_add, RM_RNE, d2b(a), d2b(b));
    check(r.fdata == d2b(a + b), "FADD.D -3.5+1.25 value", r.fdata, d2b(a + b));

    // qNaN in -> canonical qNaN out, NV=0; sNaN in -> NV=1; inf-inf -> NaN, NV=1
    r = fadd_op(f_add, RM_RNE, QNAN_D, d2b(1.0));
    check(r.fdata == QNAN_D && r.fflags == 0, "FADD.D qNaN+1.0 -> canonical qNaN, no NV");
    r = fadd_op(f_add, RM_RNE, SNAN_D, d2b(1.0));
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FADD.D sNaN+1.0 -> qNaN, NV set");
    r = fadd_op(f_sub, RM_RNE, PINF_D, PINF_D);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FADD.D inf-inf -> qNaN, NV set");
    r = fadd_op(f_add, RM_RNE, PINF_D, d2b(1.0));
    check(r.fdata == PINF_D && r.fflags == 0, "FADD.D inf+1.0 -> inf, no flags");

    test_result("T1 FADD.D add/sub: value + NX flag, exact/bypass paths, specials");
}

//-----------------------------------------------------------------------------
// T2: FADD.S add/sub -- boxed values + NaN-box check.
//-----------------------------------------------------------------------------
static void test_fadd_s_basic(void) {
    uint32_t f_add = bitv(FUNC_ADD) | bitv(FUNC_B_SINGLE);

    float a = 1.5f, b = 2.25f;
    FpuResult r = fadd_op(f_add, RM_RNE, box(f2b(a)), box(f2b(b)));
    check(r.fdata == box(f2b(a + b)), "FADD.S 1.5+2.25 value (boxed)", r.fdata, box(f2b(a + b)));
    check(r.fflags == 0, "FADD.S 1.5+2.25 exact, flags clean", r.fflags, 0);

    a = 1.0f; b = ldexpf(1.0f, -30);
    r = fadd_op(f_add, RM_RNE, box(f2b(a)), box(f2b(b)));
    float native = a + b;
    check(r.fdata == box(f2b(native)), "FADD.S 1.0+2^-30 value (bypass path)", r.fdata, box(f2b(native)));
    check((r.fflags & 0x1) == (add_exact_s(a, b, false) ? 0u : 1u), "FADD.S 1.0+2^-30 sets NX");

    // un-boxed garbage upper bits -> treated as canonical qNaN input
    uint64_t garbage = 0x1234567800000000ULL | (uint64_t)f2b(3.0f);
    r = fadd_op(f_add, RM_RNE, garbage, box(f2b(1.0f)));
    check(r.fdata == box(QNAN_S), "FADD.S unboxed src0 -> canonical qNaN in", r.fdata, box(QNAN_S));

    test_result("T2 FADD.S add: value + NX flag + NaN-box check");
}

//-----------------------------------------------------------------------------
// T3: FADD.D compare predicates (feq/flt/fle/ford/fne).
//-----------------------------------------------------------------------------
static void test_fadd_cmp(void) {
    uint32_t f_feq  = bitv(FUNC_CMP) | bitv(FUNC_CMP_FEQ)  | bitv(FUNC_DOUBLE);
    uint32_t f_flt  = bitv(FUNC_CMP) | bitv(FUNC_CMP_LT)   | bitv(FUNC_DOUBLE);
    uint32_t f_fle  = bitv(FUNC_CMP) | bitv(FUNC_CMP_LE)   | bitv(FUNC_DOUBLE);
    uint32_t f_ford = bitv(FUNC_CMP) | bitv(FUNC_CMP_FORD) | bitv(FUNC_DOUBLE);
    uint32_t f_fne  = bitv(FUNC_CMP) | bitv(FUNC_CMP_FNE)  | bitv(FUNC_DOUBLE);

    FpuResult r = fadd_op(f_feq, RM_RNE, d2b(1.0), d2b(1.0));
    check(r.xdata == 1 && r.xvld && !r.fvld, "FEQ.D 1.0==1.0 -> true, xvld routing", r.xdata, 1);

    r = fadd_op(f_flt, RM_RNE, d2b(1.0), d2b(2.0));
    check(r.xdata == 1, "FLT.D 1.0<2.0 -> true", r.xdata, 1);
    r = fadd_op(f_flt, RM_RNE, d2b(2.0), d2b(1.0));
    check(r.xdata == 0, "FLT.D 2.0<1.0 -> false", r.xdata, 0);

    r = fadd_op(f_fle, RM_RNE, d2b(1.0), d2b(1.0));
    check(r.xdata == 1, "FLE.D 1.0<=1.0 -> true", r.xdata, 1);

    r = fadd_op(f_feq, RM_RNE, PZERO_D, NZERO_D);
    check(r.xdata == 1, "FEQ.D +0==-0 -> true", r.xdata, 1);

    r = fadd_op(f_fne, RM_RNE, d2b(1.0), d2b(2.0));
    check(r.xdata == 1, "FNE.D 1.0!=2.0 -> true", r.xdata, 1);
    r = fadd_op(f_fne, RM_RNE, d2b(1.0), d2b(1.0));
    check(r.xdata == 0, "FNE.D 1.0!=1.0 -> false", r.xdata, 0);

    // NaN handling: quiet NaN never signals for feq; flt/fle DO signal on qNaN;
    // sNaN always signals, even for feq; ford is false whenever either op is NaN.
    r = fadd_op(f_feq, RM_RNE, QNAN_D, d2b(1.0));
    check(r.xdata == 0 && r.fflags == 0, "FEQ.D qNaN==1.0 -> false, no NV");
    r = fadd_op(f_flt, RM_RNE, QNAN_D, d2b(1.0));
    check(r.xdata == 0 && (r.fflags & 0x10), "FLT.D qNaN<1.0 -> false, NV set");
    r = fadd_op(f_fle, RM_RNE, QNAN_D, d2b(1.0));
    check(r.xdata == 0 && (r.fflags & 0x10), "FLE.D qNaN<=1.0 -> false, NV set");
    r = fadd_op(f_ford, RM_RNE, QNAN_D, d2b(1.0));
    check(r.xdata == 0, "FORD.D qNaN,1.0 -> false (one operand is NaN)", r.xdata, 0);
    r = fadd_op(f_feq, RM_RNE, SNAN_D, d2b(1.0));
    check(r.xdata == 0 && (r.fflags & 0x10), "FEQ.D sNaN==1.0 -> false, NV set (sNaN always signals)");

    test_result("T3 FADD.D compare predicates: feq/flt/fle/ford/fne + NaN rules");
}

//-----------------------------------------------------------------------------
// T4: FADD.D max/min -- ordering, NaN exclusion, signed zero.
//-----------------------------------------------------------------------------
static void test_fadd_maxmin(void) {
    uint32_t f_max = bitv(FUNC_MAX) | bitv(FUNC_DOUBLE);
    uint32_t f_min = bitv(FUNC_MIN) | bitv(FUNC_DOUBLE);

    FpuResult r = fadd_op(f_max, RM_RNE, d2b(1.0), d2b(2.0));
    check(r.fdata == d2b(2.0) && r.fvld && !r.xvld, "FMAX.D max(1,2)==2, fvld routing", r.fdata, d2b(2.0));

    r = fadd_op(f_min, RM_RNE, d2b(1.0), d2b(2.0));
    check(r.fdata == d2b(1.0), "FMIN.D min(1,2)==1", r.fdata, d2b(1.0));

    r = fadd_op(f_max, RM_RNE, QNAN_D, d2b(5.0));
    check(r.fdata == d2b(5.0) && (r.fflags & 0x10) == 0,
          "FMAX.D max(qNaN,5)==5, NaN excluded, no NV", r.fdata, d2b(5.0));
    r = fadd_op(f_max, RM_RNE, SNAN_D, d2b(5.0));
    check((r.fflags & 0x10) != 0, "FMAX.D max(sNaN,5) sets NV");

    r = fadd_op(f_max, RM_RNE, QNAN_D, QNAN_D);
    check(r.fdata == QNAN_D, "FMAX.D max(qNaN,qNaN)==canonical qNaN", r.fdata, QNAN_D);

    r = fadd_op(f_max, RM_RNE, PZERO_D, NZERO_D);
    check(r.fdata == PZERO_D, "FMAX.D max(+0,-0)==+0", r.fdata, PZERO_D);
    r = fadd_op(f_min, RM_RNE, PZERO_D, NZERO_D);
    check(r.fdata == NZERO_D, "FMIN.D min(+0,-0)==-0", r.fdata, NZERO_D);

    test_result("T4 FADD.D max/min: ordering, NaN exclusion, signed zero");
}

//-----------------------------------------------------------------------------
// T5: FSPU sign-inject -- sgnj/sgnjn/sgnjx, double + single, NaN-box check.
//-----------------------------------------------------------------------------
static void test_fspu_sgnj(void) {
    uint32_t f_sgnj_d  = bitv(FUNC_SPU_SGN) | bitv(FUNC_SPU_SGN_J) | bitv(FUNC_DOUBLE);
    uint32_t f_sgnjn_d = bitv(FUNC_SPU_SGN) | bitv(FUNC_SPU_SGN_N) | bitv(FUNC_DOUBLE);
    uint32_t f_sgnjx_d = bitv(FUNC_SPU_SGN) | bitv(FUNC_SPU_SGN_X) | bitv(FUNC_DOUBLE);
    uint32_t f_sgnj_s  = bitv(FUNC_SPU_SGN) | bitv(FUNC_SPU_SGN_J) | bitv(FUNC_B_SINGLE);

    FpuResult r = fspu_op(f_sgnj_d, d2b(3.0), d2b(-1.0));
    check(r.fdata == d2b(-3.0) && r.fvld && !r.xvld, "FSGNJ.D (3.0,-1.0)==-3.0", r.fdata, d2b(-3.0));

    r = fspu_op(f_sgnjn_d, d2b(3.0), d2b(-1.0));
    check(r.fdata == d2b(3.0), "FSGNJN.D (3.0,-1.0)==3.0 (opposite sign)", r.fdata, d2b(3.0));

    r = fspu_op(f_sgnjx_d, d2b(3.0), d2b(-1.0));
    check(r.fdata == d2b(-3.0), "FSGNJX.D (3.0,-1.0)==-3.0 (xor: diff signs->neg)", r.fdata, d2b(-3.0));
    r = fspu_op(f_sgnjx_d, d2b(-3.0), d2b(-1.0));
    check(r.fdata == d2b(3.0), "FSGNJX.D (-3.0,-1.0)==3.0 (xor: same signs->pos)", r.fdata, d2b(3.0));

    r = fspu_op(f_sgnj_s, box(f2b(3.0f)), box(f2b(-1.0f)));
    check(r.fdata == box(f2b(-3.0f)), "FSGNJ.S (3.0,-1.0)==-3.0 (boxed)", r.fdata, box(f2b(-3.0f)));

    // unboxed src0 -> canonical qNaN substituted before sign-inject
    uint64_t garbage = 0x0000000000000000ULL | (uint64_t)f2b(3.0f);
    r = fspu_op(f_sgnj_s, garbage, box(f2b(-1.0f)));
    check(r.fdata == box(0xFFC00000u), "FSGNJ.S unboxed src0 -> qNaN with injected sign",
          r.fdata, box(0xFFC00000u));

    test_result("T5 FSPU sign-inject: sgnj/sgnjn/sgnjx, double+single, NaN-box check");
}

//-----------------------------------------------------------------------------
// T6: FSPU fclass -- all 10 categories, double + single.
//-----------------------------------------------------------------------------
struct ClassCase { uint64_t bits; uint32_t expect_mask; const char *name; };

static void test_fspu_fclass(void) {
    uint32_t f_class_d = bitv(FUNC_CLASS) | bitv(FUNC_DOUBLE);
    uint32_t f_class_s = bitv(FUNC_CLASS) | bitv(FUNC_B_SINGLE);

    ClassCase cases_d[] = {
        { NINF_D,                 1u << 0, "class.d -inf" },
        { d2b(-1.0),              1u << 1, "class.d -normal" },
        { 0x800FFFFFFFFFFFFFULL,  1u << 2, "class.d -subnormal" },
        { NZERO_D,                1u << 3, "class.d -0" },
        { PZERO_D,                1u << 4, "class.d +0" },
        { 0x000FFFFFFFFFFFFFULL,  1u << 5, "class.d +subnormal" },
        { d2b(1.0),               1u << 6, "class.d +normal" },
        { PINF_D,                 1u << 7, "class.d +inf" },
        { SNAN_D,                 1u << 8, "class.d sNaN" },
        { QNAN_D,                 1u << 9, "class.d qNaN" },
    };
    for (auto &c : cases_d) {
        FpuResult r = fspu_op(f_class_d, c.bits, 0);
        check(r.xdata == c.expect_mask && r.xvld && !r.fvld, c.name, r.xdata, c.expect_mask);
    }

    ClassCase cases_s[] = {
        { box(NINF_S),             1u << 0, "class.s -inf" },
        { box(f2b(-1.0f)),         1u << 1, "class.s -normal" },
        { box(0x807FFFFFu),        1u << 2, "class.s -subnormal" },
        { box(NZERO_S),            1u << 3, "class.s -0" },
        { box(PZERO_S),            1u << 4, "class.s +0" },
        { box(0x007FFFFFu),        1u << 5, "class.s +subnormal" },
        { box(f2b(1.0f)),          1u << 6, "class.s +normal" },
        { box(PINF_S),             1u << 7, "class.s +inf" },
        { box(SNAN_S),             1u << 8, "class.s sNaN" },
        { box(QNAN_S),             1u << 9, "class.s qNaN" },
    };
    for (auto &c : cases_s) {
        FpuResult r = fspu_op(f_class_s, c.bits, 0);
        check(r.xdata == c.expect_mask, c.name, r.xdata, c.expect_mask);
    }

    // unboxed single input classifies as qNaN only (broken box)
    FpuResult r = fspu_op(f_class_s, 0x1234567800000000ULL | (uint64_t)f2b(1.0f), 0);
    check(r.xdata == (1u << 9), "class.s unboxed input classifies as qNaN only", r.xdata, 1u << 9);

    test_result("T6 FSPU fclass: all 10 categories, double + single, NaN-box check");
}

//-----------------------------------------------------------------------------
// T7: FCNVT float-to-float -- fcvt.d.s (widen, exact) + fcvt.s.d (narrow,
// rounding/flags).
//-----------------------------------------------------------------------------
static void test_fcnvt_f2f(void) {
    uint32_t f_widen  = bitv(FUNC_CVT_WIDDEN) | bitv(FUNC_B_SINGLE); // fcvt.d.s
    uint32_t f_narrow = bitv(FUNC_CVT_NARROW) | bitv(FUNC_DOUBLE);   // fcvt.s.d

    float sf = 1.5f;
    FpuResult r = fcnvt_op(f_widen, RM_RNE, box(f2b(sf)));
    check(r.fdata == d2b((double)sf) && r.fvld && !r.xvld,
          "FCVT.D.S 1.5f widen exact", r.fdata, d2b((double)sf));
    check(r.fflags == 0, "FCVT.D.S widen is always exact, no flags", r.fflags, 0);

    r = fcnvt_op(f_widen, RM_RNE, box(PINF_S));
    check(r.fdata == PINF_D, "FCVT.D.S +inf -> +inf", r.fdata, PINF_D);
    r = fcnvt_op(f_widen, RM_RNE, box(QNAN_S));
    check(r.fdata == QNAN_D, "FCVT.D.S qNaN -> canonical qNaN(double)", r.fdata, QNAN_D);
    r = fcnvt_op(f_widen, RM_RNE, box(SNAN_S));
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FCVT.D.S sNaN -> canonical qNaN, NV set");

    double dv = 1.5;
    r = fcnvt_op(f_narrow, RM_RNE, d2b(dv));
    check(r.fdata == box(f2b((float)dv)), "FCVT.S.D 1.5 narrow exact", r.fdata, box(f2b((float)dv)));
    check(r.fflags == 0, "FCVT.S.D 1.5 exact, no flags", r.fflags, 0);

    volatile double one = 1.0, three = 3.0;
    double third = one / three;
    r = fcnvt_op(f_narrow, RM_RNE, d2b(third));
    float native = (float)third;
    check(r.fdata == box(f2b(native)), "FCVT.S.D 1/3 narrow rounds correctly", r.fdata, box(f2b(native)));
    check((r.fflags & 0x1) == 1, "FCVT.S.D 1/3 sets NX");

    double big = 1.0e300;
    r = fcnvt_op(f_narrow, RM_RNE, d2b(big));
    check((r.fflags & 0x4) != 0, "FCVT.S.D 1e300 overflow sets OF");
    check((r.fdata & 0xFFFFFFFFULL) == PINF_S,
          "FCVT.S.D 1e300 rounds to +inf (RNE overflow)", r.fdata & 0xFFFFFFFFULL, PINF_S);

    // RTZ rounding-mode plumbing: narrow 1/3 under RTZ must truncate toward zero.
    // Computed by nudging the (compiler-rounding-mode-dependent) RNE cast back
    // toward zero by one ULP if it overshot -- NOT via fesetround(), since a
    // narrowing cast with no -frounding-math is free to ignore the dynamic
    // rounding mode (observed: clang/glibc here fold it to RNE regardless).
    float native_rtz = (float)third;
    if ((double)native_rtz > third) native_rtz = std::nextafterf(native_rtz, 0.0f);
    r = fcnvt_op(f_narrow, RM_RTZ, d2b(third));
    check(r.fdata == box(f2b(native_rtz)), "FCVT.S.D 1/3 under RTZ truncates",
          r.fdata, box(f2b(native_rtz)));

    test_result("T7 FCNVT f2f: fcvt.d.s widen (exact) + fcvt.s.d narrow (rounding/flags)");
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VFPU;
    reset_dut();

    test_fadd_d_basic();
    test_fadd_s_basic();
    test_fadd_cmp();
    test_fadd_maxmin();
    test_fspu_sgnj();
    test_fspu_fclass();
    test_fcnvt_f2f();

    printf("[fpu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
