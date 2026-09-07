//=============================================================================
// fpu_tb.cpp -- standalone unit bench for rtl/FPU.v (M5 Tasks 3 + 5)
//=============================================================================
// Verilates FPU.v + rvproc_pkg.sv alone (no IDU/RTU) and drives the frozen
// idu_fpu_ex1_*/rm/fsrc0/fsrc1/fsrc2 ports directly, same tick()-based
// clocking and check()/test_result() bookkeeping pattern as
// test/m2/unit/iu_tb.cpp.
//
// Scope: Task 3's restricted FALU sub-block -- FADD (add/sub/compare/
// min-max), FSPU (sign-inject + fclass only, no fmv.*), FCNVT (float-to-
// float only, no int<->float) -- plus Task 5's FMAU (fmul/fmadd/fmsub/
// fnmadd/fnmsub, single-cycle EX1 handshake per M5 design doc D1). This is
// a white-box bench of FPU.v's own documented contract (see FPU.v's header
// D1/D-TASK3-1/2/3), not a RISC-V compliance suite -- IDU decode already
// routes to this module and the ISA swap has flipped misa.F/D (Task 9), so
// full rv64uf/ud coverage lands with the test/m5 infra (Task 10/11).
//
// ORACLE: the host x86-64 FPU's native `double`/`float` arithmetic IS an
// IEEE-754 binary64/binary32 implementation defaulting to round-to-nearest-
// even -- bit-identical to RISC-V's RNE (rm=000), the only mode exercised
// here except one directed RTZ check via fesetround(). Result VALUES are
// checked against plain native arithmetic (FADD/FCNVT/plain FMUL) or a
// single-rounding a*b+/-c oracle shaped like std::fma() (fma_exact_d/s(),
// FMAU's fused forms). The NX (inexact) flag is checked via a widening
// trick: redo the same operation in a wider type (`long double` for double
// operands, `double` for float operands -- comfortably exact for the
// small, hand-picked operand pairs used below) and see whether narrowing
// that exact-for-these-operands result changes it. No SoftFloat oracle is
// vendored for this directed first pass.
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
// FUNC_CVT_* int<->float bits (rvproc_pkg.sv M5 Task 7 block). The 0/1/2/4
// values REUSE the FUNC_CMP_FEQ/LT/LE/FNE positions and FUNC_SPU_MV/_XF
// reuse FUNC_MAU_NEG/FUNC_CMP_FORD by this file's established bit-reuse
// convention -- FALU-compare and FCNVT/MV dispatches are mutually exclusive,
// so the reuse is legal.
static const unsigned FUNC_CVT_INT      = 4;  // trigger: fcvt int<->float family
static const unsigned FUNC_CVT_F2I      = 2;  // direction: 1=fp->int, 0=int->fp
static const unsigned FUNC_CVT_UNSIGNED = 1;  // 1=wu/lu, 0=w/l
static const unsigned FUNC_CVT_WIDE64   = 0;  // 1=l/lu, 0=w/wu
static const unsigned FUNC_SPU_MV       = 17; // trigger: fmv.{x.w,w.x,x.d,d.x}
static const unsigned FUNC_SPU_MV_XF    = 3;  // 1=fp->int (fmv.x.*), 0=int->fp
static const unsigned FUNC_MAU_FUSED  = 5;
static const unsigned FUNC_MAU_SUB    = 7;
static const unsigned FUNC_MAU_NEG    = 17;
static const unsigned FUNC_FDSU_DIV   = 5;
static const unsigned FUNC_FDSU_SQRT  = 7;

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
    dut->idu_fpu_ex1_fmau_sel   = 0;
    dut->idu_fpu_ex1_fdsu_sel   = 0;
    dut->idu_fpu_ex1_func       = 0;
    dut->idu_fpu_ex1_rm         = 0;
    dut->idu_fpu_ex1_fsrc0_data = 0;
    dut->idu_fpu_ex1_fsrc1_data = 0;
    dut->idu_fpu_ex1_fsrc2_data = 0;
    dut->idu_fpu_ex1_dst0_reg   = 0;
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

// Same idea for FMAU (a*b +/- c, single rounding): exact iff the a*b
// product AND the final sum both land exactly, for the specific
// hand-picked small operands used below (products stay well within
// long double's 64-bit mantissa for the double rows, and double's 53-bit
// mantissa comfortably covers the float rows' <=48-bit products).
static bool fma_exact_d(double a, double b, double c) {
    long double e = (long double)a * (long double)b + (long double)c;
    return (long double)(double)e == e;
}
static bool fma_exact_s(float a, float b, float c) {
    double e = (double)a * (double)b + (double)c;
    return (double)(float)e == e;
}

// Same widening-trick idea for FDIV/FSQRT (host division/sqrt is also
// IEEE-754 RNE, so it's the value oracle directly -- these only answer
// "is the RNE result exact for these specific operands").
static bool div_exact_d(double a, double b) {
    long double e = (long double)a / (long double)b;
    return (long double)(double)e == e;
}
static bool div_exact_s(float a, float b) {
    double e = (double)a / (double)b;
    return (double)(float)e == e;
}
static bool sqrt_exact_d(double a) {
    long double e = sqrtl((long double)a);
    return (long double)(double)e == e;
}
static bool sqrt_exact_s(float a) {
    double e = sqrt((double)a);
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

// fcnvt dispatch: f2f/f2i carry the FP source on fsrc0 (NaN-boxed for
// single); i2f carries the GPR value there (IDU's dis_gpr_fsrc0 mux feeds
// the same port). dst0_reg is the integer-destination preg for the f2i and
// fmv.x.* rows (checked via fpu_rtu_ex1_falu_preg).
static FpuResult fcnvt_op(uint32_t func, unsigned rm, uint64_t fsrc0,
                          uint32_t dst0_reg = 0) {
    tie_idle_inputs();
    dut->idu_fpu_ex1_fcnvt_sel  = 1;
    dut->idu_fpu_ex1_func       = func;
    dut->idu_fpu_ex1_rm         = rm;
    dut->idu_fpu_ex1_fsrc0_data = fsrc0;
    dut->idu_fpu_ex1_dst0_reg   = dst0_reg;
    dut->eval();
    FpuResult r = read_result();
    tick();
    return r;
}

static FpuResult fmau_op(uint32_t func, unsigned rm, uint64_t fsrc0, uint64_t fsrc1, uint64_t fsrc2) {
    tie_idle_inputs();
    dut->idu_fpu_ex1_fmau_sel   = 1;
    dut->idu_fpu_ex1_func       = func;
    dut->idu_fpu_ex1_rm         = rm;
    dut->idu_fpu_ex1_fsrc0_data = fsrc0;
    dut->idu_fpu_ex1_fsrc1_data = fsrc1;
    dut->idu_fpu_ex1_fsrc2_data = fsrc2;
    dut->eval();
    FpuResult r = read_result();
    tick();
    return r;
}

//-----------------------------------------------------------------------------
// FDSU dispatch helper -- multi-cycle (M5 Task 6 busy/stall FSM, mirrors
// IU.v's div_op in iu_tb.cpp). Unlike DIV, FDSU has no writeback-grant wait
// state (RTU.v:961-963/974: FPU's wbf0 writeback is unconditional), so
// `fpu_rtu_ex1_falu_fvld` alone directly encodes true completion (both the
// same-cycle abnormal fast path and the multi-cycle real-compute path) --
// no two-stage loop like DIV's cmplt-then-wb_vld is needed. After the loop
// exits, `idu_fpu_ex1_fdsu_sel` is deasserted and one more tick() is taken
// before returning, so the FSM's CMPLT->IDLE cycle doesn't spuriously
// re-trigger a fresh dispatch off the still-asserted select (no IDU exists
// in this bench to naturally drop it).
//-----------------------------------------------------------------------------
static FpuResult fdsu_op(uint32_t func, unsigned rm, uint64_t fsrc0, uint64_t fsrc1,
                          uint32_t dst0_reg = 0) {
    tie_idle_inputs();
    dut->idu_fpu_ex1_fdsu_sel   = 1;
    dut->idu_fpu_ex1_func       = func;
    dut->idu_fpu_ex1_rm         = rm;
    dut->idu_fpu_ex1_fsrc0_data = fsrc0;
    dut->idu_fpu_ex1_fsrc1_data = fsrc1;
    dut->idu_fpu_ex1_dst0_reg   = dst0_reg;
    dut->eval();

    while (!dut->fpu_rtu_ex1_falu_fvld) {
        tick();
        dut->eval();
    }
    FpuResult r = read_result();

    dut->idu_fpu_ex1_fdsu_sel = 0;
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

//-----------------------------------------------------------------------------
// T8: idu_fpu_ex1_dst0_reg -> fpu_rtu_ex1_falu_preg pass-through (M5 Task
// 4b, FPU.v:766: `assign fpu_rtu_ex1_falu_preg = idu_fpu_ex1_dst0_reg;`).
// A plain combinational wire, unconditional on which EU-internal select
// (fadd/fspu/fcnvt) drove the result -- checked once per select to confirm
// none of them re-route or gate it.
//-----------------------------------------------------------------------------
static void test_dst0_reg_preg_passthrough(void) {
    uint32_t f_add = bitv(FUNC_ADD) | bitv(FUNC_DOUBLE);
    tie_idle_inputs();
    dut->idu_fpu_ex1_fadd_sel   = 1;
    dut->idu_fpu_ex1_func       = f_add;
    dut->idu_fpu_ex1_rm         = RM_RNE;
    dut->idu_fpu_ex1_fsrc0_data = d2b(1.0);
    dut->idu_fpu_ex1_fsrc1_data = d2b(2.0);
    dut->idu_fpu_ex1_dst0_reg   = 17;
    dut->eval();
    check(dut->fpu_rtu_ex1_falu_preg == 17, "dst0_reg passthrough: fadd_sel",
          dut->fpu_rtu_ex1_falu_preg, 17);
    tick();

    uint32_t f_sgnj = bitv(FUNC_SPU_SGN) | bitv(FUNC_SPU_SGN_J);
    tie_idle_inputs();
    dut->idu_fpu_ex1_fspu_sel   = 1;
    dut->idu_fpu_ex1_func       = f_sgnj;
    dut->idu_fpu_ex1_fsrc0_data = d2b(1.0);
    dut->idu_fpu_ex1_fsrc1_data = d2b(-1.0);
    dut->idu_fpu_ex1_dst0_reg   = 22;
    dut->eval();
    check(dut->fpu_rtu_ex1_falu_preg == 22, "dst0_reg passthrough: fspu_sel",
          dut->fpu_rtu_ex1_falu_preg, 22);
    tick();

    uint32_t f_widen = bitv(FUNC_CVT_WIDDEN) | bitv(FUNC_B_SINGLE);
    tie_idle_inputs();
    dut->idu_fpu_ex1_fcnvt_sel  = 1;
    dut->idu_fpu_ex1_func       = f_widen;
    dut->idu_fpu_ex1_rm         = RM_RNE;
    dut->idu_fpu_ex1_fsrc0_data = f2b(1.5f);
    dut->idu_fpu_ex1_dst0_reg   = 31;
    dut->eval();
    check(dut->fpu_rtu_ex1_falu_preg == 31, "dst0_reg passthrough: fcnvt_sel",
          dut->fpu_rtu_ex1_falu_preg, 31);
    tick();

    test_result("T8 idu_fpu_ex1_dst0_reg -> fpu_rtu_ex1_falu_preg pass-through (M5 Task 4b)");
}

//-----------------------------------------------------------------------------
// T9: FMAU.D fmadd/fmsub/fnmadd/fnmsub -- value + NX flag, via the host's
// std::fma()-shaped fma_exact_d() oracle (single-rounding a*b+/-c).
//-----------------------------------------------------------------------------
static void test_fmau_d_basic(void) {
    uint32_t f_fmadd  = bitv(FUNC_MAU_FUSED) | bitv(FUNC_DOUBLE);
    uint32_t f_fmsub  = bitv(FUNC_MAU_FUSED) | bitv(FUNC_MAU_SUB) | bitv(FUNC_DOUBLE);
    uint32_t f_fnmsub = bitv(FUNC_MAU_FUSED) | bitv(FUNC_MAU_SUB) | bitv(FUNC_MAU_NEG) | bitv(FUNC_DOUBLE);
    uint32_t f_fnmadd = bitv(FUNC_MAU_FUSED) | bitv(FUNC_MAU_NEG) | bitv(FUNC_DOUBLE);

    double a = 1.5, b = 2.0, c = 1.0;
    FpuResult r = fmau_op(f_fmadd, RM_RNE, d2b(a), d2b(b), d2b(c));
    check(r.fdata == d2b(a * b + c) && r.fvld && !r.xvld,
          "FMADD.D 1.5*2.0+1.0 == 4.0, fvld routing", r.fdata, d2b(a * b + c));
    check(r.fflags == 0, "FMADD.D 1.5*2.0+1.0 exact, flags clean", r.fflags, 0);

    r = fmau_op(f_fmsub, RM_RNE, d2b(a), d2b(b), d2b(c));
    check(r.fdata == d2b(a * b - c), "FMSUB.D 1.5*2.0-1.0 == 2.0", r.fdata, d2b(a * b - c));

    r = fmau_op(f_fnmsub, RM_RNE, d2b(a), d2b(b), d2b(c));
    check(r.fdata == d2b(-(a * b) + c), "FNMSUB.D -(1.5*2.0)+1.0 == -2.0", r.fdata, d2b(-(a * b) + c));

    r = fmau_op(f_fnmadd, RM_RNE, d2b(a), d2b(b), d2b(c));
    check(r.fdata == d2b(-(a * b) - c), "FNMADD.D -(1.5*2.0)-1.0 == -4.0", r.fdata, d2b(-(a * b) - c));

    // Inexact addend well below the product's ulp -> NX set, value rounds
    // to the product (mirrors FADD.D's bypass-path test).
    a = 1.0; b = 1.0; c = ldexp(1.0, -60);
    r = fmau_op(f_fmadd, RM_RNE, d2b(a), d2b(b), d2b(c));
    check((r.fflags & 0x1) == (fma_exact_d(a, b, c) ? 0u : 1u),
          "FMADD.D 1.0*1.0+2^-60 sets NX", r.fflags, 1);

    // qNaN in -> canonical qNaN out, NV=0; sNaN in -> NV=1.
    r = fmau_op(f_fmadd, RM_RNE, QNAN_D, d2b(1.0), d2b(1.0));
    check(r.fdata == QNAN_D && r.fflags == 0, "FMADD.D qNaN*1.0+1.0 -> canonical qNaN, no NV");
    r = fmau_op(f_fmadd, RM_RNE, SNAN_D, d2b(1.0), d2b(1.0));
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FMADD.D sNaN*1.0+1.0 -> qNaN, NV set");

    // 0*inf -> invalid (independent of the addend).
    r = fmau_op(f_fmadd, RM_RNE, PINF_D, PZERO_D, d2b(1.0));
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FMADD.D inf*0+1.0 -> qNaN, NV set");

    // inf - inf via fmsub (product +inf, subtrahend +inf) -> invalid.
    r = fmau_op(f_fmsub, RM_RNE, PINF_D, d2b(1.0), PINF_D);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FMSUB.D inf*1.0-inf -> qNaN, NV set");

    // finite*inf -> inf, no flags.
    r = fmau_op(f_fmadd, RM_RNE, d2b(2.0), PINF_D, PZERO_D);
    check(r.fdata == PINF_D && r.fflags == 0, "FMADD.D 2.0*inf+0 -> inf, no flags");

    test_result("T9 FMAU.D fmadd/fmsub/fnmadd/fnmsub: value + NX flag, specials");
}

//-----------------------------------------------------------------------------
// T10: FMAU.S fmadd/fmsub -- boxed values + NaN-box check.
//-----------------------------------------------------------------------------
static void test_fmau_s_basic(void) {
    uint32_t f_fmadd = bitv(FUNC_MAU_FUSED) | bitv(FUNC_B_SINGLE);
    uint32_t f_fmsub = bitv(FUNC_MAU_FUSED) | bitv(FUNC_MAU_SUB) | bitv(FUNC_B_SINGLE);

    float a = 1.5f, b = 2.0f, c = 1.0f;
    FpuResult r = fmau_op(f_fmadd, RM_RNE, box(f2b(a)), box(f2b(b)), box(f2b(c)));
    check(r.fdata == box(f2b(a * b + c)), "FMADD.S 1.5*2.0+1.0 == 4.0 (boxed)",
          r.fdata, box(f2b(a * b + c)));
    check(r.fflags == 0, "FMADD.S 1.5*2.0+1.0 exact, flags clean", r.fflags, 0);

    r = fmau_op(f_fmsub, RM_RNE, box(f2b(a)), box(f2b(b)), box(f2b(c)));
    check(r.fdata == box(f2b(a * b - c)), "FMSUB.S 1.5*2.0-1.0 == 2.0 (boxed)",
          r.fdata, box(f2b(a * b - c)));

    a = 1.0f; b = 1.0f; c = ldexpf(1.0f, -30);
    r = fmau_op(f_fmadd, RM_RNE, box(f2b(a)), box(f2b(b)), box(f2b(c)));
    check((r.fflags & 0x1) == (fma_exact_s(a, b, c) ? 0u : 1u), "FMADD.S 1.0*1.0+2^-30 sets NX");

    // un-boxed garbage upper bits on any source -> treated as canonical qNaN.
    uint64_t garbage = 0x1234567800000000ULL | (uint64_t)f2b(3.0f);
    r = fmau_op(f_fmadd, RM_RNE, garbage, box(f2b(1.0f)), box(f2b(1.0f)));
    check(r.fdata == box(QNAN_S), "FMADD.S unboxed src0 -> canonical qNaN in", r.fdata, box(QNAN_S));

    test_result("T10 FMAU.S fmadd/fmsub: value + NX flag + NaN-box check");
}

//-----------------------------------------------------------------------------
// T11: FMAU plain fmul.s/d -- FUNC_MAU_FUSED=0, addend forced don't-care
// (rv12 D6: c_is_zero = !op_fused || ...).
//-----------------------------------------------------------------------------
static void test_fmau_mul(void) {
    uint32_t f_mul_d = bitv(FUNC_DOUBLE);
    uint32_t f_mul_s = bitv(FUNC_B_SINGLE);

    double a = 1.5, b = 2.0;
    FpuResult r = fmau_op(f_mul_d, RM_RNE, d2b(a), d2b(b), 0xDEADBEEFDEADBEEFULL);
    check(r.fdata == d2b(a * b) && r.fflags == 0,
          "FMUL.D 1.5*2.0 == 3.0 exact, addend don't-care", r.fdata, d2b(a * b));

    a = 1.1; b = 1.1;
    r = fmau_op(f_mul_d, RM_RNE, d2b(a), d2b(b), 0);
    check(r.fdata == d2b(a * b), "FMUL.D 1.1*1.1 value", r.fdata, d2b(a * b));
    check((r.fflags & 0x1) == (fma_exact_d(a, b, 0.0) ? 0u : 1u), "FMUL.D 1.1*1.1 sets NX");

    float fa = 1.1f, fb = 1.1f;
    r = fmau_op(f_mul_s, RM_RNE, box(f2b(fa)), box(f2b(fb)), 0);
    check(r.fdata == box(f2b(fa * fb)), "FMUL.S 1.1*1.1 value (boxed)", r.fdata, box(f2b(fa * fb)));

    test_result("T11 FMAU plain fmul.s/d: value + NX flag, addend don't-care");
}

//-----------------------------------------------------------------------------
// T12: FDIV.D basic -- value + NX flag, via fdsu_op's multi-cycle loop.
//-----------------------------------------------------------------------------
static void test_fdsu_div_d_basic(void) {
    uint32_t f_div_d = bitv(FUNC_FDSU_DIV) | bitv(FUNC_DOUBLE);

    double a = 8.0, b = 2.0;
    FpuResult r = fdsu_op(f_div_d, RM_RNE, d2b(a), d2b(b));
    check(r.fdata == d2b(a / b) && r.fflags == 0,
          "FDIV.D 8.0/2.0 == 4.0 exact", r.fdata, d2b(a / b));

    a = 1.0; b = 3.0;
    r = fdsu_op(f_div_d, RM_RNE, d2b(a), d2b(b));
    check(r.fdata == d2b(a / b), "FDIV.D 1.0/3.0 value", r.fdata, d2b(a / b));
    check((r.fflags & 0x1) == (div_exact_d(a, b) ? 0u : 1u), "FDIV.D 1.0/3.0 sets NX");

    test_result("T12 FDIV.D: value + NX flag, real-compute path");
}

//-----------------------------------------------------------------------------
// T13: FDIV.S basic + NaN-box check.
//-----------------------------------------------------------------------------
static void test_fdsu_div_s_basic(void) {
    uint32_t f_div_s = bitv(FUNC_FDSU_DIV) | bitv(FUNC_B_SINGLE);

    float fa = 8.0f, fb = 2.0f;
    FpuResult r = fdsu_op(f_div_s, RM_RNE, box(f2b(fa)), box(f2b(fb)));
    check(r.fdata == box(f2b(fa / fb)) && r.fflags == 0,
          "FDIV.S 8.0/2.0 == 4.0 exact, boxed", r.fdata, box(f2b(fa / fb)));

    fa = 1.0f; fb = 3.0f;
    r = fdsu_op(f_div_s, RM_RNE, box(f2b(fa)), box(f2b(fb)));
    check(r.fdata == box(f2b(fa / fb)), "FDIV.S 1.0/3.0 value (boxed)", r.fdata, box(f2b(fa / fb)));
    check((r.fflags & 0x1) == (div_exact_s(fa, fb) ? 0u : 1u), "FDIV.S 1.0/3.0 sets NX");

    test_result("T13 FDIV.S: value + NX flag + NaN-box check");
}

//-----------------------------------------------------------------------------
// T14: FDIV abnormal cases (IEEE 754, FPU.v SECTION 14 fdiv_* wires).
//-----------------------------------------------------------------------------
static void test_fdsu_div_abnormal(void) {
    uint32_t f_div_d = bitv(FUNC_FDSU_DIV) | bitv(FUNC_DOUBLE);

    FpuResult r = fdsu_op(f_div_d, RM_RNE, PZERO_D, PZERO_D);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FDIV.D 0.0/0.0 -> qNaN, NV set");

    r = fdsu_op(f_div_d, RM_RNE, PINF_D, PINF_D);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FDIV.D inf/inf -> qNaN, NV set");

    r = fdsu_op(f_div_d, RM_RNE, d2b(5.0), PZERO_D);
    check(r.fdata == PINF_D && (r.fflags & 0x8), "FDIV.D 5.0/+0.0 -> +Inf, DZ set");

    r = fdsu_op(f_div_d, RM_RNE, d2b(-5.0), PZERO_D);
    check(r.fdata == NINF_D && (r.fflags & 0x8), "FDIV.D -5.0/+0.0 -> -Inf, DZ set");

    r = fdsu_op(f_div_d, RM_RNE, d2b(5.0), PINF_D);
    check(r.fdata == PZERO_D && r.fflags == 0, "FDIV.D 5.0/+inf -> +0.0, no flags");

    r = fdsu_op(f_div_d, RM_RNE, SNAN_D, d2b(1.0));
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FDIV.D sNaN/1.0 -> qNaN, NV set");

    r = fdsu_op(f_div_d, RM_RNE, QNAN_D, d2b(1.0));
    check(r.fdata == QNAN_D && r.fflags == 0, "FDIV.D qNaN/1.0 -> qNaN, no NV");

    test_result("T14 FDIV abnormal: 0/0, inf/inf, x/0 DZ, finite/inf, sNaN/qNaN in");
}

//-----------------------------------------------------------------------------
// T15: FSQRT.D basic -- value + NX flag.
//-----------------------------------------------------------------------------
static void test_fdsu_sqrt_d_basic(void) {
    uint32_t f_sqrt_d = bitv(FUNC_FDSU_SQRT) | bitv(FUNC_DOUBLE);

    double a = 4.0;
    FpuResult r = fdsu_op(f_sqrt_d, RM_RNE, d2b(a), 0);
    check(r.fdata == d2b(sqrt(a)) && r.fflags == 0,
          "FSQRT.D sqrt(4.0) == 2.0 exact", r.fdata, d2b(sqrt(a)));

    a = 2.0;
    r = fdsu_op(f_sqrt_d, RM_RNE, d2b(a), 0);
    check(r.fdata == d2b(sqrt(a)), "FSQRT.D sqrt(2.0) value", r.fdata, d2b(sqrt(a)));
    check((r.fflags & 0x1) == (sqrt_exact_d(a) ? 0u : 1u), "FSQRT.D sqrt(2.0) sets NX");

    test_result("T15 FSQRT.D: value + NX flag, real-compute path");
}

//-----------------------------------------------------------------------------
// T16: FSQRT.S basic + NaN-box check.
//-----------------------------------------------------------------------------
static void test_fdsu_sqrt_s_basic(void) {
    uint32_t f_sqrt_s = bitv(FUNC_FDSU_SQRT) | bitv(FUNC_B_SINGLE);

    float fa = 4.0f;
    FpuResult r = fdsu_op(f_sqrt_s, RM_RNE, box(f2b(fa)), 0);
    check(r.fdata == box(f2b(sqrtf(fa))) && r.fflags == 0,
          "FSQRT.S sqrt(4.0) == 2.0 exact, boxed", r.fdata, box(f2b(sqrtf(fa))));

    fa = 2.0f;
    r = fdsu_op(f_sqrt_s, RM_RNE, box(f2b(fa)), 0);
    check(r.fdata == box(f2b(sqrtf(fa))), "FSQRT.S sqrt(2.0) value (boxed)", r.fdata, box(f2b(sqrtf(fa))));
    check((r.fflags & 0x1) == (sqrt_exact_s(fa) ? 0u : 1u), "FSQRT.S sqrt(2.0) sets NX");

    test_result("T16 FSQRT.S: value + NX flag + NaN-box check");
}

//-----------------------------------------------------------------------------
// T17: FSQRT abnormal cases (IEEE 754, FPU.v SECTION 14 fsqrt_* wires).
//-----------------------------------------------------------------------------
static void test_fdsu_sqrt_abnormal(void) {
    uint32_t f_sqrt_d = bitv(FUNC_FDSU_SQRT) | bitv(FUNC_DOUBLE);

    FpuResult r = fdsu_op(f_sqrt_d, RM_RNE, d2b(-4.0), 0);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FSQRT.D sqrt(-4.0) -> qNaN, NV set");

    r = fdsu_op(f_sqrt_d, RM_RNE, SNAN_D, 0);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FSQRT.D sqrt(sNaN) -> qNaN, NV set");

    r = fdsu_op(f_sqrt_d, RM_RNE, QNAN_D, 0);
    check(r.fdata == QNAN_D && r.fflags == 0, "FSQRT.D sqrt(qNaN) -> qNaN, no NV");

    r = fdsu_op(f_sqrt_d, RM_RNE, PZERO_D, 0);
    check(r.fdata == PZERO_D && r.fflags == 0, "FSQRT.D sqrt(+0.0) -> +0.0, no flags");

    r = fdsu_op(f_sqrt_d, RM_RNE, NZERO_D, 0);
    check(r.fdata == NZERO_D && r.fflags == 0, "FSQRT.D sqrt(-0.0) -> -0.0, no flags");

    r = fdsu_op(f_sqrt_d, RM_RNE, PINF_D, 0);
    check(r.fdata == PINF_D && r.fflags == 0, "FSQRT.D sqrt(+inf) -> +inf, no flags");

    r = fdsu_op(f_sqrt_d, RM_RNE, NINF_D, 0);
    check(r.fdata == QNAN_D && (r.fflags & 0x10), "FSQRT.D sqrt(-inf) -> qNaN, NV set");

    test_result("T17 FSQRT abnormal: negative, +-0, +-inf, sNaN/qNaN in");
}

//-----------------------------------------------------------------------------
// T18: FDSU busy/full signal timing + preg passthrough via fdsu_preg_flop
// during the real-compute path (FPU.v: fpu_idu_fdsu_full asserted exactly
// while fdsu_state==FDSU_BUSY; fpu_rtu_ex1_falu_preg switches to the
// dispatch-time-latched fdsu_preg_flop once fdsu_cmplt_now fires).
//-----------------------------------------------------------------------------
static void test_fdsu_busy_preg(void) {
    uint32_t f_div_d = bitv(FUNC_FDSU_DIV) | bitv(FUNC_DOUBLE);

    tie_idle_inputs();
    dut->idu_fpu_ex1_fdsu_sel   = 1;
    dut->idu_fpu_ex1_func       = f_div_d;
    dut->idu_fpu_ex1_rm         = RM_RNE;
    dut->idu_fpu_ex1_fsrc0_data = d2b(1.0);
    dut->idu_fpu_ex1_fsrc1_data = d2b(3.0);
    dut->idu_fpu_ex1_dst0_reg   = 9;
    dut->eval();
    check(!dut->fpu_rtu_ex1_falu_fvld, "FDIV.D real-compute not vld on dispatch cycle");
    check(!dut->fpu_idu_fdsu_full, "FDIV.D full not yet asserted on dispatch cycle");

    tick();
    dut->eval();
    check(dut->fpu_idu_fdsu_full == 1, "FDIV.D full asserted the cycle after dispatch");

    while (dut->fpu_idu_fdsu_full) {
        tick();
        dut->eval();
    }
    check(dut->fpu_rtu_ex1_falu_fvld == 1, "FDIV.D fvld asserted once full deasserts (CMPLT)");
    check(dut->fpu_rtu_ex1_falu_preg == 9,
          "FDIV.D preg == dispatch-time dst0_reg via fdsu_preg_flop at CMPLT",
          dut->fpu_rtu_ex1_falu_preg, 9);
    check(dut->fpu_rtu_ex1_falu_fdata == d2b(1.0 / 3.0),
          "FDIV.D real-compute value 1.0/3.0 at CMPLT", dut->fpu_rtu_ex1_falu_fdata, d2b(1.0 / 3.0));

    dut->idu_fpu_ex1_fdsu_sel = 0;
    tick();
    dut->eval();
    check(!dut->fpu_idu_fdsu_full, "FDIV.D full stays deasserted after CMPLT->IDLE");
    check(!dut->fpu_rtu_ex1_falu_fvld, "FDIV.D fvld drops after sel deasserted, CMPLT->IDLE");

    test_result("T18 FDSU busy/full timing + preg passthrough via fdsu_preg_flop");
}

//-----------------------------------------------------------------------------
// i2f inexactness oracle -- same widening trick as add_exact_*: x86-64
// long double (80-bit, 64-bit significand) holds every 32/64-bit integer
// exactly, so if the native (correctly-rounded) int->float/double cast
// differs from the integer itself, the conversion lost bits (NX).
//-----------------------------------------------------------------------------
static bool i2f_exact_f(int64_t  v) { return (long double)(float)v  == (long double)v; }
static bool i2f_exact_f(uint64_t v) { return (long double)(float)v  == (long double)v; }
static bool i2f_exact_d(int64_t  v) { return (long double)(double)v == (long double)v; }
static bool i2f_exact_d(uint64_t v) { return (long double)(double)v == (long double)v; }

//-----------------------------------------------------------------------------
// T19: FCNVT f2i, 64-bit destinations -- fcvt.l.s/l.d/lu.s/lu.d. Value +
// RNE tie handling, the 2**62/2**63/2**64 boundary rows that pin the
// donor's range terms (rv12 FPUAlu.v:1264-1271), and the NaN->MAXIMUM-only
// saturation asymmetry (NV set, NX masked). xvld routing, not fvld.
//-----------------------------------------------------------------------------
static void test_fcnvt_f2i_64(void) {
    uint32_t f_l_s  = bitv(FUNC_CVT_INT) | bitv(FUNC_CVT_F2I) | bitv(FUNC_CVT_WIDE64);
    uint32_t f_lu_s = f_l_s | bitv(FUNC_CVT_UNSIGNED);
    uint32_t f_l_d  = f_l_s | bitv(FUNC_DOUBLE);
    uint32_t f_lu_d = f_lu_s | bitv(FUNC_DOUBLE);

    FpuResult r = fcnvt_op(f_l_s, RM_RNE, box(f2b(1.75f)), 5);
    check(r.xdata == 2 && r.xvld && !r.fvld, "FCVT.L.S 1.75 -> 2, xvld routing", r.xdata, 2);
    check(r.fflags == 0x1, "FCVT.L.S 1.75 inexact, NX only", r.fflags, 0x1);
    check(dut->fpu_rtu_ex1_falu_preg == 5, "FCVT.L.S preg == dst0_reg (GPR writeback)",
          dut->fpu_rtu_ex1_falu_preg, 5);

    r = fcnvt_op(f_l_s, RM_RNE, box(f2b(-1.75f)));
    check(r.xdata == 0xFFFFFFFFFFFFFFFEULL, "FCVT.L.S -1.75 -> -2", r.xdata, 0xFFFFFFFFFFFFFFFEULL);

    r = fcnvt_op(f_l_s, RM_RNE, box(f2b(1.5f)));
    check(r.xdata == 2 && r.fflags == 0x1, "FCVT.L.S 1.5 RNE tie -> 2 (to even), NX", r.xdata, 2);

    r = fcnvt_op(f_l_s, RM_RNE, box(f2b(2.5f)));
    check(r.xdata == 2 && r.fflags == 0x1, "FCVT.L.S 2.5 RNE tie -> 2 (to even), NX", r.xdata, 2);

    r = fcnvt_op(f_l_s, RM_RNE, box(f2b(-2.5f)));
    check(r.xdata == 0xFFFFFFFFFFFFFFFEULL && r.fflags == 0x1,
          "FCVT.L.S -2.5 RNE tie -> -2 (to even), NX", r.xdata, 0xFFFFFFFFFFFFFFFEULL);

    r = fcnvt_op(f_l_s, RM_RNE, box(0x4E800000u));   // 2**30, exact
    check(r.xdata == 0x40000000ULL && r.fflags == 0, "FCVT.L.S 2**30 exact, no flags",
          r.xdata, 0x40000000ULL);

    // 2**62 fits signed 64-bit -- the old sat_si64 test fired one bit early
    // (at 2**62) and would have saturated this row to INT64_MAX.
    r = fcnvt_op(f_l_d, RM_RNE, d2b(ldexp(1.0, 62)));
    check(r.xdata == 0x4000000000000000ULL && r.fflags == 0,
          "FCVT.L.D 2**62 fits int64, no saturation", r.xdata, 0x4000000000000000ULL);

    r = fcnvt_op(f_l_d, RM_RNE, d2b(-ldexp(1.0, 62)));
    check(r.xdata == 0xC000000000000000ULL && r.fflags == 0,
          "FCVT.L.D -2**62 fits int64, no saturation", r.xdata, 0xC000000000000000ULL);

    r = fcnvt_op(f_l_d, RM_RNE, d2b(ldexp(1.0, 63) - 1024.0));   // 2**63 - 1024, exact
    check(r.xdata == 0x7FFFFFFFFFFFFC00ULL && r.fflags == 0,
          "FCVT.L.D 2**63-1024 just below INT64_MAX, no saturation",
          r.xdata, 0x7FFFFFFFFFFFFC00ULL);

    r = fcnvt_op(f_l_d, RM_RNE, d2b(ldexp(1.0, 63)));
    check(r.xdata == 0x7FFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.L.D 2**63 -> INT64_MAX, NV", r.xdata, 0x7FFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_l_d, RM_RNE, d2b(-ldexp(1.0, 63)));
    check(r.xdata == 0x8000000000000000ULL && r.fflags == 0,
          "FCVT.L.D -2**63 -> INT64_MIN exactly, no NV", r.xdata, 0x8000000000000000ULL);

    r = fcnvt_op(f_l_d, RM_RNE, d2b(-(ldexp(1.0, 63) + 2048.0)));
    check(r.xdata == 0x8000000000000000ULL && r.fflags == 0x10,
          "FCVT.L.D -(2**63+2048) -> INT64_MIN, NV", r.xdata, 0x8000000000000000ULL);

    // 2**63 fits UNSIGNED 64-bit -- the old sat_ui64 test fired at 2**63
    // (bit 63) and would have saturated this row to UINT64_MAX.
    r = fcnvt_op(f_lu_d, RM_RNE, d2b(ldexp(1.0, 63)));
    check(r.xdata == 0x8000000000000000ULL && r.fflags == 0,
          "FCVT.LU.D 2**63 fits uint64, no saturation", r.xdata, 0x8000000000000000ULL);

    r = fcnvt_op(f_lu_d, RM_RNE, d2b((double)((1LL << 53) - 1) * 2048.0));  // 2**64 - 2048
    check(r.xdata == 0xFFFFFFFFFFFFF800ULL && r.fflags == 0,
          "FCVT.LU.D 2**64-2048 fits uint64, no saturation",
          r.xdata, 0xFFFFFFFFFFFFF800ULL);

    r = fcnvt_op(f_lu_d, RM_RNE, d2b(ldexp(1.0, 64)));
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.LU.D 2**64 -> UINT64_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_lu_s, RM_RNE, box(0x4F000000u));   // 2**31f
    check(r.xdata == 0x80000000ULL && r.fflags == 0, "FCVT.LU.S 2**31 exact, no flags",
          r.xdata, 0x80000000ULL);

    r = fcnvt_op(f_lu_s, RM_RNE, box(0x53800000u));   // 2**40f
    check(r.xdata == 0x10000000000ULL && r.fflags == 0, "FCVT.LU.S 2**40 exact, no flags",
          r.xdata, 0x10000000000ULL);

    // NaN converts to the MAXIMUM of the destination, never the minimum.
    r = fcnvt_op(f_l_s, RM_RNE, box(QNAN_S));
    check(r.xdata == 0x7FFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.L.S qNaN -> INT64_MAX, NV", r.xdata, 0x7FFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_l_d, RM_RNE, SNAN_D);
    check(r.xdata == 0x7FFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.L.D sNaN -> INT64_MAX, NV", r.xdata, 0x7FFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_lu_d, RM_RNE, QNAN_D);
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.LU.D qNaN -> UINT64_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_l_d, RM_RNE, PINF_D);
    check(r.xdata == 0x7FFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.L.D +inf -> INT64_MAX, NV", r.xdata, 0x7FFFFFFFFFFFFFFFULL);
    r = fcnvt_op(f_l_d, RM_RNE, NINF_D);
    check(r.xdata == 0x8000000000000000ULL && r.fflags == 0x10,
          "FCVT.L.D -inf -> INT64_MIN, NV", r.xdata, 0x8000000000000000ULL);

    r = fcnvt_op(f_lu_d, RM_RNE, PINF_D);
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.LU.D +inf -> UINT64_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);
    r = fcnvt_op(f_lu_d, RM_RNE, NINF_D);
    check(r.xdata == 0 && r.fflags == 0x10, "FCVT.LU.D -inf -> 0, NV", r.xdata, 0);

    r = fcnvt_op(f_l_s, RM_RNE, box(f2b(1e30f)));   // huge single (e > 64)
    check(r.xdata == 0x7FFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.L.S 1e30f -> INT64_MAX, NV", r.xdata, 0x7FFFFFFFFFFFFFFFULL);

    test_result("T19 FCNVT f2i 64-bit (l.s/l.d/lu.s/lu.d): value, RNE ties, "
                "2**62/2**63/2**64 boundaries, NaN->MAX, inf, xvld routing");
}

//-----------------------------------------------------------------------------
// T20: FCNVT f2i, 32-bit destinations -- fcvt.w.s/w.d/wu.s/wu.d. The 32-bit
// answers are sign-extended to 64 bits in xdata (the donor crack's uop-1 is
// fmv.x.w for the whole w/wu family -- see f2i_xdata in FPU.v SECTION 11a),
// so w/wu rows check the extended 64-bit value. Includes the -2**32 row that
// the old min_si32 term missed (exactly 2**32 with the low 31 bits clear).
//-----------------------------------------------------------------------------
static void test_fcnvt_f2i_32(void) {
    uint32_t f_w_s  = bitv(FUNC_CVT_INT) | bitv(FUNC_CVT_F2I);
    uint32_t f_wu_s = f_w_s | bitv(FUNC_CVT_UNSIGNED);
    uint32_t f_w_d  = f_w_s | bitv(FUNC_DOUBLE);
    uint32_t f_wu_d = f_wu_s | bitv(FUNC_DOUBLE);

    FpuResult r = fcnvt_op(f_w_s, RM_RNE, box(f2b(1.75f)));
    check(r.xdata == 2 && r.xvld && !r.fvld, "FCVT.W.S 1.75 -> 2, xvld routing", r.xdata, 2);
    check(r.fflags == 0x1, "FCVT.W.S 1.75 inexact, NX only", r.fflags, 0x1);

    r = fcnvt_op(f_w_s, RM_RNE, box(f2b(-1.75f)));
    check(r.xdata == 0xFFFFFFFFFFFFFFFEULL, "FCVT.W.S -1.75 -> -2 (sign-extended)",
          r.xdata, 0xFFFFFFFFFFFFFFFEULL);

    r = fcnvt_op(f_w_s, RM_RNE, box(f2b(2.5f)));
    check(r.xdata == 2 && r.fflags == 0x1, "FCVT.W.S 2.5 RNE tie -> 2 (to even), NX", r.xdata, 2);

    r = fcnvt_op(f_w_s, RM_RNE, box(f2b(-2.5f)));
    check(r.xdata == 0xFFFFFFFFFFFFFFFEULL && r.fflags == 0x1,
          "FCVT.W.S -2.5 RNE tie -> -2 (to even), NX", r.xdata, 0xFFFFFFFFFFFFFFFEULL);

    r = fcnvt_op(f_w_s, RM_RNE, box(f2b(0.4f)));
    check(r.xdata == 0 && r.fflags == 0x1, "FCVT.W.S 0.4 -> 0, NX", r.xdata, 0);

    r = fcnvt_op(f_w_s, RM_RNE, box(0x4E800000u));   // 2**30f, exact
    check(r.xdata == 0x40000000ULL && r.fflags == 0, "FCVT.W.S 2**30 exact, no flags",
          r.xdata, 0x40000000ULL);

    r = fcnvt_op(f_w_s, RM_RNE, box(0x4F000000u));   // 2**31f
    check(r.xdata == 0x7FFFFFFFULL && r.fflags == 0x10,
          "FCVT.W.S 2**31 -> INT32_MAX, NV", r.xdata, 0x7FFFFFFFULL);

    r = fcnvt_op(f_w_s, RM_RNE, box(0xCF000000u));   // -2**31f
    check(r.xdata == 0xFFFFFFFF80000000ULL && r.fflags == 0,
          "FCVT.W.S -2**31 -> INT32_MIN exactly, no NV", r.xdata, 0xFFFFFFFF80000000ULL);

    // exactly -2**32: the old min_si32 bit-test required the low 31 bits
    // nonzero and would have returned 0 instead of INT32_MIN.
    r = fcnvt_op(f_w_s, RM_RNE, box(0xCF800000u));   // -2**32f
    check(r.xdata == 0xFFFFFFFF80000000ULL && r.fflags == 0x10,
          "FCVT.W.S -2**32 -> INT32_MIN, NV", r.xdata, 0xFFFFFFFF80000000ULL);

    r = fcnvt_op(f_w_d, RM_RNE, d2b(ldexp(1.0, 31)));
    check(r.xdata == 0x7FFFFFFFULL && r.fflags == 0x10,
          "FCVT.W.D 2**31 -> INT32_MAX, NV", r.xdata, 0x7FFFFFFFULL);

    r = fcnvt_op(f_w_d, RM_RNE, d2b(-ldexp(1.0, 32)));
    check(r.xdata == 0xFFFFFFFF80000000ULL && r.fflags == 0x10,
          "FCVT.W.D -2**32 -> INT32_MIN, NV", r.xdata, 0xFFFFFFFF80000000ULL);

    r = fcnvt_op(f_w_d, RM_RNE, QNAN_D);
    check(r.xdata == 0x7FFFFFFFULL && r.fflags == 0x10,
          "FCVT.W.D qNaN -> INT32_MAX, NV", r.xdata, 0x7FFFFFFFULL);

    r = fcnvt_op(f_w_d, RM_RNE, PINF_D);
    check(r.xdata == 0x7FFFFFFFULL && r.fflags == 0x10, "FCVT.W.D +inf -> INT32_MAX, NV",
          r.xdata, 0x7FFFFFFFULL);
    r = fcnvt_op(f_w_d, RM_RNE, NINF_D);
    check(r.xdata == 0xFFFFFFFF80000000ULL && r.fflags == 0x10,
          "FCVT.W.D -inf -> INT32_MIN, NV", r.xdata, 0xFFFFFFFF80000000ULL);

    // wu family: 32-bit answers still take the fmv.x.w sign-extension in
    // xdata (the donor crack's uop-1 is fmv.x.w for w AND wu).
    r = fcnvt_op(f_wu_s, RM_RNE, box(0x4F000000u));   // 2**31f
    check(r.xdata == 0xFFFFFFFF80000000ULL && r.fflags == 0,
          "FCVT.WU.S 2**31 -> 0x80000000, sign-extended, no NV",
          r.xdata, 0xFFFFFFFF80000000ULL);

    r = fcnvt_op(f_wu_s, RM_RNE, box(0x50000000u));   // 2**32f
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.WU.S 2**32 -> UINT32_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_wu_d, RM_RNE, d2b(ldexp(1.0, 32)));
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.WU.D 2**32 -> UINT32_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_wu_d, RM_RNE, d2b(-1.0));
    check(r.xdata == 0 && r.fflags == 0x10, "FCVT.WU.D -1.0 -> 0, NV", r.xdata, 0);

    r = fcnvt_op(f_wu_d, RM_RNE, QNAN_D);
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.WU.D qNaN -> UINT32_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);

    r = fcnvt_op(f_wu_d, RM_RNE, NINF_D);
    check(r.xdata == 0 && r.fflags == 0x10, "FCVT.WU.D -inf -> 0, NV", r.xdata, 0);
    r = fcnvt_op(f_wu_d, RM_RNE, PINF_D);
    check(r.xdata == 0xFFFFFFFFFFFFFFFFULL && r.fflags == 0x10,
          "FCVT.WU.D +inf -> UINT32_MAX, NV", r.xdata, 0xFFFFFFFFFFFFFFFFULL);

    test_result("T20 FCNVT f2i 32-bit (w.s/w.d/wu.s/wu.d): value, RNE ties, "
                "INT32/UINT32 saturation, sign-extension, -2**32 min fix");
}

//-----------------------------------------------------------------------------
// T21: FCNVT i2f -- all eight forms (fcvt.s.w/wu/l/lu, fcvt.d.w/wu/l/lu).
// Value oracle: the host's native correctly-rounded int->float/double cast;
// NX via the i2f_exact_* widening trick. FRF destination (fvld, not xvld).
//-----------------------------------------------------------------------------
static void test_fcnvt_i2f(void) {
    uint32_t f_s_w  = bitv(FUNC_CVT_INT);
    uint32_t f_s_wu = f_s_w | bitv(FUNC_CVT_UNSIGNED);
    uint32_t f_s_l  = f_s_w | bitv(FUNC_CVT_WIDE64);
    uint32_t f_s_lu = f_s_wu | bitv(FUNC_CVT_WIDE64);
    uint32_t f_d_w  = f_s_w  | bitv(FUNC_DOUBLE);
    uint32_t f_d_wu = f_s_wu | bitv(FUNC_DOUBLE);
    uint32_t f_d_l  = f_s_l  | bitv(FUNC_DOUBLE);
    uint32_t f_d_lu = f_s_lu | bitv(FUNC_DOUBLE);

    FpuResult r = fcnvt_op(f_s_w, RM_RNE, 0x000000007FFFFFFFULL);   // INT32_MAX
    check(r.fdata == box(f2b((float)(int32_t)0x7FFFFFFF)) && r.fvld && !r.xvld,
          "FCVT.S.W INT32_MAX -> 2**31 (rounds up), fvld routing",
          r.fdata, box(f2b((float)(int32_t)0x7FFFFFFF)));
    check((r.fflags & 0x1) == (i2f_exact_f((int64_t)0x7FFFFFFF) ? 0u : 1u),
          "FCVT.S.W INT32_MAX sets NX (2**31-1 not representable in single)", r.fflags, 1);

    r = fcnvt_op(f_s_w, RM_RNE, 0x0000000080000000ULL);   // INT32_MIN
    check(r.fdata == box(f2b((float)(int32_t)0x80000000u)) && r.fflags == 0,
          "FCVT.S.W INT32_MIN exact", r.fdata, box(f2b((float)(int32_t)0x80000000u)));

    r = fcnvt_op(f_s_w, RM_RNE, 0xFFFFFFFFFFFFFFFFULL);   // -1
    check(r.fdata == box(f2b(-1.0f)) && r.fflags == 0, "FCVT.S.W -1 exact", r.fdata, box(f2b(-1.0f)));

    r = fcnvt_op(f_s_w, RM_RNE, 0);
    check(r.fdata == box(0) && r.fflags == 0, "FCVT.S.W 0 -> +0.0, no flags", r.fdata, box(0));

    r = fcnvt_op(f_s_wu, RM_RNE, 0xFFFFFFFFULL);   // 2**32 - 1
    check(r.fdata == box(f2b((float)(uint32_t)0xFFFFFFFFu)),
          "FCVT.S.WU 2**32-1 -> 2**32", r.fdata, box(f2b((float)(uint32_t)0xFFFFFFFFu)));
    check((r.fflags & 0x1) == (i2f_exact_f((uint64_t)0xFFFFFFFFu) ? 0u : 1u),
          "FCVT.S.WU 2**32-1 sets NX", r.fflags, 1);

    r = fcnvt_op(f_s_wu, RM_RNE, 0x80000000ULL);   // 2**31 as unsigned
    check(r.fdata == box(f2b((float)(uint32_t)0x80000000u)) && r.fflags == 0,
          "FCVT.S.WU 2**31 exact (unsigned view)", r.fdata, box(f2b((float)(uint32_t)0x80000000u)));

    r = fcnvt_op(f_s_l, RM_RNE, 0x7FFFFFFFFFFFFFFFULL);   // INT64_MAX
    check(r.fdata == box(f2b((float)(int64_t)0x7FFFFFFFFFFFFFFFULL)),
          "FCVT.S.L INT64_MAX -> 2**63", r.fdata, box(f2b((float)(int64_t)0x7FFFFFFFFFFFFFFFULL)));
    check((r.fflags & 0x1) == (i2f_exact_f((int64_t)0x7FFFFFFFFFFFFFFFULL) ? 0u : 1u),
          "FCVT.S.L INT64_MAX sets NX", r.fflags, 1);

    r = fcnvt_op(f_s_l, RM_RNE, 0x20000000000000ULL);   // 2**53
    check(r.fdata == box(f2b((float)(int64_t)(1LL << 53))) && r.fflags == 0,
          "FCVT.S.L 2**53 exact", r.fdata, box(f2b((float)(int64_t)(1LL << 53))));

    r = fcnvt_op(f_s_l, RM_RNE, 0x8000000000000000ULL);   // -2**63
    check(r.fdata == box(f2b((float)(int64_t)(-(1LL << 63)))) && r.fflags == 0,
          "FCVT.S.L -2**63 exact", r.fdata, box(f2b((float)(int64_t)(-(1LL << 63)))));

    r = fcnvt_op(f_s_lu, RM_RNE, 0xFFFFFFFFFFFFFFFFULL);   // UINT64_MAX
    check(r.fdata == box(f2b((float)(uint64_t)0xFFFFFFFFFFFFFFFFULL)),
          "FCVT.S.LU UINT64_MAX -> 2**64", r.fdata, box(f2b((float)(uint64_t)0xFFFFFFFFFFFFFFFFULL)));
    check((r.fflags & 0x1) == (i2f_exact_f((uint64_t)0xFFFFFFFFFFFFFFFFULL) ? 0u : 1u),
          "FCVT.S.LU UINT64_MAX sets NX", r.fflags, 1);

    r = fcnvt_op(f_s_lu, RM_RNE, 0x100000000ULL);   // 2**32
    check(r.fdata == box(f2b((float)(uint64_t)(1ULL << 32))) && r.fflags == 0,
          "FCVT.S.LU 2**32 exact", r.fdata, box(f2b((float)(uint64_t)(1ULL << 32))));

    r = fcnvt_op(f_d_w, RM_RNE, 0xFFFFFFFFFFFFFFFFULL);   // -1
    check(r.fdata == d2b(-1.0) && r.fflags == 0, "FCVT.D.W -1 exact", r.fdata, d2b(-1.0));

    r = fcnvt_op(f_d_w, RM_RNE, 0x0000000080000000ULL);   // INT32_MIN
    check(r.fdata == d2b((double)(int32_t)0x80000000u) && r.fflags == 0,
          "FCVT.D.W INT32_MIN exact", r.fdata, d2b((double)(int32_t)0x80000000u));

    r = fcnvt_op(f_d_wu, RM_RNE, 0xFFFFFFFFULL);   // 2**32 - 1
    check(r.fdata == d2b(4294967295.0) && r.fflags == 0,
          "FCVT.D.WU 2**32-1 exact (fits binary64)", r.fdata, d2b(4294967295.0));

    r = fcnvt_op(f_d_l, RM_RNE, 0x8000000000000000ULL);   // -2**63
    check(r.fdata == d2b(-ldexp(1.0, 63)) && r.fflags == 0,
          "FCVT.D.L -2**63 exact", r.fdata, d2b(-ldexp(1.0, 63)));

    r = fcnvt_op(f_d_l, RM_RNE, 0x7FFFFFFFFFFFFFFFULL);   // 2**63 - 1
    check(r.fdata == d2b(ldexp(1.0, 63)),
          "FCVT.D.L 2**63-1 -> 2**63 (RNE tie to even)", r.fdata, d2b(ldexp(1.0, 63)));
    check((r.fflags & 0x1) == (i2f_exact_d((int64_t)0x7FFFFFFFFFFFFFFFULL) ? 0u : 1u),
          "FCVT.D.L 2**63-1 sets NX", r.fflags, 1);

    r = fcnvt_op(f_d_lu, RM_RNE, 0xFFFFFFFFFFFFFFFFULL);  // 2**64 - 1
    check(r.fdata == d2b(ldexp(1.0, 64)),
          "FCVT.D.LU UINT64_MAX -> 2**64", r.fdata, d2b(ldexp(1.0, 64)));
    check((r.fflags & 0x1) == (i2f_exact_d((uint64_t)0xFFFFFFFFFFFFFFFFULL) ? 0u : 1u),
          "FCVT.D.LU UINT64_MAX sets NX", r.fflags, 1);

    r = fcnvt_op(f_d_lu, RM_RNE, 0x1000000000000000ULL);   // 2**60
    check(r.fdata == d2b(ldexp(1.0, 60)) && r.fflags == 0,
          "FCVT.D.LU 2**60 exact", r.fdata, d2b(ldexp(1.0, 60)));

    test_result("T21 FCNVT i2f (s/d x w/wu/l/lu): value via native-cast oracle, NX");
}

//-----------------------------------------------------------------------------
// T22: FMV x.w/x.d/w.x/d.x -- bit-pattern moves, no conversion. FMV.X.W
// sign-extends bit 31 on RV64, FMV.X.D is identity, FMV.W.X NaN-boxes the
// 32-bit GPR view, FMV.D.X is 64-bit identity. Valid routing: x.* -> xvld
// only, w.*/d.x -> fvld only.
//-----------------------------------------------------------------------------
static void test_fspu_fmv(void) {
    uint32_t f_mv_xw = bitv(FUNC_SPU_MV) | bitv(FUNC_SPU_MV_XF);
    uint32_t f_mv_xd = f_mv_xw | bitv(FUNC_DOUBLE);
    uint32_t f_mv_wx = bitv(FUNC_SPU_MV);
    uint32_t f_mv_dx = f_mv_wx | bitv(FUNC_DOUBLE);

    FpuResult r = fspu_op(f_mv_xw, box(0xC0000000u), 0);
    check(r.xdata == 0xFFFFFFFFC0000000ULL && r.xvld && !r.fvld,
          "FMV.X.W sign-extends bit 31 (RV64), xvld routing", r.xdata, 0xFFFFFFFFC0000000ULL);
    check(r.fflags == 0, "FMV.X.W no flags");

    r = fspu_op(f_mv_xw, box(0x40000000u), 0);
    check(r.xdata == 0x0000000040000000ULL, "FMV.X.W positive: upper half zero",
          r.xdata, 0x40000000ULL);

    r = fspu_op(f_mv_xd, 0x8000000000000001ULL, 0);
    check(r.xdata == 0x8000000000000001ULL && r.xvld && !r.fvld,
          "FMV.X.D 64-bit bit-identity, xvld routing", r.xdata, 0x8000000000000001ULL);

    r = fspu_op(f_mv_wx, 0x0000000012345678ULL, 0);
    check(r.fdata == box(0x12345678u) && r.fvld && !r.xvld,
          "FMV.W.X NaN-boxes the 32-bit GPR view, fvld routing", r.fdata, box(0x12345678u));

    r = fspu_op(f_mv_wx, 0xFFFFFFFFFFFFFFFFULL, 0);
    check(r.fdata == 0xFFFFFFFFFFFFFFFFULL, "FMV.W.X all-ones GPR -> all-ones box",
          r.fdata, 0xFFFFFFFFFFFFFFFFULL);

    r = fspu_op(f_mv_dx, 0xDEADBEEF12345678ULL, 0);
    check(r.fdata == 0xDEADBEEF12345678ULL && r.fvld && !r.xvld,
          "FMV.D.X 64-bit identity, fvld routing", r.fdata, 0xDEADBEEF12345678ULL);

    test_result("T22 FMV x.w/x.d/w.x/d.x: bit moves, sign-extension, NaN-box, valid routing");
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
    test_fmau_d_basic();
    test_fmau_s_basic();
    test_fmau_mul();
    test_dst0_reg_preg_passthrough();
    test_fdsu_div_d_basic();
    test_fdsu_div_s_basic();
    test_fdsu_div_abnormal();
    test_fdsu_sqrt_d_basic();
    test_fdsu_sqrt_s_basic();
    test_fdsu_sqrt_abnormal();
    test_fdsu_busy_preg();
    test_fcnvt_f2i_64();
    test_fcnvt_f2i_32();
    test_fcnvt_i2f();
    test_fspu_fmv();

    printf("[fpu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
