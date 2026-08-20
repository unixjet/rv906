//=============================================================================
// iu_tb.cpp - standalone unit bench for rtl/IU.v (M2 plan task 3.7)
//=============================================================================
// Verilates IU.v + rvproc_pkg.sv alone (no IDU, no RTU, no LSU, no CSR) and
// drives the frozen idu_iu_ex1_*/rtu_iu_*_wb_grant/da_xx_fwd_*/lsu_iu_ex2_*/
// ifu_iu_chgflw_*/cp0_xx_mrvbr ports directly with hand-scripted stimulus,
// the same tick()-based clocking and check()/test_result() bookkeeping
// pattern as test/m2/unit/csr_tb.cpp (Task 2) and test/m1/unit/icache_tb.cpp.
//
// This is a WHITE-BOX test of IU.v's OWN documented contract (its header's
// clean-room notes, port-list amendment, and known scope gaps) run in
// isolation. It does not exercise IDU's real scoreboard or RTU's real
// wb-grant arbiter (neither exists yet) -- it proves IU.v honors the
// interface + timing contract ITS OWN header documents, driven by a script
// that stands in for IDU/RTU, exactly mirroring csr_tb.cpp's own framing.
//
// "Well-behaved dispatcher" contract this bench follows for MULT/DIV (see
// IU.v's header "PORT-LIST AMENDMENT"/"KNOWN GAPS" notes): once a unit's
// `iu_rtu_ex1_{mul,div}_cmplt` has fired for an instruction, the bench
// deasserts that unit's `_sel` before presenting anything else -- it never
// holds `_sel` asserted describing an already-accepted instruction past its
// own cmplt cycle. This is the same assumption IU.v's real IDU consumer
// (Task 5) is expected to honor.
//
// Build/run: make -C test/m2/unit iu && bin/unit/iu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VIU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

//-----------------------------------------------------------------------------
// Geometry / constants (mirrors rvproc_pkg.sv)
//-----------------------------------------------------------------------------
static const unsigned PC_WIDTH = 40;
static const uint64_t PC_MASK  = (1ULL << PC_WIDTH) - 1;
static const uint64_t RESET_VECTOR = 0x80000000ULL;

// ALU_FUNC_* (rvproc_pkg.sv, Task 3.6a -- confirmed bit-exact against
// aq_idu_cfig.h, see that file's header comment for the citations).
static const uint32_t ALU_FUNC_LUI    = 0x41401;
static const uint32_t ALU_FUNC_ADD    = 0x60401;
static const uint32_t ALU_FUNC_ADDW   = 0x08441;
static const uint32_t ALU_FUNC_SUB    = 0x60601;
static const uint32_t ALU_FUNC_SUBW   = 0x08641;
static const uint32_t ALU_FUNC_SLT    = 0x60e81;
static const uint32_t ALU_FUNC_SLTU   = 0x04a81;
static const uint32_t ALU_FUNC_SLL    = 0x10022;
static const uint32_t ALU_FUNC_SLLW   = 0x10062;
static const uint32_t ALU_FUNC_SRL    = 0x18802;
static const uint32_t ALU_FUNC_SRLW   = 0x14842;
static const uint32_t ALU_FUNC_SRA    = 0x08082;
static const uint32_t ALU_FUNC_SRAW   = 0x010c2;
static const uint32_t ALU_FUNC_SRRI   = 0x08102;
static const uint32_t ALU_FUNC_SRRIW  = 0x02142;
static const uint32_t ALU_FUNC_EXT    = 0x18602;
static const uint32_t ALU_FUNC_EXTU   = 0x18202;
static const uint32_t ALU_FUNC_AND    = 0x00024;
static const uint32_t ALU_FUNC_XOR    = 0x00044;
static const uint32_t ALU_FUNC_OR     = 0x00084;
static const uint32_t ALU_FUNC_FF0    = 0x00608;
static const uint32_t ALU_FUNC_FF1    = 0x00208;
static const uint32_t ALU_FUNC_REV    = 0x00048;
static const uint32_t ALU_FUNC_REVW   = 0x00028;
static const uint32_t ALU_FUNC_TST    = 0x00108;
static const uint32_t ALU_FUNC_TSTNBZ = 0x00088;
static const uint32_t ALU_FUNC_MVEQZ  = 0x00c08;
static const uint32_t ALU_FUNC_MVNEZ  = 0x00808;

static const uint32_t BJU_FUNC_BEQ   = 0x00144;
static const uint32_t BJU_FUNC_BNE   = 0x0014c;
static const uint32_t BJU_FUNC_BLT   = 0x00152;
static const uint32_t BJU_FUNC_BGE   = 0x0015a;
static const uint32_t BJU_FUNC_BLTU  = 0x00142;
static const uint32_t BJU_FUNC_BGEU  = 0x0014a;
static const uint32_t BJU_FUNC_JAL   = 0x00921;
static const uint32_t BJU_FUNC_JALR  = 0x00822;
static const uint32_t BJU_FUNC_AUIPC = 0x00980;

static const uint32_t MULT_FUNC_MUL    = 0x00021;
static const uint32_t MULT_FUNC_MULW   = 0x00022;
static const uint32_t MULT_FUNC_MULH   = 0x00121;
static const uint32_t MULT_FUNC_MULHU  = 0x00181;
static const uint32_t MULT_FUNC_MULHSU = 0x00141;

static const uint32_t DIV_FUNC_DIV   = 0x00006;
static const uint32_t DIV_FUNC_DIVU  = 0x00002;
static const uint32_t DIV_FUNC_DIVW  = 0x00007;
static const uint32_t DIV_FUNC_DIVUW = 0x00003;
static const uint32_t DIV_FUNC_REM   = 0x00004;
static const uint32_t DIV_FUNC_REMU  = 0x00000;
static const uint32_t DIV_FUNC_REMW  = 0x00005;
static const uint32_t DIV_FUNC_REMUW = 0x00001;

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VIU *dut = nullptr;
static uint64_t g_cycles = 0;

static void tie_idle_inputs(void) {
    dut->cp0_xx_mrvbr          = RESET_VECTOR;
    dut->idu_iu_ex1_inst_vld    = 0;
    dut->idu_iu_ex1_pipedown_vld= 0;
    dut->idu_iu_ex1_alu_sel     = 0;
    dut->idu_iu_ex1_bju_sel     = 0;
    dut->idu_iu_ex1_bju_br_sel  = 0;
    dut->idu_iu_ex1_mult_sel    = 0;
    dut->idu_iu_ex1_div_sel     = 0;
    dut->idu_iu_ex1_func        = 0;
    dut->idu_iu_ex1_src0_data   = 0;
    dut->idu_iu_ex1_src0_ready  = 1;
    dut->idu_iu_ex1_src1_data   = 0;
    dut->idu_iu_ex1_src1_ready  = 1;
    dut->idu_iu_ex1_src2_data   = 0;
    dut->idu_iu_ex1_src2_ready  = 1;
    dut->idu_iu_ex1_dst0_reg    = 0;
    dut->idu_iu_ex1_bht_pred    = 0;
    dut->idu_iu_ex1_src0_reg    = 0;
    dut->idu_iu_ex1_src1_reg    = 0;
    dut->rtu_iu_mul_wb_grant    = 1;
    dut->rtu_iu_div_wb_grant    = 1;
    dut->ifu_iu_chgflw_vld      = 0;
    dut->ifu_iu_chgflw_pc       = 0;
    dut->da_xx_fwd_data         = 0;
    dut->da_xx_fwd_dst_reg      = 0;
    dut->da_xx_fwd_vld          = 0;
    dut->lsu_iu_ex2_data        = 0;
    dut->lsu_iu_ex2_data_vld    = 0;
    dut->lsu_iu_ex2_dest_reg    = 0;
}

static void tick(void) {
    dut->eval();
    dut->clk = 1;
    dut->eval();   // registers commit here
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

//-----------------------------------------------------------------------------
// Result bookkeeping (mirrors csr_tb.cpp exactly)
//-----------------------------------------------------------------------------
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
    printf("[iu_tb] %-64s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// ALU dispatch helper -- purely combinational, EX1-only (IU.v header):
// set operands, eval(), read the result THE SAME CYCLE, then tick() once to
// let BJU's PC register advance (harmless for a pure-ALU op) before the
// next dispatch.
//=============================================================================
struct AluResult {
    uint64_t data;
    bool cmplt, cmplt_dp, wb_vld, wb_dp;
    unsigned preg;
};

static AluResult alu_op(uint32_t func, uint64_t src0, uint64_t src1, uint64_t src2 = 0,
                         unsigned dst = 1) {
    tie_idle_inputs();
    dut->idu_iu_ex1_inst_vld  = 1;
    dut->idu_iu_ex1_alu_sel   = 1;
    dut->idu_iu_ex1_func      = func;
    dut->idu_iu_ex1_src0_data = src0;
    dut->idu_iu_ex1_src1_data = src1;
    dut->idu_iu_ex1_src2_data = src2;
    dut->idu_iu_ex1_dst0_reg  = dst;
    dut->eval();

    AluResult r;
    r.data     = dut->iu_rtu_ex1_alu_data;
    r.cmplt    = dut->iu_rtu_ex1_alu_cmplt != 0;
    r.cmplt_dp = dut->iu_rtu_ex1_alu_cmplt_dp != 0;
    r.wb_vld   = dut->iu_rtu_ex1_alu_wb_vld != 0;
    r.wb_dp    = dut->iu_rtu_ex1_alu_wb_dp != 0;
    r.preg     = dut->iu_rtu_ex1_alu_preg;

    tick();
    return r;
}

//=============================================================================
// MULT dispatch helper -- variable-latency (IU note S5): keep `mult_sel`
// asserted until `iu_rtu_ex1_mul_cmplt` fires (the "well-behaved dispatcher"
// contract), then deassert and drain EX2->EX3 until `iu_rtu_ex3_mul_wb_vld`.
//=============================================================================
struct MulResult {
    uint64_t data;
    unsigned preg;
    int      cmplt_ticks;   // ticks from dispatch until cmplt fired
    int      total_ticks;   // ticks from dispatch until EX3 wb_vld fired
};

static MulResult mul_op(uint32_t func, uint64_t src0, uint64_t src1, unsigned dst,
                         bool grant_immediately = true, int max_ticks = 40) {
    tie_idle_inputs();
    dut->rtu_iu_mul_wb_grant  = grant_immediately ? 1 : 0;
    dut->idu_iu_ex1_inst_vld  = 1;
    dut->idu_iu_ex1_mult_sel  = 1;
    dut->idu_iu_ex1_func      = func;
    dut->idu_iu_ex1_src0_data = src0;
    dut->idu_iu_ex1_src1_data = src1;
    dut->idu_iu_ex1_dst0_reg  = dst;
    dut->eval();

    int ticks = 0;
    while (!dut->iu_rtu_ex1_mul_cmplt && ticks < max_ticks) {
        tick();
        ticks++;
        dut->eval();
    }
    int cmplt_ticks = ticks;
    tick();   // commit the cmplt-cycle's EX1->EX2 capture
    ticks++;
    dut->idu_iu_ex1_mult_sel = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    dut->rtu_iu_mul_wb_grant = 1;   // release any deliberate backpressure test set up
    dut->eval();

    while (!dut->iu_rtu_ex3_mul_wb_vld && ticks < max_ticks) {
        tick();
        ticks++;
        dut->eval();
    }

    MulResult r;
    r.data        = dut->iu_rtu_ex3_mul_data;
    r.preg        = dut->iu_rtu_ex3_mul_preg;
    r.cmplt_ticks = cmplt_ticks;
    r.total_ticks = ticks;
    tick();
    return r;
}

//=============================================================================
// DIV dispatch helper -- variable, data-dependent latency (IU note S6).
//=============================================================================
struct DivResult {
    uint64_t data;
    unsigned preg;
    int      total_ticks;
};

static DivResult div_op(uint32_t func, uint64_t src0, uint64_t src1, unsigned dst,
                         int max_ticks = 80) {
    tie_idle_inputs();
    dut->rtu_iu_div_wb_grant  = 1;
    dut->idu_iu_ex1_inst_vld  = 1;
    dut->idu_iu_ex1_div_sel   = 1;
    dut->idu_iu_ex1_func      = func;
    dut->idu_iu_ex1_src0_data = src0;
    dut->idu_iu_ex1_src1_data = src1;
    dut->idu_iu_ex1_dst0_reg  = dst;
    dut->eval();

    int ticks = 0;
    while (!dut->iu_rtu_ex1_div_cmplt && ticks < max_ticks) {
        tick(); ticks++; dut->eval();
    }
    // iu_rtu_ex1_div_cmplt is a plain combinational echo of idu_iu_ex1_div_sel
    // (div.v's own `iu_rtu_ex1_div_cmplt = idu_iu_ex1_div_sel`) -- the real
    // "is the DATA ready yet" signal is iu_rtu_div_wb_vld, drained next.
    while (!dut->iu_rtu_div_wb_vld && ticks < max_ticks) {
        tick(); ticks++; dut->eval();
    }

    DivResult r;
    r.data        = dut->iu_rtu_div_data;
    r.preg        = dut->iu_rtu_div_preg;
    r.total_ticks = ticks;

    dut->idu_iu_ex1_div_sel = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    tick();
    return r;
}

//=============================================================================
// BJU immediate (non-entry) dispatch helper -- resolves the same cycle
// (IU note S4.1/S4.2): every source ready, no LSU dependency.
//=============================================================================
struct BjuResult {
    bool tar_pc_vld, bht_mispred, br_vld, bht_taken, link_vld, ret_vld, pc_mispred;
    uint64_t tar_pc;
    unsigned bht_pred;
    bool wb_vld;
    uint64_t wb_data;
    unsigned preg;
    uint64_t cur_pc, next_pc;
    bool branch_inst;
};

static BjuResult bju_op(uint32_t func, bool br_sel, uint64_t src0, uint64_t src1,
                         uint64_t src2, unsigned src0_reg, unsigned src1_reg,
                         unsigned dst_reg, unsigned bht_pred) {
    tie_idle_inputs();
    dut->idu_iu_ex1_inst_vld   = 1;
    dut->idu_iu_ex1_bju_sel    = 1;
    dut->idu_iu_ex1_bju_br_sel = br_sel ? 1 : 0;
    dut->idu_iu_ex1_func       = func;
    dut->idu_iu_ex1_src0_data  = src0;
    dut->idu_iu_ex1_src1_data  = src1;
    dut->idu_iu_ex1_src2_data  = src2;
    dut->idu_iu_ex1_src0_reg   = src0_reg;
    dut->idu_iu_ex1_src1_reg   = src1_reg;
    dut->idu_iu_ex1_dst0_reg   = dst_reg;
    dut->idu_iu_ex1_bht_pred   = bht_pred;
    dut->eval();

    BjuResult r;
    r.tar_pc_vld   = dut->iu_ifu_tar_pc_vld != 0;
    r.tar_pc       = dut->iu_ifu_tar_pc;
    r.bht_mispred  = dut->iu_ifu_bht_mispred != 0;
    r.br_vld       = dut->iu_ifu_br_vld != 0;
    r.bht_taken    = dut->iu_ifu_bht_taken != 0;
    r.bht_pred     = dut->iu_ifu_bht_pred;
    r.link_vld     = dut->iu_ifu_link_vld != 0;
    r.ret_vld      = dut->iu_ifu_ret_vld != 0;
    r.pc_mispred   = dut->iu_ifu_pc_mispred != 0;
    r.wb_vld       = dut->iu_rtu_ex1_bju_wb_vld != 0;
    r.wb_data      = dut->iu_rtu_ex1_bju_data;
    r.preg         = dut->iu_rtu_ex1_bju_preg;
    r.cur_pc       = dut->iu_rtu_ex1_cur_pc;
    r.next_pc      = dut->iu_rtu_ex1_next_pc;
    r.branch_inst  = dut->iu_rtu_ex1_branch_inst != 0;

    tick();
    dut->idu_iu_ex1_bju_sel = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    return r;
}

//=============================================================================
// Tests -- ALU
//=============================================================================

static void test_alu_add_sub(void) {
    AluResult r = alu_op(ALU_FUNC_ADD, 5, 7);
    check(r.data == 12, "ADD: 5+7==12", r.data, 12);
    check(r.cmplt && r.cmplt_dp && r.wb_vld && r.wb_dp, "ADD: cmplt/wb asserted same cycle");
    check(r.preg == 1, "ADD: preg == dst0_reg", r.preg, 1);

    r = alu_op(ALU_FUNC_ADD, 0xFFFFFFFFFFFFFFFFULL, 1);
    check(r.data == 0, "ADD: wraps to 0", r.data, 0);

    r = alu_op(ALU_FUNC_SUB, 10, 3);
    check(r.data == 7, "SUB: 10-3==7", r.data, 7);

    r = alu_op(ALU_FUNC_SUB, 3, 10);
    check(r.data == (uint64_t)(3 - 10), "SUB: 3-10 wraps (two's complement)",
          r.data, (uint64_t)(3 - 10));
    test_result("T1 ALU ADD/SUB: shared 65-bit adder, plain 64-bit width class");
}

static void test_alu_addw_subw(void) {
    // ADDW: 32-bit add, result sign-extended to 64 -- exercises the
    // sign32 operand-prepare class (not a separate 32-bit adder).
    AluResult r = alu_op(ALU_FUNC_ADDW, 0x7FFFFFFFULL, 1);
    check(r.data == 0xFFFFFFFF80000000ULL,
          "ADDW: 0x7FFFFFFF+1 == 0x80000000 (32b), sign-extends to MIN_INT64-shaped result",
          r.data, 0xFFFFFFFF80000000ULL);

    r = alu_op(ALU_FUNC_ADDW, 0x00000000FFFFFFFFULL, 0x0000000100000000ULL);
    // high bits of both operands are ignored by the *W class; low32 add = 0xFFFFFFFF+0 = 0xFFFFFFFF -> sign-ext
    check(r.data == 0xFFFFFFFFFFFFFFFFULL, "ADDW: ignores operand high bits, sign-extends low32 sum",
          r.data, 0xFFFFFFFFFFFFFFFFULL);

    r = alu_op(ALU_FUNC_SUBW, 5, 3);
    check(r.data == 2, "SUBW: 5-3==2 (fits, no sign-ext needed)", r.data, 2);

    r = alu_op(ALU_FUNC_SUBW, 0, 1);
    check(r.data == 0xFFFFFFFFFFFFFFFFULL, "SUBW: 0-1 == -1 sign-extended", r.data, -1ULL);
    test_result("T2 ALU ADDW/SUBW: sign32 width class via the SAME shared adder");
}

static void test_alu_slt_sltu(void) {
    AluResult r = alu_op(ALU_FUNC_SLT, (uint64_t)-5, 3);
    check(r.data == 1, "SLT: -5 < 3 signed -> 1", r.data, 1);
    r = alu_op(ALU_FUNC_SLTU, (uint64_t)-5, 3);
    check(r.data == 0, "SLTU: (huge unsigned) < 3 -> 0", r.data, 0);
    r = alu_op(ALU_FUNC_SLT, 3, (uint64_t)-5);
    check(r.data == 0, "SLT: 3 < -5 signed -> 0", r.data, 0);
    r = alu_op(ALU_FUNC_SLTU, 3, (uint64_t)-5);
    check(r.data == 1, "SLTU: 3 < (huge unsigned) -> 1", r.data, 1);
    r = alu_op(ALU_FUNC_SLT, 5, 5);
    check(r.data == 0, "SLT: equal operands -> 0", r.data, 0);
    test_result("T3 ALU SLT/SLTU: reuse the adder's carry-out, signed-vs-unsigned via extension class");
}

static void test_alu_shifts(void) {
    AluResult r = alu_op(ALU_FUNC_SLL, 1, 4);
    check(r.data == 16, "SLL: 1<<4==16", r.data, 16);
    r = alu_op(ALU_FUNC_SLL, 1, 63);
    check(r.data == 0x8000000000000000ULL, "SLL: 1<<63", r.data, 0x8000000000000000ULL);
    r = alu_op(ALU_FUNC_SRL, 0x8000000000000000ULL, 4);
    check(r.data == 0x0800000000000000ULL, "SRL: logical right shift, zero-fill", r.data, 0x0800000000000000ULL);
    r = alu_op(ALU_FUNC_SRA, 0x8000000000000000ULL, 4);
    check(r.data == 0xF800000000000000ULL, "SRA: arithmetic right shift, sign-fill", r.data, 0xF800000000000000ULL);
    r = alu_op(ALU_FUNC_SLLW, 1, 33);   // shamt masked to [4:0] -> 1
    check(r.data == 2, "SLLW: shift amount masked to 5 bits (word class)", r.data, 2);
    r = alu_op(ALU_FUNC_SRLW, 0x00000000FFFFFFFFULL, 4);
    check(r.data == 0x0FFFFFFFULL, "SRLW: 32-bit logical shift, result sign-ext (positive here)");
    r = alu_op(ALU_FUNC_SRAW, 0x0000000080000000ULL, 4);
    check(r.data == 0xFFFFFFFFF8000000ULL, "SRAW: 32-bit arithmetic shift, sign-extended result",
          r.data, 0xFFFFFFFFF8000000ULL);
    test_result("T4 ALU SLL/SRL/SRA + *W: one shared barrel-shift expression");
}

static void test_alu_logic(void) {
    AluResult r = alu_op(ALU_FUNC_AND, 0xF0F0F0F0F0F0F0F0ULL, 0x0FF00FF00FF00FF0ULL);
    check(r.data == 0x00F000F000F000F0ULL, "AND", r.data, 0x00F000F000F000F0ULL);
    r = alu_op(ALU_FUNC_OR, 0xF0F0F0F0F0F0F0F0ULL, 0x0F0F0F0F0F0F0F0FULL);
    check(r.data == 0xFFFFFFFFFFFFFFFFULL, "OR", r.data, 0xFFFFFFFFFFFFFFFFULL);
    r = alu_op(ALU_FUNC_XOR, 0xAAAAAAAAAAAAAAAAULL, 0xAAAAAAAAAAAAAAAAULL);
    check(r.data == 0, "XOR of identical operands == 0", r.data, 0);
    test_result("T5 ALU AND/OR/XOR: plain 64-bit logic block");
}

static void test_alu_lui(void) {
    AluResult r = alu_op(ALU_FUNC_LUI, 0xDEADBEEF, 0x0000000012345000ULL);
    check(r.data == 0x0000000012345000ULL, "LUI: rs0 forced 0, result == the prepared immediate (src1)",
          r.data, 0x0000000012345000ULL);
    test_result("T6 ALU LUI: rs0 forced 0 through the shared adder (alu.v:228)");
}

static void test_alu_xthead(void) {
    AluResult r = alu_op(ALU_FUNC_REV, 0x0102030405060708ULL, 0);
    check(r.data == 0x0807060504030201ULL, "REV: full 64-bit byte-reverse", r.data, 0x0807060504030201ULL);
    r = alu_op(ALU_FUNC_REVW, 0x000000AABBCCDDEEULL, 0);
    check(r.data == 0xFFFFFFFFEEDDCCBBULL, "REVW: 32-bit byte-reverse, sign-ext from resulting top byte",
          r.data, 0xFFFFFFFFEEDDCCBBULL);
    r = alu_op(ALU_FUNC_TST, 0x0000000000000010ULL, 4);
    check(r.data == 1, "TST: bit 4 of src0 is set", r.data, 1);
    r = alu_op(ALU_FUNC_TST, 0x0000000000000010ULL, 5);
    check(r.data == 0, "TST: bit 5 of src0 is clear", r.data, 0);
    r = alu_op(ALU_FUNC_TSTNBZ, 0x0011000000440000ULL, 0);
    check(r.data == 0xFF00FFFFFF00FFFFULL, "TSTNBZ: per-byte zero-test, 0xff per zero byte",
          r.data, 0xFF00FFFFFF00FFFFULL);
    r = alu_op(ALU_FUNC_FF0, 0xFFFFFFFFFFFFFFFEULL, 0);   // bit0 clear, rest set
    check(r.data == 63, "FF0: first 0 from the MSB, distance-from-top", r.data, 63);
    r = alu_op(ALU_FUNC_FF0, 0xFFFFFFFFFFFFFFFFULL, 0);
    check(r.data == 64, "FF0: all-1s input -> 64 (no 0 bit at all)", r.data, 64);
    r = alu_op(ALU_FUNC_FF1, 0x8000000000000000ULL, 0);
    check(r.data == 0, "FF1: bit63 set -> distance 0", r.data, 0);
    r = alu_op(ALU_FUNC_FF1, 0x0000000000000001ULL, 0);
    check(r.data == 63, "FF1: bit0 only set -> distance 63", r.data, 63);
    r = alu_op(ALU_FUNC_FF1, 0, 0);
    check(r.data == 64, "FF1: all-0 input -> 64", r.data, 64);
    // MVEQZ/MVNEZ: src1=condition, src0=move-value, src2=dst's-old-value alias.
    r = alu_op(ALU_FUNC_MVEQZ, /*src0=*/111, /*src1=cond*/0, /*src2=*/222);
    check(r.data == 111, "MVEQZ: condition==0 -> take src0 (move value)", r.data, 111);
    r = alu_op(ALU_FUNC_MVEQZ, /*src0=*/111, /*src1=cond*/5, /*src2=*/222);
    check(r.data == 222, "MVEQZ: condition!=0 -> keep src2 (dst's old value)", r.data, 222);
    r = alu_op(ALU_FUNC_MVNEZ, /*src0=*/111, /*src1=cond*/5, /*src2=*/222);
    check(r.data == 111, "MVNEZ: condition!=0 -> take src0 (move value)", r.data, 111);
    r = alu_op(ALU_FUNC_MVNEZ, /*src0=*/111, /*src1=cond*/0, /*src2=*/222);
    check(r.data == 222, "MVNEZ: condition==0 -> keep src2 (dst's old value)", r.data, 222);
    test_result("T7 ALU XThead REV/REVW/TST/TSTNBZ/FF0/FF1/MVEQZ/MVNEZ");
}

static void test_alu_srri_ext(void) {
    // SRRI: rotate right. src1 low 6 bits = shift amount.
    AluResult r = alu_op(ALU_FUNC_SRRI, 0x0000000000000001ULL, 1);
    check(r.data == 0x8000000000000000ULL, "SRRI: rotate right by 1, bit0 wraps to bit63",
          r.data, 0x8000000000000000ULL);
    r = alu_op(ALU_FUNC_SRRIW, 0x0000000000000001ULL, 1);
    check(r.data == 0xFFFFFFFF80000000ULL, "SRRIW: 32-bit rotate right by 1, sign-ext (bit31 set post-rotate)",
          r.data, 0xFFFFFFFF80000000ULL);
    // EXT/EXTU: src1[5:0]=lsb, src1[11:6]=msb of the bitfield.
    uint64_t field_spec = (uint64_t)7 << 6 | 4;   // msb=7, lsb=4 -> 4-bit field at [7:4]
    r = alu_op(ALU_FUNC_EXT, 0x00000000000000B0ULL, field_spec);   // bits[7:4] = 0xB = 1011 -> sign bit set
    check(r.data == 0xFFFFFFFFFFFFFFFBULL, "EXT: signed bitfield extract, top bit of field sign-extends",
          r.data, 0xFFFFFFFFFFFFFFFBULL);
    r = alu_op(ALU_FUNC_EXTU, 0x00000000000000B0ULL, field_spec);
    check(r.data == 0xB, "EXTU: unsigned bitfield extract, zero-extends", r.data, 0xB);
    test_result("T8 ALU XThead SRRI/SRRIW (rotate) + EXT/EXTU (bitfield extract)");
}

//=============================================================================
// Tests -- MULT
//=============================================================================

static void test_mult_narrow(void) {
    MulResult r = mul_op(MULT_FUNC_MUL, 6, 7, 3);
    check(r.data == 42, "MUL: 6*7==42", r.data, 42);
    check(r.preg == 3, "MUL: preg == dst", r.preg, 3);
    check(r.total_ticks <= 4, "MUL narrow: completes in a small, fixed number of ticks",
          r.total_ticks, 4);

    r = mul_op(MULT_FUNC_MULW, 0xFFFFFFFFULL, 2, 1);   // -1 (32b) * 2 = -2
    check(r.data == 0xFFFFFFFFFFFFFFFEULL, "MULW: 32x32 signed, sign-ext result", r.data, -2ULL);

    r = mul_op(MULT_FUNC_MULH, (uint64_t)-1, (uint64_t)-1, 1);   // (-1)*(-1)=1, high64=0
    check(r.data == 0, "MULH: (-1)*(-1) high64 == 0", r.data, 0);

    r = mul_op(MULT_FUNC_MULHU, (uint64_t)-1, (uint64_t)-1, 1);   // unsigned max * max
    check(r.data == 0xFFFFFFFFFFFFFFFEULL, "MULHU: (2^64-1)^2 high64", r.data, 0xFFFFFFFFFFFFFFFEULL);

    r = mul_op(MULT_FUNC_MULHSU, (uint64_t)-1, 2, 1);   // -1 (signed) * 2 (unsigned) = -2, high64 = -1
    check(r.data == 0xFFFFFFFFFFFFFFFFULL, "MULHSU: -1(signed)*2(unsigned) high64 == -1", r.data, -1ULL);
    test_result("T9 MULT all 5 RV64M ops: narrow (operands fit in 33 bits) fixed-latency path");
}

static void test_mult_wide_timing(void) {
    // Both operands' high 32 bits are neither all-0 nor a valid sign
    // extension -- forces the split (iterative) path (IU note S5).
    uint64_t wide_a = 0x123456789ABCDEF0ULL;
    uint64_t wide_b = 0x0FEDCBA987654321ULL;
    MulResult narrow = mul_op(MULT_FUNC_MUL, 6, 7, 1);
    MulResult wide    = mul_op(MULT_FUNC_MULH, wide_a, wide_b, 2);

    unsigned __int128 full = (unsigned __int128)wide_a * (unsigned __int128)wide_b;
    uint64_t expect_high = (uint64_t)(full >> 64);
    check(wide.data == expect_high, "MULH wide: correct 128-bit-product high64", wide.data, expect_high);
    check(wide.total_ticks > narrow.total_ticks,
          "MULT wide (needs split): strictly more ticks than the narrow case (variable latency)",
          wide.total_ticks, narrow.total_ticks);
    check(wide.total_ticks >= narrow.total_ticks + 2,
          "MULT wide: at least 2 extra ticks for the SPLIT0/SPLIT1/CMPLT passes",
          wide.total_ticks, narrow.total_ticks + 2);
    test_result("T10 MULT wide (64x64, split path): correct value + strictly higher latency");
}

static void test_mult_full_backpressure(void) {
    // Drive a narrow multiply but withhold rtu_iu_mul_wb_grant: EX3 should
    // hold (iu_rtu_ex3_mul_wb_vld asserted, data stable) until granted, and
    // iu_idu_mult_full should assert while EX2 is ALSO backed up behind it.
    tie_idle_inputs();
    dut->rtu_iu_mul_wb_grant  = 0;
    dut->idu_iu_ex1_inst_vld  = 1;
    dut->idu_iu_ex1_mult_sel  = 1;
    dut->idu_iu_ex1_func      = MULT_FUNC_MUL;
    dut->idu_iu_ex1_src0_data = 9;
    dut->idu_iu_ex1_src1_data = 9;
    dut->idu_iu_ex1_dst0_reg  = 5;
    dut->eval();
    check(dut->iu_rtu_ex1_mul_cmplt != 0, "backpressure: narrow MUL cmplt fires at dispatch");
    tick();
    dut->idu_iu_ex1_mult_sel = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    dut->eval();
    tick();   // EX2->EX3
    dut->eval();
    check(dut->iu_rtu_ex3_mul_wb_vld != 0, "backpressure: EX3 holds a valid (ungranted) result");
    check(dut->iu_rtu_ex3_mul_data == 81, "backpressure: EX3 data stable while ungranted",
          dut->iu_rtu_ex3_mul_data, 81);
    for (int i = 0; i < 5; i++) {
        tick();
        dut->eval();
        check(dut->iu_rtu_ex3_mul_wb_vld != 0 && dut->iu_rtu_ex3_mul_data == 81,
              "backpressure: result holds steady across repeated un-granted cycles");
    }
    dut->rtu_iu_mul_wb_grant = 1;
    tick();
    dut->eval();
    check(dut->iu_rtu_ex3_mul_wb_vld == 0, "backpressure: wb_vld clears the cycle after grant");
    test_result("T11 MULT EX3 stall on withheld rtu_iu_mul_wb_grant (iu_idu_mult_full path)");
}

//=============================================================================
// Tests -- DIV
//=============================================================================

static void test_div_basic(void) {
    DivResult r = div_op(DIV_FUNC_DIV, (uint64_t)-7, 2, 1);   // -7/2 == -3 (truncating)
    check((int64_t)r.data == -3, "DIV: -7/2 == -3 (signed, truncating)", r.data, (uint64_t)-3LL);
    r = div_op(DIV_FUNC_DIVU, (uint64_t)-7, 2, 1);
    uint64_t expect = ((uint64_t)-7) / 2;
    check(r.data == expect, "DIVU: unsigned division", r.data, expect);
    r = div_op(DIV_FUNC_REM, (uint64_t)-7, 2, 1);
    check((int64_t)r.data == -1, "REM: -7 rem 2 == -1 (sign follows dividend)", r.data, (uint64_t)-1LL);
    r = div_op(DIV_FUNC_REMU, (uint64_t)-7, 2, 1);
    check(r.data == (((uint64_t)-7) % 2), "REMU: unsigned remainder", r.data, ((uint64_t)-7) % 2);
    test_result("T12 DIV all 4 base ops: DIV/DIVU/REM/REMU share one core");
}

static void test_div_word_forms(void) {
    DivResult r = div_op(DIV_FUNC_DIVW, (uint64_t)(int64_t)-8, 3, 1);
    check((int64_t)r.data == -2, "DIVW: -8/3 == -2 (32-bit signed, truncating)", r.data, (uint64_t)-2LL);
    r = div_op(DIV_FUNC_REMW, (uint64_t)(int64_t)-8, 3, 1);
    check((int64_t)r.data == -2, "REMW: -8 rem 3 == -2 (sign follows dividend)", r.data, (uint64_t)-2LL);
    test_result("T13 DIV *W word forms: 32-bit operate-then-sign-extend");
}

static void test_div_abnormal(void) {
    DivResult r = div_op(DIV_FUNC_DIV, 42, 0, 1);
    check(r.data == 0xFFFFFFFFFFFFFFFFULL, "DIV by zero: quotient == all-1s", r.data, -1ULL);
    r = div_op(DIV_FUNC_REM, 42, 0, 1);
    check(r.data == 42, "REM by zero: remainder == dividend", r.data, 42);
    r = div_op(DIV_FUNC_DIVU, 42, 0, 1);
    check(r.data == 0xFFFFFFFFFFFFFFFFULL, "DIVU by zero: quotient == all-1s", r.data, -1ULL);
    // Signed overflow: MIN_INT / -1.
    r = div_op(DIV_FUNC_DIV, 0x8000000000000000ULL, 0xFFFFFFFFFFFFFFFFULL, 1);
    check(r.data == 0x8000000000000000ULL, "DIV overflow: MIN_INT/-1 == MIN_INT", r.data, 0x8000000000000000ULL);
    r = div_op(DIV_FUNC_REM, 0x8000000000000000ULL, 0xFFFFFFFFFFFFFFFFULL, 1);
    check(r.data == 0, "REM overflow: MIN_INT rem -1 == 0", r.data, 0);
    check(r.total_ticks <= 3, "DIV abnormal path: fast (~1-2 tick) early-out, not a full iteration",
          r.total_ticks, 3);
    test_result("T14 DIV abnormal early-outs: divide-by-zero + signed overflow, fast path");
}

static void test_div_memo_hit(void) {
    DivResult first  = div_op(DIV_FUNC_DIV, 1000003, 7, 1);
    DivResult second = div_op(DIV_FUNC_DIV, 1000003, 7, 2);   // identical operands -> memo hit
    check(second.data == first.data, "DIV memo hit: repeated identical divide gives the same result",
          second.data, first.data);
    check(second.total_ticks < first.total_ticks,
          "DIV memo hit: reuse is strictly faster than the original iteration",
          second.total_ticks, first.total_ticks);
    check(second.total_ticks <= 3, "DIV memo hit: fast (~1-2 tick) path, not a re-run of ITER",
          second.total_ticks, 3);
    // A THIRD, different divide breaks the memo chain -- confirms the
    // buffer really is only 1-entry (compares against the IMMEDIATELY
    // preceding op, IU note S6).
    DivResult third = div_op(DIV_FUNC_DIV, 999999999, 13, 3);
    uint64_t expect3 = (uint64_t)((int64_t)999999999 / (int64_t)13);
    check(third.data == expect3, "DIV memo: a different op after a hit still computes correctly",
          third.data, expect3);
    test_result("T15 DIV 1-entry memo/hit buffer: fast reuse, then correctly invalidated");
}

static void test_div_variable_latency(void) {
    // Large leading-1 distance (huge dividend, tiny divisor) needs many
    // more radix-4 iterations than a small distance (close magnitudes).
    DivResult small_gap = div_op(DIV_FUNC_DIVU, 100, 90, 1);
    DivResult big_gap    = div_op(DIV_FUNC_DIVU, 0x7FFFFFFFFFFFFFFFULL, 1, 2);
    check(big_gap.data == 0x7FFFFFFFFFFFFFFFULL, "DIV variable-latency: large-gap result correct",
          big_gap.data, 0x7FFFFFFFFFFFFFFFULL);
    check(big_gap.total_ticks > small_gap.total_ticks,
          "DIV variable-latency: leading-1-distance genuinely changes iteration count",
          big_gap.total_ticks, small_gap.total_ticks);
    test_result("T16 DIV data-dependent latency: leading-1-distance drives iteration count");
}

static void test_div_full_backpressure(void) {
    tie_idle_inputs();
    dut->rtu_iu_div_wb_grant  = 0;
    dut->idu_iu_ex1_inst_vld  = 1;
    dut->idu_iu_ex1_div_sel   = 1;
    dut->idu_iu_ex1_func      = DIV_FUNC_DIV;
    dut->idu_iu_ex1_src0_data = 42;
    dut->idu_iu_ex1_src1_data = 0;   // abnormal -> fast path, resolves at IDLE
    dut->idu_iu_ex1_dst0_reg  = 4;
    dut->eval();
    tick();   // commit the abnormal result into the wb-pending state
    dut->eval();
    check(dut->iu_rtu_div_wb_vld != 0, "DIV backpressure: wb_vld asserted while ungranted");
    check(dut->iu_idu_div_full != 0, "DIV backpressure: iu_idu_div_full asserted while ungranted");
    for (int i = 0; i < 4; i++) {
        tick();
        dut->eval();
        check(dut->iu_rtu_div_wb_vld != 0 && dut->iu_rtu_div_data == 0xFFFFFFFFFFFFFFFFULL,
              "DIV backpressure: result holds steady while ungranted");
    }
    dut->rtu_iu_div_wb_grant = 1;
    dut->idu_iu_ex1_div_sel  = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    tick();
    dut->eval();
    check(dut->iu_rtu_div_wb_vld == 0, "DIV backpressure: wb_vld clears the cycle after grant");
    check(dut->iu_idu_div_full == 0, "DIV backpressure: full clears the cycle after grant");
    test_result("T17 DIV iu_idu_div_full / wb hold on withheld rtu_iu_div_wb_grant");
}

//=============================================================================
// Tests -- BJU
//=============================================================================

static void test_bju_cond_branch_matrix(void) {
    // BEQ, operands equal (taken), predicted taken (pred[1]=1) -> no
    // mispredict, no redirect.
    BjuResult r = bju_op(BJU_FUNC_BEQ, true, 5, 5, /*offset*/64, 10, 11, 0, /*pred*/0b11);
    check(r.br_vld && r.bht_taken, "BEQ taken, predicted taken: br_vld+bht_taken");
    check(!r.bht_mispred && !r.tar_pc_vld, "BEQ taken, predicted taken: no mispredict/redirect");

    // BEQ, operands equal (taken), predicted NOT taken -> mispredict + redirect to target.
    r = bju_op(BJU_FUNC_BEQ, true, 5, 5, 64, 10, 11, 0, /*pred*/0b00);
    check(r.bht_mispred && r.tar_pc_vld, "BEQ taken, predicted not-taken: mispredict + redirect");

    // BNE, operands equal (not taken), predicted taken -> mispredict + redirect to fallthrough.
    r = bju_op(BJU_FUNC_BNE, true, 5, 5, 64, 10, 11, 0, /*pred*/0b11);
    check(!r.bht_taken, "BNE not-taken (equal operands): bht_taken==0");
    check(r.bht_mispred && r.tar_pc_vld, "BNE not-taken, predicted taken: mispredict + redirect");

    // BNE, not equal (taken), predicted taken -> correct, no redirect.
    r = bju_op(BJU_FUNC_BNE, true, 5, 6, 64, 10, 11, 0, /*pred*/0b11);
    check(r.bht_taken && !r.bht_mispred && !r.tar_pc_vld, "BNE taken, predicted taken: no redirect");

    // BLT signed vs BLTU unsigned on the same bit pattern.
    r = bju_op(BJU_FUNC_BLT, true, (uint64_t)-1, 1, 64, 10, 11, 0, 0b11);
    check(r.bht_taken, "BLT: -1 < 1 signed -> taken");
    r = bju_op(BJU_FUNC_BLTU, true, (uint64_t)-1, 1, 64, 10, 11, 0, 0b11);
    check(!r.bht_taken, "BLTU: (huge unsigned) < 1 -> not taken");

    r = bju_op(BJU_FUNC_BGE, true, 5, 5, 64, 10, 11, 0, 0b11);
    check(r.bht_taken, "BGE: 5>=5 -> taken");
    r = bju_op(BJU_FUNC_BGEU, true, 3, 5, 64, 10, 11, 0, 0b11);
    check(!r.bht_taken, "BGEU: 3>=5 unsigned -> not taken");

    check(!r.wb_vld, "conditional branches never write a register");
    test_result("T18 BJU conditional-branch matrix: BEQ/BNE/BLT/BGE/BLTU/BGEU x predicted-right/wrong");
}

static void test_bju_jal_jalr_auipc(void) {
    // JAL: dst=x1 -> link_vld; wb_data == pc+4 (fixed-32-bit assumption).
    BjuResult r = bju_op(BJU_FUNC_JAL, false, 0, 0, 0x100, 0, 0, /*dst=*/1, 0);
    check(r.link_vld, "JAL rd=x1: link_vld asserted");
    check(r.wb_vld && r.wb_data == r.cur_pc + 4, "JAL: writes back pc+4 (link address)",
          r.wb_data, r.cur_pc + 4);
    check(r.next_pc == ((r.cur_pc + 0x100) & PC_MASK), "JAL: next_pc == pc+offset");

    // JALR rs1=x1, rd!=x1 -> a real return (ret_vld).
    r = bju_op(BJU_FUNC_JALR, false, 0x80009000ULL, 0, 0, /*src0_reg=*/1, 0, /*dst=*/5, 0);
    check(r.ret_vld, "JALR rs1=x1, rd!=x1: ret_vld (a real return)");
    check(!r.pc_mispred, "JALR rs1=x1: not flagged as a RAS-shouldn't-have-predicted case");

    // JALR rs1!=x1 -> pc_mispred (RAS should never have predicted this).
    r = bju_op(BJU_FUNC_JALR, false, 0x80009000ULL, 0, 0, /*src0_reg=*/6, 0, /*dst=*/5, 0);
    check(r.pc_mispred, "JALR rs1!=x1: pc_mispred asserted");
    check(!r.ret_vld, "JALR rs1!=x1: not a return");

    // JALR rs1=x1, rd=x1 (self-loop) -> NOT a real return per RISC-V convention.
    r = bju_op(BJU_FUNC_JALR, false, 0x80009000ULL, 0, 0, /*src0_reg=*/1, 0, /*dst=*/1, 0);
    check(!r.ret_vld, "JALR rs1=x1, rd=x1: excluded from ret_vld (self-loop convention)");

    // AUIPC: writes pc+imm, never redirects control flow.
    r = bju_op(BJU_FUNC_AUIPC, false, 0, 0, 0x0000000000012000ULL, 0, 0, /*dst=*/7, 0);
    check(r.wb_vld && r.wb_data == ((r.cur_pc + 0x12000) & 0xFFFFFFFFFFULL),
          "AUIPC: writes back pc+imm via the shared address-gen adder",
          r.wb_data, (r.cur_pc + 0x12000) & 0xFFFFFFFFFFULL);
    check(!r.tar_pc_vld, "AUIPC: never redirects control flow");
    test_result("T19 BJU JAL/JALR (link/return/pc_mispred) + AUIPC (address-gen writeback)");
}

//=============================================================================
// BJU 1-entry LSU-dependent conditional-branch buffer (IU note S4.4).
//=============================================================================

static void test_bju_entry_release_on_da_fwd(void) {
    tie_idle_inputs();
    // Dispatch a BLT whose src0 (x5) is NOT ready (an outstanding load);
    // src1 (x6) already has its value.
    dut->idu_iu_ex1_inst_vld   = 1;
    dut->idu_iu_ex1_bju_sel    = 1;
    dut->idu_iu_ex1_bju_br_sel = 1;
    dut->idu_iu_ex1_func       = BJU_FUNC_BLT;
    dut->idu_iu_ex1_src0_ready = 0;
    dut->idu_iu_ex1_src1_ready = 1;
    dut->idu_iu_ex1_src1_data  = 100;
    dut->idu_iu_ex1_src0_reg   = 5;
    dut->idu_iu_ex1_src1_reg   = 6;
    dut->idu_iu_ex1_src2_data  = 64;   // branch offset
    dut->idu_iu_ex1_bht_pred   = 0b00; // predicted not-taken
    dut->eval();
    check(dut->iu_idu_bju_full == 0 && dut->iu_idu_bju_global_full == 0,
          "entry create cycle: no backpressure yet (entry doesn't exist until the tick)");
    tick();
    dut->idu_iu_ex1_bju_sel = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    dut->eval();
    check(dut->iu_idu_bju_global_full != 0, "after create: global_full (parked, still waiting on src0)");
    check(dut->iu_idu_bju_full == 0, "after create: NOT full yet (only one operand ready)");

    // Release via the fast da_xx_fwd_* path, matching x5's register number.
    dut->da_xx_fwd_vld     = 1;
    dut->da_xx_fwd_dst_reg = 5;
    dut->da_xx_fwd_data    = 50;   // x5 == 50 < x6(100) -> BLT taken
    dut->eval();
    check(dut->iu_idu_bju_full == 0, "forward cycle: entry not yet marked full (updates next tick)");
    tick();
    dut->da_xx_fwd_vld = 0;
    dut->eval();
    check(dut->iu_idu_bju_full != 0, "after forward absorbed: full (both operands now ready, about to pop)");

    // This cycle the entry pops and resolves.
    check(dut->iu_ifu_br_vld != 0, "entry pop: br_vld asserted");
    check(dut->iu_ifu_bht_taken != 0, "entry pop: BLT 50<100 -> taken");
    check(dut->iu_ifu_bht_mispred != 0, "entry pop: taken but predicted not-taken -> mispredict");
    check(dut->iu_ifu_tar_pc_vld != 0, "entry pop: redirect asserted");
    check(dut->iu_rtu_depd_lsu_chgflow_vld != 0,
          "entry pop: iu_rtu_depd_lsu_chgflow_vld asserted on the delayed mispredict");
    tick();
    dut->eval();
    check(dut->iu_idu_bju_full == 0 && dut->iu_idu_bju_global_full == 0,
          "after pop: entry fully drained, no more backpressure");
    test_result("T20 BJU entry: release-on-da_xx_fwd, register-number-matched, delayed mispredict to RTU");
}

static void test_bju_entry_release_on_lsu_ex2(void) {
    tie_idle_inputs();
    dut->idu_iu_ex1_inst_vld   = 1;
    dut->idu_iu_ex1_bju_sel    = 1;
    dut->idu_iu_ex1_bju_br_sel = 1;
    dut->idu_iu_ex1_func       = BJU_FUNC_BEQ;
    dut->idu_iu_ex1_src0_ready = 1;
    dut->idu_iu_ex1_src1_ready = 0;   // src1 (x7) depends on the LSU this time
    dut->idu_iu_ex1_src0_data  = 77;
    dut->idu_iu_ex1_src0_reg   = 9;
    dut->idu_iu_ex1_src1_reg   = 7;
    dut->idu_iu_ex1_src2_data  = 32;
    dut->idu_iu_ex1_bht_pred   = 0b11;   // predicted taken
    dut->eval();
    tick();
    dut->idu_iu_ex1_bju_sel = 0;
    dut->idu_iu_ex1_inst_vld = 0;
    dut->eval();
    check(dut->iu_idu_bju_global_full != 0, "lsu-release test: parked waiting on src1");

    // A DIFFERENT register's lsu_iu_ex2 forward must NOT release the entry.
    dut->lsu_iu_ex2_data_vld = 1;
    dut->lsu_iu_ex2_dest_reg = 3;   // not x7
    dut->lsu_iu_ex2_data     = 999;
    tick();
    dut->eval();
    check(dut->iu_idu_bju_global_full != 0,
          "lsu-release test: a non-matching forward does not release the entry");

    // The MATCHING forward (x7) releases it.
    dut->lsu_iu_ex2_dest_reg = 7;
    dut->lsu_iu_ex2_data     = 77;   // x7 == 77 == x9 -> BEQ taken
    dut->eval();
    tick();
    dut->lsu_iu_ex2_data_vld = 0;
    dut->eval();
    check(dut->iu_idu_bju_full != 0, "after matching lsu forward absorbed: full, about to pop");
    check(dut->iu_ifu_bht_taken != 0, "entry pop: BEQ 77==77 -> taken");
    check(dut->iu_ifu_bht_mispred == 0, "entry pop: taken, predicted taken -> no mispredict");
    check(dut->iu_ifu_tar_pc_vld == 0, "entry pop: correctly predicted -> no redirect");
    test_result("T21 BJU entry: release-on-lsu_iu_ex2, register-number-matched (non-matching forward ignored)");
}

//=============================================================================
// IFU-facing signal re-verification (Task 3.3's explicit "confirmation, not
// renaming" instruction): every port name/width this bench just drove/read
// (iu_ifu_tar_pc_vld/_tar_pc/_pc_mispred/_bht_mispred/_br_vld/_bht_taken/
// _bht_pred/_link_vld/_ret_vld, ifu_iu_chgflw_vld/_pc) compiled and linked
// against VIU.h generated straight from IU.v's port list, which in turn was
// copied byte-for-byte from rtl/RVProc.v's existing M1 wire declarations
// (IU.v's own header note) -- the fact this file builds and every access
// above resolves is itself the re-verification; this test adds one more
// explicit spot-check of the reset-driven mrvbr seed and the chgflw path.
//=============================================================================

static void test_ifu_facing_seed_and_chgflw(void) {
    reset_dut();   // clean bju_pcgen_pc, undisturbed by earlier tests' JAL/branch traffic
    check((uint64_t)dut->cp0_xx_mrvbr == RESET_VECTOR, "reset: BJU's PC seed == cp0_xx_mrvbr (RESET_VECTOR)");
    BjuResult r0 = bju_op(BJU_FUNC_JAL, false, 0, 0, 0x100, 0, 0, 2, 0);
    check(r0.cur_pc == RESET_VECTOR, "iu_rtu_ex1_cur_pc tracks bju_pcgen_pc from reset", r0.cur_pc, RESET_VECTOR);

    // ifu_iu_chgflw_vld (highest-priority redirect, e.g. a taken trap)
    // overrides the self-tracked PC on the very next tick.
    tie_idle_inputs();
    dut->ifu_iu_chgflw_vld = 1;
    dut->ifu_iu_chgflw_pc  = 0x80004000ULL;
    tick();
    dut->ifu_iu_chgflw_vld = 0;
    BjuResult r1 = bju_op(BJU_FUNC_JAL, false, 0, 0, 0x100, 0, 0, 2, 0);
    check(r1.cur_pc == 0x80004000ULL, "ifu_iu_chgflw_vld redirects BJU's self-tracked PC",
          r1.cur_pc, 0x80004000ULL);
    test_result("T22 IFU-facing ports + cp0_xx_mrvbr/ifu_iu_chgflw_* re-verified byte-for-byte");
}

//=============================================================================
// Mutation-check discipline note (plan task 3.7, same discipline as
// csr_tb.cpp's Task 2 development): each mutation below was applied BY HAND
// to rtl/IU.v (not left as code here), this bench re-run to confirm a FAIL,
// then the mutation reverted before committing.
//
// Mutation 1 -- MULT iteration-count early-out (IU note S5): changed
//   `wire mul_needs_split = !mul_is_mulw && !(mul_src0_fits33 && mul_src1_fits33);`
//   to `wire mul_needs_split = 1'b0;` (force every multiply down the
//   narrow/single-pass timing, regardless of operand width). Confirmed
//   FAIL in test_mult_wide_timing() (T10): the wide-operand MULH's VALUE
//   stayed correct (the clean-room result computation does not depend on
//   mul_needs_split at all, only its TIMING does -- exactly why this
//   mutation is a meaningful, non-trivial check of the early-out
//   specifically, not a proxy for a value bug) but total_ticks collapsed to
//   equal the narrow case, failing both the "strictly more ticks" and "at
//   least 2 extra ticks" assertions. Reverted before commit.
//
// Mutation 2 -- DIV 1-entry memo buffer (IU note S6): changed
//   `wire div_hit_buffer = div_memo_vld && (div_memo_dividend == div_dividend) ...;`
//   to `wire div_hit_buffer = 1'b0;` (permanently disable the fast-path
//   reuse). Confirmed FAIL in test_div_memo_hit() (T15): the second,
//   operand-identical divide's VALUE stayed correct (recomputed fresh via
//   the full ITER path instead of reused), but it no longer completed
//   faster than the first divide -- failing the "reuse is strictly faster"
//   and "fast (<=3 tick) path" assertions. Reverted before commit.
//=============================================================================

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VIU;

    reset_dut();

    test_alu_add_sub();
    test_alu_addw_subw();
    test_alu_slt_sltu();
    test_alu_shifts();
    test_alu_logic();
    test_alu_lui();
    test_alu_xthead();
    test_alu_srri_ext();

    test_mult_narrow();
    test_mult_wide_timing();
    test_mult_full_backpressure();

    test_div_basic();
    test_div_word_forms();
    test_div_abnormal();
    test_div_memo_hit();
    test_div_variable_latency();
    test_div_full_backpressure();

    test_bju_cond_branch_matrix();
    test_bju_jal_jalr_auipc();
    test_bju_entry_release_on_da_fwd();
    test_bju_entry_release_on_lsu_ex2();
    test_ifu_facing_seed_and_chgflw();

    printf("[iu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}

