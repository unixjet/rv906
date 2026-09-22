//=============================================================================
// idu_tb.cpp - standalone unit bench for rtl/IDU.v (M2 plan task 5.5)
//=============================================================================
// Verilates IDU.v + rvproc_pkg.sv alone (no IU, no LSU, no CSR, no RTU) and
// drives the frozen ifu_idu_id_*/rtu_idu_*/iu_idu_*/lsu_idu_full ports
// directly with hand-scripted, per-cycle stimulus standing in for those
// four real neighbors, the same tick()-based clocking and check()/
// test_result() bookkeeping pattern as csr_tb.cpp/iu_tb.cpp/rtu_tb.cpp.
//
// This is a WHITE-BOX test of IDU.v's OWN documented contract (decode
// closed illegal-list, WBT except-clause matrix, GPR collision handling,
// EU dispatch, EX1 issue-gate) run in isolation, driven by a script that
// stands in for IFU/IU/LSU/CSR/RTU -- it does not exercise a real pipe.
//
// Build/run: make -C test/m2/unit idu && bin/unit/idu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VIDU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VIDU *dut = nullptr;
static uint64_t g_cycles = 0;

static void tie_idle_inputs(void) {
    dut->ifu_idu_id_inst      = 0;
    dut->ifu_idu_id_inst_vld  = 0;
    dut->ifu_idu_id_bht_pred  = 0;
    dut->ifu_idu_id_fault_pgflt  = 0;
    dut->ifu_idu_id_fault_accflt = 0;

    dut->rtu_idu_fwd0_data = 0; dut->rtu_idu_fwd0_reg = 0; dut->rtu_idu_fwd0_vld = 0;
    dut->rtu_idu_fwd1_data = 0; dut->rtu_idu_fwd1_reg = 0; dut->rtu_idu_fwd1_vld = 0;
    dut->rtu_idu_fwd2_data = 0; dut->rtu_idu_fwd2_reg = 0; dut->rtu_idu_fwd2_vld = 0;
    dut->rtu_idu_wb0_data  = 0; dut->rtu_idu_wb0_reg  = 0; dut->rtu_idu_wb0_vld  = 0;
    dut->rtu_idu_wb1_data  = 0; dut->rtu_idu_wb1_reg  = 0; dut->rtu_idu_wb1_vld  = 0;
    dut->rtu_idu_wbf0_data = 0; dut->rtu_idu_wbf0_reg = 0; dut->rtu_idu_wbf0_vld = 0;
    dut->rtu_idu_wbf1_data = 0; dut->rtu_idu_wbf1_reg = 0; dut->rtu_idu_wbf1_vld = 0;

    dut->iu_idu_mult_issue_stall = 0;
    dut->iu_idu_mult_full        = 0;
    dut->iu_idu_div_full         = 0;
    dut->iu_idu_bju_full         = 0;
    dut->iu_idu_bju_global_full  = 0;
    dut->lsu_idu_full            = 0;

    dut->rtu_idu_flush_fe        = 0;
    dut->rtu_idu_flush_stall     = 0;
    dut->rtu_idu_flush_wbt       = 0;
    dut->rtu_idu_commit          = 1;
    dut->rtu_idu_commit_for_bju  = 1;
    dut->rtu_idu_pipeline_empty  = 1;
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
// Result bookkeeping (mirrors csr_tb.cpp/iu_tb.cpp/rtu_tb.cpp exactly)
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
    printf("[idu_tb] %-64s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// Instruction encoders (standard RV64GC bit layouts)
//-----------------------------------------------------------------------------
static uint32_t enc_r(uint32_t f7, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t rd, uint32_t op) {
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
}
static uint32_t enc_i(int32_t imm12, uint32_t rs1, uint32_t f3, uint32_t rd, uint32_t op) {
    return (((uint32_t)imm12 & 0xFFFu) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
}
static uint32_t enc_s(int32_t imm12, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t op) {
    uint32_t imm = (uint32_t)imm12 & 0xFFFu;
    return ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((imm & 0x1Fu) << 7) | op;
}
static uint32_t enc_b(int32_t imm13, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t op) {
    uint32_t imm = (uint32_t)imm13 & 0x1FFFu;
    uint32_t b12 = (imm >> 12) & 1, b11 = (imm >> 11) & 1, b10_5 = (imm >> 5) & 0x3F, b4_1 = (imm >> 1) & 0xF;
    return (b12 << 31) | (b10_5 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (b4_1 << 8) | (b11 << 7) | op;
}
static uint32_t enc_u(int32_t imm20, uint32_t rd, uint32_t op) {
    return (((uint32_t)imm20 & 0xFFFFFu) << 12) | (rd << 7) | op;
}
static uint32_t enc_j(int32_t imm21, uint32_t rd, uint32_t op) {
    uint32_t imm = (uint32_t)imm21 & 0x1FFFFFu;
    uint32_t b20 = (imm >> 20) & 1, b19_12 = (imm >> 12) & 0xFF, b11 = (imm >> 11) & 1, b10_1 = (imm >> 1) & 0x3FF;
    return (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) | (rd << 7) | op;
}
// RVC encoders
static uint32_t enc_ci(uint32_t f3, uint32_t imm6, uint32_t rd_rs1, uint32_t op) {
    uint32_t bit12 = (imm6 >> 5) & 1, bits6_2 = imm6 & 0x1F;
    return (f3 << 13) | (bit12 << 12) | (rd_rs1 << 7) | (bits6_2 << 2) | op;
}
static uint32_t enc_c_shift_andi(uint32_t subop2, uint32_t rd_rs1_3, uint32_t imm6) {
    uint32_t bit12 = (imm6 >> 5) & 1, bits6_2 = imm6 & 0x1F;
    return (0x4u << 13) | (bit12 << 12) | (subop2 << 10) | (rd_rs1_3 << 7) | (bits6_2 << 2) | 0x1u;
}
static uint32_t enc_c_alu_reg(uint32_t funct2, uint32_t rd_rs1_3, uint32_t rs2_3) {
    return (0x23u << 10) | (rd_rs1_3 << 7) | (funct2 << 5) | (rs2_3 << 2) | 0x1u;
}
static uint32_t enc_cr(uint32_t funct4, uint32_t rd_rs1, uint32_t rs2) {
    return (funct4 << 12) | (rd_rs1 << 7) | (rs2 << 2) | 0x2u;
}

// Opcodes
static const uint32_t OP_LOAD  = 0x03, OP_STORE = 0x23, OP_OPIMM = 0x13, OP_OP = 0x33;
static const uint32_t OP_LUI = 0x37, OP_AUIPC = 0x17, OP_JAL = 0x6F, OP_JALR = 0x67;
static const uint32_t OP_BRANCH = 0x63, OP_SYSTEM = 0x73, OP_OPIMM32 = 0x1B, OP_OP32 = 0x3B;
static const uint32_t OP_MISCMEM = 0x0F;
static const uint32_t OP_FP = 0x53, OP_AMO = 0x2F, OP_VEC = 0x57, OP_CUSTOM0 = 0x0B;
static const uint32_t OP_LOAD_FP = 0x07, OP_STORE_FP = 0x27;
// CP0 FUNC encodings (rvproc_pkg.sv CP0_FUNC_*) needed by the M4 decode checks.
static const uint32_t CP0_FUNC_SRET   = 0x00082;
static const uint32_t CP0_FUNC_WFI    = 0x00102;
static const uint32_t CP0_FUNC_SFENCE = 0x00044;
// LSU FUNC encodings (rvproc_pkg.sv LSU_FUNC_*) needed by the M5 Task 4c
// LOAD-FP/STORE-FP decode checks.
static const uint32_t LSU_FUNC_FLW = 0x00208;
static const uint32_t LSU_FUNC_FLD = 0x0020c;
static const uint32_t LSU_FUNC_FSW = 0x00209;
static const uint32_t LSU_FUNC_FSD = 0x0020d;
// FP FUNC bit positions (rvproc_pkg.sv FUNC_* -- bit indices, not packed
// hex literals; consumed via idu_fpu_ex1_func[FUNC_X]-style tests in FPU.v).
static const uint32_t FUNC_DOUBLE_BIT   = 16;
static const uint32_t FUNC_CLASS_BIT    = 18;
static const uint32_t FUNC_CMP_BIT      = 10;
static const uint32_t FUNC_CMP_LE_BIT   = 2;
static const uint32_t FUNC_CMP_LT_BIT   = 1;
static const uint32_t FUNC_CMP_FEQ_BIT  = 0;
// M5 Task 4b: FADD/FSUB/FMINMAX/FSGNJ/FCVT (f2f) arms (rvproc_pkg.sv).
static const uint32_t FUNC_B_SINGLE_BIT   = 15;
static const uint32_t FUNC_CVT_WIDDEN_BIT = 14;
static const uint32_t FUNC_CVT_NARROW_BIT = 13;
static const uint32_t FUNC_ADD_BIT        = 12;
static const uint32_t FUNC_SUB_BIT        = 11;
static const uint32_t FUNC_MAX_BIT        = 9;
static const uint32_t FUNC_MIN_BIT        = 8;
static const uint32_t FUNC_SPU_SGN_BIT    = 6;
static const uint32_t FUNC_SPU_SGN_X_BIT  = 2;
static const uint32_t FUNC_SPU_SGN_N_BIT  = 1;
static const uint32_t FUNC_SPU_SGN_J_BIT  = 0;

static uint32_t addi(uint32_t rd, uint32_t rs1, int32_t imm) { return enc_i(imm, rs1, 0x0, rd, OP_OPIMM); }
static uint32_t add_ (uint32_t rd, uint32_t rs1, uint32_t rs2) { return enc_r(0x00, rs2, rs1, 0x0, rd, OP_OP); }
static uint32_t sub_ (uint32_t rd, uint32_t rs1, uint32_t rs2) { return enc_r(0x20, rs2, rs1, 0x0, rd, OP_OP); }
static uint32_t lw   (uint32_t rd, uint32_t rs1, int32_t imm) { return enc_i(imm, rs1, 0x2, rd, OP_LOAD); }
static uint32_t sw   (uint32_t rs2, uint32_t rs1, int32_t imm) { return enc_s(imm, rs2, rs1, 0x2, OP_STORE); }
static uint32_t mul_ (uint32_t rd, uint32_t rs1, uint32_t rs2) { return enc_r(0x01, rs2, rs1, 0x0, rd, OP_OP); }
static uint32_t div_ (uint32_t rd, uint32_t rs1, uint32_t rs2) { return enc_r(0x01, rs2, rs1, 0x4, rd, OP_OP); }
static uint32_t beq_ (uint32_t rs1, uint32_t rs2, int32_t imm) { return enc_b(imm, rs2, rs1, 0x0, OP_BRANCH); }
static uint32_t jal_ (uint32_t rd, int32_t imm) { return enc_j(imm, rd, OP_JAL); }
static uint32_t jalr_(uint32_t rd, uint32_t rs1, int32_t imm) { return enc_i(imm, rs1, 0x0, rd, OP_JALR); }
static uint32_t lui_ (uint32_t rd, int32_t imm20) { return enc_u(imm20, rd, OP_LUI); }
static uint32_t auipc_(uint32_t rd, int32_t imm20) { return enc_u(imm20, rd, OP_AUIPC); }
static uint32_t csrrw_(uint32_t rd, uint32_t rs1, uint32_t csr) { return enc_i(csr, rs1, 0x1, rd, OP_SYSTEM); }
static uint32_t fence_(void) { return enc_i(0, 0, 0x0, 0, OP_MISCMEM); }
static uint32_t fencei_(void) { return enc_i(0, 0, 0x1, 0, OP_MISCMEM); }
static uint32_t ecall_(void) { return enc_i(0x000, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t ebreak_(void) { return enc_i(0x001, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t mret_(void) { return enc_i(0x302, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t sret_(void) { return enc_i(0x102, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t wfi_(void) { return enc_i(0x105, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t dret_(void) { return enc_i(0x7b2, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t sfence_vma_(void) { return enc_r(0x09, 0, 0, 0x0, 0, OP_SYSTEM); }
static uint32_t famo_add_w(void) { return enc_r(0x00, 6, 1, 0x2, 5, OP_AMO); } // amoadd.w x5,x6,(x1)
static uint32_t famo_reserved_w(void) { return enc_r(0x14, 6, 1, 0x2, 5, OP_AMO); } // funct5=00101 (reserved)
static uint32_t fp_add(void) { return enc_r(0x00, 1, 2, 0x7, 5, OP_FP); }
// M5 Task 4a: OP-FP compare/classify (feq/flt/fle.s/d, fclass.s/d) --
// rd=5, rs1=1, rs2=2 (rs2=0 for fclass, RISC-V spec: unused operand field).
static uint32_t feq_s(void) { return enc_r(0x50, 2, 1, 0x2, 5, OP_FP); }
static uint32_t flt_s(void) { return enc_r(0x50, 2, 1, 0x1, 5, OP_FP); }
static uint32_t fle_s(void) { return enc_r(0x50, 2, 1, 0x0, 5, OP_FP); }
static uint32_t feq_d(void) { return enc_r(0x51, 2, 1, 0x2, 5, OP_FP); }
static uint32_t flt_d(void) { return enc_r(0x51, 2, 1, 0x1, 5, OP_FP); }
static uint32_t fle_d(void) { return enc_r(0x51, 2, 1, 0x0, 5, OP_FP); }
static uint32_t fclass_s(void) { return enc_r(0x70, 0, 1, 0x1, 5, OP_FP); }
static uint32_t fclass_d(void) { return enc_r(0x71, 0, 1, 0x1, 5, OP_FP); }
// M5 Task 4b: OP-FP add/sub/minmax/sgnj-family/f2f-convert (IDU.v:883-948).
// rd=5, rs1=1, rs2=2 (rs2 arbitrary/ignored for the f2f-convert pair, per
// IDU.v's comment at 938-941 -- not separately legality-checked there).
static uint32_t fadd_s(uint32_t rm) { return enc_r(0x00, 2, 1, rm, 5, OP_FP); }
static uint32_t fadd_d(uint32_t rm) { return enc_r(0x01, 2, 1, rm, 5, OP_FP); }
static uint32_t fsub_s(uint32_t rm) { return enc_r(0x04, 2, 1, rm, 5, OP_FP); }
static uint32_t fsub_d(uint32_t rm) { return enc_r(0x05, 2, 1, rm, 5, OP_FP); }
static uint32_t fmin_s(void) { return enc_r(0x14, 2, 1, 0x0, 5, OP_FP); }
static uint32_t fmax_s(void) { return enc_r(0x14, 2, 1, 0x1, 5, OP_FP); }
static uint32_t fmin_d(void) { return enc_r(0x15, 2, 1, 0x0, 5, OP_FP); }
static uint32_t fmax_d(void) { return enc_r(0x15, 2, 1, 0x1, 5, OP_FP); }
static uint32_t fsgnj_s(void)  { return enc_r(0x10, 2, 1, 0x0, 5, OP_FP); }
static uint32_t fsgnjn_s(void) { return enc_r(0x10, 2, 1, 0x1, 5, OP_FP); }
static uint32_t fsgnjx_s(void) { return enc_r(0x10, 2, 1, 0x2, 5, OP_FP); }
static uint32_t fsgnj_d(void)  { return enc_r(0x11, 2, 1, 0x0, 5, OP_FP); }
static uint32_t fsgnjn_d(void) { return enc_r(0x11, 2, 1, 0x1, 5, OP_FP); }
static uint32_t fsgnjx_d(void) { return enc_r(0x11, 2, 1, 0x2, 5, OP_FP); }
static uint32_t fcvt_s_d(uint32_t rm) { return enc_r(0x20, 1, 1, rm, 5, OP_FP); }
static uint32_t fcvt_d_s(uint32_t rm) { return enc_r(0x21, 0, 1, rm, 5, OP_FP); }
// M5 Task 4c: LOAD-FP/STORE-FP (I-type for loads, S-type for stores).
static uint32_t flw_(uint32_t rd, uint32_t rs1, int32_t imm) { return enc_i(imm, rs1, 0x2, rd, OP_LOAD_FP); }
static uint32_t fld_(uint32_t rd, uint32_t rs1, int32_t imm) { return enc_i(imm, rs1, 0x3, rd, OP_LOAD_FP); }
static uint32_t fsw_(uint32_t rs2, uint32_t rs1, int32_t imm) { return enc_s(imm, rs2, rs1, 0x2, OP_STORE_FP); }
static uint32_t fsd_(uint32_t rs2, uint32_t rs1, int32_t imm) { return enc_s(imm, rs2, rs1, 0x3, OP_STORE_FP); }
// M5 Task 5/6/7: remaining OP-FP families (FMAU/FDSU/fcvt-int/fmv). funct7
// values pinned to IDU.v's decode arms; FMA major opcode is 10000 (R4).
static uint32_t fmul_s(uint32_t rm)   { return enc_r(0x08, 2, 1, rm, 5, OP_FP); } // 0001000
static uint32_t fdiv_s(uint32_t rm)   { return enc_r(0x0C, 2, 1, rm, 5, OP_FP); } // 0001100
static uint32_t fsqrt_s(void)         { return enc_r(0x2C, 2, 1, 0x0, 5, OP_FP); } // 0101100
static uint32_t fmadd_s(void)         { return enc_r(0x08, 2, 1, 0x0, 5, 0x43u); } // rs3=2,fmt=0,op=10000(+2'b11)
static uint32_t fcvt_w_s(void)        { return enc_r(0x60, 0, 1, 0x0, 5, OP_FP); } // 1100000 (f2i)
static uint32_t fcvt_s_w(uint32_t rm) { return enc_r(0x68, 0, 1, rm, 5, OP_FP); }  // 1101000 (i2f)
static uint32_t fmv_x_w(void)         { return enc_r(0x70, 0, 1, 0x0, 5, OP_FP); } // 1110000
static uint32_t fmv_w_x(void)         { return enc_r(0x78, 0, 1, 0x0, 5, OP_FP); } // 1111000
static uint32_t vec_add(void) { return enc_r(0x00, 1, 2, 0x7, 5, OP_VEC); }
static uint32_t custom0(void) { return enc_r(0x00, 1, 2, 0x1, 5, OP_CUSTOM0); }

// RVC
static uint32_t c_nop(void)  { return enc_ci(0x0, 0, 0, 0x1); }
static uint32_t c_addi(uint32_t rd_rs1, uint32_t imm6) { return enc_ci(0x0, imm6, rd_rs1, 0x1); }
static uint32_t c_li(uint32_t rd, uint32_t imm6) { return enc_ci(0x2, imm6, rd, 0x1); }
static uint32_t c_lui(uint32_t rd, uint32_t imm6) { return enc_ci(0x3, imm6, rd, 0x1); }
static uint32_t c_andi(uint32_t rd_rs1_3, uint32_t imm6) { return enc_c_shift_andi(0x2, rd_rs1_3, imm6); }
static uint32_t c_sub(uint32_t rd_rs1_3, uint32_t rs2_3) { return enc_c_alu_reg(0x0, rd_rs1_3, rs2_3); }
static uint32_t c_mv(uint32_t rd, uint32_t rs2) { return enc_cr(0x8, rd, rs2); }
static uint32_t c_add(uint32_t rd_rs1, uint32_t rs2) { return enc_cr(0x9, rd_rs1, rs2); }
static uint32_t c_addi4spn_bad(void) { return 0x0000; } // nzuimm=0, reserved
static uint32_t c_fld(void) { return (0x1u << 13) | 0x0u; } // quadrant00, funct3=001 -> c.fld

//-----------------------------------------------------------------------------
// Helpers built on the DUT
//-----------------------------------------------------------------------------
static void write_gpr(uint32_t reg, uint64_t data) {
    dut->rtu_idu_wb0_reg  = reg;
    dut->rtu_idu_wb0_data = data;
    dut->rtu_idu_wb0_vld  = 1;
    tick();
    dut->rtu_idu_wb0_vld  = 0;
    dut->rtu_idu_wb0_reg  = 0;
    dut->rtu_idu_wb0_data = 0;
}

static void write_frf(uint32_t reg, uint64_t data) {
    dut->rtu_idu_wbf0_reg  = reg;
    dut->rtu_idu_wbf0_data = data;
    dut->rtu_idu_wbf0_vld  = 1;
    tick();
    dut->rtu_idu_wbf0_vld  = 0;
    dut->rtu_idu_wbf0_reg  = 0;
    dut->rtu_idu_wbf0_data = 0;
}

// Raw OP-FP-shaped word (op=0x53) placing rs3 at inst[31:27] and fmt at
// inst[26:25] via f7=(rs3<<2)|fmt, per the R4-type FP format. No casez arm
// decodes OP-FP yet (Task 4), but dis_fsrc0/1/2_reg5 slice these bit
// positions unconditionally, so this is sufficient to probe FRF reads.
static uint32_t enc_fp_r4(uint32_t rs3, uint32_t fmt, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t rd) {
    return enc_r((rs3 << 2) | fmt, rs2, rs1, f3, rd, 0x53u);
}

static void present(uint32_t inst, bool vld = true) {
    dut->ifu_idu_id_inst     = inst;
    dut->ifu_idu_id_inst_vld = vld;
}

static void present_fetch_fault(bool pgflt, bool accflt) {
    // Mirrors IFU.v's synthetic-NOP injection: the fault-carrying slot is
    // always addi x0,x0,0 (32'h00000013), regardless of pgflt/accflt.
    dut->ifu_idu_id_inst          = 0x00000013u;
    dut->ifu_idu_id_inst_vld      = 1;
    dut->ifu_idu_id_fault_pgflt   = pgflt  ? 1 : 0;
    dut->ifu_idu_id_fault_accflt  = accflt ? 1 : 0;
}

static void clear_fetch_fault(void) {
    dut->ifu_idu_id_fault_pgflt  = 0;
    dut->ifu_idu_id_fault_accflt = 0;
}

//=============================================================================
// Tests
//=============================================================================

static void test_reset_state(void) {
    check(dut->idu_iu_ex1_inst_vld == 0, "reset: EX1 not valid");
    check(dut->idu_iu_ex1_alu_sel == 0, "reset: no alu_sel");
    check(dut->idu_cp0_ex1_sel == 0, "reset: no cp0_sel");
    check(dut->idu_lsu_ex1_sel == 0, "reset: no lsu_sel");
    test_result("T1 reset state: EX1 empty, nothing dispatched");
}

// ---- 5.1: 32-bit decode across every rv64im class ----
static void test_decode_32bit_alu(void) {
    reset_dut();
    write_gpr(1, 0x10);
    write_gpr(2, 0x03);
    present(add_(3, 1, 2)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "add: alu_sel fires");
    check(dut->idu_iu_ex1_src0_data == 0x10, "add: src0=x1", dut->idu_iu_ex1_src0_data, 0x10);
    check(dut->idu_iu_ex1_src1_data == 0x03, "add: src1=x2", dut->idu_iu_ex1_src1_data, 0x03);
    check(dut->idu_iu_ex1_dst0_reg == 3, "add: dst0=x3", dut->idu_iu_ex1_dst0_reg, 3);
    test_result("T2 32-bit ALU decode (add): correct src/dst + alu_sel");
}

static void test_decode_32bit_addi(void) {
    reset_dut();
    write_gpr(1, 5);
    present(addi(2, 1, 7)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "addi: alu_sel fires");
    check(dut->idu_iu_ex1_src1_data == 7, "addi: src1=imm=7", dut->idu_iu_ex1_src1_data, 7);
    check(dut->idu_iu_ex1_src0_data == 5, "addi: src0=x1=5", dut->idu_iu_ex1_src0_data, 5);
    test_result("T3 32-bit ALU-imm decode (addi): immediate lands in src1_data");
}

static void test_decode_32bit_lsu(void) {
    reset_dut();
    write_gpr(1, 0x1000);
    present(lw(2, 1, 8)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_sel == 1, "lw: lsu_sel fires");
    check(dut->idu_lsu_ex1_src0_data == 0x1000, "lw: src0=x1(base)");
    check(dut->idu_lsu_ex1_src1_data == 8, "lw: src1=imm(offset)=8");
    check(dut->idu_lsu_ex1_dst0_reg == 2, "lw: dst0=x2");

    write_gpr(3, 0xAB);
    present(sw(3, 1, 4)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_sel == 1, "sw: lsu_sel fires");
    check(dut->idu_lsu_ex1_src2_data == 0xAB, "sw: src2=x3(store data)");
    test_result("T4 32-bit LSU decode (lw/sw): base+offset+store-data slots");
}

static void test_decode_32bit_bju(void) {
    reset_dut();
    write_gpr(1, 5); write_gpr(2, 5);
    present(beq_(1, 2, 16)); tick(); present(0, false);
    check(dut->idu_iu_ex1_bju_sel == 1, "beq: bju_sel fires");
    check(dut->idu_iu_ex1_src0_data == 5, "beq: src0=x1");
    check(dut->idu_iu_ex1_src1_data == 5, "beq: src1=x2");

    present(jal_(1, 0x100)); tick(); present(0, false);
    check(dut->idu_iu_ex1_bju_sel == 1, "jal: bju_sel fires");
    check(dut->idu_iu_ex1_dst0_reg == 1, "jal: dst0=x1(link)");

    present(jalr_(1, 2, 4)); tick(); present(0, false);
    check(dut->idu_iu_ex1_bju_sel == 1, "jalr: bju_sel fires");
    check(dut->idu_iu_ex1_src0_data == 5, "jalr: src0=x2(target base)");
    test_result("T5 32-bit BJU decode (beq/jal/jalr)");
}

static void test_decode_32bit_mult_div(void) {
    reset_dut();
    write_gpr(1, 6); write_gpr(2, 7);
    present(mul_(3, 1, 2)); tick(); present(0, false);
    check(dut->idu_iu_ex1_mult_sel == 1, "mul: mult_sel fires");
    present(div_(4, 1, 2)); tick(); present(0, false);
    check(dut->idu_iu_ex1_div_sel == 1, "div: div_sel fires");
    test_result("T6 32-bit MULT/DIV decode (mul/div): correct one-hot EU");
}

static void test_decode_32bit_lui_auipc(void) {
    reset_dut();
    present(lui_(5, 0x12345)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "lui: alu_sel fires");
    check(dut->idu_iu_ex1_src1_data == (uint64_t)0x12345000, "lui: src1=imm<<12",
          dut->idu_iu_ex1_src1_data, 0x12345000);

    present(auipc_(6, 0x1)); tick(); present(0, false);
    check(dut->idu_iu_ex1_bju_sel == 1, "auipc: bju_sel fires (BJU_FUNC_AUIPC)");
    check(dut->idu_iu_ex1_src2_data == 0x1000, "auipc: src2=imm<<12", dut->idu_iu_ex1_src2_data, 0x1000);
    test_result("T7 32-bit LUI/AUIPC decode: U-type immediate shift correct");
}

static void test_decode_32bit_csr(void) {
    reset_dut();
    write_gpr(1, 0xDEAD);
    present(csrrw_(2, 1, 0x340 /* mscratch */)); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1, "csrrw: cp0_sel fires");
    check(dut->idu_cp0_ex1_src0_data == 0xDEAD, "csrrw: src0=rs1 value");
    check((dut->idu_cp0_ex1_src1_data & 0xFFF) == 0x340, "csrrw: src1=csr addr");
    check(dut->idu_cp0_ex1_illegal == 0, "csrrw: legal");
    test_result("T8 32-bit CSR decode (csrrw): rs1 value + csr address slots");
}

static void test_decode_32bit_fence_ecall(void) {
    reset_dut();
    present(fence_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1, "fence: cp0_sel fires");
    check(dut->idu_cp0_ex1_illegal == 0, "fence: legal");

    present(fencei_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1, "fence.i: cp0_sel fires");
    check(dut->idu_cp0_ex1_illegal == 0, "fence.i: legal");

    present(ecall_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1, "ecall: cp0_sel fires");
    check(dut->idu_cp0_ex1_illegal == 0, "ecall: legal");

    present(ebreak_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0, "ebreak: legal");

    present(mret_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0, "mret: legal");
    test_result("T9 32-bit FENCE/FENCE.I/ECALL/EBREAK/MRET: legal single-beat CP0 ops");
}

// ---- 5.1: closed illegal-decode list ----
static void test_illegal_closed_list(void) {
    reset_dut();
    present(fp_add()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_cp0_ex1_sel == 0
          && dut->idu_fpu_ex1_fadd_sel == 1,
          "FP fadd.s: now legal, dispatches to EU_FP (THE SWAP, M5 Task 9)");

    present(vec_add()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 1, "vector op: illegal");

    present(custom0()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 1, "custom-0 (cache/perf): illegal");

    present(famo_add_w()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_lsu_ex1_sel == 1,
          "amoadd.w: legal, dispatches to LSU (M3 AMO decode)");

    present(famo_reserved_w()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1 && dut->idu_cp0_ex1_illegal == 1
          && dut->idu_lsu_ex1_sel == 0,
          "reserved AMO funct5: illegal (M3 audit; donor decd.v lists only the 9)");

    // M4: sfence.vma / sret / wfi now decode as legal CP0 ops (privilege-
    // based legality -- TSR/TW/TVM/U-mode -- is checked in CSR.v, not here).
    present(sfence_vma_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1 && dut->idu_cp0_ex1_illegal == 0
          && dut->idu_cp0_ex1_func == CP0_FUNC_SFENCE,
          "sfence.vma: legal, dispatches to CP0 (M4)");

    present(sret_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1 && dut->idu_cp0_ex1_illegal == 0
          && dut->idu_cp0_ex1_func == CP0_FUNC_SRET,
          "sret: legal, dispatches to CP0 (M4)");

    present(wfi_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_sel == 1 && dut->idu_cp0_ex1_illegal == 0
          && dut->idu_cp0_ex1_func == CP0_FUNC_WFI,
          "wfi: legal, dispatches to CP0 (M4)");

    present(dret_()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 1, "dret: illegal (no debug unit in M2)");

    // reserved-encoding malformed ecall (rs1 != 0)
    present(enc_i(0, 5, 0x0, 0, OP_SYSTEM)); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 1, "ecall with rs1!=0: illegal (malformed)");
    test_result("T10 illegal decode closed list: vector/custom/dret trap (FP now legal, M5 swap); sfence/sret/wfi legal (M4), AMO legal (M3)");
}

// ---- M4 Task 6: IFU->IDU fetch-fault channel forces EU_CP0 dispatch ----
static void test_fetch_fault_forces_cp0(void) {
    reset_dut();
    present_fetch_fault(/*pgflt=*/true, /*accflt=*/false);
    tick();
    clear_fetch_fault();
    present(0, false);
    check(dut->idu_cp0_ex1_sel == 1, "fetch pgflt: forced to EU_CP0 despite legal-looking ADDI bits");
    check(dut->idu_cp0_ex1_fetch_pgflt == 1, "fetch pgflt: idu_cp0_ex1_fetch_pgflt latched into EX1");
    check(dut->idu_cp0_ex1_fetch_accflt == 0, "fetch pgflt: accflt stays clear");
    check(dut->idu_cp0_ex1_illegal == 0, "fetch pgflt: NOT reported as illegal (distinct signal)");
    check(dut->idu_iu_ex1_alu_sel == 0, "fetch pgflt: does not also dispatch to ALU");

    reset_dut();
    present_fetch_fault(/*pgflt=*/false, /*accflt=*/true);
    tick();
    clear_fetch_fault();
    present(0, false);
    check(dut->idu_cp0_ex1_sel == 1, "fetch accflt: forced to EU_CP0");
    check(dut->idu_cp0_ex1_fetch_accflt == 1, "fetch accflt: idu_cp0_ex1_fetch_accflt latched into EX1");
    check(dut->idu_cp0_ex1_fetch_pgflt == 0, "fetch accflt: pgflt stays clear");
    test_result("T10b fetch-fault channel: ifu_idu_id_fault_{pgflt,accflt} force EU_CP0 dispatch and latch through EX1");
}

// ---- M5 Task 4a: OP-FP compare/classify decode arms. Live as of THE SWAP
// (M5 Task 9): these now decode legal (d32_illegal=0) and dispatch to
// EU_FP, so the func bits drive fadd_sel (compare arms) / fspu_sel
// (fclass) -- gated on ex1_eu_r[EU_FP_SEL], which now sets. ex1_func_r is
// not illegal-gated, so the func bits were always correct; what the swap
// flips is ex1_eu_r (see IDU.v dis_eu_final). ----
static void test_fp_cmp_class_decode(void) {
    reset_dut();
    present(feq_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_cp0_ex1_sel == 0
          && dut->idu_fpu_ex1_fadd_sel == 1,
          "feq.s: legal, dispatches to EU_FP (fadd_sel fires) -- THE SWAP");
    check(((dut->idu_fpu_ex1_func >> FUNC_CMP_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_FEQ_BIT) & 1) == 1,
          "feq.s: FUNC_CMP+FUNC_CMP_FEQ bits set");
    check(dut->idu_fpu_ex1_fspu_sel == 0,
          "feq.s: fspu_sel stays 0 (FUNC_CMP is an fadd_sel-group op, not fspu)");
    check(dut->idu_cp0_ex1_dst0_reg == 5, "feq.s: dst0_reg == rd (GPR dest, xvld-class default path)");

    reset_dut();
    present(flt_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fadd_sel == 1, "flt.s: legal, dispatches to EU_FP (THE SWAP)");
    check(((dut->idu_fpu_ex1_func >> FUNC_CMP_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_LT_BIT) & 1) == 1,
          "flt.s: FUNC_CMP+FUNC_CMP_LT bits set");

    reset_dut();
    present(fle_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fadd_sel == 1, "fle.s: legal, dispatches to EU_FP (THE SWAP)");
    check(((dut->idu_fpu_ex1_func >> FUNC_CMP_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_LE_BIT) & 1) == 1,
          "fle.s: FUNC_CMP+FUNC_CMP_LE bits set");

    reset_dut();
    present(feq_d()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fadd_sel == 1, "feq.d: legal, dispatches to EU_FP (THE SWAP)");
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_FEQ_BIT) & 1) == 1,
          "feq.d: FUNC_DOUBLE+FUNC_CMP+FUNC_CMP_FEQ bits set");

    reset_dut();
    present(flt_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_LT_BIT) & 1) == 1,
          "flt.d: FUNC_DOUBLE+FUNC_CMP+FUNC_CMP_LT bits set");

    reset_dut();
    present(fle_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CMP_LE_BIT) & 1) == 1,
          "fle.d: FUNC_DOUBLE+FUNC_CMP+FUNC_CMP_LE bits set");

    reset_dut();
    present(fclass_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0, "fclass.s: legal (THE SWAP)");
    check(((dut->idu_fpu_ex1_func >> FUNC_CLASS_BIT) & 1) == 1,
          "fclass.s: FUNC_CLASS bit set");
    check(dut->idu_fpu_ex1_fspu_sel == 1,
          "fclass.s: fspu_sel fires (FUNC_CLASS, EU_FP live -- THE SWAP)");
    check(dut->idu_cp0_ex1_dst0_reg == 5, "fclass.s: dst0_reg == rd (GPR dest, xvld-class default path)");

    reset_dut();
    present(fclass_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CLASS_BIT) & 1) == 1,
          "fclass.d: FUNC_DOUBLE+FUNC_CLASS bits set");
    test_result("T10c OP-FP compare/classify decode (M5 Task 4a): func bits correct + dispatch to EU_FP (M5 Task 9 swap)");
}

// ---- M5 Task 4b: OP-FP add/sub/minmax/sgnj-family/f2f-convert decode arms
// (IDU.v part b). Live as of THE SWAP (M5 Task 9): these now decode legal
// and dispatch to EU_FP, so fadd_sel (add/sub/minmax) / fspu_sel (sgnj) /
// fcnvt_sel (f2f-convert) fire per their func-bit groups (gated on
// ex1_eu_r[EU_FP_SEL]). Also covers the real idu_fpu_ex1_rm/idu_fpu_ex1_
// dst0_reg plumbing (RTU.v/RVProc.v wiring). ----
static void test_fp_arith_sgnj_cvt_decode(void) {
    reset_dut();
    present(fadd_s(0x5)); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_cp0_ex1_sel == 0
          && dut->idu_fpu_ex1_fadd_sel == 1,
          "fadd.s: legal, dispatches to EU_FP (fadd_sel fires) -- THE SWAP");
    check(((dut->idu_fpu_ex1_func >> FUNC_ADD_BIT) & 1) == 1,
          "fadd.s: FUNC_ADD bit set");
    check(dut->idu_fpu_ex1_rm == 0x5, "fadd.s: idu_fpu_ex1_rm == funct3 (rm field)",
          dut->idu_fpu_ex1_rm, 0x5);
    check(dut->idu_fpu_ex1_dst0_reg == 5, "fadd.s: idu_fpu_ex1_dst0_reg == rd (FRF dest tag)");
    check(dut->idu_fpu_ex1_fspu_sel == 0 && dut->idu_fpu_ex1_fcnvt_sel == 0,
          "fadd.s: fspu/fcnvt_sel stay 0 (FUNC_ADD is an fadd_sel-group op)");

    reset_dut();
    present(fadd_d(0x0)); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_ADD_BIT) & 1) == 1,
          "fadd.d: FUNC_DOUBLE+FUNC_ADD bits set");

    reset_dut();
    present(fsub_s(0x0)); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_SUB_BIT) & 1) == 1,
          "fsub.s: FUNC_SUB bit set");

    reset_dut();
    present(fsub_d(0x0)); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SUB_BIT) & 1) == 1,
          "fsub.d: FUNC_DOUBLE+FUNC_SUB bits set");

    reset_dut();
    present(fmin_s()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_MIN_BIT) & 1) == 1,
          "fmin.s: FUNC_MIN bit set");

    reset_dut();
    present(fmax_s()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_MAX_BIT) & 1) == 1,
          "fmax.s: FUNC_MAX bit set");

    reset_dut();
    present(fmin_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_MIN_BIT) & 1) == 1,
          "fmin.d: FUNC_DOUBLE+FUNC_MIN bits set");

    reset_dut();
    present(fmax_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_MAX_BIT) & 1) == 1,
          "fmax.d: FUNC_DOUBLE+FUNC_MAX bits set");

    reset_dut();
    present(fsgnj_s()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_J_BIT) & 1) == 1,
          "fsgnj.s: FUNC_SPU_SGN+FUNC_SPU_SGN_J bits set");

    reset_dut();
    present(fsgnjn_s()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_N_BIT) & 1) == 1,
          "fsgnjn.s: FUNC_SPU_SGN+FUNC_SPU_SGN_N bits set");

    reset_dut();
    present(fsgnjx_s()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_X_BIT) & 1) == 1,
          "fsgnjx.s: FUNC_SPU_SGN+FUNC_SPU_SGN_X bits set");

    reset_dut();
    present(fsgnj_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_J_BIT) & 1) == 1,
          "fsgnj.d: FUNC_DOUBLE+FUNC_SPU_SGN+FUNC_SPU_SGN_J bits set");

    reset_dut();
    present(fsgnjn_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_N_BIT) & 1) == 1,
          "fsgnjn.d: FUNC_DOUBLE+FUNC_SPU_SGN+FUNC_SPU_SGN_N bits set");

    reset_dut();
    present(fsgnjx_d()); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_SPU_SGN_X_BIT) & 1) == 1,
          "fsgnjx.d: FUNC_DOUBLE+FUNC_SPU_SGN+FUNC_SPU_SGN_X bits set");

    reset_dut();
    present(fcvt_s_d(0x0)); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_DOUBLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CVT_NARROW_BIT) & 1) == 1,
          "fcvt.s.d: FUNC_DOUBLE+FUNC_CVT_NARROW bits set (double->single)");

    reset_dut();
    present(fcvt_d_s(0x0)); tick(); present(0, false);
    check(((dut->idu_fpu_ex1_func >> FUNC_B_SINGLE_BIT) & 1) == 1
          && ((dut->idu_fpu_ex1_func >> FUNC_CVT_WIDDEN_BIT) & 1) == 1,
          "fcvt.d.s: FUNC_B_SINGLE+FUNC_CVT_WIDDEN bits set (single->double)");
    test_result("T10d OP-FP add/sub/minmax/sgnj/f2f-convert decode (M5 Task 4b): func bits + rm + FRF dst0_reg correct + dispatch to EU_FP (M5 Task 9 swap)");
}

// ---- M5 Task 4c: LOAD-FP/STORE-FP decode (D8: reuses EU_LSU, dst0_frf/
// src2_frf mark the FRF side instead of the GPR dst0_vld/src2_vld path).
// Live as of THE SWAP (M5 Task 9): these now dispatch to EU_LSU (lsu_sel
// fires) exactly like the integer loads/stores. ----
static void test_fp_ldst_decode(void) {
    reset_dut();
    write_gpr(1, 0x2000);
    present(flw_(5, 1, 8)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_sel == 1, "flw: lsu_sel fires (legal EU_LSU op) -- THE SWAP");
    check(dut->idu_lsu_ex1_func == LSU_FUNC_FLW, "flw: func == LSU_FUNC_FLW");
    check(dut->idu_lsu_ex1_src0_data == 0x2000, "flw: src0=x1(base)");
    check(dut->idu_lsu_ex1_src1_data == 8, "flw: src1=imm(offset)=8");
    check(dut->idu_lsu_ex1_dst0_reg == 5, "flw: dst0_reg==rd (FRF index, shares the bus)");
    check(dut->idu_lsu_ex1_dst0_frf == 1, "flw: dst0_frf==1 (D8 selector, not a GPR dest)");
    check(dut->idu_cp0_ex1_illegal == 0, "flw: legal (THE SWAP, D11)");

    reset_dut();
    write_gpr(1, 0x3000);
    present(fld_(6, 1, 16)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_func == LSU_FUNC_FLD, "fld: func == LSU_FUNC_FLD");
    check(dut->idu_lsu_ex1_dst0_reg == 6, "fld: dst0_reg==rd");
    check(dut->idu_lsu_ex1_dst0_frf == 1, "fld: dst0_frf==1");
    check(dut->idu_cp0_ex1_illegal == 0, "fld: legal (THE SWAP)");

    reset_dut();
    write_gpr(1, 0x4000);
    write_frf(2, 0x40091EB851EB851FULL); // 3.14 as a double bit pattern, held in f2
    present(fsw_(2, 1, 4)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_func == LSU_FUNC_FSW, "fsw: func == LSU_FUNC_FSW");
    check(dut->idu_lsu_ex1_src0_data == 0x4000, "fsw: src0=x1(base)");
    check(dut->idu_lsu_ex1_src1_data == 4, "fsw: src1=imm(offset)=4");
    check(dut->idu_lsu_ex1_src2_data == 0x40091EB851EB851FULL,
          "fsw: src2=f2 (FRF store-data, NOT a GPR read despite sharing rs2 field encoding)",
          dut->idu_lsu_ex1_src2_data, 0x40091EB851EB851FULL);
    check(dut->idu_cp0_ex1_illegal == 0, "fsw: legal (THE SWAP)");

    reset_dut();
    write_gpr(1, 0x5000);
    write_frf(3, 0xCAFEBABEDEADBEEFULL);
    present(fsd_(3, 1, 24)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_func == LSU_FUNC_FSD, "fsd: func == LSU_FUNC_FSD");
    check(dut->idu_lsu_ex1_src2_data == 0xCAFEBABEDEADBEEFULL,
          "fsd: src2=f3 (FRF store-data)", dut->idu_lsu_ex1_src2_data, 0xCAFEBABEDEADBEEFULL);
    check(dut->idu_cp0_ex1_illegal == 0, "fsd: legal (THE SWAP)");

    // A GPR write to the SAME numeric index as the FRF source register must
    // NOT leak into the FSW/FSD store-data slot -- confirms src2_frf really
    // overrides the GPR read path rather than merely coexisting with it.
    reset_dut();
    write_gpr(1, 0x6000);
    write_gpr(4, 0xBADBADBADULL);   // x4, same numeric index as f4 below
    write_frf(4, 0x1122334455667788ULL);
    present(fsd_(4, 1, 0)); tick(); present(0, false);
    check(dut->idu_lsu_ex1_src2_data == 0x1122334455667788ULL,
          "fsd: src2 reads f4, not the numerically-aliased x4 GPR value",
          dut->idu_lsu_ex1_src2_data, 0x1122334455667788ULL);

    test_result("T10e LOAD-FP/STORE-FP decode (M5 Task 4c): D8 EU_LSU reuse, dst0_frf/src2_frf route correctly, dispatch to EU_LSU (M5 Task 9 swap)");
}

// ---- M5 Task 9: remaining OP-FP families (FMAU/FDSU/fcvt-int/fmv, added in
// Tasks 5/6/7) now decode legal and dispatch to their FPU sub-block selects.
// These arms had no idu_tb decode coverage before the swap (verified only at
// the FPU.v datapath level via fpu_tb); this pins that the illegal gate
// flipped for them too, closing the decode-level loop on every FP family. ----
static void test_fp_swap_fmau_fdsu_cvt_mv_decode(void) {
    reset_dut();
    present(fmul_s(0x0)); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fmau_sel == 1,
          "fmul.s: legal, fmau_sel fires (THE SWAP)");

    reset_dut();
    present(fdiv_s(0x0)); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fdsu_sel == 1,
          "fdiv.s: legal, fdsu_sel fires (THE SWAP)");

    reset_dut();
    present(fsqrt_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fdsu_sel == 1,
          "fsqrt.s: legal, fdsu_sel fires (THE SWAP)");

    reset_dut();
    present(fmadd_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fmau_sel == 1,
          "fmadd.s: legal, fmau_sel fires (THE SWAP)");

    reset_dut();
    present(fcvt_w_s()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fcnvt_sel == 1,
          "fcvt.w.s (f2i): legal, fcnvt_sel fires (THE SWAP)");

    reset_dut();
    present(fcvt_s_w(0x0)); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fcnvt_sel == 1,
          "fcvt.s.w (i2f): legal, fcnvt_sel fires (THE SWAP)");

    reset_dut();
    present(fmv_x_w()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fspu_sel == 1,
          "fmv.x.w: legal, fspu_sel fires (THE SWAP)");

    reset_dut();
    present(fmv_w_x()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 0 && dut->idu_fpu_ex1_fspu_sel == 1,
          "fmv.w.x: legal, fspu_sel fires (THE SWAP)");

    test_result("T10f remaining OP-FP families (FMAU/FDSU/cvt-int/fmv) decode legal + dispatch (M5 Task 9 swap)");
}

// ---- 5.5: RVC pairs decode to the same EU/FUNC/*_vld shape as 32-bit twin ----
static void test_rvc_pairs(void) {
    reset_dut();
    // c.addi and addi must dispatch identically (same alu_sel path, same
    // final src1_data given the same effective immediate).
    write_gpr(5, 10);
    present(c_addi(5, 3)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.addi: alu_sel fires (same EU as addi)");
    check(dut->idu_iu_ex1_src0_data == 10, "c.addi: src0=x5=10");
    check(dut->idu_iu_ex1_src1_data == 3, "c.addi: src1=imm=3");
    check(dut->idu_iu_ex1_dst0_reg == 5, "c.addi: dst0=x5");

    present(c_li(6, 5)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.li: alu_sel fires (reuses ALU_FUNC_ADD)");
    check(dut->idu_iu_ex1_src1_data == 5, "c.li: src1=imm=5");
    check(dut->idu_iu_ex1_dst0_reg == 6, "c.li: dst0=x6");

    present(c_lui(7, 3)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.lui: alu_sel fires (reuses ALU_FUNC_LUI)");
    check(dut->idu_iu_ex1_src1_data == (uint64_t)(3ULL << 12), "c.lui: src1=imm<<12",
          dut->idu_iu_ex1_src1_data, 3ULL << 12);

    write_gpr(9 /*x9=x8+1*/, 0x77);
    present(c_andi(1 /*x9*/, 0x3F)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.andi: alu_sel fires (reuses ALU_FUNC_AND)");
    check(dut->idu_iu_ex1_src0_data == 0x77, "c.andi: src0=x9(rs1')");

    write_gpr(8, 0x100); write_gpr(9, 0x0F);
    present(c_sub(0 /*x8*/, 1 /*x9*/)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.sub: alu_sel fires (reuses ALU_FUNC_SUB)");
    check(dut->idu_iu_ex1_src0_data == 0x100, "c.sub: src0=x8(rd')");
    check(dut->idu_iu_ex1_src1_data == 0x0F, "c.sub: src1=x9(rs2')");

    write_gpr(11, 0x55);
    present(c_mv(10, 11)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.mv: alu_sel fires (reuses ALU_FUNC_ADD)");
    check(dut->idu_iu_ex1_src0_data == 0x55, "c.mv: src0=x11(value moved)");
    check(dut->idu_iu_ex1_dst0_reg == 10, "c.mv: dst0=x10");

    write_gpr(12, 3); write_gpr(13, 4);
    present(c_add(12, 13)); tick(); present(0, false);
    check(dut->idu_iu_ex1_alu_sel == 1, "c.add: alu_sel fires (reuses ALU_FUNC_ADD)");
    check(dut->idu_iu_ex1_src0_data == 3 && dut->idu_iu_ex1_src1_data == 4, "c.add: src0=x12,src1=x13");
    test_result("T11 RVC pairs decode to same EU/FUNC as 32-bit twin (contract 14)");
}

static void test_rvc_illegal(void) {
    reset_dut();
    // M8 T4c: c.fld/c.fsd/c.fldsp/c.fsdsp are now LEGAL (they were the
    // d16_illegal "no FP in M2" leftovers). c.fld decodes to the same
    // EU_LSU/LSU_FUNC_FLD as its 32-bit twin with dst0_frf=1 (D8).
    write_gpr(8, 0x1000);
    present(c_fld()); tick(); present(0, false);
    check(dut->idu_lsu_ex1_sel == 1, "c.fld: lsu_sel fires (legal EU_LSU op) -- M8 T4c");
    check(dut->idu_lsu_ex1_func == LSU_FUNC_FLD, "c.fld: func == LSU_FUNC_FLD");
    check(dut->idu_lsu_ex1_src0_data == 0x1000, "c.fld: src0=x8(base)");
    check(dut->idu_lsu_ex1_src1_data == 0, "c.fld: src1=imm=0");
    check(dut->idu_lsu_ex1_dst0_reg == 8, "c.fld: dst0=f8 (rd' 000 -> x8, FRF index)");
    check(dut->idu_lsu_ex1_dst0_frf == 1, "c.fld: dst0_frf==1 (D8 selector, not a GPR dest)");
    check(dut->idu_cp0_ex1_illegal == 0, "c.fld: legal (M8 T4c RVC FP decode)");

    present(c_addi4spn_bad()); tick(); present(0, false);
    check(dut->idu_cp0_ex1_illegal == 1, "c.addi4spn nzuimm=0: illegal (reserved encoding)");
    test_result("T12 RVC FP decode (c.fld legal, M8 T4c) + reserved-zero encodings illegal");
}

// ---- 5.5: WBT RAW/WAW except-clause matrix (each of the 5, individually
// and defeated) ----
static void test_wbt_except1_alu_bju_never_stall(void) {
    reset_dut();
    present(add_(3, 1, 2)); tick();      // creates WBT busy(x3, type=ALU)
    present(add_(4, 3, 0));               // consumer of x3, same-cycle-busy producer
    tick();
    check(dut->idu_ifu_id_stall == 0, "except1: ALU producer never stalls dependent consumer");
    present(0, false);
    test_result("T13 WBT except 1: ALU/BJU single-cycle producer exempts consumer");
}

static void test_wbt_except1_defeated_by_div(void) {
    reset_dut();
    present(div_(3, 1, 2)); tick();      // creates WBT busy(x3, type=OTHER/DIV -- no fast path)
    present(add_(4, 3, 0));
    tick();
    check(dut->idu_ifu_id_stall == 1, "except1 defeated: DIV producer (type OTHER) has no fast path");
    present(0, false);
    test_result("T14 WBT except 1 defeated: DIV producer gets no ALU/BJU fast-path exemption");
}

static void test_wbt_except2_lsu_to_condbr(void) {
    reset_dut();
    present(lw(3, 1, 0)); tick();         // creates WBT busy(x3, type=LSU, cnt=0)
    present(beq_(3, 0, 8));                // conditional-branch consumer of x3
    tick();
    check(dut->idu_ifu_id_stall == 0, "except2: LSU->cond-branch allowed through");
    present(0, false);
    test_result("T15 WBT except 2: LSU producer + BJU conditional-branch consumer exempted");
}

static void test_wbt_except2_defeated_non_condbr(void) {
    reset_dut();
    present(lw(3, 1, 0)); tick();
    present(add_(4, 3, 0));                // non-branch consumer of x3
    tick();
    check(dut->idu_ifu_id_stall == 1, "except2 defeated: LSU->ALU consumer must genuinely wait");
    present(0, false);
    test_result("T16 WBT except 2 defeated: non-branch consumer of an LSU producer really stalls");
}

static void test_wbt_except3_fwd_hit(void) {
    reset_dut();
    present(mul_(3, 1, 2)); tick();        // creates WBT busy(x3, type=MULT, cnt=0)
    present(0, false); tick();
    // force an RTU forward hit on x3 this cycle while WBT still shows busy
    dut->rtu_idu_fwd1_vld = 1; dut->rtu_idu_fwd1_reg = 3; dut->rtu_idu_fwd1_data = 0x99;
    present(add_(4, 3, 0));
    tick();
    check(dut->idu_ifu_id_stall == 0, "except3: same-cycle RTU forward satisfies readiness");
    dut->rtu_idu_fwd1_vld = 0;
    present(0, false);
    test_result("T17 WBT except 3: an RTU forward-bus hit this cycle exempts the consumer");
}

static void test_wbt_except3_defeated_2outstanding(void) {
    reset_dut();
    // 3 overlapping MULT creates to x3 (WAW except1 lets same-type MULT
    // producers pipeline without stalling dispatch) drive cnt to 2.
    present(mul_(3, 1, 2)); tick();
    present(mul_(3, 1, 2)); tick();
    present(mul_(3, 1, 2)); tick();
    present(0, false);
    // force fwd hit on x3 while cnt==2 -- except3's own negation term
    // (LSU/MULT producer with cnt==2) must NOT exempt this consumer.
    dut->rtu_idu_fwd1_vld = 1; dut->rtu_idu_fwd1_reg = 3; dut->rtu_idu_fwd1_data = 0x99;
    present(add_(4, 3, 0));
    tick();
    check(dut->idu_ifu_id_stall == 1, "except3 defeated: 2-outstanding MULT producer + fwd hit still stalls");
    dut->rtu_idu_fwd1_vld = 0;
    present(0, false);
    test_result("T18 WBT except 3 defeated: the 2-outstanding-producer corner case");
}

static void test_wbt_except4_store_data_from_load(void) {
    reset_dut();
    present(lw(3, 1, 0)); tick();          // creates WBT busy(x3, type=LSU, cnt=0)
    present(sw(3, 1, 0));                   // x3 used as STORE DATA (src2)
    // raw2_except's WB_INT_TYPE_LSU/dis_is_store term (IDU.v ~1269) exempts
    // THIS dispatch from the ordinary producer-busy stall -- observable here
    // because ctrl_ex1_stall still reflects the EX1-resident lw (not yet a
    // store), so no eval-triggering tick is needed to see the dispatch-time
    // decision in isolation.
    dut->eval();
    check(dut->idu_ifu_id_stall == 0, "except4: store-data-from-load dispatch exempted from RAW stall");
    tick();
    // sw is now EX1-resident with its src2 (x3) not yet ready. The front end
    // correctly holds it here -- ctrl_ex1_internal_stall's
    // ex1_store_src2_unrdy term (IDU.v ~1317) -- rather than letting LSU
    // capture a stale store-data operand, which was Root Cause #3 of the
    // rv64ui-p-ld_st data-corruption bug this session fixed. Donor
    // aq_lsu_ag.v:497,667,885-930 holds the analogous op via ag_src2_depd
    // until its own forward bus resolves it.
    check(dut->idu_ifu_id_stall == 1, "except4: EX1-resident store parks until its src2 is actually ready");
    // Producer's writeback lands (late-forward hit, lf2_hit) -- park releases.
    dut->rtu_idu_wb1_vld = 1; dut->rtu_idu_wb1_reg = 3; dut->rtu_idu_wb1_data = 0x77;
    tick();
    dut->rtu_idu_wb1_vld = 0;
    check(dut->idu_ifu_id_stall == 0, "except4: park releases once the late forward lands");
    check(dut->idu_lsu_ex1_src2_data == 0x77, "except4: forwarded store data reaches LSU's src2 port",
          dut->idu_lsu_ex1_src2_data, 0x77);
    present(0, false);
    test_result("T19 WBT except 4: LSU producer + store consumer's src2 (store data) exempted at dispatch, then EX1-parked until ready");
}

static void test_wbt_except4_defeated_base_reg(void) {
    reset_dut();
    present(lw(3, 1, 0)); tick();
    present(sw(1, 3, 0));                   // x3 used as BASE (src0), not store-data
    tick();
    check(dut->idu_ifu_id_stall == 1, "except4 defeated: LSU producer used as store BASE really stalls");
    present(0, false);
    test_result("T20 WBT except 4 defeated: src0 (base) is not covered by the src2-only exception");
}

static void test_waw_except_same_latency_class(void) {
    reset_dut();
    present(mul_(3, 1, 2)); tick();          // 1st MULT producer of x3
    present(mul_(3, 1, 2));                   // 2nd MULT producer of x3 (WAW)
    dut->eval();   // check THIS cycle's combinational decision, before a
                    // further tick lets the (still-presented) instruction
                    // be re-evaluated a 3rd time against the now-higher cnt
    check(dut->idu_ifu_id_stall == 0, "WAW except: same-latency-class (MULT/MULT) producers don't serialize");
    tick();
    present(0, false);
    test_result("T21 WAW except: same-latency-class WAW producers don't serialize dispatch");
}

static void test_waw_except_defeated(void) {
    reset_dut();
    present(lw(3, 1, 0)); tick();             // LSU producer of x3
    present(add_(3, 1, 2));                    // ALU producer, SAME dst -- different type, real WAW
    tick();
    check(dut->idu_ifu_id_stall == 1, "WAW defeated: LSU-then-ALU to the same dst really WAW-stalls");
    present(0, false);
    test_result("T22 WAW except defeated: different-class producers to the same register do serialize");
}

// ---- 5.5: GPR read/write, x0-hardwire, wb0==wb1 collision ----
static void test_gpr_basic_rw(void) {
    reset_dut();
    write_gpr(5, 0x1234);
    present(add_(6, 5, 0)); tick(); present(0, false);
    check(dut->idu_iu_ex1_src0_data == 0x1234, "gpr: read-back matches prior write", dut->idu_iu_ex1_src0_data, 0x1234);
    test_result("T23 GPR basic write-then-read");
}

static void test_gpr_x0_hardwire(void) {
    reset_dut();
    // attempt to write x0 -- must never actually change (it's always 0)
    dut->rtu_idu_wb0_reg = 0; dut->rtu_idu_wb0_data = 0xDEADBEEF; dut->rtu_idu_wb0_vld = 1;
    tick();
    dut->rtu_idu_wb0_vld = 0;
    present(add_(6, 0, 0)); tick(); present(0, false);
    check(dut->idu_iu_ex1_src0_data == 0, "gpr: x0 stays 0 even after an attempted write",
          dut->idu_iu_ex1_src0_data, 0);
    test_result("T24 GPR x0 hardwire: writes to x0 are always discarded");
}

static void test_gpr_wb0_eq_wb1_collision(void) {
    reset_dut();
    write_gpr(7, 0x11);   // baseline value
    // same-cycle wb0==wb1 on register 7 -- per gated_reg.v's own collision
    // case (no 2'b11 arm), the write must be DROPPED, not merged/prioritized.
    dut->rtu_idu_wb0_reg = 7; dut->rtu_idu_wb0_data = 0xAAAA; dut->rtu_idu_wb0_vld = 1;
    dut->rtu_idu_wb1_reg = 7; dut->rtu_idu_wb1_data = 0xBBBB; dut->rtu_idu_wb1_vld = 1;
    tick();
    dut->rtu_idu_wb0_vld = 0; dut->rtu_idu_wb1_vld = 0;
    present(add_(8, 7, 0)); tick(); present(0, false);
    check(dut->idu_iu_ex1_src0_data == 0x11, "gpr: wb0==wb1 collision drops the write, old value holds",
          dut->idu_iu_ex1_src0_data, 0x11);
    test_result("T25 GPR wb0==wb1 collision on one register: write silently dropped (matches donor)");
}

static void test_gpr_wb0_wb1_no_collision(void) {
    // sanity: wb0/wb1 to DIFFERENT registers both land correctly (this
    // never actually races in the real pipe -- RTU's one-hot completion
    // guarantee, Task 4.2 -- confirmed here as a non-collision baseline).
    reset_dut();
    dut->rtu_idu_wb0_reg = 9;  dut->rtu_idu_wb0_data = 0x9999; dut->rtu_idu_wb0_vld = 1;
    dut->rtu_idu_wb1_reg = 10; dut->rtu_idu_wb1_data = 0xAAAA; dut->rtu_idu_wb1_vld = 1;
    tick();
    dut->rtu_idu_wb0_vld = 0; dut->rtu_idu_wb1_vld = 0;
    present(add_(1, 9, 10)); tick(); present(0, false);
    check(dut->idu_iu_ex1_src0_data == 0x9999, "gpr: wb0->x9 landed");
    check(dut->idu_iu_ex1_src1_data == 0xAAAA, "gpr: wb1->x10 landed");
    test_result("T26 GPR wb0/wb1 to different registers: no collision, both land (never races in practice)");
}

// ---- 5.5: EU one-hot dispatch ----
static void test_eu_onehot_dispatch(void) {
    reset_dut();
    present(add_(3, 1, 2)); tick();
    check(dut->idu_iu_ex1_alu_sel == 1 && dut->idu_iu_ex1_bju_sel == 0
          && dut->idu_iu_ex1_mult_sel == 0 && dut->idu_iu_ex1_div_sel == 0
          && dut->idu_cp0_ex1_sel == 0 && dut->idu_lsu_ex1_sel == 0,
          "onehot: exactly ALU fires for an add");
    present(0, false);
    test_result("T27 EU one-hot dispatch: exactly one target selected per instruction");
}

// ---- 5.5: EX1 issue-gate hold-and-drain ----
static void test_ex1_issue_gate_commit0(void) {
    reset_dut();
    present(add_(3, 1, 2)); tick();      // latches into EX1
    present(0, false);
    dut->rtu_idu_commit = 0;
    dut->eval();
    check(dut->idu_iu_ex1_inst_vld == 1, "issue-gate: EX1 register still VALID with commit=0");
    check(dut->idu_iu_ex1_alu_sel == 0, "issue-gate: but NOT issuing (alu_sel=0) with commit=0");
    dut->rtu_idu_commit = 1;
    test_result("T28 EX1 issue-gate: commit=0 -> valid-but-not-issuing (not the same as invalid)");
}

static void test_ex1_issue_gate_full_holds_and_backpressures(void) {
    reset_dut();
    present(mul_(3, 1, 2)); tick();      // latches a MULT op into EX1
    present(0, false);
    dut->iu_idu_mult_full = 1;
    dut->eval();
    check(dut->idu_iu_ex1_mult_sel == 0, "full: mult_sel withheld while iu_idu_mult_full=1");
    check(dut->idu_ifu_id_stall == 1, "full: a stuck EX1 backpressures idu_ifu_id_stall");
    // present a DIFFERENT would-be instruction -- EX1 must NOT be replaced
    present(add_(9, 1, 2), true);
    tick();
    check(dut->idu_iu_ex1_mult_sel == 0 || dut->idu_iu_ex1_dst0_reg != 9,
          "full: EX1 did not silently replace the held MULT with the new dispatch");
    dut->iu_idu_mult_full = 0;
    present(0, false);
    dut->eval();   // check the SAME cycle mult_full clears -- the issue
                    // pulse is combinational; a further tick would already
                    // have advanced EX1 past it (nothing left to observe)
    check(dut->idu_iu_ex1_mult_sel == 1, "full: releasing iu_idu_mult_full lets the held op finally issue");
    tick();
    present(0, false);
    test_result("T29 EX1 issue-gate: <EU>_idu_full holds the instruction and backpressures the front end");
}

// ---- M5 Task 2: FRF read/write, f0 has NO hardwired-zero, wbf0==wbf1 collision ----
static void test_frf_basic_rw(void) {
    reset_dut();
    write_frf(5, 0x40091EB851EB851FULL); // 3.14 as a double bit pattern
    present(enc_fp_r4(0, 0, 0, 5, 0, 6)); tick(); present(0, false);
    check(dut->idu_fpu_ex1_fsrc0_data == 0x40091EB851EB851FULL,
          "frf: read-back matches prior write", dut->idu_fpu_ex1_fsrc0_data, 0x40091EB851EB851FULL);
    test_result("T30 FRF basic write-then-read (fsrc0 == rs1 field)");
}

static void test_frf_f0_not_hardwired(void) {
    reset_dut();
    // unlike GPR's x0, f0 is an architecturally real register -- a write
    // must stick, not be silently discarded.
    write_frf(0, 0xDEADBEEFCAFEBABEULL);
    present(enc_fp_r4(0, 0, 0, 0, 0, 6)); tick(); present(0, false);
    check(dut->idu_fpu_ex1_fsrc0_data == 0xDEADBEEFCAFEBABEULL,
          "frf: f0 holds a written value (no x0-style hardwired zero)",
          dut->idu_fpu_ex1_fsrc0_data, 0xDEADBEEFCAFEBABEULL);
    test_result("T31 FRF f0 is NOT hardwired to zero (deviation from GPR x0)");
}

static void test_frf_all_three_read_ports(void) {
    reset_dut();
    write_frf(1, 0x11);
    write_frf(2, 0x22);
    write_frf(3, 0x33);
    // rs1=1, rs2=2, rs3=3
    present(enc_fp_r4(3, 0, 2, 1, 0, 6)); tick(); present(0, false);
    check(dut->idu_fpu_ex1_fsrc0_data == 0x11, "frf: fsrc0 == rs1(f1)", dut->idu_fpu_ex1_fsrc0_data, 0x11);
    check(dut->idu_fpu_ex1_fsrc1_data == 0x22, "frf: fsrc1 == rs2(f2)", dut->idu_fpu_ex1_fsrc1_data, 0x22);
    check(dut->idu_fpu_ex1_fsrc2_data == 0x33, "frf: fsrc2 == rs3(f3)", dut->idu_fpu_ex1_fsrc2_data, 0x33);
    test_result("T32 FRF all three read ports (fsrc0/1/2 == rs1/rs2/rs3, fixed positions)");
}

static void test_frf_wbf0_eq_wbf1_collision(void) {
    reset_dut();
    write_frf(7, 0x11);   // baseline value
    // same-cycle wbf0==wbf1 on register f7 -- mirrors GPR's gated_reg.v
    // collision case (no 2'b11 arm): the write must be DROPPED.
    dut->rtu_idu_wbf0_reg = 7; dut->rtu_idu_wbf0_data = 0xAAAA; dut->rtu_idu_wbf0_vld = 1;
    dut->rtu_idu_wbf1_reg = 7; dut->rtu_idu_wbf1_data = 0xBBBB; dut->rtu_idu_wbf1_vld = 1;
    tick();
    dut->rtu_idu_wbf0_vld = 0; dut->rtu_idu_wbf1_vld = 0;
    present(enc_fp_r4(0, 0, 0, 7, 0, 6)); tick(); present(0, false);
    check(dut->idu_fpu_ex1_fsrc0_data == 0x11, "frf: wbf0==wbf1 collision drops the write, old value holds",
          dut->idu_fpu_ex1_fsrc0_data, 0x11);
    test_result("T33 FRF wbf0==wbf1 collision on one register: write silently dropped (matches GPR policy)");
}

static void test_frf_wbf0_wbf1_no_collision(void) {
    reset_dut();
    dut->rtu_idu_wbf0_reg = 9;  dut->rtu_idu_wbf0_data = 0x9999; dut->rtu_idu_wbf0_vld = 1;
    dut->rtu_idu_wbf1_reg = 10; dut->rtu_idu_wbf1_data = 0xAAAA; dut->rtu_idu_wbf1_vld = 1;
    tick();
    dut->rtu_idu_wbf0_vld = 0; dut->rtu_idu_wbf1_vld = 0;
    present(enc_fp_r4(0, 0, 10, 9, 0, 6)); tick(); present(0, false);
    check(dut->idu_fpu_ex1_fsrc0_data == 0x9999, "frf: wbf0->f9 landed");
    check(dut->idu_fpu_ex1_fsrc1_data == 0xAAAA, "frf: wbf1->f10 landed");
    test_result("T34 FRF wbf0/wbf1 to different registers: no collision, both land");
}

//=============================================================================
// Main
//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VIDU;

    reset_dut();
    test_reset_state();

    test_decode_32bit_alu();
    test_decode_32bit_addi();
    test_decode_32bit_lsu();
    test_decode_32bit_bju();
    test_decode_32bit_mult_div();
    test_decode_32bit_lui_auipc();
    test_decode_32bit_csr();
    test_decode_32bit_fence_ecall();
    test_illegal_closed_list();
    test_fetch_fault_forces_cp0();
    test_fp_cmp_class_decode();
    test_fp_arith_sgnj_cvt_decode();
    test_fp_ldst_decode();
    test_fp_swap_fmau_fdsu_cvt_mv_decode();
    test_rvc_pairs();
    test_rvc_illegal();

    test_wbt_except1_alu_bju_never_stall();
    test_wbt_except1_defeated_by_div();
    test_wbt_except2_lsu_to_condbr();
    test_wbt_except2_defeated_non_condbr();
    test_wbt_except3_fwd_hit();
    test_wbt_except3_defeated_2outstanding();
    test_wbt_except4_store_data_from_load();
    test_wbt_except4_defeated_base_reg();
    test_waw_except_same_latency_class();
    test_waw_except_defeated();

    test_gpr_basic_rw();
    test_gpr_x0_hardwire();
    test_gpr_wb0_eq_wb1_collision();
    test_gpr_wb0_wb1_no_collision();

    test_eu_onehot_dispatch();
    test_ex1_issue_gate_commit0();
    test_ex1_issue_gate_full_holds_and_backpressures();

    test_frf_basic_rw();
    test_frf_f0_not_hardwired();
    test_frf_all_three_read_ports();
    test_frf_wbf0_eq_wbf1_collision();
    test_frf_wbf0_wbf1_no_collision();

    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");
    delete dut;
    return g_fail ? 1 : 0;
}
