//=============================================================================
// lr_sc_tb.cpp - standalone unit bench for rtl/LSU.v LR.W/SC.W foundation (M3 task 1)
//=============================================================================
// Verilates LSU.v + the DCache.v it instantiates internally + SRAM.v +
// rvproc_pkg.sv (no IDU, no RTU, no CSR, no MMU.v, no SoC) and drives the
// frozen idu_lsu_ex1_*/rtu_lsu_*/cp0_lsu_*/mmu_lsu_* ports directly.
//
// Tests LR.W (load-reserved) load-buffer foundation and SC.W (store-conditional)
// address-match gate. This is white-box verification of LSU.v's own contract
// for LR/SC interlocks.
//
// Build/run: make -C test/m2/unit lr_sc && bin/unit/lr_sc_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VLSU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <unordered_map>

//-----------------------------------------------------------------------------
// FUNC encodings (rvproc_pkg.sv LSU_FUNC_*, confirmed bit-exact).
// Note: OP_LR/OP_SC will be added later during Task 3/4; for now we use
// reserved opcodes to exercise the LR/SC datapath stubs.
//-----------------------------------------------------------------------------
static const uint32_t F_LB  = 0x00302;
static const uint32_t F_LH  = 0x00306;
static const uint32_t F_LW  = 0x0030a;
static const uint32_t F_LD  = 0x0030e;
static const uint32_t F_SB  = 0x00301;
static const uint32_t F_SH  = 0x00305;
static const uint32_t F_SW  = 0x00309;
static const uint32_t F_SD  = 0x0030d;

//-----------------------------------------------------------------------------
// Dummy opcode constants for LR/SC stubs (to be wired properly in LSU.v fix)
//-----------------------------------------------------------------------------
static const uint32_t F_LR  = 0x00b08;  // M3 Task 1: LR.W (func[0]=0 -> load-like)
static const uint32_t F_SC  = 0x00b0c;  // M3 Task 1: SC.W (func[0]=0 -> load-like)

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VLSU *dut = nullptr;
static uint64_t g_cycles = 0;

static int g_cp0_lsu_mm         = 1;   // reset default per contract 3
static int g_cp0_lsu_wa         = 0;   // reset default per contract 6
static int g_cp0_lsu_dcache_en  = 1;   // this bench always runs "post-boot"

//-----------------------------------------------------------------------------
// Golden memory (sparse -- test addresses are deliberately scattered).
//-----------------------------------------------------------------------------
static std::unordered_map<uint64_t, uint8_t> g_mem;

static uint8_t mem_rd(uint64_t addr)
{
    auto it = g_mem.find(addr);
    return (it == g_mem.end()) ? 0 : it->second;
}
static void mem_wr(uint64_t addr, uint8_t v) { g_mem[addr] = v; }

static uint64_t mem_read64(uint64_t addr)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)mem_rd(addr + i) << (i * 8);
    return v;
}

//-----------------------------------------------------------------------------
// Behavioural AXI D-side slave: AWREADY/WREADY/ARREADY hardwired 1 (no
// backpressure -- this bench is not stress-testing AXI arbitration), a
// fixed 2-cycle latency to BVALID/RVALID, always a full 64B-aligned single
// beat (matches LSU.v's own convention: contract 17's 16-beat cap is
// trivially satisfied).
//-----------------------------------------------------------------------------
struct AxiDSlave {
    bool     b_pending = false; int b_cnt = 0;
    bool     r_pending = false; int r_cnt = 0; uint64_t r_addr = 0;
    int      reads = 0, writes = 0;
    uint64_t last_awaddr = 0, last_araddr = 0;

    void reset() { b_pending = false; r_pending = false; reads = 0; writes = 0; }

    void step(VLSU *d)
    {
        if (!b_pending && d->axi_d_awvalid && d->axi_d_wvalid) {
            uint64_t addr = d->axi_d_awaddr;
            uint64_t strb = d->axi_d_wstrb;
            for (int i = 0; i < 64; i++) {
                if ((strb >> i) & 1ULL) {
                    uint32_t word = d->axi_d_wdata[i / 4];
                    uint8_t byte = (uint8_t)((word >> ((i % 4) * 8)) & 0xFF);
                    mem_wr(addr + i, byte);
                }
            }
            last_awaddr = addr;
            writes++;
            b_pending = true; b_cnt = 2;
        } else if (b_pending && b_cnt > 0) {
            b_cnt--;
        }

        if (!r_pending && d->axi_d_arvalid) {
            r_addr = d->axi_d_araddr;
            last_araddr = r_addr;
            reads++;
            r_pending = true; r_cnt = 2;
        } else if (r_pending && r_cnt > 0) {
            r_cnt--;
        }
    }

    void drive(VLSU *d)
    {
        d->axi_d_awready = 1;
        d->axi_d_wready  = 1;
        bool bv = (b_pending && b_cnt == 0);
        d->axi_d_bvalid  = bv ? 1 : 0;
        d->axi_d_bresp   = 0;
        if (bv) b_pending = false;

        d->axi_d_arready = 1;
        bool rv = (r_pending && r_cnt == 0);
        d->axi_d_rvalid  = rv ? 1 : 0;
        d->axi_d_rresp   = 0;
        d->axi_d_rlast   = rv ? 1 : 0;
        if (rv) {
            for (int wi = 0; wi < 16; wi++) {
                uint32_t w = 0;
                for (int b = 0; b < 4; b++)
                    w |= ((uint32_t)mem_rd(r_addr + wi * 4 + b)) << (b * 8);
                d->axi_d_rdata[wi] = w;
            }
            r_pending = false;
        }
    }
};
static AxiDSlave g_slave;

//-----------------------------------------------------------------------------
// MMU stub: identity map + PMA cacheability (DRAM >= 0x8000_0000)
//-----------------------------------------------------------------------------
static void drive_mmu(void)
{
    uint64_t ppn = dut->lsu_mmu_va;   // PAGE NUMBER (ag_addr[63:12])
    dut->mmu_lsu_pa           = (uint32_t)(ppn & 0x0FFFFFFFULL);
    dut->mmu_lsu_pa_vld       = 1;
    bool cacheable            = ((ppn << 12) >= 0x0000000080000000ULL);
    dut->mmu_lsu_ca           = cacheable ? 1 : 0;
    dut->mmu_lsu_so           = cacheable ? 0 : 1;
    dut->mmu_lsu_buf          = cacheable ? 1 : 0;
    dut->mmu_lsu_sec          = 0;
    dut->mmu_lsu_sh           = 0;
    dut->mmu_lsu_page_fault   = 0;
    dut->mmu_lsu_access_fault = 0;
}

static void drive_csr(void)
{
    dut->cp0_lsu_dcache_en = g_cp0_lsu_dcache_en;
    dut->cp0_lsu_mm        = g_cp0_lsu_mm;
    dut->cp0_lsu_wa        = g_cp0_lsu_wa;
    dut->rtu_lsu_expt_ack  = 0;
    dut->rtu_lsu_expt_exit = 0;
}

static void idle_issue(void)
{
    dut->idu_lsu_ex1_dp_sel     = 0;
    dut->idu_lsu_ex1_sel        = 0;
    dut->idu_lsu_ex1_func       = 0;
    dut->idu_lsu_ex1_src0_data  = 0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = 0;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = 0;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = 0;
}

static void tick(void)
{
    drive_csr();
    dut->eval();
    drive_mmu();
    dut->eval();
    g_slave.step(dut);
    dut->clk = 1;
    dut->eval();
    drive_csr();
    drive_mmu();
    dut->eval();
    g_slave.drive(dut);
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
}

static void settle(int n) { for (int i = 0; i < n; i++) { idle_issue(); tick(); } }

static void reset_dut(void)
{
    dut->clk   = 0;
    dut->rst_n = 0;
    idle_issue();
    drive_csr();
    dut->mmu_lsu_pa = 0; dut->mmu_lsu_pa_vld = 0; dut->mmu_lsu_ca = 0; dut->mmu_lsu_so = 1;
    dut->mmu_lsu_buf = 0; dut->mmu_lsu_sec = 0; dut->mmu_lsu_sh = 0;
    dut->mmu_lsu_page_fault = 0; dut->mmu_lsu_access_fault = 0;
    dut->axi_d_awready = 0; dut->axi_d_wready = 0; dut->axi_d_bvalid = 0; dut->axi_d_bresp = 0;
    dut->axi_d_arready = 0; dut->axi_d_rvalid = 0; dut->axi_d_rresp = 0; dut->axi_d_rlast = 0;
    for (int i = 0; i < 16; i++) dut->axi_d_rdata[i] = 0;
    g_slave.reset();
    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

//=============================================================================
// Result bookkeeping
//=============================================================================
static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0)
{
    if (!cond) {
        g_local++;
        if (g_fail < 60)
            printf("    FAIL %-64s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name)
{
    printf("[lr_sc_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// Operation driver
//=============================================================================
struct LsuResult {
    bool     cmplt = false, wb_vld = false, expt_vld = false, lr_vld = false, sc_res = false;
    uint64_t wb_data = 0;
    unsigned wb_preg = 0, expt_vec = 0;
    uint64_t tval = 0;
    int      cycles = 0;
    bool     timed_out = false;
};

static LsuResult do_op(uint32_t func, uint64_t src0, uint64_t src1, uint64_t src2,
                       unsigned dst0, bool sel = true, bool dp_sel = true, int guard = 500)
{
    int waited = 0;
    while (dut->lsu_idu_full && waited < guard) { idle_issue(); tick(); waited++; }

    dut->idu_lsu_ex1_dp_sel     = dp_sel ? 1 : 0;
    dut->idu_lsu_ex1_sel        = sel ? 1 : 0;
    dut->idu_lsu_ex1_func       = func;
    dut->idu_lsu_ex1_src0_data  = src0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = src1;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = src2;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = dst0;
    tick();
    idle_issue();

    LsuResult r;
    for (int i = 0; i < guard; i++) {
        if (dut->lsu_rtu_ex1_cmplt_dp) {
            r.cmplt    = true;
            r.wb_vld   = dut->lsu_rtu_wb_vld != 0;
            r.wb_data  = dut->lsu_rtu_wb_data;
            r.wb_preg  = dut->lsu_rtu_wb_preg;
            r.expt_vld = dut->lsu_rtu_expt_vld != 0;
            r.expt_vec = dut->lsu_rtu_expt_vec;
            r.tval     = dut->lsu_rtu_tval;
            r.cycles   = i + 1;
            // LR/SC outputs (if implemented)
            r.lr_vld   = dut->lsu_rtu_lr_vld != 0;
            r.sc_res   = dut->lsu_rtu_sc_res != 0;
            tick();
            return r;
        }
        tick();
    }
    r.timed_out = true;
    return r;
}

//=============================================================================
// Tests
//=============================================================================

// T1: LR.W basic path - verify that issuing an LR creates a load-buffer entry
// and returns data. STUB FOR M3 TASK 1.
static void test_lr_w_basic(void)
{
    const uint64_t A = 0x0000000080010000ULL;

    // Initialize memory at A with known pattern (little-endian layout)
    for (int i = 0; i < 8; i++) mem_wr(A + i, (uint8_t)(0xAA + i));

    // Expected loaded value: bytes [AA, AB, AC, AD] -> 0xADACABAA LE
    const uint64_t EXPECT_DATA = 0xADACABAAULL;

    // Issue LR.W
    LsuResult lr = do_op(F_LR, A, 0, 0, 5);

    // Check if the instruction completed
    check(lr.cmplt, "LR.W completed", lr.cmplt, 1);

    // Check that we got data back (this confirms LSU executed the op)
    check(lr.wb_data == EXPECT_DATA, "LR returned expected data pattern",
          lr.wb_data, EXPECT_DATA);

    // LR.VLD should now be asserted (the core feature under test)
    check(lr.lr_vld == 1, "LR.W should assert lr_vld after completion", lr.lr_vld, 1);

    test_result("T1 LR.W basic path");
}

// T2: SC.W success - store-conditional at same address where LR fired should commit
static void test_sc_w_success(void)
{
    const uint64_t A = 0x0000000080020000ULL;

    // Init memory at target address
    for (int i = 0; i < 4; i++) mem_wr(A + i, 0xAA);

    // First, LR.W
    do_op(F_LR, A, 0, 0, 5);
    settle(10);

    // Now attempt SC.W with new data
    // Note: for stores, src0=address, src1=offset, src2=data
    LsuResult sc = do_op(F_SC, A, 0, 0x11223344, 0);

    // SC at matching address should succeed (sc_res=0)
    check(sc.sc_res == 0, "SC.W at matching address should assert sc_res=0 (success)", sc.sc_res, 0);

    test_result("T2 SC.W success path");
}

// T3: SC.W failure due to address mismatch - LR at addr_A, SC at addr_B should fail
static void test_sc_w_address_mismatch(void)
{
    const uint64_t ADDR_A = 0x0000000080030000ULL;
    const uint64_t ADDR_B = 0x0000000080030004ULL;

    // Init both locations
    for (int i = 0; i < 4; i++) mem_wr(ADDR_A + i, 0xAA);
    for (int i = 0; i < 4; i++) mem_wr(ADDR_B + i, 0xBB);

    // LR at addr_A
    do_op(F_LR, ADDR_A, 0, 0, 5);
    settle(10);

    // SC at addr_B (should fail due to address mismatch)
    // Note: for stores, src0=address, src1=offset, src2=data
    LsuResult sc = do_op(F_SC, ADDR_B, 0, 0x11223344, 0);

    // SC at non-matching address should fail (sc_res=1)
    check(sc.sc_res == 1, "SC.W at non-matching address should assert sc_res=1 (fail)", sc.sc_res, 1);

    test_result("T3 SC.W address mismatch");
}

// T4: AMOSWAP.W - atomic swap: returns OLD value, stores NEW value to memory.
// W-width (32-bit) AMO operates on the lower 32 bits of the aligned dword.
static void test_amo_swap_w(void)
{
    const uint64_t A = 0x0000000080040000ULL;
    const uint32_t INIT_VAL = 0xDEADBEEF;   // 32-bit for W-width AMO
    const uint32_t SWAP_VAL = 0x11223344;
    // AMOSWAP.W opcode (func[0]=0 -> load-like for read phase)
    const uint32_t F_AMOSWAP_W = 0x01018;

    // Initialize memory with known 32-bit value
    for (int i = 0; i < 4; i++) mem_wr(A + i, (uint8_t)(INIT_VAL >> (i * 8)));

    // Issue AMOSWAP.W: src0=address, src1=offset, src2=swap value
    LsuResult amo = do_op(F_AMOSWAP_W, A, 0, SWAP_VAL, 5);
    settle(30);  // let the AMO writeback STB entry drain into the cache

    // Check that the OLD value was returned in wb_data (W-width returns
    // the 32-bit value; LSU sign/zero-extends per dc_size_r).
    check((amo.wb_data & 0xFFFFFFFF) == INIT_VAL,
          "AMOSWAP.W returns OLD value (lower 32b)", amo.wb_data & 0xFFFFFFFF, INIT_VAL);

    // Read back the NEW value via a load through the DUT (cache hit on the
    // line the AMO writeback updated)
    LsuResult ld = do_op(F_LW, A, 0, 0, 5);
    check((ld.wb_data & 0xFFFFFFFF) == SWAP_VAL,
          "AMOSWAP.W stores NEW value to memory (lower 32b)", ld.wb_data & 0xFFFFFFFF, SWAP_VAL);

    test_result("T4 AMOSWAP.W read-modify-write");
}

// Build check-label strings (static buffers, test is single-threaded)
static const char *name_old_ok(const char *n) {
    static char buf[128]; snprintf(buf, sizeof(buf), "%s returns OLD", n); return buf;
}
static const char *name_new_ok(const char *n) {
    static char buf[128]; snprintf(buf, sizeof(buf), "%s stores NEW", n); return buf;
}

// Helper: run a W-width AMO op and verify OLD returned + NEW stored.
// The AMO writeback updates the resident cache line, so the NEW value is
// read back via a load through the DUT (which hits the updated cache line),
// not from the golden memory (which the dirty line hasn't reached yet).
// Each call uses a DISTINCT address (its own cache line) so a prior test's
// resident line doesn't shadow the mem_wr re-init below.
static bool run_amo_w_check(const char *name, uint32_t func, uint64_t A,
                            uint32_t init_val, uint32_t rs1, uint32_t expect_new)
{
    for (int i = 0; i < 4; i++) mem_wr(A + i, (uint8_t)(init_val >> (i * 8)));

    LsuResult amo = do_op(func, A, 0, rs1, 5);
    settle(30);  // let the AMO writeback STB entry drain into the cache

    bool old_ok = (amo.wb_data & 0xFFFFFFFF) == init_val;

    // Read back the NEW value via a load through the DUT (cache hit)
    LsuResult ld = do_op(F_LW, A, 0, 0, 5);
    bool new_ok = (ld.wb_data & 0xFFFFFFFF) == expect_new;

    check(old_ok, name_old_ok(name), amo.wb_data & 0xFFFFFFFF, init_val);
    check(new_ok, name_new_ok(name), ld.wb_data & 0xFFFFFFFF, expect_new);
    return old_ok && new_ok;
}

// T5: All W-width AMO arithmetic/logic operations (each on its own cache line)
static void test_amo_w_ops(void)
{
    // AMO opcodes (W-width, func[0]=0)
    const uint32_t F_AMOADD_W  = 0x01008;
    const uint32_t F_AMOXOR_W  = 0x01048;
    const uint32_t F_AMOAND_W  = 0x010c8;
    const uint32_t F_AMOOR_W   = 0x01088;
    const uint32_t F_AMOMIN_W  = 0x01108;
    const uint32_t F_AMOMINU_W = 0x01188;
    const uint32_t F_AMOMAX_W  = 0x01148;
    const uint32_t F_AMOMAXU_W = 0x011c8;

    // Distinct cache-line-aligned addresses (64B apart = distinct sets)
    const uint64_t BASE = 0x0000000080060000ULL;

    // AMOADD.W: NEW = OLD + rs1
    run_amo_w_check("AMOADD.W", F_AMOADD_W, BASE + 0x000, 0x10000000, 0x02345678, 0x12345678);
    // AMOXOR.W: NEW = OLD ^ rs1
    run_amo_w_check("AMOXOR.W", F_AMOXOR_W, BASE + 0x040, 0xFF00FF00, 0x0F0F0F0F, 0xF00FF00F);
    // AMOAND.W: NEW = OLD & rs1
    run_amo_w_check("AMOAND.W", F_AMOAND_W, BASE + 0x080, 0xFF00FF00, 0x0F0F0F0F, 0x0F000F00);
    // AMOOR.W:  NEW = OLD | rs1
    run_amo_w_check("AMOOR.W",  F_AMOOR_W,  BASE + 0x0C0, 0xFF00FF00, 0x0F0F0F0F, 0xFF0FFF0F);
    // AMOMIN.W (signed): OLD=+5, rs1=-3 -> NEW=-3 (0xFFFFFFFD)
    run_amo_w_check("AMOMIN.W", F_AMOMIN_W, BASE + 0x100, 0x00000005, 0xFFFFFFFD, 0xFFFFFFFD);
    // AMOMAX.W (signed): OLD=+5, rs1=-3 -> NEW=+5
    run_amo_w_check("AMOMAX.W", F_AMOMAX_W, BASE + 0x140, 0x00000005, 0xFFFFFFFD, 0x00000005);
    // AMOMINU.W (unsigned): OLD=5, rs1=0xFFFFFFFD(large) -> NEW=5
    run_amo_w_check("AMOMINU.W", F_AMOMINU_W, BASE + 0x180, 0x00000005, 0xFFFFFFFD, 0x00000005);
    // AMOMAXU.W (unsigned): OLD=5, rs1=0xFFFFFFFD(large) -> NEW=0xFFFFFFFD
    run_amo_w_check("AMOMAXU.W", F_AMOMAXU_W, BASE + 0x1C0, 0x00000005, 0xFFFFFFFD, 0xFFFFFFFD);

    test_result("T5 AMO.W ops (add/xor/and/or/min/max/minu/maxu)");
}

// T6: D-width (64-bit) AMO operations
static void test_amo_d_ops(void)
{
    const uint32_t F_AMOADD_D  = 0x0100c;
    const uint32_t F_AMOSWAP_D = 0x0101c;
    const uint32_t F_AMOAND_D  = 0x010cc;
    const uint64_t BASE = 0x0000000080070000ULL;

    // AMOSWAP.D: NEW = rs1 (full 64-bit)
    {
        const uint64_t A = BASE + 0x000;
        const uint64_t INIT = 0xDEADBEEFCAFEBABEULL, RS1 = 0x1122334455667788ULL;
        for (int i = 0; i < 8; i++) mem_wr(A + i, (uint8_t)(INIT >> (i * 8)));
        LsuResult amo = do_op(F_AMOSWAP_D, A, 0, RS1, 5);
        settle(30);
        LsuResult ld = do_op(F_LD, A, 0, 0, 5);
        check(amo.wb_data == INIT, "AMOSWAP.D returns OLD (64b)", amo.wb_data, INIT);
        check(ld.wb_data == RS1,  "AMOSWAP.D stores NEW (64b)", ld.wb_data, RS1);
    }
    // AMOADD.D: NEW = OLD + rs1 (full 64-bit)
    {
        const uint64_t A = BASE + 0x040;
        const uint64_t INIT = 0x1000000000000000ULL, RS1 = 0x023456789ABCDEF0ULL;
        for (int i = 0; i < 8; i++) mem_wr(A + i, (uint8_t)(INIT >> (i * 8)));
        LsuResult amo = do_op(F_AMOADD_D, A, 0, RS1, 5);
        settle(30);
        LsuResult ld = do_op(F_LD, A, 0, 0, 5);
        check(amo.wb_data == INIT, "AMOADD.D returns OLD (64b)", amo.wb_data, INIT);
        check(ld.wb_data == (INIT + RS1), "AMOADD.D stores NEW (64b)", ld.wb_data, INIT + RS1);
    }
    // AMOAND.D: NEW = OLD & rs1
    {
        const uint64_t A = BASE + 0x080;
        const uint64_t INIT = 0xFF00FF00FF00FF00ULL, RS1 = 0x0F0F0F0F0F0F0F0FULL;
        for (int i = 0; i < 8; i++) mem_wr(A + i, (uint8_t)(INIT >> (i * 8)));
        LsuResult amo = do_op(F_AMOAND_D, A, 0, RS1, 5);
        settle(30);
        LsuResult ld = do_op(F_LD, A, 0, 0, 5);
        check(amo.wb_data == INIT, "AMOAND.D returns OLD (64b)", amo.wb_data, INIT);
        check(ld.wb_data == (INIT & RS1), "AMOAND.D stores NEW (64b)", ld.wb_data, INIT & RS1);
    }

    test_result("T6 AMO.D ops (swap/add/and, 64-bit)");
}

// T7: AMO after a store to the same address (reproduces the riscv-test
// amoadd_w scenario: memory initialized via a store, not a direct write).
static void test_amo_after_store(void)
{
    const uint64_t A = 0x0000000080080000ULL;
    const uint32_t INIT_VAL  = 0x80000000;   // bit31 set -> needs sign-ext
    const uint32_t SWAP_VAL  = 0x11223344;
    const uint32_t F_AMOSWAP_W = 0x01018;

    // Store INIT_VAL to memory (like riscv-test's `sw a0, 0(a3)`)
    do_op(F_SW, A, 0, INIT_VAL, 0);
    settle(30);

    // AMO reads the value the store wrote
    LsuResult amo = do_op(F_AMOSWAP_W, A, 0, SWAP_VAL, 5);
    settle(30);

    check(amo.wb_data == 0xffffffff80000000ULL,
          "AMO after store returns sign-extended OLD", amo.wb_data, 0xffffffff80000000ULL);

    // Read back the NEW value via a load through the DUT
    LsuResult ld = do_op(F_LW, A, 0, 0, 5);
    check((ld.wb_data & 0xFFFFFFFF) == SWAP_VAL,
          "AMO after store stored NEW value", ld.wb_data & 0xFFFFFFFF, SWAP_VAL);

    test_result("T7 AMO after store (store-initialized memory)");
}

// T8: AMO immediately after a store to the same address, NO settle between.
// This is the exact riscv-test amoadd_w timing: the store's data may still
// be in the STB when the AMO reads, so the AMO must forward from the STB.
static void test_amo_immediately_after_store(void)
{
    const uint64_t A = 0x0000000080090000ULL;
    const uint32_t INIT_VAL  = 0x80000000;
    const uint32_t SWAP_VAL  = 0x11223344;
    const uint32_t F_AMOSWAP_W = 0x01018;

    // Store then AMO back-to-back (do_op waits for completion but NOT for
    // the STB entry to drain -- mimics the SoC pipeline).
    do_op(F_SW, A, 0, INIT_VAL, 0);
    LsuResult amo = do_op(F_AMOSWAP_W, A, 0, SWAP_VAL, 5);
    settle(30);

    check(amo.wb_data == 0xffffffff80000000ULL,
          "AMO right after store forwards STB data (sign-ext OLD)",
          amo.wb_data, 0xffffffff80000000ULL);

    LsuResult ld = do_op(F_LW, A, 0, 0, 5);
    check((ld.wb_data & 0xFFFFFFFF) == SWAP_VAL,
          "AMO right after store stored NEW value", ld.wb_data & 0xFFFFFFFF, SWAP_VAL);

    test_result("T8 AMO immediately after store (tight STB forward)");
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    dut = new VLSU;

    reset_dut();

    test_lr_w_basic();
    test_sc_w_success();
    test_sc_w_address_mismatch();
    test_amo_swap_w();
    test_amo_w_ops();
    test_amo_d_ops();
    test_amo_after_store();
    test_amo_immediately_after_store();

    printf("[lr_sc_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
