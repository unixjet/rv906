//=============================================================================
// dm_tb.cpp - standalone unit bench for rtl/TDT_DM.v (M7 Task 3)
//=============================================================================
// Verilates TDT_DM alone (no DTM, no core, no SoC) and drives it with:
//   - a C++ APB master (setup -> access -> wait pready)
//   - a fake core responder (halted after N cycles on halt_req; itr_done
//     after M cycles on itr_vld; shadow dscratch0 on wr_vld/wr_flg)
//   - a fake AXI slave (128-byte memory; single-beat; ready=1; bresp/rresp
//     manually forceable)
// Prints PASS/FAIL per row and exits non-zero on any failure.
// Build/run: make -C test/m7/unit dm && bin/unit/dm_tb
//=============================================================================

#include <verilated.h>
#include "VTDT_DM.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>

//-----------------------------------------------------------------------------
// DM register word offsets (spec 0.13)
//-----------------------------------------------------------------------------
static const uint32_t ADDR_DATA0       = 0x04;
static const uint32_t ADDR_DATA1       = 0x05;
static const uint32_t ADDR_DMCONTROL   = 0x10;
static const uint32_t ADDR_DMSTATUS    = 0x11;
static const uint32_t ADDR_HARTINFO    = 0x12;
static const uint32_t ADDR_HAWINDOW    = 0x15;
static const uint32_t ADDR_ABSTRACTCS  = 0x16;
static const uint32_t ADDR_COMMAND     = 0x17;
static const uint32_t ADDR_ABSTRACTAUTO= 0x18;
static const uint32_t ADDR_NEXTDM      = 0x1d;
static const uint32_t ADDR_ITR         = 0x1f;
static const uint32_t ADDR_PB0         = 0x20;
static const uint32_t ADDR_PB1         = 0x21;
static const uint32_t ADDR_PB2         = 0x22;
static const uint32_t ADDR_PB3         = 0x23;
static const uint32_t ADDR_DMCS2       = 0x32;
static const uint32_t ADDR_SBCS        = 0x38;
static const uint32_t ADDR_SBADDR0     = 0x39;
static const uint32_t ADDR_SBADDR1     = 0x3a;
static const uint32_t ADDR_SBDATA0     = 0x3c;
static const uint32_t ADDR_SBDATA1     = 0x3d;
static const uint32_t ADDR_SBDATA2     = 0x3e;
static const uint32_t ADDR_SBDATA3     = 0x3f;
static const uint32_t ADDR_HALTSUM0    = 0x40;
static const uint32_t ADDR_CUSCS       = 0x70;
static const uint32_t ADDR_COMPID      = 0x7f;

// ITR encodings (donor tdt_dm.v:2280-2324)
static uint32_t enc_csrrw(uint32_t csr, uint32_t rs1, uint32_t rd) {
    return (csr << 20) | (rs1 << 15) | (1u << 12) | (rd << 7) | 0x73;
}
static uint32_t enc_csrrc(uint32_t csr, uint32_t rs1, uint32_t rd) {
    return (csr << 20) | (rs1 << 15) | (3u << 12) | (rd << 7) | 0x73;
}
static const uint32_t DSCR0 = 0x7b2;
static const uint32_t DSCR1 = 0x7b3;
static const uint32_t EBREAK_INST = 0x00100073;

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VTDT_DM *dut = nullptr;
static uint64_t g_cycles = 0;

// Fake core state
static int      g_halt_delay   = 8;
static int      g_halt_cnt     = -1;
static int      g_itr_delay    = 4;
static int      g_itr_cnt      = -1;
static int      g_wr_delay     = 2;
static int      g_wr_cnt       = -1;
static uint64_t g_dscratch0    = 0;
static uint64_t g_rx_override  = 0;
static bool     g_rx_use_override = false;
static bool     g_resume_drop  = false;

// itr log
static uint32_t g_itr_log[64];
static int      g_itr_count = 0;

// wr log
static uint64_t g_wr_data_log[16];
static uint8_t  g_wr_flg_log[16];
static int      g_wr_count = 0;

// Fake AXI state
static uint8_t  g_axi_mem[128];
static uint64_t g_axi_awaddr = 0;
static uint8_t  g_axi_awsize = 0;
static int      g_axi_bresp_force = -1;
static int      g_axi_rresp_force = -1;
static bool     g_axi_write_done  = false;
static bool     g_axi_read_done   = false;
static uint64_t g_axi_read_addr   = 0;
static uint8_t  g_axi_read_size   = 0;
static uint16_t g_axi_wstrb_seen  = 0;
static bool     g_axi_issued      = false;  // AW or AR handshake seen
static bool     g_axi_op_complete = false;  // B or R handshake consumed

static void tie_idle_inputs(void) {
    dut->dm_paddr    = 0;
    dut->dm_pwrite   = 0;
    dut->dm_psel     = 0;
    dut->dm_penable  = 0;
    dut->dm_pwdata   = 0;

    dut->core_dm_halted_i       = 0;
    dut->core_dm_havereset_i    = 0;
    dut->core_dm_itr_done_i     = 0;
    dut->core_dm_retire_debug_expt_i = 0;
    dut->core_dm_wr_ready_i     = 0;
    dut->core_dm_rx_data_i      = 0;

    dut->pad_dm_awready = 1;
    dut->pad_dm_wready  = 1;
    dut->pad_dm_bid     = 0;
    dut->pad_dm_bresp   = 0;
    dut->pad_dm_bvalid  = 0;
    dut->pad_dm_arready = 1;
    dut->pad_dm_rid     = 0;
    for (int w = 0; w < 4; w++) dut->pad_dm_rdata[w] = 0;
    dut->pad_dm_rvalid  = 0;
    dut->pad_dm_rlast   = 1;
    dut->pad_dm_rresp   = 0;
}

static void fake_core_eval(void) {
    // halt responder
    if (dut->dm_core_halt_req_o && g_halt_cnt < 0)
        g_halt_cnt = g_halt_delay;
    if (g_halt_cnt >= 0) {
        g_halt_cnt--;
        if (g_halt_cnt < 0)
            dut->core_dm_halted_i = 1;
    }
    // resume: drop halted when resume_req pulses and g_resume_drop set
    if (dut->dm_core_resume_req_o && g_resume_drop)
        dut->core_dm_halted_i = 0;
    // itr responder: itr_done is a 1-cycle pulse on the retire cycle. It
    // must be a true pulse (cleared on every tick except the retire tick)
    // even when the next itr_vld arrives the tick after a done.
    if (dut->dm_core_itr_vld_o && g_itr_cnt < 0) {
        g_itr_cnt = g_itr_delay;
        if (g_itr_count < 64)
            g_itr_log[g_itr_count++] = dut->dm_core_itr_o;
    }
    {
        bool done_tick = false;
        if (g_itr_cnt >= 0) {
            g_itr_cnt--;
            if (g_itr_cnt < 0)
                done_tick = true;
        }
        dut->core_dm_itr_done_i = done_tick;
    }
    // wr responder: wr_ready likewise a 1-cycle pulse
    if (dut->dm_core_wr_vld_o && g_wr_cnt < 0) {
        g_wr_cnt = g_wr_delay;
        if (g_wr_count < 16) {
            g_wr_data_log[g_wr_count] = dut->dm_core_wdata_o;
            g_wr_flg_log[g_wr_count] = dut->dm_core_wr_flg_o;
            g_wr_count++;
        }
    }
    {
        bool ready_tick = false;
        if (g_wr_cnt >= 0) {
            g_wr_cnt--;
            if (g_wr_cnt < 0)
                ready_tick = true;
        }
        dut->core_dm_wr_ready_i = ready_tick;
        if (ready_tick) {
            if (dut->dm_core_wr_flg_o == 1)
                g_dscratch0 = dut->dm_core_wdata_o;
            if (dut->dm_core_wr_flg_o == 0) {
                if (g_rx_use_override)
                    dut->core_dm_rx_data_i = g_rx_override;
                else
                    dut->core_dm_rx_data_i = g_dscratch0;
            }
        }
    }
}

static void fake_axi_eval(void) {
    // AW+W channels
    if (dut->dm_pad_awvalid && dut->pad_dm_awready && !g_axi_write_done) {
        g_axi_awaddr = dut->dm_pad_awaddr;
        g_axi_awsize = dut->dm_pad_awsize;
        g_axi_wstrb_seen = dut->dm_pad_wstrb;
        g_axi_write_done = true;
        g_axi_issued = true;
    }
    if (dut->dm_pad_wvalid && dut->pad_dm_wready && g_axi_write_done) {
        uint64_t addr = g_axi_awaddr & 0x7f;
        for (int b = 0; b < 16; b++) {
            if ((g_axi_wstrb_seen >> b) & 1)
                g_axi_mem[(addr + b) & 0x7f] =
                    (dut->dm_pad_wdata[b/4] >> ((b%4)*8)) & 0xff;
        }
        dut->pad_dm_bvalid = 1;
        dut->pad_dm_bresp = (g_axi_bresp_force >= 0) ? g_axi_bresp_force : 0;
        g_axi_write_done = false;
    } else if (dut->pad_dm_bvalid && dut->dm_pad_bready) {
        dut->pad_dm_bvalid = 0;
        g_axi_op_complete = true;
    }
    // AR channel
    if (dut->dm_pad_arvalid && dut->pad_dm_arready && !g_axi_read_done) {
        g_axi_read_addr = dut->dm_pad_araddr;
        g_axi_read_size = dut->dm_pad_arsize;
        g_axi_read_done = true;
        g_axi_issued = true;
    }
    if (g_axi_read_done) {
        uint64_t addr = g_axi_read_addr & 0x7f;
        for (int w = 0; w < 4; w++)
            dut->pad_dm_rdata[w] = 0;
        for (int b = 0; b < 16; b++)
            dut->pad_dm_rdata[b/4] |= (uint32_t)g_axi_mem[(addr + b) & 0x7f] << ((b%4)*8);
        dut->pad_dm_rvalid = 1;
        dut->pad_dm_rresp = (g_axi_rresp_force >= 0) ? g_axi_rresp_force : 0;
        g_axi_read_done = false;
    } else if (dut->pad_dm_rvalid && dut->dm_pad_rready) {
        dut->pad_dm_rvalid = 0;
        g_axi_op_complete = true;
    }
}

static void tick(void) {
    dut->eval();
    fake_core_eval();
    fake_axi_eval();
    dut->clk = 1;
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
}

static void reset_dut(void) {
    dut->clk = 0;
    dut->tdt_rst_n = 0;
    tie_idle_inputs();
    memset(g_axi_mem, 0, sizeof(g_axi_mem));
    g_itr_count = 0;
    g_wr_count = 0;
    g_halt_cnt = -1;
    g_itr_cnt = -1;
    g_wr_cnt = -1;
    g_axi_write_done = false;
    g_axi_read_done = false;
    g_axi_issued = false;
    g_axi_op_complete = false;
    g_axi_bresp_force = -1;
    g_axi_rresp_force = -1;
    g_rx_use_override = false;
    g_resume_drop = false;
    g_dscratch0 = 0;
    for (int i = 0; i < 5; i++) tick();
    dut->tdt_rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

//-----------------------------------------------------------------------------
// Result bookkeeping
//-----------------------------------------------------------------------------
static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0) {
    if (!cond) {
        g_local++;
        if (g_fail < 40)
            printf("    FAIL %-56s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name) {
    printf("[dm_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// APB master (donor timing: pready 1 cycle after SETUP, donor :3315-3322)
//-----------------------------------------------------------------------------
static void apb_idle(void) {
    dut->dm_psel = 0;
    dut->dm_penable = 0;
}

static uint32_t apb_read(uint32_t word_addr) {
    dut->dm_paddr = word_addr << 2;
    dut->dm_pwrite = 0;
    dut->dm_psel = 1;
    dut->dm_penable = 0;
    tick();
    dut->dm_penable = 1;
    int timeout = 10;
    while (!dut->dm_pready && timeout-- > 0)
        tick();
    uint32_t rdata = dut->dm_prdata;
    tick();
    apb_idle();
    return rdata;
}

static void apb_write(uint32_t word_addr, uint32_t wdata) {
    dut->dm_paddr = word_addr << 2;
    dut->dm_pwrite = 1;
    dut->dm_pwdata = wdata;
    dut->dm_psel = 1;
    dut->dm_penable = 0;
    tick();
    dut->dm_penable = 1;
    int timeout = 10;
    while (!dut->dm_pready && timeout-- > 0)
        tick();
    tick();
    apb_idle();
}

//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------
static void dm_active(void) {
    apb_write(ADDR_DMCONTROL, 0x00000001);
    tick(); tick();
}

static void halt_core(void) {
    apb_write(ADDR_DMCONTROL, 0x80000001);
    int timeout = g_halt_delay + 10;
    while (!dut->core_dm_halted_i && timeout-- > 0)
        tick();
}

static void resume_core(void) {
    g_resume_drop = true;
    apb_write(ADDR_DMCONTROL, 0x40000001);
    int timeout = 20;
    while (dut->core_dm_halted_i && timeout-- > 0)
        tick();
    g_resume_drop = false;
}

static void clear_itr_log(void) {
    g_itr_count = 0;
    g_wr_count = 0;
}

static uint32_t read_abstractcs(void) {
    return apb_read(ADDR_ABSTRACTCS);
}

static void wait_not_busy(void) {
    // cmd_start is a registered 1-cycle pulse (donor tdt_dm.v:1897-1917);
    // cmd_work/busy assert only the cycle AFTER the command write access.
    // Settle 3 cycles so the first busy poll cannot land in that window.
    tick(); tick(); tick();
    int timeout = 50;
    while ((read_abstractcs() >> 12) & 1 && timeout-- > 0)
        tick();
}

// SBA: sbbusy is a short pulse (donor tdt_dm.v:2989-2994) that an APB poll
// can miss, so track the op through the fake slave instead: issued on the
// AW/AR handshake, complete on the B/R handshake. The DUT latches
// sbbusy/sbdata/sberror in the same tick the fake completes the op.
static bool sba_wait_start(void) {
    g_axi_issued = false;
    g_axi_op_complete = false;
    int timeout = 20;
    while (!g_axi_issued && timeout-- > 0)
        tick();
    return g_axi_issued;
}

static bool sba_wait_done(void) {
    int timeout = 50;
    while (!g_axi_op_complete && timeout-- > 0)
        tick();
    if (!g_axi_op_complete)
        return false;
    // sbbusy/sbdata/sberror latch on sba_wr_ready, a 2-stage pipeline after
    // the B/R handshake (axi_wr_ready_pre -> sba_wr_ready; donor
    // tdt_sba_axi.v:300-318), so settle before reading back SBCS/SBDATA.
    tick(); tick(); tick();
    return true;
}

static uint32_t get_cmderr(void) {
    return (read_abstractcs() >> 8) & 0x7;
}

static void clear_cmderr(void) {
    apb_write(ADDR_ABSTRACTCS, 0x700);
    tick();
}

//-----------------------------------------------------------------------------
// T1: Reset defaults
//-----------------------------------------------------------------------------
static void t01_reset_defaults(void) {
    uint32_t v;

    v = apb_read(ADDR_DMSTATUS);
    check((v & 0xF) == 2, "dmstatus.version==2", v & 0xF, 2);
    check(((v >> 22) & 1) == 1, "dmstatus.impebreak==1", (v>>22)&1, 1);
    check(((v >> 7) & 1) == 1, "dmstatus.authenticated==1", (v>>7)&1, 1);
    check(((v >> 5) & 1) == 1, "dmstatus.haresethaltreq==1", (v>>5)&1, 1);

    v = apb_read(ADDR_DMCONTROL);
    check(v == 0, "dmcontrol==0", v, 0);

    v = apb_read(ADDR_HARTINFO);
    // donor tdt_dm.v:3358: {8'h0, nscratch[3:0], 20'h0}
    // bits[23:20]=nscratch, bit[24]=dataaccess (0 = shadowed in reg map)
    check(((v >> 24) & 1) == 0, "hartinfo.dataaccess==0", (v>>24)&1, 0);
    check(((v >> 20) & 0xF) == 2, "hartinfo.nscratch==2", (v>>20)&0xF, 2);

    v = apb_read(ADDR_ABSTRACTCS);
    check(((v >> 24) & 0x1F) == 4, "abstractcs.progbufsize==4", (v>>24)&0x1F, 4);
    check(((v >> 12) & 1) == 0, "abstractcs.busy==0", (v>>12)&1, 0);
    check(((v >> 8) & 0x7) == 0, "abstractcs.cmderr==0", (v>>8)&7, 0);
    check((v & 0xF) == 2, "abstractcs.datacount==2", v&0xF, 2);

    v = apb_read(ADDR_SBCS);
    check(((v >> 29) & 0x7) == 1, "sbcs.sbversion==1", (v>>29)&7, 1);
    check(((v >> 5) & 0x7F) == 40, "sbcs.sbasize==40", (v>>5)&0x7F, 40);
    check((v & 0x1F) == 0x1c, "sbcs.sbaccess_info==0x1c", v&0x1F, 0x1c);
    check(((v >> 17) & 0x7) == 2, "sbcs.sbaccess==2", (v>>17)&7, 2);

    v = apb_read(ADDR_COMPID);
    check(v == 0xB6F00001, "compid", v, 0xB6F00001);

    v = apb_read(ADDR_NEXTDM);
    check(v == 0, "nextdm==0", v, 0);

    v = apb_read(ADDR_DMCS2);
    check(v == 0, "dmcs2==0", v, 0);

    v = apb_read(ADDR_PB0); check(v == 0, "progbuf0==0", v, 0);
    v = apb_read(ADDR_PB1); check(v == 0, "progbuf1==0", v, 0);
    v = apb_read(ADDR_PB2); check(v == 0, "progbuf2==0", v, 0);
    v = apb_read(ADDR_PB3); check(v == 0, "progbuf3==0", v, 0);

    v = apb_read(ADDR_DATA0); check(v == 0, "data0==0", v, 0);
    v = apb_read(ADDR_DATA1); check(v == 0, "data1==0", v, 0);

    v = apb_read(ADDR_HALTSUM0); check(v == 0, "haltsum0==0", v, 0);
    v = apb_read(ADDR_ITR); check(v == 0, "itr==0", v, 0);
    v = apb_read(ADDR_SBADDR0); check(v == 0, "sbaddr0==0", v, 0);
    v = apb_read(ADDR_SBADDR1); check(v == 0, "sbaddr1==0", v, 0);
    v = apb_read(ADDR_SBDATA0); check(v == 0, "sbdata0==0", v, 0);
    v = apb_read(ADDR_SBDATA1); check(v == 0, "sbdata1==0", v, 0);
    v = apb_read(ADDR_SBDATA2); check(v == 0, "sbdata2==0", v, 0);
    v = apb_read(ADDR_SBDATA3); check(v == 0, "sbdata3==0", v, 0);

    test_result("T1 reset defaults");
}

//-----------------------------------------------------------------------------
// T2: dmactive 1->0->1
//-----------------------------------------------------------------------------
static void t02_dmactive(void) {
    dm_active();
    apb_write(ADDR_DATA0, 0xDEADBEEF);
    apb_write(ADDR_DATA1, 0xCAFEBABE);
    uint32_t v = apb_read(ADDR_DATA0);
    check(v == 0xDEADBEEF, "data0 writable when active", v, 0xDEADBEEF);
    // dmactive -> 0
    apb_write(ADDR_DMCONTROL, 0x00000000);
    tick(); tick(); tick();
    v = apb_read(ADDR_DATA0);
    check(v == 0, "data0==0 after dmactive=0", v, 0);
    v = apb_read(ADDR_DATA1);
    check(v == 0, "data1==0 after dmactive=0", v, 0);
    check(dut->dm_core_halt_req_o == 0, "halt_req idle when dmactive=0",
          dut->dm_core_halt_req_o, 0);
    check(dut->dm_core_itr_vld_o == 0, "itr_vld idle when dmactive=0",
          dut->dm_core_itr_vld_o, 0);
    check(dut->dm_core_wr_vld_o == 0, "wr_vld idle when dmactive=0",
          dut->dm_core_wr_vld_o, 0);
    dm_active();
    v = apb_read(ADDR_DMCONTROL);
    check((v & 1) == 1, "dmactive==1 after re-enable", v & 1, 1);

    test_result("T2 dmactive 1->0->1");
}

//-----------------------------------------------------------------------------
// T3: Halt
//-----------------------------------------------------------------------------
static void t03_halt(void) {
    halt_core();
    uint32_t v = apb_read(ADDR_DMSTATUS);
    check(((v >> 8) & 1) == 1, "dmstatus.anyhalted==1", (v>>8)&1, 1);
    check(((v >> 9) & 1) == 1, "dmstatus.allhalted==1", (v>>9)&1, 1);
    v = apb_read(ADDR_HALTSUM0);
    check(v == 1, "haltsum0[0]==1", v, 1);

    test_result("T3 halt");
}

//-----------------------------------------------------------------------------
// T4: Resume
//-----------------------------------------------------------------------------
static void t04_resume(void) {
    resume_core();
    uint32_t v = apb_read(ADDR_DMSTATUS);
    check(((v >> 16) & 1) == 1, "dmstatus.anyresumeack==1", (v>>16)&1, 1);
    check(((v >> 17) & 1) == 1, "dmstatus.allresumeack==1", (v>>17)&1, 1);
    check(((v >> 8) & 1) == 0, "dmstatus.anyhalted==0", (v>>8)&1, 0);
    v = apb_read(ADDR_HALTSUM0);
    check(v == 0, "haltsum0[0]==0", v, 0);

    test_result("T4 resume");
}

//-----------------------------------------------------------------------------
// T5: Abstract GPR write
//-----------------------------------------------------------------------------
static void t05_gpr_write(void) {
    halt_core();
    clear_itr_log();

    apb_write(ADDR_DATA0, 0x12345678);
    apb_write(ADDR_DATA1, 0xAAAABBBB);
    uint32_t cmd = (0u << 24) | (3u << 20) | (1u << 17) | (1u << 16) | 0x1010;
    apb_write(ADDR_COMMAND, cmd);
    wait_not_busy();

    check(get_cmderr() == 0, "cmderr==0 after gpr wr", get_cmderr(), 0);
    check(g_wr_count >= 1, "wr_vld fired for gpr write", g_wr_count, 1);
    if (g_wr_count >= 1) {
        check(g_wr_flg_log[0] == 1, "wr_flg==01 for gpr write", g_wr_flg_log[0], 1);
        check(g_wr_data_log[0] == 0xAAAABBBB12345678ULL,
              "wdata=={data1,data0}", g_wr_data_log[0], 0xAAAABBBB12345678ULL);
    }
    check(g_itr_count >= 1, "itr fired for gpr write", g_itr_count, 1);
    if (g_itr_count >= 1) {
        uint32_t exp = enc_csrrc(DSCR0, 0, 16);
        check(g_itr_log[0] == exp, "itr==csrrc x16,dscratch0,x0",
              g_itr_log[0], exp);
    }

    resume_core();
    test_result("T5 abstract GPR write");
}

//-----------------------------------------------------------------------------
// T6: Abstract GPR read
//-----------------------------------------------------------------------------
static void t06_gpr_read(void) {
    halt_core();
    clear_itr_log();
    g_rx_use_override = true;
    g_rx_override = 0x0BADC0DE11223344ULL;

    uint32_t cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x1005;
    apb_write(ADDR_COMMAND, cmd);
    wait_not_busy();

    check(get_cmderr() == 0, "cmderr==0 after gpr rd", get_cmderr(), 0);
    check(g_itr_count >= 1, "itr fired for gpr read", g_itr_count, 1);
    if (g_itr_count >= 1) {
        uint32_t exp = enc_csrrw(DSCR0, 5, 0);
        check(g_itr_log[0] == exp, "itr==csrrw x0,dscratch0,x5",
              g_itr_log[0], exp);
    }

    uint32_t v = apb_read(ADDR_DATA0);
    check(v == 0x11223344, "data0==rx[31:0]", v, 0x11223344);
    v = apb_read(ADDR_DATA1);
    check(v == 0x0BADC0DE, "data1==rx[63:32]", v, 0x0BADC0DE);

    g_rx_use_override = false;
    resume_core();
    test_result("T6 abstract GPR read");
}

//-----------------------------------------------------------------------------
// T7: CSR read
//-----------------------------------------------------------------------------
static void t07_csr_read(void) {
    halt_core();
    clear_itr_log();
    g_rx_use_override = true;
    g_rx_override = 0x00000000DEADBEEFULL;

    uint32_t cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x300;
    apb_write(ADDR_COMMAND, cmd);
    wait_not_busy();

    check(get_cmderr() == 0, "cmderr==0 after csr rd", get_cmderr(), 0);
    // CSR read FSM (donor :2064-2181): X6_2_DSC1 -> C_2_X6 -> X6_2_DSC0 ->
    // DSC1_2_X6 -> RDSC0: 4 injected instructions.
    check(g_itr_count >= 4, "4 itrs for csr read", g_itr_count, 4);
    if (g_itr_count >= 4) {
        uint32_t exp0 = enc_csrrw(DSCR1, 6, 0);
        uint32_t exp1 = enc_csrrc(0x300, 0, 6);
        uint32_t exp2 = enc_csrrw(DSCR0, 6, 0);
        uint32_t exp3 = enc_csrrc(DSCR1, 0, 6);
        check(g_itr_log[0] == exp0, "itr[0]==csrrw x0,dscratch1,x6",
              g_itr_log[0], exp0);
        check(g_itr_log[1] == exp1, "itr[1]==csrrc x6,csr,x0",
              g_itr_log[1], exp1);
        check(g_itr_log[2] == exp2, "itr[2]==csrrw x0,dscratch0,x6",
              g_itr_log[2], exp2);
        check(g_itr_log[3] == exp3, "itr[3]==csrrc x6,dscratch1,x0",
              g_itr_log[3], exp3);
    }

    uint32_t v = apb_read(ADDR_DATA0);
    check(v == 0xDEADBEEF, "data0==csr value", v, 0xDEADBEEF);

    g_rx_use_override = false;
    resume_core();
    test_result("T7 CSR read");
}

//-----------------------------------------------------------------------------
// T8: cmderr=4 (command while not halted)
//-----------------------------------------------------------------------------
static void t08_cmderr4(void) {
    uint32_t cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x1000;
    apb_write(ADDR_COMMAND, cmd);
    tick(); tick();
    check(get_cmderr() == 4, "cmderr==4 when not halted", get_cmderr(), 4);
    clear_cmderr();
    check(get_cmderr() == 0, "cmderr cleared", get_cmderr(), 0);

    test_result("T8 cmderr=4 (not halted)");
}

//-----------------------------------------------------------------------------
// T9: cmderr=1 (write command while busy)
//-----------------------------------------------------------------------------
static void t09_cmderr1(void) {
    halt_core();
    uint32_t cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x300;
    apb_write(ADDR_COMMAND, cmd);
    tick(); tick();
    cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x1001;
    apb_write(ADDR_COMMAND, cmd);
    wait_not_busy();
    check(get_cmderr() == 1, "cmderr==1 when busy-write", get_cmderr(), 1);
    clear_cmderr();
    check(get_cmderr() == 0, "cmderr cleared after 1", get_cmderr(), 0);

    resume_core();
    test_result("T9 cmderr=1 (busy-write)");
}

//-----------------------------------------------------------------------------
// T10: cmderr=2 (unsupported)
//-----------------------------------------------------------------------------
static void t10_cmderr2(void) {
    halt_core();

    uint32_t cmd = (1u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x1000;
    apb_write(ADDR_COMMAND, cmd);
    tick(); tick();
    check(get_cmderr() == 2, "cmderr==2 for cmdtype=1", get_cmderr(), 2);
    clear_cmderr();

    cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x2000;
    apb_write(ADDR_COMMAND, cmd);
    tick(); tick();
    check(get_cmderr() == 2, "cmderr==2 for bad regno", get_cmderr(), 2);
    clear_cmderr();

    cmd = (0u << 24) | (1u << 20) | (1u << 17) | (1u << 16) | 0x1000;
    apb_write(ADDR_COMMAND, cmd);
    tick(); tick();
    check(get_cmderr() == 2, "cmderr==2 for aarsize=1 write", get_cmderr(), 2);
    clear_cmderr();

    resume_core();
    test_result("T10 cmderr=2 (unsupported)");
}

//-----------------------------------------------------------------------------
// T11: cmderr=3 (exception during abstract)
//-----------------------------------------------------------------------------
static void t11_cmderr3(void) {
    halt_core();
    clear_itr_log();

    apb_write(ADDR_PB0, 0x00000013);  // nop
    uint32_t cmd = (0u << 24) | (1u << 18);
    apb_write(ADDR_COMMAND, cmd);
    // cmderr=3 needs itr_done && retire_debug_expt && busy in one cycle
    // (donor :2017-2024). The first itr (pb0) retires ~5-7 cycles after the
    // command write, so hold the exception flag across that window.
    tick(); tick();
    dut->core_dm_retire_debug_expt_i = 1;
    for (int i = 0; i < 12; i++) tick();
    dut->core_dm_retire_debug_expt_i = 0;
    wait_not_busy();
    check(get_cmderr() == 3, "cmderr==3 on exception", get_cmderr(), 3);
    clear_cmderr();

    resume_core();
    test_result("T11 cmderr=3 (exception)");
}

//-----------------------------------------------------------------------------
// T12: Progbuf execution
//-----------------------------------------------------------------------------
static void t12_progbuf(void) {
    halt_core();
    clear_itr_log();

    apb_write(ADDR_PB0, 0x00000013);
    apb_write(ADDR_PB1, EBREAK_INST);
    uint32_t cmd = (0u << 24) | (1u << 18);
    apb_write(ADDR_COMMAND, cmd);
    wait_not_busy();

    check(get_cmderr() == 0, "cmderr==0 after progbuf", get_cmderr(), 0);
    check(g_itr_count >= 2, "progbuf sent 2 itrs", g_itr_count, 2);
    if (g_itr_count >= 2) {
        check(g_itr_log[0] == 0x00000013, "itr[0]==nop", g_itr_log[0], 0x13);
        check(g_itr_log[1] == EBREAK_INST, "itr[1]==ebreak", g_itr_log[1], EBREAK_INST);
    }

    resume_core();
    test_result("T12 progbuf execution");
}

//-----------------------------------------------------------------------------
// T13: SBA 32-bit write+read
//-----------------------------------------------------------------------------
static void t13_sba_32(void) {
    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17));
    apb_write(ADDR_SBADDR0, 0x80000010);
    apb_write(ADDR_SBDATA0, 0xCAFE0001);
    check(sba_wait_start(), "32b wr started (aw/ar handshake)", (uint64_t)(g_axi_issued ? 1 : 0), 1);
    check(sba_wait_done(), "32b wr done", (uint64_t)(g_axi_op_complete ? 1 : 0), 1);
    uint32_t v = apb_read(ADDR_SBCS);
    check(((v>>21)&1) == 0, "op done: 32b wr", (v>>21)&1, 0);
    check(((v>>12)&7) == 0, "sberror==0 after 32b wr", (v>>12)&7, 0);

    check(g_axi_awaddr == 0x80000010, "awaddr", g_axi_awaddr, 0x80000010);
    check(g_axi_wstrb_seen == 0x000F, "wstrb==0xF", g_axi_wstrb_seen, 0xF);
    check(g_axi_awsize == 2, "awsize==2", g_axi_awsize, 2);

    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17) | (1u << 20));
    apb_write(ADDR_SBADDR0, 0x80000010);
    check(sba_wait_start(), "32b rd started (aw/ar handshake)", (uint64_t)(g_axi_issued ? 1 : 0), 1);
    check(sba_wait_done(), "32b rd done", (uint64_t)(g_axi_op_complete ? 1 : 0), 1);
    v = apb_read(ADDR_SBDATA0);
    check(v == 0xCAFE0001, "sbdata0 readback", v, 0xCAFE0001);

    test_result("T13 SBA 32-bit");
}

//-----------------------------------------------------------------------------
// T14: SBA 64-bit and 128-bit
//-----------------------------------------------------------------------------
static void t14_sba_64_128(void) {
    // Write op: sba_wr_vld pulses 1 cycle after the SBDATA0 write and
    // sbbusy the cycle after that (donor :2989-2994); sbdata1-3 are gated
    // by sb_noerr (donor :2908-2960), so they must be written BEFORE the
    // sbdata0 trigger or sbbusy blocks them (and sets sbbusyerror).
    apb_write(ADDR_SBCS, (1u << 29) | (3u << 17));
    apb_write(ADDR_SBADDR0, 0x20);
    apb_write(ADDR_SBDATA1, 0x22222222);
    apb_write(ADDR_SBDATA0, 0x11111111);
    check(sba_wait_start(), "64b wr started (aw/ar handshake)", (uint64_t)(g_axi_issued ? 1 : 0), 1);
    check(sba_wait_done(), "op done: 64b wr", (uint64_t)(g_axi_op_complete ? 1 : 0), 1);
    check(g_axi_wstrb_seen == 0x00FF, "wstrb==0xFF for 64b", g_axi_wstrb_seen, 0xFF);

    apb_write(ADDR_SBCS, (1u << 29) | (3u << 17) | (1u << 20));
    apb_write(ADDR_SBADDR0, 0x20);
    check(sba_wait_start(), "64b rd started (aw/ar handshake)", (uint64_t)(g_axi_issued ? 1 : 0), 1);
    check(sba_wait_done(), "op done: 64b rd", (uint64_t)(g_axi_op_complete ? 1 : 0), 1);
    uint32_t v = apb_read(ADDR_SBDATA0);
    check(v == 0x11111111, "sbdata0[31:0] 64b", v, 0x11111111);
    v = apb_read(ADDR_SBDATA1);
    check(v == 0x22222222, "sbdata1[31:0] 64b", v, 0x22222222);

    apb_write(ADDR_SBCS, (1u << 29) | (4u << 17));
    apb_write(ADDR_SBADDR0, 0x30);
    apb_write(ADDR_SBDATA1, 0xBBBB1111);
    apb_write(ADDR_SBDATA2, 0xCCCC2222);
    apb_write(ADDR_SBDATA3, 0xDDDD3333);
    apb_write(ADDR_SBDATA0, 0xAAAA0000);
    check(sba_wait_start(), "128b wr started (aw/ar handshake)", (uint64_t)(g_axi_issued ? 1 : 0), 1);
    check(sba_wait_done(), "op done: 128b wr", (uint64_t)(g_axi_op_complete ? 1 : 0), 1);
    check(g_axi_wstrb_seen == 0xFFFF, "wstrb==0xFFFF for 128b", g_axi_wstrb_seen, 0xFFFF);

    test_result("T14 SBA 64/128-bit");
}

//-----------------------------------------------------------------------------
// T15: sberror=3 (misaligned)
//-----------------------------------------------------------------------------
static void t15_sberror3(void) {
    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17) | (7u << 12));
    tick();
    apb_write(ADDR_SBCS, (1u << 29) | (3u << 17));
    apb_write(ADDR_SBADDR0, 0x22);
    apb_write(ADDR_SBDATA0, 0xDEADBEEF);
    tick(); tick(); tick();
    uint32_t v = apb_read(ADDR_SBCS);
    check(((v>>12)&7) == 3, "sberror==3 misaligned", (v>>12)&7, 3);

    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17) | (7u << 12));
    tick();
    v = apb_read(ADDR_SBCS);
    check(((v>>12)&7) == 0, "sberror cleared", (v>>12)&7, 0);

    test_result("T15 sberror=3 (misaligned)");
}

//-----------------------------------------------------------------------------
// T16: sberror=4 (unsupported sbaccess)
//-----------------------------------------------------------------------------
static void t16_sberror4(void) {
    apb_write(ADDR_SBCS, (1u << 29) | (5u << 17));
    apb_write(ADDR_SBADDR0, 0x40);
    apb_write(ADDR_SBDATA0, 0x12345678);
    tick(); tick(); tick();
    uint32_t v = apb_read(ADDR_SBCS);
    check(((v>>12)&7) == 4, "sberror==4 unsupported access", (v>>12)&7, 4);

    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17) | (7u << 12));
    tick();

    test_result("T16 sberror=4 (unsupported)");
}

//-----------------------------------------------------------------------------
// T17: sberror=7 (bus error)
//-----------------------------------------------------------------------------
static void t17_sberror7(void) {
    g_axi_rresp_force = 2;
    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17) | (1u << 20));
    apb_write(ADDR_SBADDR0, 0x50);
    check(sba_wait_start(), "32b rd started (aw/ar handshake)", (uint64_t)(g_axi_issued ? 1 : 0), 1);
    check(sba_wait_done(), "op done: 32b rd", (uint64_t)(g_axi_op_complete ? 1 : 0), 1);
    uint32_t v = apb_read(ADDR_SBCS);
    check(((v>>12)&7) == 7, "sberror==7 bus error", (v>>12)&7, 7);
    g_axi_rresp_force = -1;

    apb_write(ADDR_SBCS, (1u << 29) | (2u << 17) | (7u << 12));
    tick();

    test_result("T17 sberror=7 (bus error)");
}

//-----------------------------------------------------------------------------
// T18: ndmreset (donor :808-815: ndmreset is a dmcontrol register bit
// that drives the ndmresetn PAD OUTPUT; it does NOT reset DM registers.
// Only sync_rst (dmactive 1->0) resets DM registers. This test checks
// that the ndmreset bit reads back and the pad output toggles.)
//-----------------------------------------------------------------------------
static void t18_ndmreset(void) {
    dm_active();
    // assert ndmreset
    apb_write(ADDR_DMCONTROL, 0x00000003);  // ndmreset=1, dmactive=1
    tick(); tick();
    uint32_t v = apb_read(ADDR_DMCONTROL);
    check((v & 2) == 2, "dmcontrol.ndmreset==1", v & 2, 2);
    check((v & 1) == 1, "dmcontrol.dmactive stays 1", v & 1, 1);
    check(dut->dm_core_ndmreset_n_o == 0, "ndmresetn pad output low",
          dut->dm_core_ndmreset_n_o, 0);

    // deassert ndmreset
    apb_write(ADDR_DMCONTROL, 0x00000001);
    tick(); tick();
    v = apb_read(ADDR_DMCONTROL);
    check((v & 2) == 0, "dmcontrol.ndmreset==0", v & 2, 0);
    check(dut->dm_core_ndmreset_n_o == 1, "ndmresetn pad output high",
          dut->dm_core_ndmreset_n_o, 1);

    test_result("T18 ndmreset");
}

//-----------------------------------------------------------------------------
// T19: abstractauto
//-----------------------------------------------------------------------------
static void t19_abstractauto(void) {
    halt_core();
    clear_itr_log();
    g_rx_use_override = true;
    g_rx_override = 0xFEDCBA9876543210ULL;

    apb_write(ADDR_ABSTRACTAUTO, 0x00000001);
    uint32_t cmd = (0u << 24) | (3u << 20) | (1u << 17) | (0u << 16) | 0x1007;
    apb_write(ADDR_COMMAND, cmd);
    wait_not_busy();

    clear_itr_log();
    apb_write(ADDR_DATA0, 0x12345678);
    wait_not_busy();

    check(g_itr_count >= 1, "autoexec fired itr", g_itr_count, 1);
    if (g_itr_count >= 1) {
        uint32_t exp = enc_csrrw(DSCR0, 7, 0);
        check(g_itr_log[0] == exp, "autoexec itr==csrrw x0,dscratch0,x7",
              g_itr_log[0], exp);
    }

    g_rx_use_override = false;
    apb_write(ADDR_ABSTRACTAUTO, 0);
    resume_core();
    test_result("T19 abstractauto");
}

//-----------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VTDT_DM;

    printf("=== dm_tb: TDT_DM unit bench (M7 Task 3) ===\n");

    reset_dut();
    t01_reset_defaults();

    reset_dut();
    t02_dmactive();

    reset_dut();
    dm_active();
    t03_halt();
    t04_resume();

    reset_dut();
    dm_active();
    t05_gpr_write();

    reset_dut();
    dm_active();
    t06_gpr_read();

    reset_dut();
    dm_active();
    t07_csr_read();

    reset_dut();
    dm_active();
    t08_cmderr4();

    reset_dut();
    dm_active();
    t09_cmderr1();

    reset_dut();
    dm_active();
    t10_cmderr2();

    reset_dut();
    dm_active();
    t11_cmderr3();

    reset_dut();
    dm_active();
    t12_progbuf();

    reset_dut();
    dm_active();
    t13_sba_32();

    reset_dut();
    dm_active();
    t14_sba_64_128();

    reset_dut();
    dm_active();
    t15_sberror3();

    reset_dut();
    dm_active();
    t16_sberror4();

    reset_dut();
    dm_active();
    t17_sberror7();

    reset_dut();
    t18_ndmreset();

    reset_dut();
    dm_active();
    t19_abstractauto();

    printf("=== dm_tb: %s (%d failures) ===\n",
           g_fail ? "UNIT-FAIL" : "UNIT-PASS", g_fail);

    delete dut;
    return g_fail ? 1 : 0;
}
