//=============================================================================
// dut.cpp - DUT wrapper for rv906 handwritten RTL simulation
//=============================================================================
// Derived from the donor rocket-chip project's dut2.cpp (the hand-written-RTL
// DUT wrapper used by its rocketMsim flow).
// - init() drives the reset sequence; the architectural preload (PC/GPR via
//   Verilator internal signal access) is compiled out while the core is the
//   M0 TestMaster scaffold (VERISIM_NO_CPU_STATE, see rtl/verisim.h)
// - sync() mirrors RTL architectural state into CoreState; likewise a no-op
//   in M0. M2 restores both against the real rv906 core.
//=============================================================================

#include "dut.h"
#include <stdio.h>

DUT::DUT() : vdut(new VDUT), jtag(vdut) {
#ifdef VERISIM_TRACE
    tfp = nullptr;
    trace_tick = 0;
#endif
}

DUT::~DUT() {
#ifdef VERISIM_TRACE
    if (tfp) { tfp->close(); delete tfp; }
#endif
    if (vdut) {
        vdut->final();
        delete vdut;
    }
}

//=============================================================================
// M7 Task 6: JTAG driver (dual-clock interleave)
//=============================================================================
// The driver owns BOTH clocks with a fixed interleave (design doc
// 2026-09-11-m7-debug-design.md "SoC layer", risk #2; donor ratio
// TCK=clk/4, dtmcs.idle=7):
//   * one TCK cycle = TMS/TDI settled, 4 clk edges (TCK low phase), TCK
//     rising edge, 4 clk edges (TCK high phase), TCK falling edge.
//     => 4 clk edges per TCK phase, clk/tck = 8, which meets the DMI
//     bridge contract Freq.pclk/Freq.tck > 8/(IDLE_CYCLE-4) = 8/3
//     (TDT_DTM.v TIMING CONTRACT: the 4-FF pulse syncs + APB FSM must
//     complete inside the 7-idle-TCK budget between DMI ops).
//   * the 4 clk edges of each TCK phase are TWO full core clock cycles,
//     advanced through the `core_tick` callback the harness passes in
//     (one complete step-equivalent: pre-clock input driving, clk low +
//     high edges, post-clock output read, then memory/AXI/UART service --
//     the same code path as the normal run's step()). This is what keeps
//     the core RUNNING while the scan runs: an un-serviced core would
//     sample the pre-initialised memory response (0) on its first fetch
//     and trap-loop on mtvec=0. The driver owns the interleave (when the
//     TCK edges fall and how many core cycles separate them); the tick
//     is the harness's atomic "one core clock cycle" primitive.
//   * clk and tck are NEVER toggled in the same eval (each TCK toggle
//     gets its own eval()); TMS/TDI are settled 4 clk edges BEFORE the
//     TCK rising edge that samples them (the TAP FSM and the DR shifter
//     are clocked on posedge tck, TDT_DTM.v).
//   * TDO is returned sampled AFTER the TCK falling-edge eval: TDO is
//     negedge-registered (tdo_r <= chain_shifter[0] on negedge tck,
//     TDT_DTM.v TDO section; donor tdt_dtm_chain.v:105-122), so the
//     falling-edge eval has already latched the bit shifted out by the
//     cycle's rising edge. This equals the donor's posedge-sampled TDO
//     (JTAG_DRV.vh shift_dr reads `jtag_tdo at @(posedge) -- tdo_r is
//     stable between negedges).
uint32_t JTAG::cycle(uint32_t tms, uint32_t tdi,
                     const std::function<void()> &core_tick)
{
    // TMS/TDI settled well before the TCK rising edge (a full eval below
    // plus the 4 clk edges of the low phase that follow).
    v->jtag_tms = tms & 1;
    v->jtag_tdi = tdi & 1;
    v->eval();

    // TCK low phase: two core clock cycles (4 clk edges). The DMI->APB
    // bridge (pclk domain) and the DM run on these edges between TCK
    // edges -- this is what carries a DMI request across the
    // tck<->clk pulse syncs.
    core_tick();
    core_tick();

    // TCK rising edge (TAP state machine + DR shifter clock).
    v->jtag_tck = 1;
    v->eval();
    tck_cycles++;

    // TCK high phase: two core clock cycles (4 clk edges).
    core_tick();
    core_tick();

    // TCK falling edge (TDO latched).
    v->jtag_tck = 0;
    v->eval();

    // TDO sampled after the TCK falling-edge eval.
    return v->jtag_tdo;
}

void JTAG::run_to_idle()
{
    // Idle pad state: TAP stays in Run-Test/Idle (TMS=0 in Idle), no
    // scans, DMI engine idle, TDO returns to 1 (TDT_DTM.v tdo fallthrough).
    v->jtag_tms = 0;
    v->jtag_tdi = 0;
    v->jtag_tck = 0;
    v->eval();
}

void DUT::init(RV_AType pc, RV_UType sp, RV_UType dtb) {
#ifdef VERISIM_TRACE
    Verilated::traceEverOn(true);
    tfp = new VerilatedFstC;
    vdut->trace(tfp, 99);
    tfp->open("run/dump.fst");
#endif

    // Reset sequence
    // M7 Task 6: JTAG debug pads are owned by the C++ JTAG driver (struct
    // JTAG, see the JTAG::cycle contract above). Idle at reset:
    // tck/tms/tdi=0 -> TAP in Test-Logic-Reset, no scans, DTM APB master
    // idle, dmactive stays 0 -> DM off-path identity.
    // tdt_rst_n is the DM's own async reset (D-M7-3); held 1 (released) so
    // the DM is powered but inactive (mirrors the donor ciu_rst_b power-on
    // deassert -- all core-side outputs at reset constants, SBA master
    // driving no AXI requests).
    vdut->jtag_tck   = 0;
    vdut->jtag_tms   = 0;
    vdut->jtag_tdi   = 0;
    vdut->tdt_rst_n  = 1;

    vdut->rst_n = 0;
    vdut->clk = 0;
    vdut->eval();
    vdut->clk = 1;
    vdut->eval();
    vdut->clk = 0;
    vdut->eval();

    // Release reset
    vdut->rst_n = 1;
    vdut->eval();

#ifndef VERISIM_NO_CPU_STATE
    auto *rootp = vdut->rootp;
    auto& gpr = rootp->CPU_GPR;

    // INIT_REG: Set registers AFTER reset release, BEFORE first clock
    rootp->CPU_PC = pc;
    gpr[2]  = sp;   // SP
    gpr[10] = 0;    // A0 = hartid (single-hart: 0; OpenSBI boot convention)
    gpr[11] = dtb;  // A1
    vdut->eval();
#else
    (void)pc; (void)sp; (void)dtb;
#endif
}

bool DUT::step(MEMCTLPin *mpin, UINT32 uart_irq) {
    // Write memory response data (from memory controller)
    vdut->G_io_pins_mpin_dout_data_0 = mpin->dout.data[0];
    vdut->G_io_pins_mpin_dout_data_1 = mpin->dout.data[1];
    vdut->G_io_pins_mpin_dout_data_2 = mpin->dout.data[2];
    vdut->G_io_pins_mpin_dout_data_3 = mpin->dout.data[3];
    vdut->G_io_pins_mpin_dout_data_4 = mpin->dout.data[4];
    vdut->G_io_pins_mpin_dout_data_5 = mpin->dout.data[5];
    vdut->G_io_pins_mpin_dout_data_6 = mpin->dout.data[6];
    vdut->G_io_pins_mpin_dout_data_7 = mpin->dout.data[7];

    // M6 Task 6: UART interrupt level (device/uart16550 irq()) -> PLIC
    // source 7. Driven pre-clock like the memory response data; the harness
    // samples it from the model's current state (1-step skew is inherent to
    // the post-clock device service order in TB::step and is fine for a
    // level signal).
    vdut->G_io_pins_uart_irq = uart_irq & 1;

    // Clock low phase
    vdut->clk = 0;
    vdut->eval();
#ifdef VERISIM_TRACE
    if (tfp) tfp->dump(trace_tick++);
#endif

    // Clock high phase (rising edge)
    vdut->clk = 1;
    vdut->eval();
#ifdef VERISIM_TRACE
    if (tfp) tfp->dump(trace_tick++);
#endif

    // M7 Task 5: read the dangling debug outputs into unused variables
    // (chip-level outputs, no consumer in the rv906 harness yet; the
    // Task 6 JTAG driver will make use of jtag_tdo).
    {
        volatile uint32_t dbg_tdo    = vdut->jtag_tdo;
        volatile uint32_t dbg_ndmrst = vdut->ndmreset_n;
        volatile uint32_t dbg_hrst   = vdut->hartreset_n;
        (void)dbg_tdo; (void)dbg_ndmrst; (void)dbg_hrst;
    }

    // Read memory request signals (from CPU)
    mpin->addr = vdut->G_io_pins_mpin_addr;
    mpin->cs = vdut->G_io_pins_mpin_cs;
    mpin->we = vdut->G_io_pins_mpin_we;
    mpin->ras = vdut->G_io_pins_mpin_ras;
    mpin->cas = vdut->G_io_pins_mpin_cas;
    mpin->size = vdut->G_io_pins_mpin_size;
    mpin->din.data[0] = vdut->G_io_pins_mpin_din_data_0;
    mpin->din.data[1] = vdut->G_io_pins_mpin_din_data_1;
    mpin->din.data[2] = vdut->G_io_pins_mpin_din_data_2;
    mpin->din.data[3] = vdut->G_io_pins_mpin_din_data_3;
    mpin->din.data[4] = vdut->G_io_pins_mpin_din_data_4;
    mpin->din.data[5] = vdut->G_io_pins_mpin_din_data_5;
    mpin->din.data[6] = vdut->G_io_pins_mpin_din_data_6;
    mpin->din.data[7] = vdut->G_io_pins_mpin_din_data_7;

    // Return quitted signal
    return vdut->G_RVProcAXI_OUT;
}

void DUT::sync(CoreState &cpu) {
#ifndef VERISIM_NO_CPU_STATE
    auto *rootp = vdut->rootp;
    auto& gpr = rootp->CPU_GPR;

    // Sync GPR
    for (int i = 0; i < 32; i++) {
        cpu.gpr[i] = gpr[i];
    }
    // M2: mirror the rv906 DCache arrays here (the donor's DCache mirror was
    // written against rocket's array layout and does not apply).
#else
    (void)cpu;
#endif
}

// AXI bus interface (for UART and other peripherals)
// Write slave signals to RTL (before clock edge)
#define AXI_WRITE_INPUTS(ch, N) do { \
    vdut->G_axi_bus_s_ch_##N##_raddr_s_ready = ch->raddr.s.ready; \
    vdut->G_axi_bus_s_ch_##N##_waddr_s_ready = ch->waddr.s.ready; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_0 = ch->rdat.s.data.data[0]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_1 = ch->rdat.s.data.data[1]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_2 = ch->rdat.s.data.data[2]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_3 = ch->rdat.s.data.data[3]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_4 = ch->rdat.s.data.data[4]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_5 = ch->rdat.s.data.data[5]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_6 = ch->rdat.s.data.data[6]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_data_data_7 = ch->rdat.s.data.data[7]; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_resp = ch->rdat.s.resp; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_valid = ch->rdat.s.valid; \
    vdut->G_axi_bus_s_ch_##N##_rdat_s_last = ch->rdat.s.last; \
    vdut->G_axi_bus_s_ch_##N##_wdat_s_ready = ch->wdat.s.ready; \
    vdut->G_axi_bus_s_ch_##N##_wres_s_resp = ch->wres.s.resp; \
    vdut->G_axi_bus_s_ch_##N##_wres_s_valid = ch->wres.s.valid; \
} while (0)

// Read master signals from RTL (after clock edge)
#define AXI_READ_OUTPUTS(ch, N) do { \
    ch->raddr.m.addr = vdut->G_axi_bus_s_ch_##N##_raddr_m_addr; \
    ch->raddr.m.size = vdut->G_axi_bus_s_ch_##N##_raddr_m_size; \
    ch->raddr.m.valid = vdut->G_axi_bus_s_ch_##N##_raddr_m_valid; \
    ch->raddr.m.len = vdut->G_axi_bus_s_ch_##N##_raddr_m_len; \
    ch->raddr.m.prot = vdut->G_axi_bus_s_ch_##N##_raddr_m_prot; \
    ch->raddr.m.burst = vdut->G_axi_bus_s_ch_##N##_raddr_m_burst; \
    ch->waddr.m.addr = vdut->G_axi_bus_s_ch_##N##_waddr_m_addr; \
    ch->waddr.m.size = vdut->G_axi_bus_s_ch_##N##_waddr_m_size; \
    ch->waddr.m.valid = vdut->G_axi_bus_s_ch_##N##_waddr_m_valid; \
    ch->waddr.m.len = vdut->G_axi_bus_s_ch_##N##_waddr_m_len; \
    ch->waddr.m.prot = vdut->G_axi_bus_s_ch_##N##_waddr_m_prot; \
    ch->waddr.m.burst = vdut->G_axi_bus_s_ch_##N##_waddr_m_burst; \
    ch->wdat.m.data.data[0] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_0; \
    ch->wdat.m.data.data[1] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_1; \
    ch->wdat.m.data.data[2] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_2; \
    ch->wdat.m.data.data[3] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_3; \
    ch->wdat.m.data.data[4] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_4; \
    ch->wdat.m.data.data[5] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_5; \
    ch->wdat.m.data.data[6] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_6; \
    ch->wdat.m.data.data[7] = vdut->G_axi_bus_s_ch_##N##_wdat_m_data_data_7; \
    ch->wdat.m.strobe = vdut->G_axi_bus_s_ch_##N##_wdat_m_strobe; \
    ch->wdat.m.valid = vdut->G_axi_bus_s_ch_##N##_wdat_m_valid; \
    ch->wdat.m.last = vdut->G_axi_bus_s_ch_##N##_wdat_m_last; \
    ch->rdat.m.ready = vdut->G_axi_bus_s_ch_##N##_rdat_m_ready; \
    ch->wres.m.ready = vdut->G_axi_bus_s_ch_##N##_wres_m_ready; \
} while (0)

template <> void DUT::write<2>(AXI4L::CH *ch) { AXI_WRITE_INPUTS(ch, 2); }
template <> void DUT::read<2>(AXI4L::CH *ch) { AXI_READ_OUTPUTS(ch, 2); }
