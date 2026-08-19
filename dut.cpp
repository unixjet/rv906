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

DUT::DUT() : vdut(new VDUT) {
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

void DUT::init(RV_AType pc, RV_UType sp, RV_UType dtb) {
#ifdef VERISIM_TRACE
    Verilated::traceEverOn(true);
    tfp = new VerilatedFstC;
    vdut->trace(tfp, 99);
    tfp->open("run/dump.fst");
#endif

    // Reset sequence
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
    gpr[2] = sp;   // SP
    gpr[11] = dtb; // A1
    vdut->eval();
#else
    (void)pc; (void)sp; (void)dtb;
#endif
}

bool DUT::step(MEMCTLPin *mpin) {
    // Write memory response data (from memory controller)
    vdut->G_io_pins_mpin_dout_data_0 = mpin->dout.data[0];
    vdut->G_io_pins_mpin_dout_data_1 = mpin->dout.data[1];
    vdut->G_io_pins_mpin_dout_data_2 = mpin->dout.data[2];
    vdut->G_io_pins_mpin_dout_data_3 = mpin->dout.data[3];
    vdut->G_io_pins_mpin_dout_data_4 = mpin->dout.data[4];
    vdut->G_io_pins_mpin_dout_data_5 = mpin->dout.data[5];
    vdut->G_io_pins_mpin_dout_data_6 = mpin->dout.data[6];
    vdut->G_io_pins_mpin_dout_data_7 = mpin->dout.data[7];

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
