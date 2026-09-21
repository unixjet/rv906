#ifndef DUT_H
#define DUT_H

#include "RVProc.h"
#include "io/RVProc_io.h"
#include "io/ExtMem.h"
#include "verisim.h"

#include <functional>

#ifdef VERISIM_TRACE
#include <verilated_fst_c.h>
#endif

// M7 Task 6: JTAG driver -- the C++ testbench owns BOTH clocks (the core
// clk and the JTAG tck) with a fixed interleave. The full clock-discipline
// contract is implemented in dut.cpp (see JTAG::cycle); the summary:
//   * one TCK cycle = TMS/TDI settled, 4 clk edges, TCK rising edge (eval),
//     4 clk edges, TCK falling edge (eval); TDO is returned sampled after
//     the TCK falling-edge eval (TDO is negedge-registered, TDT_DTM.v).
//   * the 4 clk edges of each TCK phase are TWO full core clock cycles,
//     advanced through the harness's `core_tick` callback (one complete
//     TB::step-equivalent: pre-clock input driving, 2 clk edges,
//     post-clock output read + memory/AXI/UART service). That is what
//     keeps the core RUNNING while the scan runs -- an un-serviced core
//     would sample the pre-initialised memory response (0) on its first
//     fetch and trap-loop. The driver owns the interleave; the tick is
//     the harness's atomic "one core clock cycle" primitive.
//   * clk and tck are never toggled in the same eval; TMS/TDI are settled
//     (4+ clk edges) before the TCK rising edge.
//   * clk/tck = 8 (>=4 clk edges per TCK phase; the donor ratio is
//     TCK=clk/4, dtmcs.idle=7 -- design doc "SoC layer" clock discipline).
struct JTAG {
    VDUT *v;
    uint64_t tck_cycles; // TCK rising edges driven (debug counter)

    JTAG(VDUT *vd) : v(vd), tck_cycles(0) {}
    // One TCK cycle: TMS/TDI presented for this cycle's rising edge; each
    // TCK phase spans two `core_tick()` calls (4 clk edges each); returns
    // TDO sampled after the TCK falling-edge eval.
    uint32_t cycle(uint32_t tms, uint32_t tdi,
                   const std::function<void()> &core_tick);
    // Restore the idle pad state (tck/tms/tdi=0) after a scan burst.
    void run_to_idle();
};

struct DUT {
	VDUT *vdut;
	JTAG jtag;
#ifdef VERISIM_TRACE
	VerilatedFstC *tfp;
	vluint64_t trace_tick;
#endif

	DUT();
	~DUT();

	void init(RV_AType pc, RV_UType sp, RV_UType dtb);
	bool step(MEMCTLPin *mpin, UINT32 uart_irq = 0);
	void sync(CoreState &cpu);

	template <int N> void write(AXI4L::CH *ch);
	template <int N> void read(AXI4L::CH *ch);
};

#endif // DUT_H
