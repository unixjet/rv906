#ifndef DUT_H
#define DUT_H

#include "RVProc.h"
#include "io/RVProc_io.h"
#include "io/ExtMem.h"
#include "verisim.h"

#ifdef VERISIM_TRACE
#include <verilated_fst_c.h>
#endif

struct DUT {
	VDUT *vdut;
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
