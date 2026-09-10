#ifndef	_UART16550_AXI4L_H_
#define	_UART16550_AXI4L_H_

#include "C2Rdef.h"
#include "RVProc.h"
#include "io/RVProc_io.h"
#include "ttysrv.h"

struct UART16550_AXI4L : AXI4L::TSlaveFSM<UINT8>, ttysrv {
	enum {
		RBR = 0,	// DLAB = 0, read
		THR = 0,	// DLAB = 0, write
		IER = 1,
		IIR = 2,	// read
		FCR = 2,	// write
		LCR = 3,
		MCR = 4,
		LSR = 5,
		MSR = 6,
		SCR = 7,
		DLL = 0,	// DLAB = 1
		DLM = 1,	// DLAB = 1
	};

	UINT8 ier, fcr, lcr, mcr, scr, dll, dlm;

	UART16550_AXI4L();

	BIT devRead(UINT8 *data, AXI4L::AXI_AType addr, UINT3 size);
	BIT devWrite(UINT8 data, AXI4L::AXI_AType addr, UINT3 size);
        void fsmUser();
	// M6 Task 6: the UART's own interrupt level (IER-based), exposed so the
	// harness can drive it into the PLIC as source 7. Mirrors fsmUser()'s
	// condition exactly -- deliberately NOT the AXI channel's `intr`, which
	// also carries 1-cycle read/write-completion pulses from the TSlaveFSM
	// base class that would otherwise latch as spurious PLIC interrupts.
	BIT irq() {
		if (ier & 2) return 1;              // THRE: tx always empty in this model
		if ((ier & 1) && ready(IN)) return 1; // RDA: rx data available
		return 0;
	}
	//_C2R_FUNC(1)
	//void step(AXI4L::CH *axi);
	void update(AXI4L::TCH<UINT8> *axi) {
		fsm(axi);
	}
};

#endif	// _UART16550_AXI4L_H_
