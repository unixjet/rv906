#include "uart16550.h"

UART16550_AXI4L::UART16550_AXI4L()
{
	path = "run/uart16550";
}

BIT UART16550_AXI4L::devRead(UINT8 *data, AXI4L::AXI_AType addr, UINT3 size)
{
	int offset = (addr >> 2) & 0x7;
	UINT32 rdata = 0;

	switch (offset) {
	case RBR:
		if (lcr & 0x80)	// DLAB
			rdata = dll;
		else
			rdata = in();
		break;
	case IER:
		if (lcr & 0x80)	// DLAB
			rdata = dlm;
		else
			rdata = ier;
		break;
	case IIR:
		if (ready(IN) && (ier & 1))
			rdata = 2 << 1;
		else if (ier & 2)	// always tx is enable
			rdata = 1 << 1;
		else
			rdata = 1;
		break;
	case LCR:
		rdata = lcr;
		break;
	case MCR:
		rdata = mcr;
		break;
	case LSR:
		rdata = 0x60;	// TEMT|THRT
		if (ready(IN))
			rdata |= 1;	// DR
		break;
	case MSR:
		if (!hup())
			rdata = 0x80;	// DCD
		break;
	case SCR:
		rdata = scr;
		break;
	}

	*data = rdata;

	return 1;
}

BIT UART16550_AXI4L::devWrite(UINT8 data, AXI4L::AXI_AType addr, UINT3 size)
{
	int offset = (addr >> 2) & 0x7;
	UINT32 wdata = data;

//	printf("UART::%s: addr = %llx, size = %x, %x\n", __func__, addr, size, wdata);
	switch (offset) {
	case THR:
		if (lcr & 0x80)	// DLAB
			dll = wdata;
		else
			ttysrv::out(wdata);
		break;
	case IER:
		if (lcr & 0x80)	// DLAB
			dlm = wdata;
		else
			ier = wdata;
		break;
	case FCR:
		fcr = wdata;
		break;
	case LCR:
		lcr = wdata;
		break;
	case MCR:
		mcr = wdata;
		break;
	case LSR:
		break;
	case MSR:
		break;
	case SCR:
		scr = wdata;
		break;
	}

	return 1;
}

void UART16550_AXI4L::fsmUser()
{
	if (ier & 2)
		nxt_intrFlag = 1;
	if ((ier & 1) && ready(IN))
		nxt_intrFlag = 1;
}
