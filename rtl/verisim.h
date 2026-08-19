// verisim.h - Verilator signal paths for the rv906 DUT
// M0: the core is TestMaster scaffolding -- there is no CPU state to
// preload or mirror. M2 defines CPU_PC / CPU_GPR / cache-array paths here.
#include "VRVProcAXI.h"
#include "VRVProcAXI___024root.h"
typedef VRVProcAXI VDUT;
#define VERISIM_NO_CPU_STATE 1
