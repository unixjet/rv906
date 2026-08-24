// verisim.h - Verilator signal paths for the rv906 DUT
//
// M2 (plan Task 7): the core is the full integer pipeline -- RVProc
// (ICache+IFU+BPU) plus IDU + IU + LSU + RTU + CSR + the real MMU.
// FetchSink is retired: its harness config bank and its committed-stream /
// resolve / perr exports are gone with it, and the M1_* macros the old
// RVProcTest.cpp consumed are removed (that harness's M1 test paths are
// neutralized in RVProcTest.cpp with pointers at Task 8, which installs
// the per-retire trace harness -- contract 13 -- on top of what this
// header defines).
//
// CPU_PC / CPU_GPR are defined for real now (restore-checklist item 1),
// so dut.cpp's #ifndef VERISIM_NO_CPU_STATE init()/sync() blocks compile
// in:
//   * CPU_GPR -- IDU's architectural GPR array `gpr_r[0:31]`. Task 7.2
//     widened IDU.v's declaration from [1:31] to [0:31] so the register
//     number maps 1:1 onto the array index (x0's READ path stays
//     hardwired to 0 by gpr_read(); a writeback targeting x0 lands in
//     dead storage -- behavior bit-identical, only the array shape
//     changed).
//   * CPU_PC  -- RTU's EX2 latch `ex2_cur_pc`: the PC of the instruction
//     that just crossed EX1->EX2, i.e. the PC of the retiring
//     instruction when ex2_retire_vld is high.
//
// Verilator keeps each module as a separate model class, so the path is a
// chain of instance pointers: rootp->RVProcAXI->u_core->{u_idu, u_rtu}.
// VERIFIED against obj/verisim/VRVProcAXI_IDU.h and VRVProcAXI_RTU.h
// after a full build (re-grep those headers if a build ever fails on the
// includes: Verilator may rename public internals).

#include "VRVProcAXI.h"
#include "VRVProcAXI___024root.h"
#include "VRVProcAXI_RVProcAXI.h"
#include "VRVProcAXI_RVProc.h"
#include "VRVProcAXI_IDU.h"
#include "VRVProcAXI_RTU.h"

typedef VRVProcAXI VDUT;

// Used as `rootp->CPU_GPR` / `rootp->CPU_PC` (dut.cpp init()/sync()), so
// the macros expand to the member chain RELATIVE to the root pointer --
// NO outer parentheses: `rootp->CPU_GPR` must splice into a plain `->`
// chain, i.e. `rootp->RVProcAXI->u_core->u_idu->gpr_r`.
#define CPU_GPR RVProcAXI->u_core->u_idu->gpr_r
#define CPU_PC  RVProcAXI->u_core->u_rtu->ex2_cur_pc
