// verisim.h - Verilator signal paths for the rv906 DUT
//
// M1 (plan Task 4.2): the core is RVProc (front end only) with FetchSink
// standing in for IDU/IU/RTU/CP0. There is still no architectural register
// file to preload or mirror, so VERISIM_NO_CPU_STATE stays set (dut.cpp's
// init()/sync() CPU_PC/CPU_GPR paths are compiled out); M2 defines CPU_PC /
// CPU_GPR / cache-array paths here, same as M0's own header note promised.
//
// FetchSink exports two groups of `verilator public` registers:
//   * the CONFIG BANK, which the harness (Task 5.2) POKES after dut.init()
//     and before the first step (rung selection, stall mode, instruction
//     budget) and PULSES mid-test for --inv-test. FetchSink hosts it (plan
//     "Global contracts": harness config mechanism) -- Task 4.1 defines the
//     bits FetchSink itself needs (cfg_sink_stall/cfg_max_insts plus the
//     cp0_ifu_* chicken-bit passthroughs already on the Task 1 skeleton);
//     Task 5.2 owns extending/consuming this bank fully.
//   * the COMMITTED STREAM + resolve event/kind, which the harness (Task
//     5.2) SAMPLES every cycle and compares against the C++ fetch-ISS.
//
// Verilator keeps RVProcAXI / RVProc / FetchSink as separate model classes
// (verified against obj/verisim/VRVProcAXI_FetchSink__Tz3.h after a build --
// the "__Tz3" parameterization suffix is Verilator-generated from
// FetchSink's concrete parameter set and may change if that parameter list
// changes; re-grep obj/verisim/*.h for the current name if a build ever
// fails on this include), so the path is a chain of instance pointers
// rather than a flattened __DOT__ name. Everything goes through FSINK() so
// a future re-inlining is a one-line fix.

#include "VRVProcAXI.h"
#include "VRVProcAXI___024root.h"
#include "VRVProcAXI_RVProcAXI.h"
#include "VRVProcAXI_RVProc.h"
#include "VRVProcAXI_FetchSink__Tz3.h"

typedef VRVProcAXI VDUT;
#define VERISIM_NO_CPU_STATE 1

// `d` is a VDUT* (or anything with ->rootp).
#define FSINK(d)              ((d)->rootp->RVProcAXI->u_core->u_fetchsink)

//-----------------------------------------------------------------------------
// Config bank (plan "Global contracts": harness config mechanism). Rung
// selection is 3 bits (cfg_ras_en/cfg_btb_en/cfg_bht_en, plan Task 5.2 owns
// mapping --m1-rung=<1..4> onto them); cfg_icache_en/cfg_iwpe/
// cfg_icache_pref_en are the ICache-facing chicken bits (not a rung, per the
// M1 spec S4.3 -- ICache stays independently controllable). All of them
// zero-init (FetchSink.v header: "an un-poked run is rung 1").
//-----------------------------------------------------------------------------
#define M1_CFG_ICACHE_EN(d)      FSINK(d)->cfg_icache_en
#define M1_CFG_IWPE(d)           FSINK(d)->cfg_iwpe
#define M1_CFG_ICACHE_PREF_EN(d) FSINK(d)->cfg_icache_pref_en
#define M1_CFG_BHT_EN(d)         FSINK(d)->cfg_bht_en
#define M1_CFG_BTB_EN(d)         FSINK(d)->cfg_btb_en
#define M1_CFG_RAS_EN(d)         FSINK(d)->cfg_ras_en
#define M1_CFG_ICACHE_INV(d)     FSINK(d)->cfg_icache_inv
#define M1_CFG_BHT_INV(d)        FSINK(d)->cfg_bht_inv
#define M1_CFG_BTB_CLR(d)        FSINK(d)->cfg_btb_clr
#define M1_CFG_SINK_STALL(d)     FSINK(d)->cfg_sink_stall
#define M1_CFG_MAX_INSTS(d)      FSINK(d)->cfg_max_insts

//-----------------------------------------------------------------------------
// Committed stream + resolve event/kind (sampled once per simulated cycle).
// ONE slot, not three -- C906 delivers a single instruction/cycle to IDU
// (plan "Global contracts"), unlike rv12/C910's 3-wide commit group.
//-----------------------------------------------------------------------------
#define M1_CMT_VALID(d)       FSINK(d)->cmt_valid
#define M1_CMT_PC(d)          FSINK(d)->cmt_pc
#define M1_CMT_OPCODE(d)      FSINK(d)->cmt_opcode
#define M1_CMT_COUNT(d)       FSINK(d)->cmt_count

// resolve_event pulses the same cycle cmt_valid does, on any commit that
// required a FetchSink-driven redirect (FetchSink.v SECTION RESOLVE);
// resolve_kind: 0 none, 1 cond-branch redirect, 2 jal/c.j, 3 preturn
// (shadow-stack pop), 4 other jalr-family (indirect jump/call, JR_TARGET).
#define M1_RESOLVE_EVENT(d)   FSINK(d)->resolve_event
#define M1_RESOLVE_KIND(d)    FSINK(d)->resolve_kind

//-----------------------------------------------------------------------------
// Termination reporting (mirrors what lands on the tohost line)
//-----------------------------------------------------------------------------
#define M1_PERR_CODE(d)       FSINK(d)->perr_code

// perr_code values (FetchSink.v): 0 none, 1 --max-insts budget exhausted
// without reaching the sentinel. C906's thinner IFU->IDU interface (no PC/
// expt fields riding along, FetchSink.v TASK 4.1 FINDINGS item 1) cannot
// support most of rv12's richer PERR_* set (payload-PC mismatch, slot-valid
// gaps, checkpoint FIFO bookkeeping) -- there is nothing on this interface
// to check those against, so this enum stays deliberately thin.

// NOTE on --inv-test pacing (Task 5.2/6): the ifu_cp0_icache_inv_done /
// bht_cp0_inv_done replies are FetchSink INPUT ports, and Verilator names
// input ports with a __PVT__ prefix, which is not a stable interface to
// build a macro on -- poll the invalidate-sweep length instead (BHT_INV_
// CYCLES etc., rvproc_pkg.sv) the same way rv12's own harness settled on
// counting cycles rather than watching the replies.
