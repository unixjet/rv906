# M7: RISC-V Standard Debug — JTAG DTM + Debug Module + SBA + core-side DTU

Status: COMPLETE (2026-09-24) — all tasks landed (T0 2b5290c, T1 903bcb8,
T2 5fb6c34, T3 a539a59, T4 04a048c, T5 0e90039, T6 b364081, T7 abd7008,
T8 49849d0, T8b ceb6834, T9 f37e4d7); T10 = final battery on the
M8-fixed tree + this status + 08-verification §8.18 final-battery block.
M7-DEBUG-PASS (58,540 TCK) + the full standard battery green in one run
(controller-verified on HEAD=0cff95c); see 08-verification.md §8.18.
Design settled 2026-09-11. Exploration notes (all facts pinned to
`file:line` in these):
- `notes/2026-09-11-m7-rv906-debug-seams.md` — rv906 insertion points
- `notes/2026-09-11-m7-donor-debug-machinery.md` — donor tdt/dtu architecture,
  register map, WIRING MAP
- `notes/2026-09-11-m7-debug-test-strategy.md` — donor debug tooling, spec
  version, unit/SoC test split, rv64mi-p-breakpoint interaction, risks,
  SBA reachability, rv12 precedent (none)

Design-doc M7 row (`2026-08-20-rv906-c906-clone-design.md` §7.4): *"M7 |
RISC-V standard Debug (JTAG DTM + DM + SBA, core-side `dtu` hooks) | debug
register/halt-resume/breakpoint test cases"*.

## Context

M6 is closed (c0d3895): interrupts + Linux 6.5 boot, all gates green
(unit suite, sweep 86/87, atomics 19/19, m4 30/30 + v 85/86, m5 46/46,
m6 8/8, Linux banner). M7 delivers RISC-V standard debug (spec 0.13 —
the donor's conformance level, pinned by `TDT_DM_VERSION=2`,
`DTM_VERSION=1`, `dcsr.xdebugver=4'b0100`, `sbversion=1`) on top of the
existing Verilator/C++ harness (no OpenOCD — the C++ testbench IS the
debug host).

## What exists today (dead seams to revive)

From the seams note (c0d3895):
- `rtl/CSR.v:1289-1304` — M4 D-M4-5 zero-trigger escape: five constant-0
  wires for tselect/tdata1-3/tcontrol; reads return 0, writes dropped.
  `dcsr`/`dpc`/`dscratch0/1` (0x7B0-0x7B3) absent entirely.
- `rtl/RTU.v` — 5-leg exception priority with dead leg 1
  (`retire_pending_bkpt_expt`, :870) and leg 4 (`retire_bkpt_expt`, :873),
  dead `dtu_int_mask_tied0` (:852), dead async-halt shortcut
  (`halt_req_dm_async_tied0`, :951; `IDLE→FE_BE→IDLE` already wired
  :997-998), trap-taken gate (:931-933) missing the donor's
  `!halt_req && !dbg_mode_on`, chgflw mux (:1031-1033) missing the
  exit-debug/dpc leg, `rtu_yy_xx_dbgon` driven 0 (:1129, dangling at
  RVProc.v:1347).
- `rtl/IDU.v:1167-1170` — dret falls to the illegal default
  (`CP0_FUNC_DRET` intentionally unpinned, rvproc_pkg.sv:483-487).
- `rtl/IFU.v` — single-issue fetch (`ifu_idu_id_inst`/`_vld`), carries no
  PC to IDU, no debug-inst injection, no dbg ports.
- `rtl/RVProcAXI.v` — no JTAG pins; AXICrossbar N_MASTERS=2 × 4 slaves
  (:236-238), M_DATA_WIDTH=512 (:238); slave map MEM 0x80000000/2G,
  CLINT, PLIC, UART (:152-159).

## Architecture

Three new RTL modules, mirroring the donor's three-part split:

1. **`rtl/TDT_DTM.v`** (new) — JTAG side, `tck` domain. TAP-5 (16-state
   IEEE 1149.1), TDRs IDCODE/DTMCS/DMI/BYPASS, and the DMI→APB bridge
   (donor `tdt_dtm_top.v` + `tdt_dtm_chain.v` + `tdt_dtm_idr.v` +
   `tdt_apb_master.v` + `tdt_dmi_pulse_sync.v`). 5-bit IR
   (IDCODE=5'h01, DMI_ACC=5'h02, DTMCS=5'h10, DMI=5'h11); DMI DR =
   `{addr[9:0], data[31:0], op[1:0]}` (44 bits, abits=10, op 01=rd/10=wr);
   per-op APB transaction with paddr = dmi_addr<<2, 7-idle-cycle budget,
   tck↔clk pulse syncs only on the request/ready pulses (payload held
   stable in flight — donor contract, `tdt_apb_master.v:128,216,230-251`).
2. **`rtl/TDT_DM.v`** (new) — Debug Module + SBA, `clk` domain. APB slave
   (register map below), abstract-command engine that reaches the core
   ONLY through the `itr` instruction channel + dscratch0/1 data path
   (donor `tdt_dm.v`: the DM never touches the GPR file directly —
   GPR read = itr `csrrw x0, dscratch0, x{regno}`; GPR write = wr dscratch0
   + itr `csrrc x{regno}, dscratch0, x0`; CSR access = REGACC FSM saving/
   restoring x6 through dscratch1), progbuf (4 entries + implied ebreak
   at index 4), SBA registers + FSM + `tdt_sba_axi` clone (single-beat
   128-bit AXI4 master, wstrb lane-shifted, prot=3'b010).
3. **`rtl/DTU.v`** (new) — core-side debug unit, `clk` domain, `rst_n`.
   Owns dcsr/dpc/dscratch0/1 AND all trigger CSRs (donor split:
   `aq_dtu_ctrl.v` + `aq_dtu_trigger_module.v` + `aq_dtu_mcontrol*.v` +
   `aq_dtu_iie_trigger.v` + `aq_dtu_mcontrol_output_select.v`), reached
   from CSR.v through the donor's cp0↔dtu port
   (`cp0_dtu_addr/wdata/wreg/rreg` ↔ `dtu_cp0_rdata`) — the M4 zero-
   trigger escape arms in CSR.v become routes to DTU.v. Builds 22-bit
   `halt_info` bundles (cause + timing + action) that the IFU/LSU latch
   onto the in-flight instruction and deliver to the RTU at retire.
   **rv906 single-issue ⇒ DTU fetch slot 1 is tied vld=0.**

Supporting edits:
- **CSR.v** — 0x7A0-0x7A5/0x7A8/0x7AA/0x7B0-0x7B3 decode arms routed via
  cp0↔dtu; `csr_access_illegal` unchanged (privilege rules already handle
  [9:8]); pm FSM gains the debug-exit arm — donor
  `aq_cp0_trap_csr.v:591-631`: `pm_wen |= rtu_cp0_exit_debug`, and the
  exit arm wins: `pm_wdata = dtu_cp0_dcsr_prv` (donor :605-606); and
  `pm = pm_bits | {2{dbgon}}` (donor :630 — in debug mode the effective
  priv is always M). MPP/SPP untouched on exit (donor has no arm for it).
  `rtu_cp0_exit_debug` := `dbg_mode_on_after_req && (dtu_rtu_resume_req ||
  (retire_vld && dret))` (donor `aq_rtu_retire.v:756-760`).
- **RTU.v** — `rtu_yy_xx_dbgon` real; halt entry: `halt_req` from
  dm-sync (timing-1, honored at a non-split retire boundary), pending
  (trigger t1), trigger t0, ebreak-in-debug, reset-halt, step;
  `dbg_mode_on` set after the FE flush, reported as `dbgon`; cause
  priority (donor :778-793): async(8) > pending(halt_info[11:8]) >
  trigger(2) > ebreak(1) > reset(5) > dm_sync(3) > step(4) — the async
  term stays tied 0 (D-M7-7). Trap-taken gate gains `!halt_req &&
  !dbg_mode_on` (:931-933); chgflw gains the exit-debug leg (dpc /
  dret); dpc latches `rtu_dtu_dpc` at `rtu_dtu_halt_ack`; while
  `dbg_mode_on` the pcgen is overridden by dpc and dpc += 4 per retired
  debug instruction (no chgflw); leg 1 (:870) and leg 4 (:873) go real;
  `dtu_int_mask_tied0` → `dcsr.step && !dcsr.stepie` (:852).
- **IFU.v** — inputs `dtu_ifu_debug_inst[31:0]` + `_vld` (donor
  `aq_ifu_ibuf.v:949,1085,1098`: when `dbgon && debug_inst_vld` the fetch
  entry is the injected word), `dtu_ifu_halt_info[21:0]` + `_vld`
  (latched onto the fetch packet, carried to RTU at retire),
  `dtu_ifu_halt_on_reset` (first fetch after reset ⇒ reset-halt, cause 5);
  outputs `ifu_dtu_exe_addr[39:0]` + `_vld` (fetch PC — the trigger
  match operand; rv906's IFU has no PC on the IDU path, so the PC
  comparison lives in the DTU fed from the IFU, exactly the donor shape).
- **LSU.v** — outputs `lsu_dtu_ldst_addr[39:0]`/`_vld`,
  `lsu_dtu_ldst_data[63:0]`/`_vld`, `lsu_dtu_ldst_type[1:0]` (10=st,
  01=ld), `lsu_dtu_ldst_bytes_vld[15:0]`, `lsu_dtu_mem_access_size[2:0]`
  (from the AG/REPLY access); inputs `dtu_lsu_halt_info[21:0]` + `_vld`
  (latched into the reply, delivered at retire) and
  `dtu_lsu_addr_trig_en`/`dtu_lsu_data_trig_en` (trigger-hit store
  suppression — a matching store must not commit, breakpoint.S contract).
- **IDU.v** — dret decode arm (SYSTEM, funct7=0, funct3=0, csr-field
  0x7B2) → `CP0_FUNC_DRET` (pinned in rvproc_pkg.sv) +
  `dp_retire_ex2_inst_dret` to the RTU.
- **RVProc.v** — DTU instance + the dtu↔cp0/rtu/ifu/lsu/hpcp nets;
  `rtu_yy_xx_dbgon` at :1347 goes live.
- **RVProcAXI.v** — JTAG pads `tck/tms/tdi` in, `tdo` out (4 pins;
  D-M7-2); `tdt_rst_n` pin (D-M7-3); TDT_DTM + TDT_DM instances;
  SBA AXI master joins the crossbar as master #3 (D-M7-4).
- **`rtl/SBA_AxiUp.v`** (new, rv906 glue) — 128→512 up-conversion of the
  SBA master for the crossbar (wdata zero-extend, wstrb expanded into
  the 64-byte beat per addr[5:0], rdata lane-extracted back to 128 at
  addr[5:0]; single-beat in and out). 40→64-bit address zero-extension.
  (The donor's crossbar is APB-in/AXI-out SoC fabric; rv906's is a plain
  512-bit AXI crossbar, so the adapter is rv906-specific.)

### DM register map (word offsets, spec 0.13; donor `tdt_dm.v:171-227`)

data0/1 0x04/0x05 (datacount=2); dmcontrol 0x10; dmstatus 0x11
(version=2, impebreak=1, haresethaltreq=1, authenticated=1); hartinfo 0x12
(nscratch=2, dataaccess=0); hawindow 0x15; abstractcs 0x16
(progbufsize=4, datacount=2, busy, cmderr[10:8]); command 0x17;
abstractauto 0x18; nextdm 0x1d (=0); **itr 0x1f** (custom, the abstract-
command instruction channel); progbuf0-3 0x20-0x23 (reads 0x24+ = 0);
dmcs2 0x32 (=0); sbcs 0x38 (sbversion=1, sbasize=40,
sbaccess_info=5'b11100); sbaddress0/1 0x39/0x3a; sbdata0-3 0x3c-0x3f;
haltsum0 0x40 (=hartsum0 = {31'b0, halted}); compid 0x7f.

dmcontrol (donor :663-836): 31 haltreq (level, timing-1), 30 resumereq
(one-shot pulse, only when halted), 29 hartreset, 28 ackhavereset, 26
hasel, bit3 setresethaltreq → halt_on_reset, bit2 clrresethaltreq, bit1
ndmreset, bit0 dmactive (gates DM out of reset via sync_rst — D-M7-9).

### Core interface (DM↔DTU, all `clk` in rv906 — D-M7-1)

The donor's `aq_dtu_cdc` crossing degenerates to same-clock 1-cycle
handshakes (pulses may be direct; the debugger polls, no latency is
architecturally significant). Signals (donor WIRING MAP, note 2 §6):
DM→DTU `halt_req` (level), `resume_req` (pulse), `halt_on_reset` (level),
`ack_havereset` (pulse), `itr[31:0]` + `itr_vld` (pulse+payload),
`wr_vld` (pulse) + `wr_flg[1:0]` + `wdata[63:0]` (00=rd dscratch0,
01=wr dscratch0; 10/11 latest_pc/satp dropped with CUSCMD, D-M7-6).
DTU→DM `halted` (=dbgon), `havereset` (until ack), `itr_done` (pulse per
retired itr inst), `retire_debug_expt_vld` (pulse, abstract cmderr=3),
`wr_ready` (pulse), `rx_data[63:0]` (per wr_flg: dscratch0).
DM→pads: `ndmreset_n`, `hartreset_n` (chip outputs in the donor; in
rv906 they are top-level outputs on RVProcAXI, dangling in the harness —
documented).

## Decisions (D-M7-*)

- **D-M7-1 — two clock domains, not three.** Donor: tck / sys_apb_clk
  (DM) / forever_cpuclk (DTU, SBA, core). rv906 is single-clock: DM and
  SBA run on the core `clk`; the ONLY remaining CDC is tck↔clk in the
  DMI bridge (the donor's own tdt_apb_master syncs). `aq_dtu_cdc`
  degenerates as above.
- **D-M7-2 — 4 JTAG pads, no trst_n.** The TAP resets via the standard
  5×TMS=1 sequence. The donor's `pad_dtm_trst_b`, `jtag2_sel`, `tap_en`
  pads and the TAP2 custom 2-wire protocol (`tdt_dtm_ctrl.v:239-343`) are
  dropped (T-Head custom).
- **D-M7-3 — separate DM reset pin `tdt_rst_n`.** Mirrors the donor's
  `ciu_rst_b` (independent of `cpurst_b`): the DM survives a core reset
  (attach-after-crash / ndmreset flow). Testbench drives it from C++.
- **D-M7-4 — SBA = crossbar master #3.** N_MASTERS 2→3 with the
  128→512 up-adapter; 40-bit SBA address zero-extended to 64. The donor
  **never validated SBA in sim** (SoC ties the SBA port off,
  `tr_axi_interconnect.v:861-895`; the donor's own pattern uses
  progbuf, not sbcs) ⇒ SBA is a donor-untested surface: gate against
  spec 0.13 text + the rv906 crossbar, recorded here.
- **D-M7-5 — 10 triggers, iie type-3 only.** 8 mcontrol (type 2, full
  0.13 tdata1 layout, match 0-5: eq/NAPOT/ge/lt/low32/up32; timing forced
  0 for execute; action 0=bkpt exception, 1=enter-debug) + 2 iie slots
  supporting ONLY type 3 (icount, count hardwired 1, donor). T-Head types
  4 (itrigger) / 5 (etrigger) unsupported — writes read back type=0;
  `tinfo`[9:4] = 6'b001000 (donor 6'b111000). `tselect[3:0]` WARL-clamps
  at 9 (donor `aq_dtu_m_iie_all.v:375-386`).
- **D-M7-6 — T-Head diagnostic CSRs dropped.** DM cuscs/cuscmd/cusbuf0-7
  (0x70-0x79) read 0 / writes ignored; DTU haltcause 0xfe0, dbgfifo
  0xfe1, pcfifo 0xfe2 same (consistent with rv906's unknown-CSR
  behavior: read 0, write dropped, no trap). M4 D-M4-5 precedent.
- **D-M7-7 — no custom async halt.** The CUSCMD-0 async (timing-0) halt
  request is dropped: `dtu_rtu_async_halt_req` tied 0 and the RTU's
  `IDLE→FE_BE` shortcut stays dead-but-present (clone discipline, like
  the M4/M6 dead seams). Only the standard timing-1 `haltreq` is live.
- **D-M7-8 — single-issue DTU.** IFU fetch slot 1 tied vld=0 (donor's
  2-slot geometry collapses).
- **D-M7-9 — DM not clock-gated.** The shared clk cannot be gated by
  dmactive; instead dmactive=0 holds the DM in reset via the donor's
  `sync_rst` (donor does both gate + reset; the reset alone is
  sufficient).
- **D-M7-10 — kept from the donor as-is.** itr + progbuf(4 + implied
  ebreak) abstract-command mechanism; halt_on_reset (cause 5);
  datacount=2; progbufsize=4; sbasize=40; SBA 128-bit DW,
  sbaccess 2/3/4; IDCODE 0x10000B6F; DMI abits=10, idle=7;
  dmversion=2 / dtmversion=1 / dcsr.xdebugver=4'b0100; 0.13 field
  layouts throughout.

### OFF-path identity argument

At reset, with JTAG idle (TAP in Test-Logic-Reset, TMS held 0 ⇒ no
scans) and no DM traffic: dmactive=0 ⇒ DM in reset (all core-side
outputs idle), all trigger CSRs reset-disabled (type=0 ⇒ no match),
`tcontrol.MTE=0`, TDO=1. Every new RTU/CSR/IFU/LSU input is tied to its
current constant (legs 1/4 = 0, int_mask = 0 (step=0), dbgon = 0,
debug_inst_vld = 0, halt_on_reset = 0, trig_en = 0). This is the exact
shape of the M6 Task 2 gate: **every existing test's behavior is
bit-identical with the debug unit present but inactive.** The SBA master
is in reset (no AXI requests). The only observable changes are the new
JTAG/tdt_rst_n pins (untouched) and reads of the now-existing debug CSRs
(no existing test touches them — rv64mi-p-breakpoint does, and its
expected behavior is re-baselined in task 8).

## Verification plan

**Unit layer** (new `test/m7/unit/`, same Makefile pattern as
`test/m2/unit/`):
- `dm_tb` — TDT_DM alone with a ~100-line C++ APB master + fake core
  responder (halted after N cycles on halt_req; itr_done after M on
  itr_vld; shadow regfile on wr_vld; rx_data from it). Rows: register
  defaults table (the C906_DEBUG_PATTERN.v value table, note 3 §1),
  halt/resume handshake + haltsum0, abstract cmd GPR/CSR + busy + cmderr
  1/2/3/4, ndmreset/dmactive, SBA register FSM vs fake AXI slave.
- `dtm_tb` — TDT_DTM alone vs C++ fake APB slave; TAP walk
  (TLR→IDLE→Shift-IR→Shift-DR), IDCODE 0x10000B6F, dtmcs
  (abits=10/version=1/idle=7), 44-bit DMI scan, DMI busy/retry.
- **Core-side rows extend the existing benches** (no new core benches):
  csr_tb gains dcsr/dpc/dscratch + trigger CSR WARL/readback rows;
  rtu_tb gains halt/resume/step/priority-leg rows (its header already
  documents legs 1/4 as structurally dead — M7 makes them live, the
  bench grows the input ports).

**SoC layer** — `RVProcAXI.v` JTAG pads + a C++ JTAG driver in
`RVProcTest.cpp` (port of the donor's `ext_debug` class,
`smart_run/tests/cases/debug/JTAG_DRV.vh` — its poll/timeout constants
and bit positions are the de-facto contract: `jtag_tlr`, `jtag_write_ir`,
`jtag_shift_dr`, `dmi_rw` + busy-poll, `halt_req`, `resume`,
`access_register_by_abscmd`, `execute_itr`, `access_memory_by_sb`).
Clock discipline: the driver owns both clocks with a fixed interleave
(≥4 clk edges per TCK phase — the donor ratio TCK=clk/4, idle=7); TDO
sampled after the TCK falling-edge eval (donor TDO is negedge-
registered); TMS/TDI never toggled while a DMI op is in flight;
clk and tck never toggled in one eval. `--m7-debug` testbench mode runs
a fixed script against a loaded ELF and prints `M7-DEBUG-PASS/FAIL`;
`test/m7/run_debug.sh` mirrors `test/m6/run_all.sh`.

**Test case priority** (note 3 §2): (1) register readbacks via JTAG;
(2) halt/resume/step + haltsum0; (3) abstract GPR/CSR r/w + cmderr-4;
(4) mcontrol execute/load/store match + second trigger + store
suppression + **rv64mi-p-breakpoint through the REAL-trigger path**;
(5) SBA r/w of the MEM aperture; (6) full battery OFF-path identity.

**rv64mi-p-breakpoint re-baseline (task 8 gate).** With real triggers
the :43/:60/:79 "unsupported type" skips fall through. The test
requires bit-exact tdata1 readback (type=2, dmode, M, execute/load/
store, timing=0, action=0, match=0), a cause-3 trap BEFORE execution
(mepc+4 skip in the handler), a trap on triggered loads, a suppressed
triggered store, and tselect=1 readback = 1. Gate = the stock ELF PLUS
a directed `test/m7/break_no_skip.S` variant that counts the traps
through tohost — if any sub-test silently takes a skip branch the trap
count is wrong and the test FAILs (no hang).

**SBA e2e (task 9):** sbcs readback (sbversion=1, sbasize=40,
sbaccess 32/64/128), sbaddress/sbdata write+read of a pattern in the
MEM aperture (byte-lane proof — read back what the test program wrote
first), tohost-region read, sberror on misalign/unsupported access.
Donor-untested surface (D-M7-4).

**Gates.** Per RTL-touching task: `make verisim` clean, unit suite
(UNIT-SUITE-PASS incl. new m7 benches), sweep 86/87 (documented
rv64ui-p-ma_data), atomics 19/19. Extended battery (m4 30/30 + v 85/86,
m5 46/46, m6 8/8, Linux 600s banner) at tasks 5, 8, 10. Every task's
commit must leave the OFF-path identity argument intact (new inputs at
their reset constants).

## Tasks (ordered, one gate each)

| # | Task | Contents | Gate |
|---|------|----------|------|
| 0 | Design doc + extraction notes (this file + 3 notes) | — | committed |
| 1 | Core-side debug state | DTU.v skeleton (dcsr/dpc/dscratch0/1, cp0↔dtu port, wake_up); CSR.v 0x7B0-0x7B3 arms + debug-mode write gate + pm exit arm + `pm|dbgon`; RTU dbgon + timing-1 halt entry/flush + resume + step + int_mask + trap-taken gate + chgflw dret leg + dpc latch/advance; IFU debug-inst injection + halt_on_reset port; IDU dret decode + CP0_FUNC_DRET | csr_tb/rtu_tb new rows green; minimal gates green (OFF-path identity) |
| 2 | DTU triggers | 10-trigger storage + comparators (mcontrol 0-5, iie icount), tselect WARL, tdata readback bit-exact for the supported field set, tcontrol/mcontext/scontext/tinfo; IFU exe_addr export + halt_info latch/transport; LSU ldst feed + reply halt_info + store suppression; RTU legs 1/4 live | rv64mi-p-breakpoint REAL-path + break_no_skip.S; minimal gates |
| 3 | DM clone | TDT_DM.v per the register map above, abstract engine (GPR/CSR), cmderr 1-4, halt/resume/halt_on_reset interface, wr/rx data path, SBA regs+FSM+AXI master | `test/m7/unit/dm_tb` PASS |
| 4 | DTM clone | TDT_DTM.v TAP-5 + TDRs + DMI→APB bridge (tck↔clk syncs) | `test/m7/unit/dtm_tb` PASS |
| 5 | SoC wiring | JTAG pads + tdt_rst_n on RVProcAXI; TDT_DTM/TDT_DM instances; DTM↔DM↔DTU↔core nets; SBA crossbar master #3 via SBA_AxiUp.v | verisim builds; full extended battery green |
| 6 | JTAG C++ driver + smoke | dut.cpp tck/clk interleave; RVProcTest.cpp ext_debug port; `--m7-debug`; run_debug.sh | e2e: IDCODE, dtmcs, dmstatus.version==2, dmactive set/clear |
| 7 | Halt/resume/step/abstract e2e | C906_DEBUG_PATTERN.s-analog spin ELF; halt → anyhalted + dpc/dcsr; abstract GPR r/w; ITR; progbuf memory r/w at 0x1f005000-analog; step (exactly one inst); resume via resumereq AND dret | run_debug.sh e2e PASS |
| 8 | Trigger e2e | mcontrol execute action-1 (halt, dcsr.cause=2) + action-0 (cause-3 trap), load/store match, second trigger (tselect=1), store suppression; rv64mi-p-breakpoint + no-skip variant | run_debug.sh trigger section; rv64mi-p-breakpoint + m4/m6 re-run green; extended battery |
| 9 | SBA e2e | sbcs readback; MEM pattern r/w at 32/64/128-bit; tohost read; sberror cases | run_debug.sh SBA section PASS |
| 10 | Acceptance + close-out | full battery + all m7 e2e; docs/10-debug.md; 08-verification.md new §; design doc M7 → COMPLETE | all gates green in one run |

Sequencing: 1→2 strict (halt machinery before triggers fire into it);
3, 4 independent of 1/2 (parallelizable); 5 needs 3+4 (+1 for the DTU
nets); 6→7→8→9 strict on the driver; 10 last.

## Risks (from note 3 §5, in rank order)

1. No runnable donor oracle (the smart_run debug case can't elaborate;
   donor SoC ties SBA off) ⇒ pin every layout to file:line (done in the
   notes); port ext_debug semantics verbatim; gate SBA on spec text.
2. Two-clock DMI in a Verilated C++ harness (TDO negedge-registered;
   syncs need clk between TCK edges) ⇒ dtm_tb first; fixed interleave.
3. Halt/resume vs the M6-hardened RTU flush FSM (m6 8/8 is the trip
   wire) ⇒ rtu_tb rows before SoC work; m6 re-run after every RTU-
   touching task.
4. Trigger match timing vs rv906's IFU/retire path (no dtu ports today)
   ⇒ clone the donor shape (IFU PC feed, LSU addr feed, RTU at retire);
   directed rows per match class.
5. SBA 128→512 lane adaptation into the crossbar ⇒ byte-lane pattern
   proof in the e2e.

## Files expected to change

New: `rtl/DTU.v`, `rtl/TDT_DTM.v`, `rtl/TDT_DM.v`, `rtl/SBA_AxiUp.v`,
`test/m7/` (Makefile, run_debug.sh, unit/Makefile, unit/dm_tb.cpp,
unit/dtm_tb.cpp, debug_spin.S, break_no_skip.S, debug ELF),
`docs/10-debug.md`.
Edits: `rtl/{CSR,RTU,IFU,LSU,IDU,rvproc_pkg,RVProc,RVProcAXI}.v`,
`dut.cpp`, `RVProcTest.cpp`, `test/m2/unit/{Makefile,csr_tb.cpp,rtu_tb.cpp}`,
`docs/08-verification.md`.
