# M7 exploration 1 — rv906 current debug seams (verified facts, c0d3895)

Agent report 2026-09-11. Paths relative to the worktree root.

## 1. rtl/CSR.v — the M4 zero-trigger escape

- Trigger CSR block `rtl/CSR.v:1289-1304`: five constant wires `tselect_value /
  tdata1_value / tdata2_value / tdata3_value / tcontrol_value` all `64'd0`
  ("zero-trigger escape hatch, D-M4-5; real triggers arrive at M7").
- Read mux arms `rtl/CSR.v:1578-1582` return 0 in any legal privilege mode;
  RMW operand mux `rtl/CSR.v:1606-1611` computes from `csr_rdata`.
- Writes: **accepted-but-ignored, no trap**. No `csr_wen && (csr_addr == CSR_T*)`
  write enable exists anywhere (full inventory `rtl/CSR.v:544-1472`); the pulse
  lands on no flop. `csr_access_illegal` (`rtl/CSR.v:1707-1708`) has NO
  address-existence check — unknown CSRs read 0 / write-dropped.
- In S/U mode 0x7Ax decodes `[9:8]=11` ⇒ `csr_priv_bad` ⇒ illegal trap vec 2.
- Address constants `rtl/rvproc_pkg.sv:317-322`: TSELECT 0x7A0, TDATA1 0x7A1,
  TDATA2 0x7A2, TDATA3 0x7A3, TCONTROL 0x7A5. (TDATA4 0x7A4 absent.)
- D-M4-5 quote, `docs/superpowers/specs/2026-08-31-m4-priv-mmu-pmp-design.md:75`:
  "Debug-trigger escape CSRs — tselect/tdata1/tdata2/tcontrol (0x7A0-7A5)
  real-but-zero-triggers so rv64mi-p-breakpoint's 'unsupported type' skip fires;
  M7 replaces with real triggers."
- **dcsr(0x7B0)/dpc(0x7B1)/dscratch0(0x7B2)/dscratch1(0x7B3): ABSENT
  everywhere** — grep over rtl/ returns nothing. Today they hit the default
  read arm (0) with writes dropped; no debug-mode access gate exists.

## 2. rtl/RTU.v — trap/flush machinery with pre-built dead seams

- Exception priority chain `rtl/RTU.v:858-887` (donor order): leg 1
  `retire_pending_bkpt_expt = 1'b0` (`:870`, "no DTU"); leg 2 interrupt (live
  since M6) with `dtu_int_mask_tied0 = 1'b0` (`:852`) — the donor's
  `dtu_rtu_int_mask` (dcsr.step && !stepie) single-step mask slot; leg 3 LSU
  async (tied 0 at LSU.v:3056); leg 4 `retire_bkpt_expt = 1'b0` (`:873`) —
  the timing-0 trigger slot; leg 5 sync EX1.
- Trap-taken ack `rtl/RTU.v:931-933`: donor's `!halt_req && !dbg_mode_on`
  qualifiers structurally absent — M7 adds them so a halt defers a pending trap.
- Flush FSM `rtl/RTU.v:936-1008`, states IDLE/FE/WAIT/BE/FE_BE; the dead
  async-halt shortcut exists: `halt_req_dm_async_tied0 = 1'b0` (`:951`) with
  the `IDLE -> FE_BE -> IDLE` transition (`:997-998`) already wired.
  `retire_inst_flush_fe_set` (`:966-967`) omits the donor's
  debug_flush/halt_req terms per comment `:968-969`.
- Changeflow-PC mux `rtl/RTU.v:1031-1033` — comment: "no DTU
  retire_exit_debug/dtu_rtu_dpc leg in M2"; dret's redirect target has a
  documented missing landing spot. `ex1_inst_chgflw` `:725` (mret-only today).
- `rtu_yy_xx_dbgon` output declared `:338`, driven `1'b0` at `:1129` ("no DTU
  in M2"); dangling at the instantiation `rtl/RVProc.v:1347` (comment
  `RVProc.v:1241-1246`). In the donor it fans out to CP0/IU/LSU/VPU/MMU/HPCP
  (= dbg_mode_on).
- No debug/halt INPUT ports on RTU (port list `:162-404` enumerated).
- Donor reference already extracted:
  `docs/superpowers/specs/notes/2026-08-20-c906-rtu-extraction.md:125-151`
  (priority legs, trap-vs-halt), `:200-204` (async halt shortcut), `:249-251`
  (rtu_dtu_* block, aq_rtu_retire.v:1176-1235), `:346-349` (debug halt shares
  the flush FSM); `notes/2026-09-08-m6-donor-interrupt-machinery.md:176-186`
  (dtu_rtu_int_mask), `:246-259` (WFI wake incl. dtu_cp0_wake_up), `:95-98`
  (donor pm FSM debug-exit arm `pm_wdata = dtu_cp0_dcsr_prv`).

## 3. rtl/IDU.v — breakpoint decode today

- CORRECTION: rv906 uses standard 5-bit mcause codes, not a 2-bit vec
  encoding. `CAUSE_BREAKPOINT = 5'd3` (`rtl/rvproc_pkg.sv:362`).
- ebreak LIVE: `is_ebreak` (`rtl/CSR.v:334`, `func==CP0_FUNC_EBREAK`) → vec 3
  via `rtl/CSR.v:1725`; c.ebreak via `rtl/IDU.v:1373-1374`; SYSTEM decode
  `rtl/IDU.v:836-840` (key `15'b000000000011100`).
- Execute trigger: DEAD (RTU leg 4 tied 0). Fetch side: IFU tags per-instruction
  faults (`ifu_idu_id_fault_pgflt/_accflt`, `rtl/IFU.v:94-95`); **IDU carries no
  PC at all** (`rtl/IDU.v:96-99` documented gap) — PC lives in IU's pcgen and
  surfaces as `iu_rtu_ex1_cur_pc` / `ex2_cur_pc` (`rtl/RTU.v:191,760`).
  M7 execute-trigger matching must add a PC to the ID/EX1 payload or match at
  retire. Data side: LSU fault generation `rtl/LSU.v:3043-3050`.
- dret: falls to the illegal default `rtl/IDU.v:1167-1170`; `CP0_FUNC_DRET`
  deliberately not pinned (`rtl/rvproc_pkg.sv:483-487`). Donor decd.v:2111-2126.

## 4. Top level

- `rtl/RVProc.v:34-140` (core shell): single `clk`/`rst_n`, AXI I ch[0] + D
  ch[1] (512-bit), mtip/msip/meip/mtime, `quitted`. Instances: icache :628,
  ifu :683, bpu :749, mmu :801, idu :859, iu :975, fpu :1087, lsu :1130,
  rtu :1247, csr :1397, pmp :1510.
- `rtl/RVProcAXI.v:25-147` (Verilator top, VDUT): `clk`/`rst_n` only + mpin
  mem pins + UART AXI slave ch[2] + `G_io_pins_uart_irq` (M6 level-input
  precedent) + `G_RVProcAXI_OUT`. NO JTAG pins — M7 adds TCK/TMS/TDI/TDO here.
  Fabric: AXICrossbar 2 masters x 4 slaves `:749-818` (`N_MASTERS=2` :236);
  slave map :152-165; core `u_core` :825-904; CLINT :637-664; PLIC :669-699;
  MEMCTL :909. Single clock domain throughout (`rtc_tick` = /100 divider,
  `:377-393`).
- Reset driven entirely in C++: `dut.cpp:41-52` (DUT::init). A JTAG trst_n /
  DM-reset pin is a new top-level pin + a domain decision (spec: DM must
  survive core reset).

## 5. Testbench infra

- `RVProcTest.cpp:53-64` per-step structure: drive AXI slave inputs, sample
  irq pre-clock, `dut.step(&mpin, uart_irq)`, read, `dut.sync(cpu)`.
  Post-clock device service order `:517-547` is load-bearing.
- `dut.cpp:69-119` DUT::step: inputs driven pre-edge, `clk=1 eval` commits,
  outputs sampled after rising edge.
- Program load / PASS: `testbench/TestBench.cpp:188-199` (ELF + symbols
  {tohost,...}), run loop `:241-325` polls `read_mem(tohost)`, verdict
  `:329-362` (PASS iff x3==1). tohost-symbol gotcha applies to any new
  directed ELF.
- JTAG driver needs: TCK/TMS/TDI drive + TDO capture sub-evals inside
  DUT::step's low/high eval structure (TAP advances on rising TCK, TDO
  changes on falling TCK); the M6 `G_io_pins_uart_irq` pre-clock-drive pattern
  (`dut.cpp:80-85`) is the template.
- Unit-bench pattern `test/m2/unit/Makefile`: explicit per-module target
  blocks (CSR :41-52, IU :57-68, RTU :73-84, IDU :89-100, FPU :174-184),
  `VFLAGS = --cc --exe -O2 -Wno-fatal --compiler clang` (:31), aggregate `run`
  :190-202 → UNIT-SUITE-PASS/FAIL. Bench idiom: tick/reset_dut/check/
  test_result (rtu_tb.cpp:135-173).
- SoC acceptance precedent: `test/m6/run_linux.sh` (marker-gated,
  LINUX_TIMEOUT) and `test/m6/run_all.sh` (ELF loop, 300s timeout each).
- `rtl/verisim.h:44-45` exposes `gpr_r` / `ex2_cur_pc` (verilator public,
  IDU.v:1596, RTU.v:760, frf_r IDU.v:1682) — C++ can shadow-check state.

## 6. rv64mi-p-breakpoint (test/rv-test/riscv-tests/isa/rv64mi/breakpoint.S)

- `:24` `csrs tcontrol, 8` (mte) — may trap; today ignored, no trap.
- `:33-35` tselect readback 0 ⇒ fall through.
- `:38-43` write tdata2/tdata1 (mcontrol type2|M|EXECUTE), read back,
  `bne → 2f` — the "unsupported type" escape; today readback 0 ⇒ skips the
  whole execute-trigger section. Same skips at :56-60 (load), :75-79 (store).
- `:90-93` `csrw tselect,1; csrr; bne x0,a1,pass` — **today reads 0 ⇒ test
  PASSES here**. A single-trigger implementation (tselect stuck 0) still
  passes at this point.
- `:95-111` (only with ≥2 triggers): second trigger fires on data2 load;
  first still fires on data1 store; store must not commit.
- Handler `:118-130`: even TESTNUMs trap, `mcause==3`, `mepc+=4; mret`
  (skip triggering instruction).

**M7 consequence:** with real mcontrol triggers, the :43/:60/:79 skips fall
through and the core must trap cause 3 at the right PC (mepc+4 skip), trap on
triggered loads, and SUPPRESS triggered stores. With a single trigger
(tselect stuck 0), :93 still passes.

## SEAMS (M7 insertion points)

RTL new modules: DTU (triggers + halt FSM, donor aq_dtu_ctrl analogue),
JTAG DTM + DM (donor tdt_dtm/tdt_dm), SBA AXI master into the fabric.

rtl/CSR.v: replace the five zero constants `:1289-1304` with real trigger
storage + write enables; read mux `:1578-1582` real values; add
dcsr/dpc/dscratch0/1 arms; `csr_access_illegal` `:1707-1708` debug-mode gate
for 0x7B0-0x7B9; pm FSM mux `:577-582` debug-exit arm; trap-entry capture
gating (no architectural capture when halting into debug); dret decode
beside `:333-340` with chgflw to dpc.

rtl/RTU.v: ports `:162-404` add dtu inputs + drive `rtu_yy_xx_dbgon` `:1129`;
legs 1/4 `:870,:873` real trigger fires; `dtu_int_mask_tied0` `:852` real;
trap-taken gate `:931-933` add `!halt_req && !dbg_mode_on`; flush FSM
`:951/:966-969/:997-998` async shortcut + fe_set terms; chgflw mux
`:1031-1033` exit-debug/dpc leg; `ex1_inst_chgflw` `:725` dret source.

rtl/IDU.v: dret arm `:1167-1170` + `CP0_FUNC_DRET` in rvproc_pkg.sv.

rtl/IFU.v: `rtu_ifu_dbg_mask` / dbgon consumption (fetch halt while entering
debug); donor `ifu_rtu_reset_halt_req` (halt-on-reset) if kept.

rtl/LSU.v: AG-stage address/data trigger match (load/store mcontrol) + store
suppression (breakpoint.S:70).

rtl/RVProc.v: dtu/DM instance; connect `.rtu_yy_xx_dbgon()` `:1347` to real
consumers.

rtl/RVProcAXI.v: JTAG pins + DTM/DM instance; SBA as third crossbar master
(N_MASTERS 2→3, `:236`) or dedicated aperture; DM reset-domain decision.

Testbench: dut.cpp JTAG pin drive/sample; RVProcTest.cpp TAP/DMI driver
beside the uart_irq precedent; verisim.h debug-reg macros; acceptance scripts
per run_all.sh / run_linux.sh shapes; re-baseline rv64mi-p-breakpoint.
