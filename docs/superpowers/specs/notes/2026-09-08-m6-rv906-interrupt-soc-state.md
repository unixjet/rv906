# M6 exploration note 1/3: rv906 current interrupt + SoC state

Source: Explore agent a2c5c066f368c18e5 (read-only, worktree), 2026-09-08.
All claims verified by direct file reads in the worktree; `$W` = worktree root.

## 1. CSR interrupt state (rtl/CSR.v)

**State: storage is largely LIVE (M4 upgraded mie/mip/mideleg beyond the M2
"RO pins" model), but NO interrupt-claim computation and NO export to RTU
exist. The header comment at `$W/rtl/CSR.v:28-29` ("mie/mip (mip read-only
wires from mtip/msip/meip)") is STALE — the M4 body supersedes it.**

- **mie (0x304) — LIVE full R/W storage.** `reg [63:0] mie_reg` at
  `$W/rtl/CSR.v:905`, write arms at :912-919: plain `mie_reg <= csr_wdata`
  on a mie write, plus an `sie` write arm that touches only delegated bits
  (`mie_reg <= (mie_reg & ~mideleg_reg) | (csr_wdata & mideleg_reg)`,
  :917-918). Read back at :1326. No write mask — all 64 bits stored as
  written.
- **mip (0x344) — hybrid: 3 RO pins + 3 writable S-flops.** Pins
  `mtip/msip/meip` are module inputs (`$W/rtl/CSR.v:271-273`).
  `wire mip_meip = meip, mip_mtip = mtip, mip_msip = msip` at :935;
  writable `ssip_f/stip_f/seip_f` flops at :906, written by mip (bits
  1/5/9, :926-929) and sip (SSIP only, :930-932). Assembly:
  `mip_value = {52'b0, meip,3'b0, mtip,3'b0, msip,3'b0} |
  (1<<9)*seip_f | (1<<5)*stip_f | (1<<1)*ssip_f` at :936-937. `sie/sip`
  views are mideleg-masked (:939-940). Section comment :896-904
  explicitly says "Interrupt delivery is M6; M4 provides the
  storage/views."
- **mideleg (0x303) — LIVE storage, mask {1,5,9}.** `reg [63:0]
  mideleg_reg` at `$W/rtl/CSR.v:503`; write `csr_wdata &
  64'h0000_0000_0000_022A` (:515-520). medeleg 16-bit at :502-513
  (bits 14/11/10 hardwired 0).
- **Delegation routing — LIVE.** `trap_deleg = trap_vld && (pm_r !=
  PRIV_M) && (trap_int ? mideleg_reg[{1'b0, trap_vec}] :
  (trap_vec <= 15) && medeleg_reg[...])` at :524-529 — the interrupt arm
  of the delegation mux exists today and is dark only because no
  interrupt ever asserts `rtu_yy_xx_expt_int`.
- **mstatus trap-entry arms — ALL EXIST and are delegation-aware (they
  fire today on sync traps):**
  - MIE/MPIE: trap_to_m pushes MPIE←MIE, MIE←0; mret pops MIE←MPIE,
    MPIE←1 (`$W/rtl/CSR.v:569-583`).
  - SIE/SPIE: trap_to_s pushes SPIE←SIE, SIE←0; sret pops (:586-600).
  - MPP: captures `pm_r` on trap_to_m, mret sets U, WARL 10→00 on write
    (:604-613). SPP: captures `pm_r[0]` on trap_to_s, sret clears
    (:615-624).
  - Reset values: MPP=PRIV_M, SPP=1, others 0 (:543-545, :616-617).
- **mcause bit 63 — capture path EXISTS, fed a permanently-0 signal.**
  `reg m_intr` at :786; on `trap_vld && !trap_deleg`:
  `m_intr <= rtu_yy_xx_expt_int; m_vector <= rtu_yy_xx_expt_vec`
  (:794-796); readback `mcause_value = {m_intr, 58'b0, m_vector}` (:803).
  scause identical for delegated traps (:806-823). There is no "OR in
  bit 63" — it latches `rtu_yy_xx_expt_int` directly, which is
  `retire_trap_int` in RTU.v and is structurally 0 today (see §2). So
  the bit-63 plumbing is ready; only the producer is dead.
- **What CSR.v exposes to RTU for interrupt entry:**
  - `cp0_rtu_trap_pc` (:139, assigned :728):
    `(pm_r == PRIV_M) ? mtvec_pc : stvec_pc` — the M/S tvec mux on
    POST-trap pm already exists; the donor timing subtlety (pm updates
    on the expt_vld cycle, RTU samples one cycle later) is documented
    at :689-697 and :725-728.
  - **mtvec/stvec are DIRECT MODE ONLY**: `mtvec_pc = {mtvec_base,
    2'b00}` with comment "mode forced 0 (direct)" (:717-718); bits
    [1:0] of a written tvec are discarded (`mtvec_base <=
    csr_wdata[PC_WIDTH-1:2]`, :707). Vectored mode is absent (M4
    out-of-scope row,
    `$W/docs/superpowers/specs/2026-08-31-m4-priv-mmu-pmp-design.md:90`).
  - `cp0_rtu_ex1_expt_int = 1'b0` — hardwired (:1474).
    `cp0_rtu_ex1_expt_vld/_vec` (:1472-1479) cover only sync causes
    (fetch fault, illegal, ecall, ebreak, CSR-access-illegal,
    xret-illegal).
  - **Absent: any masked-interrupt claim output** (no `mip & mie`
    computation, no per-priv target selection, no `cp0_rtu_int_vld`-
    shaped port at all).

## 2. RTU interrupt path (rtl/RTU.v)

**State: the entire interrupt machinery is structurally present and
provably dead — `int_vld_raw` is an internal literal 15'd0. There is NO
interrupt-claim input port. The architectural slot (retire-boundary
interrupt with epc=next-PC, FE/BE flush, tvec redirect) is already fully
built into the FSM.**

- **"DEAD" means exactly this:** `wire [14:0] int_vld_raw = 15'd0;` at
  `$W/rtl/RTU.v:744`. The header's own note (:149-157):
  "`cp0_rtu_int_vld`-shaped interrupt input: DELIBERATELY not added...
  the interrupt-cause priority encoder below (a faithful clone of
  `aq_rtu_int.v`'s casez table) is fed an internal, permanently-0
  vector -- structurally present, provably never fires... A future
  milestone wiring real interrupts adds the port and threads it in."
- **The priority encoder exists**: casez table at :747-766, donor
  priority order (bit14 mcip=16 > bit13 mhip=18 > bit12 meip=11 > bit11
  msip=3 > bit10 mtip=7 > bit9 seip=9 > bit8 ssip=1 > bit7 stip=5 > bits
  5-0 custom moip/mcip/mhip). `retire_int_inst = (|int_vld_raw) &&
  !dtu_int_mask_tied0 && !int_ex2_split_tied0` (:771) — always 0.
- **The retire-time priority chain** at :793-805, in order:
  `retire_pending_bkpt_expt` (tied 0, :788, "leg 1: no DTU") >
  `retire_int_inst` (leg 2, dead) > `retire_async_expt =
  lsu_rtu_async_expt_vld` (leg 3, REAL and wired — LSU async bus error)
  > `retire_bkpt_expt` (tied 0, :791) > `ex2_expt_vec` (leg 5, the sync
  CP0/LSU exception).
- **Which wire would carry a pending-interrupt assert, and from whom:**
  a new RTU input port (e.g. `cp0_rtu_int_vld`/15-bit vector) fed by
  CSR.v — CSR.v would compute `mip & mie` + priv/deleg targeting. Today
  neither end exists; the only interrupt-adjacent live wire is
  `rtu_yy_xx_expt_int` (RTU output, :1043) = `retire_trap_int` (:852) =
  0.
- **Trap classification**: `retire_trap_vld = ex2_retire_vld &&
  (retire_expt_inst || retire_int_inst)` (:851) — **an interrupt can
  only be recognized on a cycle where the retire heartbeat is high**,
  i.e. RV906 takes interrupts strictly between instructions, at the
  commit/retire boundary (including mid-FLUSH_WAIT drain — it must wait
  for a retire). **That slot already exists as a redirect**: the
  interrupt leg is present in `retire_inst_flush_fe_set =
  ex2_retire_vld && (retire_expt_inst || retire_int_inst ||
  ex2_inst_chgflw)` (:884-885), in `retire_trap_epc =
  retire_sync_expt ? ex2_cur_pc : ex2_next_pc` (:847 — for an async
  interrupt epc = the retiring instruction's NEXT PC, the correct
  boundary), and in the changeflow mux `retire_chgflw_pc =
  retire_trap_chgflw_vld ? cp0_rtu_trap_pc : ex2_next_pc` (:951).
- **Flush FSM behavior** (:863-919):
  `FLUSH_IDLE → FLUSH_FE → (FLUSH_WAIT until cpu_no_op drain) →
  FLUSH_BE → IDLE`. Trigger: `retire_flush_fe_set =
  retire_inst_flush_fe_set || retire_bju_flush_req` (:890). For a sync
  trap: trap declared on the retire cycle → FE next cycle (front-end
  kill) → WAIT until pipe drained (`cpu_no_op = !ex2_retire_vld &&
  !wb0 && !wb1`, :899-901) → BE (back-end kill, `rtu_yy_xx_flush`) →
  IDLE. **An interrupt needs exactly this same sequence** — nothing new
  required in the FSM; only `retire_int_inst` must become live.
  Trap-to-CSR capture (`rtu_yy_xx_expt_vld` :1042) fires on the trap
  cycle; redirect happens later via `retire_trap_chgflw_vld` held until
  `retire_flush_be` (:934-938, :956-957).
- LSU acks: `rtu_lsu_expt_ack = retire_trap_chgflw_vld &&
  retire_flush_be` (:1068) — the same ack would cover interrupt entry.

## 3. SoC peripherals / address map (rtl/RVProcAXI.v)

**State: CLINT and PLIC are fully instantiated RTL AXI4-Lite slaves
behind width adapters, live on the crossbar; the UART is an EXTERNAL
full-AXI port modeled in C++; the tohost aperture is an uncached MMIO
hole routed to DRAM. No BOOTROM exists.**

Address map constants at `$W/rtl/RVProcAXI.v:145-158` (crossbar slaves
:737-807, `DEFAULT_SLAVE = SI_MEM` :747 — unmatched addresses,
including tohost 0x7FFF_F000, route to the memory controller, confirmed
at `$W/rtl/RVProc.v:1107-1113`):

| Block | Range | Base/Mask lines | Protocol | Module |
|---|---|---|---|---|
| MEM (DRAM) | 0x80000000, upper 2GB (mask 0x80000000) | :145-146, SI_MEM=0 | Full AXI4, 512-bit, via `MEMCTL_AXI4L_step` (inst :896-966) | `$W/rtl/MEMCTL_AXI4L_step.RTL.v` |
| CLINT | 0x02000000, 64KB (mask 0xFFFF0000) | :147-148, SI_CLINT=1 | AXI4-Lite 64-bit behind `AXIWidthAdapter` 512→64 (inst :513-566), instance :629-655 | `$W/rtl/CLINT.v` |
| PLIC | 0x0C000000, 16MB (mask 0xFF000000) | :149-150, SI_PLIC=2 | AXI4-Lite 64-bit behind width adapter (:571-624), instance :660-687 | `$W/rtl/PLIC.v` |
| UART | 0x10000000, 64KB (mask 0xFFFF0000) | :151-152, SI_UART=3 | **External full-AXI4 512-bit port** (`G_axi_bus_s_ch_2_*`, wiring :689-732) — no RTL UART; modeled in C++ at **0x10001000** | `$W/device/uart16550.cpp` |

- **CLINT registers** (`$W/rtl/CLINT.v`, map comment :9-14): msip[0]
  @0x0000 (1 bit, :127,:185-191), mtimecmp[0] @0x4000 (64-bit
  byte-wise write, :128,:169-182, reset 0xFFFF_FFFF_FFFF_FFFF :171),
  mtime @0xBFF8 (64-bit, readable AND software-writable :129,
  :150-166). **mtime is free-running, incrementing on `rtc_tick`**
  (:163-165). Outputs `mtip = (mtime >= mtimecmp)`, `msip = msip_reg`
  (:196-197). Single hart only.
- **PLIC version**: a simplified "PLIC Spec 1.0.0" (`$W/rtl/PLIC.v:25`),
  **8 sources, ONE context (M-mode only), 3-bit priorities** (:28-33,
  :73). Register file: prio 1-7 @0x004-0x01C (:185), pending @0x001000
  (:186), enable ctx0 @0x002000 (:187), threshold @0x200000 (:188),
  **claim/complete @0x200004 — real, with claim-clears-pending and
  complete-releases logic** (:189, :244, :250, :272-297).
  `meip = (max_prio > threshold) && (max_id != 0)` (:179). **`int_src`
  is tied `8'b0` at instantiation** (`$W/rtl/RVProcAXI.v:668`) — no
  device sources. **No S-context exists** — Linux-in-S would need a
  second context or M-mode forwarding.
- **UART**: 16550 (not PL011) — C++ `UART16550_AXI4L` at
  `$W/device/uart16550.h:9`, registers RBR/THR/IER/IIR/FCR/LCR/MCR/LSR/
  MSR/SCR/DLL/DLM (:10-23). Base 0x10001000 with reg-shift=2
  (`$W/RVProcTest.cpp:25-26`, :30; FDT :345-351). **It has an
  interrupt output** — `fsmUser()` sets `nxt_intrFlag` on THRE or
  RX-ready (`$W/device/uart16550.cpp:98-104`), surfaced as
  `axi->intr` (`$W/io/RVProc_io.h:447,:675`) — **but it is dropped**:
  `BUS::connectInterrupts()` hardcodes `m_ch[0].intr = 0`
  (`$W/io/RVProc_io.h:737-739`), nothing reads `s_ch[DI_UART].intr`,
  and the DUT↔C++ ch_2 macro interface (`$W/dut.cpp:131-180`) carries
  no intr line.
- **tohost aperture**: `ADDR_TOHOST = 0x7FFF_F000`
  (`$W/rtl/rvproc_pkg.sv:35`, rationale :26-34 — deliberately inside the
  uncached <0x80000000 aperture so a D$ line can never swallow it). It
  matches no crossbar slave, so it defaults to SI_MEM (RVProc.v:1107-
  1113). **How the testbench observes tohost**: the C++ harness
  resolves the ELF symbol `tohost` (`$W/testbench/TestBench.cpp:190-
  197`), then polls `read_mem(tohost)` every step (:256-266); value
  semantics at :277-321: high word 0x01010000 → print one char
  (`printf("%c", (char)out)` :277-278); odd → test result (break :285);
  else syscall pointer (write=64 handled :294-308); after each event it
  writes `tohost=0`, `fromhost=1` (:322-323).
- **BOOTROM: none. Reset PC = 0x80000000**: `RESET_VECTOR =
  64'h80000000` parameter (`$W/rtl/RVProc.v:37`, passed at
  `$W/rtl/RVProcAXI.v:817`), exported as `cp0_xx_mrvbr`
  (`$W/rtl/CSR.v:1516`). **But the harness overrides it**:
  `DUT::init()` pokes `rootp->CPU_PC = pc` after reset release
  (`$W/dut.cpp:59`), where `pc = initial_pc` = **ELF `e_entry`** (set
  by load_elf, `$W/testbench/load_elf.cpp:196`; wired at
  `$W/RVProcTest.cpp:358`). So the effective boot PC is the first ELF's
  entry point (riscv-tests link at 0x80000000 anyway). There is no
  reset-vector fetch of a boot ROM; TestMaster.v no longer exists
  (retired; noted at `$W/rtl/RVProcAXI.v:9-12`).

## 4. Interrupt wiring TODAY

**State: the CLINT/PLIC → RVProc wiring is COMPLETE at the SoC level and
terminates in CSR.v's mip READBACK ONLY. Nothing downstream of mip
exists. PLIC has no sources; the UART IRQ is computed and dropped in
C++.**

Live chain (verified end-to-end):
- CLINT `mtip/msip` outputs → wires `clint_mtip/clint_msip`
  (`$W/rtl/RVProcAXI.v:365-366`, instance pins :636-637) → `RVProc`
  inputs `.mtip/.msip/.meip` (:886-888) → RVProc ports
  (`$W/rtl/RVProc.v:131-133`) → CSR instance `.mtip/.msip/.meip`
  (:1467-1469) → CSR.v pins (:271-273) → **`mip_value` readback
  only** (:935-937). Comment at RVProc.v:141-142: "terminate in CSR.v's
  mip wiring (contract 7)."
- PLIC `meip` output (:669) → `plic_meip` (:367) → same path.
- **Absent**: any connection from mip/mie into an interrupt-claim
  decision, any RTU interrupt input (§2), any WFI wake input (§6). mie
  is never ANDed with anything.
- **Unconnected peripheral interrupt ports**: PLIC `int_src` tied
  `8'b0` (RVProcAXI.v:668); UART IRQ dropped in C++ (§3); `MEMCTL`'s
  `G_axi_intr` left unconnected (`$W/rtl/RVProcAXI.v:941`).
- Grep for `msip/mtip/meip/mtime/mtimecmp` across rtl/ confirms:
  `mtime/mtimecmp` exist only inside `$W/rtl/CLINT.v`; `mtip/msip/meip`
  appear only in CLINT.v, PLIC.v, RVProcAXI.v, RVProc.v, CSR.v as
  above. No other consumers.

## 5. Testbench console / loader

**State: console observation is fully working via two independent
channels (tohost char protocol and the C++ 16550 → stdout); multi-ELF
and raw-binary loading exist; DTB is auto-generated, not loadable from
file.**

- **Console (tohost channel)**: high-word 0x01010000 prints a char
  (`$W/testbench/TestBench.cpp:277-278`). This is how riscv-tests
  v-env programs print.
- **Console (UART channel)**: the C++ 16550 model's THR writes go
  through `ttysrv::out()` → `write(ofd, &ch, 1)` with ofd=stdout
  (`$W/device/ttysrv.cpp:108-110`), driven per-step from
  `$W/RVProcTest.cpp:530-531` (`uart_cvt.fsm` + `axi_uart.update`).
  `--alloc-terminal` spawns a `utils/runttysrv` unix-socket terminal
  (ttysrv.cpp:16-40); `--expect <file>` arms expected-output matching
  (RVProcTest.cpp:400-401; ttysrv.cpp:108-125). TX capture therefore
  exists and is how Linux console output would be observed.
- **`--print-result`** (`$W/testbench/TestBench.cpp:91-92`, :343-360):
  after the run, prints `"<elf>: PASS."` when the final tohost value ==
  1, `"<elf>: FAIL. test no. = N"` otherwise (N = result>>1); with
  CONFIG_Zicsr it first checks `quitted` (which is tied 0 in RTL —
  `$W/rtl/RVProc.v:1525` — so the sim ONLY exits via tohost; there is
  no cycle timeout, only a 1M-cycle progress printout :242-245).
- **Loader** (`$W/testbench/TestBench.cpp:186-217`): **multiple ELFs
  supported** — `elf[0]` loaded with symbol table (tohost/fromhost/
  begin_signature/end_signature/_stack_top, :190-197) and sets
  `initial_pc`; `elf[i>=1]` loaded without symbols (:208-212). **Raw
  binaries**: `--kernel <file>` → `load_bin` at `kernel_addr` =
  0x80200000 for RV64I (:30-34, :213-214); `--initrd <file>` at
  `initrd_addr` = 0x84000000 (RVProcTest.cpp:275, :215-216). **DTB is
  auto-built** by `TB::build_fdt` (RVProcTest.cpp:279-355), packed and
  written to `dtb_addr` = 0x87000000 (RVProcTest.cpp:274;
  TestBench.cpp:132-183) — **no CLI option to supply an external DTB**.
  `--mem_size` (default 1GB, `$W/testbench/TestBench.h:19`),
  `--bootargs` (default "console=ttyS0 earlycon", TestBench.cpp:55),
  `--signature` (riscv-arch-test signature dump :454-479). ELF loading
  is section-based via `write_byte` into ExtMem pages; arbitrary
  (addr,data) pairs are not a CLI, but `TestBench::write_mem/
  write_byte` are public for harness extension. **`initial_pc` is only
  set by load_elf** — a kernel-only invocation (no ELF) leaves it
  indeterminate (TestBench.h:11 has no initializer).
- **Boot-register setup** (`$W/dut.cpp:54-64`): sets `CPU_PC = pc`,
  `gpr[2] = sp`, `gpr[11] = dtb` (a1). **a0 (hartid) is NOT set** — a
  Linux entry point expecting `a0=mhartid, a1=dtb` gets garbage in a0.

## 6. WFI (D-M4-7)

**State: confirmed — WFI is a plain single-cycle completing no-op with
NO flush and NO wait (the M4 doc calls it a "flushing no-op"; the RTL is
actually even weaker: it doesn't even flush).**

- Decode: SYSTEM funct7=0001000 split by rs2 — rs2=5 →
  `CP0_FUNC_WFI` (`$W/rtl/IDU.v:846-853`; `CP0_FUNC_WFI = 20'h00102`
  at `$W/rtl/rvproc_pkg.sv:493`).
- Legality: `wfi_priv_illegal = is_wfi && (pm_r != PRIV_M) && tw_f`
  (`$W/rtl/CSR.v:419`) — TW trap works.
- Completion: `is_wfi` (:318) appears in NO other logic — it is not in
  `cp0_rtu_ex1_chgflw` (:769-770: only mret/sret/fence.i), not in
  `fence_hold`/`sfence_hold`. It completes via the generic
  `cp0_rtu_ex1_cmplt_dp = ex1_active && !fence_hold && !sfence_hold`
  (:1429) — retires one cycle in EX1 like a NOP.
- **What a real WFI needs**: (a) an EX1 hold state (hold `cmplt_dp`
  low while waiting, the fence_hold pattern), (b) a wake input = "any
  pending-and-enabled interrupt for the current priv" from the M6
  claim logic, (c) flush on wake (the `retire_inst_flush_fe_set` leg or
  a chgflw), and (d) architectural resolution of whether the WFI itself
  retires before or on the wake cycle. rv64si-p-wfi passes today on the
  no-op (per
  `$W/docs/superpowers/specs/2026-08-31-m4-priv-mmu-pmp-design.md:89`).

## 7. Clocks / resets

**State: strictly single clock `clk`; no CPU:AXI ratio parameter
anywhere. mtime ticks at clk/100 via a hardcoded divider.**

- One `clk`/`rst_n` pair on RVProcAXI (`$W/rtl/RVProcAXI.v:71-72`);
  every instance (core, crossbar, CLINT, PLIC, MEMCTL, adapters) uses
  the same `clk`. The C++ harness toggles one clock
  (`$W/dut.cpp:79-91`).
- RTC divider: 7-bit counter, tick every 100 cycles — "divide clk by
  100 for ~1MHz RTC from 100MHz system clock"
  (`$W/rtl/RVProcAXI.v:369-385`).
- **Inconsistency worth flagging**: the FDT advertises
  `timebase-frequency = 1250000` (`$W/RVProcTest.cpp:287`) while the
  RTL divider is /100 — these two numbers must be reconciled at M6
  (Linux's clocksource scaling uses the DT value).
- Reset: async active-low `rst_n` throughout; DUT reset sequence at
  `$W/dut.cpp:41-52`.

## 8. Existing M4/M5 test infra reusable for M6

- **test/m4/** (`$W/test/m4/Makefile`): four SPELLLED-OUT lists
  (SI_TESTS 7 names :41-49, MI_TESTS 17 :57-75, V_TESTS 86 :81-120,
  MMU_TESTS 6 directed :126) under a strict no-`$(wildcard)`
  prohibition; builds ELFs from the in-place riscv-tests checkout
  (`RISCV_TESTS ?= .../riscv-tests`, :50) against `test/m2/env` p-env
  and `test/m4/env/v` v-env override; runners `run_directed.sh`
  (si+mi+mmu loop, grep PASS) and `run_vsample.sh` (4-test v sample).
  Includes `rv64si-p-wfi` in the si list (build dir contains
  `rv64si-p-wfi.elf`).
- **test/m5/** (`$W/test/m5/Makefile` + `run_all.sh`): 46-ELF FP runner
  — `timeout 120 bin/verisim/testbench --print-result <elf> | grep -q
  PASS` loop over `build/rv64uf-*.elf`/`rv64ud-*.elf`.
- **test/m2/run_all.sh**: the canonical sweep pattern (`cd` repo root,
  glob build dir, per-ELF timeout+grep PASS, exit=failcount).
- **/tmp/run_atomic.sh**: exists; same pattern over
  `test/m2/build/rv64ua-p-*.elf` with `timeout 60`.
- **Where test/m6 would live**: sibling `test/m6/` mirroring the m4/m5
  shape (explicit list Makefile + run_all.sh). Note the runner binary
  is `bin/verisim/testbench` and the verisim build compiles ALL of
  `rtl/*.v` by wildcard (`$W/verilator.mk:3-4`) — so CLINT.v/PLIC.v are
  already in the build.
- **A ready-made M0-era bare-metal interrupt suite already exists at
  `$W/test/`**: `test/Makefile:15-24` builds `clint_test.out`,
  `plic_test.out`, `intr_test.out`, `hello.out` from `entry.S` (WITH_
  TRAP mode, own `trap_vector` at `$W/test/entry.S:125-135`), `uart.c`
  (16550 driver at 0x10001000, `$W/test/uart.c:5-19`), `syscalls.c`.
  `test/intr_test.c` is a full MSIP+MTIP delivery test (enables
  mie.MTIP/MSIP + mstatus.MIE, triggers CLINT msip/mtimecmp, expects
  trap handler hits with `mcause == 0x8000...0003/0007`, :51-60,
  :100-148) — **it cannot pass today** (no delivery) and is the natural
  M6 acceptance seed. **Warning**: these use `test/link.ld`, which
  places data (including `tohost`, `$W/test/entry.S:210`) at
  0x90000000 — inside the CACHEABLE DRAM PMA (`$W/rtl/MMU.v:200-201`)
  — so with DCache on, a tohost store can park in the D$ and hang the
  harness (the exact hazard the M2 relocation to 0x7FFF_F000 fixed for
  the riscv-tests env; see `$W/rtl/rvproc_pkg.sv:26-34`). The linker
  script (or tohost placement) must be fixed before these tests are
  used.

## 9. mcounteren/scounteren + cycle/time/instret

**State: counteren registers are STORAGE-ONLY (R/W, readback) — the
documented "counteren chain" gating is NOT implemented. cycle/instret
read live counters from any privilege ungated; time reads 0 (absent
from the read mux).**

- Storage: `mcounteren_reg`/`scounteren_reg` 32-bit R/W at
  `$W/rtl/CSR.v:1015-1032`; readback :1328/:1337. **They are consumed
  nowhere else** (grep confirms lines 1015-1032, 1328, 1337 are the
  only references) — no gating of any read, no illegal-trap on a
  blocked access.
- `csr_read_mux`: `CSR_CYCLE → mcycle_reg`, `CSR_INSTRET →
  minstret_reg` **unconditionally** (:1344-1347) — the comment at
  :1011-1013 claims donor-style gating ("M always allowed; S gated by
  mcounteren; U by mcounteren & scounteren") but no such logic exists
  in the mux or in `csr_access_illegal` (:1447-1462, which checks only
  priv bits, RO-writes, TVM, FS).
- **time (0xC01)**: `CSR_TIME` constant exists
  (`$W/rtl/rvproc_pkg.sv:315`, "storage deferred to M6 (D-M4-9)") but
  there is **no `CSR_TIME` arm in the read mux** — `csrr time` returns
  0 from any mode. A write traps illegal (addr[11:10]=11, :1452).
- Counters themselves: `mcycle_reg` free-running every cycle
  (:977-984), `minstret_reg` auto-increment on
  `rtu_cp0_inst_retire` with write-precedence one-shot (:989-1006).
- **Does the kernel-boot path need live time?** Yes — Linux's
  clocksource/timer (`riscv_time_init`) reads `time` (or SBI time); a
  constant-0 `time` breaks scheduling immediately. It must be sourced
  from the CLINT mtime (either a live mux of the CLINT's mtime register
  into the read path, or a shadow counter ticked by the same
  rtc_tick).

## 10. misa state

**State: `misa = 64'h8000_0000_0000_112C` — MXL=64, extensions
{C(2), D(3), F(5), I(8), M(12)} only. S(18), U(20), A(0) are all 0.
This contradicts both the implemented hardware (M/S/U modes, Sv39,
AMO/LR/SC all live since M3/M4) and the DT.**

- Definition: `wire [63:0] misa_value = 64'h8000_0000_0000_112C;` at
  `$W/rtl/CSR.v:955` (comment :944-954 documents F/D flipped at the
  M5 swap; I/M/C from M2).
- Decoded: bits set = {2,3,5,8,12} = C, D, F, I, M. **No A** (despite
  M3 atomics — `$W/HANDOFF.md:50` said "misa.A=0, M3" and it was never
  flipped), **no S** (bit 18), **no U** (bit 20).
- **DT inconsistency**: `TB::build_fdt` writes `riscv,isa =
  "rv64imac"` (`$W/RVProcTest.cpp:295`) — which advertises A but not
  F/D/S/U, i.e. mismatched with misa in the opposite direction. Modern
  Linux builds its hart feature set from the DT `riscv,isa` string
  (not misa) in `riscv_fill_hwcap`, and `mmu-type = "riscv,sv39"`
  (:296) is what gates paging — but misa.S/U=0 is a spec violation
  (the hart DOES implement S and U) and older kernels / some early
  paths do consult misa (e.g. `elf_hwcap` cross-checks, and M-mode
  kernels check `misa` for extension letters). For M6, both misa (→
  IMAFDC + S + U) and the FDT isa string must be updated and kept in
  lockstep, exactly as M5's Task-9 discipline did for F/D.

# GAPS FOR M6

## (a) Bare-metal interrupt tests — in rough dependency order

1. **Interrupt-claim computation in CSR.v** (the core missing piece):
   compute pending-and-enabled (`mip & mie`), target-priv selection
   (M-level vs S-level per mideleg + current pm), and global-enable
   (mstatus.MIE / SIE per target). Storage (mie/mip/mideleg/mstatus)
   is all live; nothing ANDs them today (`$W/rtl/CSR.v:896-941`,
   :569-600).
2. **New CSR.v → RTU interrupt port(s)** (e.g. `cp0_rtu_int_vld` +
   15-bit one-hot or vec) and the matching RTU.v input replacing
   `int_vld_raw = 15'd0` (`$W/rtl/RTU.v:744`; header :149-157
   documents the deliberate omission). The 15-bit vector must be built
   in donor aq_rtu_int.v priority order (msip/mtip/meip/seip/ssip/
   stip + T-Head customs) to feed the existing casez encoder
   (:747-766).
3. **Wire CSR.v's claim through RVProc.v** (CSR and RTU are both
   instantiated there — `$W/rtl/RVProc.v:1376-1470` CSR, RTU instance
   above it; one new wire).
4. **mcause bit-63 + delegation entry**: already live/dark — verify,
   don't rebuild (m_intr capture `$W/rtl/CSR.v:794-796`; trap_deleg
   :527-529; SIE/SPIE arms :586-600; stvec trap-pc mux :728). Needs
   only an interrupt to actually fire.
5. **WFI wake** (D-M4-7): hold-in-EX1 state + wake-on-claim input +
   flush-on-wake (§6; `CP0_FUNC_WFI` decode is live).
6. **time CSR (0xC01)** (D-M4-9): live mtime read (CLINT mtime via a
   side-band input to CSR.v, or a tick-synchronized shadow). Today
   returns 0.
7. **PLIC sources**: `int_src` tied `8'b0` (`$W/rtl/RVProcAXI.v:668`)
   — need at least one real source for an external-IRQ test. The UART
   IRQ exists in C++ (`$W/device/uart16550.cpp:98-104`) but has no
   path into the RTL: needs (i) a new top-level interrupt input port
   on RVProcAXI, (ii) dut.cpp plumbing (the ch_2 macro set
   `$W/dut.cpp:131-180` carries no intr line), (iii) TB-side read of
   `axi_bus.s_ch[DI_UART].intr` (currently zeroed by
   `BUS::connectInterrupts`, `$W/io/RVProc_io.h:737-739`).
8. **PLIC S-context** if S-mode external interrupts are in scope
   (PLIC.v has exactly one M-mode context: `$W/rtl/PLIC.v:129-136`);
   the FDT already advertises an SEIP wire
   (`$W/RVProcTest.cpp:340-341`) that the RTL does not implement.
9. **Vectored mtvec/stvec** (optional): bits [1:0] discarded today
   (`$W/rtl/CSR.v:717-718`); M4 deferred the decision (design doc :90).
10. **counteren enforcement** (optional for bare-metal, required for
    spec compliance): storage-only today (§9).
11. **Test scaffolding**: fix `test/link.ld` tohost placement
    (cacheable-DRAM hang hazard, §8) or port `test/intr_test.c`/
    `clint_test.c`/`plic_test.c` into a new `test/m6/` with the
    uncached-tohost treatment (`test/m2/env/rv906.ld` pattern); the
    three C tests already encode the intended MSIP/MTIP/claim-complete
    behaviors.

## (b) Single-hart Linux boot — in rough dependency order

1. **Everything in (a) items 1-6** (interrupt entry, claim, WFI,
   time) — Linux cannot schedule without timer interrupts and a live
   `time`.
2. **misa fix**: advertise S, U, A (+ keep I M A F D C) at
   `$W/rtl/CSR.v:955`, in lockstep with the DT.
3. **FDT `riscv,isa` fix**: "rv64imac" → the real string (e.g.
   "rv64imafdc" + s/u as the kernel version requires),
   `$W/RVProcTest.cpp:295`; keep `mmu-type=sv39` (:296).
4. **Boot-protocol registers**: `dut.cpp` sets a1=dtb but not
   a0=hartid (`$W/dut.cpp:59-61`); Linux (and any SBI) entry expects
   a0=mhartid, a1=dtb pointer. Also decide M-mode-Linux (CONFIG_
   RISCV_M_MODE, direct CLINT/mtimecmp — no firmware needed) vs S-mode
   + OpenSBI (requires SBI ecall handling, an SBI firmware ELF, and
   PLIC S-context). No SBI/firmware/kernel assets exist in the tree
   today.
5. **Firmware/kernel loading**: `--kernel` loads a raw bin at fixed
   0x80200000 (`$W/testbench/TestBench.cpp:30-34,:213-214`);
   initial_pc comes only from elf[0] — for a firmware(ELF)+kernel(bin)
   split this works, but there is no way to pass an entry address for a
   bin-only boot, no external-DTB option (DTB is auto-generated at
   0x87000000), and mem_size/DTB contents (ndev, isa, timebase) are
   hardcoded in RVProcTest.cpp.
6. **timebase reconciliation**: FDT says 1250000
   (`$W/RVProcTest.cpp:287`), RTL divider is clk/100
   (`$W/rtl/RVProcAXI.v:369-385`) — pick one story.
7. **Console**: already workable — 16550 C++ model prints to stdout
   (`$W/device/ttysrv.cpp:108-110`), FDT has uart@10001000 + chosen
   stdout-path + "console=ttyS0 earlycon"
   (`$W/RVProcTest.cpp:345-354`, `$W/testbench/TestBench.cpp:55,:145`);
   `--expect` gives a boot-marker gate. UART IRQ (item 7 in (a))
   needed only for interrupt-driven console.
8. **Run-loop exit**: `quitted` is tied 0
   (`$W/rtl/RVProc.v:1525`) — Linux never writes tohost, so the
   harness runs forever; M6 needs an exit criterion (e.g. `--expect`
   marker forcing shutdown, a guest tohost shutdown write, or a
   harness-side cycle cap — none exists today; the only cap is per-
   test `timeout` in the shell runners).
9. **PLIC in S-mode** (if S-mode Linux): S-context + claim/complete
   at context 1, plus SEIP delivery (FDT already promises it).
10. **Long-run stability items to re-verify at M6**: uncached PMA for
    CLINT/PLIC/UART regions (`$W/rtl/MMU.v:182-188` — note the
    implemented `sysmap_attr` collapses to "DRAM cacheable /
    everything else uncached", :196-205, which covers all three), and
    the DCache/uncached store path for MMIO writes (proven for tohost
    at 0x7FFF_F000; CLINT mtimecmp writes take the same path).

**Key "already built, don't rebuild" list** (dark but live): the whole
RTU interrupt leg (encoder, priority slot, flush-FSM trigger,
epc=next-PC, tvec redirect mux), mcause bit-63 capture, mstatus
MIE/SIE/MPIE/SPIE/MPP/SPP trap arms, mideleg routing + stvec/sepc/
scause S-side capture, CLINT/PLIC RTL + AXI paths + SoC wiring into
CSR.v's mip, the 16550 console, multi-ELF/bin/DTB loader, and the
m4/m5 test-suite pattern.
