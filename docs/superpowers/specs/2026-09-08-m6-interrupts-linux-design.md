# M6: Interrupt path (CLINT/PLIC/CSR) + single-hart Linux boot — design doc

## Status: Exploration complete (2026-09-08, 3 agents); decisions settled; ready for execution

## Context

M4 (privilege M/S/U + Sv39 MMU + PMP + counter CSRs) and M5 (scalar FPU F/D)
are complete: M5 closed at `d036f22` with 46/46 FP compliance ELFs plus the
full M2/M5 battery (unit suite, 86/87 sweep, 19/19 atomics — see
`docs/08-verification.md` §8.15/§8.16).

M6 is the next milestone per the design doc
(`docs/superpowers/specs/2026-08-20-rv906-c906-clone-design.md` §7.4 row M6):
*"interrupt path (CLINT/PLIC/CSR) integration"*, acceptance
*"bare-metal interrupt tests; single-hart Linux boot". The standing
directive is "do as C906 does" (gate-level faithful clone, donor RTL is the
spec, cite `file:line` for every clone-fidelity decision).

M4 left two explicit M6 obligations in its deviation ledger:
- **D-M4-7:** WFI is a flushing no-op — M6 makes it real (wake on
  pending-and-enabled interrupt).
- **D-M4-9:** the `time` CSR (0xC01) was deferred to M6/CLINT — M6 must
  source it from the CLINT mtime.

## What M6 builds (settled)

1. **Interrupt claim + delivery** (CSR.v/RTU.v/RVProc.v): the donor's
   15-term claim (per-source `mie&mip`, per-priv gating, nodeleg/deleg
   split), registered export to the RTU, which lights the already-built
   donor priority encoder (RTU.v:747-766) and retire-time chain.
   Vectored tvec arm added (interrupts only, donor :1346-1359).
2. **`time` CSR (0xC01)** — read-only live mirror of the CLINT mtime.
3. **Real WFI** (discharges D-M4-7): stall until `(mip & mie) != 0`
   (donor wake condition), retire on wake; TW trap arm kept.
4. **PLIC liveness**: the C++ UART's IRQ output → new top port →
   PLIC `int_src[7]` (UART = PLIC source 7, the top of the 3-bit space).
5. **misa + FDT isa in lockstep**: misa gains A/S/U (0x112C →
   0x14112D low word); FDT `riscv,isa` "rv64imac" →
   "rv64imafdc_zicsr_zifencei" (NO zbb — not implemented).
6. **Bare-metal directed interrupt suite** (`test/m6/`): msip, mtip,
   PLIC-UART, SSIP-delegation, STIP-via-SBI-model, priority, vectored
   tvec, wfi-wake (8 tests).
7. **Single-hart Linux boot**: prebuilt OpenSBI 1.3 fw_jump + Linux 6.5
   Image + 5 MB initrd (all on disk — path (a) of Agent 3) reaching the
   "Linux version" console banner through the 16550.

## Exploration status

- [x] Agent 1: rv906 current interrupt/SoC state → `notes/2026-09-08-m6-rv906-interrupt-soc-state.md`
- [x] Agent 2: donor C906 interrupt machinery → `notes/2026-09-08-m6-donor-interrupt-machinery.md`
- [x] Agent 3: Linux boot feasibility → `notes/2026-09-08-m6-linux-boot-feasibility.md`

### Agent 1 findings (verified facts, full note linked above)

- **RTU: the whole interrupt leg is built and dark.**
  `int_vld_raw = 15'd0` (RTU.v:744); the donor aq_rtu_int.v priority
  encoder exists (:747-766); the retire-time chain has the interrupt as
  dead leg 2 (:793-805); `retire_trap_epc` already gives epc=next-PC
  for interrupts (:847); the flush FSM handles the sequence unchanged.
  M6's job = make `retire_int_inst` live via a new CSR→RTU port.
- **CSR: all storage is live** (mie full R/W :905, mip hybrid
  mtip/msip/meip pins + ssip/stip/seip flops :906-940, mideleg mask
  {1,5,9} :503, mstatus MIE/SIE/MPIE/SPIE/MPP/SPP trap arms :569-624,
  trap_deleg interrupt arm :524-529, mcause bit-63 capture :786-803,
  tvec mux :728 direct-only). **Absent: the claim computation
  (mip&mie×priv×global-enable) and any export port.**
- **SoC: CLINT/PLIC are live RTL slaves wired to CSR mip readback
  only.** CLINT: msip@0x0, mtimecmp@0x4000, mtime@0xBFF8 free-running
  on rtc_tick (clk/100), mtip/msip outputs. PLIC: 8 sources, ONE
  M-mode context, real claim/complete@0x200004, `int_src` tied
  8'b0. UART: C++ 16550 at 0x10001000, has an IRQ output but
  `BUS::connectInterrupts` zeroes it.
- **Testbench: more capable than the M2-era story.** Multi-ELF load;
  `--kernel` raw bin @0x80200000; `--initrd` @0x84000000; auto-
  generated FDT @0x87000000 (a1=dtb set; **a0=hartid NOT set**);
  16550 TX → stdout; `--expect <file>` output matching; **no cycle
  timeout / no external-DTB option / no bin entry-address option**.
  `quitted` tied 0 — sim exits only on tohost.
- **Blockers found:** misa = 0x112C lacks S/U/A despite the hardware
  (CSR.v:955; FDT says "rv64imac" — both must flip in lockstep);
  `time` (0xC01) has no read-mux arm (reads 0); FDT timebase
  1250000 vs RTL /100 divider mismatch; M0-era `test/` interrupt
  suite (intr_test.c = MSIP+MTIP delivery) exists but its link.ld puts
  tohost at 0x90000000 inside the cacheable PMA (hang hazard).

### Agent 2 findings (donor C906; full note linked above)

- **Claim scheme to clone (aq_cp0_trap_csr.v:1269-1338):** per-source
  `*_en = mie_bit & mip_bit`; M-sources gate on `pm != M || MIE`;
  delegable S-sources split into nodeleg/deleg pairs (S-mode arm
  requires SIE; U-mode is "global always on"); 15-bit `int_sel` with
  nodeleg group (mcip,mhip,meip,msip,mtip,seip,ssip,stip,moip) then
  deleg group (mcip,mhip,seip,ssip,stip,moip). Priority in the RTU
  casez (already rv906's, RTU.v:747-766): every M-target source
  outranks every S-target source.
- **Claim insertion (aq_rtu_retire.v:470-471,585-589):**
  `retire_trap_vld = retire_vld && !halt && !dbg && (expt || int)`;
  retire priority pending-bkpt > int > async > sync; EPC = next PC for
  interrupts; **tval forced 0 on int** (aq_rtu_retire.v:541-542).
- **Vectored tvec is supported by the donor**: mode bit 0 stored,
  redirect = `intr && tvec[0] ? base + 4*cause : base` — interrupts
  only (aq_cp0_trap_csr.v:1346-1359). rv906's M4 kept tvec
  direct-only (bits [1:0] discarded).
- **WFI (aq_cp0_lpmd.v:107-155):** real halt — 3-state LPMD FSM
  stalls the cp0 pipe (wfi itself does not retire until wake),
  quiesces IFU/LSU/MMU, gates the core clock; **wake = (mip & mie)
  only — deliberately no MIE/SIE/priv gate** (spec-conformant), in the
  always-on clock domain. rv12 dropped the LPMD FSM (flushing no-op);
  rv906 M6 at least needs stall+wake without clock gating.
- **The factory ships a CLINT** (msip@0x0, mtimecmp@0x4000, **S
  banks** ssip@0xC000/stimecmp@0xD000; mtime is an *external*
  system counter sampled in; mtip = mtime >= mtimecmp) **and a PLIC**
  (256 sources, 5-bit prio with MSB = M/S bank flag, per-hart M and S
  contexts {threshold, claim/complete@+0x4}, S enable bank @+0x2080,
  meip/seip outputs). `time` (0xC01) = read-only mirror of the same
  live counter; **no 0xB04 mtime alias** (that's MHPMCNT4).
- **Factory ships NO Linux content** (no OpenSBI/DTB/kernel/bootrom;
  smart_run is a sim SoC with 40 PLIC sources incl. uart0 and a
  PLIC-IRQ smoke test `C906_plic_int_smoke.s` that is a ready-made
  directed test).
- **CRITICAL inheritance: rv12 already cloned this exact donor
  interrupt network** — full 15-term int_sel (CSR.v:2212-2290), same
  casez priority, registered ACTIVE-LOW `cp0_rtu_xx_int_b` +
  `cp0_rtu_xx_vec` (CSR.v:2300-2321), mideleg/mdeleg_vld recompute,
  and the same M0-ported CLINT.v/PLIC.v wired identically to rv906's
  (RVProcAXI.v:924-926 / RVProc.v:1939-1941). RTU leg:
  `rob_read0_int_vld = !cp0_rtu_xx_int_b && !tme && !rob_commit2`
  (rv12 RTU.v:2028). rv12's documented divergences are rv906's
  deviation ledger candidates: no S-CLINT pins (S bits = CSR flops
  only), WFI no-op, mtime inside CLINT (writable), single-context
  PLIC.

### Agent 3 findings (Linux boot feasibility; full note linked above)

- **Boot path (a) settled: OpenSBI 1.3 fw_jump + prebuilt Linux 6.5 —
  no new artifacts.** All assets on disk at `/home/vlsilab/zhouz/
  workspace/C2RTL/rvproc/RVProc6/vla/rvvla/test/llm_demo/prebuilt/
  linux/`: `fw_jump.elf` (1,157,328 B, entry **0x80000000 ==
  RESET_VECTOR**, `_jump_addr` **0x80200000 == TB kernel_addr**,
  fixed FDT address **0x82200000** verified by disassembly), `Image`
  (Linux 6.5, 13,683,200 B), `rootfs.cpio` (5,027,840 B — the only
  small rootfs). Identical images in the buildroot `output/images/`.
  Path (b) hand-rolled stub is infeasible (stock kernel is
  CONFIG_RISCV_SBI=y — a stub ECALL-dies); path (c) fw_dynamic offers
  zero single-hart benefit.
- **Harness deltas (small, enumerated — verified in-tree):**
  `dtb_addr` 0x87000000 → 0x82200000 (RVProcTest.cpp:274, one line —
  fw_jump passes the fixed 0x82200000 to the kernel AND uses it as its
  own DTB; the constant was verified by disassembling the shipped
  fw_jump.elf: `fw_next_arg1` = 0x411<<21, `_jump_addr` = 0x80200000,
  `fw_next_mode` = 1/S, entry = 0x80000000);
  `a0 = hartid` poke missing (dut.cpp:59-61 pokes only PC/SP/a1);
  FDT `riscv,isa` "rv64imac" → "rv64imafdc_zicsr_zifencei"
  (RVProcTest.cpp:295; kernel .config has SVPBMT/ZBB/ZICBOM — all
  hwcap-gated optimizations, safe to omit: alternatives skip, no
  trap); timebase 1250000 → 1000000 (:287, D-M6-4);
  **FDT has NO `/chosen` node at all (Agent 3 was wrong about
  stdout-path):** without it the 8250 console and earlycon never
  attach and the banner never reaches stdout — Task 5 adds
  `chosen { stdout-path="/uart@10001000"; bootargs="console=ttyS0
  earlycon"; }` (bare `earlycon` picks the SBI earlycon,
  CONFIG_SERIAL_EARLYCON_RISCV_SBI=y; `console=ttyS0` is the
  permanent console; the 8250 probe succeeds because LSR is
  hardcoded 0x60 = THRE|TEMT).
- **Boot flow:** fw_jump (M) → PMP all-memory RWX → mret to kernel
  (S) at 0x80200000 with a0=hartid, a1=0x82200000. Kernel: SBI timer
  (sbi_set_timer → OpenSBI sets mtimecmp → MTIP → M-mode OpenSBI sets
  sip.STIP (flop CSR) → delegated S trap), earlycon via SBI (bare
  `earlycon` from the new `/chosen` bootargs; 8250 probe succeeds
  because LSR is hardcoded 0x60; polled console needs no PLIC
  S-context).
- **OpenSBI's fdt_timer_mtimer** expects mtimecmp@base+0x4000,
  mtime@base+0xBFF8 — exactly rv906's CLINT map.
- **Exit criterion:** external `timeout N testbench ... | grep -q
  "Linux version"` (the existing run-script idiom, zero harness
  changes). Full boot to userspace is a stretch goal (hours in
  Verilator); the banner proves the entire M→S transition (PMP, satp,
  interrupts, timer, SBI, console) works.
- **Known risk:** the kernel's PLIC driver will set up an S-mode
  context (writes to +0x201004 — undecoded by rv906's single-context
  PLIC → silently ignored). Fallback if it misbehaves: drop the plic
  node from the FDT (OpenSBI/kernel survive without it single-hart).
- **No precedent in rv12** (its M6 never ran); the only working Linux
  boot on this infrastructure is the C2RTL RVProc6 flow in
  `rvvla/test/llm_demo/` — the artifact/loader reference.

## Task decomposition (final — each row is one subagent dispatch + gate)

| # | Task | Contents | Gate focus |
|---|---|---|---|
| 0 | M6 design doc + extraction notes | this doc + 3 `notes/` pinning every donor span | docs committed |
| 1 | CSR claim computation | 15-term `int_sel` per donor aq_cp0_trap_csr.v:1269-1338 (per-source `mie&mip`; M-gate `pm!=M||MIE`; S-trio nodeleg/deleg pairs with SIE arm, U always-on); registered 15-bit export + registered active-low (rv12 CSR.v:2300-2321 template; cite the donor's own export registration at the RTL site); vectored tvec arm (`intr && tvec[0] ? base+4*cause : base`, donor :1346-1359, stvec too) | csr_tb claim rows (all 9 sources × M/S/U × deleg) |
| 2 | RTU interrupt leg live + first e2e | new CSR→RTU input port replaces `int_vld_raw = 15'd0` (RTU.v:744); the existing casez (RTU.v:747-766) becomes the live priority/cause encoder; verify tval=0-for-int in the retire chain (donor aq_rtu_retire.v:541-542); epc=next-PC (:847) and flush FSM untouched; RVProc.v wiring; **first end-to-end: directed msip self-interrupt** (cause 3\|bit63) | msip e2e PASS + full battery (int_sel=0 at reset ⇒ bit-identical OFF path) |
| 3 | `time` CSR (0xC01) | read-mux arm ← CLINT mtime (new CSR.v input; RVProc.v wire from the existing `clint_mtime`); same mirror serves M/S/U `time`; counteren stays lenient (D-M6-5) | csr_tb time row; time-increases directed |
| 4 | Real WFI (D-M4-7) | wfi → flush (existing FSM) + pipe hold: no fetch/issue until wake; CSR wake condition `(mip & mie) != 0` in the always-on domain (donor aq_cp0_lpmd.v wake, NO MIE/SIE/priv gate); wfi retires as no-op on wake; TW=1 && pm<M trap arm kept | rv64si-p-wfi (wfi must NOT halt: SIE=0+SSIP pending); directed wfi-wake on MTIP |
| 5 | misa + FDT isa lockstep | misa low word 0x112C → 0x14111D (IMACFDSU; D-M6-6 — the 0x14112D first drafted here has a bit4/bit5 F/G transposition); FDT `riscv,isa` → "rv64imafdc_zicsr_zifencei" (RVProcTest.cpp:295; NO zbb — SVPBMT/ZBB/ZICBOM are hwcap-gated optimizations, safe to omit); FDT timebase-frequency → 1000000 (D-M6-4); **new `/chosen` node: `stdout-path="/uart@10001000"` + `bootargs="console=ttyS0 earlycon"`** (without it no console attaches — banner never reaches stdout); cross-check kernel `.config` `CONFIG_RISCV_ISA_*` vs implemented before boot | full battery unchanged (sweep 86/87, unit, atomics, si 7/7) |
| 6 | PLIC UART IRQ | C++ UART IRQ flag (uart16550.cpp:98-104) → new RVProcAXI top port → PLIC.v `int_src[7]` (UART = source 7; RVProcAXI.v:668 un-ties the bus); TB reads the model flag in `TB::step()` | directed PLIC-UART e2e (cause 11\|bit63, claim/complete @0x200004) |
| 7 | Bare-metal interrupt suite | `test/m6/` Makefile + 8 directed tests: (1) msip, (2) mtip mtimecmp tick + time check, (3) plic-uart, (4) ssip delegation (mideleg[1] → S trap cause 1\|bit63 via stvec), (5) stip SBI-model (M-handler sets stip_f + sret → S trap cause 5\|bit63), (6) priority (MEIP+MSIP+MTIP → cause 11 first), (7) vectored tvec (mtvec[0]=1, MSIP → epc=base+12), (8) wfi-wake; link.ld tohost → 0x7FFFF000 (uncached aperture) | 8/8 via run_all.sh |
| 8 | Linux boot | copy fw_jump.elf/Image/rootfs.cpio → `test/m6/linux/`; `dtb_addr` → 0x82200000; `a0=hartid` poke in dut.cpp; `run_linux.sh` = `timeout 21600 bin/verisim/testbench fw_jump.elf --kernel Image --initrd rootfs.cpio \| tee boot.log` + `grep -q "Linux version"`; verify FDT isa string ⊆ implemented ISA (Zbb omitted) | BOOT-PASS (banner); stretch: "Run /init" |
| 9 | Acceptance + close-out | full battery re-run (unit suite, 86/87, 19/19, 46/46 M5, si 7/7) + test/m6 8/8 + boot banner; docs: 08-verification §8.17, design doc → COMPLETE | all gates green |

**Sequencing notes:** Tasks 1→2 are strictly ordered (export then
consumer); 3/4/6 are independent of each other but land after 2 (shared
wiring/commit cadence); 5 before 8 (boot reads misa); 7 folds the
tests Tasks 2/3/4/6 already need as seeds; 8 is the long pole (hours
of sim — run in background early, iterate on failures in parallel).

## Deviation ledger

- **D-M6-1 — CLINT S banks absent.** rv906's CLINT (M0 port) has only
  the M bank (msip/mtimecmp/mtime). No ssip@0xC000 / stimecmp@0xD000;
  SSIP/STIP/SEIP are CSR flops only (M4). rv12 precedent. The SBI
  timer model (sbi_set_timer → mtimecmp → MTIP in M → OpenSBI sets
  sip.STIP → delegated S trap) makes S-CLINT unnecessary for boot.
- **D-M6-2 — WFI without LPMD clock gating.** Donor
  aq_cp0_lpmd.v:107-155 does 3-state LPMD + pipe stall + IFU/LSU/MMU
  quiesce + core clock gate. rv906: flush + fetch/issue hold + wake on
  `(mip & mie)` only. No clock gate, no quiesce handshake (the flush
  already drains the pipe; in-order single-issue needs nothing else).
- **D-M6-3 — PLIC: single M-mode context, 7 sources.** Donor: 256
  sources, 5-bit prio (MSB = M/S bank), M+S contexts. rv906 (M0 port):
  7 sources (3-bit), 1 M context. UART IRQ → source 7. No S-mode
  device IRQs — the SBI timer + polled 8250 console don't need them.
  Kernel PLIC-driver S-context setup writes undecoded offsets →
  silently ignored (known no-op; fallback: drop the plic node from
  the FDT).
- **D-M6-4 — timebase frequency.** FDT `timebase-frequency` set to
  1000000 to match the RTL `rtc_tick = clk/100` (nominal 100 MHz).
  Guest timer scaling is correct in sim-time; absolute wall rate is
  meaningless in Verilator.
- **D-M6-5 — counteren not enforced** (M4 carry-over):
  mcounteren/scounteren are storage-only; user cycle/time/instret
  reads always allowed. Lenient; boot-safe (a stricter machine could
  trap where rv906 won't — never the reverse).

- **D-M6-6 — misa target corrected (row 5 value is wrong).** Row 5's
  `misa → 0x14112D` (and the pre-M6 constant `0x112C`) both carry a
  bit4/bit5 transposition: they set bit5 (the G meta-extension) and
  omit bit4 (F). rv906 implements F (M5) and its own FDT string
  "rv64imafdc" requires it, so the correct value is **0x14111D**
  (IMACFDSU = I|M|A|F|D|C|S|U). Task 5 uses 0x14111D; this supersedes
  row 5's 0x14112D. The CSR.v comment's "F(bit5)" label was the source
  of the original error.

## Verification plan

**Two-sided acceptance** (M4 precedent):
1. **OFF path:** at reset no interrupt source is pending (mip pins
   0, flops 0) ⇒ `int_sel = 0` ⇒ `int_vld_raw = 0` — the pre-M6
   constant. The ENTIRE existing battery must stay bit-identical after
   Tasks 1-4: unit suite, sweep 86/87 (documented rv64ui-p-ma_data),
   atomics 19/19, M5 46/46 (spot 3 mid-run, full at close-out), si
   7/7.
2. **ON path:** test/m6 8/8; Linux boot banner.

**Standard gates every RTL task:** `make verisim` clean; unit suite
PASS; sweep 86/87; atomics 19/19.

**Linux boot gate:** `timeout 21600 bin/verisim/testbench
test/m6/linux/fw_jump.elf --kernel test/m6/linux/Image --initrd
test/m6/linux/rootfs.cpio | tee test/m6/boot.log; grep -q "Linux
version" test/m6/boot.log` → BOOT-PASS. Stretch (not gating):
`grep "Run /init"`.

## Files expected to change

- **Major edits:** `rtl/CSR.v` (claim + export, vectored tvec, time
  arm, wfi wake pulse), `rtl/RTU.v` (claim input port, wfi hold,
  tval=0-for-int check), `rtl/RVProc.v` (wires).
- **Small edits:** `rtl/RVProcAXI.v` (+`uart_irq` port → PLIC
  int_src[7]; un-tie :668), `testbench/RVProcTest.cpp` (dtb_addr, FDT
  isa/timebase, UART-IRQ read in step), `testbench/dut.cpp`
  (a0=hartid), `testbench/RVProc_io.h` if the IRQ crosses the C++/RTL
  boundary there.
- **New:** `test/m6/` (Makefile, 8 directed tests, env/, run_all.sh,
  `linux/` assets + run_linux.sh), `docs/08-verification.md` §8.17.
- **Unchanged by construction:** CLINT.v, PLIC.v (already live RTL),
  FPU/IDU/LSU/MMU/PMP.
