# M6 exploration note 3/3: Linux boot feasibility

Source: Explore agent ab070699d6760c5a5 (read-only), 2026-09-08.
Worktree = `.../rv906/.claude/worktrees/m2-integer` @ `d036f22` (M5-closed).

## 1. Testbench loader capabilities

The C++ harness is `testbench/TestBench.cpp` (base) + `RVProcTest.cpp`
(rv906 TB subclass) + `testbench/load_elf.cpp` + `testbench/fdt.cpp`.
(No `test/testbench.cpp`; TestMaster.v was retired in M2.)

**(a) ELF loading — program-header based, not fixed DRAM_BASE.**
`load_elf()` walks `PT_LOAD` headers, writes each at `phdr.p_paddr`
into the host-side page-sparse `ExtMem`; entry = `ehdr.e_entry`
(`load_elf.cpp:107,:113-124,:196`). Also scans `.symtab` for
`tohost/fromhost/begin_signature/end_signature/_stack_top`
(`load_elf.cpp:44-70`, `TestBench.cpp:190-197`). Nothing forces
0x80000000 — firmware kits just happen to link there.

**(b) Raw binary — two fixed roles only.** `load_bin(file, addr)`
(`load_elf.cpp:226-248`) invoked for `--kernel` → raw @ **0x80200000**
(RV64; `TestBench.cpp:30-34,:213-214`) and `--initrd` → raw @
**0x84000000** (`RVProcTest.cpp:275`; FDT gets linux,initrd-start/end,
`TestBench.cpp:146-159`). No generic `--raw file@addr`.

**(c) Multiple images — yes.** Every unrecognized arg is an ELF:
`elf[0]` sets initial_pc/SP/tohost symbols; `elf[1..]` load without
symbols (`TestBench.cpp:189-212`). Plus one `--kernel`, one
`--initrd`, and an auto-generated DTB (built by `TB::build_fdt()`,
`RVProcTest.cpp:279-355`, written to `run/c2rtl.dtb` and copied to
`dtb_addr` = **0x87000000** at `RVProcTest.cpp:274`, copy loop
`TestBench.cpp:169-183`). **No `--dtb/--dts` flag** — DTB is always
harness-generated.

**(d) Initial PC — effectively fixed at the reset vector.**
`DUT::init()` pokes `rootp->CPU_PC = pc; gpr[2] = sp; gpr[11] = dtb`
before the first clock (`dut.cpp:33-65`; CPU_PC =
`RVProcAXI->u_core->u_rtu->ex2_cur_pc`, `verisim.h:44-45`). But the
fetch PC is the RTL `RESET_VECTOR` = 0x80000000, hardwired
(`RVProcAXI.v:816`; `CSR.v:70,:1516` `cp0_xx_mrvbr`; IFU latches it
on boot_rst_vld, `IFU.v:200-203,:328,:421`). The harness documents the
mismatch (`RVProcTest.cpp:384-388`: "the ISS follows the reset
vector"). Consequence: **any M-stage payload must be linked/loaded at
0x80000000** (OpenSBI fw_jump is exactly that — non-issue).

**(e) UART capture / console PASS — capture yes, PASS-on-string only
externally.** TX bytes → `ttysrv::out()` → stdout (or UNIX-socket pty
under `--alloc-terminal`, `device/ttysrv.cpp:12-55,:108-127`).
`--expect <file>` arms line matching (`RVProcTest.cpp:400-401`;
`ttysrv.cpp:112-126,:179-200`; `>` lines feed UART input, `<` ends) —
**but a match never terminates or fails the run** (no
`TestBench::stop` hook). The existing pass idiom is external:
`timeout N testbench ... | grep -q PASS` (all run_all.sh). Same trick
works for a Linux banner (`grep -q "Linux version"`) with zero
harness changes.

**(f) Timeout / PASS / FAIL — tohost only; no C++ timeout.**
`run()` loops `while (!quitted)`; `quitted` is hardwired 0
(`RVProc.v:1525`). Exit: (1) tohost symbol value bit0 set → break,
result = gpr[3] (riscv-tests protocol; high-word 0x01010000 = putc,
`TestBench.cpp:256-324`); (2) SIGINT. **A payload with no tohost
symbol runs forever** (`tohost = -1`, `:188`); only a 1M-cycle
progress print (`:242-245`). All timeouts are external `timeout` in
the shell scripts. tohost must stay in the uncached 0x7FFF_F000
aperture (write-back D$ polls ExtMem).

**(g) Address map (guest-visible):**

```
s[0] MEM   0x80000000 mask 0x80000000  → 0x8000_0000–0xFFFF_FFFF (2GB) — MEMCTL → ExtMem; DEFAULT_SLAVE
s[1] CLINT 0x02000000 mask 0xFFFF0000  → 64KB — width adapter → CLINT.v
s[2] PLIC  0x0C000000 mask 0xFF000000  → 16MB — width adapter → PLIC.v
s[3] UART  0x10000000 mask 0xFFFF0000  → C++ model decodes 0x10001000 (32 B) → DI_UART; everything else → DI_EXT_MEM
```
(`RVProcAXI.v:145-158`; C++ decode `RVProcTest.cpp:22-31`. FDT
matches: `RVProcTest.cpp:310-355`. tohost 0x7FFF_F000 falls through
to the memory slave.)

## 2. Existing firmware/boot assets

**In the rv906 worktree / main checkout: none** (no OpenSBI, kernel,
DTB, bootrom). Bare-metal kit in `test/`: `entry.S` (M-mode crt0,
tohost `_exit`), `hello.c`, `clint_test.c`, `plic_test.c` (register
access only — int_src tied 0), `intr_test.c` (MTIP/MSIP handlers —
the M6 acceptance seed), `uart.c` (16550 @0x10001000),
`syscalls.c`, `printf.c`, Makefile (`-march=rv64imac_zicsr`).

**Closest in-repo M→S boot stub:** `test/m4/env/v/` — upstream
`env/v` with `vm.c`'s `vm_boot` (builds sv39 tables, switches satp,
mret into S) + rv906 `EXTRA_INIT` installing the uncached-tohost
megapage (`riscv_test.h:96-109`).

**Prebuilt Linux/OpenSBI assets — exist in three places:**

1. `/home/vlsilab/zhouz/workspace/C2RTL/linux/buildroot-2025.02.4-rv64gc/output/`
   — full buildroot workspace (2025-10-25): `images/fw_jump.elf` +
   `.bin`, `fw_dynamic.elf` + `.bin` (OpenSBI 1.3, PLAT=generic);
   `images/Image` — Linux 6.5, 13,683,200 B (source + .config at
   `output/build/linux-6.5/`: `CONFIG_64BIT=y`, `CONFIG_RISCV_SBI=y`
   + V01 + HVC_RISCV_SBI, `CONFIG_SERIAL_8250_CONSOLE=y` +
   `CONFIG_SERIAL_EARLYCON_RISCV_SBI`, `CONFIG_FPU=y`,
   `CONFIG_RISCV_ISA_ZBB=y`, no SMP, `CONFIG_BLK_DEV_INITRD=y`,
   `CONFIG_VIRTIO_BLK=y`); `images/rootfs.cpio` (1.99 GB — big);
   host toolchain `output/host/bin/riscv64-buildroot-linux-gnu-*`
   (gcc 13.3.0). fw_jump specifics: entry 0x80000000; PT_LOADs at
   0x80000000 (R E, 0x2a7d8) / 0x80040000 (RW); `_jump_addr` =
   **0x80200000**; `fw_next_arg1` returns **0x82200000** (fixed FDT
   address); `fw_next_mode` = PRV_S; `fw_boot_hart` = -1.
2. `/home/vlsilab/zhouz/workspace/C2RTL/linux/release/
   buildroot-2025.02.4-rv64gc-static/20250826/output/images/` —
   frozen release copy (same Image size; `start-qemu.sh` boots QEMU
   virt with fw_jump.bin).
3. `/home/vlsilab/zhouz/workspace/C2RTL/rvproc/RVProc6/vla/rvvla/test/
   llm_demo/prebuilt/linux/` — **the RVProc6 Linux-boot kit (small,
   sim-friendly)**: `boot.elf` (5,928 B, entry 0xa0000000 — FPGA
   BRAM stub, source `rvproc6/test/linux/boot.S`: clears mie/mip,
   sets SP/a0=0/a1=DTB, jumps to 0x80000000), `fw_jump.elf`
   (1,157,328 B — same constants, verified by disassembly), `Image`
   (same 13,683,200 B kernel), `rootfs.cpio` (**5,027,840 B — the
   only small rootfs**), `c2rtl.dts` (RVProc6-FPGA flavor: memory 512
   MB, uart@10001000 ns16550a reg-shift 2, clint@10010000,
   plic@18000000 — the OLD RVProc6 map, NOT rv906's
   0x02000000/0x0C000000). Host loader: `rvproc6/utils/rv64.cpp`
   (`--elf`×N + `--dts` via dtc + `--kernel`@0x80200000 +
   `--initrd`@0x88000000 + `--dtb`; README:205-229 documents the
   five-image flow).

**Shared test tree `/home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/`:**
riscv-tests (+spike/sim/arch-test/testfloat/cvw), `zephyr/` (a Zephyr
boot precedent against the OLD RVProc6 testbench; `c2rtl.dts6`
documents the old map), `nuttx/`, `qemu/`. No OpenSBI/kernel assets.

**Board reference:** `RVProc6/RVProc6.dut/board/nexysa7/` — full
alternate TB (C2RTL core + C++ CLINT/PLIC/IntrConn models,
multi-context 8-IRQ, uart16550, SPI-SD, GPIO; map UART 0x10000000,
CLINT 0x10010000, PLIC 0x18000000, SPI 0x20000100, GPIO 0x20000200,
EXT_MEM 0x80000000; FDT dtb_addr 0x87c00000/initrd 0x87000000;
IRQ fan-in `intr_conn.step(...)` + `irq[0]=(custom_irq<<16)|intr.irq[0]`
fed to `cpu.step` — i.e. the RVProc6 core consumes a packed IRQ word,
NOT rv906's three-pin mtip/msip/meip). `RVProc6.dut/RVProcTest.cpp`
also has an AHCI SATA model (`--disk-image`) — the disk-backed Linux
path.

## 3. UART identity

C++ model, not RTL: `device/uart16550.{h,cpp}`,
`UART16550_AXI4L : AXI4L::TSlaveFSM<UINT8>, ttysrv` — ns16550A
registers (RBR/THR, IER, IIR/FCR, LCR, MCR, LSR, MSR, SCR, DLL/DLM),
word-stride `(addr>>2)&7` matching the FDT `reg-shift=<2>
reg-io-width=<4>` (`RVProcTest.cpp:350-351`). LSR hardcodes 0x60
(TEMT|THRE) — TX never back-pressures; THR → `ttysrv::out`
(`uart16550.cpp:40-42,:70`). RTL exports crossbar slave 3 as
`G_axi_bus_s_ch_2_*`; `RVProcTest.cpp` decodes only 0x10001000 and
services it in `TB::step()` (`:530-531`). TX → stdout (or
`run/uart16550` socket). The model raises an interrupt flag
(`fsmUser()`, `:99-105`) but it dies: PLIC `int_src` tied 8'b0
(`RVProcAXI.v:668`).

## 4. What "single-hart Linux boot" minimally requires

Hard RTL prerequisites (already tracked): interrupt delivery
(RTU `int_vld_raw` producer + CSR claim), `time` CSR 0xC01 from
CLINT mtime, real WFI wake. OpenSBI's SBI-set-timer emulation waits
on MTIP; Linux `time_init()` calls `sbi_set_timer` early. (Kernel
`drivers/clocksource/timer-clint.c:276` matches "riscv,clint0" —
could drive CLINT from S if mideleg handed STIP, but that's the SBI
model anyway.)

**(a) OpenSBI fw_jump + mainline Linux — well-trodden, artifacts
exist.**
- fw_jump: entry 0x80000000 == RESET_VECTOR (no reset change);
  **ignores incoming a1, returns fixed `FW_JUMP_FDT_ADDR` =
  0x82200000** (verified by disassembly; source `opensbi-1.3/
  firmware/fw_jump.S:46-52`) → **TB `dtb_addr` must move
  0x87000000 → 0x82200000** (one line, `RVProcTest.cpp:274`).
  Needs from the DTB: CLINT ("riscv,clint0"; OpenSBI computes
  mtimecmp at base+0x4000, mtime at base+0xBFF8 — exactly rv906's
  map, `fdt_timer_mtimer.c:135-151` + `aclint_mtimer.h:23`), PLIC
  ("riscv,plic0"), UART ("ns16550a" + reg-shift 2), /memory for PMP.
  The TB-generated FDT provides all at the right rv906 addresses.
- Image: raw @ 0x80200000 — **exactly the TB's `kernel_addr`
  default AND fw_jump's `_jump_addr`**; 2 MB-aligned. Console via
  8250 (`ttyS0`, FDT `stdout-path = "/uart@10001000"`,
  `--bootargs` default "console=ttyS0 earlycon"). Initrd optional
  (5 MB cpio @ 0x84000000 gives userspace; without it boot still
  reaches the "Run /sbin/init" panic — enough for a banner marker).
- Simulator needs: fixed reset PC with firmware first (have),
  console visible (have), timer (RTL prerequisite), **an exit
  criterion** (OpenSBI/Linux never write tohost): external
  `timeout ... | grep -q "<marker>"` (zero changes) or an
  in-harness expect hook.

**(b) Hand-rolled M-mode stub, no OpenSBI — not plausible for this
stock kernel.** `CONFIG_RISCV_SBI=y` (mandatory on the standard
platform): early console, timer (`sbi_set_timer`), HSM probes are
all SBI calls — a stock kernel entered via mret-to-S stub ECALLs and
dies. Alternatives: rebuild the kernel
`CONFIG_RISCV_M_MODE=y + CONFIG_NONPORTABLE=y` (different,
less-tested config) or write a mini-SBI emulator (re-implementing
the bottom of OpenSBI). The existing `llm_demo/boot.elf` is a stub
that JUMPS to OpenSBI, not one that replaces it (exists only because
the FPGA flow resets at BRAM 0xa0000000). **Conclusion: effectively
we need OpenSBI anyway; use path (a).**

**(c) Artifacts — everything is on disk.** Prebuilt: fw_jump.elf +
Image + 5 MB rootfs.cpio (llm_demo/prebuilt/linux, known-good on the
sibling RVProc6 core) or the buildroot images. Rebuilding OpenSBI:
full 1.3 source at `output/build/opensbi-1.3/`; builds with
`/opt/xpack-riscv-none-elf-gcc-15.2.0-1` (bare-metal) or
bit-identically with `output/host/bin/riscv64-buildroot-linux-gnu-
gcc`. Knobs: `FW_TEXT_START=0x80000000`, `FW_JUMP_ADDR`,
`FW_JUMP_FDT_ADDR` (or fw_dynamic + a host-written
`fw_dynamic_info` struct — needs new TB features). Rebuilding the
kernel: full 6.5 tree + config at `output/build/linux-6.5/`,
fragments at `C2RTL/linux/custom/configs/6.5.x/` (`rv64.config` +
`rv64gc.frag` FPU=y — the `rv64i.frag`'s "RVProc6 doesn't support
FPU" note is obsolete for rv906 post-M5). DTB: TB generates its own
— no external dts/dtc needed.

**Config-level caution:** kernel has `CONFIG_RISCV_ISA_ZBB=y`; the
TB FDT says `riscv,isa = "rv64imac"` (`RVProcTest.cpp:295`). The
kernel selects alternative paths from this string (hwcap) — it must
list what rv906 actually implements: **`rv64imafdc`** (+
`_zicsr_zifencei`); OMIT `zbb` (rv906 does not implement it — a
wrong string enables the ZBB alternative and traps). `mmu-type
"riscv,sv39"` already set (`:296`).

## 5. rv12 precedent check

**rv12 never booted Linux — its M6 has not run.** Head is M5-FPU
(`ba6f972`); README's "Linux boot" line is the Phase-1 GOAL, and the
known-gaps section says "The interrupt path is built and
unexercised (M6): ... CLINT/PLIC integration and the kit's three
interrupt programs wait there" (`README.md:121-124`) — exactly
rv906's situation. rv12's CLINT.v/PLIC.v are byte-identical ports
to rv906's (same M0 rocketM port), and its PLIC/CLINT → cp0 wiring
(RVProcAXI.v:924-926 → RVProc.v:1939-1941) is the direct wiring
precedent. **No project in the rv12 lineage has an M6/Linux-boot
precedent to copy; the only working Linux boot on this
infrastructure is the C2RTL RVProc6 core flow in
`RVProc6/vla/rvvla/test/llm_demo/`** — the real reference for
artifacts and loader conventions.

## 6. CLINT/PLIC RTL in the worktree

**CLINT.v** (module CLINT, XLEN=64, BASE 0x02000000; AXI4-Lite):
msip[0]@0x0000 (1 bit), mtimecmp[0]@0x4000 (64-bit byte-strobe),
mtime@0xBFF8 (64-bit, readable+writable); mtime increments on
`rtc_tick` (= clk/100 divider, RVProcAXI.v:369-385); `mtip = (mtime
>= mtimecmp)`, `msip = msip_reg`. **No S banks** (no ssip@0xC000 /
stimecmp@0xD000 — the donor's S-mode CLINT is absent from the M0
port).

**PLIC.v** (module PLIC, N_SOURCE=8, N_PRIORITY=8, BASE 0x0C000000):
prio 1-7 @0x004-0x01C (3-bit), pending @0x1000, enable ctx0 @0x2000,
threshold @0x200000, claim/complete @0x200004 (level-sensitive,
claim clears pending, complete re-latches if still asserted);
`meip = (max_prio > threshold) && max_id != 0`. **ONE M-mode context
only — no S context (context 1 @0x201004 unimplemented).**
Consequence: no S-mode device IRQs; boot-to-banner survives because
the 8250 console is polled.

SoC wiring: CLINT/PLIC instantiated on the crossbar with width
adapters (`RVProcAXI.v:513-687`); `.int_src(8'b0)` tied (`:668`);
`.mtip(clint_mtip)/.msip(clint_msip)/.meip(plic_meip)` into RVProc
(`:885-888`) → CSR.v pins → mip readback only. Delivery dead:
`RTU.v:744 int_vld_raw = 15'd0`, `CSR.v:1474
cp0_rtu_ex1_expt_int = 1'b0`, no time CSR, WFI no-op.

## BOOT PATH OPTIONS

| | (a) OpenSBI fw_jump + prebuilt Linux 6.5 (RECOMMENDED) | (b) Hand-rolled M-mode stub, no OpenSBI | (c) fw_dynamic + host info struct |
|---|---|---|---|
| **Artifacts** | All on disk: fw_jump.elf + Image + 5 MB rootfs.cpio (llm_demo/prebuilt/linux or buildroot images). DTB auto-generated. | ~512 B stub + mini-SBI emulator (KBs of asm) OR kernel rebuild M-mode/NONPORTABLE. Template: test/m4/env/v. | fw_dynamic.elf + host-placed fw_dynamic_info (magic 0x4942534f) + kernel + DTB. |
| **Loader changes** | **One line:** dtb_addr 0x87000000 → 0x82200000 (RVProcTest.cpp:274). Everything else already works. | Same + stub placement (stub as elf[0] @0x80000000). | New TB feature: write 8-word struct + poke a2 (dut.cpp pokes only PC/SP/a1). |
| **TB changes** | Exit criterion (external grep idiom = zero changes; or ttysrv expect hook / --pass-string). FDT riscv,isa → "rv64imafdc" (+zicsr_zifencei, NO zbb). | Same console/exit story. | Same as (a) + a2-struct writer. |
| **RTL prereqs (common)** | Interrupt delivery, time CSR, WFI wake. | Same + S-mode delegation if kernel drives CLINT directly. | Same as (a). |
| **Biggest risk** | **Sim wall-clock**: full boot = hundreds of millions of Verilator cycles (hours). Scope acceptance to the earliest marker ("Linux version …" via earlycon) + long timeout. Secondary: FDT isa string vs ZBB (trap) and FP (silent disable); single-context PLIC = no S-mode device IRQs. | **Kernel reconfiguration** — "write a small OpenSBI"; strictly more work, entire debug surface on custom code. | **Loader complexity for zero single-hart payoff.** |

**Bottom line:** path (a) needs no new artifacts; the loader already
supports the exact multi-image layout fw_jump expects; the total
harness-side delta is one address constant, one FDT string, and a
console-string PASS criterion. The gating work is the M6 RTL:
interrupt delivery (RTU/CSR), time CSR, WFI wake.
