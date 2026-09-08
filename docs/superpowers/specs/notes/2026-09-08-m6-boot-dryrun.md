# M6 dry-run: pre-fix Linux boot on d036f22 RTL (2026-09-08)

Command (5-min cap):
```
timeout 300 bin/verisim/testbench <prebuilt>/fw_jump.elf \
    --kernel <prebuilt>/Image --initrd <prebuilt>/rootfs.cpio
```
(`<prebuilt>` = `.../rvvla/test/llm_demo/prebuilt/linux`)

## Result

- Ran to **35,000,000 cycles** (~117K c/s while spinning), then killed
  by timeout. `tohost=0` throughout (no riscv-tests protocol for
  boot payloads).
- **Zero console output** — expected: no `/chosen` node in the TB FDT
  (no bootargs/stdout-path), so neither OpenSBI nor kernel has a
  console. (See design doc: Task 5 adds it.)
- **Hang point: `0x800006c2` in OpenSBI `sbi_init`** — the
  coldboot-secondary-wait loop:

```
800006b6:  ld   a5, 24(s2)        # coldboot_lottery+24
800006ba:  fence r, rw
800006be:  bnez a5, 800007a6
800006c2:  wfi                     # <-- spins here
800006c6:  csrr a5, mip
800006ca:  srli a4, a5, 0x3
800006ce:  andi a4, a4, 257        # mip[11]|mip[3] = MEIP|MSIP
800006d2:  beqz a4, 800006c2
800006d4:  j    800006b6
```

  (Pre-loop, at sbi_init+0xa4-0xaa, it ORs this hart's bit into
  `coldboot_wait_hmask` — waiting for other harts' IPIs.)

## Interpretation

1. **OpenSBI M-mode init executes cleanly on rv906** — fetch, PMP,
   CSRs, M-mode traps (ecall from the fw code path) all work. No
   illegal-instruction or page-fault detours up to the wait loop.
2. The wait is the **expected first failure**: with no DTB at
   0x82200000 (FDT is at 0x87000000 pre-fix), OpenSBI's domain init
   gets a garbage/empty FDT and waits for harts that don't exist.
   The one-line `dtb_addr` fix (Task 8) is expected to clear this:
   a proper single-cpu FDT yields a 1-hart domain.
   **If it does NOT clear after the dtb fix, suspect:** (a) the FDT
   parse path reading 0x82200000 through the MMU (M-mode, bare —
   should be identity), (b) `fw_boot_hart=-1` coldboot logic needing
   something else from the FDT, (c) a real domain/hmask bug — in that
   case debug with the ISS checker + PC tracing.
3. **Cycle rate ~117K c/s while spinning**, and the spin is inflated:
   each WFI (no-op in M4) costs ~1 DRAM read (flush → refetch of the
   loop's instruction line; the [axi] counter climbs ~125K/1M cycles
   with last_addr pinned at 800006c0 = the WFI itself). Real boot
   doesn't spin on WFI (interrupts deliver) — but budget wall-clock
   accordingly: expect ~0.1-1M c/s for memory-active kernel code,
   banner at tens-to-hundreds of M cycles → 1 min - 4 h. Use
   `timeout 21600` + background run.
4. **The M1 ISS checker runs by default** (`checker=1` in the log).
   It is a free validation net for the boot run, BUT it was
   validated through M5 (integer+FPU, S/U-mode exceptions) and has
   never seen a *taken interrupt*. If the first real interrupt
   produces a checker mismatch that is an ISS modeling gap (not an
   RTL bug), re-run the boot with `--no-checker` and record which
   way it went. Keep the checker on for the first attempt.
5. Wall-clock sanity: the M5-compliant ELFs (100K-1M cycles) each
   finish in seconds; 35M spin cycles took 300 s. The kernel banner
   is the long pole — no other boot-stage work is hidden (fw load
   was done by cycle ~2M).

## For Task 8 (boot run)

- After dtb_addr/a0/FDT fixes, first expected console text: the
  **OpenSBI banner** ("OpenSBI v1.3" — via its console, which now
  finds stdout-path) — that is the earliest observable progress.
  Then kernel earlycon ("Linux version 6.5...").
- Grep targets in order: `OpenSBI` → `Linux version` → (stretch)
  `Run /init`.
- If OpenSBI banner appears but kernel banner stalls: it's past the
  M→S jump; next suspects = SBI timer (sbi_set_timer → MTIP →
  OpenSBI → STIP) and S-mode interrupts — exactly Tasks 1-4's
  delivery path under real load.
