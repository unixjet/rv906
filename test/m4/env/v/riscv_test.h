/* test/m4/env/v/riscv_test.h - the M4 v-env, UPSTREAM's PLUS ONE MACRO,
 * PORTED from rv12/test/m4/env/v/riscv_test.h with a DEVIATION from its
 * precedent (see part 1 below).
 *
 * *** THIS FILE IS AN INHERITANCE, NOT A COPY. *** A vendored copy of
 * env/v/riscv_test.h -- or worse of vm.c's 300+ lines -- would drift from
 * upstream. This file is a few lines of preprocessor over the real one,
 * reached with #include_next: `-I test/m4/env/v -I $(RISCV_TESTS)/env/v`
 * finds this file first and the upstream one second.
 *
 * WHAT IT ADDS: EXTRA_INIT, which rv12's version does TWO things in; this
 * one does only ONE, and the missing half is a deliberate deviation, not an
 * omission.
 *
 * -------------------------------------------------------------------------
 * (1) NO jTLB/uTLB PRIMING INVALL HERE -- DEVIATION FROM RV12'S PRECEDENT.
 * -------------------------------------------------------------------------
 * rv12's inheritor of this file puts a bare `sfence.vma x0, x0` in
 * EXTRA_INIT because rv12's jTLB is a two-level SRAM structure whose
 * replacement-FIFO column is undefined before the first INVALL -- a real
 * hazard under Verilator's zero-filled SRAM arrays, where a refill before
 * priming writes no way and the jTLB never retains.
 *
 * rv906's TLB has no such hazard. It is a SINGLE 128-entry FULLY-ASSOCIATIVE
 * FLOP ARRAY (rvproc's D11/D4 deviation from the donor's uTLB+jTLB split),
 * and every entry's valid bit is a genuine register reset by `rst_n`, not an
 * uninitialized SRAM word:
 *
 *   rtl/MMU.v:685-693 (per-entry generate, mirroring PMP.v's register-file
 *   pattern):
 *     always @(posedge clk or negedge rst_n) begin
 *         if (!rst_n)
 *             tlb_vld[te] <= 1'b0;
 *         else if (tlb_inv_all)
 *             tlb_vld[te] <= 1'b0;
 *         ...
 *
 * Every `tlb_vld[te]` is a flop with an explicit `!rst_n` reset to 0, so the
 * array is cleanly invalid out of reset on the real hardware AND under
 * Verilator alike -- there is no "undefined column" state to prime away.
 * `env/v/vm.c:291`'s `flush_page(DRAM_BASE)` (an IVA-flavor
 * `sfence.vma a5, x0`) is the only sfence the v-env issues before the first
 * translated access, and on rv906 that is sufficient: a stale entry cannot
 * exist because the array starts empty. Adding rv12's priming INVALL here
 * would be harmless but is not needed, so it is left out to keep this file
 * honest about what rv906 actually requires.
 *
 * -------------------------------------------------------------------------
 * (2) THE UNCACHED-tohost MEGAPAGE -- the other half of rv906v.ld, PORTED
 *     from rv12 verbatim in mechanism (same address arithmetic, same sv39
 *     shape; rv906 implements sv39 and nothing else, same as rv12).
 * -------------------------------------------------------------------------
 * rv906v.ld moves `.tohost` to PA 0x7FFF_F000; read that file's header for
 * why. The kernel reaches `tohost` PC-RELATIVELY (-mcmodel=medany), so a
 * symbol 4 KB below the text base is a VA 4 KB below the kernel window --
 *
 *     kernel window   VA 0xFFFF_FFFF_FFE0_0000 -> PA 0x8000_0000, 2 MB
 *                     (vm.c:277-278, kernel_l2pt[511])
 *     tohost's VA     0xFFFF_FFFF_FFE0_0000 + (0x7FFF_F000 - 0x8000_0000)
 *                   = 0xFFFF_FFFF_FFDF_F000
 *
 * -- and vm_boot leaves the megapage that contains it EMPTY. This installs
 * it: one 2 MB leaf at kernel_l2pt[510] covering VA 0xFFFF_FFFF_FFC0_0000 ->
 * PA 0x7FE0_0000, so VA 0xFFFF_FFFF_FFDF_F000 (offset 0x1F_F000 into the
 * frame) is PA 0x7FFF_F000. The VPN split is sv39's: VA[38:0]=0x7F_FFDF_F000,
 * VPN[2]=511 (l1pt[511] -> kernel_l2pt, already set by vm_boot), VPN[1]=510,
 * offset 0x1F_F000.
 *
 * Same safety argument as rv12's: `pt` is a zeroed .bss global, extra_boot
 * runs in M-mode with satp=0 so the store is untranslated, vm_boot never
 * touches index 510, the mapped VA range is used by nothing else, and the
 * leaf is R|W (no X, no U) with A and D PRESET -- required by rv906's
 * fault-based A/D policy (D-M4-1, rtl/MMU.v: `ptw_is_store && !pte_d`) for a
 * page software never wants a fault on -- and PPN[0]=0 so it is not a
 * misaligned 2 MB superpage.
 *
 * `kernel_l2pt` is `pt[2]` and `PTES_PER_PT` is 512 for RV64
 * (env/encoding.h RISCV_PGLEVEL_BITS=9), so kernel_l2pt[510] is at byte
 * (2*512 + 510)*8 = 12272 = 0x2FF0 from `pt`. A wrong constant here is not a
 * silent failure: the first `do_tohost` would take a store page fault at a
 * kernel VA and vm.c's fault handler assert would fire.
 */

#ifndef _RV906_M4_V_ENV_H
#define _RV906_M4_V_ENV_H

/* The real one: resumed after THIS file's directory on the -I path. */
#include_next "riscv_test.h"

/* kernel_l2pt[510], in bytes from `pt` -- (2 * PTES_PER_PT + 510) * 8. */
#define RV906_V_TOHOST_PTE_OFF  ((2 * 512 + 510) * 8)
/* PPN(0x7FE00000) << PTE_PPN_SHIFT | V | R | W | A | D. */
#define RV906_V_TOHOST_PTE      ((0x7FE00000 >> RISCV_PGSHIFT) << PTE_PPN_SHIFT \
                                 | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D)

/* The included header defines EXTRA_INIT unconditionally (empty); redefine
 * it after the include so the preprocessor sees the new body at the
 * RVTEST_CODE_BEGIN use site.
 *
 * t0/t1/t2 only: `extra_boot` is reached by `call` (env/v/entry.S) and must
 * not disturb ra; entry.S reloads a0 after the call. No `sfence.vma` here --
 * see part (1) above; rv906's TLB needs none. */
#undef EXTRA_INIT
#define EXTRA_INIT                                                      \
  la t0, pt;                                                            \
  li t1, RV906_V_TOHOST_PTE_OFF;                                        \
  add t0, t0, t1;                                                       \
  li t2, RV906_V_TOHOST_PTE;                                            \
  sd t2, 0(t0);

#endif /* _RV906_M4_V_ENV_H */
