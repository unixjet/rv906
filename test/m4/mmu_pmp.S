# mmu_pmp.S - the OpenSBI-shaped PMP set, MMU off and MMU on.
#
# PORT of rv12/test/m4/mmu_pmp.S. Verified against rtl/PMP.v before porting:
# rv906's readback is CONFORMANT (rtl/PMP.v's own header, lines 20-26: "rv906
# implements the conformant readback (the same call rv12 made)" -- i.e. NAPOT
# reads pmpaddr[8:0] as ones and [9] live, OFF/TOR reads [9:0] as zero, and
# the raw C906 donor behavior this fixes -- low 9 bits always zero -- is
# EXPLICITLY NOT what rv906 implements). This file's only readback assertion
# (case 7) is on the pmpcfg0 BYTE (L,A,X,W,R), whose shape is unaffected by
# the pmpaddr-specific fix either way, so the test transfers unchanged; it is
# noted here because the readback question is exactly the kind of fact this
# directed set must not silently assume.
#
# Four shapes, in the order a real boot uses them and in the order this
# machine can survive them:
#   2/3  the WHOLE-SPACE NAPOT entry -- literally what env/v/vm.c writes and
#        what every riscv-test env writes, so it is the configuration the
#        whole suite has been running under without ever asserting it;
#   4    a TOR WINDOW that grants R and refuses W, the shape an SBI uses to
#        protect its own text from the kernel;
#   5    NO MATCHING ENTRY -- M-mode passes by default, S-mode is DENIED.
#        This is the one that catches a PMP wired the wrong way round;
#   6    a LOCKED entry with no permissions, which refuses M-MODE ITSELF.
#        That is the L bit's whole point and it is irreversible until reset,
#        so it is last and it covers one page nothing else needs.
#
# EACH SHAPE IS RUN UNTRANSLATED AND THEN TRANSLATED, because the PMP checks
# the PHYSICAL address: with the MMU on, the address it sees is the walk's
# output, so a check that had quietly become a VA compare would pass the
# first half and fail the second.
#
# THE GRANULARITY IS 4 KB (rtl/PMP.v: PMP_ADDR_W=29 stores PA[39:11], G=10):
# a region of 2^k bytes is `(base >> 2) | ((2^k / 8) - 1)`, which for 4 KB is
# the page base shifted down by two with the low nine bits set.

#include "mmu_env.h"
#include "test_macros.h"

RVTEST_RV64M
RVTEST_CODE_BEGIN

  MMU_BUILD_L1L2(page_table_1, page_table_2, page_table_3)
  MMU_MAP_4K(page_table_3, 2, page_x, MMU_LEAF_RWX)
  MMU_MAP_4K(page_table_3, 3, page_y, MMU_LEAF_RWX)
  MMU_MAP_4K(page_table_3, 4, page_lock, MMU_LEAF_RWX)
  MMU_ON(page_table_1)

  # 2: the whole-space NAPOT entry, untranslated
  li TESTNUM, 2
  li t0, -1
  csrw pmpaddr0, t0
  li t0, PMP_NAPOT | PMP_R | PMP_W | PMP_X
  csrw pmpcfg0, t0
  la a3, page_x
  li a0, 0x1111
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die

  # 3: the same entry, translated, as S-mode
  li TESTNUM, 3
  MMU_MPRV_S
  li a3, 0x2000
  li a0, 0x2222
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die
  MMU_MPRV_OFF

  # 4: a TOR window over page_x granting R but not W. Entry 0's pmpaddr is
  #    the window's BOTTOM (its cfg is OFF, so it matches nothing itself) and
  #    entry 1 is the TOR entry whose pmpaddr is the top. Entry 2 opens the
  #    rest.
  li TESTNUM, 4
  la t0, page_x
  srli t0, t0, 2
  csrw pmpaddr0, t0
  la t0, page_y                     # page_y is the next page: the window's top
  srli t0, t0, 2
  csrw pmpaddr1, t0
  li t0, -1
  csrw pmpaddr2, t0
  li t0, (PMP_TOR | PMP_R) << 8
  li t1, (PMP_NAPOT | PMP_R | PMP_W | PMP_X) << 16
  or t0, t0, t1
  csrw pmpcfg0, t0                  # cfg0 = 0 (OFF), cfg1 = TOR|R, cfg2 = all
  MMU_MPRV_S
  li a3, 0x2000
  ld a2, 0(a3)                      # R is granted
  li t2, 0x2222
  bne a2, t2, die
  li a6, CAUSE_STORE_ACCESS
  la a7, 4f
  li a0, 0x3333
  sd a0, 0(a3)                      # W is not
  j die
4:
  li a3, 0x3000                     # page_y is OUTSIDE the window
  li a0, 0x4444
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die
  MMU_MPRV_OFF

  # 5: no entry matches anything -- M passes by default, S is denied
  li TESTNUM, 5
  csrw pmpcfg0, x0
  la a3, page_x
  ld a2, 0(a3)                      # M-mode: the default is ALLOW
  li t2, 0x2222
  bne a2, t2, die
  MMU_MPRV_S
  li a3, 0x2000
  li a6, CAUSE_LOAD_ACCESS
  la a7, 5f
  ld a2, 0(a3)                      # S-mode: the default is DENY
  j die
5:
  MMU_MPRV_OFF

  # 6: a LOCKED entry over page_lock with no permissions. L makes the entry
  #    apply to M-MODE as well, so an M-mode store to that page faults -- and
  #    the entry can never be taken back, which is why it is last and why it
  #    covers a page nothing else uses. Entry 1 re-opens everything else.
  li TESTNUM, 6
  la t0, page_lock
  srli t0, t0, 2
  li t1, 0x1FF
  or t0, t0, t1
  csrw pmpaddr0, t0
  li t0, -1
  csrw pmpaddr1, t0
  li t0, (PMP_L | PMP_NAPOT)                                          \
         | ((PMP_NAPOT | PMP_R | PMP_W | PMP_X) << 8)
  csrw pmpcfg0, t0
  la a3, page_lock
  li a6, CAUSE_STORE_ACCESS
  la a7, 6f
  li a0, 0x5555
  sd a0, 0(a3)                      # M-mode, and the L bit refuses it
  j die
6:

  # 7: the locked cfg byte is FROZEN -- writing pmpcfg0 again cannot clear it
  li TESTNUM, 7
  csrw pmpcfg0, x0
  csrr t0, pmpcfg0
  andi t1, t0, 0xFF
  li t2, PMP_L | PMP_NAPOT
  bne t1, t2, die

  # 8: and with the locked byte still in place, a page OUTSIDE it is
  #    reachable from M -- the entry is a hole, not a wall
  li TESTNUM, 8
  li t0, -1
  csrw pmpaddr1, t0
  li t0, (PMP_L | PMP_NAPOT)                                          \
         | ((PMP_NAPOT | PMP_R | PMP_W | PMP_X) << 8)
  csrw pmpcfg0, t0
  la a3, page_x
  li a0, 0x6666
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die

  RVTEST_PASS

  TEST_PASSFAIL

  .align 2
  .global mtvec_handler
mtvec_handler:
  csrr t0, mcause
  bne t0, a6, mmu_bad_cause
  csrw mepc, a7
  mret

mmu_bad_cause:
  MMU_REPORT_CAUSE
die:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

.align 12
page_table_1: .dword 0
.align 12
page_table_2: .dword 0
.align 12
page_table_3: .dword 0
.align 12
page_x: .dword 0
.align 12
page_y: .dword 0
.align 12
page_lock: .dword 0

RVTEST_DATA_END
