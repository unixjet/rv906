# mmu_pmpstore.S - a store the PMP denies: PRECISE, cause 7, and the SQ
#                  entry given back through the pop.
#
# PORT of rv12/test/m4/mmu_pmpstore.S. Runs TRANSLATED so the PMP's PHYSICAL-
# address check is the one exercised, and covers the store-side access-fault
# path plus one AMO whose read/write halves are both PMP-checked.
#
# WHY THE DENIAL IS A W-WITHOUT-R WINDOW RATHER THAN A CLOSED ONE. The test
# needs the LOAD to succeed and the STORE to fail on the SAME address, so
# that "the store faulted" cannot be confused with "the address is
# unreachable". Entry 0 is a NAPOT region over the target page carrying R
# and X but not W, and entry 1 opens everything else.
#
# PRECISION IS THE POINT AND IT IS CHECKED THREE WAYS: the cause is 7
# (store/AMO access fault, NOT a page fault -- a PMP denial is a physical-
# memory refusal and the two causes must not be confused), mepc is the
# storing instruction, and the store DID NOT LAND.

#include "mmu_env.h"
#include "test_macros.h"

RVTEST_RV64M
RVTEST_CODE_BEGIN

  MMU_BUILD_L1L2(page_table_1, page_table_2, page_table_3)
  MMU_MAP_4K(page_table_3, 2, page_ro, MMU_LEAF_RWX)
  MMU_MAP_4K(page_table_3, 3, page_rw, MMU_LEAF_RWX)
  MMU_ON(page_table_1)

  # PMP entry 0: a 4 KB NAPOT region over page_ro, R + X but NO W.
  # NAPOT addressing on RV64: pmpaddr = (base >> 2) | ((size/2 - 1) >> 2), so
  # a 4 KB region is the page base >> 2 with the low 9 bits set.
  la t0, page_ro
  srli t0, t0, 2
  li t1, 0x1FF
  or t0, t0, t1
  csrw pmpaddr0, t0
  # PMP entry 1: everything, R + W + X, so only page_ro is restricted.
  li t0, -1
  csrw pmpaddr1, t0
  li t0, (PMP_NAPOT | PMP_R | PMP_X)                                  \
         | ((PMP_NAPOT | PMP_R | PMP_W | PMP_X) << 8)
  csrw pmpcfg0, t0

  MMU_MPRV_S

  # 2: a store to the UNRESTRICTED page still works -- the control that says
  #    pmpcfg0 did not simply close the machine
  li TESTNUM, 2
  li a3, 0x3000
  li a0, 0x5a5a5a5a
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die

  # 3: a LOAD from the restricted page works (R is granted)
  li TESTNUM, 3
  li a3, 0x2000
  ld a2, 0(a3)
  li t2, 0x1111
  bne a2, t2, die

  # 4: THE ROW -- a STORE to the restricted page takes cause 7, precisely
  li TESTNUM, 4
  li a6, CAUSE_STORE_ACCESS
  la a7, 4f
  li a0, 0xDEADBEEF
  sd a0, 0(a3)
  j die
4:

  # 5: and the store did NOT land -- the SQ entry was popped back, not
  #    drained
  li TESTNUM, 5
  ld a2, 0(a3)
  li t2, 0x1111
  bne a2, t2, die

  # 6: the same denial for an AMO. An AMO needs BOTH read and write
  #    permission; which access fault a machine reports for the half it
  #    refuses is not something this file adjudicates -- it FAULTS, on the
  #    right instruction, and the AMO's store half does not land.
  li TESTNUM, 6
  li a6, CAUSE_LOAD_ACCESS
  li a5, CAUSE_STORE_ACCESS
  la a7, 6f
  li a1, 1
  amoadd.d a4, a1, (a3)
  j die
6:
  li a5, 0
  ld a2, 0(a3)
  li t2, 0x1111
  bne a2, t2, die

  # 7: the machine is still ordering stores afterwards -- the pop path must
  #    leave the queue usable, not merely non-fatal
  li TESTNUM, 7
  li a3, 0x3000
  li a0, 0x3c3c3c3c
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die

  MMU_MPRV_OFF
  RVTEST_PASS

  TEST_PASSFAIL

  .align 2
  .global mtvec_handler
mtvec_handler:
  csrr t0, mcause
  beq t0, a6, 1f
  bne t0, a5, mmu_bad_cause         # a5 is the second accepted cause, or 0
1:
  # mepc must be the storing instruction itself: a7 is the instruction AFTER
  # the `j die` that follows it, so mepc + 8 is the landing pad.
  csrr t1, mepc
  addi t1, t1, 8
  bne t1, a7, mmu_imprecise
  csrw mepc, a7
  mret

mmu_imprecise:
  addi TESTNUM, TESTNUM, 100
  j die
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
page_ro: .dword 0x1111
.align 12
page_rw: .dword 0

RVTEST_DATA_END
