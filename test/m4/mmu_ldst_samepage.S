# mmu_ldst_samepage.S - a load and a store to the SAME faulting page,
#                       reaching the fault-based A/D pager back to back.
#
# ADAPTED from rv12/test/m4/mmu_ldst_samepage.S -- NOT a straight port of
# its rationale. rv12's file exists to regress DEVIATION D-M4-7, a livelock
# specific to rv12's TWO-PORT DTLB: two independent accesses (a load and a
# store) that both present to the uTLB's two ports IN THE SAME CYCLE and
# BOTH abort on the same coincidence, which an operand-merge bug read as a
# flush and discarded a parked page fault, looping forever.
#
# rv906's D-side MMU interface has NO SECOND PORT to race:
#
#   rtl/MMU.v:126-129  "walk abort on a flushed requester ... lsu_mmu_abort
#                       is new (donor's lsu_mmu_abort0/1 collapsed to one --
#                       rv906's single-outstanding DTLB request, D1)."
#   rtl/LSU.v:192-195  "the walk-abort pin for THIS port (donor's
#                       lsu_mmu_abort0/1 collapsed to one -- rv906's
#                       single-outstanding DTLB request)."
#
# One request, one abort pin, one walker: the specific both-ports-abort-in-
# the-same-cycle race D-M4-7 stood in for CANNOT occur on this machine, so
# there is no livelock analog to regress here. What this file keeps instead
# is the property one level up -- a load and a store that both fault on the
# SAME page, issued back to back with no data dependency between them, both
# routed through the fault-based A/D pager (rtl/MMU.v: `!pte_a` and
# `ptw_is_store && !pte_d` both page-fault, D-M4-1) -- as a liveness/
# coherence check of the single-outstanding walker taking two sequential
# faults on one leaf and having each retry land correctly. If this ever
# regresses on rv906 it will most likely HANG rather than fail outright,
# same honest failure mode as rv12's file, which is why `run_all.sh`'s
# timeout (rc 124) is still the thing that catches it.
#
# THE RECIPE: a LOAD and a STORE to the SAME page, no dependency between
# them, addressed from a base 16 bytes into the page BELOW the target so
# `base + offset` carries across the page boundary inside AG -- the natural
# `la`-relative addressing shape riscv-tests uses everywhere. The page's
# leaf has A = 0, so each access's walk PAGE-FAULTS on first touch; the
# handler below is the fault-based A/D dance (set A if clear, else set D,
# sfence the page, return WITHOUT advancing mepc so the access retries) --
# `env/v/vm.c`'s handle_fault algorithm, self-contained here in assembly
# since this test runs in the p-env, not the v-env.

#include "mmu_env.h"
#include "test_macros.h"

RVTEST_RV64M
RVTEST_CODE_BEGIN

  MMU_BUILD_L1L2(page_table_1, page_table_2, page_table_3)
  # VA 0x2000 -> datapage, R|W|X|U but **A and D CLEAR**: the first touch of
  # either kind faults, which is the whole point.
  MMU_MAP_4K(page_table_3, 2, datapage, PTE_V | PTE_U | PTE_R | PTE_W | PTE_X)
  # VA 0x3000 -> datapage2, the same shape, for the reversed-order pair below.
  MMU_MAP_4K(page_table_3, 3, datapage2, PTE_V | PTE_U | PTE_R | PTE_W | PTE_X)
  MMU_ON(page_table_1)
  MMU_MPRV_S

  li a5, 0x2000
  addi a4, a5, -16          # a base 16 bytes into the page BELOW
  li a0, 0x5a5a

  # 2: the pair. `lw` first, `sw` second, no register dependency between
  #    them, both `16(a4)` so both cross the 4 K boundary inside AG.
  li TESTNUM, 2
  lw a2, 16(a4)
  sw a0, 16(a4)

  # 3: and the same pair the other way round, because the older-access
  #    select must not care which side wins
  li TESTNUM, 3
  li a5, 0x3000
  addi a4, a5, -16
  li a0, 0x3c3c
  sw a0, 16(a4)
  lw a2, 16(a4)
  li t2, 0x3c3c
  bne a2, t2, die

  # 4: the first page's store landed too
  li TESTNUM, 4
  li a5, 0x2000
  lw a2, 0(a5)
  li t2, 0x5a5a
  bne a2, t2, die

  MMU_MPRV_OFF
  RVTEST_PASS

  TEST_PASSFAIL

  .align 2
  .global mtvec_handler
mtvec_handler:
  # Only the three page-fault causes are handled; the A/D dance below is
  # env/v/vm.c's, and a fault on a leaf that already has BOTH bits is a
  # machine bug, not a pager event -- report it rather than loop.
  csrr t0, mcause
  li t1, CAUSE_LOAD_PAGE_FAULT
  beq t0, t1, 1f
  li t1, CAUSE_STORE_PAGE_FAULT
  beq t0, t1, 1f
  li t1, CAUSE_FETCH_PAGE_FAULT
  bne t0, t1, mmu_bad_cause

1:
  # Which leaf faulted? mtval is the VA; bit 12 picks page 2 from page 3.
  csrr t2, mtval
  srli t2, t2, 12
  andi t2, t2, 1            # 0 -> VA 0x2000 (l3[2]), 1 -> VA 0x3000 (l3[3])
  la t3, page_table_3 + 16
  beqz t2, 2f
  la t3, page_table_3 + 24
2:
  ld t0, 0(t3)
  andi t1, t0, PTE_A
  bnez t1, 3f
  ori t0, t0, PTE_A
  j 4f
3:
  andi t1, t0, PTE_D
  bnez t1, mmu_ad_twice
  ori t0, t0, PTE_D
4:
  sd t0, 0(t3)
  # The VA-qualified flavour, exactly as env/v/vm.c uses it: this is the
  # sfence the pager owes for the entry it just changed, and the reason the
  # retry sees the new A/D bits instead of the uTLB's copy.
  sfence.vma
  mret

mmu_ad_twice:
  # A and D were already set and the machine faulted anyway.
  addi TESTNUM, TESTNUM, 200
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
datapage: .dword 0
.align 12
datapage2: .dword 0

RVTEST_DATA_END
