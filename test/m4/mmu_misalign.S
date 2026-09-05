# mmu_misalign.S - rv906 contract 3: misalignment is TRAP-ONLY, and
#                   MXSTATUS.MM has NO EFFECT on that -- causes 4 and 6,
#                   through translation and without it.
#
# ADAPTED from rv12/test/m4/mmu_misalign.S -- NOT a straight port. rv12's
# file exercises MXSTATUS.MM=1 SPLITTING a misaligned access (silently
# completing it) versus MM=0 TRAPPING it, because that donor's LSU honors
# MM. rv906's LSU.v does not:
#
#   rtl/LSU.v:17  "contract 3 (misalignment: trap-only, ignores MXSTATUS.mm)"
#   rtl/LSU.v:394 "contract 3: `cp0_lsu_mm` is NEVER consulted here --
#                  always trap."
#   rtl/LSU.v:395 `wire _cp0_lsu_mm_unused = cp0_lsu_mm;`
#   rtl/LSU.v:384-386 `ag_misalign` (half/word/dword alignment check) feeds
#                  `misalign_issue` unconditionally -- no `cp0_lsu_mm` term
#                  anywhere in that expression.
#
# So on rv906 EVERY misaligned load/store/AMO traps regardless of MM, and
# this file asserts exactly that, both ways: MM=0 traps (the case rv12's
# file also checks) AND MM=1 (RESET VALUE, rvproc_pkg.sv/CSR.v) ALSO traps
# -- the negative that would catch a machine that quietly grew rv12's split
# path. rv64mi-p-ma_addr already exercises the trapping shapes at boot
# MM=1; this file's job is to pin MXSTATUS.MM as a documented no-op via an
# explicit CSR poke (MXSTATUS is CSR 0x7C0, MM is bit 15 -- rvproc_pkg.sv
# CSR_MXSTATUS/MXSTATUS_MM), which is spec-legal because a machine
# configuration a test can reach with a CSR write is the test's business.
#
# THE ATOMIC ARM traps exactly like the store arm (cause 6) -- LSU.v's
# `ag_misalign` makes no AMO/store/load distinction beyond the cause-vector
# select (cause 6 for store/AMO/SC, cause 4 for load/LR), so there is no
# separate "AMO never gates on MM" claim to make here as there was for
# rv12: NOTHING on rv906 gates on MM.
#
# Both halves run TRANSLATED (MPRV, MPP=S, a 4 KB leaf) and then again
# UNTRANSLATED, because the misalign check is upstream of the MMU answer
# and must not have acquired a dependence on it.

#include "mmu_env.h"
#include "test_macros.h"

#define RV906_MXSTATUS    0x7C0
#define RV906_MXSTATUS_MM (1 << 15)

RVTEST_RV64M
RVTEST_CODE_BEGIN

  MMU_BUILD_L1L2(page_table_1, page_table_2, page_table_3)
  MMU_MAP_4K(page_table_3, 2, datapage, MMU_LEAF_RWX)
  MMU_ON(page_table_1)

  # --- MM = 1 (reset): rv906 traps ANYWAY -- the negative rv12 has no ------
  # --- analog for, because rv12 would SPLIT here and rv906 never does. -----
  MMU_MPRV_S
  li a3, 0x2000

  li TESTNUM, 2
  li a6, CAUSE_MISALIGNED_STORE
  la a7, 2f
  li a0, 0x0123456789abcdef
  sd a0, 3(a3)                    # 8 bytes at VA 0x2003 -- misaligned, traps
  j die
2:

  li TESTNUM, 3
  li a6, CAUSE_MISALIGNED_LOAD
  la a7, 3f
  ld a2, 3(a3)
  j die
3:

  # --- MM = 0: the same two accesses TRAP, exactly as with MM = 1 ----------
  li t0, RV906_MXSTATUS_MM
  csrc RV906_MXSTATUS, t0

  # 4: a misaligned STORE traps cause 6 (store/AMO address misaligned)
  li TESTNUM, 4
  li a6, CAUSE_MISALIGNED_STORE   # what the handler must see
  la a7, 4f                       # where the handler must return to
  sd a0, 3(a3)
  j die                           # the handler skips this by loading a7
4:

  # 5: a misaligned LOAD traps cause 4 (load address misaligned)
  li TESTNUM, 5
  li a6, CAUSE_MISALIGNED_LOAD
  la a7, 5f
  ld a2, 3(a3)
  j die
5:

  # 6: a misaligned ATOMIC traps cause 6 with MM = 0 ...
  li TESTNUM, 6
  li a6, CAUSE_MISALIGNED_STORE
  la a7, 6f
  li a1, 1
  addi a5, a3, 4
  amoadd.d a4, a1, (a5)
  j die
6:

  # 7: ... and STILL traps with MM back ON -- the control that pins contract
  #    3 rather than merely rv64mi-p-ma_addr's boot default
  li t0, RV906_MXSTATUS_MM
  csrs RV906_MXSTATUS, t0
  li TESTNUM, 7
  li a6, CAUSE_MISALIGNED_STORE
  la a7, 7f
  li a1, 1
  addi a5, a3, 4
  amoadd.d a4, a1, (a5)
  j die
7:

  # 8: an ALIGNED store/load still completes normally -- the control that
  #    says the CSR pokes above did not disturb ordinary accesses
  li TESTNUM, 8
  li a0, 0x1122334455667788
  sd a0, 0(a3)
  ld a2, 0(a3)
  bne a2, a0, die

  # --- UNTRANSLATED: the same two traps -------------------------------------
  MMU_MPRV_OFF
  li t0, RV906_MXSTATUS_MM
  csrc RV906_MXSTATUS, t0

  li TESTNUM, 9
  la a3, datapage
  li a6, CAUSE_MISALIGNED_STORE
  la a7, 9f
  sd a0, 3(a3)
  j die
9:
  li TESTNUM, 10
  li a6, CAUSE_MISALIGNED_LOAD
  la a7, 10f
  ld a2, 3(a3)
  j die
10:
  li t0, RV906_MXSTATUS_MM
  csrs RV906_MXSTATUS, t0

  RVTEST_PASS

  TEST_PASSFAIL

  .align 2
  .global mtvec_handler
mtvec_handler:
  # Every trap in this file is expected and names its own cause in a6 and
  # its own landing pad in a7. A cause that is not the expected one fails
  # with the standard encoding, so a WRONG trap is as visible as a missing
  # one.
  csrr t0, mcause
  bne t0, a6, mmu_bad_cause
  # Return through mepc/mret rather than `jr`, so the MPRV arrangement
  # carries on exactly as mmu_env.h's header describes -- a `jr` would leave
  # the hart with MPP=M and every later access would silently stop
  # translating.
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
datapage: .dword 0
  .dword 0

RVTEST_DATA_END
