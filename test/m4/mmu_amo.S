# mmu_amo.S - an AMO through translation needs BOTH R and W, and it is a
#             STORE for the fault-based A/D pager (sets D, not just A).
#
# ADAPTED from rv12/test/m4/mmu_amo.S -- NOT a straight port. rv12's file
# exists to regress AM-5, a bug specific to that donor's TWO-LOOKUP AMO
# (a separate uTLB read for the load half and the store half, needing
# `lsu_mmu_stamo_vld` to force the second lookup's physical address from the
# first instead of re-walking and risking a spurious cause-15). rv906's AMO
# does ONE translation, reused for both halves:
#
#   rtl/LSU.v:881  `wire issue_real = ag_issue_ready && mmu_lsu_pa_vld;` --
#                  ONE MMU request per instruction, at AG/issue.
#   rtl/LSU.v:1155-1212  the AMO's read-modify-write commit (SECTION AMO)
#                  runs entirely off the physical address issue_real already
#                  latched; there is no second `lsu_mmu_va_vld` for the
#                  write-back half to diverge from.
#
# So AM-5's bug class -- two lookups that can disagree -- cannot occur here,
# and this file is not a port of that regression.
#
# WHAT REPLACES IT: reading rtl/MMU.v's/LSU.v's is_load/is_store
# classification for that ONE lookup surfaced a real question worth a
# directed test. The MMU's permission and A/D checks are driven off a single
# wire:
#
#   rtl/LSU.v:371  `wire ag_is_store = idu_lsu_ex1_func[0];`
#   rtl/LSU.v:418  `assign lsu_mmu_st_inst = ag_is_store;`
#   rtl/IDU.v:523  AMO's func literal: `{8'h01, 3'b000, funct5, width, 2'b00}`
#                  -- func[0] IS ZERO for every AMO.
#   rtl/MMU.v:506-507  `ptw_is_load <= !lsu_mmu_st_inst; ptw_is_store <=
#                  lsu_mmu_st_inst;`
#
# Chased through, an AMO's `lsu_mmu_st_inst` is 0, so the walker classifies
# every AMO as a LOAD, not a store, for its own permission math:
#
#   rtl/MMU.v:583-589  `(ptw_is_load && !pte_r ...) || (ptw_is_store &&
#                  !pte_w) || ... || (ptw_is_store && !pte_d)`
#
# An AMO only ever tests the `ptw_is_load` arm (needs R) and NEVER the
# `ptw_is_store` arm (needs W, needs D) by this reading -- which, if it holds
# at simulation, means an amoadd through a leaf that grants R+X but not W, or
# a leaf with D=0, would NOT page-fault the way the RISC-V privileged spec
# requires (an AMO is a read-modify-WRITE and needs W and dirties the page).
# Cases 3-4 below are written to the SPEC-CORRECT expectation on purpose:
# should rv906 not fault there, that is the finding this file exists to
# report, not something this file works around.

#include "mmu_env.h"
#include "test_macros.h"

RVTEST_RV64M
RVTEST_CODE_BEGIN

  MMU_BUILD_L1L2(page_table_1, page_table_2, page_table_3)
  # VA 0x2000: full RWX, A and D both already set -- the baseline.
  MMU_MAP_4K(page_table_3, 2, page_full, PTE_V | PTE_U | PTE_R | PTE_W \
                                          | PTE_X | PTE_A | PTE_D)
  # VA 0x3000: R + X but NO W, A and D set -- the write-permission probe.
  MMU_MAP_4K(page_table_3, 3, page_rx, PTE_V | PTE_U | PTE_R | PTE_X \
                                        | PTE_A | PTE_D)
  # VA 0x4000: full RWX, A set but D CLEAR -- the dirty-bit probe.
  MMU_MAP_4K(page_table_3, 4, page_noD, PTE_V | PTE_U | PTE_R | PTE_W \
                                         | PTE_X | PTE_A)
  MMU_ON(page_table_1)
  MMU_MPRV_S

  # 2: the baseline -- amoadd.d through a fully-permissioned, A/D-set leaf
  li TESTNUM, 2
  li a3, 0x2000
  li a1, 5
  amoadd.d a4, a1, (a3)
  li t2, 0x1111
  bne a4, t2, die                  # AMO returns the OLD value
  ld a2, 0(a3)
  li t2, 0x1116                    # 0x1111 + 5
  bne a2, t2, die

  # 3: THE ROW -- amoadd.d through a leaf with NO W. The privileged spec
  #    requires an AMO to fault here exactly as a plain store would (cause
  #    15, store/AMO page fault): a read-modify-write with no write
  #    permission is not a partial success.
  li TESTNUM, 3
  li a6, CAUSE_STORE_PAGE_FAULT
  la a7, 3f
  li a3, 0x3000
  li a1, 1
  amoadd.d a4, a1, (a3)
  j die
3:

  # 4: amoadd.d through a leaf with A set but D CLEAR. D-M4-1's fault-based
  #    dirty policy (`ptw_is_store && !pte_d`) must see the AMO as the store
  #    it is and fault, exactly as a plain store would on the same leaf --
  #    this file asserts the fault fires; the fault-then-retry-succeeds shape
  #    is already covered by mmu_ldst_samepage.S's handler, so this case
  #    stays a denial-pattern check like case 3 rather than duplicating it.
  li TESTNUM, 4
  li a6, CAUSE_STORE_PAGE_FAULT
  la a7, 4f
  li a3, 0x4000
  li a1, 7
  amoadd.d a4, a1, (a3)
  j die
4:

  MMU_MPRV_OFF
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
page_full: .dword 0x1111
.align 12
page_rx: .dword 0x9999
.align 12
page_noD: .dword 0x2222

RVTEST_DATA_END
