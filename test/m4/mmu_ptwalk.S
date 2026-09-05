# mmu_ptwalk.S - the walk sees a PTE the D-cache is still holding, and the
#                twin: it sees one the D-cache has just EVICTED.
#
# PORT of rv12/test/m4/mmu_ptwalk.S, recomputed for rv906's D-cache geometry.
#
# *** WHY A WALK CAN MISS A PTE AT ALL. *** Software publishes a page table
# entry with an ORDINARY STORE, so the PTE lands wherever an ordinary store
# lands: in the D-cache, dirty, with memory still holding the old word. The
# page-table walker then has to read it. If the walker borrows the load
# pipe's D-cache probe, a hit returns the dirty line and the walk sees the
# store -- case 3 below.
#
# THE EVICTION TWIN IS THE ONE THAT IS EASY TO GET WRONG. Suppose the dirty
# PTE's cache line has just been EVICTED, so the probe misses and the walk's
# read goes to the bus WHILE THE VICTIM WRITEBACK IS STILL IN FLIGHT. Then a
# bus read that overtakes its own writeback returns the PRE-STORE word, and
# the walk installs a stale translation -- silently, because a stale-but-
# valid PTE faults nothing. Case 5/6 force the eviction and then walk.
#
# GEOMETRY, RECOMPUTED FOR RV906 (rv12's numbers do NOT transfer): rv906's
# D-cache is 32 KB, 4-WAY, 64 B lines, 128 sets, index = PA[12:6]
# (rvproc_pkg.sv: DCACHE_WAYS=4, DCACHE_LINE_BYTES=64, DCACHE_SETS=128), so
# the congruent-address stride is SETS*LINE_BYTES = 128*64 = 0x2000 (8 KB) --
# NOT rv12's 32 KB/2-way numbers. Forcing an eviction in a 4-way set needs
# touching more than 4 congruent addresses; the sweep below touches FIVE
# (EVICT_BASE, +1*STEP, +2*STEP, +3*STEP, +4*STEP), which is the minimum
# count that guarantees at least one of the set's original occupants (the
# PTE's own line included) has been evicted regardless of which replacement
# policy chooses the victim -- rtl/DCache.v's own header describes the
# policy as a "small round-robin counter" minimal scheme (way-select is
# supplied by LSU.v as `dc_req_way_sel`; DCache.v itself holds no policy
# state), and 5 touches exceeds the way count under ANY policy, round-robin
# included.
#
# The eviction sweep runs UNTRANSLATED (MPRV off) so it needs no mappings of
# its own and cannot disturb the uTLB state the case under test depends on.

#include "mmu_env.h"
#include "test_macros.h"

# 0x8010_0000 is past this image and inside RAM; congruent addresses are
# EVICT_STEP apart (rv906's D-cache SETS*LINE_BYTES).
#define EVICT_BASE 0x80100000
#define EVICT_STEP 0x2000

RVTEST_RV64M
RVTEST_CODE_BEGIN

  MMU_BUILD_L1L2(page_table_1, page_table_2, page_table_3)
  MMU_MAP_4K(page_table_3, 2, page_a, MMU_LEAF_RWX)
  MMU_ON(page_table_1)
  MMU_MPRV_S

  li a3, 0x2000

  # 2: the baseline -- the leaf published before satp, walked normally
  li TESTNUM, 2
  ld a2, 0(a3)
  li t2, 0xAAAA
  bne a2, t2, die

  # 3: republish the leaf with an ordinary store, WITHOUT touching memory in
  #    between, and walk it. The store is in the D-cache dirty; the walk
  #    must see it.
  li TESTNUM, 3
  MMU_REMAP_4K(page_table_3, 2, page_b, MMU_LEAF_RWX)
  sfence.vma
  ld a2, 0(a3)
  li t2, 0xBBBB
  bne a2, t2, die

  # 4: and the store really was a cache hit rather than a write-through: read
  #    the PTE back through an ordinary load and it must be the new one.
  li TESTNUM, 4
  MMU_MPRV_OFF
  ld a2, page_table_3 + 16
  la t0, page_b
  srl t0, t0, RISCV_PGSHIFT - PTE_PPN_SHIFT
  li t1, MMU_LEAF_RWX
  or t0, t0, t1
  bne a2, t0, die

  # 5: republish the leaf, EVICT its line with a 5-address sweep, then walk.
  li TESTNUM, 5
  MMU_MAP_4K(page_table_3, 2, page_c, MMU_LEAF_RWX)
  li t2, EVICT_BASE
  li t3, EVICT_STEP
  ld t4, 0(t2)
  add t2, t2, t3
  ld t4, 0(t2)
  add t2, t2, t3
  ld t4, 0(t2)
  add t2, t2, t3
  ld t4, 0(t2)
  add t2, t2, t3
  ld t4, 0(t2)
  MMU_MPRV_S
  sfence.vma
  ld a2, 0(a3)
  li t2, 0xCCCC
  bne a2, t2, die

  # 6: the same again with the eviction sweep STORING rather than loading, so
  #    the victim writeback the walk has to beat is one this test created
  li TESTNUM, 6
  MMU_MPRV_OFF
  MMU_MAP_4K(page_table_3, 2, page_d, MMU_LEAF_RWX)
  li t2, EVICT_BASE
  li t3, EVICT_STEP
  sd t2, 0(t2)
  add t2, t2, t3
  sd t2, 0(t2)
  add t2, t2, t3
  sd t2, 0(t2)
  add t2, t2, t3
  sd t2, 0(t2)
  add t2, t2, t3
  sd t2, 0(t2)
  MMU_MPRV_S
  sfence.vma
  ld a2, 0(a3)
  li t2, 0xDDDD
  bne a2, t2, die

  MMU_MPRV_OFF
  RVTEST_PASS

  TEST_PASSFAIL

  .align 2
  .global mtvec_handler
mtvec_handler:
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
page_a: .dword 0xAAAA
.align 12
page_b: .dword 0xBBBB
.align 12
page_c: .dword 0xCCCC
.align 12
page_d: .dword 0xDDDD

RVTEST_DATA_END
