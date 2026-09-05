/* test/m4/mmu_env.h - the sv39 scaffolding the directed MMU set shares.
 * PORT of rv12/test/m4/mmu_env.h. Macros renamed RV12_* -> RV906_*.
 *
 * *** WHY THERE IS A SHARED HEADER AT ALL. *** Every shape below needs the
 * same instructions before it can say anything: build a three-level sv39
 * table, point satp at it, prime the TLB, and arrange for data accesses to
 * translate. The arithmetic lives here once so a wrong PPN shift is one bug
 * to find instead of ten.
 *
 * THE ENVIRONMENT IS test/m2's p-env (riscv_test.h by path). The v-env's
 * demand pager would sit between every one of these tests and the machine;
 * these shapes are about a SPECIFIC translation event.
 *
 * THE PRIVILEGE ARRANGEMENT IS rv64si-p-dirty's: MPRV with MPP=S makes
 * LOADS AND STORES translate as S-mode while the code keeps FETCHING
 * untranslated in M-mode. A test can build, poke and re-read its own page
 * tables at their physical addresses without mapping the text at all, and a
 * fault lands in the p-env's mtvec_handler with the machine still able to
 * reach everything.
 *
 * A HANDLER RESTORES NOTHING. On a trap from M-mode the hardware writes
 * MPP=M, which makes MPRV inert (it only applies when MPP != M) -- exactly
 * what lets a handler read/write the page tables at their PHYSICAL
 * addresses. `mret` then returns to M-mode, sets MPP=U, and LEAVES MPRV set
 * (the spec clears MPRV only when the restored MPP is not M), so the
 * retried access translates as U-mode instead of S-mode. Every leaf in this
 * set carries PTE_U, so the permission being checked is the same one either
 * way.
 *
 * MSTATUS BIT POSITIONS VERIFIED AGAINST rtl/CSR.v (:639-640's layout
 * comment: "[19]MXR [18]SUM [17]MPRV ... [12:11]MPP") -- these are the
 * standard RISC-V positions encoding.h's MSTATUS_MPRV/MSTATUS_SUM/
 * MSTATUS_MPP already encode, so the macros below transfer from rv12
 * unchanged; rv906's CSR.v honors the same layout (:614-616 strobes
 * sum_f/mxr_f/mprv_f from those exact bit indices).
 */

#ifndef _RV906_M4_MMU_ENV_H
#define _RV906_M4_MMU_ENV_H

#include "riscv_test.h"

/* THE REPORTING CONVENTION FOR EVERY TEST IN THIS SET: an unexpected trap
 * fails with gp = 16 * mcause + TESTNUM, so the tohost word names the cause
 * and the case together (tohost = (gp << 1) | 1). A test that EXPECTS a
 * trap compares mcause itself and does not come here. */
#define MMU_REPORT_CAUSE                                                \
        csrr t0, mcause;                                                \
        slli t0, t0, 4;                                                 \
        add TESTNUM, TESTNUM, t0;

/* l1[0] -> l2, l2[0] -> l3. Clobbers t0, t1. */
#define MMU_BUILD_L1L2(l1, l2, l3)                                      \
        la t0, l2;                                                      \
        srl t0, t0, RISCV_PGSHIFT - PTE_PPN_SHIFT;                      \
        ori t0, t0, PTE_V;                                              \
        sd t0, l1, t1;                                                  \
        la t0, l3;                                                      \
        srl t0, t0, RISCV_PGSHIFT - PTE_PPN_SHIFT;                      \
        ori t0, t0, PTE_V;                                              \
        sd t0, l2, t1;

/* l3[idx] = a 4 KB leaf for the page holding `sym`, with `flags`.
 * `sym` must be RISCV_PGSIZE-aligned. Clobbers t0, t1. */
#define MMU_MAP_4K(l3, idx, sym, flags)                                 \
        la t0, sym;                                                     \
        srl t0, t0, RISCV_PGSHIFT - PTE_PPN_SHIFT;                      \
        li t1, flags;                                                   \
        or t0, t0, t1;                                                  \
        sd t0, l3 + ((idx) * 8), t1;

/* Publish a leaf while the MPRV arrangement is already ON.
 * *** THE PAGE TABLES LIVE AT PHYSICAL ADDRESSES NOTHING MAPS, so a store
 * to one is an M-MODE store or it is a page fault. *** MMU_MAP_4K on its
 * own is correct only BEFORE MMU_MPRV_S; after it, the same store
 * translates as S-mode, the VA 0x8000_xxxx has no leaf, and the test faults
 * where it meant to publish. This drops MPRV around the store and puts it
 * back. Clobbers t0, t1, a1. */
#define MMU_REMAP_4K(l3, idx, sym, flags)                               \
        MMU_MPRV_OFF                                                    \
        MMU_MAP_4K(l3, idx, sym, flags)                                 \
        MMU_MPRV_S

/* satp <- sv39 rooted at `l1`, then an sfence.vma INVALL. rv906's TLB does
 * NOT require this priming (see test/m4/env/v/riscv_test.h part (1): every
 * tlb_vld flop resets to 0, rtl/MMU.v:685-693) -- it is kept here anyway
 * because it is architecturally correct (satp changed, old translations
 * for a stale root must not be reused) and because MMU_REMAP_4K/handler
 * paths below rely on the SAME sfence.vma flavor to retire a changed PTE.
 * Clobbers a0, a1. */
#define MMU_ON(l1)                                                      \
        li a0, (SATP_MODE & ~(SATP_MODE<<1)) * SATP_MODE_SV39;          \
        la a1, l1;                                                      \
        srl a1, a1, RISCV_PGSHIFT;                                      \
        or a1, a1, a0;                                                  \
        csrw satp, a1;                                                  \
        sfence.vma;

/* Data accesses translate as S-mode, with SUM so a U leaf is reachable.
 * Clobbers a1. MPP must be 0 (U) when this runs -- which it is after
 * RVTEST_CODE_BEGIN's own `mret` and after any handler's, because `mret`
 * leaves MPP=U. */
#define MMU_MPRV_S                                                      \
        li a1, ((MSTATUS_MPP & ~(MSTATUS_MPP<<1)) * PRV_S)              \
               | MSTATUS_MPRV | MSTATUS_SUM;                            \
        csrs mstatus, a1;

/* Leave the arrangement: fetch AND data are M-mode again. Clobbers t0. */
#define MMU_MPRV_OFF                                                    \
        li t0, MSTATUS_MPRV;                                            \
        csrc mstatus, t0;

/* The permission words the set uses. A/D are PRESET wherever a test is not
 * about A/D, because rv906's A/D policy is FAULT-BASED (D-M4-1,
 * rtl/MMU.v: `!pte_a` and `ptw_is_store && !pte_d` both raise a page
 * fault, matching rv12's donor-faithful policy): a leaf without them
 * faults on first touch and the test would be measuring the pager it does
 * not have. */
#define MMU_LEAF_RWX (PTE_V | PTE_U | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D)
#define MMU_LEAF_RW  (PTE_V | PTE_U | PTE_R | PTE_W | PTE_A | PTE_D)
#define MMU_LEAF_R   (PTE_V | PTE_U | PTE_R | PTE_A | PTE_D)

#endif /* _RV906_M4_MMU_ENV_H */
