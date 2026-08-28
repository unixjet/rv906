/* macros.h - assembler helpers for M2's hand-written directed ELFs (plan 8.5)
 *
 * A FRESH helper set for the Task 9 directed tests (alu_seq.S, bju_seq.S,
 * muldiv_seq.S, ld_st_uncached.S, ld_st_cached.S, csr_trap.S). It provides
 * just enough to write a self-contained test that (a) starts at the reset
 * vector, (b) optionally turns the caches on, (c) checks values against
 * hand-computed constants, and (d) reports PASS/FAIL through tohost -- the
 * same tohost protocol the riscv-tests use and that testbench/TestBench.cpp
 * already polls (1 = PASS, 2*n+1 = FAIL of case n).
 *
 * DELIBERATELY NOT carried over from test/m1/macros.h (contract 15): the
 * synthetic-oracle machinery -- JR_TARGET / the shadow call stack / the
 * ^pc[7:4] conditional-branch rule / the SENTINEL jal -- encoded M1's fake
 * BJU rules and has no meaning against a real pipeline and a real reference
 * model. A branch here is taken on its REAL operands, a jump lands where its
 * operand says, and a test ends by STORE-ING to tohost (not by a sentinel
 * the fetch oracle scans for).
 *
 * Conventions (pinned by this repo's RTL, not by rv12):
 *   .text.init @ M2_TEXT_BASE (0x8000_0000), the reset vector;
 *   tohost @ M2_TOHOST (0x7FFF_F000), the uncached aperture (see common.ld).
 */

#ifndef M2_MACROS_H
#define M2_MACROS_H

/* The reset vector the core boots from (common.ld's .text.init /
 * rvproc_pkg.sv cp0_xx_mrvbr). */
#define M2_TEXT_BASE 0x80000000

/* The uncached tohost address (common.ld's .tohost, contract 6). */
#define M2_TOHOST 0x7FFF_F000

/* T-Head machine-control register (rvproc_pkg.sv CSR_MHCR, confirmed
 * aq_cp0_regs.v:847). bit0 = ie (ICache), bit1 = de (DCache), bit2 = wa. */
#define M2_MHCR 0x7C1

/* Open the test's .text.init and define the reset vector. Use once, at the
 * very top of the file, before any code. */
#define M2_TEXT_START \
    .section .text.init, "ax", @progbits; \
    .globl _start; \
_start:

/* Turn ICache+DCache on (MHCR ie|de = 0x3, wa left 0 per contract 6). Must
 * run in M-mode -- which is where every directed test starts (the core resets
 * in M-mode and these tests never drop privilege). Skip for tests that do no
 * memory access (caches are irrelevant to a pure ALU/branch stream). */
#define M2_ENABLE_CACHES \
    li a0, 0x3; \
    csrw M2_MHCR, a0

/* Store `val` to tohost (uncached, reaches ExtMem straight) and spin. The
 * harness reads tohost, breaks on the low bit set, and reports
 * 1 = PASS / (2*n+1) = FAIL of case n. */
.macro m2_report val
    li t6, (\val);
    sw t6, tohost;
1:  j  1b;
.mend

.macro m2_pass
    m2_report 1;
.mend

.macro m2_fail n
    m2_report (2 * (\n) + 1);
.mend

/* Emit the tohost/fromhost data symbols in the uncached aperture (common.ld
 * maps .tohost to M2_TOHOST). Use once, after all code. tohost and fromhost
 * are 64 B apart (.align 6) so the 16-byte AXI beat the memory model writes
 * for the tohost store cannot clobber fromhost. */
#define M2_DATA \
    .section .tohost; \
    .align 6; \
    .globl tohost; \
tohost: .quad 0; \
    .align 6; \
    .globl fromhost; \
fromhost: .quad 0; \
    .quad 0

#endif /* M2_MACROS_H */
