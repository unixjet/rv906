/* seq.S - straight-line fetch: RVC and 32-bit instructions, no control flow
 *
 * M1 spec S4.2's first directed test ("seq (straight-line)"). NOTE: this is
 * NOT the same file as test/m1/iss_selftest.S -- that file (Task 5.3) is a
 * dense mix of every branch/call/return/indirect-jump corner case, built to
 * be the golden ISS's own hand-derived-expected-stream self-test. This file
 * has NO control transfer at all until the sentinel, so the committed stream
 * is simply every instruction in address order: it isolates the fetch
 * pipeline, the ICache refill path, and IPACK's live RVC-boundary/straddle
 * logic (IFU notes S4.2 -- entry0's carry register, no precomputed predecode
 * array the way C910 has) from every redirect path. If seq fails, nothing
 * else in this suite is worth debugging.
 *
 * The 20-byte repeated block is deliberately not a multiple of 4, so the
 * blocks walk through every alignment phase of the ICache's 4-byte (2
 * halfword) per-cycle fetch group as well as the 64-byte cache-line
 * boundary; its 32-bit instructions straddle both boundaries as the pattern
 * rotates through the block. 45 repeats = 315 instructions + preamble +
 * sentinel -- comfortably more than one full 32KB-cache-line's worth of
 * misses/refills at this instruction density.
 *
 * Nothing here may be a branch, jump, call or return: `.option norvc` blocks
 * also stop the assembler from compressing the 32-bit instructions, which is
 * what keeps the length mix under this file's control rather than the
 * assembler's.
 */

#include "macros.h"

    M1_TEXT_START

    /* Preamble: an unaligned 32-bit instruction right at the reset vector,
     * so the very first fetch group already mixes lengths. */
    .option push
    .option rvc
    c.nop                       /* 0x80000000, 2 bytes */
    .option norvc
    addi  x1, x1, 1             /* 0x80000002, straddles the 4-byte boundary */
    .option rvc
    c.li  x10, 0
    .option norvc
    lui   x11, 0x1
    .option pop

    .rept 45
    .option push
    .option norvc
    addi  x1, x1, 1             /* 4 */
    .option rvc
    c.addi x2, 1                /* 2 */
    .option norvc
    lui   x3, 0x1               /* 4 */
    .option rvc
    c.nop                       /* 2 */
    c.mv  x4, x5                /* 2 */
    .option norvc
    ori   x6, x6, 3             /* 4 */
    .option rvc
    c.li  x7, 3                 /* 2 */
    .option pop
    .endr

    SENTINEL

    M1_TOHOST_DATA
