/* thrash.S - ICache thrash: footprint > 32KB, hammering across the cache's
 * own 256 sets and the BTB's future 64KiB alias period
 *
 * M1 spec S4.2's "icache thrash (>32KB code footprint through the BTB's
 * 64KiB alias period and the ICache's own sets)". rv906's ICache is 32KB,
 * 2-way, 256 sets, 64-byte lines (IFU notes S3): set index = addr[13:6], so
 * two addresses exactly 16KB apart (2^14) alias to the SAME set. This file
 * is authored at rung 1 (no real BTB exists yet -- Task 8/9's job), but its
 * SIZE is chosen now so it is already BTB-alias-relevant once that lands:
 * a 96KB total footprint is 6x the ICache's 16KB same-set-alias period and
 * 1.5x the BTB's own 64KiB alias period, so this file is re-run unmodified
 * at every later rung.
 *
 * TWO KINDS OF PRESSURE
 * ----------------------------------------------------------------------
 *  1. Region A: >32KB of pure straight-line code (no control flow at all).
 *     A single forward pass over N*16KB touches every one of the 256 sets N
 *     times each (once per 16KB "wrap"); with 2 ways, the FIRST TWO passes
 *     over a given set fill both ways for free, but every pass after that
 *     forces the FIFO replacement policy (icache.v's `icache_refill_fifo`
 *     bit, ICache.v SECTION ARRAYS -- rv906's own extraction-note-cited
 *     refill/FIFO logic) to actually evict something. 3 full 16KB passes
 *     (48KB) guarantees at least one forced eviction on every set.
 *  2. Region B: a DELIBERATE same-set hopscotch. M1 has no execute stage, so
 *     revisiting an address means a real control-flow cycle (an infinite
 *     loop) -- the escape, same as jal_chain.S/dense_br.S, is to visit
 *     EXPLICIT INSTANCES of the same set at monotonically increasing
 *     addresses (16KB apart) via forward-only jumps, rather than literally
 *     jumping backward to the same code. Six blocks, each a full 16KB past
 *     the last, forces FIVE consecutive evictions of the SAME set (16KB is
 *     also comfortably a multiple of the BTB's own future alias period).
 *
 * Nothing here is a control transfer except region B's forward hop chain
 * and the sentinel, so region A's expected committed stream is pure address
 * order and region B's is the six block entry points in sequence -- a
 * divergence can only mean a refill/replacement corrupted or dropped data.
 */

#include "macros.h"

    M1_TEXT_START

    /*---------------------------------------------------------------------
     * Region A: 3200 x 16-byte blocks = 51200 bytes (50KB), all 32-bit
     * instructions (kept simple and RVC-free so the footprint arithmetic
     * above is exact) -- more than 3 full 16KB same-set-alias passes.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    .rept 3200
    addi  x5, x5, 1
    addi  x6, x6, 1
    addi  x7, x7, 1
    addi  x8, x8, 1
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * Region B: the same-set hopscotch. PAD_AT to a clean 16KB-aligned
     * start past region A, then six blocks each exactly 0x4000 (16KB) apart
     * -- all six alias to the SAME ICache set. Each block does a little
     * real work then jumps forward to the next; the last escapes to the
     * sentinel. Entered at the FIRST (lowest) block, unlike the backward
     * gadgets elsewhere in this suite, since every hop here is forward.
     *--------------------------------------------------------------------*/
    PAD_AT(0xE000, hop0)
    .option push
    .option norvc
    addi  x9, x9, 1
    jal   x0, hop1
    .option pop

    PAD_AT(0x12000, hop1)
    .option push
    .option norvc
    addi  x9, x9, 2
    jal   x0, hop2
    .option pop

    PAD_AT(0x16000, hop2)
    .option push
    .option norvc
    addi  x9, x9, 3
    jal   x0, hop3
    .option pop

    PAD_AT(0x1A000, hop3)
    .option push
    .option norvc
    addi  x9, x9, 4
    jal   x0, hop4
    .option pop

    PAD_AT(0x1E000, hop4)
    .option push
    .option norvc
    addi  x9, x9, 5
    jal   x0, hop5
    .option pop

    PAD_AT(0x22000, hop5)
    .option push
    .option norvc
    addi  x9, x9, 6
    jal   x0, done
    .option pop

    PAD_AT(0x26000, done)

    SENTINEL

    M1_TOHOST_DATA
