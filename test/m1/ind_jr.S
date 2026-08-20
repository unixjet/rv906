/* ind_jr.S - indirect-jump tables through non-link registers
 *
 * M1 spec S4.2's "indirect-jump tables (jr through a non-link register)".
 * With every predictor off (rung 1) an indirect transfer gets NO front-end
 * prediction at all: it resolves purely through FetchSink's fake BJU at
 * commit time, exactly the mechanism Task 9's indirect BTB will later race
 * against (this file is authored now and re-run unmodified then).
 *
 * THE CONTRACT BEING EXERCISED (spec S4.1 / macros.h's header)
 * -----------------------------------------------------------------
 * An indirect jump that is neither a call nor a return lands at
 *   JR_TARGET(pc) = (pc & ~0x3F) + 0x40 + (((pc >> 6) & 3) << 6)
 * -- a 64-byte-aligned pad 64 to 256 bytes ahead, the distance chosen by
 * pc[7:6]. Both FetchSink.v and the C++ ISS implement that formula
 * independently, and JR_PAD places a landing pad exactly there, so a
 * one-bit error in either implementation shows up as a committed-stream
 * divergence on the very first jump. All four distances ((pc>>6)&3 = 0..3)
 * are covered below.
 *
 * An INDIRECT CALL (dst==x1, src NOT x1) takes JR_TARGET as its target AND
 * pushes its fall-through, so every such site here is paired with a return
 * inside its own landing pad -- the shadow stack is empty again by the end
 * of each slot (callret.S is the file that actually stresses depth/nesting).
 *
 * LAYOUT RULE. One indirect jump per 256-byte slot, at an offset whose
 * (pc >> 6) & 3 is known, so JR_PAD's `.org` addresses stay monotonic and no
 * two pads collide (macros.h's usage rules). Gaps are M1_FILL, never
 * executed.
 */

#include "macros.h"

    M1_TEXT_START

    .option push
    .option norvc
    jal   x0, site1
    .option pop

    /*--- slot 0x200: c.jr through x6 -- the canonical "jr table" jump,
     *    (pc>>6)&3 == 0, pad 64 bytes ahead ------------------------------*/
    PAD_AT(0x200, site1)
    .option push
    .option rvc
    c.jr  x6
    .option pop
    JR_PAD(site1, pad1)                 /* 0x240 */
    .option push
    .option norvc
    jal   x0, site2
    .option pop

    /*--- slot 0x300: the 32-bit form, jalr x0, 0(x6) ------------------------*/
    PAD_AT(0x300, site2)
    .option push
    .option norvc
    jalr  x0, 0(x6)
    .option pop
    JR_PAD(site2, pad2)                 /* 0x340 */
    .option push
    .option norvc
    jal   x0, site3
    .option pop

    /*--- slot 0x400: rd is a real but NON-LINK register -> still ind_br,
     *    and specifically NOT a call, so nothing is pushed ------------------*/
    PAD_AT(0x400, site3)
    .option push
    .option norvc
    jalr  x7, 0(x6)
    .option pop
    JR_PAD(site3, pad3)                 /* 0x440 */
    .option push
    .option norvc
    jal   x0, site4
    .option pop

    /*--- slot 0x500: c.jr through x7, (pc>>6)&3 == 1, pad 128 bytes ahead --*/
    PAD_AT(0x540, site4)
    .option push
    .option rvc
    c.jr  x7
    .option pop
    JR_PAD(site4, pad4)                 /* 0x5C0 */
    .option push
    .option norvc
    jal   x0, site5
    .option pop

    /*--- slot 0x600: the jump itself STRADDLES the 4-byte ICache fetch
     *    group (offset 2 mod 4), so the entry0 carry register holds an
     *    indirect jump's own opcode across the boundary -----------------*/
    PAD_AT(0x60E, site5)
    .option push
    .option norvc
    jalr  x0, 0(x6)
    .option pop
    JR_PAD(site5, pad5)                 /* 0x640 */
    .option push
    .option norvc
    jal   x0, site6
    .option pop

    /*--- slot 0x700: "return-shaped" through x2 -- a non-link register, so
     *    it is plain ind_br and the shadow stack is NOT touched ------------*/
    PAD_AT(0x700, site6)
    .option push
    .option rvc
    c.jr  x2
    .option pop
    JR_PAD(site6, pad6)                 /* 0x740 */
    .option push
    .option norvc
    jal   x0, site7
    .option pop

    /*--- slot 0x880: INDIRECT CALL, (pc>>6)&3 == 2, pad 192 bytes ahead ----*/
    PAD_AT(0x880, site7)
    .option push
    .option norvc
    jalr  x1, 0(x6)                     /* pcall + ind_br: push, -> 0x940 */
site7_after:
    jal   x0, site8                     /* reached by the pad's return */
    .option pop
    JR_PAD(site7, pad7)                 /* 0x940 */
    .option push
    .option norvc
    jalr  x0, 0(x1)                     /* preturn: pops site7_after */
    .option pop

    /*--- slot 0x9C0: c.jalr through x6's shadow (rd forced x1 by the
     *    encoding) -- a call, and (pc>>6)&3 == 3, pad 256 bytes ahead ------*/
    PAD_AT(0x9C0, site8)
    .option push
    .option norvc
    jalr  x1, 0(x7)                     /* pcall + ind_br: push, -> 0xAC0 */
site8_after:
    jal   x0, site9
    .option pop
    JR_PAD(site8, pad8)                 /* 0xAC0 */
    .option push
    .option norvc
    jalr  x0, 0(x1)                     /* preturn: pops site8_after */
    .option pop

    /*--- slot 0xB00: rd == rs1 == x6. Neither is a link register, so this
     *    is plain ind_br: no push, no pop -----------------------------------*/
    PAD_AT(0xB00, site9)
    .option push
    .option norvc
    jalr  x6, 0(x6)
    .option pop
    JR_PAD(site9, pad9)                 /* 0xB40 */
    .option push
    .option norvc
    jal   x0, site10
    .option pop

    /*--- slot 0xC00: "return-shaped" via x1 as SOURCE but with a NON-x1
     *    destination that's also not x0 -- per C906's own rule (rtl/
     *    FetchSink.v TASK 4.1 FINDINGS item 2 / callret.S's header) this
     *    IS classified preturn regardless of rd/imm, unlike C910 -- included
     *    here too since a "jr table" full of ind_br entries is exactly where
     *    a wrongly-guessed ABI-shaped classifier would misfire. -----------*/
    PAD_AT(0xC00, site10)
    .option push
    .option norvc
    jal   x1, site10_call        /* push site10_after */
site10_after:
    jal   x0, site11
    .option pop

    PAD_AT(0xC40, site10_call)
    .option push
    .option norvc
    jalr  x9, 0(x1)               /* preturn (src=x1, dst=x9!=x1): pops
                                      site10_after, despite the non-x0/x1 rd */
    .option pop

    /*--- slot 0xD00: "return-shaped" register, NON-ZERO immediate, still a
     *    preturn on C906 (imm is not part of the classification) ----------*/
    PAD_AT(0xD00, site11)
    .option push
    .option norvc
    jal   x1, site11_call        /* push site11_after */
site11_after:
    jal   x0, done
    .option pop

    PAD_AT(0xD40, site11_call)
    .option push
    .option norvc
    jalr  x0, 8(x1)               /* preturn despite imm=8: pops site11_after */
    .option pop

    PAD_AT(0xE00, done)

    SENTINEL

    M1_TOHOST_DATA
