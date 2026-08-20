/* macros.h - assembler helpers shared by the M1 directed fetch tests
 * (plan Task 5.3; reused unmodified by Task 6's directed suite)
 *
 * The tests are preprocessed (.S, capital S), so these are plain cpp macros.
 * File-structure precedent: rv12's own test/m1/macros.h (style reference
 * only, per the plan -- the CONTENT below is this repo's own contract, not
 * copied logic).
 *
 * CONTRACT REMINDERS (docs/superpowers/specs/2026-08-20-m1-ifu-design.md S4.1
 * and the plan's "Global contracts"), because the .S files have to agree
 * with BOTH independently-implemented oracles (rtl/FetchSink.v and the C++
 * fetch-ISS in m1_iss.h):
 *   - a conditional branch is TAKEN iff ^pc[7:4] (parity of the four
 *     byte-address bits 7..4 of the BRANCH's own PC) -- the actual register
 *     operands are decorative: M1 has no execute stage, so nothing reads
 *     them;
 *   - an indirect jump that is not classified as a return lands at
 *     JR_TARGET(pc) = (pc & ~0x3F) + 0x40 + (((pc >> 6) & 3) << 6);
 *   - a call (dst register == x1 -- rv906/C906 is x1-ONLY, confirmed from
 *     aq_iu_bju.v: no C910-style x5 alternate link register) pushes its
 *     fall-through on the shadow stack; a return (jalr-family, src reg ==
 *     x1, and NOT also dst == x1) pops it; a return on an empty stack falls
 *     through.
 */

#ifndef M1_MACROS_H
#define M1_MACROS_H

/* The address the linker script places .text.init at (= FetchSink.v's
 * RESET_VECTOR default / rvproc_pkg.sv's cp0_xx_mrvbr). */
#define M1_TEXT_BASE 0x80000000

/* The byte `.org` pads with. 0x0101 as a halfword is `c.addi x2, x2, 0`: a
 * 16-bit NON-control instruction, so a fill region that does get executed
 * (a bug, since every pad here is meant to be reached only via an indirect
 * jump landing exactly on it) produces a defined, boring committed entry in
 * both oracles instead of the all-zero illegal RVC encoding. */
#define M1_FILL 0x01

/* JR_TARGET as a section-relative offset. Base-invariant here because
 * M1_TEXT_BASE is 256-byte aligned, so applying the formula to an offset and
 * adding the base gives the same answer as applying it to the full address. */
#define JR_TARGET_OFF(off) \
    ((((off) & ~0x3F) + 0x40) + ((((off) >> 6) & 3) << 6))

/* Start of a test: opens .text.init and defines the base symbol JR_PAD needs.
 * Use once, at the top of the file, before any code. */
#define M1_TEXT_START \
    .section .text.init, "ax", @progbits; \
    .globl _start; \
_m1_text_base:; \
_start:

/* SENTINEL: the `jal x0, 0` self-loop that ends every M1 test (spec S4.1,
 * FetchSink.v's SENTINEL localparam 32'h0000_006F). `.option norvc` is
 * mandatory -- with RVC enabled the assembler would compress this to
 * `c.j 0` (0xa001), which is NOT the encoding the harness and FetchSink
 * scan for. */
#define SENTINEL \
    .option push; \
    .option norvc; \
900: jal x0, 900b; \
    .option pop

/* JR_PAD(jmp_label, pad_label): open the landing pad for the indirect jump
 * at `jmp_label`, i.e. place `pad_label` at JR_TARGET(jmp_label).
 *
 * Usage rules (both are `.org` restrictions, and both fail loudly):
 *   - `jmp_label` must already be defined, so the pad is emitted AFTER the
 *     jump it serves;
 *   - pads must be emitted in increasing address order, because `.org` can
 *     only move the location counter forward.
 * The pad is 64 to 256 bytes past the jump, so at most four jumps in the
 * same 64-byte line can be served before the pads start colliding -- put
 * one indirect jump per 64-byte line unless the collision is intentional,
 * and give each site's continuation code room to hop OVER the pad's `.org`
 * gap (that gap fills with M1_FILL and is otherwise dead code) rather than
 * falling into it.
 *
 *   jr_site:  c.jr x6
 *             ...
 *             JR_PAD(jr_site, jr_site_pad)
 *             c.nop
 *             ...
 */
#define JR_PAD(jmp_label, pad_label) \
    .org JR_TARGET_OFF((jmp_label) - _m1_text_base), M1_FILL; \
pad_label:

/* PAD_AT(off, label): place `label` at a literal section-relative offset. */
#define PAD_AT(off, label) \
    .org (off), M1_FILL; \
label:

/* The tohost/fromhost line the testbench looks up by ELF symbol and
 * FetchSink writes its report to. Emit once per test, after the code. */
#define M1_TOHOST_DATA \
    .section .tohost, "aw", @progbits; \
    .globl tohost; \
tohost: .dword 0; \
    .globl fromhost; \
fromhost: .dword 0

#endif /* M1_MACROS_H */
