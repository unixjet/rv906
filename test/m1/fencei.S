/* fencei.S - fence.i / self-modifying code via the host-patch mechanism
 *
 * M1 spec S4.2's "fence.i after host-patched code". M1 has no store unit and
 * cannot execute fence.i itself (no LSU, no CSR/CP0 write path exists before
 * M2) -- per the plan's Task 6.1 note, BOTH halves of a self-modifying-code
 * test are therefore the HOST's job, driven by RVProcTest.cpp's
 * `--fencei-patch=<addr>:<word32>:<commit>` flag (see that file's header
 * comment and `TB::fencei_apply()`): at commit boundary <commit> (a count of
 * instructions already COMPARED by the online checker, not a cycle -- so it
 * is identical regardless of stalls/refills/--sink-stall) the host (a)
 * writes <word32> into the ELF image in ExtMem at <addr>, and (b) pulses
 * ICache.v's own `cfg_icache_inv`/`cp0_ifu_icache_inv_req` for one cycle --
 * the same mechanism rv12 settled on for its own C910 clone, since it is the
 * one event both independently-written oracles (rtl/FetchSink.v's committed-
 * stream export and the C++ ISS's lazy `rd_half()`, m1_iss.h "LAZINESS") can
 * name identically: "after N instructions have committed", not "after cycle
 * C" or "after the RTL's own internal refill state reaches some value".
 *
 * PROGRAM SHAPE
 * ----------------------------------------------------------------------
 * `patch_site` starts as a single 32-bit NOP (`addi x0,x0,0`) -- a full
 * 4-byte-aligned instruction slot, chosen deliberately so the host's
 * always-32-bit-wide `--fencei-patch` write replaces exactly one instruction
 * and never corrupts a neighbour (a 2-byte RVC slot would have its adjacent
 * halfword clobbered by a 4-byte write). The program fetches straight
 * through `patch_site` ONCE while it still reads as a NOP -- this is what
 * gets the containing 64-byte cache line allocated into the ICache with the
 * OLD encoding, which is the entire point: without a prior real visit, the
 * line was never cached and re-reading it after the patch would just look
 * like an ordinary cold miss, never actually exercising the INVALIDATE path
 * at all. FILLER then runs long enough (more than the 6-halfword IBUF depth
 * plus the pipeline's own few stages, per rtl/FetchSink.v TASK 4.1 FINDINGS
 * item 3's squash-window discussion) that nothing from that line can still
 * be buffered anywhere in the front end by the time the second visit is
 * attempted. At the exact commit boundary computed below (right after the
 * FILLER's last instruction, before `patch_site` is fetched a second time),
 * the host patches `patch_site` from the NOP into `jal x0, patch_done` and
 * pulses the invalidate -- so the SECOND visit, forced to refill by the
 * invalidate, reads the NEW bytes and jumps to `patch_done`, never falling
 * back into FILLER (which would otherwise re-run it and hang). If the patch
 * did NOT actually take effect (stale NOP still served), execution falls
 * through into `patch_unreached` instead -- a distinctly different, visibly
 * diverging committed stream (on top of the harness's own `--fencei-patch`
 * "never applied" check, RVProcTest.cpp's `fencei_fired` flag).
 *
 * COMMIT-COUNT BOOK-KEEPING. Nothing before `second_entry`'s `jal` is a
 * branch, so the instruction count from `_start` through that `jal`
 * (inclusive) is exact and unaffected by anything RTL-timing-dependent --
 * confirmed against `test/m1/fencei.out`'s own disassembly (`make -C test/m1
 * dis T=fencei`) and cross-checked against the checker's own "N instructions
 * compared" report, not hand-counted alone. run_all.sh passes the resulting
 * `--fencei-patch=<addr of patch_site>:<jal encoding>:<commit count>` triple.
 */

#include "macros.h"

    M1_TEXT_START

    /* Preamble is a single 4-byte instruction (not RVC) so `patch_site`
     * below lands 4-byte aligned -- required by RVProcTest.cpp's own
     * `--fencei-patch` parser (host writes are always a 4-byte-aligned
     * 32-bit word). */
    .option push
    .option norvc
    addi  x1, x1, 1
    .option pop

    /* patch_site: starts life as a plain 32-bit NOP. Forced non-RVC (a full
     * 4-byte instruction) so the host's word-wide patch can never clobber a
     * neighbouring halfword. */
    .option push
    .option norvc
patch_site:
    addi  x0, x0, 0
    .option pop

    /* FILLER: 24 32-bit instructions (96 bytes = 48 halfwords), comfortably
     * more than the 6-halfword IBUF plus a few pipeline stages of residue,
     * so nothing from `patch_site`'s cache line can still be in flight by
     * the time execution reaches `second_entry` below. */
    .option push
    .option norvc
    .rept 24
    addi  x2, x2, 1
    .endr
    .option pop

second_entry:
    /* Re-fetch patch_site. On an unpatched image this falls through (the
     * NOP re-executes) into patch_unreached below -- a live, visible
     * failure signature distinct from the harness's own "never applied"
     * check. On the patched image, `patch_site` is now `jal x0,
     * patch_done` and control leaves straight for `patch_done`. */
    jal   x0, patch_site

patch_unreached:
    .option push
    .option rvc
    c.nop
    .option pop

    PAD_AT(0x200, patch_done)
    .option push
    .option rvc
    c.li  x3, 9
    .option pop

    SENTINEL

    M1_TOHOST_DATA
