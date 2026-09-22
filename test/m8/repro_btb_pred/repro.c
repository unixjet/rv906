/* M8 T4c directed repro: BPU predicted-taken-branch, valid+correct BTB hit.
 *
 * BUG (Class-B rv906 RTL, fixed at rtl/BPU.v:824):
 *   The donor C906 IFU redirects a taken branch that has a valid BTB entry
 *   through the BTB's OWN PCGEN-stage channel -- `btb_xx_chgflw_vld` /
 *   `btb_pcgen_tar_pc` (refs/openc906/.../ifu/rtl/aq_ifu_btb.v:71-74,740-747)
 *   consumed at aq_ifu_pcgen.v:279-282 -- one fetch stage BEFORE the branch
 *   reaches the ID stage, so the wrong-path fall-through is never fetched.
 *   rv906 collapses the donor's 2-stage BTB CAM into the ID stage (see the
 *   BPU.v SECTION BTB header) and exposes no PCGEN-stage BTB channel (port
 *   freeze). Its ID-stage `pred_chgflw` is a verbatim donor clone that
 *   evaluates to 0 in exactly the "BHT predicts taken + BTB hit + target
 *   correct" case (where the donor's other channel does the job) -> NO
 *   redirect -> the fetch streams linearly past the branch while bju_pcgen
 *   tracks the resolved taken path -> prediction/fetch desync -> a later
 *   branch mispredicts from the desynced bju_pcgen -> bogus redirect target
 *   -> misaligned load traps -> hang.
 *
 * This program is the SMALLEST repro found (see /tmp/m8t4b_stage/m2.c): a
 * double literal + 8-byte memcpy + one printf. The FP is incidental; the
 * hang loop lives in the clib printf/float-format code path (a backward
 * bne that is BHT-predicted-taken with a correct BTB hit). The marks to
 * 0x10000000 (the UART THR) are just progress sentinels for the log.
 *
 * EXPECTED:
 *   Fixed RTL (BPU.v:824 with the `pred_br_taken` term) -> PASS, tohost=1,
 *     ~7k final_cycles (see build/repro.log).
 *   Pre-fix RTL (the verbatim donor formula) -> HANG: tohost is never
 *     written, so run.sh's timeout fires and reports FAIL. That is the
 *     regression guard: a reverted BPU.v:824 makes this case FAIL.
 *
 * NOTE: this case is intentionally NOT under cases/ and its ELF is built
 * into this directory's own build/ (not test/m8/build/), so run_all.sh's
 * `build/*.elf` glob never picks it up and it is not part of the donor
 * parity set. It is a standalone regression guard only.
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern void __exit(void);
static void mark(char c){ *(volatile uint32_t*)0x10000000u = (uint32_t)c; }

int main(void){
  mark('1');
  double r = 0.75;
  mark('2');
  uint64_t bits; __builtin_memcpy(&bits, &r, 8);
  mark('3');
  printf("RES lo=%08x hi=%08x\n", (unsigned)bits, (unsigned)(bits>>32));
  mark('4');
  __exit();
  return 0;
}
