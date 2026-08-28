/* rv906.ld - linker script for the riscv-tests p-env on rv906
 *
 * This is the UPSTREAM env/p/link.ld (kept verbatim next to it as
 * link.upstream.ld; diff the two) with ONE change: the `.tohost` output
 * section moves OUT of the 0x8000_0000 cacheable image and into the UNCACHED
 * aperture at 0x7FFF_F000. Text and data are laid out exactly as upstream.
 *
 * WHY tohost MUST BE UNCACHED
 *   The rv906 PMA (rtl/MMU.v `pma_cacheable`, S2.3.3) marks only
 *   [0x8000_0000, 0xFFFF_FFFF] as cacheable; EVERYTHING BELOW 0x8000_0000 is
 *   uncached. So `mmu_lsu_ca` is 0 for any address in the uncached aperture,
 *   and LSU.v's `touches_array = issue_real && mmu_lsu_ca && !ag_misalign`
 *   is 0 -- the access bypasses the DCache array and goes straight to the
 *   testbench memory model, where the C++ harness polls for the tohost word.
 *   This matters the MOMENT the MHCR preamble (env/riscv_test.h) turns the
 *   DCache ON: a tohost left in the cacheable image would park the pass/fail
 *   store in the dcache and the run would hang unless the test flushed -- and
 *   the stock p-env never flushes.
 *
 * WHY 0x7FFF_F000 SPECIFICALLY
 *   0x7FFF_F000 sits just 4 KB below the 0x8000_0000 reset vector, so the
 *   `sw TESTNUM, tohost, t5` write_tohost trampoline (riscv_test.h) stays
 *   inside +/-2 GB reach under -mcmodel=medany, and it is clear of the
 *   CLINT (0x0200_0000) / PLIC (0x0C00_0000) / UART (0x1000_1000) windows
 *   the AXI address decoder carves out -- it lands in DRAM's uncached
 *   aperture. (The same 4 KB-below-the-image trick M1's test/m1/common.ld
 *   uses for its .text.uncached block.)
 *
 * WHY .tohost COMES FIRST IN THIS FILE
 *   A linker script's location counter only moves forward, so an address
 *   BELOW the text base has to be assigned before `. = 0x80000000`.
 *
 * WHY THE BLOCK IS PADDED TO 0x80
 *   The testbench memory model writes a whole AXI beat and IGNORES wstrb
 *   (io/ExtMem.h mem_access does `memcpy(mem, din, 1 << size)`), and the
 *   store write-through path issues awsize = 4, i.e. 16-byte beats. So a
 *   4-byte store to tohost rewrites the whole 16-byte block around it, and
 *   NOTHING else may share that block or it gets clobbered. Upstream already
 *   keeps tohost and fromhost 64 B apart with `.align 6` (riscv_test.h
 *   RVTEST_DATA_BEGIN); the `. = 0x80` here additionally reserves the tail
 *   so no other input section can be packed into fromhost's block either.
 *   Resulting layout: tohost = 0x7FFF_F000, fromhost = 0x7FFF_F040.
 */

OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{
  /* uncached aperture (below 0x80000000) -- must precede the 0x80000000 image */
  . = 0x7FFFF000;
  .tohost : { *(.tohost) . = 0x80; }

  /* from here on: upstream env/p/link.ld, unchanged except that .tohost is
   * no longer in this list. */
  . = 0x80000000;
  .text.init : { *(.text.init) }
  . = ALIGN(0x1000);
  .text : { *(.text) }
  . = ALIGN(0x1000);
  .data : { *(.data) }
  .bss : { *(.bss) }
  _end = .;
}
