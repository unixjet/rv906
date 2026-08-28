/* common.ld - linker script for M2's hand-written directed ELFs (plan 8.5)
 *
 * A FRESH, minimal harness for the Task 9 directed tests (alu_seq.S,
 * bju_seq.S, muldiv_seq.S, ld_st_uncached.S, ld_st_cached.S, csr_trap.S).
 * It uses the same two load-bearing conventions as the riscv-tests p-env
 * override (env/rv906.ld, plan 8.4):
 *
 *   1. .text at 0x8000_0000 -- the reset vector the core boots from
 *      (rvproc_pkg.sv cp0_xx_mrvbr wiring), and the reset/entry point the
 *      C++ harness passes to dut.init().
 *   2. .tohost at 0x7FFF_F000 -- inside the UNCACHED aperture. The PMA
 *      (rtl/MMU.v pma_cacheable) marks only [0x8000_0000, 0xFFFF_FFFF]
 *      cacheable, so a store to tohost has mmu_lsu_ca=0 and bypasses the
 *      DCache array (LSU.v touches_array) straight to ExtMem, where the
 *      harness polls it. This holds whether MHCR.de is 0 (caches off) or 1
 *      (caches on) -- the tohost aperture is uncached by ADDRESS, not by
 *      the MHCR enable. Padded to 0x80 because the memory model writes a
 *      whole 16-byte AXI beat ignoring wstrb, so nothing else may share the
 *      block (tohost = 0x7FFF_F000, fromhost = 0x7FFF_F040).
 *
 * DELIBERATELY NOT carried over from test/m1: the .text.uncached block
 * (M1's fence.i/uncached-fetch experiment) and every synthetic-oracle
 * convention. M2's directed tests run through the REAL pipeline and report
 * through tohost like a riscv-test, so none of M1's fetch-oracle layout is
 * needed.
 *
 * Everything a test executes lives in .text.init, in ONE section (the test
 * bodies are short and branch-local, so a single relocatable section is
 * enough; unlike M1 there is no `.org` landing-pad machinery).
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

SECTIONS
{
  /* uncached aperture (below 0x80000000) -- must precede the image */
  . = 0x7FFFF000;
  .tohost : { *(.tohost) . = 0x80; }

  . = 0x80000000;
  .text : {
      *(.text.init)
      *(.text)
      *(.text.*)
  }
  . = ALIGN(64);
  .rodata : { *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*) }
  . = ALIGN(64);
  .data : { *(.data) *(.data.*) *(.sdata) *(.sdata.*) }
  . = ALIGN(64);
  .bss : { *(.bss) *(.bss.*) *(.sbss) *(.sbss.*) *(COMMON) }

  /DISCARD/ : { *(.comment) *(.note.*) *(.riscv.attributes) }
}
