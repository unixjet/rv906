/* common.ld - linker script for M6's directed interrupt ELFs
 *
 * Same two load-bearing conventions as test/m2/common.ld:
 *
 *   1. .text at 0x8000_0000 -- the reset vector the core boots from.
 *   2. .tohost at 0x7FFF_F000 -- inside the UNCACHED aperture (the PMA
 *      marks only [0x8000_0000, 0xFFFF_FFFF] cacheable), so the PASS/
 *      FAIL store bypasses the DCache to ExtMem where the harness
 *      polls it. Padded to 0x80 because the memory model writes a whole
 *      16-byte AXI beat ignoring wstrb.
 *
 * M6 tests are self-contained M-mode (or M->S) assembly programs:
 * preamble sets mtvec/stvec + enables the interrupt source under test,
 * the handler validates cause/epc/tval, then tohost=1 (PASS) or
 * tohost=<testnum> (FAIL).
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

SECTIONS
{
  /* uncached aperture (below 0x80000000) -- must precede the image */
  . = 0x7FFFF000;
  .tohost : { tohost = .; *(.tohost) . = 0x80; }

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
