/* link_m8.ld - M8 link script: donor smart_run case images for the rv906
 * harness (design doc 2026-09-21-m8-crosscheck-design.md section 4.6).
 *
 * Ported from the donor smart_run/tests/lib/linker.lcf (image at 0x0 in
 * its SRAM map: MEM1 0x0/0x40000 text, MEM2 0x40000/0xc0000 data,
 * __kernel_stack = 0xee000) to the rv906 sysmap (rtl/RVProcAXI.v:152-159):
 *
 *   - Image at 0x80000000 -- MEM origin, the reset vector the core boots
 *     from. ENTRY(__start); the crt0 startup code sits in .text.startup,
 *     which the .text output section places FIRST, so __start lands at
 *     0x80000000 (same crt0-first order as the donor's linker.lcf,
 *     achieved by section wildcard instead of object-name reference).
 *   - .tohost/.fromhost at 0x7FFFF000 -- inside the UNCACHED aperture
 *     (the PMA marks only [0x80000000, 0xFFFFFFFF] cacheable), same
 *     convention as test/m2/common.ld:38-39 and test/m6/common.ld:25.
 *     The `tohost` symbol is exported here and is what the C++ harness
 *     polls BY SYMBOL (testbench/load_elf.cpp find_symbols; an ELF that
 *     does not export it hangs the run). Padded to 0x80 because the
 *     memory model writes a whole 16-byte AXI beat ignoring wstrb.
 *   - __kernel_stack grows DOWN from the top of the 1 MB window at
 *     0x80100000 (top = 0x80200000). crt0's trap-handler pushes
 *     (-16/-24(sp)) and the case handlers' frames (SETINT -24/-32(sp),
 *     interrupt-case stack_push -24) all stay inside the window. The
 *     case image itself must stay < 1 MB so it never reaches the window.
 *   - The trap handler + vector_table live in their own .crt_text output
 *     section: the scaffold's `.align 10` (1024 B) would otherwise
 *     propagate to the .text output-section alignment and push the image
 *     start off 0x80000000. (The scaffold bytes themselves are
 *     unchanged -- this is a link-section detail, like the donor's own
 *     .lcf placing crt0 first.)
 *
 *   - PAGE-TABLE SCRATCH HOLE (documented, NOT enforced): 0x81000000 is
 *     reserved for the MMU/exception cases' Sv39 root table (design
 *     section 4.2/4.3, T4 -- satp PPN 0x81000 = 16 MB into the 2 GB
 *     MEM window). The linked image is
 *     < 1 MB (ends below 0x80100000) and the stack window ends at
 *     0x80200000, so no section the linker places can land there.
 *     Recorded so a future case that grows past 1 MB is a review item,
 *     not a silent clobber (design R6).
 *
 * BSS is NOT zeroed by crt0 (the donor relies on its tb's "wipe memory
 * to 0"); on rv906 the C++ ExtMem model starts at 0, so .bss is zero
 * without an init loop.
 */

OUTPUT_ARCH(riscv)
ENTRY(__start)

__kernel_stack = 0x80200000;   /* top of the 1 MB window at 0x80100000 */

SECTIONS
{
  /* uncached aperture (below the image) -- must precede it */
  . = 0x7FFFF000;
  .tohost : { tohost = .; fromhost = . + 0x40; *(.tohost) . = 0x80; }

  . = 0x80000000;
  .text :
  {
      *(.text.startup)
      *(.text*)
  }

  /* crt0 trap handler + vector_table (own section: the .align 10 inside
   * must not set the .text output-section alignment) */
  .crt_text :
  {
      *(.crt_text)
  }

  .rodata :
  {
      *(.rodata*)
      *(.srodata)
      *(.srodata.*)
      *(.srodata.cst4*)
      *(.srodata.cst8*)
  }

  .data :
  {
      *(.data*)
      *(.sdata*)
  }

  .bss :
  {
      *(.bss) *(.sbss) *(COMMON)
      . = ALIGN(8);
  }

  end = .;
}
