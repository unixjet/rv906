/* rv906v.ld - linker script for the riscv-tests v-env (sv39) on rv906.
 *
 * PORT of rv12/test/m4/env/v/rv12v.ld -- same mechanism, same address, same
 * reasoning. This is the UPSTREAM env/v/link.ld with ONE change, mirroring
 * the change test/m2/env/rv906.ld makes to env/p/link.ld for the SAME
 * reason: the `.tohost` output section moves OUT of the 0x8000_0000
 * cacheable image and into the UNCACHED aperture at 0x7FFF_F000. Text and
 * data are laid out exactly as upstream, one page lower because .tohost no
 * longer sits between .text.init and .text.
 *
 * WHY tohost MUST BE UNCACHED HERE TOO -- rv906's PMA marks
 * [0x8000_0000, 0xFFFF_FFFF] cacheable (test/m2/env/rv906.ld's own header;
 * rtl/MMU.v's sysmap/pma_cacheable table agrees) and everything below
 * 0x8000_0000 is NOT. rv906's D-cache is write-back with no coherent port
 * into the C++ harness's ExtMem model, so a tohost write that HITS a cached
 * line would park the pass/fail word where the harness's poll loop cannot
 * see it. Same argument as test/m2/env/rv906.ld, restated for the v-env,
 * which is the worse case of the two: env/v/vm.c's do_tohost READS tohost
 * before it writes it (`while (tohost) fromhost = 0;`), so the read alone
 * would allocate the line and every store would hit regardless of the
 * D-cache's write-allocate policy.
 *
 * WHY 0x7FFF_F000 SPECIFICALLY -- same address rv12 uses and the same
 * reason: bit 31 is the PMA cacheable bit and it is tested on the PHYSICAL
 * address the walk produces, so bit31=0 is uncached/strongly-ordered
 * whether translation is on or off. 0x7FFF_F000 sits 4 KB below the reset
 * vector, clear of any peripheral window, so the address decoder defaults
 * it to the memory slave.
 *
 * *** AND IT NEEDS ONE PAGE-TABLE ENTRY THE v-ENV DOES NOT BUILD, WHICH IS
 * *** WHY THIS FILE HAS A COMPANION. *** vm_boot maps exactly one kernel
 * megapage -- VA 0xFFFF_FFFF_FFE0_0000 -> PA 0x8000_0000, 2 MB
 * (vm.c:277-278) -- and the kernel reaches `tohost` pc-relatively
 * (-mcmodel=medany), so moving the symbol 4 KB below the text base moves
 * its VA 4 KB below the kernel window, into the megapage vm_boot leaves
 * EMPTY. test/m4/env/v/riscv_test.h's EXTRA_INIT installs that one entry
 * (kernel_l2pt[510] -> PA 0x7FE0_0000) before vm_boot runs. Read its
 * header: the two files are one mechanism and neither works alone.
 *
 * WHY .tohost COMES FIRST IN THIS FILE
 *   A linker script's location counter only moves forward, so an address
 *   BELOW the text base has to be assigned before `. = 0x80000000`. Same
 *   trick, same reason, as test/m2/env/rv906.ld.
 *
 * WHY THE BLOCK IS PADDED TO 0x80
 *   Ported verbatim from rv12v.ld: whatever beat width the memory model
 *   writes, nothing else may share tohost/fromhost's 16-byte blocks or it
 *   gets clobbered by an unrelated store landing in the same beat. Upstream
 *   already keeps tohost and fromhost 64 B apart with `.align 6`
 *   (riscv_test.h RVTEST_DATA_BEGIN); the `. = 0x80` here additionally
 *   reserves the tail so no other input section can be packed into
 *   fromhost's block either.
 *   Resulting layout: tohost = 0x7FFFF000, fromhost = 0x7FFFF040.
 */

OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{
  /* uncached aperture (bit31 = 0) -- must precede the 0x80000000 image */
  . = 0x7FFFF000;
  .tohost : { *(.tohost) . = 0x80; }

  /* from here on: upstream env/v/link.ld, unchanged except that .tohost is
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
