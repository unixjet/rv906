OUTPUT_ARCH(riscv)
ENTRY(_start)
SECTIONS {
    . = 0x80000000;
    .text : { *(.text.init) *(.text) }
    /* TestMaster overwrites the WHOLE 64-byte line at 0x90001000 on its
     * completion write (full wstrb; MEMCTL writes the full line regardless).
     * Pad .tohost to 0x40 so no future section can silently land inside
     * 0x90001000-0x9000103F and get clobbered. */
    . = 0x90001000;
    .tohost : { *(.tohost) . = 0x40; }
}
