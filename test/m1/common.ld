/* common.ld - linker script for the M1 directed fetch tests (plan Task 5.3;
 * reused unmodified by Task 6's directed suite)
 *
 * Adapted from rv12's own test/m1/common.ld (same file-structure precedent,
 * cited in the plan as a style reference for the M1 test build flow), with
 * the two load-bearing conventions it establishes kept identical because
 * they are pinned by facts about THIS repo's RTL, not by rv12's:
 *   1. .text at 0x80000000, which is FetchSink.v's RESET_VECTOR parameter
 *      default and rvproc_pkg.sv's cp0_xx_mrvbr wiring -- the core boots
 *      here, so the golden ISS's walk (m1_iss.h, M1::RESET_VECTOR) must
 *      start here too;
 *   2. the tohost line at rvproc_pkg.sv's ADDR_TOHOST (0x9000_1000), padded
 *      to a full 64-byte line -- the D-side AXI write FSM FetchSink lifted
 *      from TestMaster.v writes a whole line regardless of wstrb, so
 *      nothing else may share it.
 *
 * Everything a test executes lives in .text.init, in ONE section, because
 * the JR_PAD macro in macros.h places indirect-jump landing pads with
 * `.org`, whose operand is an offset from the start of the CURRENT section.
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

SECTIONS
{
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

    /* tohost/fromhost: see the padding note above. */
    . = 0x90001000;
    .tohost : { *(.tohost) . = 0x40; }

    /DISCARD/ : { *(.comment) *(.note.*) *(.riscv.attributes) }
}
