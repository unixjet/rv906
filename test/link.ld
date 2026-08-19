OUTPUT_ARCH( "riscv" )
ENTRY(_start)

__stack_size = DEFINED(__stack_size) ? __stack_size : 0x10000;   /* 64KB stack */
__heap_size = DEFINED(__heap_size) ? __heap_size : 0x80000;      /* 512KB heap */

MEMORY
{
  ICCM (rx)  : ORIGIN = 0x80000000, LENGTH = 0x100000  /* 1MB - code */
  XMEM (rwx) : ORIGIN = 0x90000000, LENGTH = 0x100000  /* 1MB - AXI external memory */
}

SECTIONS
{
  /* Code in ICCM */
  . = 0x80000000;
  .text.init : { *(.text.init) } > ICCM
  .text : { *(.text) *(.text.*) } > ICCM

  /* Data in external AXI memory (0x90000000) */
  . = 0x90000000;
  .rodata : { *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*) } > XMEM
  .data : { *(.data) *(.sdata) } > XMEM
  .init_array : {
    __init_array_start = .;
    *(.init_array)
    __init_array_end = .;
  } > XMEM
  .fini_array : {
    __fini_array_start = .;
    *(.fini_array)
    __fini_array_end = .;
  } > XMEM
  .bss : {
    __bss_start = .;
    *(.bss) *(.sbss)
    __bss_end = .;
  } > XMEM
  . = ALIGN(8);

  /* Stack section */
  _stack_start = .;
  _stack_top = . + __stack_size;
  . = _stack_top;

  /* Gap for argc/argv (4KB) */
  . = . + 0x1000;

  /* Heap section */
  _heap_start = .;
  _heap_end = . + __heap_size;
  . = _heap_end;

  _end = .;
}
