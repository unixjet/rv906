    .section .tohost, "aw", @progbits
    .globl tohost
tohost:   .dword 0
    .globl fromhost
fromhost: .dword 0

    .section .text.init, "ax", @progbits
    .globl _start
_start:
1:  j 1b
