.section .text
.global main
.global _start
_start:
    addi x10, x0, 0
    addi x11, x0, 1
    addi x12, x0, 11

loop:
    add  x10, x10, x1
    bne  x11, x12, loop    # branch back while i != 11
    addi x11, x11, 1       # --- delay slot: i++ (always executes)
    ebreak