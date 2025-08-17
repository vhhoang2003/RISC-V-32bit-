    .option norvc
    .text
    .globl _start
_start:
    # Base init
    addi x1, x0, 10
    addi x2, x0, 20

    # R-type
    add  x3,  x1, x2
    sub  x4,  x2, x1
    sll  x5,  x1, x2
    slt  x6,  x1, x2
    sltu x7,  x1, x2
    xor  x8,  x1, x2
    srl  x9,  x2, x1
    sra  x10, x2, x1
    or   x11, x1, x2
    and  x12, x1, x2

    # I-type arith/logic
    addi x13, x0, 100
    slti x14, x1, 50
    sltiu x15, x1, 50
    xori x16, x1, 0x55
    ori  x17, x1, 0x0f
    andi x18, x1, 0xf0
    slli x19, x1, 1
    srli x20, x1, 1
    srai x21, x1, 1

    lui   x30, 0x12345
    auipc x29, 0x1

    la   x22, byte0
    lb   x23, 0(x22)
    lbu  x24, 0(x22)

    la   x5, half0        
    lh   x25, 0(x5)
    lhu  x26, 0(x5)

    la   x6, word0         
    lw   x27, 0(x6)

    # Memory: store
    la   x28, dstbuf
    sb   x23, 0(x28)
    sh   x25, 2(x28)
    sw   x27, 4(x28)

    # Branches
    beq  x1, x1, 1f
    addi x0, x0, 0          
1:
    bne  x1, x2, 2f
    addi x0, x0, 0
2:
    blt  x1, x2, 3f
    addi x0, x0, 0
3:
    bge  x2, x1, 4f
    addi x0, x0, 0
4:
    bltu x1, x2, 5f
    addi x0, x0, 0
5:
    bgeu x2, x1, 6f
    addi x0, x0, 0
6:

    # Jumps
    jal  x31, after_jal
    addi x0, x0, 0
after_jal:
    la   x5, ret_here
    jalr x0, x5, 0
    addi x0, x0, 0
ret_here:
    fence
    ecall
    ebreak

done:
    j done

    .data
byte0:  .byte 0x11
half0:  .half 0x2233
word0:  .word 0x44556677
dstbuf: .space 8
