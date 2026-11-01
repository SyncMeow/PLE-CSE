# sw_rrtc/soc/crt0.s (copied from sw_coremark/crt0.s)
.section .text.startup
.global _start

_start:
    li sp, 0x67FFFFF0
    li s0, 0x40005000

    li t0, 33
    sw t0, 4(s0)

    la a0, __bss_start
    la a1, __bss_end
    bgeu a0, a1, end_clear_bss
clear_bss_loop:
    sw zero, 0(a0)
    addi a0, a0, 4
    bltu a0, a1, clear_bss_loop
end_clear_bss:

    li t0, 64
    sw t0, 4(s0)

    call __libc_init_array

    li s0, 0x40005000
    li t0, 35
    sw t0, 4(s0)

    call main

# end of the program (also fatal()); the RTL sim stops when this j . retires
.global _exit_spin
_exit_spin:
    j _exit_spin
