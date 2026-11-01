/* sw_rrtc/soc/linker.ld (copied from sw_coremark/linker.ld); the arena runs from _end up to ARENA_END */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    DDR2_RAM (rwx) : ORIGIN = 0x60000000, LENGTH = 128M
}

_STACK_SIZE = 512K;
_HEAP_SIZE  = 512K;

SECTIONS
{
    .text : {
        . = ALIGN(4);
        _stext = .;
        KEEP(*crt0.o(.text.startup))
        *(.text.startup)
        *(.text .text.*)
        *(.gnu.linkonce.t.*)
        _etext = .;
    } > DDR2_RAM

    .rodata : {
        . = ALIGN(4);
        _srodata = .;
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
        _erodata = .;
    } > DDR2_RAM

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        *(.sdata .sdata.*)
        _edata = .;
    } > DDR2_RAM

    .bss : {
        . = ALIGN(4);
        _bss_start = .;
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        /* small-data .sbss (gcc puts globals <= 8 bytes there): must be inside the range crt0
           zeroes. It used to fall after __bss_end and kept whatever DDR held (FreeRTOS on the board). */
        *(.sbss .sbss.*)
        . = ALIGN(4);
        _bss_end = .;
        __bss_end = .;
    } > DDR2_RAM

    .stack : {
        . = ALIGN(16);
        _stack_end = .;
        . += _STACK_SIZE;
        _stack_start = .;
    } > DDR2_RAM

    .heap : {
        . = ALIGN(8);
        _heap_start = .;
        . += _HEAP_SIZE;
        _heap_end = .;
    } > DDR2_RAM

    _end = .;
}