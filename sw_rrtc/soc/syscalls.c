// sw_rrtc/soc/syscalls.c (copied from sw_coremark/syscalls.c)
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/stat.h>

#define UART_BASE      0x40005000
#define UART_RX_FIFO   (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_TX_FIFO   (*(volatile uint32_t *)(UART_BASE + 0x04))
#define UART_STATUS    (*(volatile uint32_t *)(UART_BASE + 0x08))

void uart_putc(char c) {
    while (UART_STATUS & 0x08);
    UART_TX_FIFO = c;
}

char uart_getc() {
    while (!(UART_STATUS & 0x01));
    return (char)UART_RX_FIFO;
}

int32_t _write(int32_t file, char *ptr, int32_t len) {
    for (int32_t i = 0; i < len; i++) {
        if (ptr[i] == '\n') uart_putc('\r');
        uart_putc(ptr[i]);
    }
    return len;
}

extern char _end;
void *_sbrk(ptrdiff_t incr) {
    static char *heap_ptr = &_end;

    heap_ptr = (char *)(((uintptr_t)heap_ptr + 7) & ~7);

    char *prev_heap_ptr = heap_ptr;
    heap_ptr += incr;
    return (void *)prev_heap_ptr;
}


int32_t _close(int32_t file) {
    return -1;
}

int32_t _fstat(int32_t file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int32_t _isatty(int32_t file) {
    return 1;
}

int32_t _lseek(int32_t file, int32_t ptr, int32_t dir) {
    return 0;
}

int32_t _read(int32_t file, char *ptr, int32_t len) {
    return 0;
}

void _exit(int32_t status) {
    while (1) {
    }
}

int32_t _kill(int32_t pid, int32_t sig) {
    return -1;
}

int32_t _getpid(void) {
    return 1;
}

uint64_t get_system_cycles() {
    uint32_t cycles_lo, cycles_hi, cycles_hi_check;
    do {
        __asm__ volatile ("csrr %0, mcycleh" : "=r"(cycles_hi));
        __asm__ volatile ("csrr %0, mcycle"  : "=r"(cycles_lo));
        __asm__ volatile ("csrr %0, mcycleh" : "=r"(cycles_hi_check));
    } while (cycles_hi != cycles_hi_check);
    return ((uint64_t)cycles_hi << 32) | cycles_lo;
}

void DebugLog(const char* s) {
	printf(s);
}