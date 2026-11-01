// Cycle timing on the SoC: mcycle (the only cycle CSR the CPU implements). Compiled out on the host.
// NO_TIMING removes the fine-grained per-category reads (tnow) but keeps the per-run totals (tnow64),
// so a NO_TIMING build measures how much the instrumentation itself costs.
#pragma once
#include <stdint.h>

#if defined(__riscv)
static inline uint64_t tnow64(void)
{
    uint32_t hi, lo, hi2;
    do {
        __asm__ volatile("csrr %0, mcycleh" : "=r"(hi));
        __asm__ volatile("csrr %0, mcycle" : "=r"(lo));
        __asm__ volatile("csrr %0, mcycleh" : "=r"(hi2));
    } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}
#elif defined(HOST_TIMING) && defined(__x86_64__)
#include <x86intrin.h>
static inline uint64_t tnow64(void) { return __rdtsc(); }   // host-only timed build (rrtc_host_timed)
#else
static inline uint64_t tnow64(void) { return 0; }
#endif

#if defined(__riscv) && !defined(NO_TIMING)
#define TIMING 1
static inline uint32_t tnow(void)
{
    uint32_t c;
    __asm__ volatile("csrr %0, mcycle" : "=r"(c));
    return c;
}
#elif defined(HOST_TIMING) && defined(__x86_64__)
#define TIMING 1
static inline uint32_t tnow(void) { return (uint32_t)__rdtsc(); }
#else
#define TIMING 0
static inline uint32_t tnow(void) { return 0; }
#endif
