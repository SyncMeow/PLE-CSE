// SoC driver: runs PROB_FIRST..PROB_LAST x SEEDS (compile-time, see Makefile) and prints CSV over UART.
#include <stdio.h>
#include <stdint.h>
#include "bench.h"
#include "problems.h"
#include "timer.h"

#ifndef ARENA_END
#define ARENA_END 0x67F00000u   // sp starts at 0x67FF_FFF0 (crt0); 1 MB left for the stack
#endif
#if NN_W < 1 || NN_B < 1 || NN_B > 64 || NN_M < 1 || NN_M > 8 || NN_BITS < 1 || NN_BITS > 9
#error "bad NN parameters"
#endif
#define SBRK_RESERVE (1u << 20)  // _sbrk (newlib stdio buffers) grows from _end

void fatal(const char *msg)
{
    extern void _exit_spin(void) __attribute__((noreturn));
    printf("FATAL,%s\n", msg);
    _exit_spin();
}

static void _exit_spin_wrap(void)
{
    extern void _exit_spin(void) __attribute__((noreturn));
    _exit_spin();
}

static int scene_on(int s)
{
    for (const char *c = SCENES; *c; ++c)
        if (*c - '0' == s) return 1;
    return 0;
}

int main(void)
{
    extern char _end;
    uint32_t base = ((uint32_t)&_end + SBRK_RESERVE + 63u) & ~63u;
    if (base >= ARENA_END) fatal("image too large");

    // instrumentation cost: one tnow() pair around nothing
    uint32_t cal = 0xffffffffu;
    for (int k = 0; k < 64; ++k) {
        uint32_t a = tnow(), b = tnow();
        if (b - a < cal) cal = b - a;
    }
    rrtc_settings_t s;
    rrtc_default_settings(&s);
    s.nn.mode = NN_MODE; s.nn.m = NN_MODE ? NN_M : 0; s.nn.W = NN_W; s.nn.B = NN_B; s.nn.bits = NN_BITS;
    s.max_iterations = MAX_ITER;
    s.max_samples = MAX_SAMPLES;
    // planner buffers (rrtc_solve + nn_shared_alloc) must fit next to the largest environment (~8 MB)
    uint64_t need = (uint64_t)s.max_samples * (NN_STRIDE * 4 + 4 + 4 + 4 + (uint64_t)s.nn.m * 12) + (8u << 20);
    if (need > ARENA_END - base) {
        printf("FATAL,arena %lu B < %lu B needed: lower MAX_SAMPLES or NN_M\n", (unsigned long)(ARENA_END - base),
               (unsigned long)need);
        _exit_spin_wrap();
    }
    printf("RRTC START,timing=%d,cal=%lu,nn=%d,m=%d,W=%d,B=%d,bits=%d,max_iter=%lu,max_samples=%lu,arena=%lu\n",
           TIMING, (unsigned long)cal, s.nn.mode, s.nn.m, s.nn.W, s.nn.B, s.nn.bits,
           (unsigned long)s.max_iterations, (unsigned long)s.max_samples, (unsigned long)(ARENA_END - base));
    for (int i = PROB_FIRST; i <= PROB_LAST && i < n_problems; ++i) {
        if (!scene_on(problems[i].scene) || !problems[i].valid) continue;
        arena_init((void *)base, ARENA_END - base);
        mvt_t mvt;
        points_t filt;
        prep_t pp;
        bench_prepare(i, &mvt, &filt, &pp);
        bench_print_prep(i, &pp);
        for (int k = 0; k < SEEDS; ++k) {
            rrtc_result_t r;
            s.seed = (uint32_t)(SEED0 + k);
            bench_run(i, &s, &r, 0);
            bench_print_run(i, &s, &r);
        }
    }
    printf("RRTC DONE\n");
    return 0;
}
