// Shared driver pieces for the host and SoC builds.
#pragma once
#include "env.h"
#include "rrtc.h"
#include "stats.h"

typedef struct {
    uint32_t raw_n, filt_n, n_vox, pool_used, n_pool;
    uint64_t cyc_filter, cyc_build;   // SoC only (sampling is not timed: it stands in for the camera)
} prep_t;

// Sample + CenterVox + MVT for problem i; sets cc_mvt. Leaves everything in the arena.
void bench_prepare(int i, mvt_t *mvt, points_t *filtered, prep_t *pp);
// One planning run (arena state is restored afterwards)
void bench_run(int i, const rrtc_settings_t *s, rrtc_result_t *r, shadow_t *sh);

void bench_print_prep(int i, const prep_t *pp);
void bench_print_run(int i, const rrtc_settings_t *s, const rrtc_result_t *r);
void fatal(const char *msg);
