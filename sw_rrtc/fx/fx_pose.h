// Pose check of the golden model: FKU + SCU + QD hierarchy; env queries go to a pluggable checker.
#pragma once
#include <stdint.h>
#include "fx_fk.h"

#define FX_WS_FRAC   14                    // workspace LSB = 2^-14 m
#define FX_WS_LSB_M  (1.0 / 16384.0)

// Env oracle: centre in workspace coords (s18, origin w_min), radius r_q in LSB; 1 = hit
typedef int (*fx_env_fn)(int32_t cx, int32_t cy, int32_t cz, uint32_t rq);

typedef struct {
    uint64_t poses, env_queries, env_hits, self_pairs, self_hits;
} fx_pose_stats_t;

extern fx_pose_stats_t fx_ps;
extern int32_t fx_w_min;                   // w_min in workspace LSB (floor(-1.19 m * 2^14))
extern uint32_t fx_rq[FX_N_SPHERES];       // ceil((r + r_point + eps) / LSB)
extern int fx_cur_sphere, fx_cur_fine;      // sphere being queried (trace tags)

// delta_fk_m: FK error bound folded into eps; flat = 1 queries the 59 fine spheres only
void fx_pose_init(double delta_fk_m, int flat, fx_env_fn env);
int fx_pose_ok(const int16_t *q);          // 1 = collision-free
void fx_fk_ws(const int16_t *q, int32_t c[FX_N_SPHERES][3]);
