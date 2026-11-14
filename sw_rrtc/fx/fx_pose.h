// Pose check of the golden model: FKU + SCU + QD hierarchy; env queries go to a pluggable checker.
#pragma once
#include <stdint.h>
#include "fx_fk.h"

#define FX_WS_FRAC   14                    // workspace LSB = 2^-14 m
#define FX_WS_LSB_M  (1.0 / 16384.0)
#define FX_DELTA_FK_MM 0.28                // eps uses 2 x measured delta_FK (F = 18, TS = 6: 0.137 mm)

// Env oracle: centre in workspace coords (s18, origin w_min), radius r_q in LSB; 1 = hit
typedef int (*fx_env_fn)(int32_t cx, int32_t cy, int32_t cz, uint32_t rq);

typedef struct {
    uint64_t poses, env_queries, env_hits, self_pairs, self_hits;
} fx_pose_stats_t;

extern fx_pose_stats_t fx_ps;
extern int32_t fx_w_min;                   // w_min in workspace LSB (-2 m = -32768)
extern uint32_t fx_rq[FX_N_SPHERES];       // ceil((r + r_point + eps) / LSB)
extern int fx_cur_sphere, fx_cur_fine;      // sphere being queried (trace tags)

// delta_fk_m: FK error bound folded into eps; flat = 1 queries the 59 fine spheres only
void fx_pose_init(double delta_fk_m, int flat, fx_env_fn env);
int fx_pose_ok(const int16_t *q);          // 1 = collision-free
int fx_self_ok(const int32_t c[FX_N_SPHERES][3]);   // SCU: 1 = no self collision
int fx_env_ok(const int32_t c[FX_N_SPHERES][3]);    // QD + CSE: 1 = no environment collision
uint64_t fx_self_thr(int i);               // pair threshold, outer pairs first then inner (RTL ROM)
#define FX_SELF_MAX_ROWS 16                // rows (distinct first spheres) of one outer pair
int fx_self_rows(int o);                   // row filter of the SCU: rows of outer pair o
int fx_self_row_a(int o, int r);           // sphere of row r
uint64_t fx_self_fthr(int o, int r);       // threshold of row r against the outer sphere fx_self_outer[o][1]
int fx_self_ok_filt(const int32_t c[FX_N_SPHERES][3], uint32_t *tests);   // fx_self_ok with the row filter
void fx_fk_ws(const int16_t *q, int32_t c[FX_N_SPHERES][3]);
