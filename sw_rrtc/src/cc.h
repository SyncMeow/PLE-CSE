// Collision-check hooks called from the generated FK/CC code (gen/panda_fk.c).
#pragma once
#include <stdint.h>
#include "env.h"
#include "stats.h"
#include "timer.h"
#include "vmath.h"

extern const mvt_t *cc_mvt;        // environment of the current problem
extern const points_t *cc_brute;   // if set, use brute force instead of the MVT (validation)
extern uint32_t cc_t_fk;           // mcycle at the start of panda_fkcc

static inline void cc_fk_done(void)
{
#if TIMING
    uint32_t now = tnow();
    st.cyc_fk += (uint32_t)(now - cc_t_fk);
    st.timer_reads++;
#endif
}

int cc_env_sphere(float x, float y, float z, float r);

// Host-only experiment hooks (host/envstat.c); the SoC build compiles them out
#ifdef ENV_STATS
int envstat_query(int id, float x, float y, float z, float r);
int envstat_all_query(int id, float x, float y, float z, float r);
void panda_env_all(const float *q);
void envstat_edge_begin(void);
void envstat_pose(const float *q);
void envstat_edge_end(const float *start, const float *vec, uint32_t n, uint32_t blocks);
#define CC_ENV(id, x, y, z, r) envstat_query((id), (x), (y), (z), (r))
#define ENVSTAT(call) call
#else
#define CC_ENV(id, x, y, z, r) cc_env_sphere((x), (y), (z), (r))
#define ENVSTAT(call) ((void)0)
#endif

// VAMP sphere_sphere_sql2 < 0 (sign bit, as _mm256_testz_ps)
static inline int cc_self_pair(float ax, float ay, float az, float ar, float bx, float by, float bz, float br)
{
    float xs = ax - bx, ys = ay - by, zs = az - bz;
    float sum = (xs * xs) + (ys * ys) + (zs * zs);
    float rs = ar + br;
    st.self_pairs++;
    return (f2u(sum - rs * rs) >> 31) != 0;
}

// One pose (FK + environment + self collisions): 1 = valid
int cc_validate_pose(const float *q);
