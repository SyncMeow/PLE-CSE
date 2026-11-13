// FKU (to workspace coords) + SCU (fixed-point self collision) + QD (outer -> fine hierarchy).
#include "fx_pose.h"
#include <math.h>

#define R_POINT_M 0.0025                    // VAMP POINT_RADIUS

fx_pose_stats_t fx_ps;
int32_t fx_w_min;
uint32_t fx_rq[FX_N_SPHERES];
static uint64_t self_thr[FX_N_SELF_OUTER + FX_N_SELF_INNER];   // outer pairs first, then inner
int fx_cur_sphere, fx_cur_fine;
static int g_flat;
static fx_env_fn g_env;

void fx_pose_init(double delta_fk_m, int flat, fx_env_fn env)
{
    g_flat = flat;
    g_env = env;
    fx_w_min = -32768;                      // -2 m: the preprocessing origin (ENV_ORIGIN)
    // centre error: floor to LSB (< sqrt3 LSB) + delta_FK; points are stored exactly at LSB (< sqrt3 LSB)
    const double e_c = sqrt(3.0) + delta_fk_m / FX_WS_LSB_M, e_p = sqrt(3.0);
    for (int s = 0; s < FX_N_SPHERES; ++s)
        fx_rq[s] = (uint32_t)ceil(((double)fx_sphere_r[s] + R_POINT_M) / FX_WS_LSB_M + e_c + e_p);
    for (int i = 0; i < FX_N_SELF_OUTER + FX_N_SELF_INNER; ++i) {
        const uint8_t *p = i < FX_N_SELF_OUTER ? fx_self_outer[i] : fx_self_inner[i - FX_N_SELF_OUTER];
        double t = ((double)fx_sphere_r[p[0]] + (double)fx_sphere_r[p[1]]) / FX_WS_LSB_M + 2.0 * e_c;
        self_thr[i] = (uint64_t)ceil(t * t);
    }
}

void fx_fk_ws(const int16_t *q, int32_t c[FX_N_SPHERES][3])
{
    int64_t b[FX_N_SPHERES][3];
    fx_fk_base(q, b);
    for (int s = 0; s < FX_N_SPHERES; ++s)
        for (int k = 0; k < 3; ++k) c[s][k] = (int32_t)(b[s][k] >> (fx_fk_frac - FX_WS_FRAC)) - fx_w_min;
}

static int pair_hit(const int32_t *a, const int32_t *b, uint64_t thr)
{
    uint64_t d2 = 0;
    for (int k = 0; k < 3; ++k) { int64_t d = (int64_t)a[k] - b[k]; d2 += (uint64_t)(d * d); }
    fx_ps.self_pairs++;
    return d2 <= thr;
}

static int env_hit(const int32_t c[FX_N_SPHERES][3], int s)
{
    fx_ps.env_queries++;
    fx_cur_sphere = s;
    fx_cur_fine = s < FX_N_SPHERES - FX_N_OUTER;
    int h = g_env(c[s][0], c[s][1], c[s][2], fx_rq[s]);
    fx_ps.env_hits += (uint64_t)h;
    return h;
}

int fx_pose_ok(const int16_t *q)
{
    int32_t c[FX_N_SPHERES][3];
    fx_ps.poses++;
    fx_fk_ws(q, c);
    if (!fx_env_ok(c)) return 0;
    return fx_self_ok(c);
}

// environment (QD): outer sphere hit -> query its fine spheres; a fine hit = collision; 1 = free
int fx_env_ok(const int32_t c[FX_N_SPHERES][3])
{
    if (g_flat) {
        for (int o = 0; o < FX_N_OUTER; ++o)
            for (int k = fx_outer_kid_off[o]; k < fx_outer_kid_off[o + 1]; ++k)
                if (env_hit(c, fx_outer_kids[k])) return 0;
    } else {
        for (int o = 0; o < FX_N_OUTER; ++o) {
            if (!env_hit(c, fx_outer[o])) continue;
            for (int k = fx_outer_kid_off[o]; k < fx_outer_kid_off[o + 1]; ++k)
                if (env_hit(c, fx_outer_kids[k])) return 0;
        }
    }
    return 1;
}

uint64_t fx_self_thr(int i) { return self_thr[i]; }

// self collision (SCU): outer link pair hit -> test its sphere pairs
int fx_self_ok(const int32_t c[FX_N_SPHERES][3])
{
    for (int o = 0; o < FX_N_SELF_OUTER; ++o) {
        if (!pair_hit(c[fx_self_outer[o][0]], c[fx_self_outer[o][1]], self_thr[o])) continue;
        for (int i = fx_self_off[o]; i < fx_self_off[o + 1]; ++i)
            if (pair_hit(c[fx_self_inner[i][0]], c[fx_self_inner[i][1]], self_thr[FX_N_SELF_OUTER + i])) {
                fx_ps.self_hits++;
                return 0;
            }
    }
    return 1;
}
