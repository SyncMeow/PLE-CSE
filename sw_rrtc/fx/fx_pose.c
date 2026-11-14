// FKU (to workspace coords) + SCU (fixed-point self collision) + QD (outer -> fine hierarchy).
#include "fx_pose.h"
#include <math.h>
#ifdef FX_SELF_XCHECK
#include <stdio.h>
#endif

#define R_POINT_M 0.0025                    // VAMP POINT_RADIUS

fx_pose_stats_t fx_ps;
int32_t fx_w_min;
uint32_t fx_rq[FX_N_SPHERES];
static uint64_t self_thr[FX_N_SELF_OUTER + FX_N_SELF_INNER];   // outer pairs first, then inner
// row filter of the SCU: rows = the distinct first spheres of an outer pair's inner pairs (list order)
static uint8_t self_row[FX_N_SELF_INNER];                      // inner pair -> row within its outer pair
static uint8_t self_nrow[FX_N_SELF_OUTER];
static uint8_t self_row_a[FX_N_SELF_OUTER][FX_SELF_MAX_ROWS];
static uint64_t self_fthr[FX_N_SELF_OUTER][FX_SELF_MAX_ROWS];
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
    // Filter threshold of row a against the outer sphere B of the other link. For every b of the row:
    //   |c_a - c_B| <= |c_a - c_b| + |c_b - c_B|,  |c_a - c_b| <= t_ab + 1 when the pair hits (thr = ceil(t_ab^2)),
    //   |c_b - c_B| <= obs_bB + 4 e_c: b and B are rigid, each computed centre is within e_c of the true one, and
    //   obs_bB is the computed distance at one pose. So a row whose filter test misses has no inner hit.
    // host/fx_scucheck (make fx-check) tests the rigid assumption and filter == plain on random poses.
    int32_t c0[FX_N_SPHERES][3];
    const int16_t q0[7] = {0};
    fx_fk_ws(q0, c0);
    for (int o = 0; o < FX_N_SELF_OUTER; ++o) {
        double bound[FX_SELF_MAX_ROWS] = {0};
        int ob = fx_self_outer[o][1];
        self_nrow[o] = 0;
        for (int i = fx_self_off[o]; i < fx_self_off[o + 1]; ++i) {
            int a = fx_self_inner[i][0], b = fx_self_inner[i][1], r = 0;
            while (r < self_nrow[o] && self_row_a[o][r] != a) ++r;
            if (r == self_nrow[o]) {
                if (r == FX_SELF_MAX_ROWS) __builtin_trap();     // raise FX_SELF_MAX_ROWS for this sphere model
                self_row_a[o][self_nrow[o]++] = (uint8_t)a;
            }
            self_row[i] = (uint8_t)r;
            double obs = 0;
            for (int k = 0; k < 3; ++k) { double d = (double)c0[b][k] - c0[ob][k]; obs += d * d; }
            double t = ((double)fx_sphere_r[a] + (double)fx_sphere_r[b]) / FX_WS_LSB_M + 2.0 * e_c + 1.0 + sqrt(obs) + 4.0 * e_c;
            if (t > bound[r]) bound[r] = t;
        }
        for (int r = 0; r < self_nrow[o]; ++r) self_fthr[o][r] = (uint64_t)ceil(bound[r] * bound[r]);
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
#ifdef FX_SELF_XCHECK                       // every planner pose: the row filter must not change the result
    {
        static uint64_t n, bad, tests, plain;
        uint32_t t;
        uint64_t p0 = fx_ps.self_pairs, h0 = fx_ps.self_hits;
        int a = fx_self_ok(c), b = fx_self_ok_filt(c, &t);
        plain += fx_ps.self_pairs - p0; fx_ps.self_pairs = p0; fx_ps.self_hits = h0;
        ++n; bad += (uint64_t)(a != b); tests += t;
        if ((n & 0xffff) == 0 || a != b)
            fprintf(stderr, "SCUXCHK poses=%llu mismatch=%llu tests_plain=%.2f tests_filter=%.2f\n", (unsigned long long)n,
                    (unsigned long long)bad, (double)plain / (double)n, (double)tests / (double)n);
    }
#endif
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
int fx_self_rows(int o) { return self_nrow[o]; }
int fx_self_row_a(int o, int r) { return self_row_a[o][r]; }
uint64_t fx_self_fthr(int o, int r) { return self_fthr[o][r]; }

static int d2_le(const int32_t *a, const int32_t *b, uint64_t thr)
{
    uint64_t d2 = 0;
    for (int k = 0; k < 3; ++k) { int64_t d = (int64_t)a[k] - b[k]; d2 += (uint64_t)(d * d); }
    return d2 <= thr;
}

// SCU with the row filter (the RTL order): all outer pairs, then the rows of the hit outer pairs against the
// other link's outer sphere, then the inner pairs of the rows that pass. Same result as fx_self_ok() on FK poses.
int fx_self_ok_filt(const int32_t c[FX_N_SPHERES][3], uint32_t *tests)
{
    uint8_t hit[FX_N_SELF_OUTER], pass[FX_N_SELF_OUTER][FX_SELF_MAX_ROWS];
    uint32_t n = FX_N_SELF_OUTER;
    int ok = 1;
    for (int o = 0; o < FX_N_SELF_OUTER; ++o) hit[o] = (uint8_t)d2_le(c[fx_self_outer[o][0]], c[fx_self_outer[o][1]], self_thr[o]);
    for (int o = 0; o < FX_N_SELF_OUTER; ++o) {
        if (!hit[o]) continue;
        for (int r = 0; r < self_nrow[o]; ++r)
            pass[o][r] = (uint8_t)d2_le(c[self_row_a[o][r]], c[fx_self_outer[o][1]], self_fthr[o][r]);
        n += self_nrow[o];
    }
    for (int o = 0; o < FX_N_SELF_OUTER && ok; ++o) {
        if (!hit[o]) continue;
        for (int i = fx_self_off[o]; i < fx_self_off[o + 1]; ++i) {
            if (!pass[o][self_row[i]]) continue;
            ++n;
            if (d2_le(c[fx_self_inner[i][0]], c[fx_self_inner[i][1]], self_thr[FX_N_SELF_OUTER + i])) { ok = 0; break; }
        }
    }
    if (tests) *tests = n;                 // defined for collision-free poses only (a hit ends the RTL early)
    return ok;
}

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
