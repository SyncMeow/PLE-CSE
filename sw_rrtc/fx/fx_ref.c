// DPI-C entry points into the golden model for the PLE RTL testbenches (Platform/accel/tb).
// Includes fx_plan.c / fx_pose.c / fx_fk_gen.c so the testbench calls the very same static functions.
// Built as C with gcc into libfxref.a (Platform/accel/Makefile), linked into the Verilator models.
#include "fx_plan.c"
#include "fx_pose.c"
#include "fx_fk_gen.c"      // defines macros F / TS / MUL: keep it the last include

#ifdef __cplusplus   // Verilator builds this file as C++; DPI expects C linkage
extern "C" {
#endif

// ---- SMP
void fx_ref_smp_reset(int seed) { smp_reset((uint32_t)seed); }
int fx_ref_smp_shift(int j) { return (int)h_shift[j]; }         // rotation the CPU writes to the PLE
void fx_ref_smp_next(short *q)                                   // q[7]
{
    int16_t t[FX_DIM];
    smp_next(t);
    for (int j = 0; j < FX_DIM; ++j) q[j] = t[j];
}

// Halton step when digit k is the lowest free digit: inv_k - (b-1) * sum_{i<k} inv_i (mod 2^32)
unsigned fx_ref_smp_delta(int j, int k)
{
    uint32_t d = h_inv[j][k];
    for (int i = 0; i < k; ++i) d -= (h_base[j] - 1) * h_inv[j][i];
    return d;
}

// ---- TREE + NNU: the testbench mirrors every insert, then asks for the nearest node
void fx_ref_tree_clear(int tree) { trees[tree].n = 0; }
int fx_ref_tree_insert(int tree, const short *q, int parent)      // returns node index or -1
{
    int16_t t[FX_DIM];
    for (int j = 0; j < FX_DIM; ++j) t[j] = q[j];
    return tree_insert(&trees[tree], t, (uint32_t)parent);
}
int fx_ref_nnu(int tree, const short *q, long long *d2)          // returns nearest index
{
    int16_t t[FX_DIM];
    int64_t d;
    for (int j = 0; j < FX_DIM; ++j) t[j] = q[j];
    uint32_t i = nnu(&trees[tree], t, &d);
    *d2 = d;
    return (int)i;
}

// ---- FKU: sphere centres in workspace coordinates (golden fx_fk_ws), c[70 * 3] = sphere-major x, y, z
void fx_ref_fk_ws(const short *q, int *c)
{
    static int init;
    int16_t t[FX_DIM];
    int32_t cc[FX_N_SPHERES][3];
    if (!init && !fx_w_min) fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0);    // keep an env set by fx_ref_env_load
    init = 1;
    for (int j = 0; j < FX_DIM; ++j) t[j] = q[j];
    fx_fk_ws(t, cc);
    for (int s = 0; s < FX_N_SPHERES; ++s)
        for (int k = 0; k < 3; ++k) c[3 * s + k] = cc[s][k];
}

// ---- SCU: golden self-collision check on c[70 * 3] (sphere-major x, y, z); 1 = no self collision
int fx_ref_self_ok(const int *c)
{
    int32_t cc[FX_N_SPHERES][3];
    short q0[FX_DIM] = {0};
    int dummy[3 * FX_N_SPHERES];
    fx_ref_fk_ws(q0, dummy);                                     // makes sure fx_pose_init ran
    for (int s = 0; s < FX_N_SPHERES; ++s)
        for (int k = 0; k < 3; ++k) cc[s][k] = c[3 * s + k];
    return fx_self_ok(cc);
}

long long fx_ref_self_pairs(void) { return (long long)fx_ps.self_pairs; }   // golden pair-test count so far

// inner pair i (0..689): its spheres, its outer pair's spheres; returns the inner threshold
long long fx_ref_self_info(int i, int *a, int *b, int *oa, int *ob, long long *othr)
{
    int o = 0;
    while (i >= fx_self_off[o + 1]) ++o;
    *a = fx_self_inner[i][0];
    *b = fx_self_inner[i][1];
    *oa = fx_self_outer[o][0];
    *ob = fx_self_outer[o][1];
    *othr = (long long)fx_self_thr(o);
    return (long long)fx_self_thr(FX_N_SELF_OUTER + i);
}

// ---- QD + CSE: environment of a MotionBenchMaker problem, golden pose environment check
#include "fx_env.h"
#include <stdlib.h>
#include "env.h"
#include "problems.h"

static fx_env_t ref_env;
static int ref_cse(int32_t cx, int32_t cy, int32_t cz, uint32_t rq) { return fx_cse_ref(&ref_env, cx, cy, cz, rq); }

// builds format-v0 environment of problem i (same as rrtc_fx); flat = 1 queries fine spheres only
int fx_ref_env_load(int i, int flat)
{
    static void *mem;
    points_t raw;
    if (!mem) mem = malloc(1u << 27);
    arena_init(mem, 1u << 27);
    pc_sample(&problems[i], &raw);
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, flat, ref_cse);
    fx_env_build(raw.x, raw.y, raw.z, raw.n, ENV_HASH_CAP, &ref_env);
    return ref_env.valid ? (int)ref_env.n_pts : -1;
}
// 32-bit word w of entry addr of the loaded environment in the cse_env_mem write-port layout (fx_env_dump):
// region 0 HDR (word addr), 1 DIR, 2 CELL (w = 0..3 of the 128-bit word), 3 PTS (w 0 = {y, x}, 1 = z)
int fx_ref_env_word(int region, int addr, int w)
{
    const fx_env_t *e = &ref_env;
    if (region == 0) {
        static const int k3[9] = {0, 0, 0, 0, 1, 2, 0, 1, 2};
        if (addr == 0) return (e->err ? 2 : 0) | (e->valid ? 1 : 0);
        if (addr == 1) return (int)e->n_pts;
        if (addr == 2) return (int)e->n_cells;
        return addr < 6 ? e->glo[k3[addr]] : e->ghi[k3[addr]];
    }
    if (region == 1) return e->dir[addr];
    if (region == 2) {
        const env_cell_t *c = &e->cell[addr];
        unsigned __int128 v = (unsigned __int128)c->base | ((unsigned __int128)(c->count & 0x3ffu) << 16);
        for (int k = 0; k < 3; ++k) {
            v |= (unsigned __int128)(c->lo[k] & 0x7ffu) << (26 + 11 * k);
            v |= (unsigned __int128)(c->hi[k] & 0x7ffu) << (59 + 11 * k);
        }
        return (int)(uint32_t)(v >> (32 * w));
    }
    return w == 0 ? (int)((uint32_t)e->py[addr] << 16 | e->px[addr]) : e->pz[addr];
}
int fx_ref_prob_valid(int i) { return i >= 0 && i < (int)n_problems && problems[i].valid; }
int fx_ref_cse(int cx, int cy, int cz, int rq) { return fx_cse_ref(&ref_env, cx, cy, cz, (uint32_t)rq); }

// Performance model only (not part of the golden definition): cycles one CSE unit spends on a query,
// design doc 4.1 cost: global cull 1, per cell walked 1 (DIR), non-empty +1 (CELL) +1 (AABB),
// AABB pass + ceil(points tested / lanes) (points stop at the first hit, as in fx_cse_ref)
int fx_ref_cse_cost(int cx, int cy, int cz, int rq, int lanes)
{
    const fx_env_t *e = &ref_env;
    const int32_t c[3] = {cx, cy, cz}, r = rq;
    const int64_t r2 = (int64_t)rq * rq;
    int cost = 1;
    if (!e->n_pts) return cost;
    for (int k = 0; k < 3; ++k)
        if (c[k] + r < e->glo[k] || c[k] - r > e->ghi[k]) return cost;
    int32_t lo[3], hi[3];
    for (int k = 0; k < 3; ++k) {
        int32_t a = (c[k] - r) >> ENV_QSHIFT, b = (c[k] + r) >> ENV_QSHIFT;
        lo[k] = a < 0 ? 0 : (a > ENV_G - 1 ? ENV_G - 1 : a);
        hi[k] = b < 0 ? 0 : (b > ENV_G - 1 ? ENV_G - 1 : b);
        if ((c[k] + r) < 0 || a > ENV_G - 1) return cost;
    }
    for (int32_t gz = lo[2]; gz <= hi[2]; ++gz)
        for (int32_t gy = lo[1]; gy <= hi[1]; ++gy)
            for (int32_t gx = lo[0]; gx <= hi[0]; ++gx) {
                cost++;
                uint16_t d = e->dir[(gz * ENV_G + gy) * ENV_G + gx];
                if (!(d & 0x8000u)) continue;
                cost += 2;
                const env_cell_t *cl = &e->cell[d & 0x0fffu];
                const int32_t org[3] = {gx << ENV_QSHIFT, gy << ENV_QSHIFT, gz << ENV_QSHIFT};
                int64_t ad = 0;
                for (int k = 0; k < 3; ++k) {
                    int32_t b0 = org[k] + cl->lo[k], b1 = org[k] + cl->hi[k];
                    int32_t t = c[k] < b0 ? b0 : (c[k] > b1 ? b1 : c[k]);
                    ad += (int64_t)(c[k] - t) * (c[k] - t);
                }
                if (ad > r2) continue;
                int n = 0, hit = 0;
                for (uint32_t i = cl->base; i < (uint32_t)cl->base + cl->count && !hit; ++i) {
                    n++;
                    int64_t dx = cx - e->px[i], dy = cy - e->py[i], dz = cz - e->pz[i];
                    hit = dx * dx + dy * dy + dz * dz <= r2;
                }
                cost += (n + lanes - 1) / lanes;
                if (hit) return cost;
            }
    return cost;
}
int fx_ref_env_ok(const int *c)                                  // 1 = no environment collision
{
    int32_t cc[FX_N_SPHERES][3];
    for (int s = 0; s < FX_N_SPHERES; ++s)
        for (int k = 0; k < 3; ++k) cc[s][k] = c[3 * s + k];
    return fx_env_ok(cc);
}

// ---- EVU: golden edge_ok with the full pose check (environment of fx_ref_env_load + self); 1 = edge free
int fx_ref_edge_ok(const short *start, const int *vec, long long len_h, int *poses)
{
    static fx_result_t dummy;
    int16_t s[FX_DIM];
    int32_t v[FX_DIM];
    for (int j = 0; j < FX_DIM; ++j) { s[j] = start[j]; v[j] = vec[j]; }
    dummy.poses = 0;
    g_r = &dummy;
    g_pose_ok = fx_pose_ok;
    int ok = edge_ok(s, v, (uint64_t)len_h);
    *poses = (int)dummy.poses;
    return ok;
}

// ---- CORE: a whole golden planning run on problem i (environment from fx_ref_env_load)
// stats: status, iterations, size_start, size_goal, dd_rejects, edges, nn_queries, path_len
// sg: start[7], goal[7]; shift: Halton rotation the CPU would write; path: path_len x 7
// directed tests: DD radius for the following fx_ref_solve calls (0 = default); boundary counters of the last run
static uint32_t ref_radius;
void fx_ref_set_radius(int radius) { ref_radius = (uint32_t)radius; }
int fx_ref_cov_dd_eq(void) { return (int)fx_cov_dd_eq; }
int fx_ref_cov_conn0(void) { return (int)fx_cov_conn0; }
void fx_ref_set_force_conn0(int k) { fx_force_conn0_at = (uint32_t)k; }

int fx_ref_solve(int i, int seed, int max_iter, int range_h, int *stats, short *sg, int *shift, short *path)
{
    static fx_result_t r;
    fx_settings_t s;
    int16_t qs[FX_DIM], qg[FX_DIM];
    fx_default_settings(&s);
    s.range_h = (uint32_t)range_h;
    if (ref_radius) s.radius = ref_radius;
    fx_cov_dd_eq = fx_cov_conn0 = 0;
    s.seed = (uint32_t)seed;
    s.max_iterations = (uint32_t)max_iter;
    for (int j = 0; j < FX_DIM; ++j) {
        qs[j] = fx_q_from_float(problems[i].start[j]);
        qg[j] = fx_q_from_float(problems[i].goal[j]);
        sg[j] = qs[j];
        sg[FX_DIM + j] = qg[j];
    }
    smp_reset((uint32_t)seed);
    for (int j = 0; j < FX_DIM; ++j) shift[j] = (int)h_shift[j];
    fx_solve(qs, qg, &s, fx_pose_ok, &r);
    stats[0] = r.status; stats[1] = (int)r.iterations; stats[2] = (int)r.size_start; stats[3] = (int)r.size_goal;
    stats[4] = (int)r.dd_rejects; stats[5] = (int)r.edges; stats[6] = (int)r.nn_queries; stats[7] = (int)r.path_len;
    for (uint32_t p = 0; p < r.path_len; ++p)
        for (int j = 0; j < FX_DIM; ++j) path[p * FX_DIM + j] = r.path[p][j];
    return r.status;
}

// ---- ALU: the golden exact-integer helpers
unsigned long long fx_ref_isqrt(unsigned long long v) { return fx_isqrt(v); }
long long fx_ref_fdiv(long long a, long long b) { return fx_fdiv(a, b); }

// ---- RKG: run the golden edge_ok with a recording pose callback (every pose of the edge, VAMP order)
#define REF_MAX_POSES 8192
static int16_t rec[REF_MAX_POSES][FX_DIM];
static int rec_n;
static int rec_pose(const int16_t *q)
{
    if (rec_n < REF_MAX_POSES) for (int j = 0; j < FX_DIM; ++j) rec[rec_n][j] = q[j];
    rec_n++;
    return 1;
}
int fx_ref_rake(const short *start, const int *vec, long long len_h)   // returns the pose count
{
    static fx_result_t dummy;
    int16_t s[FX_DIM];
    int32_t v[FX_DIM];
    for (int j = 0; j < FX_DIM; ++j) { s[j] = start[j]; v[j] = vec[j]; }
    g_r = &dummy;
    g_pose_ok = rec_pose;
    rec_n = 0;
    edge_ok(s, v, (uint64_t)len_h);
    return rec_n;
}
void fx_ref_rake_pose(int i, short *q)                                  // i-th recorded pose
{
    for (int j = 0; j < FX_DIM; ++j) q[j] = (i < REF_MAX_POSES) ? rec[i][j] : 0;
}

#ifdef __cplusplus
}
#endif
