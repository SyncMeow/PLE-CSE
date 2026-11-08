// PLE golden model: CTRL + SMP + NNU + ALU + RKG + PBT, control flow of src/rrtc.c (VAMP RRTC).
#include "fx.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------- joint limits (VAMP panda.hh)
static const float lim_m[FX_DIM] = {
    5.934199810028076f, 3.6651999950408936f, 5.934199810028076f, 3.2288999557495117f,
    5.934199810028076f, 3.909600019454956f, 5.934199810028076f};
static const float lim_a[FX_DIM] = {
    -2.967099905014038f, -1.8325999975204468f, -2.967099905014038f, -3.1415998935699463f,
    -2.967099905014038f, -0.08730000257492065f, -2.967099905014038f};

// ---------------------------------------------------------------- SMP: integer Halton
// u = sum_k d_k * floor(2^32 / b^(k+1)) (Q0.32); updated incrementally, exact by construction
#define H_DIGITS 24
static const uint32_t h_base[FX_DIM] = {3, 5, 7, 11, 13, 17, 19};
static uint32_t h_inv[FX_DIM][H_DIGITS], h_dig[FX_DIM][H_DIGITS], h_u[FX_DIM], h_shift[FX_DIM];
static int16_t smp_a[FX_DIM];
static uint16_t smp_m[FX_DIM];

static void smp_reset(uint32_t seed)
{
    for (int j = 0; j < FX_DIM; ++j) {
        uint64_t p = h_base[j];
        for (int k = 0; k < H_DIGITS; ++k) {
            h_inv[j][k] = p > (1ull << 32) ? 0 : (uint32_t)((1ull << 32) / p);
            h_dig[j][k] = 0;
            if (p <= (1ull << 32)) p *= h_base[j];
        }
        h_u[j] = 0;
        h_shift[j] = 0;
        smp_a[j] = fx_q_from_float(lim_a[j]);
        smp_m[j] = (uint16_t)fx_q_from_float(lim_m[j]);
    }
    uint64_t x = 0x9E3779B97F4A7C15ULL * (seed + 1);   // same splitmix64 as src/rrtc.c
    for (int j = 0; seed && j < FX_DIM; ++j) {
        uint64_t z = (x += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        z ^= z >> 31;
        h_shift[j] = (uint32_t)(z >> 40) << 8;
    }
}

static void smp_next(int16_t *q)
{
    for (int j = 0; j < FX_DIM; ++j) {
        const uint32_t b = h_base[j];
        for (int k = 0; k < H_DIGITS; ++k) {
            if (++h_dig[j][k] < b) { h_u[j] += h_inv[j][k]; break; }
            h_dig[j][k] = 0;
            h_u[j] -= (b - 1) * h_inv[j][k];
        }
        uint32_t u = h_u[j] + h_shift[j];                       // rotation wraps mod 2^32
        q[j] = (int16_t)(smp_a[j] + (int32_t)(((uint64_t)(u >> 16) * smp_m[j]) >> 16));
    }
}

// ---------------------------------------------------------------- TREE + NNU
typedef struct {
    int16_t q[FX_NODE_CAP][FX_DIM];
    uint16_t parent[FX_NODE_CAP];
    uint32_t rad[FX_NODE_CAP];
    uint32_t n;
} tree_t;

static tree_t trees[2];

static int64_t dist2(const int16_t *a, const int16_t *b)
{
    int64_t s = 0;
    for (int j = 0; j < FX_DIM; ++j) { int64_t d = (int64_t)a[j] - b[j]; s += d * d; }
    return s;
}

// Streaming brute force: strict < keeps the lowest index on ties
static uint32_t nnu(const tree_t *t, const int16_t *q, int64_t *d2)
{
    int64_t best = INT64_MAX;
    uint32_t bi = 0;
    for (uint32_t i = 0; i < t->n; ++i) {
        int64_t d = dist2(t->q[i], q);
        if (d < best) { best = d; bi = i; }
    }
    *d2 = best;
    return bi;
}

static int tree_insert(tree_t *t, const int16_t *q, uint32_t parent)
{
    if (t->n >= FX_NODE_CAP) return -1;
    memcpy(t->q[t->n], q, sizeof(fxq_t));
    t->parent[t->n] = (uint16_t)parent;
    t->rad[t->n] = FX_RAD_INF;
    return (int)t->n++;
}

// ---------------------------------------------------------------- EVU front end: RKG
uint32_t fx_cur_edge, fx_cur_pose;
static fx_pose_fn g_pose_ok;
static fx_result_t *g_r;

// Poses start + floor(vec * m / (8n)), m = 1..8n, VAMP order m = (i+1)n - k; 1 = edge free
static int edge_ok(const int16_t *start, const int32_t *vec, uint64_t len_h)
{
    g_r->edges++;
    fx_cur_edge++;
    fx_cur_pose = 0;
    uint64_t n = (len_h * (FX_RES / FX_RAKE) + (1ull << FX_HFRAC) - 1) >> FX_HFRAC;   // ceil(len*4)
    if (n < 1) n = 1;
    const int64_t den = (int64_t)(FX_RAKE * n);
    for (uint64_t k = 0; k < n; ++k)
        for (int i = 0; i < FX_RAKE; ++i) {
            int64_t m = (int64_t)((uint64_t)(i + 1) * n - k);
            int16_t p[FX_DIM];
            for (int j = 0; j < FX_DIM; ++j) p[j] = (int16_t)(start[j] + fx_fdiv((int64_t)vec[j] * m, den));
            g_r->poses++;
            int ok = g_pose_ok(p);
            fx_cur_pose++;
            if (!ok) return 0;
        }
    return 1;
}

// ---------------------------------------------------------------- PBT
static uint32_t fnv(uint32_t h, const int16_t *q)
{
    for (int j = 0; j < FX_DIM; ++j) {
        uint16_t u = (uint16_t)q[j];
        h ^= u & 0xffu; h *= 16777619u;
        h ^= u >> 8;    h *= 16777619u;
    }
    return h;
}

static int path_push(fx_result_t *r, const int16_t *q)
{
    if (r->path_len >= FX_PATH_CAP) return -1;
    memcpy(r->path[r->path_len++], q, sizeof(fxq_t));
    return 0;
}

// a-chain reversed from a_last, then b-chain from parent(b_near); reversed if tree a is the goal tree
static int backtrace(fx_result_t *r, const tree_t *ta, uint32_t a_last, const tree_t *tb, uint32_t b_near, int a_is_start)
{
    uint32_t seq[2 * FX_NODE_CAP], na = 0, nb = 0, c;
    for (c = a_last;; c = ta->parent[c]) { seq[na++] = c; if (ta->parent[c] == c) break; }
    uint32_t tot = na;
    for (c = b_near; tb->parent[c] != c; c = tb->parent[c]) { seq[tot++] = tb->parent[c]; ++nb; }
    r->path_len = 0;
    for (uint32_t s = 0; s < tot; ++s) {
        uint32_t k = a_is_start ? s : tot - 1 - s;               // final order: start -> goal
        uint32_t idx = k < na ? seq[na - 1 - k] : seq[k];
        const int16_t *q = k < na ? ta->q[idx] : tb->q[idx];
        if (path_push(r, q)) return -1;
    }
    (void)nb;
    return 0;
}

static void path_finish(fx_result_t *r)
{
    uint32_t h = 2166136261u;
    r->cost = 0.0f;
    for (uint32_t i = 0; i < r->path_len; ++i) {
        h = fnv(h, r->path[i]);
        if (i) r->cost += sqrtf((float)dist2(r->path[i], r->path[i - 1])) / (float)(1 << FX_JFRAC);
    }
    r->path_hash = h;
}

// ---------------------------------------------------------------- CTRL
void fx_default_settings(fx_settings_t *s)
{
    s->range_h = 1u << FX_HFRAC;                     // 1.0 rad
    s->radius = 4u << FX_HFRAC;                      // 4.0 rad
    s->alpha = (uint32_t)(0.0001 * 4294967296.0);    // 1e-4
    s->min_radius = 1u << FX_HFRAC;                  // 1.0 rad
    s->tree_ratio = 1u << 16;                        // 1.0
    s->dynamic_domain = 1;
    s->balance = 1;
    s->max_iterations = 1000000u;
    s->seed = 0;
}

// Length of a joint vector in 2^-24 rad: floor(sqrt(d2 * 2^24))
static uint64_t len_h(int64_t d2) { return fx_isqrt((uint64_t)d2 << FX_HFRAC); }

static void dd_grow(uint32_t *rad, uint32_t alpha)
{
    if (*rad != FX_RAD_INF) *rad += (uint32_t)(((uint64_t)*rad * alpha) >> 32);
}

static void dd_shrink(uint32_t *rad, const fx_settings_t *s)
{
    if (*rad == FX_RAD_INF) { *rad = s->radius; return; }
    uint32_t v = *rad - (uint32_t)(((uint64_t)*rad * s->alpha) >> 32);
    *rad = v > s->min_radius ? v : s->min_radius;
}

void fx_solve(const int16_t *start, const int16_t *goal, const fx_settings_t *s, fx_pose_fn pose_ok, fx_result_t *r)
{
    memset(r, 0, sizeof(*r));
    g_pose_ok = pose_ok;
    g_r = r;
    trees[0].n = trees[1].n = 0;
    smp_reset(s->seed);

    // DIRECT
    int32_t vec[FX_DIM];
    for (int j = 0; j < FX_DIM; ++j) vec[j] = (int32_t)goal[j] - start[j];
    if (edge_ok(start, vec, len_h(dist2(goal, start)))) {
        path_push(r, start);
        path_push(r, goal);
        path_finish(r);
        r->status = FX_SOLVED;
        r->size_start = r->size_goal = 1;
        return;
    }
    tree_insert(&trees[0], start, 0);
    tree_insert(&trees[1], goal, 0);
    tree_t *ta = &trees[1], *tb = &trees[0];       // swapped on the first iteration
    int a_is_start = 0;
    r->status = FX_FAIL_ITER;

    while (r->iterations < s->max_iterations) {
        r->iterations++;
        // BALANCE: |na - nb| < ratio * na (Q16.16)
        uint64_t na = ta->n, nb = tb->n, diff = na > nb ? na - nb : nb - na;
        if (!s->balance || (diff << 16) < (uint64_t)s->tree_ratio * na) {
            tree_t *t = ta; ta = tb; tb = t;
            a_is_start = !a_is_start;
        }
        int16_t q[FX_DIM];
        smp_next(q);
        int64_t nd2;
        uint32_t near = nnu(ta, q, &nd2);
        r->nn_queries++;
        uint32_t nrad = ta->rad[near];
        // DD: reject when rad < nd  <=>  rad^2 < nd2 * 2^24 (exact)
        if (s->dynamic_domain && nrad != FX_RAD_INF && (uint64_t)nrad * nrad < ((uint64_t)nd2 << FX_HFRAC)) {
            r->dd_rejects++;
            continue;
        }
        // EXTEND
        int16_t qn[FX_DIM];
        memcpy(qn, ta->q[near], sizeof(fxq_t));
        uint64_t nd_h = len_h(nd2), elen;
        int reach = nd_h < s->range_h;
        int32_t ext[FX_DIM];
        for (int j = 0; j < FX_DIM; ++j) {
            int64_t d = (int64_t)q[j] - qn[j];
            ext[j] = reach ? (int32_t)d : (int32_t)fx_fdiv(d * (int64_t)s->range_h, (int64_t)nd_h);
        }
        elen = reach ? nd_h : s->range_h;
        if (!edge_ok(qn, ext, elen)) {
            if (s->dynamic_domain) dd_shrink(&ta->rad[near], s);
            continue;
        }
        int16_t qnew[FX_DIM];
        for (int j = 0; j < FX_DIM; ++j) qnew[j] = (int16_t)(qn[j] + ext[j]);
        int ni = tree_insert(ta, qnew, near);
        if (ni < 0) { r->status = FX_FAIL_CAPACITY; break; }
        if (s->dynamic_domain) dd_grow(&ta->rad[near], s->alpha);

        // CONNECT: nodes new + floor(ovec * k / n_ext), k = 1..n_ext, all added to tree a
        int64_t od2;
        uint32_t other = nnu(tb, qnew, &od2);
        r->nn_queries++;
        uint64_t od_h = len_h(od2);
        uint64_t n_ext = (od_h + s->range_h - 1) / s->range_h;
        uint64_t inc_h = n_ext ? od_h / n_ext : 0;
        int32_t ovec[FX_DIM];
        for (int j = 0; j < FX_DIM; ++j) ovec[j] = (int32_t)tb->q[other][j] - qnew[j];
        int16_t prior[FX_DIM];
        memcpy(prior, qnew, sizeof(fxq_t));
        uint32_t prior_i = (uint32_t)ni;
        uint64_t k;
        int cap = 0;
        for (k = 1; k <= n_ext; ++k) {
            int16_t nx[FX_DIM];
            int32_t inc[FX_DIM];
            for (int j = 0; j < FX_DIM; ++j) {
                nx[j] = (int16_t)(qnew[j] + fx_fdiv((int64_t)ovec[j] * (int64_t)k, (int64_t)n_ext));
                inc[j] = (int32_t)nx[j] - prior[j];
            }
            if (!edge_ok(prior, inc, inc_h)) break;
            int xi = tree_insert(ta, nx, prior_i);
            if (xi < 0) { cap = 1; break; }
            prior_i = (uint32_t)xi;
            memcpy(prior, nx, sizeof(fxq_t));
        }
        if (cap) { r->status = FX_FAIL_CAPACITY; break; }
        if (k > n_ext) {
            if (backtrace(r, ta, prior_i, tb, other, a_is_start)) r->status = FX_FAIL_CAPACITY;
            else { r->status = FX_SOLVED; path_finish(r); }
            break;
        }
    }
    r->size_start = trees[0].n;
    r->size_goal = trees[1].n;
}
