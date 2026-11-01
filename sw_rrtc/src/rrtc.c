// Port of VAMP's RRTC (planning/rrtc.hh), validate_vector (planning/validate.hh) and Halton
// sampler (random/halton.hh), keeping VAMP's float operation order.
#include "rrtc.h"
#include "cc.h"
#include "env.h"
#include "stats.h"
#include "timer.h"
#include "vmath.h"

#define RAKE 8
#define FMAX 3.40282347e+38f

// ---------------------------------------------------------------- Halton
static const float primes[16] = {3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59};
static float hb[PANDA_DIM], hn[PANDA_DIM], hd[PANDA_DIM], hshift[PANDA_DIM];
static uint32_t h_iter;

void halton_reset(uint32_t seed)
{
    for (int j = 0; j < PANDA_DIM; ++j) { hb[j] = primes[j]; hn[j] = 0.0f; hd[j] = 1.0f; hshift[j] = 0.0f; }
    h_iter = 0;
    uint64_t x = 0x9E3779B97F4A7C15ULL * (seed + 1);   // splitmix64 for the per-seed rotation
    for (int j = 0; seed && j < PANDA_DIM; ++j) {
        uint64_t z = (x += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        z ^= z >> 31;
        hshift[j] = (float)(uint32_t)(z >> 40) * (1.0f / 16777216.0f);
    }
}

void halton_next(float *q)
{
    if (++h_iter > 1000000u) {   // VAMP: precision limit, restart with rotated bases
        float b0 = hb[0];
        for (int j = 0; j < PANDA_DIM - 1; ++j) hb[j] = hb[j + 1];
        hb[PANDA_DIM - 1] = b0;
        for (int j = 0; j < PANDA_DIM; ++j) { hn[j] = 0.0f; hd[j] = 1.0f; }
        h_iter = 0;
    }
    for (int j = 0; j < PANDA_DIM; ++j) {
        float b = hb[j], n = hn[j], d = hd[j];
        float xf = d - n;
        if (xf == 1.0f) {
            d = vfloorf(d * b);
            n = 1.0f;
        } else {
            float y = vfloorf(d / b);
            while (xf <= y) y = vfloorf(y / b);
            n = vfloorf((b + 1.0f) * y) - xf;
        }
        hn[j] = n; hd[j] = d;
        float u = n / d;
        if (hshift[j] != 0.0f) { u = u + hshift[j]; if (u >= 1.0f) u = u - 1.0f; }
        q[j] = u * panda_s_m[j] + panda_s_a[j];
    }
    q[PANDA_DIM] = 0.0f;
}

// ---------------------------------------------------------------- validate_vector
int validate_vector(const float *start, const float *vec, float distance)
{
    float block[RAKE][PANDA_DIM], back[PANDA_DIM];
    st.edges++;
    for (int i = 0; i < RAKE; ++i) {
        const float pct = (float)(i + 1) / (float)RAKE;
        for (int j = 0; j < PANDA_DIM; ++j) block[i][j] = start[j] + (vec[j] * pct);
    }
    float nf = vceilf(distance / (float)RAKE * (float)PANDA_RESOLUTION);
    uint32_t n = nf > 1.0f ? (uint32_t)nf : 1u;
    for (int j = 0; j < PANDA_DIM; ++j) back[j] = vec[j] / (float)(RAKE * n);
    int ok = 1;
    uint32_t blocks = 0;
    ENVSTAT(envstat_edge_begin());
    for (uint32_t k = 0; k < n && ok; ++k) {
        if (k)
            for (int i = 0; i < RAKE; ++i)
                for (int j = 0; j < PANDA_DIM; ++j) block[i][j] = block[i][j] - back[j];
        ++blocks;
        for (int i = 0; i < RAKE && ok; ++i) ok = cc_validate_pose(block[i]);
    }
    st.checks += (blocks * RAKE + 31) / 32;    // CCU CHECK carries up to 32 poses
    ENVSTAT(envstat_edge_end(start, vec, n, blocks));
    return ok;
}

// ---------------------------------------------------------------- RRTC
void rrtc_default_settings(rrtc_settings_t *s)
{
    s->range = 1.0f;
    s->dynamic_domain = 1;
    s->radius = 4.0f;
    s->alpha = 0.0001f;
    s->min_radius = 1.0f;
    s->balance = 1;
    s->tree_ratio = 1.0f;
    s->max_iterations = 1000000u;
    s->max_samples = 1000000u;
    s->nn.mode = NN_BRUTE; s->nn.m = 0; s->nn.W = 8; s->nn.B = 32; s->nn.bits = 9;
    s->shadow = 0;
    s->seed = 0;
}

static float *buf;
static uint32_t *parents;
static float *radii;
static const rrtc_settings_t *S;
static shadow_t *SH;

static inline float *node(uint32_t i) { return buf + i * NN_STRIDE; }

static uint32_t timed_nearest(nn_tree_t *t, const float *q, float *dist, int qtype)
{
    st.nn_queries[qtype]++;
#if TIMING
    uint32_t t0 = tnow();
#endif
    uint32_t g = nn_nearest(t, q, dist);
#if TIMING
    st.cyc_nn_query += (uint32_t)(tnow() - t0);
    st.timer_reads += 2;
#endif
    return g;
}

static void shadow_score(const nn_tree_t *t, const float *q, int qtype)
{
    if (!S->shadow) return;
    uint64_t keep = st.nn_dist_evals;
    float ed2;
    nn_nearest_exact(t, q, &ed2);
    st.nn_dist_evals = keep;
    nn_shadow(t, q, ed2, qtype, SH);
}

static void timed_insert(nn_tree_t *t, uint32_t g)
{
#if TIMING
    uint32_t t0 = tnow();
#endif
    nn_insert(t, g);
#if TIMING
    st.cyc_nn_insert += (uint32_t)(tnow() - t0);
    st.timer_reads += 2;
#endif
}

static void vsub(float *o, const float *a, const float *b) { for (int j = 0; j < NN_STRIDE; ++j) o[j] = a[j] - b[j]; }
static void vadd(float *o, const float *a, const float *b) { for (int j = 0; j < NN_STRIDE; ++j) o[j] = a[j] + b[j]; }
static void vscale(float *o, const float *a, float k) { for (int j = 0; j < NN_STRIDE; ++j) o[j] = a[j] * k; }

static uint32_t fnv(uint32_t h, const float *q)
{
    for (int j = 0; j < PANDA_DIM; ++j) {
        uint32_t u = f2u(q[j]);
        for (int b = 0; b < 4; ++b) { h ^= (u >> (8 * b)) & 0xffu; h *= 16777619u; }
    }
    return h;
}

// VAMP's path: reverse(a-chain from a_last) + b-chain from parent(b_near); reversed if a is the goal tree
static void walk_path(rrtc_result_t *r, uint32_t a_last, uint32_t b_near, int a_is_start)
{
    uint32_t n = 0, c;
    for (c = a_last;; c = parents[c]) { ++n; if (parents[c] == c) break; }
    for (c = b_near; parents[c] != c; c = parents[c]) ++n;
    uint32_t *seq = arena_alloc(n * 4), k = 0;
    for (c = a_last;; c = parents[c]) { seq[k++] = c; if (parents[c] == c) break; }
    for (uint32_t i = 0; i < k / 2; ++i) { uint32_t t = seq[i]; seq[i] = seq[k - 1 - i]; seq[k - 1 - i] = t; }
    for (c = b_near; parents[c] != c; c = parents[c]) seq[k++] = parents[c];
    if (!a_is_start)
        for (uint32_t i = 0; i < n / 2; ++i) { uint32_t t = seq[i]; seq[i] = seq[n - 1 - i]; seq[n - 1 - i] = t; }
    r->path_len = n;
    r->cost = 0.0f;
    uint32_t h = 2166136261u;
    for (uint32_t i = 0; i < n; ++i) {
        h = fnv(h, node(seq[i]));
        if (i) r->cost = r->cost + vsqrtf(nn_dist2(node(seq[i]), node(seq[i - 1])));
    }
    r->path_hash = h;
}

void rrtc_solve(const float *start, const float *goal, const rrtc_settings_t *s, rrtc_result_t *r, shadow_t *sh)
{
    S = s; SH = sh;
    r->solved = 0; r->cap_hit = 0; r->path_len = 0; r->cost = 0.0f; r->path_hash = 0;
    const uint32_t cap = s->max_samples;
    buf = arena_alloc(cap * NN_STRIDE * 4);
    parents = arena_alloc(cap * 4);
    radii = arena_alloc(cap * 4);
    nn_shared_alloc(&s->nn, cap);
    nn_tree_t start_tree, goal_tree;
    nn_init(&start_tree, &s->nn, buf, 0, cap);
    nn_init(&goal_tree, &s->nn, buf, 1, cap);
    halton_reset(s->seed);

    float sv[NN_STRIDE] = {0}, gv[NN_STRIDE] = {0}, tmp[NN_STRIDE];
    for (int j = 0; j < PANDA_DIM; ++j) { sv[j] = start[j]; gv[j] = goal[j]; }

    vsub(tmp, gv, sv);
    if (validate_vector(sv, tmp, vsqrtf(nn_dist2(gv, sv)))) {
        r->solved = 1; r->size_start = 1; r->size_goal = 1; r->path_len = 2;
        r->cost = vsqrtf(nn_dist2(gv, sv));
        r->path_hash = fnv(fnv(2166136261u, sv), gv);
        return;
    }

    nn_tree_t *ta = &goal_tree, *tb = &start_tree;   // start_tree_first: swapped on iteration 1
    int a_is_start = 0;
    uint32_t iter = 0, free_i = 0;
    for (int j = 0; j < NN_STRIDE; ++j) node(0)[j] = sv[j];
    timed_insert(&start_tree, 0);
    parents[0] = 0; radii[0] = FMAX;
    for (int j = 0; j < NN_STRIDE; ++j) node(1)[j] = gv[j];
    timed_insert(&goal_tree, 1);
    parents[1] = 1; radii[1] = FMAX;
    free_i = 2;

    while (iter++ < s->max_iterations && free_i < cap) {
        float asize = (float)ta->n, bsize = (float)tb->n;
        float ratio = (asize > bsize ? asize - bsize : bsize - asize) / asize;
        if (!s->balance || ratio < s->tree_ratio) {
            nn_tree_t *t = ta; ta = tb; tb = t;
            a_is_start = !a_is_start;
        }
        float q[NN_STRIDE];
#if TIMING
        uint32_t tr = tnow();
#endif
        halton_next(q);
#if TIMING
        st.cyc_rng += (uint32_t)(tnow() - tr);
        st.timer_reads += 2;
#endif
        float nd;
        uint32_t near = timed_nearest(ta, q, &nd, QT_EXTEND);
        float nrad = radii[near];
        int rejected = s->dynamic_domain && nrad < nd;
        shadow_score(ta, q, rejected ? QT_EXTEND_REJECTED : QT_EXTEND);
        if (rejected) { st.dd_rejects++; continue; }

        const float *nc = node(near);
        float nvec[NN_STRIDE], ext[NN_STRIDE];
        vsub(nvec, q, nc);
        int reach = nd < s->range;
        if (reach) for (int j = 0; j < NN_STRIDE; ++j) ext[j] = nvec[j];
        else vscale(ext, nvec, s->range / nd);

        if (validate_vector(nc, ext, reach ? nd : s->range)) {
            float *newc = node(free_i);
            vadd(newc, nc, ext);
            timed_insert(ta, free_i);
            parents[free_i] = near;
            radii[free_i] = FMAX;
            uint32_t new_i = free_i++;
            if (s->dynamic_domain && nrad != FMAX) radii[near] = radii[near] * (1.0f + s->alpha);

            float od;
            uint32_t other = timed_nearest(tb, newc, &od, QT_CONNECT);
            shadow_score(tb, newc, QT_CONNECT);
            const float *oc = node(other);
            float ovec[NN_STRIDE], inc[NN_STRIDE], prior[NN_STRIDE];
            vsub(ovec, oc, newc);
            uint32_t n_ext = (uint32_t)vceilf(od / s->range);
            float inc_len = od / (float)n_ext;
            vscale(inc, ovec, 1.0f / (float)n_ext);
            for (int j = 0; j < NN_STRIDE; ++j) prior[j] = newc[j];
            uint32_t i_ext = 0;
            for (; i_ext < n_ext && validate_vector(prior, inc, inc_len) && free_i < cap; ++i_ext) {
                float *nx = node(free_i);
                vadd(nx, prior, inc);
                timed_insert(ta, free_i);
                parents[free_i] = free_i - 1;
                radii[free_i] = FMAX;
                free_i++;
                for (int j = 0; j < NN_STRIDE; ++j) prior[j] = nx[j];
            }
            (void)new_i;
            if (i_ext == n_ext) {
                r->solved = 1;
                walk_path(r, free_i - 1, other, a_is_start);
                break;
            }
        } else if (s->dynamic_domain) {
            if (nrad == FMAX) radii[near] = s->radius;
            else {
                float v = radii[near] * (1.0f - s->alpha);
                radii[near] = v > s->min_radius ? v : s->min_radius;
            }
        }
    }
    st.iterations = iter;
    r->cap_hit = free_i >= cap;
    r->size_start = start_tree.n;
    r->size_goal = goal_tree.n;
}
