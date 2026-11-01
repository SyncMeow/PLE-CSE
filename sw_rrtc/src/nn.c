#include "nn.h"
#include "env.h"
#include "stats.h"
#include "vmath.h"
#include <math.h>

// VAMP squared_l2_norm on one AVX register: ((e0+e4)+(e2+e6)) + ((e1+e5)+(e3+e7)), e7 = pad = 0
float nn_dist2(const float *a, const float *b)
{
    float p[8];
    for (int i = 0; i < 7; ++i) { float e = a[i] - b[i]; p[i] = e * e; }
    p[7] = 0.0f;
    float s0 = p[4] + p[0], s1 = p[5] + p[1], s2 = p[6] + p[2], s3 = p[7] + p[3];
    return (s0 + s2) + (s1 + s3);
}

// ---------------------------------------------------------------- shared storage
static uint32_t *g_gid;
static uint64_t *g_key[NN_MAX_SHIFTS];
static uint32_t *g_ord[NN_MAX_SHIFTS];
static uint64_t spread[512];

void nn_shared_alloc(const nn_params_t *p, uint32_t cap)
{
    g_gid = arena_alloc(cap * 4);
    for (int t = 0; t < p->m; ++t) {
        g_key[t] = arena_alloc(cap * 8);
        g_ord[t] = arena_alloc(cap * 4);
    }
    // bit k of a coordinate goes to bit 7k of the key
    for (uint32_t c = 0; c < 512; ++c) {
        uint64_t v = 0;
        for (int k = 0; k < 9; ++k)
            if (c & (1u << k)) v |= 1ULL << (7 * k);
        spread[c] = v;
    }
}

void nn_init(nn_tree_t *t, const nn_params_t *p, const float *buf, int side, uint32_t cap)
{
    t->buf = buf;
    t->n = 0;
    t->n_sorted = 0;
    t->mode = p->mode; t->m = p->m; t->W = p->W; t->B = p->B; t->bits = p->bits;
    t->gdir = side ? -1 : 1;
    t->gid = side ? g_gid + cap - 1 : g_gid;
    for (int s = 0; s < p->m; ++s) {
        t->s[s].dir = side ? -1 : 1;
        t->s[s].k = side ? g_key[s] + cap - 1 : g_key[s];
        t->s[s].o = side ? g_ord[s] + cap - 1 : g_ord[s];
    }
}

#define GID(t, i)  ((t)->gid[(t)->gdir * (int32_t)(i)])
#define KEY(a, i)  ((a)->k[(a)->dir * (int32_t)(i)])
#define ORD(a, i)  ((a)->o[(a)->dir * (int32_t)(i)])
#define NODE(t, i) ((t)->buf + (uint32_t)GID(t, i) * NN_STRIDE)

// ---------------------------------------------------------------- Morton keys
static uint64_t morton_key(const nn_tree_t *t, const float *q, int shift)
{
    const float off = (float)shift / (float)(PANDA_DIM + 1);   // Chan's shifts t / (d + 1)
    const float scale = (float)(1u << t->bits);
    const int cmax = (1 << t->bits) - 1;
    uint64_t key = 0;
    for (int j = 0; j < PANDA_DIM; ++j) {
        float u = (q[j] - panda_s_a[j]) * panda_d_m[j];          // joint limits -> [0, 1]
        float us = (u + off) * 0.5f;                              // shifted copy inside [0, 1)
        int c = (int)(us * scale);
        c = c < 0 ? 0 : (c > cmax ? cmax : c);
        key |= spread[c << (9 - t->bits)] << (PANDA_DIM - 1 - j);
    }
    return key;
}

static uint32_t lower_bound(const dearr_t *a, uint32_t n, uint64_t key)
{
    uint32_t lo = 0, hi = n;
    while (lo < hi) {
        uint32_t mid = (lo + hi) >> 1;
        if (KEY(a, mid) < key) lo = mid + 1; else hi = mid;
    }
    return lo;
}

static void merge_buffer(nn_tree_t *t)
{
    const uint32_t ns = t->n_sorted, nb = t->n - ns;
    uint64_t bk[64];
    uint32_t bo[64];
    st.morton_merges++;
    for (int s = 0; s < t->m; ++s) {
        dearr_t *a = &t->s[s];
        for (uint32_t j = 0; j < nb; ++j) {        // insertion sort of the buffer, stable
            uint64_t k = morton_key(t, NODE(t, ns + j), s);
            uint32_t i = j;
            while (i > 0 && bk[i - 1] > k) { bk[i] = bk[i - 1]; bo[i] = bo[i - 1]; --i; }
            bk[i] = k; bo[i] = ns + j;
        }
        int32_t i = (int32_t)ns - 1, j = (int32_t)nb - 1, k = (int32_t)(ns + nb) - 1;
        while (j >= 0) {                           // backward in-place merge, buffer after ties
            if (i >= 0 && KEY(a, i) > bk[j]) { KEY(a, k) = KEY(a, i); ORD(a, k) = ORD(a, i); --i; }
            else { KEY(a, k) = bk[j]; ORD(a, k) = bo[j]; --j; }
            --k;
        }
    }
    t->n_sorted = t->n;
}

// Merge whatever sits in the buffer (host experiments: every node in the sorted orders)
void nn_flush(nn_tree_t *t)
{
    if (t->m > 0 && t->n > t->n_sorted) merge_buffer(t);
}

void nn_insert(nn_tree_t *t, uint32_t gid)
{
    GID(t, t->n) = gid;
    t->n++;
    st.nn_inserts++;
    if (t->m > 0 && t->n - t->n_sorted >= (uint32_t)t->B)
        merge_buffer(t);
}

// ---------------------------------------------------------------- queries
static inline void consider(const nn_tree_t *t, const float *q, uint32_t li, float *bd, uint32_t *bi)
{
    float d2 = nn_dist2(NODE(t, li), q);
    if (d2 < *bd || (d2 == *bd && li < *bi)) { *bd = d2; *bi = li; }
}

static uint32_t nn_exact_evals;

uint32_t nn_nearest_exact(const nn_tree_t *t, const float *q, float *dist2)
{
    float bd = 3.0e38f;
    uint32_t bi = 0;
    for (uint32_t li = 0; li < t->n; ++li) {
        float d2 = nn_dist2(NODE(t, li), q);
        if (d2 < bd) { bd = d2; bi = li; }
    }
    *dist2 = bd;
    nn_exact_evals += t->n;
    return bi;
}

uint32_t nn_nearest(nn_tree_t *t, const float *q, float *dist)
{
    nn_exact_evals = 0;
    float bd = 3.0e38f;
    uint32_t bi = 0xffffffffu;
    if (t->mode == NN_BRUTE) {
        bi = nn_nearest_exact(t, q, &bd);
    } else {
        for (uint32_t li = t->n_sorted; li < t->n; ++li) consider(t, q, li, &bd, &bi);
        for (int s = 0; s < t->m; ++s) {
            const dearr_t *a = &t->s[s];
            uint32_t pos = lower_bound(a, t->n_sorted, morton_key(t, q, s));
            uint32_t lo = pos > (uint32_t)t->W ? pos - (uint32_t)t->W : 0;
            uint32_t hi = pos + (uint32_t)t->W < t->n_sorted ? pos + (uint32_t)t->W : t->n_sorted;
            for (uint32_t j = lo; j < hi; ++j) consider(t, q, ORD(a, j), &bd, &bi);
            st.nn_dist_evals += hi - lo;
        }
        st.nn_dist_evals += t->n - t->n_sorted;
    }
    st.nn_dist_evals += nn_exact_evals;
    *dist = vsqrtf(bd);
    return GID(t, bi);
}

static int ratio_bin(float best, float exact)
{
    if (best <= exact) return 0;
    if (exact <= 0.0f) return SH_BINS - 1;
    int b = (int)(log2f(best / exact) * 8.0f) + 1;   // log2(sqrt(best / exact)) in 1/16 steps
    return b >= SH_BINS - 1 ? SH_BINS - 1 : b;
}

void nn_shadow(const nn_tree_t *t, const float *q, float exact_d2, int qtype, shadow_t *sh)
{
    float buf_best = 3.0e38f;
    uint32_t bi = 0xffffffffu;
    for (uint32_t li = t->n_sorted; li < t->n; ++li) consider(t, q, li, &buf_best, &bi);
    float best[NN_MAX_SHIFTS][SH_NW];
    for (int s = 0; s < sh_m[SH_NM - 1]; ++s) {
        const dearr_t *a = &t->s[s];
        uint32_t pos = lower_bound(a, t->n_sorted, morton_key(t, q, s));
        float b = 3.0e38f;
        int w0 = 0;
        for (int wi = 0; wi < SH_NW; ++wi) {     // grow the window [pos - w, pos + w)
            for (int w = w0; w < sh_W[wi]; ++w) {
                if (pos >= (uint32_t)w + 1) { float d = nn_dist2(NODE(t, ORD(a, pos - w - 1)), q); if (d < b) b = d; }
                if (pos + (uint32_t)w < t->n_sorted) { float d = nn_dist2(NODE(t, ORD(a, pos + w)), q); if (d < b) b = d; }
            }
            w0 = sh_W[wi];
            best[s][wi] = b;
        }
    }
    for (int mi = 0; mi < SH_NM; ++mi)
        for (int wi = 0; wi < SH_NW; ++wi) {
            float b = buf_best;
            for (int s = 0; s < sh_m[mi]; ++s) if (best[s][wi] < b) b = best[s][wi];
            sh->queries[qtype][mi][wi]++;
            sh->hits[qtype][mi][wi] += (b <= exact_d2);
            sh->hist[qtype][mi][wi][ratio_bin(b, exact_d2)]++;
        }
}
