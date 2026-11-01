// VCC (Chen & Yeh, ICRA'26) pre-planning and collision checking, scalar version.
#include "env.h"
#include "vmath.h"
#include "stats.h"

// ---------------------------------------------------------------- arena
static uint8_t *a_base;
static uint32_t a_size, a_used;

void arena_init(void *base, uint32_t size) { a_base = base; a_size = size; a_used = 0; }
uint32_t arena_mark(void) { return a_used; }
void arena_release(uint32_t mark) { a_used = mark; }

void *arena_alloc(uint32_t bytes)
{
    uint32_t off = (a_used + 63u) & ~63u;
    if (off + bytes > a_size || off + bytes < off) {
        extern void fatal(const char *msg);
        fatal("arena exhausted");
    }
    a_used = off + bytes;
    return a_base + off;
}

// ---------------------------------------------------------------- point cloud sampling
// Port of vamp/pointcloud.py (geometrout primitive surface sampling); numpy's RNG -> PCG32.
static uint64_t pcg_state;
static uint32_t pcg32(void)
{
    uint64_t old = pcg_state;
    pcg_state = old * 6364136223846793005ULL + 1442695040888963407ULL;
    uint32_t xs = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    uint32_t rot = (uint32_t)(old >> 59u);
    return (xs >> rot) | (xs << ((-rot) & 31));
}
static float urand(void) { return (float)(pcg32() >> 8) * (1.0f / 16777216.0f); }      // [0, 1)
static float urange(float lo, float hi) { return lo + (hi - lo) * urand(); }

static int pick(const float *cum, int n, float u)
{
    for (int i = 0; i < n - 1; ++i)
        if (u < cum[i]) return i;    // np.searchsorted(cumsum, u, side="right")
    return n - 1;
}

static void place(const prim_t *pr, float lx, float ly, float lz, points_t *o)
{
    const float *R = pr->rot;
    uint32_t i = o->n++;
    o->x[i] = R[0] * lx + R[1] * ly + R[2] * lz + pr->pos[0];
    o->y[i] = R[3] * lx + R[4] * ly + R[5] * lz + pr->pos[1];
    o->z[i] = R[6] * lx + R[7] * ly + R[8] * lz + pr->pos[2];
}

static void sample_box(const prim_t *pr, points_t *o)
{
    float d[3] = {pr->dim[0] * 2.0f, pr->dim[1] * 2.0f, pr->dim[2] * 2.0f};
    float a[6] = {d[1] * d[2], d[1] * d[2], d[0] * d[2], d[0] * d[2], d[0] * d[1], d[0] * d[1]};
    float sum = a[0] + a[1] + a[2] + a[3] + a[4] + a[5], cum[6], c = 0.0f;
    for (int i = 0; i < 6; ++i) { c += a[i] / sum; cum[i] = c; }
    for (int k = 0; k < VCC_SAMPLES; ++k) {
        float p[3];
        for (int j = 0; j < 3; ++j) p[j] = urange(-1.0f, 1.0f) * d[j] * 0.5f;
        int side = pick(cum, 6, urand());
        p[side >> 1] = (side & 1) ? -d[side >> 1] * 0.5f : d[side >> 1] * 0.5f;
        place(pr, p[0], p[1], p[2], o);
    }
}

static void sample_cylinder(const prim_t *pr, points_t *o)
{
    const float pi = 3.14159265359f;
    float r = pr->dim[0], h = pr->dim[1];
    float cap = pi * r * r, side = h * 2.0f * pi * r, area = 2.0f * cap + side;
    float cum[3] = {cap / area, cap / area + side / area, 1.0f};
    for (int k = 0; k < VCC_SAMPLES; ++k) {
        float ang = urange(-pi, pi);
        float cx = vamp_cosf(ang), cy = vamp_sinf(ang), z;
        int s = pick(cum, 3, urand());
        if (s == 1) {
            cx *= r; cy *= r;
            z = urange(-h * 0.5f, h * 0.5f);
        } else {
            float rr = urange(0.0f, r);   // VAMP scales caps linearly in r (not sqrt), kept as is
            cx *= rr; cy *= rr;
            z = (s == 0) ? -h * 0.5f : h * 0.5f;
        }
        place(pr, cx, cy, z, o);
    }
}

void pc_sample(const problem_t *p, points_t *out)
{
    uint32_t n = (uint32_t)p->n_prims * VCC_SAMPLES;
    out->x = arena_alloc(n * 4); out->y = arena_alloc(n * 4); out->z = arena_alloc(n * 4);
    out->n = 0;
    pcg_state = 0x853c49e6748fea9bULL ^ ((uint64_t)p->scene << 32) ^ p->index;
    // VAMP order: cylinders first, then boxes
    for (int t = PRIM_CYLINDER; t >= PRIM_BOX; --t)
        for (int i = 0; i < p->n_prims; ++i) {
            const prim_t *pr = &prims[p->prim_off + i];
            if (pr->type != t) continue;
            if (t == PRIM_BOX) sample_box(pr, out); else sample_cylinder(pr, out);
        }
}

// ---------------------------------------------------------------- three-level table helpers
static int32_t *tbl_slot(int32_t *pool, uint32_t *used, int G, int vx, int vy, int vz, int create)
{
    int32_t *X = pool;
    if (X[vx] < 0) {
        if (!create) return 0;
        X[vx] = (int32_t)*used;
        for (int i = 0; i < G; ++i) pool[*used + i] = -1;
        *used += G;
    }
    int32_t *Y = pool + X[vx];
    if (Y[vy] < 0) {
        if (!create) return 0;
        Y[vy] = (int32_t)*used;
        for (int i = 0; i < G; ++i) pool[*used + i] = -1;
        *used += G;
    }
    return pool + Y[vy] + vz;
}

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------- CenterVox
void centervox(const points_t *raw, points_t *out)
{
    const float wmin = -VCC_WS_HALF, width = 2.0f * VCC_WS_HALF;
    const int G = (int)vfloorf(width / VCC_L_FILTER);
    const float s = (float)G / width, L = width / (float)G;
    uint32_t mark = arena_mark();
    uint32_t cap = (uint32_t)(G + G * G + G * G * G);
    int32_t *pool = arena_alloc(cap * 4);
    uint32_t used = (uint32_t)G;
    for (int i = 0; i < G; ++i) pool[i] = -1;
    // kept point per voxel: index into raw + its squared distance to the voxel centre
    uint32_t *keep = arena_alloc(raw->n * 4);
    float *kd = arena_alloc(raw->n * 4);
    uint32_t nk = 0;
    for (uint32_t i = 0; i < raw->n; ++i) {
        float p[3] = {raw->x[i], raw->y[i], raw->z[i]};
        if (p[0] < wmin || p[1] < wmin || p[2] < wmin ||
            p[0] > wmin + width || p[1] > wmin + width || p[2] > wmin + width)
            continue;
        int v[3];
        float d2 = 0.0f;
        for (int k = 0; k < 3; ++k) {
            v[k] = clampi((int)((p[k] - wmin) * s), 0, G - 1);
            float c = wmin + ((float)v[k] + 0.5f) * L, e = p[k] - c;
            d2 = d2 + e * e;
        }
        int32_t *slot = tbl_slot(pool, &used, G, v[0], v[1], v[2], 1);
        if (*slot < 0) {
            *slot = (int32_t)nk;
            keep[nk] = i; kd[nk] = d2; nk++;
        } else if (d2 < kd[*slot]) {
            keep[*slot] = i; kd[*slot] = d2;
        }
    }
    // results go below the scratch so it can be released
    float *x = arena_alloc(nk * 4), *y = arena_alloc(nk * 4), *z = arena_alloc(nk * 4);
    for (uint32_t k = 0; k < nk; ++k) { x[k] = raw->x[keep[k]]; y[k] = raw->y[keep[k]]; z[k] = raw->z[keep[k]]; }
    // compact: move the result to the mark (memmove semantics, regions may overlap)
    arena_release(mark);
    out->x = arena_alloc(nk * 4); out->y = arena_alloc(nk * 4); out->z = arena_alloc(nk * 4);
    for (uint32_t k = 0; k < nk; ++k) out->x[k] = x[k];
    for (uint32_t k = 0; k < nk; ++k) out->y[k] = y[k];
    for (uint32_t k = 0; k < nk; ++k) out->z[k] = z[k];
    out->n = nk;
}

// ---------------------------------------------------------------- MVT
void mvt_build(mvt_t *t, const points_t *pts, float r_max)
{
    t->wmin = -VCC_WS_HALF;
    t->width = 2.0f * VCC_WS_HALF;
    t->G = (int)vfloorf(t->width / (r_max + VCC_POINT_RADIUS));   // Eq. 1
    t->s = (float)t->G / t->width;                                // Eq. 4
    const int G = t->G;
    t->pool_cap = (uint32_t)(G + G * G + G * G * G);
    t->pool = arena_alloc(t->pool_cap * 4);
    for (int i = 0; i < G; ++i) t->pool[i] = -1;
    t->pool_used = (uint32_t)G;
    t->vox_cap = pts->n ? pts->n : 1;
    t->vox = arena_alloc(t->vox_cap * sizeof(voxel_t));
    t->n_vox = 0;
    uint32_t *map = arena_alloc((pts->n ? pts->n : 1) * 4);
    for (int k = 0; k < 3; ++k) { t->glo[k] = 3.0e38f; t->ghi[k] = -3.0e38f; }

    // Phase 1: tables, voxel structures, per-point voxel index cache
    for (uint32_t i = 0; i < pts->n; ++i) {
        float p[3] = {pts->x[i], pts->y[i], pts->z[i]};
        int v[3];
        for (int k = 0; k < 3; ++k) {
            v[k] = clampi((int)((p[k] - t->wmin) * t->s), 0, G - 1);   // Eq. 2
            if (p[k] < t->glo[k]) t->glo[k] = p[k];
            if (p[k] > t->ghi[k]) t->ghi[k] = p[k];
        }
        int32_t *slot = tbl_slot(t->pool, &t->pool_used, G, v[0], v[1], v[2], 1);
        if (*slot < 0) {
            voxel_t *vx = &t->vox[t->n_vox];
            vx->count = 0;
            for (int k = 0; k < 3; ++k) { vx->lo[k] = 3.0e38f; vx->hi[k] = -3.0e38f; }
            *slot = (int32_t)t->n_vox++;
        }
        t->vox[*slot].count++;
        map[i] = (uint32_t)*slot;
    }
    // Phase 2: SIMD-padded capacities, then distribute coordinates
    uint32_t total = 0;
    for (uint32_t j = 0; j < t->n_vox; ++j) {
        t->vox[j].off = total;
        total += (t->vox[j].count + VCC_PAD - 1) / VCC_PAD * VCC_PAD;
    }
    t->n_pool = total;
    t->px = arena_alloc((total ? total : 1) * 4);
    t->py = arena_alloc((total ? total : 1) * 4);
    t->pz = arena_alloc((total ? total : 1) * 4);
    const float inf = u2f(0x7f800000u);
    for (uint32_t k = 0; k < total; ++k) { t->px[k] = inf; t->py[k] = inf; t->pz[k] = inf; }
    for (uint32_t j = 0; j < t->n_vox; ++j) t->vox[j].count = 0;
    for (uint32_t i = 0; i < pts->n; ++i) {
        voxel_t *vx = &t->vox[map[i]];
        uint32_t at = vx->off + vx->count++;
        float p[3] = {pts->x[i], pts->y[i], pts->z[i]};
        t->px[at] = p[0]; t->py[at] = p[1]; t->pz[at] = p[2];
        for (int k = 0; k < 3; ++k) {
            if (p[k] < vx->lo[k]) vx->lo[k] = p[k];
            if (p[k] > vx->hi[k]) vx->hi[k] = p[k];
        }
    }
}

static inline float aabb_distsq(const float *lo, const float *hi, float cx, float cy, float cz)
{
    float c[3] = {cx, cy, cz}, d2 = 0.0f;
    for (int k = 0; k < 3; ++k) {
        float e = 0.0f;
        if (c[k] < lo[k]) e = lo[k] - c[k];
        else if (c[k] > hi[k]) e = c[k] - hi[k];
        d2 = d2 + e * e;
    }
    return d2;
}

// Algorithm 1 (scalar: one sphere per call)
int mvt_sphere_collides(const mvt_t *t, float cx, float cy, float cz, float r)
{
    const float rp = r + VCC_POINT_RADIUS;                                      // line 1
    if (cx + rp < t->glo[0] || cx - rp > t->ghi[0] || cy + rp < t->glo[1] ||    // line 2
        cy - rp > t->ghi[1] || cz + rp < t->glo[2] || cz - rp > t->ghi[2]) {
        st.env_global_culled++;
        return 0;
    }
    const float rsq = rp * rp, rv = rp * t->s;                                  // line 6
    const float v[3] = {(cx - t->wmin) * t->s, (cy - t->wmin) * t->s, (cz - t->wmin) * t->s};  // line 5
    int bmin[3], bmax[3];
    for (int k = 0; k < 3; ++k) {                                               // lines 8-9
        bmin[k] = clampi((int)vfloorf(v[k] - rv), 0, t->G - 1);
        bmax[k] = clampi((int)vfloorf(v[k] + rv), 0, t->G - 1);
    }
    const int32_t *pool = t->pool;
    for (int x = bmin[0]; x <= bmax[0]; ++x) {
        int32_t oy = pool[x];
        if (oy < 0) continue;                                                   // line 11
        const int32_t *Y = pool + oy;
        for (int y = bmin[1]; y <= bmax[1]; ++y) {
            int32_t oz = Y[y];
            if (oz < 0) continue;                                               // line 14
            const int32_t *Z = pool + oz;
            for (int z = bmin[2]; z <= bmax[2]; ++z) {
                int32_t idx = Z[z];
                if (idx < 0) continue;                                          // line 17
                const voxel_t *vx = &t->vox[idx];
                st.voxels++;
                if (aabb_distsq(vx->lo, vx->hi, cx, cy, cz) > rsq) {           // line 19
                    st.voxels_culled++;
                    continue;
                }
                const float *px = t->px + vx->off, *py = t->py + vx->off, *pz = t->pz + vx->off;
                for (uint32_t i = 0; i < vx->count; ++i) {                     // lines 21-22
                    float dx = px[i] - cx, dy = py[i] - cy, dz = pz[i] - cz;
                    st.points++;
                    if (dx * dx + dy * dy + dz * dz <= rsq) return 1;
                }
            }
        }
    }
    return 0;
}

int brute_sphere_collides(const points_t *pts, float cx, float cy, float cz, float r)
{
    const float rp = r + VCC_POINT_RADIUS, rsq = rp * rp;
    for (uint32_t i = 0; i < pts->n; ++i) {
        float dx = pts->x[i] - cx, dy = pts->y[i] - cy, dz = pts->z[i] - cz;
        if (dx * dx + dy * dy + dz * dz <= rsq) return 1;
    }
    return 0;
}
