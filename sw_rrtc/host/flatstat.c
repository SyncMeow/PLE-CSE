// Host-only, design-independent workload measurements on a flat robot-sphere query stream:
// every pose the planner checks is also queried with all 59 robot spheres (no link bounding
// spheres, no hierarchy), in VAMP's sphere order. The planner itself is unchanged (a link's
// bounding sphere encloses its robot spheres, so validity does not depend on the hierarchy).
//  (3) voxel indexing: cells / non-empty voxels per query, blocks touched and occupied for
//      block sides 2, 4, 8, AABB-passing voxels and points tested.
//  (2) batch ordering (-R): batches of P = 8/16/32 poses (an edge has at most 32) rebuilt from each edge, three orders
//      (pose-major, sphere-major, Morton of the centre voxel): position of the first colliding
//      query, and (P = 32) LRU stack (reuse) distances of voxel references, full and early-exit.
#include <stdlib.h>
#include <string.h>
#include "flatstat.h"
#include "cc.h"
#include "panda.h"
#include "vmath.h"

flatstat_t fs;
int fs_reuse_on;

static const uint8_t FINE_ORDER[PANDA_N_SPHERES] = {
    57, 58, 55, 56, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 32, 33, 34, 35, 36, 29, 30,
    31, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 13, 14, 15, 16, 9, 10, 11, 12, 5, 6, 7, 8, 1, 2, 3, 4, 0};

// ---------------------------------------------------------------- grid helpers
static const mvt_t *cur;
#define NBS 3
static const int BS_SIDE[NBS] = {2, 4, 8};
static uint8_t occ[NBS][16 * 16 * 16];
static int nbk[NBS];

static int32_t voxel_at(const mvt_t *t, int x, int y, int z)
{
    int32_t oy = t->pool[x];
    if (oy < 0) return -1;
    int32_t oz = t->pool[oy + y];
    if (oz < 0) return -1;
    return t->pool[oz + z];
}

static void prepare(const mvt_t *t)
{
    cur = t;
    for (int b = 0; b < NBS; ++b) {
        nbk[b] = (t->G + BS_SIDE[b] - 1) / BS_SIDE[b];
        memset(occ[b], 0, sizeof(occ[b]));
    }
    for (int x = 0; x < t->G; ++x)
        for (int y = 0; y < t->G; ++y)
            for (int z = 0; z < t->G; ++z)
                if (voxel_at(t, x, y, z) >= 0)
                    for (int b = 0; b < NBS; ++b) {
                        int s = BS_SIDE[b], n = nbk[b];
                        occ[b][((x / s) * n + y / s) * n + z / s] = 1;
                    }
}

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int sphere_range(const mvt_t *t, float cx, float cy, float cz, float r, int *bmin, int *bmax)
{
    const float rp = r + VCC_POINT_RADIUS;
    if (cx + rp < t->glo[0] || cx - rp > t->ghi[0] || cy + rp < t->glo[1] ||
        cy - rp > t->ghi[1] || cz + rp < t->glo[2] || cz - rp > t->ghi[2])
        return 0;
    const float rv = rp * t->s;
    const float v[3] = {(cx - t->wmin) * t->s, (cy - t->wmin) * t->s, (cz - t->wmin) * t->s};
    for (int k = 0; k < 3; ++k) {
        bmin[k] = clampi((int)vfloorf(v[k] - rv), 0, t->G - 1);
        bmax[k] = clampi((int)vfloorf(v[k] + rv), 0, t->G - 1);
    }
    return 1;
}

static void hadd(uint64_t *h, int n, uint64_t v) { h[v < (uint64_t)n ? v : (uint64_t)n - 1]++; }

// One robot-sphere query: workload counters; returns hit; fills the non-empty voxel list if asked
static int query(float cx, float cy, float cz, float r, int32_t *vox, int *nvox, int count)
{
    const mvt_t *t = cur;
    if (nvox) *nvox = 0;
    if (count) fs.queries++;
    int bmin[3], bmax[3];
    if (!sphere_range(t, cx, cy, cz, r, bmin, bmax)) { if (count) fs.culled++; return 0; }
    const float rp = r + VCC_POINT_RADIUS, rsq = rp * rp, c[3] = {cx, cy, cz};
    uint64_t cells = 1, ne = 0;
    for (int k = 0; k < 3; ++k) cells *= (uint64_t)(bmax[k] - bmin[k] + 1);
    int hit = 0;
    for (int x = bmin[0]; x <= bmax[0]; ++x)
        for (int y = bmin[1]; y <= bmax[1]; ++y)
            for (int z = bmin[2]; z <= bmax[2]; ++z) {
                int32_t idx = voxel_at(t, x, y, z);
                if (idx < 0) continue;
                ne++;
                if (vox) vox[(*nvox)++] = idx;
                const voxel_t *v = &t->vox[idx];
                float d2 = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    float e = c[k] < v->lo[k] ? v->lo[k] - c[k] : (c[k] > v->hi[k] ? c[k] - v->hi[k] : 0.0f);
                    d2 = d2 + e * e;
                }
                if (d2 > rsq) continue;
                if (count) fs.aabb_pass++;
                if (hit) continue;   // points: until the first colliding point, like Algorithm 1
                for (uint32_t i = 0; i < v->count; ++i) {
                    float dx = t->px[v->off + i] - cx, dy = t->py[v->off + i] - cy, dz = t->pz[v->off + i] - cz;
                    if (count) fs.points++;
                    if (dx * dx + dy * dy + dz * dz <= rsq) { hit = 1; break; }
                }
            }
    if (!count) return hit;
    fs.cells += cells;
    fs.nonempty += ne;
    fs.hits += (uint64_t)hit;
    hadd(fs.h_cells, FS_HC, cells);
    hadd(fs.h_nonempty, FS_HC, ne);
    for (int b = 0; b < NBS; ++b) {
        int s = BS_SIDE[b], n = nbk[b];
        uint64_t nb = 0, no = 0;
        for (int x = bmin[0] / s; x <= bmax[0] / s; ++x)
            for (int y = bmin[1] / s; y <= bmax[1] / s; ++y)
                for (int z = bmin[2] / s; z <= bmax[2] / s; ++z) {
                    nb++;
                    no += occ[b][(x * n + y) * n + z];
                }
        fs.blocks[b] += nb;
        fs.blocks_occ[b] += no;
        hadd(fs.h_blocks[b], FS_HB, nb);
    }
    return hit;
}

// ---------------------------------------------------------------- (3): every pose the planner checks
static void rd_reset(uint32_t nvox);
void fs_problem(const mvt_t *t) { prepare(t); }
void fs_run_begin(void) { fs.runs++; if (fs_reuse_on) rd_reset(cur->n_vox); }

void envstat_pose(const float *q)
{
    stats_t keep = st;
    float sp[PANDA_N_SPHERES][4];
    panda_sphere_fk(q, sp);
    fs.poses++;
    int first = -1;
    for (int k = 0; k < PANDA_N_SPHERES; ++k) {
        const float *s = sp[FINE_ORDER[k]];
        if (query(s[0], s[1], s[2], s[3], 0, 0, 1) && first < 0) first = k;
    }
    if (first >= 0) { fs.colliding_poses++; fs.q_to_first_hit_pose += (uint64_t)first + 1; }
    st = keep;
}

// ---------------------------------------------------------------- (2): batches rebuilt per edge
#define MAXP 32
#define QMAX (MAXP * PANDA_N_SPHERES)
typedef struct { uint16_t sphere, pose, seq; uint8_t hit, nv; uint32_t key; int32_t v[27]; } q_t;
static q_t qall[QMAX];
static uint16_t ord[3][QMAX];

static uint32_t part1by2(uint32_t v)
{
    v &= 0x3ff;
    v = (v | (v << 16)) & 0x030000FF;
    v = (v | (v << 8)) & 0x0300F00F;
    v = (v | (v << 4)) & 0x030C30C3;
    v = (v | (v << 2)) & 0x09249249;
    return v;
}

static const q_t *sort_base;
static int cmp_sphere(const void *a, const void *b)
{
    const q_t *p = &sort_base[*(const uint16_t *)a], *q = &sort_base[*(const uint16_t *)b];
    if (p->sphere != q->sphere) return p->sphere < q->sphere ? -1 : 1;   // sphere = rank in VAMP order
    return p->pose < q->pose ? -1 : (p->pose > q->pose);
}
static int cmp_morton(const void *a, const void *b)
{
    const q_t *p = &sort_base[*(const uint16_t *)a], *q = &sort_base[*(const uint16_t *)b];
    if (p->key != q->key) return p->key < q->key ? -1 : 1;
    return p->seq < q->seq ? -1 : (p->seq > q->seq);
}

// LRU stack distance via a Fenwick tree over access times (one stream per order x {full, early-exit})
typedef struct { int32_t *last; uint32_t *bit; uint32_t T, cap; } rd_t;
static rd_t rd[3][2];
static uint32_t rd_nvox;

static void rd_reset(uint32_t nvox)
{
    for (int o = 0; o < 3; ++o)
        for (int e = 0; e < 2; ++e) {
            rd_t *r = &rd[o][e];
            r->last = realloc(r->last, nvox * sizeof(int32_t));
            for (uint32_t i = 0; i < nvox; ++i) r->last[i] = -1;
            r->T = 0;
            if (!r->bit) { r->cap = 1u << 16; r->bit = calloc(r->cap + 1, 4); }
            else memset(r->bit, 0, (r->cap + 1) * 4);
        }
    rd_nvox = nvox;
}
static void bit_add(rd_t *r, uint32_t i, int d) { for (++i; i <= r->cap; i += i & (0u - i)) r->bit[i] += (uint32_t)d; }
static uint32_t bit_sum(const rd_t *r, uint32_t i) { uint32_t s = 0; for (; i; i -= i & (0u - i)) s += r->bit[i]; return s; }
static void rd_grow(rd_t *r)
{
    // rebuild a twice-as-large tree from the live markers (a marker sits at each voxel's last time)
    uint32_t ncap = r->cap * 2;
    free(r->bit);
    r->bit = calloc(ncap + 1, 4);
    r->cap = ncap;
    for (uint32_t v = 0; v < rd_nvox; ++v)
        if (r->last[v] >= 0) bit_add(r, (uint32_t)r->last[v], 1);
}
static void rd_access(rd_t *r, int32_t v, uint64_t *hist)
{
    if (r->T >= r->cap) rd_grow(r);
    uint32_t t = r->T++;
    if (r->last[v] < 0) hist[FS_RD - 1]++;   // cold
    else {
        uint32_t l = (uint32_t)r->last[v];
        uint32_t d = bit_sum(r, t) - bit_sum(r, l + 1);   // distinct voxels touched strictly between l and t
        int b = 0;
        while (b < FS_RD - 2 && (1u << b) <= d) ++b;       // bin 0: d=0, bin b: 2^(b-1) <= d < 2^b
        hist[b]++;
        bit_add(r, l, -1);
    }
    bit_add(r, t, 1);
    r->last[v] = (int32_t)t;
}

static void eval_batch(int P, const q_t *base, int nq)
{
    int pi = P == 8 ? 0 : P == 16 ? 1 : 2;
    sort_base = base;
    for (int i = 0; i < nq; ++i) ord[0][i] = ord[1][i] = ord[2][i] = (uint16_t)i;
    qsort(ord[1], (size_t)nq, 2, cmp_sphere);
    qsort(ord[2], (size_t)nq, 2, cmp_morton);
    fs.ord_batches[pi]++;
    fs.ord_queries[pi] += (uint64_t)nq;
    for (int o = 0; o < 3; ++o) {
        int first = -1;
        for (int i = 0; i < nq && first < 0; ++i)
            if (base[ord[o][i]].hit) first = i;
        if (first >= 0) { fs.ord_hit_batches[pi][o]++; fs.ord_q_to_hit[pi][o] += (uint64_t)first + 1; }
        if (P == 32 && fs_reuse_on) {
            for (int i = 0; i < nq; ++i) {
                const q_t *q = &base[ord[o][i]];
                for (int k = 0; k < q->nv; ++k) {
                    rd_access(&rd[o][0], q->v[k], fs.rd_full[o]);
                    if (first < 0 || i <= first) rd_access(&rd[o][1], q->v[k], fs.rd_ee[o]);
                }
            }
        }
    }
}

void envstat_edge_begin(void) {}

void envstat_edge_end(const float *start, const float *vec, uint32_t n, uint32_t blocks)
{
    if (!fs_reuse_on) return;
    stats_t keep = st;
    // same pose arithmetic as validate_vector (rrtc.c); enough poses for the largest batch size
    float block[8][PANDA_DIM], back[PANDA_DIM];
    for (int i = 0; i < 8; ++i) {
        const float pct = (float)(i + 1) / 8.0f;
        for (int j = 0; j < PANDA_DIM; ++j) block[i][j] = start[j] + (vec[j] * pct);
    }
    for (int j = 0; j < PANDA_DIM; ++j) back[j] = vec[j] / (float)(8 * n);
    uint32_t total = n * 8, need = ((blocks * 8 + MAXP - 1) / MAXP) * MAXP;
    if (need > total) need = total;
    static float poses[1 << 16][PANDA_DIM];
    uint32_t np = 0;
    for (uint32_t k = 0; k * 8 < need && np + 8 <= (1u << 16); ++k) {
        if (k)
            for (int i = 0; i < 8; ++i)
                for (int j = 0; j < PANDA_DIM; ++j) block[i][j] = block[i][j] - back[j];
        for (int i = 0; i < 8; ++i) memcpy(poses[np++], block[i], sizeof(block[i]));
    }
    for (int pi = 0; pi < 3; ++pi) {
        const int P = 8 << pi;
        uint32_t issued = ((blocks * 8 + P - 1) / P) * P;
        if (issued > total) issued = total;
        for (uint32_t b0 = 0; b0 < issued; b0 += (uint32_t)P) {
            int nq = 0;
            for (uint32_t p = b0; p < b0 + (uint32_t)P && p < issued; ++p) {
                float sp[PANDA_N_SPHERES][4];
                panda_sphere_fk(poses[p], sp);
                for (int k = 0; k < PANDA_N_SPHERES; ++k) {
                    const float *s = sp[FINE_ORDER[k]];
                    q_t *q = &qall[nq];
                    int nv = 0;
                    q->hit = (uint8_t)query(s[0], s[1], s[2], s[3], q->v, &nv, 0);   // counters belong to (3)
                    q->nv = (uint8_t)nv;
                    q->sphere = (uint16_t)k;
                    q->pose = (uint16_t)(p - b0);
                    q->seq = (uint16_t)nq;
                    int v[3];
                    for (int a = 0; a < 3; ++a)
                        v[a] = clampi((int)vfloorf((s[a] - cur->wmin) * cur->s), 0, cur->G - 1);
                    q->key = (part1by2((uint32_t)v[0]) << 2) | (part1by2((uint32_t)v[1]) << 1) | part1by2((uint32_t)v[2]);
                    nq++;
                }
            }
            eval_batch(P, qall, nq);
        }
    }
    st = keep;
}

// unused here: the planner's own environment queries pass straight through
int envstat_query(int id, float x, float y, float z, float r)
{
    (void)id;
    return cc_env_sphere(x, y, z, r);
}
int envstat_all_query(int id, float x, float y, float z, float r)
{
    (void)id;
    return mvt_sphere_collides(cc_mvt, x, y, z, r);
}
