// Host-only experiments on the environment queries of the planner (built with -DENV_STATS).
//  A: per sphere query, what the CCU's voxel-lookup stage would have to do with the MVT
//     (one cell per cycle over the sphere's voxel range) vs a 4x4x4 Morton block bitmap
//     (8 parity banks, then one cycle per non-empty voxel), split into robot spheres and
//     link bounding spheres (r > r_max).
//  B: voxel-header / point reuse inside one CHECK batch (<= 32 poses) under three orders:
//     as issued (pose-major, VAMP hierarchy), sphere-major, Morton order of the sphere centre;
//     plus how far into the batch the first colliding query lands (early-exit cost).
// Query results are always the planner's own (cc_env_sphere); these hooks only observe.
#include <stdlib.h>
#include <string.h>
#include "envstat.h"
#include "cc.h"
#include "panda.h"
#include "vmath.h"

envstat_t es;
int es_reuse_on;

// ---------------------------------------------------------------- block occupancy (4x4x4)
#define BW 8                        // blocks per axis after padding G <= 32
static uint8_t blk_occ[BW * BW * BW];
static const mvt_t *blk_for;

static int32_t voxel_at(const mvt_t *t, int x, int y, int z)
{
    int32_t oy = t->pool[x];
    if (oy < 0) return -1;
    int32_t oz = t->pool[oy + y];
    if (oz < 0) return -1;
    return t->pool[oz + z];
}

static void build_blocks(const mvt_t *t)
{
    memset(blk_occ, 0, sizeof(blk_occ));
    for (int x = 0; x < t->G; ++x)
        for (int y = 0; y < t->G; ++y)
            for (int z = 0; z < t->G; ++z)
                if (voxel_at(t, x, y, z) >= 0) blk_occ[((x >> 2) * BW + (y >> 2)) * BW + (z >> 2)] = 1;
    blk_for = t;
}

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Voxel range of Algorithm 1 lines 5-9; returns 0 if the sphere is culled by the global AABB (line 2)
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

static void hist_add(uint32_t *h, int n, uint32_t v) { h[v < (uint32_t)n ? v : (uint32_t)n - 1]++; }

// ---------------------------------------------------------------- experiment A
static void profile_query(int cls, float cx, float cy, float cz, float r, int hit);
static void profile_check(int done, int hit) { if (done != hit) es.hit_mismatch++; }

static void profile_query(int cls, float cx, float cy, float cz, float r, int hit)
{
    const mvt_t *t = cc_mvt;
    es_class_t *c = &es.cls[cls];
    c->queries++;
    int bmin[3], bmax[3];
    if (!sphere_range(t, cx, cy, cz, r, bmin, bmax)) { c->gculled++; return; }
    c->hits += (uint32_t)hit;
    const float rp = r + VCC_POINT_RADIUS, rsq = rp * rp;
    uint32_t cells = 1;
    for (int k = 0; k < 3; ++k) cells *= (uint32_t)(bmax[k] - bmin[k] + 1);
    // MVT walk in Algorithm 1 order; "exit" counters stop at the first colliding point
    uint32_t xl = 0, yl = 0, zl = 0, nonempty = 0, cells_exit = 0, ne_exit = 0, pts_exit = 0, pts_full = 0;
    int done = 0;
    for (int x = bmin[0]; x <= bmax[0]; ++x) {
        if (!done) xl++;
        int32_t oy = t->pool[x];
        for (int y = bmin[1]; y <= bmax[1]; ++y) {
            if (oy >= 0 && !done) yl++;
            int32_t oz = oy >= 0 ? t->pool[oy + y] : -1;
            for (int z = bmin[2]; z <= bmax[2]; ++z) {
                if (!done) { cells_exit++; if (oz >= 0) zl++; }
                int32_t idx = oz >= 0 ? t->pool[oz + z] : -1;
                if (idx < 0) continue;
                nonempty++;
                if (!done) ne_exit++;
                const voxel_t *vx = &t->vox[idx];
                pts_full += vx->count;
                if (done) continue;
                pts_exit += vx->count;
                float d2 = 0.0f, cc[3] = {cx, cy, cz};
                for (int k = 0; k < 3; ++k) {
                    float e = cc[k] < vx->lo[k] ? vx->lo[k] - cc[k] : (cc[k] > vx->hi[k] ? cc[k] - vx->hi[k] : 0.0f);
                    d2 = d2 + e * e;
                }
                if (d2 > rsq) continue;   // local AABB (line 19)
                for (uint32_t i = 0; i < vx->count; ++i) {
                    float dx = t->px[vx->off + i] - cx, dy = t->py[vx->off + i] - cy, dz = t->pz[vx->off + i] - cz;
                    c->points_tested++;
                    if (dx * dx + dy * dy + dz * dz <= rsq) { done = 1; break; }
                }
            }
        }
    }
    // Morton 4x4x4 blocks: one read per block, 8 banks by block-coordinate parity
    uint32_t nb = 0, nb_occ = 0, bank[8] = {0};
    for (int bx = bmin[0] >> 2; bx <= bmax[0] >> 2; ++bx)
        for (int by = bmin[1] >> 2; by <= bmax[1] >> 2; ++by)
            for (int bz = bmin[2] >> 2; bz <= bmax[2] >> 2; ++bz) {
                nb++;
                bank[((bx & 1) << 2) | ((by & 1) << 1) | (bz & 1)]++;
                nb_occ += blk_occ[(bx * BW + by) * BW + bz];
            }
    uint32_t bank_cycles = 0;
    for (int b = 0; b < 8; ++b) if (bank[b] > bank_cycles) bank_cycles = bank[b];
    profile_check(done, hit);
    c->cells += cells; c->cells_exit += cells_exit;
    c->xlook += xl; c->ylook += yl; c->zlook += zl;
    c->nonempty += nonempty; c->nonempty_exit += ne_exit;
    c->points += pts_full; c->points_exit += pts_exit;
    c->blocks += nb; c->blocks_occ += nb_occ; c->bank_cycles += bank_cycles;
    hist_add(c->h_cells, ES_HC, cells);
    hist_add(c->h_nonempty, ES_HN, nonempty);
    hist_add(c->h_blocks, ES_HB, nb);
    hist_add(c->h_bankcyc, ES_HB, bank_cycles);
}

// ---------------------------------------------------------------- experiment B
// A CCU batch is <= 32 poses of one edge in VAMP's rake-8 layout. It is rebuilt here in full: every pose
// of each batch the planner issued, every hierarchical query of each pose (panda_env_all, no early exit),
// so the issue order is not favoured by the planner having stopped at the first collision.
#define QMAX 8192
typedef struct { float x, y, z, r; uint16_t id, pose, seq, rank; uint8_t hit; uint32_t key; } q_t;
static q_t qb[QMAX], qs[QMAX];
static int nq, cur_pose, in_edge;

static uint32_t part1by2(uint32_t v)
{
    v &= 0x3ff;
    v = (v | (v << 16)) & 0x030000FF;
    v = (v | (v << 8)) & 0x0300F00F;
    v = (v | (v << 4)) & 0x030C30C3;
    v = (v | (v << 2)) & 0x09249249;
    return v;
}

// LRU of voxel headers; returns 1 on hit
typedef struct { int32_t tag[ES_LRU_MAX]; int n, cap; } lru_t;
static int lru_access(lru_t *l, int32_t tag)
{
    for (int i = 0; i < l->n; ++i)
        if (l->tag[i] == tag) {
            for (int j = i; j > 0; --j) l->tag[j] = l->tag[j - 1];
            l->tag[0] = tag;
            return 1;
        }
    if (l->n < l->cap) l->n++;
    for (int j = l->n - 1; j > 0; --j) l->tag[j] = l->tag[j - 1];
    l->tag[0] = tag;
    return 0;
}

static int cmp_sphere(const void *a, const void *b)
{
    const q_t *p = a, *q = b;
    if (p->rank != q->rank) return p->rank < q->rank ? -1 : 1;   // keeps VAMP's hierarchy order of spheres
    if (p->pose != q->pose) return p->pose < q->pose ? -1 : 1;
    return p->seq < q->seq ? -1 : (p->seq > q->seq);
}
static int cmp_morton(const void *a, const void *b)
{
    const q_t *p = a, *q = b;
    if (p->key != q->key) return p->key < q->key ? -1 : 1;
    return p->seq < q->seq ? -1 : (p->seq > q->seq);
}

#define SEEN_MAX (1 << 16)
static void eval_order(int ord, const q_t *list)
{
    const mvt_t *t = cc_mvt;
    es_order_t *o = &es.ord[ord];
    lru_t lru[ES_NLRU];
    for (int k = 0; k < ES_NLRU; ++k) { lru[k].n = 0; lru[k].cap = es_lru_sizes[k]; }
    static uint8_t seen[SEEN_MAX];
    if (t->n_vox > SEEN_MAX) es.seen_overflow++;
    memset(seen, 0, t->n_vox < SEEN_MAX ? t->n_vox : SEEN_MAX);
    int first_hit = -1;
    for (int i = 0; i < nq; ++i) {
        const q_t *q = &list[i];
        int live = first_hit < 0;   // STOP_ON_FIRST_HIT: work up to and including the first colliding query
        if (q->hit && first_hit < 0) first_hit = i;
        if (live) o->q_ee++;
        int bmin[3], bmax[3];
        if (!sphere_range(t, q->x, q->y, q->z, q->r, bmin, bmax)) continue;
        for (int x = bmin[0]; x <= bmax[0]; ++x)
            for (int y = bmin[1]; y <= bmax[1]; ++y)
                for (int z = bmin[2]; z <= bmax[2]; ++z) {
                    int32_t idx = voxel_at(t, x, y, z);
                    if (idx < 0) continue;
                    o->voxel_refs++;
                    o->point_refs += t->vox[idx].count;
                    if (idx < SEEN_MAX && !seen[idx]) { seen[idx] = 1; o->distinct++; o->distinct_points += t->vox[idx].count; }
                    if (live) o->refs_ee++;
                    for (int k = 0; k < ES_NLRU; ++k)
                        if (!lru_access(&lru[k], idx)) {
                            o->misses[k]++; o->miss_points[k] += t->vox[idx].count;
                            if (live) { o->misses_ee[k]++; o->miss_points_ee[k] += t->vox[idx].count; }
                        }
                }
    }
    if (first_hit >= 0) { o->hit_batches++; o->queries_to_first_hit += (uint64_t)first_hit + 1; }
}

static void flush_batch(void)
{
    if (nq == 0) return;
    es.batches++;
    es.batch_queries += (uint64_t)nq;
    uint16_t first[128];
    for (int k = 0; k < 128; ++k) first[k] = 0xffff;
    for (int i = 0; i < nq; ++i) {
        qb[i].seq = (uint16_t)i;
        if (first[qb[i].id] == 0xffff) first[qb[i].id] = (uint16_t)i;
    }
    for (int i = 0; i < nq; ++i) qb[i].rank = first[qb[i].id];
    eval_order(0, qb);
    memcpy(qs, qb, (size_t)nq * sizeof(q_t));
    qsort(qs, (size_t)nq, sizeof(q_t), cmp_sphere);
    eval_order(1, qs);
    memcpy(qs, qb, (size_t)nq * sizeof(q_t));
    qsort(qs, (size_t)nq, sizeof(q_t), cmp_morton);
    eval_order(2, qs);
    nq = 0;
}

// Called from panda_env_all: result from the MVT, planner counters left untouched
int envstat_all_query(int id, float x, float y, float z, float r)
{
    stats_t keep = st;
    int hit = mvt_sphere_collides(cc_mvt, x, y, z, r);
    st = keep;
    if (nq >= QMAX) { es.dropped++; return hit; }
    q_t *q = &qb[nq++];
    q->x = x; q->y = y; q->z = z; q->r = r; q->id = (uint16_t)id; q->pose = (uint16_t)cur_pose;
    q->hit = (uint8_t)(hit && id < PANDA_N_SPHERES);   // a pose collides only on a robot-sphere hit
    const mvt_t *t = cc_mvt;
    int v[3];
    float c[3] = {x, y, z};
    for (int k = 0; k < 3; ++k) v[k] = clampi((int)vfloorf((c[k] - t->wmin) * t->s), 0, t->G - 1);
    q->key = (part1by2((uint32_t)v[0]) << 2) | (part1by2((uint32_t)v[1]) << 1) | part1by2((uint32_t)v[2]);
    return hit;
}

// ---------------------------------------------------------------- hooks
void envstat_edge_begin(void) { in_edge = 1; }
void envstat_pose(const float *q) { (void)q; }

void envstat_edge_end(const float *start, const float *vec, uint32_t n, uint32_t blocks)
{
    in_edge = 0;
    if (!es_reuse_on) return;
    if (blk_for != cc_mvt) build_blocks(cc_mvt);
    // same pose arithmetic as validate_vector (rrtc.c)
    float block[8][PANDA_DIM], back[PANDA_DIM];
    for (int i = 0; i < 8; ++i) {
        const float pct = (float)(i + 1) / 8.0f;
        for (int j = 0; j < PANDA_DIM; ++j) block[i][j] = start[j] + (vec[j] * pct);
    }
    for (int j = 0; j < PANDA_DIM; ++j) back[j] = vec[j] / (float)(8 * n);
    // only the batches the planner issued: up to and including the one with its first collision
    uint32_t batches = (blocks * 8 + 31) / 32, poses = batches * 32 < n * 8 ? batches * 32 : n * 8;
    nq = 0;
    for (uint32_t k = 0; k * 8 < poses; ++k) {
        if (k)
            for (int i = 0; i < 8; ++i)
                for (int j = 0; j < PANDA_DIM; ++j) block[i][j] = block[i][j] - back[j];
        for (int i = 0; i < 8; ++i) {
            cur_pose = (int)(k * 8 + (uint32_t)i) % 32;
            panda_env_all(block[i]);
        }
        if ((k * 8 + 8) % 32 == 0) flush_batch();
    }
    flush_batch();
}

int envstat_query(int id, float x, float y, float z, float r)
{
    if (blk_for != cc_mvt) build_blocks(cc_mvt);
    int hit = cc_env_sphere(x, y, z, r);
    profile_query(id >= PANDA_N_SPHERES, x, y, z, r, hit);
    if (!in_edge) es.outside_edge++;
    return hit;
}
