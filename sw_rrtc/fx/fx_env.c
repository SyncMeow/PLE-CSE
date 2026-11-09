// Build-engine reference (quantize, filter, table) for format v0, and the CSE reference.
#include "fx_env.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

fx_cse_stats_t fx_cs;

#define FINE_G 128                          // fine cells per axis covered by 16-bit q

static uint32_t spread3(uint32_t v)         // 10-bit -> every third bit
{
    v &= 0x3ff;
    v = (v | (v << 16)) & 0x030000ff;
    v = (v | (v << 8)) & 0x0300f00f;
    v = (v | (v << 4)) & 0x030c30c3;
    v = (v | (v << 2)) & 0x09249249;
    return v;
}
static uint32_t morton3(uint32_t x, uint32_t y, uint32_t z) { return spread3(x) | (spread3(y) << 1) | (spread3(z) << 2); }

typedef struct { uint32_t key; uint16_t q[3]; } kp_t;

static int cmp_kp(const void *a, const void *b)
{
    uint32_t x = ((const kp_t *)a)->key, y = ((const kp_t *)b)->key;
    return x < y ? -1 : x > y;
}

void fx_env_build(const float *x, const float *y, const float *z, uint32_t n, int32_t w_min, fx_env_t *e)
{
    memset(e, 0, sizeof(*e));
    const int32_t qmax = -2 * w_min;        // workspace [w_min, -w_min)
    int32_t *best = malloc(sizeof(int32_t) * FINE_G * FINE_G * FINE_G);
    uint16_t *bd = malloc(sizeof(uint16_t) * FINE_G * FINE_G * FINE_G);
    uint16_t (*kept)[3] = malloc(sizeof(uint16_t[3]) * (n ? n : 1));
    uint32_t nk = 0;
    for (uint32_t i = 0; i < FINE_G * FINE_G * FINE_G; ++i) best[i] = -1;
    // filter: one point per fine cell, smallest centre distance, ties keep the first
    for (uint32_t i = 0; i < n; ++i) {
        const float p[3] = {x[i], y[i], z[i]};
        int32_t q[3];
        int out = 0;
        for (int k = 0; k < 3; ++k) {
            q[k] = (int32_t)floor((double)p[k] * 16384.0) - w_min;     // exact: power-of-two scale
            if (q[k] < 0 || q[k] >= qmax) out = 1;
        }
        if (out) continue;
        uint32_t f = ((uint32_t)(q[2] >> ENV_FSHIFT) * FINE_G + (uint32_t)(q[1] >> ENV_FSHIFT)) * FINE_G + (uint32_t)(q[0] >> ENV_FSHIFT);
        uint32_t d = 0;
        for (int k = 0; k < 3; ++k) {
            int32_t o = (q[k] >> (ENV_FSHIFT - ENV_S)) & ((1 << ENV_S) - 1), ev = 2 * o + 1 - (1 << ENV_S);
            d += (uint32_t)(ev * ev);
        }
        if (best[f] < 0) {
            best[f] = (int32_t)nk;
            bd[f] = (uint16_t)d;
            for (int k = 0; k < 3; ++k) kept[nk][k] = (uint16_t)q[k];
            nk++;
        } else if (d < bd[f]) {
            bd[f] = (uint16_t)d;
            for (int k = 0; k < 3; ++k) kept[best[f]][k] = (uint16_t)q[k];
        }
    }
    free(best);
    free(bd);
    if (nk > ENV_MAX_PTS) { e->err = 1; free(kept); return; }
    // order: query-cell Morton, then fine-cell Morton (one point per fine cell, so keys are unique)
    kp_t *kp = malloc(sizeof(kp_t) * (nk ? nk : 1));
    for (uint32_t i = 0; i < nk; ++i) {
        const uint16_t *q = kept[i];
        uint32_t qc = morton3(q[0] >> ENV_QSHIFT, q[1] >> ENV_QSHIFT, q[2] >> ENV_QSHIFT);
        uint32_t fc = morton3((q[0] >> ENV_FSHIFT) & 3, (q[1] >> ENV_FSHIFT) & 3, (q[2] >> ENV_FSHIFT) & 3);
        kp[i].key = (qc << 6) | fc;
        memcpy(kp[i].q, q, sizeof(kp[i].q));
    }
    free(kept);
    qsort(kp, nk, sizeof(kp_t), cmp_kp);
    for (int k = 0; k < 3; ++k) { e->glo[k] = 0xffff; e->ghi[k] = 0; }
    uint32_t prev = 0xffffffffu;
    for (uint32_t i = 0; i < nk; ++i) {
        const uint16_t *q = kp[i].q;
        e->px[i] = q[0]; e->py[i] = q[1]; e->pz[i] = q[2];
        if ((kp[i].key >> 6) != prev) {
            if (e->n_cells >= ENV_MAX_SLOTS) { e->err = 1; free(kp); return; }
            prev = kp[i].key >> 6;
            env_cell_t *c = &e->cell[e->n_cells];
            c->base = (uint16_t)i;
            c->count = 0;
            for (int k = 0; k < 3; ++k) { c->lo[k] = 0x7ff; c->hi[k] = 0; }
            uint32_t d = ((uint32_t)(q[2] >> ENV_QSHIFT) * ENV_G + (uint32_t)(q[1] >> ENV_QSHIFT)) * ENV_G + (uint32_t)(q[0] >> ENV_QSHIFT);
            e->dir[d] = (uint16_t)(0x8000u | e->n_cells);
            e->n_cells++;
        }
        env_cell_t *c = &e->cell[e->n_cells - 1];
        c->count++;
        for (int k = 0; k < 3; ++k) {
            uint16_t off = q[k] & ((1 << ENV_QSHIFT) - 1);
            if (off < c->lo[k]) c->lo[k] = off;
            if (off > c->hi[k]) c->hi[k] = off;
            if (q[k] < e->glo[k]) e->glo[k] = q[k];
            if (q[k] > e->ghi[k]) e->ghi[k] = q[k];
        }
    }
    free(kp);
    e->n_pts = nk;
    e->valid = 1;
}

static inline int64_t sq(int64_t v) { return v * v; }

int fx_cse_def(const fx_env_t *e, int32_t cx, int32_t cy, int32_t cz, uint32_t rq)
{
    const int64_t r2 = sq(rq);
    for (uint32_t i = 0; i < e->n_pts; ++i)
        if (sq((int64_t)cx - e->px[i]) + sq((int64_t)cy - e->py[i]) + sq((int64_t)cz - e->pz[i]) <= r2) return 1;
    return 0;
}

static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

int fx_cse_ref(const fx_env_t *e, int32_t cx, int32_t cy, int32_t cz, uint32_t rq)
{
    const int32_t c[3] = {cx, cy, cz}, r = (int32_t)rq;
    const int64_t r2 = sq(rq);
    fx_cs.queries++;
    if (!e->n_pts) return 0;
    // G: global cull, sphere box vs global AABB
    for (int k = 0; k < 3; ++k)
        if (c[k] + r < e->glo[k] || c[k] - r > e->ghi[k]) { fx_cs.gculled++; return 0; }
    // R: cell range per axis, clamped to the grid (arithmetic shift = floor)
    int32_t lo[3], hi[3];
    for (int k = 0; k < 3; ++k) {
        lo[k] = clampi((c[k] - r) >> ENV_QSHIFT, 0, ENV_G - 1);
        hi[k] = clampi((c[k] + r) >> ENV_QSHIFT, 0, ENV_G - 1);
        if ((c[k] + r) < 0 || ((c[k] - r) >> ENV_QSHIFT) > ENV_G - 1) return 0;
    }
    // E + O: enumerate cells, skip empty ones
    for (int32_t gz = lo[2]; gz <= hi[2]; ++gz)
        for (int32_t gy = lo[1]; gy <= hi[1]; ++gy)
            for (int32_t gx = lo[0]; gx <= hi[0]; ++gx) {
                fx_cs.cells++;
                uint16_t d = e->dir[(gz * ENV_G + gy) * ENV_G + gx];
                if (!(d & 0x8000u)) continue;
                fx_cs.nonempty++;
                const env_cell_t *cl = &e->cell[d & 0x0fffu];
                // A: local AABB, exact clamp distance
                const int32_t org[3] = {gx << ENV_QSHIFT, gy << ENV_QSHIFT, gz << ENV_QSHIFT};
                int64_t ad = 0;
                for (int k = 0; k < 3; ++k) {
                    int32_t b0 = org[k] + cl->lo[k], b1 = org[k] + cl->hi[k];
                    int32_t t = clampi(c[k], b0, b1);
                    ad += sq((int64_t)c[k] - t);
                }
                if (ad > r2) continue;
                fx_cs.aabb_pass++;
                // D: point distances, stop at the first hit
                for (uint32_t i = cl->base; i < (uint32_t)cl->base + cl->count; ++i) {
                    fx_cs.points++;
                    if (sq((int64_t)cx - e->px[i]) + sq((int64_t)cy - e->py[i]) + sq((int64_t)cz - e->pz[i]) <= r2) {
                        fx_cs.hits++;
                        return 1;
                    }
                }
            }
    return 0;
}

#ifndef __riscv   // trace dump: host only (the SoC baseline build has no __int128 or files)
#include <stdio.h>

static FILE *open2(const char *dir, const char *tag, const char *name)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s_%s", dir, tag, name);
    return fopen(path, "w");
}

int fx_env_dump(const fx_env_t *e, const char *dir, const char *tag)
{
    FILE *f = open2(dir, tag, "hdr.txt");
    if (!f) return -1;
    fprintf(f, "valid %d\nerr %d\nn_pts %u\nn_cells %u\nG %d\nqshift %d\nglo %u %u %u\nghi %u %u %u\n", e->valid, e->err,
            e->n_pts, e->n_cells, ENV_G, ENV_QSHIFT, e->glo[0], e->glo[1], e->glo[2], e->ghi[0], e->ghi[1], e->ghi[2]);
    fclose(f);
    // HDR as words for $readmemh: 0 {err, valid}, 1 n_pts, 2 n_cells, 3-5 glo x/y/z, 6-8 ghi x/y/z
    if (!(f = open2(dir, tag, "hdr.hex"))) return -1;
    fprintf(f, "%08x\n%08x\n%08x\n", (unsigned)((e->err ? 2 : 0) | (e->valid ? 1 : 0)), e->n_pts, e->n_cells);
    for (int k = 0; k < 3; ++k) fprintf(f, "%08x\n", e->glo[k]);
    for (int k = 0; k < 3; ++k) fprintf(f, "%08x\n", e->ghi[k]);
    fclose(f);
    if (!(f = open2(dir, tag, "dir.hex"))) return -1;
    for (int i = 0; i < ENV_NCELL; ++i) fprintf(f, "%04x\n", e->dir[i]);
    fclose(f);
    if (!(f = open2(dir, tag, "cell.hex"))) return -1;
    for (uint32_t i = 0; i < e->n_cells; ++i) {
        const env_cell_t *c = &e->cell[i];
        unsigned __int128 w = (unsigned __int128)c->base | ((unsigned __int128)(c->count & 0x3ffu) << 16);
        for (int k = 0; k < 3; ++k) {
            w |= (unsigned __int128)(c->lo[k] & 0x7ffu) << (26 + 11 * k);
            w |= (unsigned __int128)(c->hi[k] & 0x7ffu) << (59 + 11 * k);
        }
        uint64_t lo = (uint64_t)w, hi = (uint64_t)(w >> 64);
        fprintf(f, "%016llx%016llx\n", (unsigned long long)hi, (unsigned long long)lo);
    }
    fclose(f);
    const uint16_t *p[3] = {e->px, e->py, e->pz};
    const char *nm[3] = {"ptx.hex", "pty.hex", "ptz.hex"};
    for (int k = 0; k < 3; ++k) {
        if (!(f = open2(dir, tag, nm[k]))) return -1;
        for (uint32_t i = 0; i < e->n_pts; ++i) fprintf(f, "%04x\n", p[k][i]);
        fclose(f);
    }
    return 0;
}
#endif
