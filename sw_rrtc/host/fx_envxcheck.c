// Cross-check of fx_env_build / fx_cse_ref against the preprocessing golden vectors (core.cpp of
// vcc-based-accelerator, hw/vectors/<scene>): status, points, directory, cells, global AABB, queries.
//   fx_envxcheck <vectors_dir> [scene ...]      (default: s0_pkgvec .. s6_dense)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fx_env.h"

#define Q_PAD 43u                           // core.cpp query(): radius + point radius 41 + cover 2

static int errs_total;

static FILE *open_in(const char *dir, const char *scene, const char *name, const char *mode)
{
    char path[1024];
    snprintf(path, sizeof path, "%s/%s/%s", dir, scene, name);
    FILE *f = fopen(path, mode);
    if (!f) { fprintf(stderr, "fatal: cannot open %s\n", path); exit(2); }
    return f;
}

// one hex word per line; returns the number of lines read (up to max)
static size_t read_hex(const char *dir, const char *scene, const char *name, char (*line)[40], size_t max)
{
    FILE *f = open_in(dir, scene, name, "r");
    size_t n = 0;
    while (n < max && fscanf(f, "%39s", line[n]) == 1) n++;
    fclose(f);
    return n;
}

static uint32_t hex_field(const char *s, int first, int digits)    // digits hex chars starting at s[first]
{
    char buf[9];
    memcpy(buf, s + first, (size_t)digits);
    buf[digits] = 0;
    return (uint32_t)strtoul(buf, NULL, 16);
}

static int report(const char *scene, const char *what, long idx, long got, long exp)
{
    static int shown;
    if (shown++ < 20) printf("ENVX MISMATCH scene=%s %s[%ld] got=%ld expected=%ld\n", scene, what, idx, got, exp);
    return 1;
}

static void check_scene(const char *dir, const char *scene)
{
    static char line[70000][40];
    static fx_env_t e;
    int errs = 0;
    size_t nm = read_hex(dir, scene, "meta.hex", line, 16);
    if (nm < 13) { fprintf(stderr, "fatal: %s/meta.hex too short\n", scene); exit(2); }
    uint32_t meta[16];
    for (size_t i = 0; i < nm; ++i) meta[i] = hex_field(line[i], 0, 8);
    const uint32_t n_in = meta[0], x_status = meta[1], x_kept = meta[2], x_cells = meta[3], cap = meta[12];

    FILE *f = open_in(dir, scene, "input_points_f32.bin", "rb");
    float *raw = malloc(sizeof(float) * 3 * (n_in ? n_in : 1));
    float *x = malloc(sizeof(float) * (n_in ? n_in : 1)), *y = malloc(sizeof(float) * (n_in ? n_in : 1));
    float *z = malloc(sizeof(float) * (n_in ? n_in : 1));
    if (fread(raw, sizeof(float) * 3, n_in, f) != n_in) { fprintf(stderr, "fatal: %s input too short\n", scene); exit(2); }
    fclose(f);
    for (uint32_t i = 0; i < n_in; ++i) { x[i] = raw[3 * i]; y[i] = raw[3 * i + 1]; z[i] = raw[3 * i + 2]; }
    fx_env_build(x, y, z, n_in, cap, &e);
    free(raw); free(x); free(y); free(z);

    if ((uint32_t)e.status != x_status) errs += report(scene, "status", 0, e.status, x_status);
    if (e.valid != (x_status == 0)) errs += report(scene, "valid", 0, e.valid, x_status == 0);
    uint32_t nq = 0;
    if (x_status == 0 && e.valid) {
        if (e.n_pts != x_kept) errs += report(scene, "n_pts", 0, e.n_pts, x_kept);
        if (e.n_cells != x_cells) errs += report(scene, "n_cells", 0, e.n_cells, x_cells);
        // points: {z, y, x}
        size_t np = read_hex(dir, scene, "exp_points.hex", line, 70000);
        if (np != x_kept) errs += report(scene, "exp_points lines", 0, (long)np, x_kept);
        for (size_t i = 0; i < np && i < e.n_pts; ++i) {
            uint32_t pz = hex_field(line[i], 0, 4), py = hex_field(line[i], 4, 4), px = hex_field(line[i], 8, 4);
            if (e.px[i] != px || e.py[i] != py || e.pz[i] != pz)
                errs += report(scene, "point", (long)i, ((long)e.pz[i] << 32) | ((long)e.py[i] << 16) | e.px[i],
                               ((long)pz << 32) | ((long)py << 16) | px);
        }
        // directory: theirs is indexed by the coarse Morton key, ours by (z*G + y)*G + x
        static uint16_t cell_g[ENV_MAX_SLOTS][3];
        size_t nd = read_hex(dir, scene, "exp_cellid.hex", line, 32768);
        if (nd != 32768) errs += report(scene, "exp_cellid lines", 0, (long)nd, 32768);
        for (uint32_t k = 0; k < nd; ++k) {
            uint32_t g[3] = {0, 0, 0};
            for (int b = 0; b < 5; ++b)
                for (int a = 0; a < 3; ++a) g[a] |= ((k >> (3 * b + a)) & 1u) << b;
            uint32_t id = hex_field(line[k], 0, 4), d = e.dir[(g[2] * ENV_G + g[1]) * ENV_G + g[0]];
            uint32_t exp_d = id == 0xffffu ? 0u : (0x8000u | id);
            if (d != exp_d) errs += report(scene, "dir", k, d, exp_d);
            if (id != 0xffffu && id < ENV_MAX_SLOTS)
                for (int a = 0; a < 3; ++a) cell_g[id][a] = (uint16_t)g[a];
        }
        // cells: start, count, hi z y x, lo z y x (absolute); ours stores offsets from the cell origin
        size_t nc = read_hex(dir, scene, "exp_cells.hex", line, ENV_MAX_SLOTS);
        if (nc != x_cells) errs += report(scene, "exp_cells lines", 0, (long)nc, x_cells);
        for (size_t c = 0; c < nc && c < e.n_cells; ++c) {
            const env_cell_t *cl = &e.cell[c];
            if (cl->base != hex_field(line[c], 0, 4)) errs += report(scene, "cell.base", (long)c, cl->base, hex_field(line[c], 0, 4));
            if (cl->count != hex_field(line[c], 4, 4)) errs += report(scene, "cell.count", (long)c, cl->count, hex_field(line[c], 4, 4));
            for (int a = 0; a < 3; ++a) {
                uint32_t org = (uint32_t)cell_g[c][a] << ENV_QSHIFT;
                uint32_t xhi = hex_field(line[c], 8 + 4 * (2 - a), 4), xlo = hex_field(line[c], 20 + 4 * (2 - a), 4);
                if (org + cl->lo[a] != xlo) errs += report(scene, "cell.lo", (long)c * 3 + a, org + cl->lo[a], xlo);
                if (org + cl->hi[a] != xhi) errs += report(scene, "cell.hi", (long)c * 3 + a, org + cl->hi[a], xhi);
            }
        }
        if (e.n_pts)
            for (int a = 0; a < 3; ++a) {
                if (e.glo[a] != meta[6 + a]) errs += report(scene, "glo", a, e.glo[a], meta[6 + a]);
                if (e.ghi[a] != meta[9 + a]) errs += report(scene, "ghi", a, e.ghi[a], meta[9 + a]);
            }
        // queries: {r, z, y, x}; hit iff |c - p|^2 <= (r + 43)^2
        static char ql[4096][40], xl[4096][40];
        nq = (uint32_t)read_hex(dir, scene, "queries.hex", ql, 4096);
        size_t nx = read_hex(dir, scene, "exp_query.hex", xl, 4096);
        if (nx != nq) errs += report(scene, "exp_query lines", 0, (long)nx, nq);
        for (uint32_t i = 0; i < nq && i < nx; ++i) {
            uint32_t r = hex_field(ql[i], 0, 4) + Q_PAD;
            int32_t cz = (int32_t)hex_field(ql[i], 4, 4), cy = (int32_t)hex_field(ql[i], 8, 4), cx = (int32_t)hex_field(ql[i], 12, 4);
            int exp = (int)hex_field(xl[i], 0, 1);
            int ref = fx_cse_ref(&e, cx, cy, cz, r), def = fx_cse_def(&e, cx, cy, cz, r);
            if (ref != exp) errs += report(scene, "query(ref)", i, ref, exp);
            if (def != exp) errs += report(scene, "query(def)", i, def, exp);
        }
    }
    printf("ENVX scene=%s in=%u status=%d points=%u cells=%u queries=%u errors=%d\n", scene, n_in, e.status, e.n_pts,
           e.n_cells, nq, errs);
    errs_total += errs;
}

// Boundary cases the vectors do not reach; expected values follow core.cpp Model::build():
// cull when norm >= 19497^2, CAPACITY when a new voxel arrives with points*4 >= cap*3.
static void check_directed(void)
{
    static fx_env_t e;
    int errs = 0;
    float x[16], y[16], z[16];
    for (int i = 0; i < 16; ++i) { x[i] = (float)i * 0.0625f - 0.5f; y[i] = 0.0f; z[i] = 0.0f; }   // 16 fine voxels
    const float on = 19497.0f / 16384.0f, in = 19496.0f / 16384.0f, zero = 0.0f;
    fx_env_build(&on, &zero, &zero, 1, ENV_HASH_CAP, &e);
    if (e.status != ENV_ST_OK || e.n_pts != 0) errs += report("directed", "cull at radius", 0, e.n_pts, 0);
    fx_env_build(&in, &zero, &zero, 1, ENV_HASH_CAP, &e);
    if (e.status != ENV_ST_OK || e.n_pts != 1) errs += report("directed", "keep inside radius", 0, e.n_pts, 1);
    fx_env_build(x, y, z, 12, 16, &e);
    if (e.status != ENV_ST_OK || e.n_pts != 12) errs += report("directed", "12 points at cap 16", 0, e.status, ENV_ST_OK);
    fx_env_build(x, y, z, 13, 16, &e);
    if (e.status != ENV_ST_CAP || e.valid) errs += report("directed", "13 points at cap 16", 0, e.status, ENV_ST_CAP);
    // two errors in one stream: the first one in input order wins (core.cpp returns at the first error)
    const float inf = 1.0f / 0.0f;
    y[14] = inf;                                               // capacity at point 12, non-finite at point 14
    fx_env_build(x, y, z, 16, 16, &e);
    if (e.status != ENV_ST_CAP) errs += report("directed", "capacity before inf", 0, e.status, ENV_ST_CAP);
    y[14] = 0.0f; y[3] = inf;                                  // non-finite at point 3, capacity later
    fx_env_build(x, y, z, 16, 16, &e);
    if (e.status != ENV_ST_INVALID) errs += report("directed", "inf before capacity", 0, e.status, ENV_ST_INVALID);
    printf("ENVX scene=directed cases=6 errors=%d\n", errs);
    errs_total += errs;
}

int main(int argc, char **argv)
{
    check_directed();
    static const char *def[] = {"s0_pkgvec", "s1_shelf", "s2_edge", "s3_cap1024", "s4_inf", "s5_empty", "s6_dense"};
    if (argc < 2) { fprintf(stderr, "usage: fx_envxcheck <vectors_dir> [scene ...]\n"); return 2; }
    int n = 0;
    if (argc > 2) for (int i = 2; i < argc; ++i, ++n) check_scene(argv[1], argv[i]);
    else for (size_t i = 0; i < sizeof def / sizeof def[0]; ++i, ++n) check_scene(argv[1], def[i]);
    printf("TB_ENVX scenes=%d errors=%d\n", n, errs_total);
    return errs_total ? 1 : 0;
}
