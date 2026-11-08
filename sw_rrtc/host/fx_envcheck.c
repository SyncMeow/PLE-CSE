// Checks fx_cse_ref (hardware-structured) against fx_cse_def (definition) on format-v0 environments.
//   fx_envcheck [first:last] [queries_per_problem]
#include <stdio.h>
#include <stdlib.h>
#include "env.h"
#include "problems.h"
#include "fx_env.h"

void fatal(const char *msg) { fprintf(stderr, "fatal: %s\n", msg); exit(2); }

static uint64_t rs = 0x2545F4914F6CDD1Dull;
static uint32_t xr(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return (uint32_t)(rs >> 32); }
static int32_t rnd(int32_t lo, int32_t hi) { return lo + (int32_t)(xr() % (uint32_t)(hi - lo + 1)); }

static const int quad[][4] = {{2, 3, 6, 7}, {1, 4, 8, 9}, {4, 4, 7, 9}, {2, 6, 9, 11}, {6, 6, 7, 11}, {1, 2, 2, 3}};

int main(int argc, char **argv)
{
    int first = 0, last = n_problems - 1;
    long nq = argc > 2 ? atol(argv[2]) : 20000;
    if (argc > 1 && sscanf(argv[1], "%d:%d", &first, &last) == 1) last = first;
    const int32_t w_min = -19497;
    static fx_env_t env;
    void *mem = malloc(1u << 28);
    uint64_t total = 0, mism = 0, hits = 0, edge_q = 0, edge_hit = 0, cells_max = 0, problems_n = 0;
    uint32_t pts_min = 0xffffffffu, pts_max = 0, cells_n_max = 0;
    for (int i = first; i <= last && i < n_problems; ++i) {
        if (!problems[i].valid) continue;
        arena_init(mem, 1u << 28);
        points_t raw;
        pc_sample(&problems[i], &raw);
        fx_env_build(raw.x, raw.y, raw.z, raw.n, w_min, &env);
        if (!env.valid) { printf("ENVERR,%d\n", i); continue; }
        problems_n++;
        if (env.n_pts < pts_min) pts_min = env.n_pts;
        if (env.n_pts > pts_max) pts_max = env.n_pts;
        if (env.n_cells > cells_n_max) cells_n_max = env.n_cells;
        for (long t = 0; t < nq; ++t) {
            int32_t c[3];
            uint32_t r;
            int edge = (t % 4) == 0 && env.n_pts;
            if (edge) {
                // exact boundary: centre = point + integer offset with |offset| = r
                uint32_t pi = xr() % env.n_pts;
                const int *qd = quad[xr() % 6];
                int32_t k = rnd(1, 800);
                const uint16_t p[3] = {env.px[pi], env.py[pi], env.pz[pi]};
                int perm = (int)(xr() % 3);
                for (int a = 0; a < 3; ++a) c[a] = p[a] + ((xr() & 1) ? 1 : -1) * qd[(a + perm) % 3] * k;
                r = (uint32_t)(qd[3] * k);
                if (r > 8192) r = 8192, c[0] = p[0] + 8192, c[1] = p[1], c[2] = p[2];
            } else {
                for (int a = 0; a < 3; ++a) c[a] = rnd(-6000, 45000);   // includes outside the workspace
                r = (uint32_t)rnd(0, 8192);
            }
            int a = fx_cse_ref(&env, c[0], c[1], c[2], r), b = fx_cse_def(&env, c[0], c[1], c[2], r);
            total++;
            hits += (uint64_t)b;
            if (edge) { edge_q++; edge_hit += (uint64_t)b; }
            if (a != b) {
                if (mism < 10) printf("MISMATCH,prob=%d,c=%d,%d,%d,r=%u,ref=%d,def=%d\n", i, c[0], c[1], c[2], r, a, b);
                mism++;
            }
        }
        (void)cells_max;
    }
    printf("ENVCHECK,problems=%llu,queries=%llu,hits=%llu,edge_queries=%llu,edge_hits=%llu,mismatch=%llu,"
           "pts_min=%u,pts_max=%u,cells_max=%u\n",
           (unsigned long long)problems_n, (unsigned long long)total, (unsigned long long)hits,
           (unsigned long long)edge_q, (unsigned long long)edge_hit, (unsigned long long)mism, pts_min, pts_max, cells_n_max);
    printf("CSESTATS,queries=%llu,gculled=%llu,cells=%llu,nonempty=%llu,aabb_pass=%llu,points=%llu\n",
           (unsigned long long)fx_cs.queries, (unsigned long long)fx_cs.gculled, (unsigned long long)fx_cs.cells,
           (unsigned long long)fx_cs.nonempty, (unsigned long long)fx_cs.aabb_pass, (unsigned long long)fx_cs.points);
    return mism ? 1 : 0;
}
