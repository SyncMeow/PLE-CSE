// Morton-window vs brute-force nearest neighbour on synthetic trees of growing size (host only).
// Trees: uniform, clustered (16 Gaussian blobs) and random-walk (each node a 0.3-rad step from a
// random earlier node, the branchy shape RRT grows) inside Panda's joint limits. Queries are
// uniform samples, like RRT-C's. Every node is in the sorted Morton orders (no buffer), and every
// (m, W) of the shadow grid is scored against the exact answer.
//   nn_scale [-q queries] [-s seed]   -> lines NNSCALE,dist,N,qt,m,W,queries,hits,hist...
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "env.h"
#include "nn.h"
#include "panda.h"
#include "stats.h"
#include "vmath.h"


void fatal(const char *msg) { fprintf(stderr, "fatal: %s\n", msg); exit(2); }

static uint64_t rs;
static double urand(void)
{
    rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
    return (double)(rs >> 11) * (1.0 / 9007199254740992.0);
}
static double gauss(void) { return sqrt(-2.0 * log(urand() + 1e-300)) * cos(6.283185307179586 * urand()); }
static float clampj(int j, double v)
{
    double lo = (double)panda_s_a[j], hi = (double)panda_s_a[j] + (double)panda_s_m[j] * 0.999999;
    return (float)(v < lo ? lo : v > hi ? hi : v);
}
static void uniform_q(float *q)
{
    for (int j = 0; j < PANDA_DIM; ++j) q[j] = clampj(j, (double)panda_s_a[j] + (double)panda_s_m[j] * urand());
    q[PANDA_DIM] = 0.0f;
}

int main(int argc, char **argv)
{
    int Q = 20000, opt;
    uint64_t seed = 1;
    while ((opt = getopt(argc, argv, "q:s:")) != -1) {
        if (opt == 'q') Q = atoi(optarg);
        else if (opt == 's') seed = strtoull(optarg, 0, 10);
    }
    static const int NS[] = {100, 300, 1000, 3000, 10000, 30000, 100000};
    const char *DIST[] = {"uniform", "clustered", "walk"};
    const uint32_t CAP = 100000;
    const uint32_t bytes = 1u << 30;
    void *mem = malloc(bytes);
    if (!mem) fatal("malloc");
    for (int d = 0; d < 3; ++d)
        for (size_t ni = 0; ni < sizeof(NS) / sizeof(NS[0]); ++ni) {
            const uint32_t N = (uint32_t)NS[ni];
            rs = 0x9E3779B97F4A7C15ULL ^ (seed * 1000003u + (uint64_t)d * 101u + ni);
            arena_init(mem, bytes);
            nn_params_t p = {NN_BRUTE, 8, 8, 64, 9};
            float *buf = arena_alloc(CAP * NN_STRIDE * 4);
            nn_shared_alloc(&p, CAP);
            nn_tree_t t;
            nn_init(&t, &p, buf, 0, CAP);
            float centre[16][PANDA_DIM];
            for (int c = 0; c < 16; ++c) uniform_q(centre[c]);
            for (uint32_t i = 0; i < N; ++i) {
                float *q = buf + (size_t)i * NN_STRIDE;
                if (d == 0 || i == 0) uniform_q(q);
                else if (d == 1) {
                    const float *c = centre[(int)(urand() * 16) & 15];
                    for (int j = 0; j < PANDA_DIM; ++j) q[j] = clampj(j, (double)c[j] + 0.05 * (double)panda_s_m[j] * gauss());
                } else {
                    const float *par = buf + (size_t)(urand() * i) * NN_STRIDE;
                    double dir[PANDA_DIM], nrm = 0.0;
                    for (int j = 0; j < PANDA_DIM; ++j) { dir[j] = gauss(); nrm += dir[j] * dir[j]; }
                    nrm = sqrt(nrm);
                    for (int j = 0; j < PANDA_DIM; ++j) q[j] = clampj(j, (double)par[j] + 0.3 * dir[j] / nrm);
                }
                q[PANDA_DIM] = 0.0f;
                nn_insert(&t, i);
            }
            nn_flush(&t);
            static shadow_t sh;
            memset(&sh, 0, sizeof(sh));
            for (int k = 0; k < Q; ++k) {
                float q[NN_STRIDE], ed2;
                uniform_q(q);
                nn_nearest_exact(&t, q, &ed2);
                nn_shadow(&t, q, ed2, 0, &sh);
            }
            for (int mi = 0; mi < SH_NM; ++mi)
                for (int wi = 0; wi < SH_NW; ++wi) {
                    printf("NNSCALE,%s,%u,%d,%d,%u,%u", DIST[d], N, sh_m[mi], sh_W[wi], sh.queries[0][mi][wi],
                           sh.hits[0][mi][wi]);
                    for (int b = 0; b < SH_BINS; ++b) printf(",%u", sh.hist[0][mi][wi][b]);
                    printf("\n");
                }
            fflush(stdout);
        }
    return 0;
}
