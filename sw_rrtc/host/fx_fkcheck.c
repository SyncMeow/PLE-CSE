// delta_FK: max |fixed FK centre - float FK centre| over random joint vectors (same quantized q).
//   fx_fkcheck [N] [seed]
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "panda.h"
#include "fx.h"
#include "fx_fk.h"

void fatal(const char *msg) { fprintf(stderr, "fatal: %s\n", msg); exit(2); }

static uint64_t rs = 88172645463325252ull;
static uint32_t xr(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return (uint32_t)(rs >> 32); }

int main(int argc, char **argv)
{
    long n = argc > 1 ? atol(argv[1]) : 10000000L;
    if (argc > 2) rs ^= (uint64_t)atol(argv[2]) * 0x9E3779B97F4A7C15ull;
    const double lsb = 1.0 / (double)(1ll << fx_fk_frac);
    double worst = 0.0, sum = 0.0, vamp_worst = 0.0;
    long worst_i = 0;
    int worst_s = 0;
    for (long it = 0; it < n; ++it) {
        int16_t q[FX_DIM];
        float f[8] = {0};
        for (int j = 0; j < FX_DIM; ++j) {
            int lo = fx_q_from_float(panda_s_a[j]), span = fx_q_from_float(panda_s_m[j]);
            q[j] = (int16_t)(lo + (int)(xr() % (uint32_t)span));
            f[j] = (float)q[j] / (float)(1 << FX_JFRAC);
        }
        int64_t c[FX_N_SPHERES][3];
        double ref[FX_N_SPHERES][3], xd[FX_DIM];
        float vf[PANDA_N_SPHERES][4];
        for (int j = 0; j < FX_DIM; ++j) xd[j] = (double)f[j];
        fx_fk_base(q, c);
        fx_fk_ref(xd, ref);
        panda_sphere_fk(f, vf);
        for (int s = 0; s < FX_N_SPHERES; ++s) {
            double e2 = 0.0, v2 = 0.0;
            for (int k = 0; k < 3; ++k) {
                double d = (double)c[s][k] * lsb - ref[s][k];
                e2 += d * d;
                if (s < PANDA_N_SPHERES) { double w = (double)vf[s][k] - ref[s][k]; v2 += w * w; }
            }
            double e = sqrt(e2);
            sum += e;
            if (e > worst) { worst = e; worst_i = it; worst_s = s; }
            if (sqrt(v2) > vamp_worst) vamp_worst = sqrt(v2);
        }
    }
    printf("FKCHECK,F=%d,n=%ld,max_err_mm=%.5f,mean_err_mm=%.6f,worst_iter=%ld,worst_sphere=%d,vamp_float_vs_ref_mm=%.6f\n", fx_fk_frac, n,
           worst * 1e3, sum / ((double)n * FX_N_SPHERES) * 1e3, worst_i, worst_s, vamp_worst * 1e3);
    return 0;
}
