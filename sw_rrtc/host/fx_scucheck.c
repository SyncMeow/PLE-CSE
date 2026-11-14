// SCU row filter checks (fx_pose.c): (1) every inner sphere b is rigid with the outer sphere B of its link: the
// computed distance |c_b - c_B| stays within 4 e_c of its value at q = 0 (the filter threshold proof needs this);
// (2) fx_self_ok_filt == fx_self_ok on every sampled pose.   fx_scucheck [poses]
#include "fx_pose.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static double dist(const int32_t *a, const int32_t *b)
{
    double d2 = 0;
    for (int k = 0; k < 3; ++k) { double d = (double)a[k] - b[k]; d2 += d * d; }
    return sqrt(d2);
}

int main(int argc, char **argv)
{
    long n = argc > 1 ? atol(argv[1]) : 200000, mismatch = 0, hits = 0;
    const double ec = sqrt(3.0) + FX_DELTA_FK_MM * 1e-3 / FX_WS_LSB_M;
    const int16_t q0[7] = {0};
    int32_t c0[FX_N_SPHERES][3], c[FX_N_SPHERES][3];
    double worst = 0, tests = 0;
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0);
    fx_fk_ws(q0, c0);
    srand(7);
    for (long t = 0; t < n; ++t) {
        int16_t q[7];
        uint32_t nt;
        for (int j = 0; j < 7; ++j) q[j] = (int16_t)(rand() & 0xffff);
        fx_fk_ws(q, c);
        for (int o = 0; o < FX_N_SELF_OUTER; ++o)
            for (int i = fx_self_off[o]; i < fx_self_off[o + 1]; ++i) {
                int b = fx_self_inner[i][1], ob = fx_self_outer[o][1];
                double d = fabs(dist(c[b], c[ob]) - dist(c0[b], c0[ob]));
                if (d > worst) worst = d;
            }
        int plain = fx_self_ok(c), filt = fx_self_ok_filt(c, &nt);
        mismatch += plain != filt;
        hits += !plain;
        tests += nt;
    }
    printf("SCUCHECK,poses=%ld,self_collisions=%ld,mismatch=%ld,rigid_worst_lsb=%.2f,rigid_limit_lsb=%.2f,mean_tests=%.1f\n", n, hits,
           mismatch, worst, 4.0 * ec, tests / (double)n);
    return (mismatch != 0 || worst > 4.0 * ec) ? 1 : 0;
}
