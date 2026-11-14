// Dumps the golden self-collision pair tables and thresholds (fx_pose.c) for the SCU RTL generator.
//   OUTER <o> <a> <b> <thr>       INNER <o> <a> <b> <thr>       FILT <o> <a> <thr> (row a against outer sphere b of o)
#include <stdio.h>
#include "fx_pose.h"

int main(void)
{
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0);                    // same eps as rrtc_fx
    for (int o = 0; o < FX_N_SELF_OUTER; ++o)
        printf("OUTER %d %d %d %llu\n", o, fx_self_outer[o][0], fx_self_outer[o][1],
               (unsigned long long)fx_self_thr(o));
    for (int o = 0; o < FX_N_SELF_OUTER; ++o)
        for (int i = fx_self_off[o]; i < fx_self_off[o + 1]; ++i)
            printf("INNER %d %d %d %llu\n", o, fx_self_inner[i][0], fx_self_inner[i][1],
                   (unsigned long long)fx_self_thr(FX_N_SELF_OUTER + i));
    for (int o = 0; o < FX_N_SELF_OUTER; ++o)
        for (int r = 0; r < fx_self_rows(o); ++r)
            printf("FILT %d %d %llu\n", o, fx_self_row_a(o, r), (unsigned long long)fx_self_fthr(o, r));
    return 0;
}
