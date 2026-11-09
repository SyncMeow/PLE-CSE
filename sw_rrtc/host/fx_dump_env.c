// Writes the format-v0 environment of problem i (same build as rrtc_fx / fx_ref_env_load) as the raw bytes
// of fx_env_t, for the SoC baseline image (soc/main_fx_soc.c with FX_ENV_BLOB): the RTL simulation then
// skips point sampling and the environment build. Layout is ABI independent (no pointers, no 64-bit fields).
//   fx_dump_env <problem> <out.bin>
#include <stdio.h>
#include <stdlib.h>
#include "env.h"
#include "problems.h"
#include "fx.h"
#include "fx_pose.h"
#include "fx_env.h"

void fatal(const char *msg)
{
    fprintf(stderr, "fatal: %s\n", msg);
    exit(2);
}

static fx_env_t e;

int main(int argc, char **argv)
{
    if (argc != 3) { fprintf(stderr, "usage: fx_dump_env <problem> <out.bin>\n"); return 1; }
    int i = atoi(argv[1]);
    void *mem = malloc(1u << 27);
    points_t raw;
    arena_init(mem, 1u << 27);
    pc_sample(&problems[i], &raw);
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0);
    fx_env_build(raw.x, raw.y, raw.z, raw.n, fx_w_min, &e);
    if (!e.valid) fatal("environment invalid");
    FILE *f = fopen(argv[2], "wb");
    if (!f || fwrite(&e, sizeof e, 1, f) != 1) fatal("write");
    fclose(f);
    printf("FXENV,%d,%u,%u,%zu\n", i, e.n_pts, e.n_cells, sizeof e);
    return 0;
}
