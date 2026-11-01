// ctypes entry points for host/validate.py (shared library build only).
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "bench.h"
#include "cc.h"
#include "panda.h"
#include "problems.h"

void fatal(const char *msg)
{
    fprintf(stderr, "fatal: %s\n", msg);
    exit(2);
}

static void *mem;
static const uint32_t arena_bytes = 1u << 30;
static mvt_t mvt;
static points_t filt;
static prep_t pp;

int api_n_problems(void) { return n_problems; }
int api_problem_info(int i, int *scene, int *index, int *valid, float *start, float *goal)
{
    const problem_t *p = &problems[i];
    *scene = p->scene; *index = p->index; *valid = p->valid;
    memcpy(start, p->start, 7 * 4);
    memcpy(goal, p->goal, 7 * 4);
    return p->n_prims;
}

// Builds the environment of problem i; returns the filtered point count
int api_prepare(int i)
{
    if (!mem) mem = malloc(arena_bytes);
    arena_init(mem, arena_bytes);
    bench_prepare(i, &mvt, &filt, &pp);
    cc_brute = 0;
    return (int)filt.n;
}
int api_raw_count(void) { return (int)pp.raw_n; }
int api_grid(void) { return mvt.G; }
void api_points(float *xyz)
{
    for (uint32_t k = 0; k < filt.n; ++k) { xyz[3 * k] = filt.x[k]; xyz[3 * k + 1] = filt.y[k]; xyz[3 * k + 2] = filt.z[k]; }
}
// Raw (unfiltered) cloud of problem i: n_prims * 10000 points
int api_raw_points(int i, float *xyz)
{
    if (!mem) mem = malloc(arena_bytes);
    arena_init(mem, arena_bytes);
    points_t raw;
    pc_sample(&problems[i], &raw);
    for (uint32_t k = 0; k < raw.n; ++k) { xyz[3 * k] = raw.x[k]; xyz[3 * k + 1] = raw.y[k]; xyz[3 * k + 2] = raw.z[k]; }
    return (int)raw.n;
}

void api_sphere_fk(const float *q, float *out) { panda_sphere_fk(q, (float (*)[4])out); }
// Pose validity with the MVT (brute = 0), brute force over the filtered cloud (1), or no environment (2)
static points_t empty;
int api_validate(const float *q, int brute)
{
    cc_brute = brute == 1 ? &filt : (brute == 2 ? &empty : 0);
    int ok = cc_validate_pose(q);
    cc_brute = 0;
    return ok;
}
// Single sphere against the cloud
int api_sphere(float x, float y, float z, float r, int brute)
{
    return brute ? brute_sphere_collides(&filt, x, y, z, r) : mvt_sphere_collides(&mvt, x, y, z, r);
}
void api_halton(uint32_t seed, int n, float *out)
{
    halton_reset(seed);
    for (int k = 0; k < n; ++k) { float q[8]; halton_next(q); memcpy(out + 7 * k, q, 28); }
}
float api_sinf(float x) { return vamp_sinf(x); }
float api_cosf(float x) { return vamp_cosf(x); }

// Plans problem i (after api_prepare(i)); out = {solved, iters, size_s, size_g, path_len, hash, cap_hit}
void api_plan(int i, uint32_t seed, int mode, int m, int W, int B, uint32_t *out)
{
    rrtc_settings_t s;
    rrtc_default_settings(&s);
    s.seed = seed; s.nn.mode = mode; s.nn.m = m; s.nn.W = W; s.nn.B = B;
    rrtc_result_t r;
    bench_run(i, &s, &r, 0);
    out[0] = (uint32_t)r.solved; out[1] = st.iterations; out[2] = r.size_start; out[3] = r.size_goal;
    out[4] = r.path_len; out[5] = r.path_hash; out[6] = (uint32_t)r.cap_hit;
}
