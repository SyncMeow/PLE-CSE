// Golden-model driver: fixed-point planner (fx/) on the MotionBenchMaker Panda problems.
// Step 1: poses are checked by the float FK + MVT (src/cc.c); later steps swap in fixed-point checkers.
//   rrtc_fx [-p first:last] [-S scenes] [-n seeds] [-s seed0] [-i max_iter] [-m step] [-d delta_fk_mm] [-f]
//   -m 1: float FK + float MVT; -m 2: fixed FK + fixed self collision + float MVT;
//   -m 3: all fixed point, environment format v0 + cse_ref (default); -V: check every query against the definition
//   -T dir: write the environment image and every query with its expected result (CSE testbench)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "bench.h"
#include "cc.h"
#include "problems.h"
#include "fx.h"
#include "fx_pose.h"
#include "fx_env.h"

static const mvt_t *g_mvt;
static fx_env_t g_env;
static int g_verify;
static uint64_t g_verify_bad;
static FILE *g_trace;

void fatal(const char *msg)
{
    fprintf(stderr, "fatal: %s\n", msg);
    exit(2);
}

static int pose_ok_float(const int16_t *q)
{
    float f[8] = {0};
    for (int j = 0; j < FX_DIM; ++j) f[j] = (float)q[j] / (float)(1 << FX_JFRAC);   // exact
    return cc_validate_pose(f);
}

// Step 2 env oracle: fixed centre back to metres, radius without r_point (the MVT adds it)
static int env_float(int32_t cx, int32_t cy, int32_t cz, uint32_t rq)
{
    const double l = FX_WS_LSB_M;
    return mvt_sphere_collides(g_mvt, (float)((cx + fx_w_min) * l), (float)((cy + fx_w_min) * l),
                               (float)((cz + fx_w_min) * l), (float)(rq * l - 0.0025));
}

// Step 3 env oracle: CSE reference on the format-v0 environment
static int env_fx(int32_t cx, int32_t cy, int32_t cz, uint32_t rq)
{
    int h = fx_cse_ref(&g_env, cx, cy, cz, rq);
    if (g_verify && h != fx_cse_def(&g_env, cx, cy, cz, rq)) g_verify_bad++;
    // tag user kill_on_hit env_buf cx cy cz rq expected; user = {slot[3:0], sphere_id[7:0], 4'b0}
    if (g_trace) fprintf(g_trace, "%u %u %d 0 %d %d %d %u %d\n", fx_cur_edge & 1u,
                         ((fx_cur_pose & 7u) << 12) | ((uint32_t)fx_cur_sphere << 4), fx_cur_fine, cx, cy, cz, rq, h);
    return h;
}

int main(int argc, char **argv)
{
    int first = 0, last = n_problems - 1, nseeds = 1, seed0 = 0, opt, step = 3, flat = 0;
    const char *trace_dir = 0;
    double delta_mm = FX_DELTA_FK_MM;
    uint32_t scene_mask = 0x7f;
    fx_settings_t s;
    fx_default_settings(&s);
    while ((opt = getopt(argc, argv, "p:S:n:s:i:m:d:fVT:")) != -1) {
        switch (opt) {
        case 'p': if (sscanf(optarg, "%d:%d", &first, &last) == 1) last = first; break;
        case 'S': scene_mask = 0; for (char *c = optarg; *c; ++c) if (*c >= '0' && *c <= '6') scene_mask |= 1u << (*c - '0'); break;
        case 'n': nseeds = atoi(optarg); break;
        case 's': seed0 = atoi(optarg); break;
        case 'i': s.max_iterations = (uint32_t)atol(optarg); break;
        case 'm': step = atoi(optarg); break;
        case 'd': delta_mm = atof(optarg); break;
        case 'f': flat = 1; break;
        case 'V': g_verify = 1; break;
        case 'T': trace_dir = optarg; break;
        default: fprintf(stderr, "bad option\n"); return 1;
        }
    }
    fx_pose_init(delta_mm * 1e-3, flat, step == 3 ? env_fx : env_float);
    printf("FXCFG,step=%d,delta_fk_mm=%g,flat=%d,fk_frac=%d\n", step, delta_mm, flat, fx_fk_frac);
    const uint32_t arena_bytes = 1u << 30;
    void *mem = malloc(arena_bytes);
    static fx_result_t r;
    if (!mem) fatal("malloc");
    for (int i = first; i <= last && i < n_problems; ++i) {
        if (!(scene_mask & (1u << problems[i].scene)) || !problems[i].valid) continue;
        arena_init(mem, arena_bytes);
        mvt_t mvt;
        points_t filt;
        prep_t pp;
        if (step == 3) {
            points_t raw;
            pc_sample(&problems[i], &raw);
            fx_env_build(raw.x, raw.y, raw.z, raw.n, fx_w_min, &g_env);
            if (!g_env.valid) { printf("FXENVERR,%u,%u\n", problems[i].scene, problems[i].index); continue; }
            if (trace_dir) {
                char tag[32];
                snprintf(tag, sizeof tag, "s%u_p%u", problems[i].scene, problems[i].index);
                if (fx_env_dump(&g_env, trace_dir, tag)) fatal("cannot write trace");
            }
            printf("FXPREP,%u,%u,%u,%u\n", problems[i].scene, problems[i].index, g_env.n_pts, g_env.n_cells);
        } else {
            bench_prepare(i, &mvt, &filt, &pp);
            g_mvt = &mvt;
        }
        int16_t qs[FX_DIM], qg[FX_DIM];
        for (int j = 0; j < FX_DIM; ++j) { qs[j] = fx_q_from_float(problems[i].start[j]); qg[j] = fx_q_from_float(problems[i].goal[j]); }
        for (int k = 0; k < nseeds; ++k) {
            s.seed = (uint32_t)(seed0 + k);
            fx_ps = (fx_pose_stats_t){0};
            if (trace_dir) {
                char path[512];
                snprintf(path, sizeof path, "%s/s%u_p%u_seed%u_queries.txt", trace_dir, problems[i].scene, problems[i].index, s.seed);
                if (!(g_trace = fopen(path, "w"))) fatal("cannot write trace");
                fprintf(g_trace, "# tag user kill_on_hit env_buf cx cy cz rq expected\n");
            }
            fx_solve(qs, qg, &s, step == 1 ? pose_ok_float : fx_pose_ok, &r);
            if (g_trace) { fclose(g_trace); g_trace = 0; }
            // FXRUN,scene,index,seed,status,iterations,size_start,size_goal,dd_rejects,edges,poses,nn_queries,path_len,path_hash,cost,env_queries,self_pairs
            printf("FXRUN,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%08x,%.6f,%llu,%llu\n", problems[i].scene, problems[i].index,
                   s.seed, r.status, r.iterations, r.size_start, r.size_goal, r.dd_rejects, r.edges, r.poses,
                   r.nn_queries, r.path_len, r.path_hash, (double)r.cost,
                   (unsigned long long)fx_ps.env_queries, (unsigned long long)fx_ps.self_pairs);
        }
        fflush(stdout);
    }
    if (g_verify) printf("FXVERIFY,mismatch=%llu,queries=%llu\n", (unsigned long long)g_verify_bad, (unsigned long long)fx_cs.queries);
    printf("FXCSE,queries=%llu,gculled=%llu,cells=%llu,nonempty=%llu,aabb_pass=%llu,points=%llu,hits=%llu\n",
           (unsigned long long)fx_cs.queries, (unsigned long long)fx_cs.gculled, (unsigned long long)fx_cs.cells,
           (unsigned long long)fx_cs.nonempty, (unsigned long long)fx_cs.aabb_pass, (unsigned long long)fx_cs.points,
           (unsigned long long)fx_cs.hits);
    return 0;
}
