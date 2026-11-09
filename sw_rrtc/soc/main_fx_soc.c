// SoC driver for the fixed-point golden planner (fx_solve, all fixed point, cse_ref in software):
// CPU baseline for the PLE cycle comparison. Runs the tb_core problem list x FX_SEEDS and prints
// FXRUN lines (same fields as rrtc_fx) plus mcycle counts of fx_solve.
// FX_PROFILE=1 also splits the cycles into environment queries (CSE work) and pose checks (FK + self + env).
#include <stdio.h>
#include <stdint.h>
#include "bench.h"
#include "problems.h"
#include "timer.h"
#include "fx.h"
#include "fx_pose.h"
#include "fx_env.h"

#ifndef ARENA_END
#define ARENA_END 0x64000000u
#endif
#ifndef FX_SEEDS
#define FX_SEEDS 2
#endif
#ifndef FX_PFIRST
#define FX_PFIRST 0
#endif
#ifndef FX_PLAST
#define FX_PLAST 13
#endif
#define HEAP_RESERVE (24u << 20)   // newlib malloc (fx_env_build needs ~13 MB) grows from _end

void fatal(const char *msg)
{
    extern void _exit_spin(void) __attribute__((noreturn));
    printf("FATAL,%s\n", msg);
    _exit_spin();
}

#ifdef FX_ENV_BLOB
// environment built on the host (host/fx_dump_env.c), linked in by soc/env_blob.S; checked by its size
extern const fx_env_t fx_env_blob;
extern const char fx_env_blob_end[];
#define ENV (&fx_env_blob)
#else
static fx_env_t g_env;
#define ENV (&g_env)
#endif
static uint64_t cyc_env, cyc_pose;

static int env_fx(int32_t cx, int32_t cy, int32_t cz, uint32_t rq)
{
#if FX_PROFILE
    uint32_t t0 = tnow();
    int h = fx_cse_ref(ENV, cx, cy, cz, rq);
    cyc_env += tnow() - t0;
    return h;
#else
    return fx_cse_ref(ENV, cx, cy, cz, rq);
#endif
}

static int pose_fx(const int16_t *q)
{
#if FX_PROFILE
    uint32_t t0 = tnow();
    int ok = fx_pose_ok(q);
    cyc_pose += tnow() - t0;
    return ok;
#else
    return fx_pose_ok(q);
#endif
}

// same problems as Platform/accel/tb/tb_core.sv
static const int plist[14] = {1, 40, 100, 160, 210, 260, 320, 380, 410, 470, 530, 600, 650, 690};

int main(void)
{
    extern char _end;
    uint32_t base = ((uint32_t)&_end + HEAP_RESERVE + 63u) & ~63u;
    if (base >= ARENA_END) fatal("image too large");
    static fx_result_t r;
    fx_settings_t s;
    fx_default_settings(&s);
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, env_fx);
    printf("FXSOC START,profile=%d,seeds=%d\n", FX_PROFILE, FX_SEEDS);
    for (int pi = FX_PFIRST; pi <= FX_PLAST && pi < 14; ++pi) {
        int i = plist[pi];
#ifdef FX_ENV_BLOB
        if (FX_PFIRST != FX_PLAST) fatal("one problem per image with FX_ENV_BLOB");
        if ((uint32_t)(fx_env_blob_end - (const char *)&fx_env_blob) != sizeof(fx_env_t)) fatal("env blob size");
        (void)base;
#else
        arena_init((void *)base, ARENA_END - base);
        points_t raw;
        pc_sample(&problems[i], &raw);
        fx_env_build(raw.x, raw.y, raw.z, raw.n, fx_w_min, &g_env);
#endif
        if (!ENV->valid) { printf("FXENVERR,%d\n", i); continue; }
        printf("FXENV,%d,%lu,%lu\n", i, (unsigned long)ENV->n_pts, (unsigned long)ENV->n_cells);
        int16_t qs[FX_DIM], qg[FX_DIM];
        for (int j = 0; j < FX_DIM; ++j) { qs[j] = fx_q_from_float(problems[i].start[j]); qg[j] = fx_q_from_float(problems[i].goal[j]); }
        for (int k = 0; k < FX_SEEDS; ++k) {
            s.seed = (uint32_t)k;
            fx_ps = (fx_pose_stats_t){0};
            cyc_env = cyc_pose = 0;
            uint64_t t0 = tnow64();
            fx_solve(qs, qg, &s, pose_fx, &r);
            uint64_t t1 = tnow64();
            // FXSOC,prob,seed,status,iterations,size_start,size_goal,dd_rejects,edges,poses,nn_queries,path_len,path_hash,env_queries,self_pairs,cycles,cyc_pose,cyc_env
            printf("FXSOC,%d,%u,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%08lx,%llu,%llu,%llu,%llu,%llu\n", i, (unsigned)s.seed,
                   r.status, (unsigned long)r.iterations, (unsigned long)r.size_start, (unsigned long)r.size_goal,
                   (unsigned long)r.dd_rejects, (unsigned long)r.edges, (unsigned long)r.poses,
                   (unsigned long)r.nn_queries, (unsigned long)r.path_len, (unsigned long)r.path_hash,
                   (unsigned long long)fx_ps.env_queries, (unsigned long long)fx_ps.self_pairs,
                   (unsigned long long)(t1 - t0), (unsigned long long)cyc_pose, (unsigned long long)cyc_env);
        }
    }
    printf("FXSOC DONE\n");
    return 0;
}
