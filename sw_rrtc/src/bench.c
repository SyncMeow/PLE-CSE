#include "bench.h"
#include "cc.h"
#include "vmath.h"
#include "problems.h"
#include "timer.h"
#include <stdio.h>
#include <string.h>

stats_t st;

void bench_prepare(int i, mvt_t *mvt, points_t *filtered, prep_t *pp)
{
    const problem_t *p = &problems[i];
    points_t raw;
    memset(pp, 0, sizeof(*pp));
    uint32_t mark = arena_mark();
    pc_sample(p, &raw);
    pp->raw_n = raw.n;
    uint64_t t0 = tnow64();
    centervox(&raw, filtered);   // releases its own scratch; raw stays below the result
    uint64_t t1 = tnow64();
    mvt_build(mvt, filtered, PANDA_MAX_RADIUS);
    uint64_t t2 = tnow64();
    (void)mark;
    pp->filt_n = filtered->n;
    pp->n_vox = mvt->n_vox;
    pp->pool_used = mvt->pool_used;
    pp->n_pool = mvt->n_pool;
    pp->cyc_filter = t1 - t0;
    pp->cyc_build = t2 - t1;
    cc_mvt = mvt;
}

void bench_run(int i, const rrtc_settings_t *s, rrtc_result_t *r, shadow_t *sh)
{
    const problem_t *p = &problems[i];
    uint32_t mark = arena_mark();
    memset(&st, 0, sizeof(st));
    if (sh) memset(sh, 0, sizeof(*sh));
    uint64_t t0 = tnow64();
    rrtc_solve(p->start, p->goal, s, r, sh);
    st.cyc_total = tnow64() - t0;
    arena_release(mark);
}


void bench_print_prep(int i, const prep_t *pp)
{
    const problem_t *p = &problems[i];
    printf("PREP,%u,%u,%lu,%lu,%lu,%lu,%lu,%llu,%llu\n", p->scene, p->index, (unsigned long)pp->raw_n,
           (unsigned long)pp->filt_n, (unsigned long)pp->n_vox, (unsigned long)pp->pool_used,
           (unsigned long)pp->n_pool, (unsigned long long)pp->cyc_filter, (unsigned long long)pp->cyc_build);
}

void bench_print_run(int i, const rrtc_settings_t *s, const rrtc_result_t *r)
{
    const problem_t *p = &problems[i];
    printf("RUN,%u,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
           "%lu,%lu,%lu,%08lx,%08lx,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%lu,%llu\n",
           p->scene, p->index, (unsigned long)s->seed, s->nn.mode, s->nn.m, s->nn.W, s->nn.B, s->nn.bits,
           r->solved, r->cap_hit, (unsigned long)st.iterations, (unsigned long)r->size_start,
           (unsigned long)r->size_goal, (unsigned long)st.nn_queries[QT_EXTEND],
           (unsigned long)st.nn_queries[QT_CONNECT], (unsigned long)st.dd_rejects, (unsigned long)st.nn_inserts,
           (unsigned long)st.edges, (unsigned long)st.checks, (unsigned long)st.poses,
           (unsigned long)st.env_queries, (unsigned long)st.env_hits, (unsigned long)st.env_global_culled,
           (unsigned long)st.voxels, (unsigned long)st.voxels_culled, (unsigned long)st.points,
           (unsigned long)st.self_pairs, (unsigned long)st.morton_merges, (unsigned long)r->path_len,
           (unsigned long)r->path_hash, (unsigned long)f2u(r->cost), (unsigned long long)st.cyc_total,
           (unsigned long long)st.cyc_nn_query, (unsigned long long)st.cyc_nn_insert,
           (unsigned long long)st.cyc_fkcc, (unsigned long long)st.cyc_fk, (unsigned long long)st.cyc_env,
           (unsigned long long)st.cyc_rng, (unsigned long)st.timer_reads, (unsigned long long)st.nn_dist_evals);
}
