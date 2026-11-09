// Finds planner configurations that reach the two boundaries random runs almost never hit, for directed
// RTL tests (Platform/accel/tb/tb_core.sv +directed): every config printed here is re-checked by a full
// golden run that counts the event (fx_cov_dd_eq / fx_cov_conn0).
//   dd:    rad^2 == nd2 << 24 in a DD check. Candidates: DD checks on a node whose radius is still the
//          first-failure value (cfg radius) and nd2 = m^2; rerun with radius = m << 12.
//   conn0: CONNECT with n_ext = 0 (the new node equals a node of the other tree). The Halton rotation is
//          chosen so that sample n equals the goal (or the start); rerun and count.
//   fx_cov_search dd|conn0 <first> <last> <seeds> <range_h> [max_hits]
// Output: COV,<kind>,<problem>,<seed>,<range_h>,<radius>,<min_radius>,<shift0..6>,<events>,<status>,<iterations>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

static fx_env_t env;
static int env_fx(int32_t cx, int32_t cy, int32_t cz, uint32_t rq) { return fx_cse_ref(&env, cx, cy, cz, rq); }

static uint32_t cur_radius;
#define MAXC 4096
static uint32_t cand[MAXC];
static int ncand;
static unsigned long long n_obs, n_obs_r;
static void observer(uint32_t rad, int64_t nd2)
{
    n_obs++; n_obs_r += rad == cur_radius;
    if (ncand >= MAXC || nd2 <= 0) return;
    uint64_t m = fx_isqrt((uint64_t)nd2);
    if ((int64_t)(m * m) != nd2 || m >= (1u << 19)) return;
    // radius setting that would scale this node's radius to m << 12 (if the trajectory stays the same)
    uint64_t rr = (uint64_t)cur_radius * (m << 12) / rad;
    if (rr >= 0xffffffffu) return;
    for (int i = 0; i < ncand; ++i) if (cand[i] == (uint32_t)rr) return;
    cand[ncand++] = (uint32_t)rr;
}

static fx_result_t r;
static void solve(const int16_t *qs, const int16_t *qg, fx_settings_t *s)
{
    fx_cov_dd_eq = fx_cov_conn0 = 0;
    fx_solve(qs, qg, s, fx_pose_ok, &r);
}

static void emit(const char *kind, int i, const fx_settings_t *s, const uint32_t *sh, uint32_t ev)
{
    printf("COV,%s,%d,%u,%u,%u,%u", kind, i, s->seed, s->range_h, s->radius, s->min_radius);
    for (int j = 0; j < FX_DIM; ++j) printf(",%u", sh[j]);
    printf(",%u,%d,%u\n", ev, r.status, r.iterations);
    fflush(stdout);
}

// Halton state (no rotation) after n samples: h_u of fx_plan.c (this tool links fx_plan.c's statics via include)
#include "fx_plan.c"

int main(int argc, char **argv)
{
    if (argc < 6) { fprintf(stderr, "usage: fx_cov_search dd|conn0 first last seeds range_h [max_hits]\n"); return 1; }
    const char *kind = argv[1];
    int first = atoi(argv[2]), last = atoi(argv[3]), nseeds = atoi(argv[4]), max_hits = argc > 6 ? atoi(argv[6]) : 4;
    uint32_t range_h = (uint32_t)strtoul(argv[5], 0, 0);
    void *mem = malloc(1u << 27);
    int hits = 0;
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, env_fx);
    for (int i = first; i <= last && i < n_problems && hits < max_hits; ++i) {
        if (!problems[i].valid) continue;
        points_t raw;
        arena_init(mem, 1u << 27);
        pc_sample(&problems[i], &raw);
        fx_env_build(raw.x, raw.y, raw.z, raw.n, fx_w_min, &env);
        if (!env.valid) continue;
        int16_t qs[FX_DIM], qg[FX_DIM];
        for (int j = 0; j < FX_DIM; ++j) { qs[j] = fx_q_from_float(problems[i].start[j]); qg[j] = fx_q_from_float(problems[i].goal[j]); }
        for (int sd = 0; sd < nseeds && hits < max_hits; ++sd) {
            fx_settings_t s;
            fx_default_settings(&s);
            s.seed = (uint32_t)sd;
            s.range_h = range_h;
            s.max_iterations = 20000;
            if (!strcmp(kind, "dd")) {
                fx_shift_override = 0;
                cur_radius = s.radius;
                ncand = 0;
                fx_dd_observer = observer;
                solve(qs, qg, &s);
                fx_dd_observer = 0;
                int nc = ncand;
                if (getenv("COV_DEBUG")) fprintf(stderr, "p%d s%d status=%d iter=%u dd_rejects=%u cand=%d\n", i, sd, r.status, r.iterations, r.dd_rejects, nc);
                static uint32_t c2[MAXC];
                memcpy(c2, cand, sizeof(uint32_t) * (size_t)nc);
                for (int c = 0; c < nc && hits < max_hits; ++c)
                    for (int d = -24; d <= 24 && hits < max_hits; ++d) {
                        fx_settings_t t = s;
                        t.radius = (uint32_t)((int64_t)c2[c] + d);
                        if (t.radius < t.min_radius) continue;
                        solve(qs, qg, &t);
                        if (fx_cov_dd_eq) { smp_reset(t.seed); emit("dd", i, &t, h_shift, fx_cov_dd_eq); hits++; break; }
                    }
            } else {
                for (int n = 2; n <= 60 && hits < max_hits; ++n) {
                    for (int which = 0; which < 2 && hits < max_hits; ++which) {
                        const int16_t *tq = which ? qs : qg;            // sample n = goal or start
                        static uint32_t sh[FX_DIM];
                        int16_t dummy[FX_DIM];
                        int ok = 1;
                        fx_shift_override = 0;
                        smp_reset(0);
                        for (int k = 0; k < n; ++k) smp_next(dummy);
                        for (int j = 0; j < FX_DIM && ok; ++j) {
                            int32_t t = tq[j] - smp_a[j];
                            if (t < 0 || t >= smp_m[j]) { ok = 0; break; }
                            uint32_t v = (uint32_t)(((uint64_t)t << 16) + smp_m[j] - 1) / smp_m[j];
                            if ((int32_t)(((uint64_t)v * smp_m[j]) >> 16) != t || v > 0xffff) { ok = 0; break; }
                            sh[j] = (v << 16) + 0x8000u - h_u[j];
                        }
                        if (!ok) continue;
                        fx_shift_override = sh;
                        solve(qs, qg, &s);
                        if (fx_cov_conn0) { emit("conn0", i, &s, sh, fx_cov_conn0); hits++; }
                        fx_shift_override = 0;
                    }
                }
            }
        }
    }
    fprintf(stderr, "hits=%d dd_checks=%llu at_cfg_radius=%llu\n", hits, n_obs, n_obs_r);
    return hits ? 0 : 3;
}
