// Host driver: same planner/collision code as the SoC, many runs, shadow-mode Morton scoring.
//   rrtc_host [-p first:last] [-S scenes] [-n seeds] [-s seed0] [-N brute|morton] [-m M] [-W W]
//             [-B B] [-b bits] [-i max_iter] [-c max_samples] [-H shadow.csv] [-P]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "bench.h"
#include "problems.h"
#if defined(FLAT_STATS)
#include "flatstat.h"

static void pr(const char *tag, const uint64_t *v, int n)
{
    printf("%s", tag);
    for (int i = 0; i < n; ++i) printf(",%llu", (unsigned long long)v[i]);
    printf("\n");
}

static void print_flatstat(void)
{
    pr("FSRUNS", &fs.runs, 1);
    pr("FSPOSE", &fs.poses, 3);
    pr("FSQ", &fs.queries, 7);
    pr("FSBLK", fs.blocks, 3);
    pr("FSBLKOCC", fs.blocks_occ, 3);
    pr("FSHC", fs.h_cells, FS_HC);
    pr("FSHN", fs.h_nonempty, FS_HC);
    for (int b = 0; b < 3; ++b) pr(b == 0 ? "FSHB2" : b == 1 ? "FSHB4" : "FSHB8", fs.h_blocks[b], FS_HB);
    pr("FSOB", fs.ord_batches, 3);
    pr("FSOQ", fs.ord_queries, 3);
    pr("FSOHB", &fs.ord_hit_batches[0][0], 9);
    pr("FSOQH", &fs.ord_q_to_hit[0][0], 9);
    for (int o = 0; o < 3; ++o) { char t[16]; snprintf(t, sizeof t, "FSRDF%d", o); pr(t, fs.rd_full[o], FS_RD); }
    for (int o = 0; o < 3; ++o) { char t[16]; snprintf(t, sizeof t, "FSRDE%d", o); pr(t, fs.rd_ee[o], FS_RD); }
}
#elif defined(ENV_STATS)
#include "envstat.h"

static void print_envstat(void)
{
    for (int c = 0; c < 2; ++c) {
        const es_class_t *k = &es.cls[c];
        printf("ESCLS,%d,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu", c,
               (unsigned long long)k->queries, (unsigned long long)k->gculled, (unsigned long long)k->hits,
               (unsigned long long)k->cells, (unsigned long long)k->cells_exit, (unsigned long long)k->xlook,
               (unsigned long long)k->ylook, (unsigned long long)k->zlook, (unsigned long long)k->nonempty,
               (unsigned long long)k->nonempty_exit, (unsigned long long)k->points, (unsigned long long)k->points_exit,
               (unsigned long long)k->blocks, (unsigned long long)k->blocks_occ, (unsigned long long)k->bank_cycles);
        printf(",%llu", (unsigned long long)k->points_tested);
        printf(",HC"); for (int i = 0; i < ES_HC; ++i) printf(",%u", k->h_cells[i]);
        printf(",HN"); for (int i = 0; i < ES_HN; ++i) printf(",%u", k->h_nonempty[i]);
        printf(",HB"); for (int i = 0; i < ES_HB; ++i) printf(",%u", k->h_blocks[i]);
        printf(",HK"); for (int i = 0; i < ES_HB; ++i) printf(",%u", k->h_bankcyc[i]);
        printf("\n");
    }
    printf("ESCHK,dropped=%llu,seen_overflow=%llu,hit_mismatch=%llu,outside_edge=%llu\n",
           (unsigned long long)es.dropped, (unsigned long long)es.seen_overflow,
           (unsigned long long)es.hit_mismatch, (unsigned long long)es.outside_edge);
    for (int o = 0; o < 3; ++o) {
        const es_order_t *k = &es.ord[o];
        printf("ESORD,%d,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu", o, (unsigned long long)es.batches,
               (unsigned long long)es.batch_queries, (unsigned long long)k->voxel_refs, (unsigned long long)k->point_refs,
               (unsigned long long)k->distinct, (unsigned long long)k->distinct_points,
               (unsigned long long)k->hit_batches, (unsigned long long)k->queries_to_first_hit);
        for (int i = 0; i < ES_NLRU; ++i) printf(",%d,%llu,%llu", es_lru_sizes[i], (unsigned long long)k->misses[i], (unsigned long long)k->miss_points[i]);
        printf(",EE,%llu,%llu", (unsigned long long)k->q_ee, (unsigned long long)k->refs_ee);
        for (int i = 0; i < ES_NLRU; ++i) printf(",%llu,%llu", (unsigned long long)k->misses_ee[i], (unsigned long long)k->miss_points_ee[i]);
        printf("\n");
    }
}
#endif

void fatal(const char *msg)
{
    fprintf(stderr, "fatal: %s\n", msg);
    exit(2);
}

static void print_shadow(FILE *f, int i, uint32_t seed, const shadow_t *sh)
{
    const problem_t *p = &problems[i];
    for (int q = 0; q < QT_SHADOW_N; ++q)
        for (int mi = 0; mi < SH_NM; ++mi)
            for (int wi = 0; wi < SH_NW; ++wi) {
                fprintf(f, "SHADOW,%u,%u,%u,%d,%d,%d,%u,%u", p->scene, p->index, seed, q, sh_m[mi], sh_W[wi],
                        sh->queries[q][mi][wi], sh->hits[q][mi][wi]);
                for (int b = 0; b < SH_BINS; ++b) fprintf(f, ",%u", sh->hist[q][mi][wi][b]);
                fputc('\n', f);
            }
}

int main(int argc, char **argv)
{
    int first = 0, last = n_problems - 1, nseeds = 1, seed0 = 0, opt, prep_only = 0;
    uint32_t scene_mask = 0x7f;
    const char *shadow_path = 0;
    rrtc_settings_t s;
    rrtc_default_settings(&s);
    while ((opt = getopt(argc, argv, "p:S:n:s:N:m:W:B:b:i:c:H:PR")) != -1) {
        switch (opt) {
        case 'p': if (sscanf(optarg, "%d:%d", &first, &last) == 1) last = first; break;
        case 'S': scene_mask = 0; for (char *c = optarg; *c; ++c) if (*c >= '0' && *c <= '6') scene_mask |= 1u << (*c - '0'); break;
        case 'n': nseeds = atoi(optarg); break;
        case 's': seed0 = atoi(optarg); break;
        case 'N': s.nn.mode = strcmp(optarg, "morton") == 0 ? NN_MORTON : NN_BRUTE; break;
        case 'm': s.nn.m = atoi(optarg); break;
        case 'W': s.nn.W = atoi(optarg); break;
        case 'B': s.nn.B = atoi(optarg); break;
        case 'b': s.nn.bits = atoi(optarg); break;
        case 'i': s.max_iterations = (uint32_t)atol(optarg); break;
        case 'c': s.max_samples = (uint32_t)atol(optarg); break;
        case 'H': shadow_path = optarg; break;
        case 'P': prep_only = 1; break;
#if defined(FLAT_STATS)
        case 'R': fs_reuse_on = 1; break;
#elif defined(ENV_STATS)
        case 'R': es_reuse_on = 1; break;
#endif
        default: fprintf(stderr, "bad option\n"); return 1;
        }
    }
    if (s.nn.mode == NN_MORTON && s.nn.m < 1) s.nn.m = 1;
    if (s.nn.W < 1 || s.nn.B < 1 || s.nn.B > 64 || s.nn.m > NN_MAX_SHIFTS || s.nn.bits < 1 || s.nn.bits > 9) fatal("bad NN params");
    FILE *shf = 0;
    shadow_t *sh = 0;
    if (shadow_path) {
        shf = fopen(shadow_path, "w");
        if (!shf) fatal("cannot open shadow file");
        s.shadow = 1;
        if (s.nn.mode == NN_BRUTE) s.nn.m = sh_m[SH_NM - 1];   // keep all shifted orders up to date
        sh = malloc(sizeof(*sh));
    }
    const uint32_t arena_bytes = 1u << 30;
    void *mem = malloc(arena_bytes);
    if (!mem) fatal("malloc");
    arena_init(mem, arena_bytes);

    for (int i = first; i <= last && i < n_problems; ++i) {
        if (!(scene_mask & (1u << problems[i].scene)) || !problems[i].valid) continue;
        arena_init(mem, arena_bytes);
        mvt_t mvt;
        points_t filt;
        prep_t pp;
        bench_prepare(i, &mvt, &filt, &pp);
        bench_print_prep(i, &pp);
#if defined(FLAT_STATS)
        fs_problem(&mvt);
#endif
        if (prep_only) continue;
        for (int k = 0; k < nseeds; ++k) {
            rrtc_result_t r;
            s.seed = (uint32_t)(seed0 + k);
#if defined(FLAT_STATS)
            fs_run_begin();
#endif
            bench_run(i, &s, &r, sh);
            bench_print_run(i, &s, &r);
            if (shf) print_shadow(shf, i, s.seed, sh);
        }
        fflush(stdout);
    }
    if (shf) fclose(shf);
#if defined(FLAT_STATS)
    print_flatstat();
#elif defined(ENV_STATS)
    print_envstat();
#endif
    return 0;
}
