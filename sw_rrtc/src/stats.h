// Per-run counters and cycle accumulators.
#pragma once
#include <stdint.h>

// query types; the shadow analysis also splits EXTEND into accepted / rejected by the dynamic domain
enum { QT_EXTEND = 0, QT_CONNECT = 1, QT_N = 2, QT_EXTEND_REJECTED = 2, QT_SHADOW_N = 3 };

typedef struct {
    // planner
    uint32_t iterations;
    uint32_t nn_queries[QT_N];
    uint32_t dd_rejects;         // samples rejected by the dynamic domain after their NN query
    uint32_t nn_inserts;
    uint64_t nn_dist_evals;      // distances computed by the NN search (brute force: sum of tree sizes)
    uint32_t edges;              // validate_vector calls
    uint32_t checks;             // CCU-style CHECK batches (<= 32 poses each)
    uint32_t poses;              // poses actually checked (stops at the first colliding one)
    // collision check (Algorithm 1)
    uint32_t env_queries;        // spheres sent to the environment check
    uint32_t env_hits;
    uint32_t env_global_culled;
    uint32_t voxels;             // non-empty voxels reached
    uint32_t voxels_culled;      // rejected by the voxel's local AABB
    uint32_t points;             // point-sphere distance tests
    uint32_t self_pairs;         // self-collision sphere pairs tested
    uint32_t morton_merges;
    // cycles (SoC only)
    uint64_t cyc_total, cyc_nn_query, cyc_nn_insert, cyc_fkcc, cyc_fk, cyc_env, cyc_rng;
    uint32_t timer_reads;        // for subtracting the instrumentation overhead
} stats_t;

extern stats_t st;
