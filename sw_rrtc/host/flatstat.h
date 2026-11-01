// Host-only flat robot-sphere workload counters (see flatstat.c).
#pragma once
#include <stdint.h>
#include "env.h"

#define FS_HC 28     // cells / non-empty voxels per query: 0..27
#define FS_HB 28     // blocks per query
#define FS_RD 22     // reuse-distance bins: 0, [1,2), [2,4) ... , last = cold

typedef struct {
    uint64_t runs;
    // (3) workload of the flat stream
    uint64_t poses, colliding_poses, q_to_first_hit_pose;
    uint64_t queries, culled, hits, cells, nonempty, aabb_pass, points;
    uint64_t blocks[3], blocks_occ[3];                  // block sides 2, 4, 8
    uint64_t h_cells[FS_HC], h_nonempty[FS_HC], h_blocks[3][FS_HB];
    // (2) batch order, P = 8/16/32 poses, orders: pose-major, sphere-major, Morton
    uint64_t ord_batches[3], ord_queries[3], ord_hit_batches[3][3], ord_q_to_hit[3][3];
    uint64_t rd_full[3][FS_RD], rd_ee[3][FS_RD];        // P = 32
} flatstat_t;

extern flatstat_t fs;
extern int fs_reuse_on;

void fs_problem(const mvt_t *t);
void fs_run_begin(void);
