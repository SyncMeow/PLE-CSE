// Host-only experiment counters (see envstat.c).
#pragma once
#include <stdint.h>

#define ES_HC 260      // histogram: voxel cells per query (last bin = overflow)
#define ES_HN 64       // non-empty voxels per query
#define ES_HB 32       // Morton blocks / bank cycles per query
#define ES_NLRU 4
#define ES_LRU_MAX 64
static const int es_lru_sizes[ES_NLRU] = {4, 8, 16, 64};

typedef struct {
    uint64_t queries, gculled, hits;
    uint64_t cells, cells_exit, xlook, ylook, zlook;
    uint64_t nonempty, nonempty_exit, points, points_exit, points_tested;   // points_tested = st.points
    uint64_t blocks, blocks_occ, bank_cycles;
    uint32_t h_cells[ES_HC], h_nonempty[ES_HN], h_blocks[ES_HB], h_bankcyc[ES_HB];
} es_class_t;

typedef struct {
    uint64_t voxel_refs, point_refs, distinct, distinct_points;
    uint64_t misses[ES_NLRU], miss_points[ES_NLRU];
    uint64_t hit_batches, queries_to_first_hit;
    uint64_t q_ee, refs_ee, misses_ee[ES_NLRU], miss_points_ee[ES_NLRU];   // with STOP_ON_FIRST_HIT
} es_order_t;

typedef struct {
    es_class_t cls[2];     // 0: robot spheres (r <= r_max), 1: link bounding spheres
    es_order_t ord[3];     // 0: as issued, 1: sphere-major, 2: Morton order of the centre voxel
    uint64_t batches, batch_queries;
    uint64_t dropped, seen_overflow, hit_mismatch, outside_edge;   // must all be 0
} envstat_t;

extern envstat_t es;
extern int es_reuse_on;
