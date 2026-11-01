// Nearest-neighbour structures for one RRT-C tree: exact brute force and Morton-window search.
#pragma once
#include <stdint.h>
#include "panda.h"

#define NN_MAX_SHIFTS 8
#define NN_STRIDE     8            // node coordinates are stored like VAMP: 7 floats + 1 zero pad

enum { NN_BRUTE = 0, NN_MORTON = 1 };

// Shadow-mode grid: every (W, m) combination is evaluated on every exact query
#define SH_NW 6
#define SH_NM 4
#define SH_BINS 65                 // log2(ratio) in 1/16 steps over [0, 4), last bin = overflow
static const int sh_W[SH_NW] = {1, 2, 4, 8, 16, 32};
static const int sh_m[SH_NM] = {1, 2, 4, 8};

typedef struct {
    uint32_t queries[3][SH_NM][SH_NW];   // [QT_EXTEND (accepted) / QT_CONNECT / QT_EXTEND_REJECTED][m][W]
    uint32_t hits[3][SH_NM][SH_NW];      // returned distance == exact nearest distance
    uint32_t hist[3][SH_NM][SH_NW][SH_BINS];
} shadow_t;

// Double-ended array: element i is at base[dir * i] (the two trees of a run share one allocation)
typedef struct { uint64_t *k; uint32_t *o; int dir; } dearr_t;

typedef struct {
    const float *buf;      // global node buffer (NN_STRIDE floats per node)
    uint32_t *gid;         // local index -> global node index (double-ended)
    int gdir;
    uint32_t n;
    int mode, m, W, B, bits;
    uint32_t n_sorted;     // local [0, n_sorted) are in the sorted arrays, [n_sorted, n) is the buffer
    dearr_t s[NN_MAX_SHIFTS];
} nn_tree_t;

// m > 0 maintains m shifted Morton orders (also in NN_BRUTE mode, for the shadow analysis); B <= 64
typedef struct {
    int mode, m, W, B, bits;
} nn_params_t;

// cap = max nodes of both trees together; side 0 / 1 picks the end of the shared arrays
void nn_shared_alloc(const nn_params_t *p, uint32_t cap);
void nn_init(nn_tree_t *t, const nn_params_t *p, const float *buf, int side, uint32_t cap);
void nn_insert(nn_tree_t *t, uint32_t gid);
void nn_flush(nn_tree_t *t);
// Returns the global index of the nearest node and its L2 distance (VAMP's float sum order)
uint32_t nn_nearest(nn_tree_t *t, const float *q, float *dist);
uint32_t nn_nearest_exact(const nn_tree_t *t, const float *q, float *dist2);
// Shadow mode: evaluate every (W, m) of the grid against the exact answer
void nn_shadow(const nn_tree_t *t, const float *q, float exact_d2, int qtype, shadow_t *sh);

float nn_dist2(const float *a, const float *b);
