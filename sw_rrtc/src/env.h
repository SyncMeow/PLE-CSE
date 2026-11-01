// VCC environment: point cloud sampling, CenterVox filtering, Multilevel Voxel Table, Algorithm 1.
#pragma once
#include <stdint.h>
#include "problems.h"

#define VCC_POINT_RADIUS   0.0025f   // VAMP POINT_RADIUS
#define VCC_WS_HALF        1.19f     // VAMP ROBOT_MAX_RADII["panda"], around the first joint (origin)
#define VCC_L_FILTER       0.031f    // paper Sec. V-A
#define VCC_SAMPLES        10000     // points per primitive, paper Sec. V-A
#define VCC_PAD            4         // point arrays padded to a 4-lane (NEON/CCU) width with +inf

typedef struct { float *x, *y, *z; uint32_t n; } points_t;

typedef struct {
    uint32_t off;        // first point in the point pool (SoA)
    uint32_t count;      // real points (capacity = count rounded up to VCC_PAD)
    float lo[3], hi[3];  // local AABB
} voxel_t;

// Three-level table: X[G] -> Y table offset, Y[G] -> Z table offset, Z[G] -> voxel index; -1 = empty
typedef struct {
    int G;
    float wmin, width, s;   // workspace corner (same on all axes), side length, scale G / width
    int32_t *pool;          // Table Pool (X table at offset 0)
    uint32_t pool_used, pool_cap;
    voxel_t *vox;
    uint32_t n_vox, vox_cap;
    float *px, *py, *pz;    // Point Coordinate Pool
    uint32_t n_pool;
    float glo[3], ghi[3];   // global AABB of the points
} mvt_t;

// Bump allocator over one arena (SoC: DDR after the image; host: malloc'ed)
void arena_init(void *base, uint32_t size);
void *arena_alloc(uint32_t bytes);
uint32_t arena_mark(void);
void arena_release(uint32_t mark);

// Raw cloud: VCC_SAMPLES surface points per primitive (VAMP's sampler, own PRNG)
void pc_sample(const problem_t *p, points_t *out);
// CenterVox: keep the point nearest each L_FILTER voxel centre, drop points outside the workspace
void centervox(const points_t *raw, points_t *out);
// MVT two-phase construction
void mvt_build(mvt_t *t, const points_t *pts, float r_max);
// Algorithm 1 for one sphere: 1 = collides with the cloud
int mvt_sphere_collides(const mvt_t *t, float cx, float cy, float cz, float r);
// Reference: same test against every point (validation only)
int brute_sphere_collides(const points_t *pts, float cx, float cy, float cz, float r);
