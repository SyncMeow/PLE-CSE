// Fixed-point golden model of the PLE/CSE accelerator (records/20261107-ple-cse-design.md).
// Every rounding is floor; RTL must match these definitions bit for bit.
#pragma once
#include <stdint.h>

#define FX_DIM        7
#define FX_JFRAC      12                      // joint angle: s16, LSB = 2^-12 rad
#define FX_HFRAC      24                      // high-precision length / DD radius: LSB = 2^-24 rad
#define FX_RAD_INF    0xFFFFFFFFu             // DD radius "infinite"
#ifndef FX_NODE_CAP                           // nodes per tree; must equal the RTL ple_tree CAP
#define FX_NODE_CAP   1024
#endif
#define FX_PATH_CAP   64
#define FX_RES        32                      // VAMP Panda resolution
#define FX_RAKE       8

typedef int16_t fxq_t[FX_DIM];                // one configuration

enum { FX_SOLVED = 0, FX_FAIL_ITER = 1, FX_FAIL_CAPACITY = 2, FX_ENV_ERR = 3 };

typedef struct {
    uint32_t range_h;       // extension range, 2^-24 rad
    uint32_t radius;        // DD radius on first failure, 2^-24 rad
    uint32_t alpha;         // DD alpha, Q0.32
    uint32_t min_radius;    // 2^-24 rad
    uint32_t tree_ratio;    // Q16.16
    int dynamic_domain, balance;
    uint32_t max_iterations;
    uint32_t seed;          // 0 = plain Halton; >0 = Cranley-Patterson rotation
} fx_settings_t;

typedef struct {
    int status;
    uint32_t iterations, size_start, size_goal;
    uint32_t dd_rejects, edges, poses, nn_queries;
    uint32_t path_len;
    uint32_t path_hash;     // FNV-1a over the int16 path coordinates
    float cost;             // reporting only (float sqrt), not part of the hardware
    int16_t path[FX_PATH_CAP][FX_DIM];
} fx_result_t;

// Edge oracle: 1 = pose collision-free. Step 1 wraps the float checker; later the fixed-point one.
typedef int (*fx_pose_fn)(const int16_t *q);

extern uint32_t fx_cur_edge, fx_cur_pose;   // edge counter and pose index within the edge (trace tags)

void fx_default_settings(fx_settings_t *s);
void fx_solve(const int16_t *start, const int16_t *goal, const fx_settings_t *s, fx_pose_fn pose_ok,
              fx_result_t *r);

// ---- exact integer helpers shared by the model pieces
static inline int64_t fx_fdiv(int64_t a, int64_t b)          // floor(a / b), b > 0
{
    int64_t q = a / b;
    if ((a % b) != 0 && a < 0) --q;
    return q;
}
static inline uint64_t fx_isqrt(uint64_t v)                  // floor(sqrt(v))
{
    uint64_t r = 0, bit = 1ull << 62;
    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}
static inline int16_t fx_q_from_float(float x)               // floor(x * 2^12)
{
    float t = x * (float)(1 << FX_JFRAC);
    int32_t i = (int32_t)t;
    if ((float)i > t) --i;
    return (int16_t)i;
}
