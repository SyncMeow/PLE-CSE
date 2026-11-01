// Dynamic-domain balanced RRT-Connect, ported from VAMP planning/rrtc.hh (Apache-2.0).
#pragma once
#include <stdint.h>
#include "nn.h"

typedef struct {
    float range;          // VAMP ROBOT_RRT_RANGES["panda"] = 1.0
    int dynamic_domain;
    float radius, alpha, min_radius;
    int balance;
    float tree_ratio;
    uint32_t max_iterations, max_samples;
    nn_params_t nn;
    int shadow;           // host: also run the exact search on every query and score the Morton grid
    uint32_t seed;        // 0 = VAMP's Halton sequence; >0 = Cranley-Patterson rotation of it
} rrtc_settings_t;

typedef struct {
    int solved;
    uint32_t size_start, size_goal;
    uint32_t path_len;
    float cost;
    uint32_t path_hash;   // FNV-1a over the path's float bits (host/SoC bit-identity check)
    int cap_hit;          // ran out of max_samples
} rrtc_result_t;

void rrtc_default_settings(rrtc_settings_t *s);
// Allocates its buffers from the arena; caller releases the arena afterwards
void rrtc_solve(const float *start, const float *goal, const rrtc_settings_t *s, rrtc_result_t *r,
                shadow_t *sh);

// Halton sampler (VAMP rng/halton.hh)
void halton_reset(uint32_t seed);
void halton_next(float *q);

// validate_vector with VAMP's rake = 8 pose layout; 1 = whole segment collision-free
int validate_vector(const float *start, const float *vec, float distance);
