// CPU-side PLE driver (see ple_drv.h).
#include "ple_drv.h"

#ifdef PLE_DRV_HOSTSIM
static inline uint32_t rd(ple_dev_t *d, uint32_t off) { (void)d; return ple_io_rd(off); }
static inline void wr(ple_dev_t *d, uint32_t off, uint32_t v) { (void)d; ple_io_wr(off, v); }
#else
static inline uint32_t rd(ple_dev_t *d, uint32_t off) { return d->base[off >> 2]; }
static inline void wr(ple_dev_t *d, uint32_t off, uint32_t v) { d->base[off >> 2] = v; }
#endif

void ple_default_cfg(ple_cfg_t *c)
{
    c->mode = PLE_MODE_DD | PLE_MODE_BALANCE;
    c->range_h = 1u << 24;
    c->radius = 4u << 24;
    c->alpha = 429496u;                           // (uint32_t)(1e-4 * 2^32)
    c->min_radius = 1u << 24;
    c->tree_ratio = 1u << 16;
    c->max_iterations = 1000000u;
}

void ple_seed_shift(uint32_t seed, uint32_t shift[PLE_DIM])
{
    uint64_t x = 0x9E3779B97F4A7C15ULL * (seed + 1);
    for (int j = 0; j < PLE_DIM; ++j) {
        shift[j] = 0;
        if (!seed) continue;
        uint64_t z = (x += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        z ^= z >> 31;
        shift[j] = (uint32_t)(z >> 40) << 8;
    }
}

int16_t ple_q_from_float(float x)
{
    float t = x * 4096.0f;
    int32_t i = (int32_t)t;
    if ((float)i > t) --i;
    return (int16_t)i;
}

int ple_busy(ple_dev_t *d) { return (int)(rd(d, PLE_STATUS) & 1u); }
int ple_done(ple_dev_t *d) { return (int)((rd(d, PLE_STATUS) >> 1) & 1u); }

int ple_configure(ple_dev_t *d, const ple_cfg_t *c)
{
    if (ple_busy(d) || c->range_h == 0) return -1;
    wr(d, PLE_MODE, c->mode);
    wr(d, PLE_RANGE, c->range_h);
    wr(d, PLE_RADIUS, c->radius);
    wr(d, PLE_ALPHA, c->alpha);
    wr(d, PLE_MIN_RADIUS, c->min_radius);
    wr(d, PLE_TREE_RATIO, c->tree_ratio);
    wr(d, PLE_MAX_ITER, c->max_iterations);
    return 0;
}

static uint32_t pack(int16_t a, int16_t b) { return (uint32_t)(uint16_t)a | ((uint32_t)(uint16_t)b << 16); }

int ple_set_problem(ple_dev_t *d, const int16_t start[PLE_DIM], const int16_t goal[PLE_DIM], const uint32_t shift[PLE_DIM])
{
    if (ple_busy(d)) return -1;
    for (int k = 0; k < 4; ++k) {
        wr(d, PLE_START + 4u * k, k < 3 ? pack(start[2 * k], start[2 * k + 1]) : pack(start[6], 0));
        wr(d, PLE_GOAL + 4u * k, k < 3 ? pack(goal[2 * k], goal[2 * k + 1]) : pack(goal[6], 0));
    }
    for (int j = 0; j < PLE_DIM; ++j) wr(d, PLE_SHIFT + 4u * j, shift[j]);
    return 0;
}

// a pending IRQ survives a start (ple_top rule), so a stale one from an earlier run is cleared first
void ple_start(ple_dev_t *d, int irq_enable)
{
    wr(d, PLE_IRQ, 1u);
    wr(d, PLE_CTRL, 1u | (irq_enable ? 4u : 0u));
}
void ple_abort(ple_dev_t *d) { wr(d, PLE_CTRL, 2u | (rd(d, PLE_CTRL) & 4u)); }
void ple_irq_clear(ple_dev_t *d) { wr(d, PLE_IRQ, 1u); }
int ple_irq_pending(ple_dev_t *d) { return (int)(rd(d, PLE_IRQ) & 1u); }

int ple_wait(ple_dev_t *d, uint32_t max_polls)
{
    for (uint32_t n = 0; max_polls == 0 || n < max_polls; ++n) {
        uint32_t s = rd(d, PLE_STATUS);
        if (s & 2u) return (int)((s >> 4) & 0xfu);
    }
    return -1;
}

void ple_read_result(ple_dev_t *d, ple_result_t *r)
{
    uint32_t s = rd(d, PLE_STATUS), z = rd(d, PLE_SIZES);
    r->status = (int)((s >> 4) & 0xfu);
    r->iterations = rd(d, PLE_ITERATIONS);
    r->size_start = z & 0xffffu;
    r->size_goal = z >> 16;
    r->path_len = rd(d, PLE_PATH_LEN);
    r->cycles = rd(d, PLE_C_CYCLES);
    r->edges = rd(d, PLE_C_EDGES);
    r->poses = rd(d, PLE_C_POSES);
    r->queries = rd(d, PLE_C_QUERIES);
    r->killed = rd(d, PLE_C_KILLED);
    r->dd_rejects = rd(d, PLE_C_DD_REJECTS);
    r->nn_queries = rd(d, PLE_C_NN);
    r->self_pairs = rd(d, PLE_C_SELF_PAIRS);
}

uint32_t ple_read_path(ple_dev_t *d, int16_t (*path)[PLE_DIM], uint32_t max)
{
    uint32_t n = rd(d, PLE_PATH_LEN);
    for (uint32_t i = 0; i < n && i < max; ++i)
        for (int k = 0; k < 4; ++k) {
            uint32_t w = rd(d, PLE_PATH + 16u * i + 4u * k);
            path[i][2 * k] = (int16_t)(w & 0xffffu);
            if (k < 3) path[i][2 * k + 1] = (int16_t)(w >> 16);
        }
    return n;
}
