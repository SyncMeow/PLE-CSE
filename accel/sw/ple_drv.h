// CPU-side driver for the PLE (Platform/accel/rtl/ple/ple_top.sv, register map in its header).
// The base address is a parameter: it is fixed at SoC integration (MMIO.bd), not here.
// Register access goes through ple_wr32 / ple_rd32: volatile MMIO on the SoC; with PLE_DRV_HOSTSIM the
// testbench harness provides them (Platform/accel/tb/tb_drv.sv runs this driver against the RTL).
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PLE_DIM 7

// register byte offsets
enum {
    PLE_CTRL = 0x000, PLE_STATUS = 0x004, PLE_IRQ = 0x008, PLE_MODE = 0x010,
    PLE_RANGE = 0x014, PLE_RADIUS = 0x018, PLE_ALPHA = 0x01C, PLE_MIN_RADIUS = 0x020, PLE_TREE_RATIO = 0x024,
    PLE_MAX_ITER = 0x028, PLE_W_MIN = 0x030, PLE_START = 0x040, PLE_GOAL = 0x050, PLE_SHIFT = 0x060,
    PLE_ITERATIONS = 0x080, PLE_SIZES = 0x084, PLE_PATH_LEN = 0x088,
    PLE_C_CYCLES = 0x0C0, PLE_C_EDGES = 0x0C4, PLE_C_POSES = 0x0C8, PLE_C_QUERIES = 0x0CC, PLE_C_KILLED = 0x0D0,
    PLE_C_SELF_PAIRS = 0x0D4, PLE_C_DD_REJECTS = 0x0D8, PLE_C_NN = 0x0DC, PLE_PATH = 0x400
};
enum { PLE_SOLVED = 0, PLE_FAIL_ITER = 1, PLE_FAIL_CAPACITY = 2, PLE_ENV_ERR = 3, PLE_ABORTED = 4 };
#define PLE_MODE_ENV_BUF  (1u << 0)
#define PLE_MODE_FLAT     (1u << 1)
#define PLE_MODE_DD       (1u << 2)
#define PLE_MODE_BALANCE  (1u << 3)

typedef struct { volatile uint32_t *base; } ple_dev_t;

typedef struct {
    uint32_t mode;              // PLE_MODE_* bits
    uint32_t range_h;           // 2^-24 rad, >= 1
    uint32_t radius;            // DD radius on the first failure, 2^-24 rad
    uint32_t alpha;             // Q0.32
    uint32_t min_radius;        // 2^-24 rad
    uint32_t tree_ratio;        // Q16.16
    uint32_t max_iterations;
} ple_cfg_t;

typedef struct {
    int status;                 // PLE_SOLVED ...
    uint32_t iterations, size_start, size_goal, path_len;
    uint32_t cycles, edges, poses, queries, killed, dd_rejects, nn_queries;
    uint32_t self_pairs;        // SCU pair tests run by the hardware (includes speculative poses)
} ple_result_t;

// VAMP defaults (same as golden fx_default_settings): range 1 rad, radius 4, alpha 1e-4, min 1, ratio 1, DD + balance
void ple_default_cfg(ple_cfg_t *c);
// Halton rotation for a seed (splitmix64, same as golden smp_reset); seed 0 = no rotation
void ple_seed_shift(uint32_t seed, uint32_t shift[PLE_DIM]);
// Joint angle (rad) to the PLE format: floor(x * 2^12)
int16_t ple_q_from_float(float x);

// all return 0 on success, -1 if the PLE is busy (configuration is ignored while busy)
int ple_configure(ple_dev_t *d, const ple_cfg_t *c);
int ple_set_problem(ple_dev_t *d, const int16_t start[PLE_DIM], const int16_t goal[PLE_DIM], const uint32_t shift[PLE_DIM]);
void ple_start(ple_dev_t *d, int irq_enable);   // clears a pending IRQ, then starts
void ple_abort(ple_dev_t *d);                 // taken at the next iteration boundary; result PLE_ABORTED
int ple_busy(ple_dev_t *d);
int ple_done(ple_dev_t *d);
// polls STATUS until done (max_polls = 0: no limit); returns the result code, -1 on timeout
int ple_wait(ple_dev_t *d, uint32_t max_polls);
void ple_irq_clear(ple_dev_t *d);
int ple_irq_pending(ple_dev_t *d);
void ple_read_result(ple_dev_t *d, ple_result_t *r);
// copies up to max nodes of the path (valid for PLE_SOLVED only); returns the path length
uint32_t ple_read_path(ple_dev_t *d, int16_t (*path)[PLE_DIM], uint32_t max);

#ifdef PLE_DRV_HOSTSIM
uint32_t ple_io_rd(uint32_t off);                 // provided by the testbench harness
void ple_io_wr(uint32_t off, uint32_t v);
#endif

#ifdef __cplusplus
}
#endif
