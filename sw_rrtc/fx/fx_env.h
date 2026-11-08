// Environment format v0 (records/20261107-ple-cse-design.md 5.1) and the CSE reference.
#pragma once
#include <stdint.h>

#define ENV_G          20                  // query cells per axis (12.5 cm)
#define ENV_QSHIFT     11                  // query cell = q >> 11
#define ENV_FSHIFT     9                   // fine cell  = q >> 9 (3.125 cm)
#define ENV_S          4                   // in-cell bits used for the centre distance
#define ENV_NCELL      (ENV_G * ENV_G * ENV_G)
#define ENV_MAX_SLOTS  4096                // 12-bit slot
#define ENV_MAX_PTS    65535

typedef struct {
    uint16_t base;                         // first point in PTS
    uint16_t count;                        // points in this query cell (10 bits; <= 64 for Panda)
    uint16_t lo[3], hi[3];                 // tight AABB, offset from the cell origin (11 bits)
} env_cell_t;

typedef struct {
    // HDR
    int valid, err;
    uint32_t n_pts, n_cells;
    uint16_t glo[3], ghi[3];               // global AABB (tight)
    // DIR: {occ[15], slot[11:0]}
    uint16_t dir[ENV_NCELL];
    // CELL
    env_cell_t cell[ENV_MAX_SLOTS];
    // PTS (SoA)
    uint16_t px[ENV_MAX_PTS], py[ENV_MAX_PTS], pz[ENV_MAX_PTS];
} fx_env_t;

typedef struct {
    uint64_t queries, hits, gculled, cells, nonempty, aabb_pass, points;
} fx_cse_stats_t;

extern fx_cse_stats_t fx_cs;

// Raw float points (metres, robot base frame) -> filtered, tabled environment; w_min in LSB
void fx_env_build(const float *x, const float *y, const float *z, uint32_t n, int32_t w_min, fx_env_t *e);
// CSE reference, structured like the hardware (global cull, range, dir, local AABB, points); 1 = hit
int fx_cse_ref(const fx_env_t *e, int32_t cx, int32_t cy, int32_t cz, uint32_t rq);
// Definition: exists p with |c - p|^2 <= rq^2 (brute force over all points)
int fx_cse_def(const fx_env_t *e, int32_t cx, int32_t cy, int32_t cz, uint32_t rq);

// Trace output for the CSE testbench: <dir>/<tag>_{hdr.txt,hdr.hex,dir.hex,cell.hex,ptx.hex,pty.hex,ptz.hex}
// hdr.hex: 9 words, 0 {err, valid}, 1 n_pts, 2 n_cells, 3-5 glo x/y/z, 6-8 ghi x/y/z
// cell.hex: 128-bit words, [15:0] base, [25:16] count, [36:26] lo_x, [47:37] lo_y, [58:48] lo_z,
//           [69:59] hi_x, [80:70] hi_y, [91:81] hi_z, upper bits 0
int fx_env_dump(const fx_env_t *e, const char *dir, const char *tag);
