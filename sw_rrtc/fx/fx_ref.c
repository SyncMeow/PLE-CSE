// DPI-C entry points into the golden model for the PLE RTL testbenches (Platform/accel/tb).
// Includes fx_plan.c so the testbench calls the very same static functions; do not link fx_plan.c too.
#include "fx_plan.c"
#include "fx_pose.c"
#include "fx_fk_gen.c"      // defines macros F / TS / MUL: keep it the last include

#ifdef __cplusplus   // Verilator builds this file as C++; DPI expects C linkage
extern "C" {
#endif

// ---- SMP
void fx_ref_smp_reset(int seed) { smp_reset((uint32_t)seed); }
int fx_ref_smp_shift(int j) { return (int)h_shift[j]; }         // rotation the CPU writes to the PLE
void fx_ref_smp_next(short *q)                                   // q[7]
{
    int16_t t[FX_DIM];
    smp_next(t);
    for (int j = 0; j < FX_DIM; ++j) q[j] = t[j];
}

// Halton step when digit k is the lowest free digit: inv_k - (b-1) * sum_{i<k} inv_i (mod 2^32)
unsigned fx_ref_smp_delta(int j, int k)
{
    uint32_t d = h_inv[j][k];
    for (int i = 0; i < k; ++i) d -= (h_base[j] - 1) * h_inv[j][i];
    return d;
}

// ---- TREE + NNU: the testbench mirrors every insert, then asks for the nearest node
void fx_ref_tree_clear(int tree) { trees[tree].n = 0; }
int fx_ref_tree_insert(int tree, const short *q, int parent)      // returns node index or -1
{
    int16_t t[FX_DIM];
    for (int j = 0; j < FX_DIM; ++j) t[j] = q[j];
    return tree_insert(&trees[tree], t, (uint32_t)parent);
}
int fx_ref_nnu(int tree, const short *q, long long *d2)          // returns nearest index
{
    int16_t t[FX_DIM];
    int64_t d;
    for (int j = 0; j < FX_DIM; ++j) t[j] = q[j];
    uint32_t i = nnu(&trees[tree], t, &d);
    *d2 = d;
    return (int)i;
}

// ---- FKU: sphere centres in workspace coordinates (golden fx_fk_ws), c[70 * 3] = sphere-major x, y, z
void fx_ref_fk_ws(const short *q, int *c)
{
    static int init;
    int16_t t[FX_DIM];
    int32_t cc[FX_N_SPHERES][3];
    if (!init) { fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0); init = 1; }   // same eps as rrtc_fx
    for (int j = 0; j < FX_DIM; ++j) t[j] = q[j];
    fx_fk_ws(t, cc);
    for (int s = 0; s < FX_N_SPHERES; ++s)
        for (int k = 0; k < 3; ++k) c[3 * s + k] = cc[s][k];
}

// ---- SCU: golden self-collision check on c[70 * 3] (sphere-major x, y, z); 1 = no self collision
int fx_ref_self_ok(const int *c)
{
    int32_t cc[FX_N_SPHERES][3];
    short q0[FX_DIM] = {0};
    int dummy[3 * FX_N_SPHERES];
    fx_ref_fk_ws(q0, dummy);                                     // makes sure fx_pose_init ran
    for (int s = 0; s < FX_N_SPHERES; ++s)
        for (int k = 0; k < 3; ++k) cc[s][k] = c[3 * s + k];
    return fx_self_ok(cc);
}

// inner pair i (0..689): its spheres, its outer pair's spheres; returns the inner threshold
long long fx_ref_self_info(int i, int *a, int *b, int *oa, int *ob, long long *othr)
{
    int o = 0;
    while (i >= fx_self_off[o + 1]) ++o;
    *a = fx_self_inner[i][0];
    *b = fx_self_inner[i][1];
    *oa = fx_self_outer[o][0];
    *ob = fx_self_outer[o][1];
    *othr = (long long)fx_self_thr(o);
    return (long long)fx_self_thr(FX_N_SELF_OUTER + i);
}

// ---- ALU: the golden exact-integer helpers
unsigned long long fx_ref_isqrt(unsigned long long v) { return fx_isqrt(v); }
long long fx_ref_fdiv(long long a, long long b) { return fx_fdiv(a, b); }

// ---- RKG: run the golden edge_ok with a recording pose callback (every pose of the edge, VAMP order)
#define REF_MAX_POSES 8192
static int16_t rec[REF_MAX_POSES][FX_DIM];
static int rec_n;
static int rec_pose(const int16_t *q)
{
    if (rec_n < REF_MAX_POSES) for (int j = 0; j < FX_DIM; ++j) rec[rec_n][j] = q[j];
    rec_n++;
    return 1;
}
int fx_ref_rake(const short *start, const int *vec, long long len_h)   // returns the pose count
{
    static fx_result_t dummy;
    int16_t s[FX_DIM];
    int32_t v[FX_DIM];
    for (int j = 0; j < FX_DIM; ++j) { s[j] = start[j]; v[j] = vec[j]; }
    g_r = &dummy;
    g_pose_ok = rec_pose;
    rec_n = 0;
    edge_ok(s, v, (uint64_t)len_h);
    return rec_n;
}
void fx_ref_rake_pose(int i, short *q)                                  // i-th recorded pose
{
    for (int j = 0; j < FX_DIM; ++j) q[j] = (i < REF_MAX_POSES) ? rec[i][j] : 0;
}

#ifdef __cplusplus
}
#endif
