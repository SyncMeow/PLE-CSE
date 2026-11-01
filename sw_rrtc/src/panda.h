// Panda constants copied from VAMP robots/panda.hh (Apache-2.0, see ../VENDORED.md).
#pragma once

#define PANDA_DIM         7
#define PANDA_N_SPHERES   59
#define PANDA_MIN_RADIUS  0.012000000104308128f
#define PANDA_MAX_RADIUS  0.07999999821186066f
#define PANDA_RESOLUTION  32

// q = u * s_m + s_a maps the unit cube onto the joint limits; d_m = 1 / s_m
static const float panda_s_m[PANDA_DIM] = {
    5.934199810028076f, 3.6651999950408936f, 5.934199810028076f, 3.2288999557495117f,
    5.934199810028076f, 3.909600019454956f, 5.934199810028076f};
static const float panda_s_a[PANDA_DIM] = {
    -2.967099905014038f, -1.8325999975204468f, -2.967099905014038f, -3.1415998935699463f,
    -2.967099905014038f, -0.08730000257492065f, -2.967099905014038f};
static const float panda_d_m[PANDA_DIM] = {
    0.16851471364498138f, 0.2728364169597626f, 0.16851471364498138f, 0.30970299243927f,
    0.16851471364498138f, 0.2557806372642517f, 0.16851471364498138f};

// Generated (gen/panda_fk.c): sphere centres/radii, and fused FK + collision check (1 = valid)
void panda_sphere_fk(const float *q, float out[PANDA_N_SPHERES][4]);
int panda_fkcc(const float *q);
