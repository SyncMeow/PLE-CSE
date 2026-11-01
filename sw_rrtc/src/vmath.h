// Scalar float math, bit-identical on the host and on the SoC (Zfinx) and to VAMP's AVX code.
// Build with -ffp-contract=off: a fused multiply-add would round differently.
#pragma once
#include <stdint.h>
#include <string.h>

static inline uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static inline float u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

// float -> int32, round to nearest even (_mm256_cvtps_epi32 under the default MXCSR)
static inline int32_t cvt_rne(float f)
{
#ifdef __riscv
    int32_t i;
    __asm__("fcvt.w.s %0, %1, rne" : "=r"(i) : "r"(f));
    return i;
#else
    return (int32_t)__builtin_lrintf(f);
#endif
}

// floor/ceil are exact operations; any correct implementation gives the same bits
static inline float vfloorf(float f)
{
    if (!(f > -8388608.0f && f < 8388608.0f)) return f;  // |f| >= 2^23 is already integral
    float t = (float)(int32_t)f;
    return (t > f) ? t - 1.0f : t;
}
static inline float vceilf(float f)
{
    if (!(f > -8388608.0f && f < 8388608.0f)) return f;
    float t = (float)(int32_t)f;
    return (t < f) ? t + 1.0f : t;
}

static inline float vsqrtf(float f) { return __builtin_sqrtf(f); }

// VAMP's AVX sin (cephes, from gruntthepeon.free.fr/ssemath), one lane
static inline float vamp_sinf(float x)
{
    uint32_t sign = f2u(x) & 0x80000000u;
    x = u2f(f2u(x) & 0x7fffffffu);
    float y = x * 1.27323954473516f;
    int32_t j = cvt_rne(y);
    j = (j + 1) & ~1;
    y = (float)j;
    uint32_t swap = (uint32_t)(j & 4) << 29;
    int use_sin_poly = (j & 2) == 0;
    sign ^= swap;
    float xm1 = y * -0.78515625f, xm2 = y * -2.4187564849853515625e-4f, xm3 = y * -3.77489497744594108e-8f;
    x = x + xm1;
    x = x + xm2;
    x = x + xm3;
    float z = x * x;
    float c = 2.443315711809948E-005f;
    c = c * z;
    c = c + -1.388731625493765E-003f;
    c = c * z;
    c = c + 4.166664568298827E-002f;
    c = c * z;
    c = c * z;
    float tmp = z * 0.5f;
    c = c - tmp;
    c = c + 1.0f;
    float s = -1.9515295891E-4f;
    s = s * z;
    s = s + 8.3321608736E-3f;
    s = s * z;
    s = s + -1.6666654611E-1f;
    s = s * z;
    s = s * x;
    s = s + x;
    // blend by masks: (~mask & c) + (mask & s), the zeroed lane is +0
    float r = use_sin_poly ? (0.0f + s) : (c + 0.0f);
    return u2f(f2u(r) ^ sign);
}

// VAMP: cos(x) = sin(x + pi/2), wrapped once into [-pi, pi)
static inline float vamp_cosf(float x)
{
    const float pi = 3.14159265359f;
    float v = x + pi * 0.5f;
    float w = v - ((v >= pi) ? 2.0f * pi : 0.0f);
    return vamp_sinf(w);
}
