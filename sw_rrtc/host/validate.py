#!/usr/bin/env python3
"""Validate the scalar VCC/RRTC port against VAMP and against internal references.

Run with a Python that has vamp installed:  validate.py <libvcc.so> [--quick]
Checks (each prints PASS/FAIL with the numbers it judged):
  V1 FK spheres vs vamp.panda.fk (tolerance: VAMP is built with -ffp-contract=fast, so not bit-exact)
  V2 self-collision: our fkcc with an empty environment vs vamp.panda.validate(q, Environment())
  V3 MVT (Algorithm 1) vs brute force over the same filtered cloud, sphere level and pose level,
     plus a positive control that the comparison detects a 5% radius error
  V4 our pose validity vs VAMP's CAPT on the same filtered cloud (reported, see the bounding-sphere note)
  V5 Halton seed 0 vs vamp.panda.halton()
  V6 raw cloud size per scene vs the paper; CenterVox size vs VAMP's SCDF filter (r = 0.02)
  V7 end-to-end RRTC (brute NN) vs VAMP rrtc on the same cloud: solved/iterations/tree sizes
"""
import ctypes as C
import sys

import numpy as np
import vamp

lib = C.CDLL(sys.argv[1])
QUICK = '--quick' in sys.argv
fp = C.POINTER(C.c_float)
lib.api_prepare.restype = C.c_int
lib.api_validate.argtypes = [fp, C.c_int]
lib.api_sphere.argtypes = [C.c_float, C.c_float, C.c_float, C.c_float, C.c_int]
lib.api_sphere_fk.argtypes = [fp, fp]
lib.api_points.argtypes = [fp]
lib.api_raw_points.argtypes = [C.c_int, fp]
lib.api_halton.argtypes = [C.c_uint32, C.c_int, fp]
lib.api_plan.argtypes = [C.c_int, C.c_uint32, C.c_int, C.c_int, C.c_int, C.c_int, C.POINTER(C.c_uint32)]
lib.api_problem_info.argtypes = [C.c_int] + [C.POINTER(C.c_int)] * 3 + [fp, fp]

SCENES = ['table_pick', 'table_under_pick', 'box', 'bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'cage']
PAPER_RAW = [120000, 120000, 70000, 70000, 150000, 210000, 80000]
LO = np.array(vamp.panda.lower_bounds(), dtype=np.float32)
HI = np.array(vamp.panda.upper_bounds(), dtype=np.float32)
rng = np.random.default_rng(1)
fails = []


def ptr(a):
    return a.ctypes.data_as(fp)


def verdict(name, ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {name}: {msg}')
    if not ok:
        fails.append(name)


def info(i):
    s, x, v = C.c_int(), C.c_int(), C.c_int()
    st, gl = np.zeros(7, np.float32), np.zeros(7, np.float32)
    lib.api_problem_info(i, C.byref(s), C.byref(x), C.byref(v), ptr(st), ptr(gl))
    return s.value, x.value, v.value, st, gl


def rand_q(n):
    return (LO + rng.random((n, 7), dtype=np.float32) * (HI - LO)).astype(np.float32)


def prepare(i):
    n = lib.api_prepare(i)
    pts = np.zeros((n, 3), np.float32)
    lib.api_points(ptr(pts))
    return pts


def our_valid(q, mode):
    q = np.ascontiguousarray(q, np.float32)
    return bool(lib.api_validate(ptr(q), mode))


# ---------------------------------------------------------------- V1
N = 2000 if QUICK else 20000
maxerr, maxr = 0.0, 0.0
for q in rand_q(N):
    out = np.zeros((59, 4), np.float32)
    lib.api_sphere_fk(ptr(q), ptr(out))
    ref = np.array([[s.x, s.y, s.z, s.r] for s in vamp.panda.fk(q)], np.float32)
    maxerr = max(maxerr, float(np.abs(out[:, :3] - ref[:, :3]).max()))
    maxr = max(maxr, float(np.abs(out[:, 3] - ref[:, 3]).max()))
verdict('V1 FK', maxerr < 1e-5 and maxr == 0.0, f'{N} configs, max |dxyz| = {maxerr:.3g} m, max |dr| = {maxr:.3g}')

# ---------------------------------------------------------------- V2
empty = vamp.Environment()
N = 5000 if QUICK else 50000
mism, ncoll = 0, 0
for q in rand_q(N):
    a, b = our_valid(q, 2), vamp.panda.validate(q, empty)
    mism += a != b
    ncoll += not b
verdict('V2 self-collision', mism == 0, f'{N} configs, {ncoll} self-colliding (VAMP), {mism} mismatches')

# ---------------------------------------------------------------- V3 / V4
probs = [i for i in range(lib.api_n_problems()) if info(i)[2]]
sel = probs[::50] if QUICK else probs[::10]
sph_mism = pose_mism = ctrl_mism = sph_n = pose_n = 0
capt_us_only = capt_vamp_only = capt_n = 0
for i in sel:
    pts = prepare(i)
    lo, hi = pts.min(0) - 0.2, pts.max(0) + 0.2
    # spheres concentrated near the cloud, radii over the robot range and the link bounding spheres
    c = pts[rng.integers(0, len(pts), 3000)] + rng.normal(0, 0.05, (3000, 3)).astype(np.float32)
    c = np.vstack([c, (lo + rng.random((1000, 3)) * (hi - lo)).astype(np.float32)])
    r = rng.uniform(0.012, 0.18, len(c)).astype(np.float32)
    for (x, y, z), rr in zip(c, r):
        a = lib.api_sphere(float(x), float(y), float(z), float(rr), 0)
        b = lib.api_sphere(float(x), float(y), float(z), float(rr), 1)
        bc = lib.api_sphere(float(x), float(y), float(z), float(rr) * 1.05, 1)
        sph_mism += a != b
        ctrl_mism += a != bc
        sph_n += 1
    env = vamp.Environment()
    env.add_pointcloud(pts.tolist(), *vamp.panda.min_max_radii(), vamp.POINT_RADIUS)
    s, x, v, st, gl = info(i)
    # poses: random, and along the start-goal line (near obstacles in these scenes)
    qs = np.vstack([rand_q(300), st + np.linspace(0, 1, 200)[:, None] * (gl - st)]).astype(np.float32)
    for q in qs:
        a, b = our_valid(q, 0), our_valid(q, 1)
        pose_mism += a != b
        pose_n += 1
        cv = vamp.panda.validate(q, env)
        capt_n += 1
        capt_us_only += (not a) and cv
        capt_vamp_only += a and (not cv)
verdict('V3 MVT vs brute (spheres)', sph_mism == 0, f'{sph_n} spheres over {len(sel)} problems, {sph_mism} mismatches')
verdict('V3 MVT vs brute (poses)', pose_mism == 0, f'{pose_n} poses, {pose_mism} mismatches')
verdict('V3 positive control', ctrl_mism > 0, f'brute with radius x1.05 differs on {ctrl_mism} spheres (must be > 0)')
print(f'INFO V4 vs VAMP CAPT: {capt_n} poses, we-collide-CAPT-free {capt_us_only}, '
      f'CAPT-collides-we-free {capt_vamp_only}')
verdict('V4 CAPT never stricter than us', capt_vamp_only == 0,
        'CAPT may miss hits on link bounding spheres > r_max (allowed); it must never report a hit we miss')

# ---------------------------------------------------------------- V5
h = vamp.panda.halton()
N = 20000
ours = np.zeros((N, 7), np.float32)
lib.api_halton(0, N, ptr(ours))
ref = np.array([h.next() for _ in range(N)], np.float32)
diff = int((ours.view(np.uint32) != ref.view(np.uint32)).any(1).sum())
maxd = float(np.abs(ours - ref).max())
verdict('V5 Halton', maxd < 1e-6, f'{N} samples, {diff} not bit-identical, max |d| = {maxd:.3g}')

# ---------------------------------------------------------------- V6
raw_ok = True
sizes = {k: [] for k in range(7)}
for i in probs[::25] if QUICK else probs[::5]:
    s = info(i)[0]
    raw = np.zeros((300000, 3), np.float32)
    n = lib.api_raw_points(i, ptr(raw))
    raw_ok &= n == PAPER_RAW[s]
    ours_n = len(prepare(i))
    scdf, _ = vamp.filter_pointcloud(raw[:n], 0.02, 1.19, [0, 0, 0], [-1.19] * 3, [1.19] * 3, True)
    sizes[s].append((ours_n, len(scdf)))
verdict('V6 raw cloud size = paper Sec. V-A', raw_ok, str(PAPER_RAW))
for s in range(7):
    if sizes[s]:
        a = np.array(sizes[s])
        print(f'INFO V6 {SCENES[s]:17s} CenterVox {a[:, 0].mean():8.0f}  SCDF(0.02) {a[:, 1].mean():8.0f}  '
              f'ratio {a[:, 0].mean() / a[:, 1].mean():.2f}')

# ---------------------------------------------------------------- V7
settings = vamp.RRTCSettings()
settings.range = vamp.ROBOT_RRT_RANGES['panda']
settings.max_iterations = vamp.DEFAULT_ITERATIONS
settings.max_samples = vamp.DEFAULT_ITERATIONS
same = n7 = solved_both = 0
it_ratio = []
for i in (probs[::20] if QUICK else probs):
    pts = prepare(i)
    s, x, v, st, gl = info(i)
    env = vamp.Environment()
    env.add_pointcloud(pts.tolist(), *vamp.panda.min_max_radii(), vamp.POINT_RADIUS)
    h = vamp.panda.halton()
    res = vamp.panda.rrtc(st, gl, env, settings, h)
    out = (C.c_uint32 * 7)()
    lib.api_plan(i, 0, 0, 0, 8, 32, out)
    n7 += 1
    key_v = (bool(res.solved), int(res.iterations), tuple(int(z) for z in res.size))
    key_o = (bool(out[0]), int(out[1]), (int(out[2]), int(out[3])))
    same += key_v == key_o
    if res.solved and out[0]:
        solved_both += 1
        it_ratio.append((out[1] + 1) / (res.iterations + 1))
it = np.array(it_ratio)
print(f'INFO V7 {n7} problems: identical (solved, iterations, tree sizes) on {same}; both solved {solved_both}; '
      f'iteration ratio ours/VAMP median {np.median(it):.3f}, p5 {np.percentile(it, 5):.3f}, p95 {np.percentile(it, 95):.3f}')
verdict('V7 end-to-end', same >= 0.8 * n7 and solved_both == n7,
        'expect most runs identical (FMA rounding in VAMP and CAPT bounding-sphere misses explain the rest)')

print('ALL PASS' if not fails else f'FAILED: {fails}')
sys.exit(1 if fails else 0)
