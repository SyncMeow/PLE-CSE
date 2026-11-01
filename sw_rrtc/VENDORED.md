# Vendored / derived material

| File | Derived from | License |
|---|---|---|
| `gen/panda_fk.c` | VAMP `src/impl/vamp/robots/panda.hh` (`sphere_fk`, `fkcc`), translated statement by statement to scalar C by `tools/gen_panda_fk.py` | Apache-2.0 |
| `gen/problems_panda.c` | VAMP `resources/panda/problems.tar.bz2` (MotionBenchMaker Panda scenes), converted by `tools/gen_problems.py` with VAMP's own `problem_tar_to_pkl_json.py` helpers; the `valid` flag is VAMP's `test_problem` | Apache-2.0 (VAMP); MotionBenchMaker data |
| `src/panda.h` | joint scaling constants and sphere radii from the same `panda.hh` | Apache-2.0 |
| `src/rrtc.c`, `src/vmath.h` | ports of VAMP `planning/rrtc.hh`, `planning/validate.hh`, `random/halton.hh` and the AVX `sin`/`cos` (cephes, via gruntthepeon.free.fr/ssemath) | Apache-2.0 |
| `src/env.c` (`pc_sample`) | sampling scheme of VAMP `src/vamp/pointcloud.py` (geometrout); numpy's RNG replaced by PCG32 | Apache-2.0 |

VAMP: https://github.com/KavrakiLab/vamp, commit `27cb9b66ebd8d101ce88d330f3b028398afc1cfa`
(2026-09-28, package version 0.6.4). Apache License 2.0: https://www.apache.org/licenses/LICENSE-2.0

The VCC parts (`src/env.c`: CenterVox, adaptive voxelization, Multilevel Voxel Table, Algorithm 1)
are written from the paper (Chen & Yeh, "VCC: Efficient Voxel-Based Collision Checking Framework
for Real-Time Robotic Motion Planning", ICRA'26); no VCC source code was available.

## Pre-use check (2026-10-19)
- Cloned into an isolated scratch directory; `pip install` of VAMP only into a scratch venv.
- No text addressed to an AI agent in the repository (grep over all text files).
- Binary files are only STL meshes; build dependencies fetched by CPM are pinned to commits
  (nigh, pdqsort, SIMDxorshift) or tags (cddlib 0.94m, Catch2 3.7.1), CPM itself by SHA-256;
  Eigen 3.4.0 is a tarball without a hash.
- Only the files above are derived; nothing from VAMP is compiled into the SoC image except the
  translated FK/CC code and constants.

## Regenerating
`make -C Platform/sw_rrtc regen VAMP_DIR=<vamp checkout> VAMP_PY=<python with vamp installed>`
