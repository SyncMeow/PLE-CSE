#!/usr/bin/env python3
"""Design-independent results (stdlib only).

  analyze_flat.py --flat 'expFlat/flat_s?.csv' --nn 'expFlat/nnscale_*.csv'
(3) voxel indexing on the flat robot-sphere stream: work per query, block-size sweep, a
    parametric cost model (cost of a cell lookup = 1, of a block-bitmap read = beta, of a
    non-empty voxel = gamma) and index storage per format.
(2) batch ordering: first colliding query per batch size and order; LRU miss rate for any cache
    size from the reuse-distance histograms (full batches and with early exit).
(1) nearest neighbour on synthetic trees: cheapest (m, W) reaching a hit-rate target vs brute force.
"""
import argparse
import glob
import math
from collections import defaultdict

SCENES = ['table_pick', 'table_under_pick', 'box', 'bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'cage']
ORD = ['pose-major', 'sphere-major', 'Morton']
BS = [2, 4, 8]
G = 28


def load_flat(paths):
    agg, per_scene, preps = defaultdict(lambda: None), {}, []
    for p in sorted(paths):
        tags = {}
        scene = None
        for l in open(p):
            v = l.strip().split(',')
            if v[0] == 'PREP':
                preps.append([int(x) for x in v[1:]])
                scene = int(v[1])
            elif v[0].startswith('FS'):
                tags[v[0]] = [int(x) for x in v[1:]]
        per_scene[scene] = tags
        for k, x in tags.items():
            agg[k] = x if agg[k] is None else [a + b for a, b in zip(agg[k], x)]
    return dict(agg), per_scene, preps


def q(h, p):
    tot = sum(h)
    acc = 0
    for i, c in enumerate(h):
        acc += c
        if acc >= p * tot:
            return i
    return len(h) - 1


def part3(a, per_scene, preps):
    queries, culled, hits, cells, ne, aabb, pts = a['FSQ']
    poses, colp, q2h = a['FSPOSE']
    nq = queries - culled
    print('\n## (3) Voxel indexing, flat robot-sphere stream (every checked pose x 59 spheres)')
    print(f'poses {poses:,}; queries {queries:,}; culled by the global AABB {culled / queries * 100:.1f}%; '
          f'hit {hits / queries * 100:.2f}%')
    print(f'per non-culled query: cells {cells / nq:.2f} (p50 {q(a["FSHC"], .5)}, p90 {q(a["FSHC"], .9)}, '
          f'max {max(i for i, c in enumerate(a["FSHC"]) if c)}), non-empty {ne / nq:.2f} '
          f'-> empty {(1 - ne / cells) * 100:.1f}% of the cells walked')
    print(f'downstream work per non-culled query: voxels passing the local AABB {aabb / nq:.2f}, '
          f'points tested {pts / nq:.2f}')
    print(f'pose level: {colp / poses * 100:.1f}% of poses hit the environment; robot spheres queried until the '
          f'first hit in a colliding pose: {q2h / max(1, colp):.1f} of 59')
    print('\nblock-size sweep (per non-culled query)')
    print(f'{"block":>7s} {"blocks":>7s} {"occupied":>9s} {"empty blocks":>13s} {"bitmap+rank B":>14s}')
    for i, k in enumerate(BS):
        b, o = a['FSBLK'][i], a['FSBLKOCC'][i]
        nb = math.ceil(G / k) ** 3
        print(f'{k}x{k}x{k:<3d} {b / nq:7.2f} {o / nq:9.2f} {(1 - o / b) * 100:12.1f}% '
              f'{nb * (k ** 3 // 8 + 2):14,d}')
    print('\nparametric model: speed-up of the voxel-lookup work = cells / (beta*blocks + gamma*non-empty)')
    print('(cost of one cell lookup in the current table walk = 1; ideal skip-empty bound = cells / non-empty '
          f'= {cells / ne:.2f})')
    for gamma in (1, 2):
        print(f'gamma = {gamma}:  ' + '   '.join(
            f'{k}^3: ' + ' '.join(f'b={beta}->{cells / (beta * a["FSBLK"][i] + gamma * ne):.2f}'
                                  for beta in (0.5, 1, 2, 4)) for i, k in enumerate(BS)))
    print('\nper scene (non-culled query): cells, empty %, 4^3 blocks, speed-up at beta=1, gamma=1')
    for s in sorted(per_scene):
        t = per_scene[s]
        Q, C, H, ce, n_, _, _ = t['FSQ']
        nqs = Q - C
        print(f'  {SCENES[s]:17s} {ce / nqs:6.2f} {(1 - n_ / ce) * 100:6.1f}% {t["FSBLK"][1] / nqs:6.2f} '
              f'{ce / (t["FSBLK"][1] + n_):6.2f}')
    print('\nindex storage per problem (bytes, voxel headers excluded; mean per scene)')
    print(f'{"scene":17s} {"voxels":>7s} {"dense 28^3":>11s} {"MVT 3-level":>12s} {"hash(lf .5)":>12s} '
          f'{"Morton 4^3":>11s}')
    by = defaultdict(list)
    for p in preps:
        by[p[0]].append(p)
    for s in sorted(by):
        g = by[s]
        nv = sum(p[4] for p in g) / len(g)
        mvt = sum(p[5] for p in g) * 4 / len(g)
        print(f'{SCENES[s]:17s} {nv:7.0f} {G ** 3 * 4:11,d} {mvt:12,.0f} {nv / 0.5 * 8:12,.0f} '
              f'{math.ceil(G / 4) ** 3 * (8 + 2):11,d}')


def part2(a):
    print('\n## (2) Batch ordering (flat stream, batches rebuilt from every edge the planner checked)')
    ob, oq = a['FSOB'], a['FSOQ']
    hb, qh = a['FSOHB'], a['FSOQH']
    print(f'{"P poses":>8s} {"batches":>9s} {"q/batch":>8s} {"hit batches":>12s}   queries to first hit: '
          + '  '.join(f'{o:>12s}' for o in ORD))
    for i, P in enumerate((8, 16, 32)):
        print(f'{P:8d} {ob[i]:9,d} {oq[i] / ob[i]:8.0f} {hb[i * 3] / ob[i] * 100:11.1f}%   '
              + '                      '[:0] + '  '.join(f'{qh[i * 3 + o] / max(1, hb[i * 3 + o]):12.1f}'
                                                       for o in range(3)))
    print('\nLRU miss rate of voxel references vs cache size (P = 32): full batches | with early exit')
    sizes = [1, 2, 4, 8, 16, 32, 64, 128, 256]
    print(f'{"order":14s} ' + ' '.join(f'{c:>6d}' for c in sizes) + ' | ' + ' '.join(f'{c:>6d}' for c in sizes))
    for o in range(3):
        row = []
        for key in (f'FSRDF{o}', f'FSRDE{o}'):
            h = a[key]
            tot = sum(h)
            cells = []
            for c in sizes:
                j = int(math.log2(c))
                miss = sum(h[j + 1:])   # bins b >= j+1 hold d >= 2^j, plus cold (last bin)
                cells.append(f'{miss / tot * 100:5.1f}%')
            row.append(' '.join(f'{x:>6s}' for x in cells))
        print(f'{ORD[o]:14s} ' + row[0] + ' | ' + row[1])
    for o in range(3):
        print(f'  references {ORD[o]}: full {sum(a[f"FSRDF{o}"]):,}  early-exit {sum(a[f"FSRDE{o}"]):,}  '
              f'(cold {a[f"FSRDF{o}"][-1]:,})')


def part1(paths, target):
    rows = defaultdict(lambda: [0, 0])
    for p in paths:
        for l in open(p):
            v = l.strip().split(',')
            if v[0] != 'NNSCALE':
                continue
            k = (v[1], int(v[2]), int(v[3]), int(v[4]))
            rows[k][0] += int(v[5])
            rows[k][1] += int(v[6])
    print(f'\n## (1) Nearest neighbour on synthetic trees: cheapest (m, W) with hit rate >= {target * 100:.0f}%')
    print('cost = distances computed per query (brute force: N; Morton: m * min(2W, N), binary search not counted)')
    for dist in ('uniform', 'clustered', 'walk'):
        Ns = sorted({k[1] for k in rows if k[0] == dist})
        print(f'\n{dist}')
        print(f'{"N":>8s} {"hit m1W1":>9s} {"best (m,W)":>11s} {"hit":>7s} {"cost":>7s} {"vs brute":>9s}')
        for N in Ns:
            best = None
            for (d, n, m, W), (qq, hh) in rows.items():
                if d != dist or n != N:
                    continue
                hit = hh / qq
                cost = m * min(2 * W, N)
                if hit >= target and (best is None or cost < best[0]):
                    best = (cost, m, W, hit)
            h11 = rows[(dist, N, 1, 1)]
            if best:
                print(f'{N:8d} {h11[1] / h11[0] * 100:8.1f}% {f"({best[1]},{best[2]})":>11s} '
                      f'{best[3] * 100:6.1f}% {best[0]:7d} {best[0] / N:8.3f}x')
            else:
                print(f'{N:8d} {h11[1] / h11[0] * 100:8.1f}% {"none":>11s}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--flat')
    ap.add_argument('--nn')
    ap.add_argument('--target', type=float, default=0.95)
    a = ap.parse_args()
    if a.flat:
        agg, per_scene, preps = load_flat(glob.glob(a.flat))
        part3(agg, per_scene, preps)
        part2(agg)
    if a.nn:
        part1(glob.glob(a.nn), a.target)


if __name__ == '__main__':
    main()
