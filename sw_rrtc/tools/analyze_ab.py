#!/usr/bin/env python3
"""Experiments A and B from rrtc_host_es output (stdlib only).

  analyze_ab.py es_s0.csv ... es_s6.csv     (one file per scene, as run by the Makefile recipe)
A: voxel-lookup work per sphere query, MVT (one cell per cycle) vs 4x4x4 Morton block bitmap
   (8 parity banks + one cycle per non-empty voxel), robot spheres vs link bounding spheres.
B: voxel-header traffic inside a CHECK batch under three query orders, and early-exit delay.
"""
import sys

SCENES = ['table_pick', 'table_under_pick', 'box', 'bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'cage']
CLS = ['robot spheres (r<=r_max)', 'link bounding spheres']
ORD = ['as issued (pose-major)', 'sphere-major', 'Morton(centre voxel)']
F = 'queries gculled hits cells cells_exit xlook ylook zlook nonempty nonempty_exit points points_exit blocks blocks_occ bank_cycles points_tested'.split()


def parse(path):
    cls, ords, prep = {}, {}, []
    for l in open(path):
        v = l.strip().split(',')
        if v[0] == 'ESCLS':
            c = int(v[1])
            d = dict(zip(F, map(int, v[2:18])))
            i = v.index('HC')
            n = v.index('HN')
            b = v.index('HB')
            k = v.index('HK')
            d['h_cells'] = list(map(int, v[i + 1:n]))
            d['h_nonempty'] = list(map(int, v[n + 1:b]))
            d['h_blocks'] = list(map(int, v[b + 1:k]))
            d['h_bankcyc'] = list(map(int, v[k + 1:]))
            cls[c] = d
        elif v[0] == 'ESORD':
            o = int(v[1])
            d = dict(zip('batches batch_queries voxel_refs point_refs distinct distinct_points hit_batches q_to_hit'.split(),
                         map(int, v[2:10])))
            e = v.index('EE')
            rest = v[10:e]
            d['lru'] = {int(rest[j]): (int(rest[j + 1]), int(rest[j + 2])) for j in range(0, len(rest), 3)}
            ee = list(map(int, v[e + 1:]))
            d['q_ee'], d['refs_ee'] = ee[0], ee[1]
            d['lru_ee'] = {s: (ee[2 + 2 * k], ee[3 + 2 * k]) for k, s in enumerate(sorted(d['lru']))}
            ords[o] = d
        elif v[0] == 'PREP':
            prep.append(list(map(int, v[1:])))
    return cls, ords, prep


def add(a, b):
    if a is None:
        return {k: (list(x) if isinstance(x, list) else (dict(x) if isinstance(x, dict) else x)) for k, x in b.items()}
    for k, x in b.items():
        if isinstance(x, list):
            a[k] = [p + q for p, q in zip(a[k], x)]
        elif isinstance(x, dict):
            a[k] = {s: (a[k][s][0] + x[s][0], a[k][s][1] + x[s][1]) for s in x}
        else:
            a[k] += x
    return a


def hq(h, p):
    tot = sum(h)
    acc = 0
    for i, c in enumerate(h):
        acc += c
        if acc >= p * tot:
            return i
    return len(h) - 1


def report_a(name, cls):
    print(f'\n### {name}')
    print(f'{"class":26s} {"queries":>9s} {"culled":>7s} {"cells/q":>8s} {"p50":>4s} {"p90":>4s} {"max":>4s} '
          f'{"empty%":>7s} {"blocks/q":>8s} {"bank cyc/q":>10s} {">1 bank cyc":>11s} {"MVT cyc/q":>9s} '
          f'{"Morton cyc/q":>12s} {"ratio":>6s}')
    tot_mvt = tot_mor = tot_mvt_x = tot_mor_x = 0
    for c in (0, 1):
        d = cls[c]
        n = d['queries'] - d['gculled']
        if n == 0:
            continue
        mvt = d['cells']                         # CCU B stage: one cell per cycle, whole range
        mor = d['bank_cycles'] + d['nonempty']   # bitmap reads + one cycle per non-empty voxel
        mvt_x = d['cells_exit']                  # same, stopping at the first colliding point
        mor_x = d['bank_cycles'] + d['nonempty_exit']
        tot_mvt += mvt; tot_mor += mor; tot_mvt_x += mvt_x; tot_mor_x += mor_x
        multi = 1 - d['h_bankcyc'][1] / max(1, n)
        print(f'{CLS[c]:26s} {d["queries"]:9d} {d["gculled"] / d["queries"] * 100:6.1f}% {d["cells"] / n:8.2f} '
              f'{hq(d["h_cells"], .5):4d} {hq(d["h_cells"], .9):4d} {max(i for i, x in enumerate(d["h_cells"]) if x):4d} '
              f'{(1 - d["nonempty"] / d["cells"]) * 100:6.1f}% {d["blocks"] / n:8.2f} {d["bank_cycles"] / n:10.2f} '
              f'{multi * 100:10.1f}% {mvt / n:9.2f} {mor / n:12.2f} {mvt / mor:6.2f}')
    print(f'  B-stage cycles, all queries: MVT {tot_mvt:,}  Morton {tot_mor:,}  -> x{tot_mvt / tot_mor:.2f} fewer '
          f'(with early exit: x{tot_mvt_x / tot_mor_x:.2f})')
    q = sum(cls[c]['queries'] for c in (0, 1))
    print(f'  share of queries that are link bounding spheres: {cls[1]["queries"] / q * 100:.1f}%; '
          f'software MVT table reads per query: X {sum(cls[c]["xlook"] for c in (0, 1)) / q:.2f}, '
          f'Y {sum(cls[c]["ylook"] for c in (0, 1)) / q:.2f}, Z {sum(cls[c]["zlook"] for c in (0, 1)) / q:.2f}')


def report_b(name, ords):
    print(f'\n### {name}: {ords[0]["batches"]:,} CHECK batches, {ords[0]["batch_queries"] / ords[0]["batches"]:.1f} queries/batch')
    sizes = sorted(ords[0]['lru'])
    print(f'{"order":24s} ' + ' '.join(f'{"LRU" + str(s) + " miss":>12s}' for s in sizes) +
          f' {"ideal (distinct)":>16s} {"queries to 1st hit":>18s}')
    for o in (0, 1, 2):
        d = ords[o]
        cells = ' '.join(f'{d["lru"][s][0] / d["voxel_refs"] * 100:11.1f}%' for s in sizes)
        print(f'{ORD[o]:24s} {cells} {d["distinct"] / d["voxel_refs"] * 100:15.1f}% '
              f'{d["q_to_hit"] / max(1, d["hit_batches"]):18.1f}')
    base = ords[0]
    print('  header fetches vs as-issued: ' + ', '.join(
        f'{ORD[o]} LRU{s}: x{ords[o]["lru"][s][0] / base["lru"][s][0]:.3f}' for o in (1, 2) for s in (4, 16)))
    print(f'  point traffic (LRU16 misses, points): issued {base["lru"][16][1]:,}, '
          f'sphere-major {ords[1]["lru"][16][1]:,}, Morton {ords[2]["lru"][16][1]:,}')
    print(f'  hit batches: {base["hit_batches"]:,} of {base["batches"]:,} ({base["hit_batches"] / base["batches"] * 100:.1f}%)')
    print('  with STOP_ON_FIRST_HIT (work up to the first colliding query of the batch):')
    print(f'  {"order":24s} {"queries":>14s} {"voxel refs":>14s} ' + ' '.join(f'{"LRU" + str(s) + " fetch":>14s}' for s in sizes))
    for o in (0, 1, 2):
        d = ords[o]
        print(f'  {ORD[o]:24s} {d["q_ee"]:14,d} {d["refs_ee"]:14,d} ' + ' '.join(f'{d["lru_ee"][s][0]:14,d}' for s in sizes)
              + f'   (x{d["lru_ee"][16][0] / base["lru_ee"][16][0]:.2f} LRU16 fetches vs issued)')


def report_mem(prep):
    print('\n### MVT table storage vs a Morton occupancy bitmap (G = 28 -> 7^3 blocks, padded 8^3)')
    print(f'{"scene":17s} {"voxels":>7s} {"MVT tables B":>13s} {"bitmap+base B":>14s}')
    by = {}
    for p in prep:
        by.setdefault(p[0], []).append(p)
    for s in sorted(by):
        g = by[s]
        nv = sum(p[4] for p in g) / len(g)
        tb = sum(p[5] for p in g) * 4 / len(g)
        print(f'{SCENES[s]:17s} {nv:7.0f} {tb:13.0f} {7 ** 3 * (8 + 2):7d} / {8 ** 3 * (8 + 2):d}')


def main():
    tot_c, tot_o, prep = {0: None, 1: None}, {0: None, 1: None, 2: None}, []
    for path in sys.argv[1:]:
        cls, ords, pr = parse(path)
        s = pr[0][0]
        prep += pr
        report_a(SCENES[s], cls)
        for c in (0, 1):
            tot_c[c] = add(tot_c[c], cls[c])
        for o in (0, 1, 2):
            tot_o[o] = add(tot_o[o], ords[o])
    print('\n## Experiment A: voxel lookup, MVT vs Morton block bitmap')
    report_a('ALL scenes', tot_c)
    print('\n## Experiment B: query order inside a CHECK batch')
    report_b('ALL scenes', tot_o)
    report_mem(prep)


if __name__ == '__main__':
    main()
