#!/usr/bin/env python3
"""Summaries for the four questions (stdlib only).

  analyze.py --brute brute.csv [--shadow shadow.csv] [--morton 'morton_m*_W*.csv'] [--soc soc.log ...]
  1. NN share of time (SoC logs): pure software, and projected with the environment check S x faster
  2. Morton hit rate vs (W, m) (shadow.csv)
  3. iterations / CHECKs of closed-loop Morton vs brute force (paired by problem and seed)
  4. tree size distribution (brute.csv)
"""
import argparse
import glob
import math
import re
import statistics as stt
from collections import defaultdict

RUN_COLS = ('tag scene index seed nn m W B bits solved cap_hit iters size_s size_g nnq_e nnq_c dd_rej inserts '
            'edges checks poses envq env_hits env_gcull voxels vox_cull points self_pairs merges path_len '
            'path_hash cost_bits cyc_total cyc_nnq cyc_nni cyc_fkcc cyc_fk cyc_env cyc_rng timer_reads '
            'nn_evals').split()
PREP_COLS = 'tag scene index raw filt n_vox pool_used n_pool cyc_filter cyc_build'.split()
SCENES = ['table_pick', 'table_under_pick', 'box', 'bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'cage']
QT = ['EXTEND(accepted)', 'CONNECT', 'EXTEND(dd-rejected)']


def parse(path, tag, cols):
    return parse_text(open(path, errors='replace').read(), tag, cols)


def parse_text(text, tag, cols):
    rows = []
    for l in text.splitlines():
        if l.startswith(tag + ','):
            v = l.strip().split(',')
            d = {}
            for k, c in enumerate(cols):
                if k < len(v):
                    x = v[k]
                    d[c] = int(x, 16) if c in ('path_hash', 'cost_bits') else (int(x) if x.lstrip('-').isdigit() else x)
            rows.append(d)
    return rows


def q(xs, p):
    xs = sorted(xs)
    if not xs:
        return float('nan')
    k = (len(xs) - 1) * p
    lo = math.floor(k)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def gmean(xs):
    return math.exp(sum(math.log(x) for x in xs) / len(xs)) if xs else float('nan')


def item4(runs):
    print('\n## 4. Tree size at solution (start + goal tree nodes), brute-force NN')
    print(f'{"scene":17s} {"runs":>5s} {"solved":>6s} {"median":>7s} {"p90":>7s} {"p99":>8s} {"max":>8s} '
          f'{"iters med":>9s} {"iters p99":>9s} {"NN evals/query med":>18s}')
    groups = defaultdict(list)
    for r in runs:
        groups[r['scene']].append(r)
    groups['all'] = runs
    for s in list(range(7)) + ['all']:
        g = groups[s]
        n = [r['size_s'] + r['size_g'] for r in g]
        it = [r['iters'] for r in g]
        ev = [r['nn_evals'] / max(1, r['nnq_e'] + r['nnq_c']) for r in g if 'nn_evals' in r]
        name = SCENES[s] if s != 'all' else 'ALL'
        print(f'{name:17s} {len(g):5d} {sum(r["solved"] for r in g):6d} {q(n, .5):7.0f} {q(n, .9):7.0f} '
              f'{q(n, .99):8.0f} {max(n):8d} {q(it, .5):9.0f} {q(it, .99):9.0f} {q(ev, .5) if ev else float("nan"):18.1f}')
    n = sorted(r['size_s'] + r['size_g'] for r in runs)
    edges = [2, 10, 100, 1000, 10000, 100000, 10 ** 7]
    lo = 0
    print('histogram:', ', '.join(f'[{a},{b}): {sum(a <= x < b for x in n) / len(n) * 100:.1f}%'
                                 for a, b in zip([0] + edges[:-1], edges)))


def item2(path):
    agg = defaultdict(lambda: [0, 0, [0] * 65])
    for l in open(path):
        v = l.strip().split(',')
        qt, m, W, nq, nh = int(v[4]), int(v[5]), int(v[6]), int(v[7]), int(v[8])
        a = agg[(qt, m, W)]
        a[0] += nq
        a[1] += nh
        for b, c in enumerate(v[9:]):
            a[2][b] += int(c)
    Ws = sorted({k[2] for k in agg})
    ms = sorted({k[1] for k in agg})
    print('\n## 2. Morton hit rate (shadow mode: planner uses the exact NN, Morton scored on the side)')
    for qt in sorted({k[0] for k in agg}):
        tot = agg[(qt, ms[0], Ws[0])][0]
        print(f'\n{QT[qt]}  ({tot} queries)   hit rate %  /  distance ratio median, p95')
        print('m \\ W  ' + ''.join(f'{W:>18d}' for W in Ws))
        for m in ms:
            cells = []
            for W in Ws:
                nq, nh, h = agg[(qt, m, W)]
                med, p95 = hist_q(h, .5), hist_q(h, .95)
                cells.append(f'{100 * nh / max(1, nq):6.1f} {med:5.2f} {p95:5.2f}')
            print(f'{m:<6d} ' + ''.join(f'{c:>18s}' for c in cells))


def hist_q(h, p):
    tot = sum(h)
    if not tot:
        return float('nan')
    acc = 0
    for b, c in enumerate(h):
        acc += c
        if acc >= p * tot:
            return 1.0 if b == 0 else 2 ** (b / 16)   # upper edge of the bin
    return float('inf')


def item3(brute, files):
    base = {(r['scene'], r['index'], r['seed']): r for r in brute}
    print('\n## 3. Closed-loop Morton vs brute force (paired by problem and seed)')
    print(f'{"m":>2s} {"W":>3s} {"solved":>7s} {"iters x(gmean)":>14s} {"iters x(p90)":>12s} '
          f'{"CHECK x(total)":>14s} {"CHECK x(gmean)":>14s} {"CHECK x(p90)":>12s} {"NN evals x(total)":>17s}')
    for f in sorted(files, key=lambda p: tuple(int(x) for x in re.findall(r'_m(\d+)_W(\d+)', p)[0])):
        runs = parse(f, 'RUN', RUN_COLS)
        m, W = runs[0]['m'], runs[0]['W']
        it_r, ck_r = [], []
        ck_tot = ck_base = ev_tot = ev_base = 0
        solved = 0
        for r in runs:
            b = base.get((r['scene'], r['index'], r['seed']))
            if not b:
                continue
            solved += r['solved']
            it_r.append((r['iters'] + 1) / (b['iters'] + 1))
            ck_r.append((r['checks'] + 1) / (b['checks'] + 1))
            ck_tot += r['checks']
            ck_base += b['checks']
            if 'nn_evals' in r and 'nn_evals' in b:
                ev_tot += r['nn_evals']
                ev_base += b['nn_evals']
        print(f'{m:2d} {W:3d} {solved:4d}/{len(runs):<4d} {gmean(it_r):14.3f} {q(it_r, .9):12.3f} '
              f'{ck_tot / ck_base:14.3f} {gmean(ck_r):14.3f} {q(ck_r, .9):12.3f} '
              f'{(ev_tot / ev_base) if ev_base else float("nan"):17.3f}')


def item1(logs):
    print('\n## 1. NN share of time on the SoC')
    for path in logs:
        txt = open(path, errors='replace').read()
        if 'RRTC START' in txt:
            txt = txt[txt.rindex('RRTC START'):]   # only the last load
        m = re.search(r'RRTC START,timing=(\d+),cal=(\d+),nn=(\d+),m=(\d+),W=(\d+)', txt)
        cal = int(m.group(2)) if m else 0
        runs = parse_text(txt, 'RUN', RUN_COLS)
        preps = parse_text(txt, 'PREP', PREP_COLS)
        if not runs:
            print(f'{path}: no RUN lines')
            continue
        # the instrumentation's own cost: each tnow() costs about cal cycles, split over the categories
        tot = sum(r['cyc_total'] for r in runs)
        over = sum(r['timer_reads'] for r in runs) * cal
        nnq = sum(r['cyc_nnq'] for r in runs)
        nni = sum(r['cyc_nni'] for r in runs)
        fkcc = sum(r['cyc_fkcc'] for r in runs)
        fk = sum(r['cyc_fk'] for r in runs)
        env = sum(r['cyc_env'] for r in runs)
        rng = sum(r['cyc_rng'] for r in runs)
        self_ = fkcc - fk - env
        other = tot - nnq - nni - fkcc - rng
        prep = sum(p['cyc_filter'] + p['cyc_build'] for p in preps)
        print(f'{path}: {len(runs)} runs, {len(preps)} problems, nn={m.group(3)} m={m.group(4)} W={m.group(5)}, '
              f'timer overhead ~{over / tot * 100:.1f}% of planning cycles (cal={cal})')
        print(f'  planning cycles: NN query {nnq / tot * 100:.2f}%  NN insert {nni / tot * 100:.2f}%  '
              f'FK {fk / tot * 100:.1f}%  env (Alg. 1) {env / tot * 100:.1f}%  self-coll+rest of fkcc '
              f'{self_ / tot * 100:.1f}%  sampler {rng / tot * 100:.2f}%  other {other / tot * 100:.1f}%')
        print(f'  pre-planning (CenterVox + MVT) = {prep / (prep + tot) * 100:.1f}% of end-to-end '
              f'({prep / 75e6:.2f} s vs planning {tot / 75e6:.2f} s at 75 MHz)')
        nn = nnq + nni
        per = [(r['cyc_nnq'] + r['cyc_nni']) / max(1, r['cyc_total']) for r in runs]
        print(f'  per-run NN share: median {q(per, .5) * 100:.2f}%  p90 {q(per, .9) * 100:.2f}%  '
              f'max {max(per) * 100:.2f}%')
        print('  projected NN share if collision work runs S x faster on an accelerator (FK stays on the CPU):')
        print('    S       env only: planning / end-to-end      env + self-collision: planning / end-to-end')
        for S in (1, 2, 5, 10, 20, 50, 100, 1e9):
            pa = tot - env + env / S
            pb = tot - env - self_ + (env + self_) / S
            print(f'    {"inf" if S >= 1e9 else int(S):>4}   {nn / pa * 100:8.2f}% / {nn / (pa + prep) * 100:6.2f}%'
                  f'            {nn / pb * 100:8.2f}% / {nn / (pb + prep) * 100:6.2f}%')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--brute')
    ap.add_argument('--shadow')
    ap.add_argument('--morton')
    ap.add_argument('--soc', nargs='*', default=[])
    a = ap.parse_args()
    if a.soc:
        item1(a.soc)
    if a.shadow:
        item2(a.shadow)
    if a.brute:
        brute = parse(a.brute, 'RUN', RUN_COLS)
        if a.morton:
            item3(brute, glob.glob(a.morton))
        item4(brute)


if __name__ == '__main__':
    main()
