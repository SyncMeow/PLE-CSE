#!/usr/bin/env python3
"""Compare golden-model runs (FXRUN lines) with the float baseline (RUN lines of rrtc_host).

Usage: cmp_fx.py <float.csv> <fx.csv> [<fx.csv> ...]
"""
import math
import sys


def load_float(path):
    d = {}
    for line in open(path):
        if line.startswith('RUN,'):
            f = line.split(',')
            d[(f[1], f[2], f[3])] = dict(ok=f[9] == '1', it=int(f[11]), size=max(int(f[12]), int(f[13])),
                                         edges=int(f[18]), poses=int(f[20]), envq=int(f[21]), selfp=int(f[27]),
                                         pl=int(f[29]))
    return d


def load_fx(path):
    d, cfg = {}, ''
    for line in open(path):
        if line.startswith('FXCFG'):
            cfg = line.strip()
        if line.startswith('FXRUN,'):
            f = line.strip().split(',')
            r = dict(ok=f[4] == '0', status=int(f[4]), it=int(f[5]), size=max(int(f[6]), int(f[7])),
                     edges=int(f[9]), poses=int(f[10]), pl=int(f[12]))
            if len(f) > 16:
                r['envq'], r['selfp'] = int(f[15]), int(f[16])
            d[(f[1], f[2], f[3])] = r
    return d, cfg


def pct(v, ps=(50, 90, 99, 99.9)):
    v = sorted(v)
    return ' '.join(f'p{p}={v[min(len(v) - 1, int(len(v) * p / 100))]}' for p in ps)


def summary(name, d, keys):
    it = [d[k]['it'] for k in keys]
    s = f'{name:>10}: solved {sum(d[k]["ok"] for k in keys):5d}/{len(keys)}  iters {pct(it)}  max_tree {max(d[k]["size"] for k in keys)}'
    s += f'  poses/edge {sum(d[k]["poses"] for k in keys) / max(1, sum(d[k]["edges"] for k in keys)):.2f}'
    if all('envq' in d[k] for k in keys):
        P = max(1, sum(d[k]['poses'] for k in keys))
        s += f'  envq/pose {sum(d[k]["envq"] for k in keys) / P:.2f}  selfpairs/pose {sum(d[k]["selfp"] for k in keys) / P:.1f}'
    print(s)


def main():
    fl = load_float(sys.argv[1])
    for path in sys.argv[2:]:
        fx, cfg = load_fx(path)
        keys = sorted(set(fl) & set(fx))
        print(f'== {path}  {cfg}  runs={len(keys)}')
        summary('float', fl, keys)
        summary('fx', fx, keys)
        same = sum(fx[k]['it'] == fl[k]['it'] for k in keys)
        both = sum(fx[k]['ok'] and fl[k]['ok'] for k in keys)
        print(f'  identical iteration count: {same}/{len(keys)}   solved by both: {both}')
        prob = {}
        for k in keys:
            prob.setdefault(k[:2], []).append((fx[k]['it'] + 1, fl[k]['it'] + 1))
        r = sorted(math.log(sum(a for a, _ in v) / sum(b for _, b in v)) for v in prob.values())
        q = lambda x: math.exp(r[min(len(r) - 1, int(len(r) * x))])
        print(f'  per-problem iteration ratio fx/float: p10 {q(0.1):.2f}  p50 {q(0.5):.2f}  p90 {q(0.9):.2f}  max {math.exp(r[-1]):.2f}')
        bad = [k for k in keys if not fx[k]['ok']]
        print(f'  fx unsolved ({len(bad)}): ' + ' '.join(f'{k[0]}#{k[1]}s{k[2]}(st{fx[k]["status"]},float_it={fl[k]["it"]})' for k in bad[:12]))


if __name__ == '__main__':
    main()
