#!/usr/bin/env python3
"""Independent reader for the CSE trace (environment format v0 + query list).

Rebuilds the environment from the hex files, checks the table against the points, and recomputes
every query's expected result from the definition (exists p: |c - p|^2 <= rq^2).
Usage: check_cse_trace.py <trace_dir> <tag> <queries.txt>     e.g. cse_trace s0_p1 cse_trace/s0_p1_seed0_queries.txt
"""
import sys

G, QSHIFT = 20, 11


def hexfile(path, width=None):
    return [int(l, 16) for l in open(path) if l.strip()]


def main():
    d, tag, qpath = sys.argv[1], sys.argv[2], sys.argv[3]
    hdr = dict(l.split(None, 1) for l in open(f'{d}/{tag}_hdr.txt'))
    n_pts, n_cells = int(hdr['n_pts']), int(hdr['n_cells'])
    glo = list(map(int, hdr['glo'].split()))
    ghi = list(map(int, hdr['ghi'].split()))
    dirw = hexfile(f'{d}/{tag}_dir.hex')
    cells = hexfile(f'{d}/{tag}_cell.hex')
    pts = list(zip(*(hexfile(f'{d}/{tag}_pt{a}.hex') for a in 'xyz')))
    assert len(dirw) == G ** 3 and len(cells) == n_cells and len(pts) == n_pts
    f = lambda w, lo, n: (w >> lo) & ((1 << n) - 1)
    errors = 0
    # table vs points: every point is in exactly its cell's range; AABB tight; global AABB tight
    owner = [None] * n_pts
    for gi, w in enumerate(dirw):
        if not w >> 15:
            continue
        gx, gy, gz = gi % G, (gi // G) % G, gi // (G * G)
        c = cells[w & 0xfff]
        base, cnt = f(c, 0, 16), f(c, 16, 10)
        lo = [f(c, 26 + 11 * k, 11) for k in range(3)]
        hi = [f(c, 59 + 11 * k, 11) for k in range(3)]
        if c >> 92:
            errors += 1
        org = [gx << QSHIFT, gy << QSHIFT, gz << QSHIFT]
        mins, maxs = [1 << 20] * 3, [-1] * 3
        for i in range(base, base + cnt):
            if owner[i] is not None:
                errors += 1
            owner[i] = gi
            for k in range(3):
                if pts[i][k] >> QSHIFT != org[k] >> QSHIFT:
                    errors += 1
                off = pts[i][k] - org[k]
                mins[k], maxs[k] = min(mins[k], off), max(maxs[k], off)
        if mins != lo or maxs != hi:
            errors += 1
    errors += sum(o is None for o in owner)
    for k in range(3):
        if min(p[k] for p in pts) != glo[k] or max(p[k] for p in pts) != ghi[k]:
            errors += 1
    # queries vs definition
    nq = mism = hits = 0
    for line in open(qpath):
        if line.startswith('#'):
            continue
        tagq, user, koh, eb, cx, cy, cz, rq, exp = map(int, line.split())
        r2 = rq * rq
        h = int(any((cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2 <= r2 for x, y, z in pts))
        nq += 1
        hits += h
        mism += h != exp
    print(f'TRACECHECK,{tag},points={n_pts},cells={n_cells},table_errors={errors},queries={nq},hits={hits},mismatch={mism}')
    return 1 if errors or mism else 0


if __name__ == '__main__':
    sys.exit(main())
