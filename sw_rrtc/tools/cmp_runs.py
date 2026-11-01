#!/usr/bin/env python3
"""Check that SoC runs (sim or board log) are bit-identical to the host build.

Every PREP/RUN line of the SoC log is recomputed by the host binary with the same NN settings,
and all non-cycle columns must match exactly (counters, tree sizes, path hash, cost bits).
Usage: cmp_runs.py --soc <log> --host-bin <rrtc_host> [--expect-runs N]
"""
import argparse
import re
import subprocess
import sys

RUN_CYCLE_COLS = range(32, 40)    # cyc_total .. timer_reads
PREP_CYCLE_COLS = range(8, 10)


def lines(text, tag):
    return [l.strip().split(',') for l in text.splitlines() if l.startswith(tag + ',')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--soc', required=True)
    ap.add_argument('--host-bin', required=True)
    ap.add_argument('--expect-runs', type=int, default=0)
    a = ap.parse_args()
    log = open(a.soc, errors='replace').read()
    if 'RRTC START' in log:
        log = log[log.rindex('RRTC START'):]   # drop output of an earlier run before the last load
    m = re.search(r'RRTC START,timing=(\d+),cal=(\d+),nn=(\d+),m=(\d+),W=(\d+),B=(\d+),bits=(\d+),'
                  r'max_iter=(\d+),max_samples=(\d+)', log)
    if not m:
        sys.exit('FAIL: no RRTC START line in the SoC log')
    nn, mm, W, B, bits, max_iter, max_samples = (int(m.group(k)) for k in (3, 4, 5, 6, 7, 8, 9))
    done = 'RRTC DONE' in log
    soc_prep, soc_run = lines(log, 'PREP'), lines(log, 'RUN')
    fatal = 'FATAL,' in log
    if fatal:
        print('SoC log has FATAL: ' + log[log.index('FATAL,'):].splitlines()[0])
    bad = 0
    for p in soc_prep:
        scene, index = int(p[1]), int(p[2])
        g = scene * 100 + index - 1
        runs = [r for r in soc_run if r[1] == p[1] and r[2] == p[2]]
        seeds = sorted(int(r[3]) for r in runs)
        args = [a.host_bin, '-p', f'{g}:{g}', '-N', 'morton' if nn else 'brute', '-W', str(W), '-B', str(B),
                '-b', str(bits), '-i', str(max_iter), '-c', str(max_samples)]
        if nn:
            args += ['-m', str(mm)]
        if seeds:
            args += ['-s', str(seeds[0]), '-n', str(seeds[-1] - seeds[0] + 1)]
        else:
            args += ['-P']
        out = subprocess.run(args, capture_output=True, text=True, check=True).stdout
        hp, hr = lines(out, 'PREP'), lines(out, 'RUN')
        ref = [c for k, c in enumerate(hp[0]) if k not in PREP_CYCLE_COLS]
        got = [c for k, c in enumerate(p) if k not in PREP_CYCLE_COLS]
        if ref != got:
            bad += 1
            print(f'MISMATCH PREP scene {scene} #{index}\n  soc  {got}\n  host {ref}')
        for r in runs:
            h = next(x for x in hr if x[3] == r[3])
            ref = [c for k, c in enumerate(h) if k not in RUN_CYCLE_COLS]
            got = [c for k, c in enumerate(r) if k not in RUN_CYCLE_COLS]
            if ref != got:
                bad += 1
                diff = [k for k in range(len(ref)) if ref[k] != got[k]]
                print(f'MISMATCH RUN scene {scene} #{index} seed {r[3]} columns {diff}\n  soc  {got}\n  host {ref}')
    n = len(soc_run)
    ok = bad == 0 and done and not fatal and n > 0 and (a.expect_runs == 0 or n == a.expect_runs)
    print(f'{"PASS" if ok else "FAIL"}: {len(soc_prep)} PREP + {n} RUN lines compared, {bad} mismatches, '
          f'RRTC DONE {"seen" if done else "MISSING"}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
