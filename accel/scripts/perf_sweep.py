#!/usr/bin/env python3
# PLE performance sweeps (tb_core +perf=1 with the deterministic CSE timing model of tb/cse_bfm.sv).
# Every run is still checked bit for bit against the golden fx_solve; a config with errors != 0 fails.
#   perf_sweep.py <build/accel> <out_dir> [--gfirst 0 --glast 699 --seeds 1 --iters 100000 -j 12] [--only name,...]
#   --glast -1 runs tb_core's 14-problem list instead (the problems of the CPU baseline, tools/run_fx_soc.sh)
# Writes <out_dir>/<config>.log and <out_dir>/runs.csv (one row per config x run) and summary.csv.
import argparse, csv, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

# name -> (variant, plusargs); variant = tb_core_<variant>_perf build directory
def configs():
    c = {}
    c["ideal"] = ("", "+cse_mode=1 +lat=1 +ii=1")
    c["nominal"] = ("", "+cse_mode=2 +nu=4 +lanes=8 +lat=4")
    for lat in (1, 4, 16, 64, 256, 1024):
        c[f"lat{lat}"] = ("", f"+cse_mode=1 +lat={lat} +ii=1")
    for ii in (1, 2, 4, 8, 16, 32, 64):
        c[f"ii{ii}"] = ("", f"+cse_mode=1 +lat=4 +ii={ii}")
    for nu in (1, 2, 4, 8, 16):
        c[f"nu{nu}"] = ("", f"+cse_mode=2 +nu={nu} +lanes=8 +lat=4")
    for nm, (v, a) in list(c.items()):
        if nm in ("nominal", "nu1", "nu2", "lat64", "lat256", "lat1024", "ii8", "ii16", "ideal"):
            c[f"ak_{nm}"] = ("ak", a)
    for nm in ("ideal", "nominal", "nu1", "nu2", "ii8"):
        c[f"flat_{nm}"] = ("flat", c[nm][1])
        c[f"flatak_{nm}"] = ("flatak", c[nm][1])
    return c


def run(build, out, name, variant, args, common):
    exe = os.path.join(build, f"tb_core{'_' + variant if variant else ''}_perf", "Vtb_core")
    if not variant:
        exe = os.path.join(build, "tb_core_perf", "Vtb_core")
    log = os.path.join(out, name + ".log")
    cmd = [exe, "+perf=1", "+extra=0"] + common + args.split()
    with open(log, "w") as f:
        rc = subprocess.call(cmd, stdout=f, stderr=subprocess.STDOUT)
    return name, rc, log


def parse(log):
    runs, tb, cse = [], None, None
    for line in open(log):
        if line.startswith("PERF "):
            runs.append({k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", line)})
        elif line.startswith("TB_CORE"):
            tb = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", line)}
        elif line.startswith("PERFCSE"):
            cse = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", line)}
    return runs, tb, cse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build"); ap.add_argument("out")
    ap.add_argument("--gfirst", type=int, default=0); ap.add_argument("--glast", type=int, default=699)
    ap.add_argument("--seeds", type=int, default=1); ap.add_argument("--iters", type=int, default=100000)
    ap.add_argument("-j", type=int, default=12); ap.add_argument("--only", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    # --glast -1: tb_core's own 14-problem list (the CPU baseline set), seeds as given
    common = ["+directed=0"] + ([f"+gfirst={a.gfirst}", f"+glast={a.glast}"] if a.glast >= 0 else []) + \
             [f"+seeds={a.seeds}", f"+iters={a.iters}"]
    cfg = configs()
    if a.only:
        cfg = {k: v for k, v in cfg.items() if k in a.only.split(",")}
    bad = 0
    rows, summ = [], []
    with ThreadPoolExecutor(a.j) as ex:
        for name, rc, log in ex.map(lambda kv: run(a.build, a.out, kv[0], kv[1][0], kv[1][1], common), cfg.items()):
            runs, tb, cse = parse(log)
            ok = rc == 0 and tb is not None and tb.get("errors", 1) == 0
            if not ok:
                bad += 1
            print(f"{name:18s} rc={rc} runs={len(runs)} errors={tb.get('errors') if tb else 'NA'} "
                  f"cycles={sum(r['cycles'] for r in runs)}", flush=True)
            for r in runs:
                rows.append(dict(config=name, **r))
            s = dict(config=name, variant=cfg[name][0], args=cfg[name][1], rc=rc, nruns=len(runs),
                     errors=tb.get("errors") if tb else -1)
            for k in ("cycles", "queries", "killed", "poses", "iterations", "edges") + tuple(
                    k for k in (runs[0] if runs else {}) if k.startswith("B_")):
                s[k] = sum(r[k] for r in runs)
            if cse:
                s.update({"cse_" + k: v for k, v in cse.items()})
            summ.append(s)
    for fn, data in (("runs.csv", rows), ("summary.csv", summ)):
        keys = []
        for d in data:
            for k in d:
                if k not in keys:
                    keys.append(k)
        with open(os.path.join(a.out, fn), "w", newline="") as f:
            w = csv.DictWriter(f, keys); w.writeheader(); w.writerows(data)
    print(f"configs={len(cfg)} failed={bad}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
