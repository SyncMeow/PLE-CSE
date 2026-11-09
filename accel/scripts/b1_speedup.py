#!/usr/bin/env python3
# B1: PLE cycles (tb_core perf runs) vs fx_solve on the RTL CPU (sw_rrtc tools/run_fx_soc.sh), same 28 runs.
# Checks first that the CPU runs are the golden runs: every planner field equal to host rrtc_fx (FXRUN) and to
# the PLE run (iterations, sizes, edges, nn queries, status).
#   b1_speedup.py <fx_soc_dir> <host_golden.csv> <perf_28 dir> [config ...]
import csv, glob, math, os, re, sys

fx_dir, golden_csv, perf_dir = sys.argv[1:4]
cfgs = sys.argv[4:] or ["ideal", "nominal"]

# FXSOC,prob,seed,status,iterations,size_start,size_goal,dd_rejects,edges,poses,nn_queries,path_len,path_hash,
#       env_queries,self_pairs,cycles,cyc_pose,cyc_env
cpu = {}
for f in glob.glob(os.path.join(fx_dir, "p*_prof0_blob", "sim.log")):
    for line in open(f, errors="replace"):
        i = line.find("FXSOC,")
        if i < 0 or line[i:].startswith("FXSOC START") or line[i:].startswith("FXSOC DONE"):
            continue
        v = line[i:].strip().split(",")
        cpu[(int(v[1]), int(v[2]))] = v
gold = {}
for v in csv.reader(open(golden_csv)):
    # <prob>,FXRUN,scene,index,seed,status,iterations,size_start,size_goal,dd_rejects,edges,poses,nn_queries,path_len,hash,cost,envq,selfp
    gold[(int(v[0]), int(v[4]))] = v
bad = 0
for k, v in sorted(cpu.items()):
    g = gold.get(k)
    if g is None:
        continue
    cpu_f = v[3:15]                      # status .. self_pairs
    gold_f = g[5:15] + g[16:18]          # status .. hash, env_queries, self_pairs (skip float cost)
    if [x.lower() for x in cpu_f] != [x.lower() for x in gold_f]:
        print("MISMATCH cpu vs host golden", k, cpu_f, gold_f)
        bad += 1
print(f"cpu runs={len(cpu)} compared with host golden={sum(1 for k in cpu if k in gold)} mismatches={bad}")


def perf(cfg):
    out = {}
    for line in open(os.path.join(perf_dir, cfg + ".log")):
        if line.startswith("PERF "):
            d = {k: int(x) for k, x in re.findall(r"(\w+)=(-?\d+)", line)}
            out[(d["prob"], d["seed"])] = d
    return out


for cfg in cfgs:
    p = perf(cfg)
    rows, mism = [], 0
    for k, v in sorted(cpu.items()):
        d = p.get(k)
        if d is None:
            print("missing PLE run", cfg, k); mism += 1; continue
        same = (int(v[3]), int(v[4]), int(v[5]), int(v[6]), int(v[8]), int(v[10])) == \
               (d["status"], d["iterations"], d.get("size_start", int(v[5])), d.get("size_goal", int(v[6])), d["edges"], d["nn"])
        if not same:
            print("MISMATCH cpu vs PLE", cfg, k); mism += 1
        rows.append((k, int(v[15]), d["cycles"], int(v[9])))
    sp = [c / h for _, c, h, _ in rows]
    tot_c = sum(r[1] for r in rows); tot_h = sum(r[2] for r in rows)
    geo = math.exp(sum(math.log(x) for x in sp) / len(sp))
    sps = sorted(sp)
    print(f"{cfg}: runs={len(rows)} mismatches={mism} cpu_cycles={tot_c} ple_cycles={tot_h} "
          f"speedup_total_cycles={tot_c / tot_h:.0f}x geomean={geo:.0f}x min={sps[0]:.0f}x median={sps[len(sps)//2]:.0f}x max={sps[-1]:.0f}x "
          f"time_speedup_cpu75_ple100={tot_c / 75e6 / (tot_h / 100e6):.0f}x")
    if cfg == cfgs[0]:
        print("prob,seed,poses,cpu_cycles,ple_cycles,speedup,cpu_cycles_per_pose,ple_cycles_per_pose")
        for (k, c, h, n) in rows:
            print(f"{k[0]},{k[1]},{n},{c},{h},{c / h:.0f},{c / max(n, 1):.0f},{h / max(n, 1):.1f}")
