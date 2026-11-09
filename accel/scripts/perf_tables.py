#!/usr/bin/env python3
# Tables for the report from perf_sweep.py's summary.csv: B2 cycle breakdown, B3 CSE timing sweeps,
# B4 auto-kill on/off, hierarchical vs flat queries.  perf_tables.py <summary.csv>
import csv, sys

rows = {r["config"]: r for r in csv.DictReader(open(sys.argv[1]))}
B = ["B_CTRL", "B_SMP", "B_NN", "B_FK", "B_QDFULL", "B_SCU", "B_TAG", "B_CSE", "B_DRAIN"]
NAMES = {"B_CTRL": "ctrl/ALU", "B_SMP": "SMP", "B_NN": "NN", "B_FK": "FKU", "B_QDFULL": "QD full",
         "B_SCU": "SCU", "B_TAG": "tag wait", "B_CSE": "CSE wait", "B_DRAIN": "drain"}
I = lambda r, k: int(r[k])
base = I(rows["ideal"], "cycles")


def bd(name):
    r = rows[name]
    c = I(r, "cycles")
    s = sum(I(r, b) for b in B)
    assert s == c, (name, s, c)          # every busy cycle is in exactly one bucket
    return c, [100.0 * I(r, b) / c for b in B]


print("## B2 cycle breakdown (% of PLE busy cycles)")
print("| config | cycles | vs ideal | " + " | ".join(NAMES[b] for b in B) + " |")
print("|---" * (len(B) + 3) + "|")
for n in ("ideal", "nominal", "nu2", "nu1", "lat64", "lat256", "ii4", "ii8"):
    if n in rows:
        c, p = bd(n)
        print(f"| {n} | {c:,} | {c / base:.2f} | " + " | ".join(f"{x:.1f}" for x in p) + " |")

print("\n## B3 CSE timing sweeps (total PLE cycles relative to ideal CSE)")
for title, keys in (("latency (ii = 1)", [f"lat{x}" for x in (1, 4, 16, 64, 256, 1024)]),
                    ("issue interval ii (lat = 4)", [f"ii{x}" for x in (1, 2, 4, 8, 16, 32, 64)]),
                    ("work-based units nu (8 point lanes, lat 4)", [f"nu{x}" for x in (1, 2, 4, 8, 16)])):
    ks = [k for k in keys if k in rows]
    print(f"\n{title}")
    print("| " + " | ".join(ks) + " |")
    print("|---" * len(ks) + "|")
    print("| " + " | ".join(f"{I(rows[k], 'cycles') / base:.2f}" for k in ks) + " |")
    if title.startswith("work"):
        q = [I(rows[k], "queries") for k in ks]
        w = [I(rows[k], "cse_work_cycles") for k in ks]
        print("| " + " | ".join(f"util {wk / I(rows[k], 'cse_nu') / I(rows[k], 'cycles'):.2f}" for k, wk in zip(ks, w)) + " |")
        print(f"mean CSE work per query (cycles, one unit): {w[0] / q[0]:.2f}")

print("\n## B4 auto-kill (same CSE timing, AK off -> on)")
print("| config | cycles off | cycles on | queries off | queries on | KILLED off | KILLED on | CSE work off | CSE work on |")
print("|---" * 9 + "|")
for n in ("ideal", "nominal", "nu1", "nu2", "lat64", "lat256", "lat1024", "ii8", "ii16"):
    a, k = rows.get(n), rows.get("ak_" + n)
    if a and k:
        print(f"| {n} | {I(a,'cycles'):,} | {I(k,'cycles'):,} | {I(a,'queries'):,} | {I(k,'queries'):,} | "
              f"{I(a,'killed'):,} | {I(k,'killed'):,} | {I(a,'cse_work_cycles'):,} | {I(k,'cse_work_cycles'):,} |")

print("\n## hierarchical vs flat (fine spheres only)")
print("| config | cycles hier | cycles flat | queries hier | queries flat | CSE work hier | CSE work flat |")
print("|---" * 7 + "|")
for n in ("ideal", "nominal", "nu1", "nu2", "ii8"):
    a, f = rows.get(n), rows.get("flat_" + n)
    if a and f:
        print(f"| {n} | {I(a,'cycles'):,} | {I(f,'cycles'):,} | {I(a,'queries'):,} | {I(f,'queries'):,} | "
              f"{I(a,'cse_work_cycles'):,} | {I(f,'cse_work_cycles'):,} |")
r = rows["ideal"]
print(f"\nideal: runs={r['nruns']} iterations={I(r,'iterations'):,} edges={I(r,'edges'):,} poses={I(r,'poses'):,} "
      f"queries={I(r,'queries'):,} cycles/pose={I(r,'cycles') / I(r,'poses'):.1f} FKU cycles/pose={I(r,'B_FK') / I(r,'poses'):.1f} "
      f"queries/pose={I(r,'queries') / I(r,'poses'):.2f}")
