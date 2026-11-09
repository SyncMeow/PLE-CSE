#!/usr/bin/env python3
# Report figures from perf_sweep.py's summary.csv (+ the CAP sweep): B2 breakdown, B3 sweeps, CAP failure rate.
#   perf_figs.py <summary.csv> <out_dir> [<capsweep_dir>]
import csv, glob, os, re, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURF, INK, INK2, GRID = "#fcfcfb", "#1f1e1c", "#5c5b55", "#e4e3de"
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # validated (dataviz validator)
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "text.color": INK, "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
                     "grid.linewidth": 0.6})

src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
rows = {r["config"]: r for r in csv.DictReader(open(src))}
I = lambda n, k: int(rows[n][k])
base = I("ideal", "cycles")

# ---- B2: stacked breakdown, 6 groups (fixed hue order)
groups = [("FKU", ["B_FK"]), ("SCU", ["B_SCU"]), ("CSE (QD full + wait + tag)", ["B_QDFULL", "B_CSE", "B_TAG"]),
          ("control / ALU", ["B_CTRL"]), ("NN + sampler", ["B_NN", "B_SMP"]), ("drain", ["B_DRAIN"])]
cfgs = [c for c in ("ideal", "nominal", "nu2", "nu1", "ii8", "lat256") if c in rows]
label = {"ideal": "ideal CSE", "nominal": "4 units (nominal)", "nu2": "2 units", "nu1": "1 unit",
         "ii8": "1 query / 8 cyc", "lat256": "latency 256"}
fig, ax = plt.subplots(figsize=(8, 3.6))
for yi, c in enumerate(cfgs):
    left = 0.0
    for gi, (gname, ks) in enumerate(groups):
        v = sum(I(c, k) for k in ks) / base
        ax.barh(yi, v, left=left, color=CAT[gi], edgecolor=SURF, linewidth=2, height=0.62,
                label=gname if yi == 0 else None)
        if v > 0.12:
            ax.text(left + v / 2, yi, f"{100 * v * base / I(c, 'cycles'):.0f}%", ha="center", va="center",
                    fontsize=8, color="white" if gi in (0, 5) else INK)
        left += v
    ax.text(left + 0.03, yi, f"{left:.2f}x", va="center", fontsize=9, color=INK)
ax.set_yticks(range(len(cfgs)), [label[c] for c in cfgs])
ax.invert_yaxis()
ax.set_xlabel("PLE cycles, relative to an ideal CSE (labels: share of each run)")
ax.grid(axis="y", visible=False)
ax.set_title("Where the PLE spends its cycles", loc="left", fontsize=12, color=INK)
ax.legend(ncol=3, fontsize=8, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.2))
fig.tight_layout()
fig.savefig(os.path.join(out, "b2_breakdown.png"), dpi=180)

# ---- B3: three sweeps, one series each (one hue, no legend)
sw = [("CSE latency (1 query / cycle)", "cycles", [1, 4, 16, 64, 256, 1024], "lat"),
      ("CSE issue interval (latency 4)", "cycles / query", [1, 2, 4, 8, 16, 32, 64], "ii"),
      ("CSE units, design-doc cost model", "units", [1, 2, 4, 8, 16], "nu")]
fig, axs = plt.subplots(1, 3, figsize=(10, 3.2), sharey=True)
for ax, (t, xl, xs, p) in zip(axs, sw):
    pts = [(x, I(f"{p}{x}", "cycles") / base) for x in xs if f"{p}{x}" in rows]
    if p in ("lat", "ii") and not any(x == 1 for x, _ in pts):          # lat1 / ii1 are the ideal CSE
        pts.append((1, 1.0)); pts.sort()
    if p == "nu" and "nominal" in rows and not any(x == 4 for x, _ in pts):
        pts.append((4, I("nominal", "cycles") / base)); pts.sort()
    ax.plot([x for x, _ in pts], [y for _, y in pts], color=CAT[0], linewidth=2, marker="o", markersize=6,
            markeredgecolor=SURF, markeredgewidth=1.5)
    for x, y in pts:
        if y > 1.4 or x == pts[0][0]:
            ax.annotate(f"{y:.2f}", (x, y), textcoords="offset points", xytext=(0, 7), ha="center", fontsize=8, color=INK2)
    ax.set_xscale("log", base=2)
    ax.set_xticks([x for x, _ in pts], [str(x) for x, _ in pts])
    ax.set_xlabel(xl)
    ax.set_title(t, loc="left", fontsize=10, color=INK)
axs[0].set_ylabel("PLE cycles / ideal-CSE cycles")
fig.tight_layout()
fig.savefig(os.path.join(out, "b3_cse_sweep.png"), dpi=180)

# ---- CAP: FAIL_CAPACITY rate vs tree capacity (golden model, 699 problems x 3 seeds)
if len(sys.argv) > 3:
    pts = []
    for f in glob.glob(os.path.join(sys.argv[3], "cap*.csv")):
        cap = int(re.search(r"cap(\d+)\.csv", f).group(1))
        st = [line.split(",")[4] for line in open(f) if line.startswith("FXRUN")]
        pts.append((cap, 100.0 * st.count("2") / len(st)))
    pts.sort()
    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot([c for c, _ in pts], [r for _, r in pts], color=CAT[0], linewidth=2, marker="o", markersize=6,
            markeredgecolor=SURF, markeredgewidth=1.5)
    for c, r in pts:
        ax.annotate(f"{r:.1f}%", (c, r), textcoords="offset points", xytext=(0, 7), ha="center", fontsize=8, color=INK2)
    ax.set_xscale("log", base=2)
    ax.set_xticks([c for c, _ in pts], [str(c) for c, _ in pts])
    ax.set_xlabel("tree capacity (nodes per tree)")
    ax.set_ylabel("FAIL_CAPACITY runs (%)")
    ax.set_title("Tree capacity vs failures (2,097 runs)", loc="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "cap_fail.png"), dpi=180)
print("figures in", out)
