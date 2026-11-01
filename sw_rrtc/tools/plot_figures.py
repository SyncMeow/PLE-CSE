#!/usr/bin/env python3
"""Report figures (PNG, light theme, English) from the board log and the host experiments.

  plot_figures.py --board last_rrtc_brute.log --exp Platform/build/sw_rrtc_host/exp2 --out <dir>
Needs matplotlib. Figures:
  fig1_stage_time      VCC stage time per problem on the SoC, by scene (board)
  fig2_stage_share     share of end-to-end time per stage, overall and by scene (board)
  fig3_fs_curve        NN share of time vs collision-check speed-up S (board, projected)
  fig4_hit_heatmap     Morton hit rate vs (m, W), shadow mode, B = 1 (host)
  fig5_iter_check_cdf  iterations / CHECKs, brute force vs closed-loop Morton, B = 1 (host)
  fig6_tree_size       tree size at solution (host)
"""
import argparse
import bisect
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze as A  # noqa: E402

SER = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6']
FG, FG2, MUTED, GRID, RULE, THR = '#151716', '#4f5552', '#7d837f', '#e8ebe9', '#c9cecb', '#d03b3b'
SCENES = ['table_pick', 'table_under_pick', 'box', 'bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'cage']
CLK = 75e6
STAGES = [('filter', 'CenterVox filtering'), ('build', 'MVT build'), ('nn', 'Nearest neighbour'),
          ('fk', 'Forward kinematics'), ('env', 'Env. check (Alg. 1)'), ('self', 'Self-collision + rest of FK/CC'),
          ('other', 'Sampler + other')]

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.edgecolor': RULE, 'axes.labelcolor': FG2,
    'xtick.color': FG2, 'ytick.color': FG2, 'axes.titlesize': 11, 'axes.titleweight': 'bold',
    'axes.titlecolor': FG, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': False,
    'figure.facecolor': 'white', 'axes.facecolor': 'white', 'savefig.dpi': 200, 'legend.frameon': False,
})


def ygrid(ax):
    ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def xgrid(ax):
    ax.grid(axis='x', color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def save(fig, out, name):
    p = os.path.join(out, name + '.png')
    fig.savefig(p, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print('wrote', p)


# ---------------------------------------------------------------- data
def board(path):
    txt = open(path, errors='replace').read()
    txt = txt[txt.rindex('RRTC START'):]
    runs, preps = A.parse_text(txt, 'RUN', A.RUN_COLS), A.parse_text(txt, 'PREP', A.PREP_COLS)
    per = []
    for s in range(7):
        R = [r for r in runs if r['scene'] == s]
        P = [p for p in preps if p['scene'] == s]
        n = len(P)
        g = lambda k: sum(r[k] for r in R) / n / CLK  # noqa: E731
        fkcc, fk, env = g('cyc_fkcc'), g('cyc_fk'), g('cyc_env')
        nn = g('cyc_nnq') + g('cyc_nni')
        per.append({'scene': SCENES[s], 'n': n,
                    'filter': sum(p['cyc_filter'] for p in P) / n / CLK, 'build': sum(p['cyc_build'] for p in P) / n / CLK,
                    'nn': nn, 'fk': fk, 'env': env, 'self': fkcc - fk - env,
                    'other': g('cyc_total') - nn - fkcc})
    tot = {k: sum(r['cyc_' + k] for r in runs) for k in ('total', 'nnq', 'nni', 'fkcc', 'fk', 'env')}
    tot['prep'] = sum(p['cyc_filter'] + p['cyc_build'] for p in preps)
    return runs, preps, per, tot


# ---------------------------------------------------------------- figures
def fig_stage_time(per, out):
    fig, ax = plt.subplots(figsize=(9, 4.2))
    y = list(range(len(per)))[::-1]
    left = [0.0] * len(per)
    for j, (k, name) in enumerate(STAGES):
        w = [r[k] for r in per]
        ax.barh(y, w, left=left, height=0.62, color=SER[j], edgecolor='white', linewidth=1.2, label=name)
        left = [a + b for a, b in zip(left, w)]
    for yi, t in zip(y, left):
        ax.text(t + 0.03, yi, f'{t:.2f} s', va='center', fontsize=9, color=FG2)
    ax.set_yticks(y, [r['scene'] for r in per])
    ax.set_xlabel('Seconds per problem at 75 MHz (pre-planning + one planning run)')
    ax.set_title('VCC stage time per problem on the SoC (board, 699 problems, seed 0)', loc='left')
    ax.set_xlim(0, max(left) * 1.12)
    xgrid(ax)
    ax.legend(ncol=4, loc='upper center', bbox_to_anchor=(0.5, -0.16), fontsize=9)
    save(fig, out, 'fig1_stage_time')


def fig_stage_share(per, out):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={'width_ratios': [1, 1.25]})
    n_all = sum(r['n'] for r in per)
    mean = {k: sum(r[k] * r['n'] for r in per) / n_all for k, _ in STAGES}
    total = sum(mean.values())
    order = sorted(range(len(STAGES)), key=lambda j: mean[STAGES[j][0]])
    ys = list(range(len(order)))
    vals = [mean[STAGES[j][0]] / total * 100 for j in order]
    a1.barh(ys, vals, height=0.62, color=[SER[j] for j in order])
    for yi, j, v in zip(ys, order, vals):
        a1.text(v + 0.8, yi, f'{v:.1f}%  ({mean[STAGES[j][0]]:.3f} s)', va='center', fontsize=9, color=FG2)
    a1.set_yticks(ys, [STAGES[j][1] for j in order])
    a1.set_xlim(0, max(vals) * 1.45)
    a1.set_xlabel('Share of mean end-to-end time per problem (%)')
    a1.set_title(f'All scenes: mean {total:.2f} s per problem', loc='left')
    xgrid(a1)
    # 100% stacked by scene
    y = list(range(len(per)))[::-1]
    left = [0.0] * len(per)
    for j, (k, name) in enumerate(STAGES):
        tots = [sum(r[kk] for kk, _ in STAGES) for r in per]
        w = [r[k] / t * 100 for r, t in zip(per, tots)]
        a2.barh(y, w, left=left, height=0.62, color=SER[j], edgecolor='white', linewidth=1.2, label=name)
        for yi, l0, wi in zip(y, left, w):
            if wi >= 7:
                a2.text(l0 + wi / 2, yi, f'{wi:.0f}', ha='center', va='center', fontsize=8, color='white')
        left = [a + b for a, b in zip(left, w)]
    a2.set_yticks(y, [r['scene'] for r in per])
    a2.set_xlim(0, 100)
    a2.set_xlabel('Share of end-to-end time (%)')
    a2.set_title('By scene', loc='left')
    a2.legend(ncol=4, loc='upper center', bbox_to_anchor=(0.45, -0.16), fontsize=9)
    fig.suptitle('Share of time per VCC stage on the SoC (board, 699 problems, seed 0)', x=0.01, ha='left',
                 fontweight='bold', color=FG)
    fig.tight_layout()
    save(fig, out, 'fig2_stage_share')


def fig_fs(tot, out):
    nn = tot['nnq'] + tot['nni']
    env, self_ = tot['env'], tot['fkcc'] - tot['fk'] - tot['env']
    T, prep = tot['total'], tot['prep']
    S = [10 ** (k / 20) for k in range(0, 61)]
    curves = {
        'plan': ([nn / (T - env + env / s) * 100 for s in S], [nn / (T - env - self_ + (env + self_) / s) * 100 for s in S]),
        'e2e': ([nn / (T - env + env / s + prep) * 100 for s in S],
                [nn / (T - env - self_ + (env + self_) / s + prep) * 100 for s in S]),
    }
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, key, title in zip(axes, ('plan', 'e2e'), ('Share of planning time', 'Share of end-to-end time (incl. pre-planning)')):
        a, b = curves[key]
        ax.plot(S, a, color=SER[0], lw=2, label='Offload env. check only')
        ax.plot(S, b, color=SER[1], lw=2, label='Offload env. check + self-collision')
        ax.axhline(10, color=THR, lw=1.3, ls=(0, (5, 4)), label='10% threshold')
        for ys, c in ((a, SER[0]), (b, SER[1])):
            ax.plot(S[-1], ys[-1], 'o', color=c, ms=6, mec='white', mew=1.5)
            ax.text(S[-1] * 1.15, ys[-1], f'{ys[-1]:.2f}%', va='center', fontsize=9, color=FG2)
        ax.text(1.05, 11.4, 'end labels: S = 1000 (S -> inf: +0.04 pt at most)', fontsize=8, color=MUTED)
        ax.set_xscale('log')
        ax.set_xlim(1, 1000 * 2.6)
        ax.set_xticks([1, 10, 100, 1000], ['x1', 'x10', 'x100', 'x1000'])
        ax.set_xlabel('Collision-check speed-up S (log)')
        ax.set_title(title, loc='left')
        ax.set_ylim(0, 12)
        ygrid(ax)
    axes[0].set_ylabel('Nearest-neighbour share (%)')
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, ncol=3, loc='lower center', bbox_to_anchor=(0.5, -0.06), fontsize=9)
    fig.suptitle('Nearest-neighbour share of time vs accelerator speed-up (board cycles, FK stays on the CPU)',
                 x=0.01, ha='left', fontweight='bold', color=FG)
    fig.tight_layout()
    save(fig, out, 'fig3_fs_curve')


def fig_heat(exp, out):
    agg = {}
    for l in open(os.path.join(exp, 'shadowB1.csv')):
        v = l.split(',')
        k = (int(v[4]), int(v[5]), int(v[6]))
        a = agg.setdefault(k, [0, 0])
        a[0] += int(v[7])
        a[1] += int(v[8])
    Ws, ms = [1, 2, 4, 8, 16, 32], [1, 2, 4, 8]
    names = ['EXTEND (accepted)', 'CONNECT', 'EXTEND (rejected by dynamic domain)']
    cmap = LinearSegmentedColormap.from_list('seq', ['#cde2fb', '#0d366b'])
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    for qt, ax in enumerate(axes):
        M = [[agg[(qt, m, W)][1] / agg[(qt, m, W)][0] * 100 for W in Ws] for m in ms]
        ax.imshow(M, cmap=cmap, vmin=40, vmax=100, aspect='auto')
        for i in range(len(ms)):
            for j in range(len(Ws)):
                v = M[i][j]
                ax.text(j, i, f'{v:.0f}' if v >= 99.95 else f'{v:.1f}', ha='center', va='center', fontsize=9,
                        color='white' if (v - 40) / 60 > 0.55 else FG)
        ax.set_xticks(range(len(Ws)), [f'W={w}' for w in Ws])
        ax.set_yticks(range(len(ms)), [f'm={m}' for m in ms])
        ax.set_title(f'{names[qt]}\n{agg[(qt, 1, 1)][0]:,} queries', loc='left', fontsize=10)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.tick_params(length=0)
    fig.suptitle('Morton window search: exact-hit rate (%) by shifts m and half-window W (shadow mode, buffer B = 1)',
                 x=0.01, ha='left', fontweight='bold', color=FG)
    fig.tight_layout()
    save(fig, out, 'fig4_hit_heatmap')


def fig_cdf(exp, out):
    br = A.parse(os.path.join(exp, 'brute.csv'), 'RUN', A.RUN_COLS)
    cfg = [('Brute force (exact)', br)] + [
        (f'Morton m={m}, W={W}', A.parse(os.path.join(exp, f'mortonB1_m{m}_W{W}.csv'), 'RUN', A.RUN_COLS))
        for m, W in [(1, 1), (1, 4), (2, 4), (4, 8)]]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, key, label in zip(axes, ('iters', 'checks'), ('Iterations', 'CHECKs (<= 32 poses each)')):
        for i, (name, R) in reversed(list(enumerate(cfg))):   # brute force drawn last, dashed, on top
            v = sorted(max(r[key], 1) for r in R)               # 0 iterations (direct start-goal edge) shown at 1
            xs = sorted(set(v))
            ys = [bisect.bisect_right(v, x) / len(v) * 100 for x in xs]
            ck = sum(r['checks'] for r in R) / sum(r['checks'] for r in br)
            lab = name if key == 'iters' else f'{name}  (total x{ck:.2f})'
            ax.step(xs, ys, where='post', color=FG if i == 0 else SER[i], lw=1.6 if i == 0 else 1.8,
                    ls=(0, (4, 3)) if i == 0 else '-', label=lab, zorder=3 if i == 0 else 2)
        ax.set_xscale('log')
        ax.set_xlabel(label + ' per run (log; 0 shown as 1)')
        ax.set_ylim(0, 101)
        ax.set_title(label, loc='left')
        ygrid(ax)
        h, l = ax.get_legend_handles_labels()
        ax.legend(h[::-1], l[::-1], loc='lower right', fontsize=8.5)
    axes[0].set_ylabel('Runs with value <= x (%)')
    fig.suptitle('Iterations and CHECKs per run: brute force vs closed-loop Morton (699 problems x 10 seeds, B = 1)',
                 x=0.01, ha='left', fontweight='bold', color=FG)
    fig.tight_layout()
    save(fig, out, 'fig5_iter_check_cdf')


def fig_tree(exp, out):
    br = A.parse(os.path.join(exp, 'brute.csv'), 'RUN', A.RUN_COLS)
    sizes = [r['size_s'] + r['size_g'] for r in br if r['solved']]
    edges = [2 ** k for k in range(1, 10)]
    hist = [sum(edges[i] <= x < edges[i + 1] for x in sizes) for i in range(len(edges) - 1)]
    pct = [h / len(sizes) * 100 for h in hist]
    labels = [f'{edges[i]}-{edges[i + 1] - 1}' for i in range(len(edges) - 1)]
    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.bar(range(len(pct)), pct, width=0.82, color=SER[0])
    for i, v in enumerate(pct):
        ax.text(i, v + 0.8, f'{v:.1f}%', ha='center', fontsize=9, color=FG2)
    ax.set_xticks(range(len(pct)), labels)
    ax.set_xlabel('Nodes in both trees at solution (log2 bins)')
    ax.set_ylabel('Share of solved runs (%)')
    med, p90, p99 = A.q(sizes, .5), A.q(sizes, .9), A.q(sizes, .99)
    ax.set_title(f'Tree size at solution: median {med:.0f}, p90 {p90:.0f}, p99 {p99:.0f}, max {max(sizes)} '
                 f'({len(sizes):,} solved runs, brute-force NN)', loc='left')
    ax.set_ylim(0, max(pct) * 1.15)
    ygrid(ax)
    save(fig, out, 'fig6_tree_size')


def fig_nn_tradeoff(exp, out):
    br = A.parse(os.path.join(exp, 'brute.csv'), 'RUN', A.RUN_COLS)
    bc, bn = sum(r['checks'] for r in br), sum(r['nn_evals'] for r in br)

    def ratio(f):
        R = A.parse(os.path.join(exp, f), 'RUN', A.RUN_COLS)
        return sum(r['checks'] for r in R) / bc, sum(r['nn_evals'] for r in R) / bn
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.add_patch(plt.Rectangle((0.9, 0.25), 0.1, 0.75, color='#e3eedf', zorder=0, lw=0))
    ax.axhline(1, color=FG, lw=1.4, ls=(0, (4, 3)), zorder=1)
    ax.axvline(1, color=FG, lw=1.4, ls=(0, (4, 3)), zorder=1)
    ax.text(1.62, 1.07, 'brute-force NN work', fontsize=9, color=FG, ha='right')
    b32 = [ratio(f'morton_m{m}_W{W}.csv') for m in (1, 2, 4, 8) for W in (1, 2, 4, 8, 16, 32)]
    ax.scatter([x for x, _ in b32], [y for _, y in b32], s=40, facecolors='none', edgecolors=MUTED, lw=1.3, zorder=2,
               label='Morton, buffer B = 32 (16 settings; most trees fit in the exact buffer)')
    for i, (m, W) in enumerate([(1, 1), (1, 4), (2, 4), (4, 8), (8, 8)]):
        x, y = ratio(f'mortonB1_m{m}_W{W}.csv')
        ax.scatter([x], [y], s=70, color=SER[i % len(SER)], zorder=3)
        ax.annotate(f'm={m}, W={W}', (x, y), xytext=(8, -4 if i != 2 else -16), textcoords='offset points', fontsize=9,
                    color=SER[i % len(SER)])
    ax.scatter([1], [1], s=150, marker='*', color=FG, zorder=4, label='Brute force (exact)')
    ax.scatter([], [], s=70, color=FG2, label='Morton, buffer B = 1 (Morton search only; labelled)')
    ax.set_yscale('log')
    ax.set_yticks([0.25, 0.5, 1, 2, 4], ['0.25', '0.5', '1', '2', '4'])
    ax.set_xlim(0.9, 1.65)
    ax.set_ylim(0.25, 6)
    ax.text(0.904, 0.27, 'better than\nbrute force\non both axes', fontsize=9, color='#1d7a3a', va='bottom')
    ax.set_xlabel('Total CHECKs relative to brute force')
    ax.set_ylabel('NN distance computations\nrelative to brute force (log)')
    ax.set_title('Closed-loop Morton NN: saving NN work costs extra CHECKs (699 problems x 10 seeds)', loc='left')
    ygrid(ax)
    ax.legend(fontsize=8.5, loc='upper right')
    save(fig, out, 'fig12_nn_tradeoff')


def nn_time_table(exptime, reps=3, nboot=1000, seed=1):
    """Per NN setting, ratios to brute force: NN time (reps), distances, and paired-bootstrap 95% CIs for poses and planning time."""
    import random
    cfgs = [('Brute force', 'brute')] + \
        [(f'm={m}, W={W}', f'mortonB1_m{m}_W{W}') for m, W in [(1, 1), (1, 4), (2, 4), (4, 8), (8, 8)]] + \
        [(f'm={m}, W={W}', f'morton_m{m}_W{W}') for m in (1, 2, 4, 8) for W in (1, 2, 4, 8, 16, 32)]
    load = lambda f, r: A.parse(os.path.join(exptime, f'{f}_r{r}.csv'), 'RUN', A.RUN_COLS)
    key = lambda x: (x['scene'], x['index'], x['seed'])
    base = [load('brute', r) for r in range(1, reps + 1)]
    b0 = {key(x): x for x in base[0]}
    nnt = lambda R: sum(x['cyc_nnq'] + x['cyc_nni'] for x in R)
    tot = lambda R: sum(x['cyc_total'] for x in R)
    rng = random.Random(seed)

    def boot(pairs):
        n = len(pairs)
        bs = []
        for _ in range(nboot):
            a = b = 0
            for _ in range(n):
                x, y = pairs[rng.randrange(n)]
                a += x
                b += y
            bs.append(a / b)
        bs.sort()
        return sum(x for x, _ in pairs) / sum(y for _, y in pairs), bs[int(.025 * nboot)], bs[int(.975 * nboot) - 1]
    rows = []
    for label, f in cfgs:
        runs = [load(f, r) for r in range(1, reps + 1)]
        po = boot([(x['poses'], b0[key(x)]['poses']) for x in runs[0]])
        ck = boot([(x['checks'], b0[key(x)]['checks']) for x in runs[0]])
        pl = boot([(x['cyc_total'], b0[key(x)]['cyc_total']) for x in runs[0]])
        rows.append(dict(label=label, file=f, poses=po, ck=ck, plan=pl,
                         nn=sorted(nnt(R) / nnt(B) for R, B in zip(runs, base)),
                         plan_reps=sorted(tot(R) / tot(B) for R, B in zip(runs, base)),
                         dist=sum(x['nn_evals'] for x in runs[0]) / sum(x['nn_evals'] for x in base[0]),
                         nn_share=nnt(base[0]) / tot(base[0])))
    return rows


def fig_nn_time(exptime, out):
    rows = nn_time_table(exptime)
    n = len(rows)
    ys = []
    y = 0
    for i in range(n):           # gaps between brute / B=1 / B=32 groups
        if i in (1, 6):
            y += 0.8
        ys.append(y)
        y += 1
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 6.2), sharey=True, gridspec_kw=dict(width_ratios=[1.25, 1, 1]))
    panels = [('nn', '(a) NN time: query + insert (measured)', 'log'),
              ('poses', '(b) Poses collision-checked (95% CI)', 'lin'),
              ('plan', '(c) Total planning time (measured, 95% CI)', 'lin')]
    for ax, (k, title, sc) in zip(axes, panels):
        ax.axvline(1, color=FG, lw=1.3, ls=(0, (4, 3)), zorder=1)
        for r, yy in zip(rows, ys):
            col = FG if r['file'] == 'brute' else (SER[0] if 'B1' in r['file'] else SER[1])
            if k in ('poses', 'plan'):
                v, lo, hi = r[k]
                ax.plot([lo, hi], [yy, yy], color=col, lw=2.2, alpha=.55, solid_capstyle='round')
                ax.plot([v], [yy], 'o', color=col, ms=5)
            else:
                v = r[k]
                ax.plot([v[0], v[-1]], [yy, yy], color=col, lw=2.2, alpha=.55, solid_capstyle='round')
                ax.plot([v[len(v) // 2]], [yy], 'o', color=col, ms=5)
                if k == 'nn' and r['file'] != 'brute':
                    ax.plot([r['dist']], [yy], 'o', mfc='none', mec=col, ms=6, mew=1.2)
        ax.set_title(title, loc='left', fontsize=10.5)
        if sc == 'log':
            ax.set_xscale('log')
            ax.set_xticks([0.25, 0.5, 1, 2, 4, 8], ['0.25', '0.5', '1', '2', '4', '8'])
        xgrid(ax)
        ax.set_xlabel('relative to brute force')
    axes[0].set_yticks(ys, [r['label'] for r in rows], fontsize=8.5)
    axes[0].invert_yaxis()
    for i, yy in ((1, ys[1]), (6, ys[6])):
        axes[0].text(0.02, yy - 0.75, 'Morton, buffer B = 1' if i == 1 else 'Morton, buffer B = 32 (newest 32 nodes scanned exactly)',
                     transform=axes[0].get_yaxis_transform(), fontsize=9, fontweight='bold', va='center',
                     color=SER[0] if i == 1 else SER[1])
    axes[0].plot([], [], 'o', color=FG2, ms=5, label='time (median of 3 runs; bar = min-max)')
    axes[0].plot([], [], 'o', mfc='none', mec=FG2, ms=6, label='distance computations only')
    fig.legend(*axes[0].get_legend_handles_labels(), loc='lower left', ncol=2, fontsize=9, frameon=False,
               bbox_to_anchor=(0.01, 0.0))
    share = rows[0]['nn_share'] * 100
    fig.suptitle(f'Closed-loop Morton NN vs brute force on the host (699 problems x 10 seeds; NN = {share:.2f}% of brute-force planning time)',
                 x=0.01, ha='left', fontweight='bold', color=FG)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    save(fig, out, 'fig13_nn_time')


def fig_nn_simple(exptime, out):
    rows = {r['file']: r for r in nn_time_table(exptime, nboot=200)}
    pick = [('mortonB1_m1_W1', 'm=1, W=1'), ('mortonB1_m1_W4', 'm=1, W=4'), ('mortonB1_m2_W4', 'm=2, W=4'),
            ('mortonB1_m4_W8', 'm=4, W=8')]
    series = [('NN distance computations', lambda r: r['dist'], SER[0]),
              ('CHECKs', lambda r: r['ck'][0], SER[1]),
              ('Total planning time (measured)', lambda r: r['plan'][0], FG2)]
    fig, ax = plt.subplots(figsize=(10, 4.6))
    w = 0.26
    for j, (name, get, col) in enumerate(series):
        for i, (f, _) in enumerate(pick):
            v = get(rows[f])
            x = i + (j - 1) * w
            ax.bar(x, v, width=w - 0.03, color=col, label=name if i == 0 else None)
            ax.text(x, v + 0.04, f'{v:.2f}', ha='center', fontsize=9, color=FG2)
    ax.axhline(1, color=THR, lw=1.6, ls=(0, (5, 4)), label='Brute force = 1')
    ax.set_xticks(range(len(pick)), [l for _, l in pick])
    ax.set_ylabel('Relative to brute force')
    ax.set_ylim(0, 3.0)
    ax.set_title('Morton NN can cut NN work, but adds CHECKs, and total planning time always goes up', loc='left')
    ygrid(ax)
    ax.legend(fontsize=9, loc='upper center', ncol=2)
    save(fig, out, 'fig14_nn_simple')


def fig_nn_all(exptime, out):
    """Closed-loop B = 1, every (m, W) of the hit-rate heat map: NN work, CHECKs, measured planning time."""
    load = lambda f, r: A.parse(os.path.join(exptime, f'{f}_r{r}.csv'), 'RUN', A.RUN_COLS)
    base = [load('brute', r) for r in (1, 2, 3)]
    tot = lambda R: sum(x['cyc_total'] for x in R)
    Ms, Ws = (1, 2, 4, 8), (1, 2, 4, 8, 16, 32)
    val = {}
    for m in Ms:
        for W in Ws:
            runs = [load(f'mortonB1_m{m}_W{W}', r) for r in (1, 2, 3)]
            plan = sorted(tot(R) / tot(B) for R, B in zip(runs, base))[1]
            val[m, W] = (sum(x['nn_evals'] for x in runs[0]) / sum(x['nn_evals'] for x in base[0]),
                         sum(x['checks'] for x in runs[0]) / sum(x['checks'] for x in base[0]), plan)
    series = [('NN distance computations', SER[0]), ('CHECKs', SER[1]), ('Total planning time (measured)', FG2)]
    fig, axes = plt.subplots(1, 4, figsize=(14, 4.8), sharey=True)
    w = 0.27
    for ax, m in zip(axes, Ms):
        for j, (name, col) in enumerate(series):
            for i, W in enumerate(Ws):
                v = val[m, W][j]
                x = i + (j - 1) * w
                ax.bar(x, v, width=w - 0.03, color=col, label=name if (m == 1 and i == 0) else None)
                ax.text(x, v * 1.06, f'{v:.2f}', ha='center', va='bottom', rotation=90, fontsize=7, color=FG2)
        ax.axhline(1, color=THR, lw=1.4, ls=(0, (5, 4)), label='Brute force = 1' if m == 1 else None)
        ax.set_yscale('log')
        ax.set_xticks(range(len(Ws)), [str(W) for W in Ws])
        ax.set_xlabel('W')
        ax.set_title(f'm = {m}', loc='left')
        ygrid(ax)
    axes[0].set_yticks([0.25, 0.5, 1, 2, 4, 8], ['0.25', '0.5', '1', '2', '4', '8'])
    axes[0].set_ylim(0.25, 12)
    axes[0].set_ylabel('Relative to brute force (log)')
    fig.legend(*axes[0].get_legend_handles_labels(), loc='lower center', ncol=4, fontsize=9, frameon=False)
    fig.suptitle('Every (m, W) of the hit-rate map, closed loop (B = 1): less NN work means more CHECKs; '
                 'total planning time is always above brute force', x=0.01, ha='left', fontweight='bold', color=FG)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    save(fig, out, 'fig15_nn_all')
    return val


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', required=True)
    ap.add_argument('--exp', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs, preps, per, tot = board(a.board)
    if len(runs) != 699 or len(preps) != 699:
        sys.exit(f'expected 699 RUN/PREP lines after the last RRTC START, got {len(runs)}/{len(preps)}')
    fig_stage_time(per, a.out)
    fig_stage_share(per, a.out)
    fig_fs(tot, a.out)
    fig_heat(a.exp, a.out)
    fig_cdf(a.exp, a.out)
    fig_tree(a.exp, a.out)
    fig_nn_tradeoff(a.exp, a.out)



# ---------------------------------------------------------------- design-independent figures (expFlat)
def _flat(expflat):
    import glob as _g
    agg, per = {}, {}
    for p in sorted(_g.glob(os.path.join(expflat, 'flat_s?.csv'))):
        tags, scene = {}, None
        for l in open(p):
            v = l.strip().split(',')
            if v[0] == 'PREP':
                scene = int(v[1])
            elif v[0].startswith('FS'):
                tags[v[0]] = [int(x) for x in v[1:]]
        per[scene] = tags
        for k, x in tags.items():
            agg[k] = x if k not in agg else [a + b for a, b in zip(agg[k], x)]
    return agg, per


def fig_voxel_work(expflat, out):
    agg, per = _flat(expflat)
    fig, ax = plt.subplots(figsize=(9, 3.9))
    ys = list(range(7))[::-1]
    for s, y in zip(range(7), ys):
        Q, C, H, cells, ne = per[s]['FSQ'][:5]
        n = Q - C
        ax.barh(y, ne / n, color=SER[0], height=0.6, edgecolor='white', linewidth=1.2,
                label='non-empty cells' if s == 0 else None)
        ax.barh(y, (cells - ne) / n, left=ne / n, color='#C9D1D8', height=0.6, edgecolor='white', linewidth=1.2,
                label='empty cells' if s == 0 else None)
        ax.text(cells / n + 0.08, y, f'{cells / n:.2f} cells, {(1 - ne / cells) * 100:.0f}% empty', va='center',
                fontsize=9, color=FG2)
    ax.set_yticks(ys, SCENES)
    Q, C, H, cells, ne = agg['FSQ'][:5]
    ax.set_xlabel('Voxel cells per robot-sphere query (after the global-AABB cull)')
    ax.set_title(f'Cells walked per query: {(1 - ne / cells) * 100:.1f}% are empty '
                 f'({(Q - C):,} queries, every checked pose x 59 spheres)', loc='left')
    ax.set_xlim(0, 10.5)
    xgrid(ax)
    ax.legend(loc='lower right', fontsize=9)
    save(fig, out, 'fig7_voxel_work')


def fig_param_speedup(expflat, out):
    agg, _ = _flat(expflat)
    Q, C, H, cells, ne = agg['FSQ'][:5]
    betas = [0.125 * 2 ** (k / 8) for k in range(0, 49)]   # 0.125 .. 8
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for i, k in enumerate((2, 4, 8)):
        b = agg['FSBLK'][i]
        for gamma, ls in ((1, '-'), (2, (0, (5, 3)))):
            ys = [cells / (beta * b + gamma * ne) for beta in betas]
            ax.plot(betas, ys, color=SER[i], lw=2, ls=ls,
                    label=f'{k}x{k}x{k} blocks, gamma={gamma}')
    ax.axhline(cells / ne, color=FG2, lw=1.2, ls=(0, (2, 3)))
    ax.text(0.13, cells / ne * 1.04, f'skip-empty bound {cells / ne:.1f}x', fontsize=9, color=FG2)
    ax.axhline(1, color=THR, lw=1.2, ls=(0, (5, 4)))
    ax.text(0.13, 1.05, 'no gain', fontsize=9, color=THR)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xticks([0.125, 0.25, 0.5, 1, 2, 4, 8], ['1/8', '1/4', '1/2', '1', '2', '4', '8'])
    ax.set_yticks([0.5, 1, 2, 4, 8, 12], ['0.5', '1', '2', '4', '8', '12'])
    ax.set_xlabel('beta = cost of one block-bitmap read / cost of one cell lookup (log)')
    ax.set_ylabel('voxel-lookup speed-up (log)')
    ax.set_title('Morton block bitmap vs cell-by-cell table walk: speed-up = cells / (beta*blocks + gamma*non-empty)',
                 loc='left')
    ygrid(ax)
    ax.set_ylim(0.2, 16)
    ax.legend(fontsize=8.5, ncol=2, loc='lower left')
    save(fig, out, 'fig8_param_speedup')


def fig_first_hit(expflat, out):
    agg, _ = _flat(expflat)
    qh, hb = agg['FSOQH'], agg['FSOHB']
    orders = ['pose-major', 'sphere-major', 'Morton']
    fig, ax = plt.subplots(figsize=(9, 3.9))
    w = 0.26
    for o in range(3):
        xs = [i + (o - 1) * w for i in range(3)]
        vals = [qh[i * 3 + o] / hb[i * 3 + o] for i in range(3)]
        ax.bar(xs, vals, width=w - 0.02, color=SER[o], label=orders[o])
        for x, v in zip(xs, vals):
            ax.text(x, v + 8, f'{v:.0f}', ha='center', fontsize=9, color=FG2)
    ax.set_xticks(range(3), ['8 poses', '16 poses', '32 poses'])
    ax.set_xlabel('Batch size')
    ax.set_ylabel('Queries before the first collision')
    ax.set_title('Early exit: queries processed before the first colliding query (batches with a collision)',
                 loc='left')
    ygrid(ax)
    ax.legend(fontsize=9)
    save(fig, out, 'fig9_first_hit')


def fig_lru(expflat, out):
    agg, _ = _flat(expflat)
    sizes = [2 ** j for j in range(0, 11)]
    orders = ['pose-major', 'sphere-major', 'Morton']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, pref, title in zip(axes, ('FSRDF', 'FSRDE'), ('Whole batches (no early exit)', 'With early exit')):
        for o in range(3):
            h = agg[f'{pref}{o}']
            tot = sum(h)
            ys = [sum(h[j + 1:]) / tot * 100 for j in range(len(sizes))]
            ax.plot(sizes, ys, color=SER[o], lw=2, marker='o', ms=4, label=f'{orders[o]} ({tot / 1e6:.0f} M refs)')
        ax.set_xscale('log', base=2)
        ax.set_xticks(sizes, [str(s) for s in sizes])
        ax.set_xlabel('LRU cache size (voxel entries, log)')
        ax.set_title(title, loc='left')
        ygrid(ax)
        ax.legend(fontsize=8.5)
    axes[0].set_ylabel('Miss rate (%)')
    fig.suptitle('Voxel reuse inside 32-pose batches: LRU miss rate for any cache size (from reuse distances)',
                 x=0.01, ha='left', fontweight='bold', color=FG)
    fig.tight_layout()
    save(fig, out, 'fig10_lru')


def fig_nn_scale(expflat, out):
    import glob as _g
    rows = {}
    for p in _g.glob(os.path.join(expflat, 'nnscale_*.csv')):
        for l in open(p):
            v = l.strip().split(',')
            k = (v[1], int(v[2]), int(v[3]), int(v[4]))
            r = rows.setdefault(k, [0] * 65)
            rows[k] = [a + int(b) for a, b in zip(r, v[7:])]
    fig, ax = plt.subplots(figsize=(9, 4.4))
    Ns = sorted({k[1] for k in rows})
    ax.plot(Ns, Ns, color=FG, lw=1.6, ls=(0, (4, 3)), label='brute force (N distances)')
    for i, dist in enumerate(('uniform', 'walk', 'clustered')):
        xs, ys = [], []
        for N in Ns:
            best = None
            for (d, n, m, W), h in rows.items():
                if d == dist and n == N and sum(h[:3]) / sum(h) >= 0.95:
                    c = m * min(2 * W, N)
                    best = c if best is None or c < best else best
            if best:
                xs.append(N)
                ys.append(best)
        lab = {'uniform': 'Morton, uniform tree', 'walk': 'Morton, random-walk tree (RRT-like)',
               'clustered': 'Morton, clustered tree'}[dist]
        ax.plot(xs, ys, color=SER[i], lw=2, marker='o', ms=5, label=lab)
        for N in Ns:
            if N not in xs and N > max(xs):
                ax.plot([N], [6e4 / 1.6 ** i], marker='x', ms=9, mew=2.2, color=SER[i], ls='none')
    ax.axvline(292, color=THR, lw=1.3, ls=(0, (5, 4)))
    ax.text(292 * 1.08, 30, 'largest tree in\nMotionBenchMaker (292)', fontsize=9, color=THR)
    ax.plot([], [], marker='x', ms=8, mew=2, color=FG2, ls='none', label='no tested (m, W) reaches 95%')
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlabel('Tree size N (nodes, log)')
    ax.set_ylabel('Distances per query (log)')
    ax.set_title('Cheapest Morton (m, W) with 95% of answers within 1.09x of the true nearest, vs brute force',
                 loc='left')
    ygrid(ax)
    ax.legend(fontsize=9, loc='lower right')
    save(fig, out, 'fig11_nn_scale')


def main_flat():
    ap = argparse.ArgumentParser()
    ap.add_argument('--expflat', required=True)
    ap.add_argument('--exp')
    ap.add_argument('--exptime')
    ap.add_argument('--out', required=True)
    a, _ = ap.parse_known_args()
    os.makedirs(a.out, exist_ok=True)
    fig_voxel_work(a.expflat, a.out)
    fig_param_speedup(a.expflat, a.out)
    fig_first_hit(a.expflat, a.out)
    fig_lru(a.expflat, a.out)
    fig_nn_scale(a.expflat, a.out)
    if a.exp:
        fig_nn_tradeoff(a.exp, a.out)
    if a.exptime:
        fig_nn_time(a.exptime, a.out)
        fig_nn_simple(a.exptime, a.out)


if __name__ == '__main__':
    if '--expflat' in sys.argv:
        main_flat()
    else:
        main()
