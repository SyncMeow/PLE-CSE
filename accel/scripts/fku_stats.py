#!/usr/bin/env python3
"""FKU schedule statistics (docs/PLE_ARCH_STUDY.md): nodes, multiplies, dependency depth, results_at per (U, L).
Usage: python3 -I fku_stats.py ../../sw_rrtc/tools/gen_fx_fku_rtl.py ../../sw_rrtc/fx/fx_fk_gen.c"""
import importlib.util, sys
spec = importlib.util.spec_from_file_location('g', sys.argv[1]); g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
F, TS, tabs, stmts, outs = g.parse(sys.argv[2])
nodes, out_res = g.lower(stmts, outs, F)
outn = {r[2] for r in out_res.values() if r[0] == 'V' and isinstance(r[2], str)}
vv = [n for n in nodes if any(m[0] == 'VV' for m in nodes[n]['mul'])]
nm = lambda ns: sum(len(nodes[n]['mul']) for n in ns)
print('nodes', len(nodes), 'mults', nm(nodes), 'VV-nodes', len(vv), 'VV-mults', sum(1 for n in nodes for m in nodes[n]['mul'] if m[0]=='VV'))
print('output nodes', len(outn), 'mults', nm(outn), '| non-output nodes', len(nodes)-len(outn), 'mults', nm(set(nodes)-outn))
print('outputs', len(out_res), 'const outputs', sum(1 for r in out_res.values() if r[0]=='C'), 'distinct V outputs', len(outn))
hist = {}
for n in nodes: hist[len(nodes[n]['mul'])] = hist.get(len(nodes[n]['mul']), 0) + 1
print('mults per node', sorted(hist.items()))
# dependency depth with unlimited units
issue, unit, t_last, deps = g.schedule(nodes, 999, 999)
print('unlimited units: results_at', t_last, 'peak nodes/cycle', max(sum(1 for n in issue if issue[n]==t) for t in range(t_last)))
lastlink = max(issue[n] for n in nodes if n not in outn)
print('unlimited: last non-output issue', lastlink, 'first output issue', min(issue[n] for n in outn))
print('U L results_at  link_done  mults')
for U, L in [(2,4),(2,6),(2,8),(2,12),(2,16),(3,4),(4,4),(6,0),(8,0),(4,8),(2,24),(2,34)]:
    issue, unit, t_last, deps = g.schedule(nodes, U, L)
    ll = max(issue[n] for n in nodes if n not in outn)
    print(U, L, t_last, ll + g.LAT, 3*(U+L))
issue, unit, t_last, deps = g.schedule(nodes, 2, 4)
for u in range(6):
    ts = sorted(issue[n] for n in nodes if unit[n]==u)
    print('unit', u, 'nodes', len(ts), 'first', ts[0], 'last', ts[-1], 'mults', nm([n for n in nodes if unit[n]==u]))
