#!/usr/bin/env python3
"""FKU modulo scheduling (docs/PLE_ARCH_STUDY.md): units needed when a new pose starts every P cycles.
Usage: python3 -I fku_modulo.py ../../sw_rrtc/tools/gen_fx_fku_rtl.py ../../sw_rrtc/fx/fx_fk_gen.c"""
# modulo scheduling: a new pose starts every P cycles; slot (unit, t mod P) may be used once
import importlib.util, sys
spec = importlib.util.spec_from_file_location('g', sys.argv[1]); g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
F, TS, tabs, stmts, outs = g.parse(sys.argv[2])
nodes, out_res = g.lower(stmts, outs, F)
LAT, SC = g.LAT, g.SC_LAT
deps = {n: set() for n in nodes}; scs = {}
for n, d in nodes.items():
    srcs = [s for m in d['mul'] for s in (m[2:] if m[0] == 'VV' else [m[2]])] + [s for _, s in d['add']]
    deps[n] = {s for s in srcs if isinstance(s, str)}; scs[n] = [s for s in srcs if isinstance(s, tuple)]
users = {n: set() for n in nodes}
for n, ds in deps.items():
    for x in ds: users[x].add(n)
prio = {}
def p(n):
    if n not in prio: prio[n] = LAT + max([p(u) for u in users[n]], default=0)
    return prio[n]
vv = {n: any(m[0] == 'VV' for m in nodes[n]['mul']) for n in nodes}
def sched(U, L, P):
    used = set(); issue = {}; left = set(nodes); t = 0
    while left and t < 2000:
        ready = sorted([n for n in left if all(x in issue and issue[x] + LAT <= t for x in deps[n]) and all(t >= s[1] + SC for s in scs[n])], key=lambda n: (-p(n), n))
        fu = [u for u in range(U) if (u, t % P) not in used]; fl = [u for u in range(U, U + L) if (u, t % P) not in used]
        for n in ready:
            if vv[n]:
                if not fu: continue
                u = fu.pop(0)
            elif fl: u = fl.pop(0)
            elif fu: u = fu.pop(0)
            else: continue
            used.add((u, t % P)); issue[n] = t; left.discard(n)
        t += 1
    if left: return None
    # value lifetime: register-cycles per pose -> average live values = sum(life)/P
    life = 0
    for n in nodes:
        end = max([issue[u] for u in users[n]], default=issue[n] + LAT)
        life += max(0, end - (issue[n] + LAT)) + 1
    return max(issue.values()) + LAT, life
print('P units(U+L) mults latency in_flight live_values(avg)')
for P in [52, 40, 32, 26, 20, 16, 13, 10, 8, 6, 4, 2, 1]:
    best = None
    for tot in range(2, 300):
        for U in sorted({2, 3, 4, 6, 8, min(tot, 12), min(tot, 34)}):
            if U > tot or U * P < 34: continue
            r = sched(U, tot - U, P)
            if r: best = (tot, U, r); break
        if best: break
    tot, U, (lat, life) = best
    print(P, f'{tot} (U={U})', 3 * tot, lat, -(-lat // P), round(life / P))
