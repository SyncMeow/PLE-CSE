#!/usr/bin/env python3
"""Generate Platform/accel/rtl/ple/ple_fku.sv from the golden fixed-point FK (fx/fx_fk_gen.c).

The RTL must equal the golden model bit for bit, so this reads the golden C (constants, tables, every
MUL with its floor) instead of the float source. Steps:
  1. parse fx_fk_base; SSA-rename reused v[] slots; terms: K, +-V, MUL(c, V), MUL(V, V), sin/cos
  2. alias copies / negations / sin-cos reads, fold constants, drop values the outputs do not need
  3. simulate the result in Python against the parsed golden semantics on random poses (bit-exact)
     and measure value ranges -> register width
  4. list-schedule statements on U link units (3 multipliers, V*V allowed) and L sphere lanes
     (3 multipliers, constant * V only); each statement: read (t) -> multiply (t+1) -> add/write (t+2)
  5. emit SystemVerilog: dedicated registers per value, per-slot operand muxes over the schedule
Usage: gen_fx_fku_rtl.py <fx_fk_gen.c> <out.sv> [U] [L] [LAT] [SC_LAT] [W]  (last three: mutants only)
"""
import math
import random
import re
import sys

N_SPH = 70
WS_FRAC = 14
W_MIN = math.floor(-1.19 * 16384.0)  # as fx_pose.c fx_pose_init (-19497)
LAT = 3                            # issue -> result usable by an issue LAT cycles later (argv[5] overrides: mutant only)
SC_LAT = 3                         # sin/cos of joint j usable from cycle j + SC_LAT (argv[6] overrides: mutant only)


def floor_shift(x, s):
    return x >> s                  # Python >> on ints is floor, like the C arithmetic shift


# ---------------------------------------------------------------- 1. parse
def parse(path):
    src = open(path).read()
    F = int(re.search(r'#define F (\d+)', src).group(1))
    TS = int(re.search(r'#define TS (\d+)', src).group(1))
    tabs = {}
    for name in ('sin_tab', 'cos_tab'):
        m = re.search(name + r'\[\d+\] = \{([^}]*)\}', src)
        tabs[name] = [int(t.strip()[:-2]) for t in m.group(1).split(',')]
    body = src[src.index('void fx_fk_base'):src.index('void fx_fk_ref')]
    stmts = []
    ver = {}

    def use(tok):                  # v[3] -> SSA name of the latest version
        arr, idx = tok[0], int(re.search(r'\d+', tok).group(0))
        key = (arr, idx)
        if key not in ver:
            raise SystemExit(f'use before def: {tok}')
        return f'{arr}{idx}_{ver[key]}'

    for line in body.splitlines():
        m = re.match(r'\s+([vy])\[(\d+)\] = (.*);$', line)
        if not m:
            continue
        arr, idx, rhs = m.group(1), int(m.group(2)), m.group(3)
        terms = []
        for t in rhs.split(' + '):
            t = t.strip()
            if re.fullmatch(r'-?\d+LL', t):
                terms.append(('K', int(t[:-2])))
            elif re.fullmatch(r'\(-[vy]\[\d+\]\)', t):
                terms.append(('V', -1, use(t[2:-1])))
            elif re.fullmatch(r'[vy]\[\d+\]', t):
                terms.append(('V', 1, use(t)))
            elif mm := re.fullmatch(r'MUL\((-?\d+)LL, ([vy]\[\d+\])\)', t):
                terms.append(('CV', int(mm.group(1)), use(mm.group(2))))
            elif mm := re.fullmatch(r'MUL\(([vy]\[\d+\]), ([vy]\[\d+\])\)', t):
                terms.append(('VV', use(mm.group(1)), use(mm.group(2))))
            elif mm := re.fullmatch(r'(sin|cos)_(\d)', t):
                terms.append(('S', mm.group(1), int(mm.group(2))))
            else:
                raise SystemExit(f'unknown term: {t!r} in {line.strip()}')
        key = (arr, idx)
        ver[key] = ver.get(key, -1) + 1
        stmts.append((f'{arr}{idx}_{ver[key]}', terms))
    outs = {}                      # (sphere, axis) -> final SSA name
    for s in range(N_SPH):
        for k in range(3):
            outs[(s, k)] = f'y{4 * s + k}_{ver[("y", 4 * s + k)]}'
    return F, TS, tabs, stmts, outs


# ---------------------------------------------------------------- golden semantics (reference)
def lut(tab, q, TS):
    u = q + 32768
    k, fr = u >> TS, u & ((1 << TS) - 1)
    return tab[k] + floor_shift((tab[k + 1] - tab[k]) * fr, TS)


def eval_golden(stmts, tabs, F, TS, q):
    env = {}
    for name, terms in stmts:
        acc = 0
        for t in terms:
            if t[0] == 'K': acc += t[1]
            elif t[0] == 'V': acc += t[1] * env[t[2]]
            elif t[0] == 'CV': acc += floor_shift(t[1] * env[t[2]], F)
            elif t[0] == 'VV': acc += floor_shift(env[t[1]] * env[t[2]], F)
            elif t[0] == 'S': acc += lut(tabs['sin_tab' if t[1] == 'sin' else 'cos_tab'], q[t[2]], TS)
        env[name] = acc
    return env


# ---------------------------------------------------------------- 2. alias / fold / dead code
def lower(stmts, outs, F):
    """Operands resolve to ('C', value) or ('V', sign, base); base = node name or ('sin'|'cos', joint)."""
    alias, nodes = {}, {}

    def res(n):
        return alias.get(n, ('V', 1, n))

    for name, terms in stmts:
        if len(terms) == 1 and terms[0][0] == 'S':
            alias[name] = ('V', 1, (terms[0][1], terms[0][2]))
            continue
        mul, add, k = [], [], 0
        for t in terms:
            if t[0] == 'K':
                k += t[1]
            elif t[0] == 'V':
                r = res(t[2])
                if r[0] == 'C': k += t[1] * r[1]
                else: add.append((t[1] * r[1], r[2]))
            elif t[0] == 'CV':
                r = res(t[2])
                if r[0] == 'C': k += floor_shift(t[1] * r[1], F)
                else: mul.append(('CV', t[1] * r[1], r[2]))
            elif t[0] == 'VV':
                ra, rb = res(t[1]), res(t[2])
                if ra[0] == 'C' and rb[0] == 'C':
                    k += floor_shift(ra[1] * rb[1], F)
                elif ra[0] == 'C' or rb[0] == 'C':
                    c, o = (ra, rb) if ra[0] == 'C' else (rb, ra)
                    mul.append(('CV', c[1] * o[1], o[2]))
                else:
                    mul.append(('VV', ra[1] * rb[1], ra[2], rb[2]))
        if not mul and not add:
            alias[name] = ('C', k)
        elif not mul and len(add) == 1 and k == 0:
            alias[name] = ('V', add[0][0], add[0][1])
        else:
            nodes[name] = dict(mul=mul, add=add, k=k)
    out_res = {key: res(n) for key, n in outs.items()}
    live, stack = set(), [r[2] for r in out_res.values() if r[0] == 'V' and isinstance(r[2], str)]
    while stack:
        n = stack.pop()
        if n in live or n not in nodes: continue
        live.add(n)
        for m in nodes[n]['mul']:
            for src in (m[2:] if m[0] == 'VV' else [m[2]]):
                if isinstance(src, str): stack.append(src)
        for _, src in nodes[n]['add']:
            if isinstance(src, str): stack.append(src)
    nodes = {n: d for n, d in nodes.items() if n in live}
    for n, d in nodes.items():
        assert len(d['mul']) <= 3 and len(d['add']) <= 1, (n, d)
    return nodes, out_res


def eval_lowered(nodes, order, out_res, tabs, F, TS, q):
    env = {}
    sc = {('sin', j): lut(tabs['sin_tab'], q[j], TS) for j in range(7)}
    sc.update({('cos', j): lut(tabs['cos_tab'], q[j], TS) for j in range(7)})

    def val(src):
        return sc[src] if isinstance(src, tuple) else env[src]
    for n in order:
        d = nodes[n]
        acc = d['k']
        for m in d['mul']:
            if m[0] == 'CV': acc += floor_shift(m[1] * val(m[2]), F)
            else: acc += floor_shift(m[1] * val(m[2]) * val(m[3]), F)
        for sg, src in d['add']:
            acc += sg * val(src)
        env[n] = acc
    out = {}
    for key, r in out_res.items():
        out[key] = r[1] if r[0] == 'C' else r[1] * val(r[2])
    return out, env


# ---------------------------------------------------------------- 4. schedule
def schedule(nodes, U, L):
    deps = {n: set() for n in nodes}
    for n, d in nodes.items():
        for m in d['mul']:
            for src in (m[2:] if m[0] == 'VV' else [m[2]]):
                if isinstance(src, str): deps[n].add(src)
        for _, src in d['add']:
            if isinstance(src, str): deps[n].add(src)
    users = {n: set() for n in nodes}
    for n, ds in deps.items():
        for x in ds: users[x].add(n)
    prio = {}

    def p(n):
        if n not in prio: prio[n] = LAT + max([p(u) for u in users[n]], default=0)
        return prio[n]
    for n in nodes: p(n)
    needs_vv = {n: any(m[0] == 'VV' for m in nodes[n]['mul']) for n in nodes}
    sc_srcs = {}
    for n, d in nodes.items():
        srcs = [src for m in d['mul'] for src in (m[2:] if m[0] == 'VV' else [m[2]])] + [src for _, src in d['add']]
        sc_srcs[n] = [src for src in srcs if isinstance(src, tuple)]
    # sin/cos of joint j become readable at cycle j + LAT
    issue, unit = {}, {}
    t = 0
    left = set(nodes)
    while left:
        ready = [n for n in left if all(x in issue and issue[x] + LAT <= t for x in deps[n])
                 and all(t >= src[1] + SC_LAT for src in sc_srcs[n])]
        ready.sort(key=lambda n: (-p(n), n))
        free_u, free_l = list(range(U)), list(range(U, U + L))
        for n in ready:
            if needs_vv[n]:
                if not free_u: continue
                unit[n] = free_u.pop(0)
            else:
                if free_l: unit[n] = free_l.pop(0)
                elif free_u: unit[n] = free_u.pop(0)
                else: continue
            issue[n] = t
            left.discard(n)
        t += 1
    t_last = max(issue.values()) + LAT          # first cycle all results are written
    return issue, unit, t_last, deps


# ---------------------------------------------------------------- 5. emit
def lit(v, w):
    return f"{w}'sd{v}" if v >= 0 else f"-{w}'sd{-v}"


def emit(path, F, TS, tabs, nodes, out_res, issue, unit, t_last, U, L, W, CW):
    names = sorted(nodes, key=lambda n: issue[n])
    nid = {n: i for i, n in enumerate(names)}
    NU = U + L
    TW = max(1, (t_last + 4).bit_length())
    o = []
    o.append('// GENERATED by Platform/sw_rrtc/tools/gen_fx_fku_rtl.py from fx/fx_fk_gen.c. Do not edit.')
    o.append('// FKU: fixed-point Panda FK, bit-exact with the golden fx_fk_ws(): joint angles -> 70 sphere centres')
    o.append(f'// in workspace coordinates (s18, LSB 2^-14 m, origin w_min). Static schedule: {U} link units (V*V)')
    o.append(f'// + {L} sphere lanes (c*V), 3 multipliers each; {len(nodes)} statements, results complete at cycle')
    o.append(f'// {t_last}; done rises {t_last + 2} clock edges after the start edge. q must stay stable until done.')
    o.append('// start is accepted only while busy = 0; busy falls one cycle after done, so a start in the done cycle')
    o.append('// is dropped. Simulation checks every write for register-width overflow ($fatal).')
    o.append('module ple_fku')
    o.append('  import ple_pkg::*;')
    o.append('(')
    o.append('    input  logic                         clk,')
    o.append('    input  logic                         rst_n,')
    o.append('    input  logic                         start,          // ignored while busy')
    o.append('    input  conf_t                        q,')
    o.append('    output logic                         busy,')
    o.append('    output logic                         done,           // one-cycle pulse, c valid from then on')
    o.append(f'    output logic [{N_SPH - 1}:0][2:0][17:0]       c               // [sphere][axis], signed workspace coords')
    o.append(');')
    o.append('')
    o.append(f'  localparam int F  = {F};')
    o.append(f'  localparam int W  = {W};                              // value register width (measured range + margin)')
    o.append(f'  localparam int TS = {TS};')
    o.append(f'  localparam int CW = {CW};                             // constant coefficient width')
    o.append('')
    o.append(f'  logic [{TW - 1}:0] t;')
    o.append('  logic run;')
    o.append('  always_ff @(posedge clk) begin')
    o.append('    if (!rst_n) begin')
    o.append("      run <= 1'b0; busy <= 1'b0; done <= 1'b0;")
    o.append('    end else begin')
    o.append("      done <= 1'b0;")
    o.append("      if (start && !busy) begin run <= 1'b1; busy <= 1'b1; t <= '0; end")
    o.append('      else if (run) begin')
    o.append(f"        t <= t + 1'b1;")
    o.append(f"        if (t == {TW}'d{t_last}) begin run <= 1'b0; done <= 1'b1; end")
    o.append('      end')
    o.append("      if (done) busy <= 1'b0;")
    o.append('    end')
    o.append('  end')
    o.append('')
    # sin/cos: one joint per cycle t = 0..6; ROM word {t[k], t[k+1] - t[k]}
    sw = max(abs(v) for v in tabs['sin_tab'] + tabs['cos_tab']).bit_length() + 1
    dw = max(abs(a - b) for tb in tabs.values() for a, b in zip(tb[1:], tb)).bit_length() + 1
    o.append(f'  // sin / cos: table + linear interpolation (golden lut()); ROM word = {{t[k] ({sw}b), t[k+1]-t[k] ({dw}b)}}')
    o.append(f'  localparam int SW = {sw}, DW = {dw};')
    for tn in ('sin', 'cos'):
        tb = tabs[tn + '_tab']
        o.append(f'  (* rom_style = "block" *) logic [SW+DW-1:0] {tn}_rom [{len(tb) - 1}];')
        o.append('  initial begin')
        for i in range(len(tb) - 1):
            o.append(f"    {tn}_rom[{i}] = {{{lit(tb[i], sw)}, {lit(tb[i + 1] - tb[i], dw)}}};")
        o.append('  end')
    o.append('  logic [2:0] sj, sj1, sj2;')
    o.append('  logic sv1, sv2;')
    o.append('  logic [SW+DW-1:0] srom, crom;')
    o.append('  logic [TS-1:0] fr1;')
    o.append('  logic signed [W-1:0] sin_v [7], cos_v [7];')
    o.append('  logic signed [SW-1:0] sb2, cb2;')
    o.append('  logic signed [DW+TS:0] sp2, cp2;')
    o.append('  always_ff @(posedge clk) begin')
    o.append("    logic [15:0] u;")
    o.append("    u = 16'(17'(signed'(q[sj])) + 17'd32768);           // q + 32768 in 0..65535")
    o.append("    srom <= sin_rom[u[15:TS]];")
    o.append("    crom <= cos_rom[u[15:TS]];")
    o.append("    fr1  <= u[TS-1:0];")
    o.append("    sj1  <= sj;")
    o.append(f"    sv1  <= run && t < {TW}'d7;")
    o.append("    sb2  <= signed'(srom[SW+DW-1:DW]);")
    o.append("    cb2  <= signed'(crom[SW+DW-1:DW]);")
    o.append("    sp2  <= (DW+TS+1)'(signed'(srom[DW-1:0]) * signed'({1'b0, fr1}));")
    o.append("    cp2  <= (DW+TS+1)'(signed'(crom[DW-1:0]) * signed'({1'b0, fr1}));")
    o.append("    sj2  <= sj1;")
    o.append("    sv2  <= sv1;")
    o.append("    if (sv2) begin")
    o.append("      sin_v[sj2] <= W'(sb2) + W'(sp2 >>> TS);")
    o.append("      cos_v[sj2] <= W'(cb2) + W'(cp2 >>> TS);")
    o.append("    end")
    o.append('  end')
    o.append(f"  assign sj = (t < {TW}'d7) ? t[2:0] : 3'd0;               // keep the joint index in range")
    o.append('')
    # value registers
    o.append(f'  logic signed [W-1:0] r [{len(names)}];          // one register per scheduled value')
    o.append('')

    def src_expr(s):
        if isinstance(s, tuple):
            return f'{s[0]}_v[{s[1]}]'
        return f'r[{nid[s]}]'

    # per unit: operand registers for 3 multipliers + additive
    for u in range(NU):
        mine = [n for n in names if unit[n] == u]
        kind = 'link unit (V*V)' if u < U else 'sphere lane (c*V)'
        o.append(f'  // ---- unit {u}: {kind}, {len(mine)} statements')
        o.append(f'  logic signed [W-1:0] a{u}_x [3], a{u}_y [3], a{u}_add;')
        o.append(f'  logic signed [CW-1:0] a{u}_c [3];')
        o.append(f'  logic [2:0] a{u}_vv;                               // multiplier m is V*V (else c*V)')
        o.append(f'  logic signed [W-1:0] a{u}_k;')
        o.append(f'  logic signed [2*W+CW-1:0] b{u}_p [3];')
        o.append(f'  logic signed [W-1:0] b{u}_add, b{u}_k;')
        o.append('  always_ff @(posedge clk) begin')
        o.append(f"    a{u}_vv <= '0; a{u}_add <= '0; a{u}_k <= '0;")
        o.append(f"    for (int m = 0; m < 3; ++m) begin a{u}_x[m] <= '0; a{u}_y[m] <= '0; a{u}_c[m] <= '0; end")
        o.append('    if (run) begin')
        o.append('      unique case (t)')
        for n in mine:
            d = nodes[n]
            lines = []
            for mi, m in enumerate(d['mul']):
                if m[0] == 'CV':
                    lines.append(f"a{u}_x[{mi}] <= {src_expr(m[2])}; a{u}_c[{mi}] <= {lit(m[1], CW)};")
                else:
                    neg = '-' if m[1] < 0 else ''
                    lines.append(f"a{u}_x[{mi}] <= {neg}{src_expr(m[2])}; a{u}_y[{mi}] <= {src_expr(m[3])}; a{u}_vv[{mi}] <= 1'b1;")
            for s, b in d['add']:
                lines.append(f"a{u}_add <= {'-' if s < 0 else ''}{src_expr(b)};")
            if d['k']:
                assert abs(d['k']) < (1 << (W - 1)), d['k']
                lines.append(f"a{u}_k <= {lit(d['k'], W)};")
            o.append(f"        {TW}'d{issue[n]}: begin " + ' '.join(lines) + ' end')
        o.append('        default: ;')
        o.append('      endcase')
        o.append('    end')
        o.append('  end')
        o.append('  always_ff @(posedge clk) begin                      // multiply, floor each product')
        o.append(f'    b{u}_add <= a{u}_add; b{u}_k <= a{u}_k;')
        o.append('    for (int m = 0; m < 3; ++m)')
        o.append(f'      b{u}_p[m] <= a{u}_vv[m] ? (2*W+CW)\'(a{u}_x[m] * a{u}_y[m]) >>> F : (2*W+CW)\'(a{u}_x[m] * a{u}_c[m]) >>> F;')
        o.append('  end')
        o.append('')
    # write stage: every value is written by one unit at one fixed cycle (issue + 2): no tags, no muxes
    for u in range(NU):
        o.append(f"  logic signed [W-1:0] s{u};")
        o.append(f"  assign s{u} = W'(b{u}_p[0] + b{u}_p[1] + b{u}_p[2]) + b{u}_add + b{u}_k;")
    o.append('`ifndef SYNTHESIS')
    o.append('  // full-width sums must fit in W bits (the register width is chosen from sampled ranges)')
    for u in range(NU):
        o.append(f"  logic signed [2*W+CW+2:0] w{u};")
        o.append(f"  assign w{u} = (2*W+CW+3)'(b{u}_p[0]) + (2*W+CW+3)'(b{u}_p[1]) + (2*W+CW+3)'(b{u}_p[2]) + (2*W+CW+3)'(b{u}_add) + (2*W+CW+3)'(b{u}_k);")
    o.append('  always_ff @(posedge clk) begin')
    o.append('    if (run) begin')
    for n in names:
        o.append(f"      if (t == {TW}'d{issue[n] + 2} && w{unit[n]} != (2*W+CW+3)'(s{unit[n]})) $fatal(1, \"ple_fku: value {nid[n]} overflows W bits\");")
    o.append('    end')
    o.append('  end')
    o.append('`endif')
    o.append('  always_ff @(posedge clk) begin                        // write each value at its scheduled cycle')
    o.append('    if (run) begin')
    for n in names:
        o.append(f"      if (t == {TW}'d{issue[n] + 2}) r[{nid[n]}] <= s{unit[n]};")
    o.append('    end')
    o.append('  end')
    o.append('')
    # outputs
    o.append('  // workspace coordinates: (value >> (F - 14)) - w_min, sampled when all results are written')
    o.append('  always_ff @(posedge clk) begin')
    o.append(f"    if (run && t == {TW}'d{t_last}) begin")
    for (sph, k), r in sorted(out_res.items()):
        if r[0] == 'C':
            o.append(f"      c[{sph}][{k}] <= {lit(floor_shift(r[1], F - WS_FRAC) - W_MIN, 18)};")
        else:
            neg = '-' if r[1] < 0 else ''
            o.append(f"      c[{sph}][{k}] <= 18'(({neg}{src_expr(r[2])}) >>> (F - {WS_FRAC})) + {lit(-W_MIN, 18)};")
    o.append('    end')
    o.append('  end')
    o.append('')
    o.append('endmodule')
    open(path, 'w').write('\n'.join(o) + '\n')


def main():
    src, out = sys.argv[1], sys.argv[2]
    U = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    L = int(sys.argv[4]) if len(sys.argv) > 4 else 4
    global LAT
    if len(sys.argv) > 5: LAT = int(sys.argv[5])        # a wrong LAT must make tb_fku fail (generator mutant)
    global SC_LAT
    if len(sys.argv) > 6: SC_LAT = int(sys.argv[6])     # a wrong sin/cos readiness must make tb_fku fail
    F, TS, tabs, stmts, outs = parse(src)
    nodes, out_res = lower(stmts, outs, F)
    issue, unit, t_last, deps = schedule(nodes, U, L)
    order = sorted(nodes, key=lambda n: issue[n])
    # 3. bit-exact check of the lowering against the parsed golden statements + value ranges
    rng = random.Random(1)
    vmax, mism = 0, 0
    lim = [(-12154, 24306), (-7507, 15012), (-12154, 24306), (-12868, 13225), (-12154, 24306), (-358, 16013), (-12154, 24306)]
    for it in range(4000):
        if it < 16:
            q = [a if (it >> j) & 1 else a + m - 1 for j, (a, m) in enumerate(lim)]
        else:
            q = [rng.randrange(-32768, 32768) if it % 10 == 0 else a + rng.randrange(m) for a, m in lim]
        gold = eval_golden(stmts, tabs, F, TS, q)
        got, env = eval_lowered(nodes, order, out_res, tabs, F, TS, q)
        for key, n in outs.items():
            if got[key] != gold[n]: mism += 1
        vmax = max([vmax] + [abs(v) for v in env.values()])
    # hill climbing from the worst sampled poses: push the largest |value| further before choosing W
    def worst(qv):
        return max(abs(v) for v in eval_lowered(nodes, order, out_res, tabs, F, TS, qv)[1].values())
    best_q, best_v = None, -1
    for it in range(200):
        qv = [rng.randrange(-32768, 32768) for _ in range(7)]
        v = worst(qv)
        if v > best_v: best_q, best_v = qv, v
    for it in range(3000):
        qv = list(best_q)
        j = rng.randrange(7)
        qv[j] = max(-32768, min(32767, qv[j] + rng.randrange(-4096, 4097)))
        v = worst(qv)
        if v >= best_v: best_q, best_v = qv, v
    vmax = max(vmax, best_v)
    W = vmax.bit_length() + 2
    if len(sys.argv) > 7: W = int(sys.argv[7])           # too-narrow W must trip the overflow check (mutant only)
    CW = max([abs(m[1]) for d in nodes.values() for m in d['mul'] if m[0] == 'CV'] + [1]).bit_length() + 1
    units = [sum(1 for n in nodes if unit[n] == u) for u in range(U + L)]
    print(f'FKUGEN statements={len(stmts)} scheduled={len(nodes)} U={U} L={L} per_unit={units} '
          f'results_at={t_last} max_abs={vmax} (hill-climb {best_v}) W={W} CW={CW} lowering_mismatches={mism}')
    if mism:
        raise SystemExit('lowering does not match the golden statements')
    emit(out, F, TS, tabs, nodes, out_res, issue, unit, t_last, U, L, W, CW)


if __name__ == '__main__':
    main()
