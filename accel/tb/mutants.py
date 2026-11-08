#!/usr/bin/env python3
"""Write planted-bug copies of PLE modules; each must make its unit test fail.

Usage: mutants.py <rtl/ple dir> <out dir>
fku_m4 / fku_m5 (written by the Makefile): ple_fku generated with LAT = 2 (consumer reads before the producer writes) /
with sin-cos readiness one cycle early (SC_LAT = 2).
"""
import sys

MUTANTS = {
    'smp_m1': ('ple_smp.sv', 'u[j] + delta[j][k];', "u[j] + delta[j][k] + 32'(k == 3);",
               'Halton step off by one when digit 3 is the lowest free digit'),
    'smp_m2': ('ple_smp.sv', 'assign delta[j][k] = D;', "assign delta[j][k] = D + 32'(k == 15);",
               'wrong step-table entry at digit 15 (never reached by sampling; only the table check sees it)'),
    'nnu_m4': ('ple_nnu.sv', "if (s4_last) done <= 1'b1;", '/* done never raised */',
               'done never pulses (the watchdog must end the run)'),
    'rkg_m1': ('ple_rkg.sv', 'if (r < 0) begin', 'if (r <= 0) begin', 'row step borrows when the remainder is 0'),
    'rkg_m2': ('ple_rkg.sv', "if (r >= signed'(RW'(den))) begin", "if (r > signed'(RW'(den))) begin",
               'in-row step misses the carry when the remainder equals den'),
    'rkg_m3': ('ple_rkg.sv', "34'h0FF_FFFF", "34'h0", 'n rounded down instead of up'),
    'rkg_m4': ('ple_rkg.sv', "(nz ? ~dq[j] : ~dq[j] + 1'b1)", "(~dq[j] + 1'b1)",
               'negative floor division truncates toward zero'),
    'rkg_m5': ('ple_rkg.sv', 'S_ROW:  if (p_ready && p_last) state <= S_IDLE;',
               'S_ROW:  if (start) state <= S_DIV; else if (p_ready && p_last) state <= S_IDLE;',
               'start while busy restarts the edge'),
    'alu_m1': ('ple_isqrt.sv', 'if (rem >= res + bitv) begin', 'if (rem > res + bitv) begin',
               'isqrt misses the exact-square case'),
    'alu_m2': ('ple_div7.sv', "(nz ? ~qm[j] : ~qm[j] + 1'b1)", "(~qm[j] + 1'b1)",
               'negative floor division truncates toward zero'),
    'alu_m3': ('ple_div7.sv', 'if (t >= {1\'b0, bh}) begin', 'if (t > {1\'b0, bh}) begin',
               'division misses the exact-subtract case'),
    'fku_m1': ('ple_fku.sv', "a2_x[0] <= sin_v[0]; a2_c[0] <= -18'sd7864;",
               "a2_x[0] <= sin_v[0]; a2_c[0] <= -18'sd7865;", 'one FK coefficient off by one LSB'),
    'fku_m2': ('ple_fku.sv', "(2*W+CW)'(a2_x[m] * a2_c[m]) >>> F;", "(2*W+CW)'(a2_x[m] * a2_c[m]) / (1 <<< F);",
               'one lane truncates products toward zero instead of floor'),
    'fku_m3': ('ple_fku.sv', "sin_v[sj2] <= W'(sb2) + W'(sp2 >>> TS);", "sin_v[sj2] <= W'(sb2) + W'(sp2 / (1 <<< TS));",
               'sin interpolation truncates toward zero'),
    'fku_m6': ('ple_fku.sv', " + 18'sd19497;", " + 18'sd19498;", 'w_min offset off by one (all outputs)', 'all'),
    'fku_m7': ('ple_fku.sv', "6'd11: begin a0_x[0] <= -r[23];", "6'd11: begin a0_x[0] <= r[23];",
               'negation alias lost inside one V*V product'),
    'fku_m8': ('ple_fku.sv', "sin_v[sj2] <= W'(sb2) + W'(sp2 >>> TS);", "sin_v[sj2] <= W'(cb2) + W'(cp2 >>> TS);",
               'sin and cos swapped'),
    'fku_m9': ('ple_fku.sv', "a0_k <= 21'sd87294;", "a0_k <= 21'sd87295;", 'one folded constant off by one'),
    'nnu_m1': ('ple_nnu.sv', 'rb_d2 < d2)', 'rb_d2 <= d2)', 'running minimum keeps the later row on ties'),
    'nnu_m2': ('ple_nnu.sv', 'cd[2*i+1] < cd[2*i])', 'cd[2*i+1] <= cd[2*i])', 'lane reduction keeps the higher lane on ties'),
    'nnu_m3': ('ple_nnu.sv', "(32'(s1_row) * K + l) < 32'(nh)", "(32'(s1_row) * K + l) <= 32'(nh)",
               'one stale node past the end of the tree is considered'),
}


def main():
    src, out = sys.argv[1], sys.argv[2]
    for name, entry in MUTANTS.items():
        f, a, b, what = entry[:4]
        every = len(entry) > 4 and entry[4] == 'all'
        s = open(f'{src}/{f}').read()
        if (s.count(a) == 0) if every else (s.count(a) != 1):
            sys.exit(f'{name}: pattern found {s.count(a)} times in {f}')
        open(f'{out}/{name}.sv', 'w').write(s.replace(a, b))
        print(f'{name}: {what}')


if __name__ == '__main__':
    main()
