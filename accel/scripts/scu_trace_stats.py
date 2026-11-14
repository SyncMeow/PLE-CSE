#!/usr/bin/env python3
# SCU time distribution from a tb_core trace (+scu_trace=<file>): cycles, wait, collide, hit outer pairs.
#   scu_trace_stats.py <trace.txt>
import sys, collections
rows=[l.split() for l in open(sys.argv[1])]
n=len(rows); d=[int(r[0]) for r in rows]; w=[int(r[1]) for r in rows]; c=[int(r[2]) for r in rows]
hm=[int(r[3],16) for r in rows]; ng=[bin(x).count('1') for x in hm]
print('checks',n,'collide',sum(c),'mean_cycles %.1f'%(sum(d)/n),'mean_wait %.1f'%(sum(w)/n),'min',min(d),'max',max(d))
sd=sorted(d)
print('pct', {p: sd[min(n-1,int(n*p/100))] for p in (10,25,50,75,90,99)})
h=collections.Counter(min(x//10,15) for x in d)
print('hist10', [(k*10, '%.1f%%'%(100*h[k]/n)) for k in sorted(h)])
g=collections.Counter(ng); print('hit_groups', [(k,'%.1f%%'%(100*g[k]/n)) for k in sorted(g)])
f=[0]*21
for x in hm:
    for i in range(21):
        if x>>i&1: f[i]+=1
print('group_freq%', [round(100*x/n,1) for x in f])
