"""Undo the V.32bis differential encoder and test the coded dibit stream against a scrambled-ones source.
Scrambler (V.32bis 5.2): out_n = in_n ^ out_{n-a} ^ out_{n-23}, a=5 (answerer GPA) or 18 (caller GPC).
For in = all ones, raising the polynomial to the 8th power gives out_n ^ out_{n-8a} ^ out_{n-184} = 1, i.e. in
symbol steps z_m ^ z_{m-a} ^ z_{m-23} = const for every bit position k of the 8-bit symbol group -- testable
on Q1,Q2 alone (positions 0,1), whatever the six uncoded bits are."""
import sys
from sites import sites
from cmp_map import CONV_ISO, G
S='/private/tmp/claude-501/-Users-scottcryan-v90modem/8fb1ce38-0fb9-4688-a35e-483c585993ad/scratchpad/'
def ung(i,j):
    p=(i+j)&1; ip=i-p; z1=ip&1; i2=ip-z1; j2=j-z1
    return (p,z1,((i2+j2)//2)&1)
def frac(bits,a,b=23):
    n=len(bits); same=sum(1 for m in range(b,n) if (bits[m]^bits[m-a]^bits[m-b])==1)
    return same/(n-b)
CONV=[[0,2,3,1],[4,7,5,6],[1,3,2,0],[7,4,6,5],[2,0,1,3],[6,5,7,4],[3,1,0,2],[5,6,4,7]]
def best_init(Z,P,flip):
    best=(0,0)
    for init in range(8):
        s=init; ok=0
        for z0,z1,z2 in Z: ok+=((s&1)^flip)==z0; s=CONV[s][P[z0][2*z2+z1]]
        best=max(best,(ok,init))
    return best
rows=[]
for pk,c,gi in (('rec2_z.pkl',0.48,0),('rec1_z.pkl',0.49,1)):
    ij=sites(S+pk,c); g=G[gi]
    Z=[ung(*g(i,j)) for i,j in ij]
    for P,flip in CONV_ISO:
        d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]
        streams={'d (no diff undo)':d,
                 'Q=d[n]-d[n-1]':[(d[n]-d[n-1])%4 for n in range(1,len(d))],
                 'Q=d[n-1]-d[n]':[(d[n-1]-d[n])%4 for n in range(1,len(d))]}
        for name,q in streams.items():
            for bname,f in (('bit0',lambda v:v&1),('bit1',lambda v:v>>1),('b0^b1',lambda v:(v&1)^(v>>1))):
                bits=[f(v) for v in q]
                for a in (5,18):
                    fr=frac(bits,a); rows.append((max(fr,1-fr),pk,P[0],name,bname,a,fr))
rows.sort(reverse=True)
print('best matches of z_m^z_{m-a}^z_{m-23} == const  (1.0 = scrambled marks; 0.5 = random)')
for r in rows[:12]: print('  %.3f  %s P0=%s %-16s %-6s a=%d  (frac ones=%.3f)'%r)
import collections
print('median over all candidates: %.3f'%sorted(r[0] for r in rows)[len(rows)//2])
