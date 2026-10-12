"""Solve the V.32bis answerer scrambler (GPA: out_n = in_n ^ out_{n-5} ^ out_{n-23}, in = all ones) for its register
from the Q1,Q2 bits alone (RANSAC over GF(2) with 5 percent bit errors); then predict all eight bits of every
symbol and learn where the six uncoded bits put the point inside its coset."""
import sys, random
from collections import Counter, defaultdict
from sites import sites
from cmp_map import CONV_ISO, G
from descramble import ung, S
N0=23
def masks(nbits):
    """o_n as a 24-bit mask over (o_{-23}..o_{-1}, 1): index j<23 means o_{j-23}, bit 23 = constant"""
    m=[1<<j for j in range(N0)]+[]
    out=list(m)                                   # o_{-23..-1}
    for n in range(nbits):
        out.append(out[-5]^out[-23]^(1<<23))      # in = 1
    return out[N0:]
def solve(eqs):
    """eqs: list of (mask24, value) -> register (23-bit int) or None; gaussian elimination"""
    rows=[]
    for m,v in eqs:
        r=m&((1<<23)-1)
        c=(m>>23)&1
        rows.append([r,v^c])
    piv={}
    for r,v in rows:
        for b,(pr,pv) in piv.items():
            if r>>b&1: r^=pr; v^=pv
        if r:
            b=r.bit_length()-1
            for b2,(pr,pv) in list(piv.items()):
                if pr>>b&1: piv[b2]=(pr^r,pv^v)
            piv[b]=(r,v)
    if len(piv)<23: return None
    reg=0
    for b,(r,v) in piv.items():
        if v: reg|=1<<b
    return reg
def evalreg(reg,m):
    return (bin(m&((1<<23)-1)&reg).count('1')+((m>>23)&1))&1
def run(pk,c,gi,order):
    ij=sites(S+pk,c); g=G[gi]; Z=[ung(*g(i,j)) for i,j in ij]
    P,flip=CONV_ISO[0]
    d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]
    Q=[(d[n]-d[n-1])%4 for n in range(1,len(d))]
    bits=[]                                       # observed (position, value)
    for m,q in enumerate(Q):
        b=(q&1,q>>1) if order==0 else (q>>1,q&1)
        bits+= [(8*m,b[0]),(8*m+1,b[1])]
    nsym=len(Q); M=masks(8*nsym+8)
    random.seed(7); best=(0,None)
    sub=bits[:600]                                # solve on the start, then verify everywhere
    for t in range(400):
        eq=random.sample(sub,30)
        reg=solve([(M[p],v) for p,v in eq])
        if reg is None: continue
        ok=sum(1 for p,v in sub if evalreg(reg,M[p])==v)
        if ok>best[0]: best=(ok,reg)
    ok,reg=best
    allok=sum(1 for p,v in bits if evalreg(reg,M[p])==v)
    return Z,Q,d,reg,M,allok/len(bits),ok/len(sub)
if __name__=='__main__':
    for pk,c,gi in (('rec2_z.pkl',0.48,0),('rec1_z.pkl',0.49,1)):
        for order in (0,1):
            Z,Q,d,reg,M,fa,fs=run(pk,c,gi,order)
            print(f'{pk} bit order {order}: register {reg:#07x}  agrees on first 600 observed bits {100*fs:.1f}%, whole record {100*fa:.1f}%')
