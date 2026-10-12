"""Model transmitter + receiver for the recovered MICA V.32terbo chain, run on NON-CONSTANT data.
TX: random bytes -> GPA self-synchronising scrambler (8 bits/symbol: Q1,Q2, then six word bits, LSB first)
    -> differential encoder (d = d_prev + Q mod 4) -> V.32bis 8-state coder (y0 = bit0 of state)
    -> coset (y0, d) + 6-bit word -> point (energy rank, pointmap_gen) -> V.34 warp, kappa 0.4 -> noise.
RX: unwarp -> nearest point -> (coset, word) -> d, y0 -> Q = d[n]-d[n-1] -> 8-bit group -> descramble.
Also shows that scrambled MARKS cannot tell a self-synchronising scrambler from an additive one."""
import random, math, sys
from collections import defaultdict
from pointmap_gen import gen, REC, coset
from cmp_map import CONV_ISO
CONV=[[0,2,3,1],[4,7,5,6],[1,3,2,0],[7,4,6,5],[2,0,1,3],[6,5,7,4],[3,1,0,2],[5,6,4,7]]
P,FLIP=CONV_ISO[0]
def ung(i,j):
    p=(i+j)&1; ip=i-p; z1=ip&1; i2=ip-z1; j2=j-z1
    return (p,z1,((i2+j2)//2)&1)
C,RM=REC['rec2_z.pkl']; TAB=gen(C,RM)
BYCW={}                       # (coset, word) -> site
for p,w in TAB.items(): BYCW[(coset(*p),w)]=p
CO_OF={}                      # ung triple -> coset
for p in TAB:
    t=ung(*p); assert CO_OF.setdefault(t,coset(*p))==coset(*p)
PTS=list(TAB)
KAPPA=0.4; E=sum((i-C[0])**2+(j-C[1])**2 for i,j in PTS)/512; CW=KAPPA/E
phi=lambda z:1+z/6+z*z/120
def warp(x): return x*phi(CW*abs(x)**2)
def unwarp(y):
    r=abs(y)
    if r<1e-9: return y
    lo,hi=0.0,r
    for _ in range(40):
        m=(lo+hi)/2
        if m*phi(CW*m*m)<r: lo=m
        else: hi=m
    return y*((lo+hi)/2/r)
SITE_CW={p:(coset(*p),TAB[p]) for p in PTS}
IDEAL=[(complex(i-C[0],j-C[1]),p) for (i,j),p in zip(PTS,PTS)]
def nearest(x):
    i=round(x.real+C[0]); j=round(x.imag+C[1])
    best=None
    for di in (-1,0,1):
        for dj in (-1,0,1):
            p=(i+di,j+dj)
            if p in TAB:
                d=abs(complex(p[0]-C[0],p[1]-C[1])-x)
                if best is None or d<best[0]: best=(d,p)
    return best[1] if best else None
def scramble_selfsync(bits,reg=None):
    out=list(reg) if reg else [0]*23
    for b in bits: out.append(b^out[-5]^out[-23])
    return out[23:]
def descramble_selfsync(bits):
    return [bits[n]^(bits[n-5] if n>=5 else 0)^(bits[n-23] if n>=23 else 0) for n in range(len(bits))]
def tx(sbits):
    s=0; d=0; syms=[]
    for m in range(len(sbits)//8):
        g=sbits[8*m:8*m+8]; q=g[0]|(g[1]<<1); word=sum(g[2+k]<<k for k in range(6))
        d=(d+q)%4; z0=(s&1)^FLIP
        z1=z2=None
        for a in (0,1):
            for b in (0,1):
                if P[z0][2*b+a]==d: z1,z2=a,b
        cos=CO_OF[(z0,z1,z2)]; syms.append(BYCW[(cos,word)])
        s=CONV[s][d]
    return syms
def rx(zs):
    ung_of={coset(*p):ung(*p) for p in PTS}
    prev=0; bits=[]; bad=0
    for z in zs:
        p=nearest(unwarp(z))
        if p is None: bad+=1; p=PTS[0]
        cos,word=SITE_CW[p]; z0,z1,z2=ung_of[cos]; d=P[z0][2*z2+z1]
        q=(d-prev)%4; prev=d
        bits+=[q&1,q>>1]+[(word>>k)&1 for k in range(6)]
    return bits,bad
if __name__=='__main__':
    random.seed(11); n=20000
    for label,src in (('random data',[random.getrandbits(1) for _ in range(8*n)]),
                      ('text-like data',[((b>>k)&1) for b in (random.choice(b'the quick brown fox jumps over ' ) for _ in range(n)) for k in range(8)]),
                      ('marks',[1]*(8*n))):
        sb=scramble_selfsync(src); syms=tx(sb)
        for sigma in (0.0,0.088,0.15):
            rng=random.Random(5)
            zs=[warp(complex(p[0]-C[0],p[1]-C[1]))+complex(rng.gauss(0,sigma),rng.gauss(0,sigma)) for p in syms]
            rb,bad=rx(zs); out=descramble_selfsync(rb)
            # skip the first 23 bits while the descrambler fills; count bit errors
            err=sum(1 for a,b in zip(out[184:],src[184:]) if a!=b)
            rawerr=sum(1 for a,b in zip(rb,sb) if a!=b)
            print('%-15s sigma/axis %.3f cell: symbols %d, received scrambled-bit errors %d (%.3f%%), descrambled data errors %d of %d'%(label,sigma,len(syms),rawerr,100*rawerr/len(sb),err,len(src)-184))
    # marks cannot distinguish scrambler types
    reg=[random.getrandbits(1) for _ in range(23)]
    k=reg[:]
    for _ in range(4000): k.append(k[-5]^k[-23])
    add=[1^x for x in k[23:]]                   # additive: out = in ^ keystream
    ss=scramble_selfsync([1]*4000,[1^x for x in reg])   # self-sync with matching initial state
    ok=all(add[n]==(1^k[23+n]) for n in range(4000))
    rel=lambda o:all((o[m]^o[m-5]^o[m-23])==1 for m in range(23,len(o)))
    print('marks: additive-scrambler output satisfies the in=1 recursion:',rel(add),'; self-sync output does:',rel(ss))
