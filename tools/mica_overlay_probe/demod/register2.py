import random, itertools
from register import masks, solve, evalreg
from sites import sites
from cmp_map import CONV_ISO, G
from descramble import ung, S
pk,c,gi='rec2_z.pkl',0.48,0
ij=sites(S+pk,c); g=G[gi]; Z=[ung(*g(i,j)) for i,j in ij]
P,flip=CONV_ISO[0]; d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]
Q=[(d[n]-d[n-1])%4 for n in range(1,len(d))][:700]
M=masks(8*len(Q)+8)
res=[]
for kA,kB in itertools.permutations(range(8),2):
    bits=[]
    for m,q in enumerate(Q): bits+=[(8*m+kA,q&1),(8*m+kB,q>>1)]
    random.seed(3); best=0
    for t in range(150):
        eq=random.sample(bits[:400],30)
        reg=solve([(M[p],v) for p,v in eq])
        if reg is None: continue
        ok=sum(1 for p,v in bits[:400] if evalreg(reg,M[p])==v); best=max(best,ok)
    res.append((best/400,kA,kB))
res.sort(reverse=True)
print('best (bit0 pos, bit1 pos):',[(round(a,3),b,c) for a,b,c in res[:6]])
