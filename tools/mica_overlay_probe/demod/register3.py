"""Register solve on the clean region (violations are exactly 0 after the first ~1500 / 4000 symbols)."""
import random, itertools, pickle
from register import masks, solve, evalreg
from sites import sites
from cmp_map import CONV_ISO, G
from descramble import ung, S
START={'rec2_z.pkl':2000,'rec1_z.pkl':4500}
def getQ(pk,c,gi):
    ij=sites(S+pk,c); g=G[gi]; Z=[ung(*g(i,j)) for i,j in ij]
    P,flip=CONV_ISO[0]; d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]
    return ij,Z,d,[(d[n]-d[n-1])%4 for n in range(1,len(d))]
if __name__=='__main__':
    for pk,c,gi in (('rec2_z.pkl',0.48,0),('rec1_z.pkl',0.49,1)):
        ij,Z,d,Q=getQ(pk,c,gi); s0=START[pk]; Qc=Q[s0:]
        M=masks(8*len(Qc)+8); res=[]
        for kA,kB in itertools.permutations(range(8),2):
            bits=[]
            for m,q in enumerate(Qc[:300]): bits+=[(8*m+kA,q&1),(8*m+kB,q>>1)]
            random.seed(3); best=(0,0)
            for t in range(40):
                reg=solve([(M[p],v) for p,v in random.sample(bits,28)])
                if reg is None: continue
                ok=sum(1 for p,v in bits if evalreg(reg,M[p])==v)
                if ok>best[0]: best=(ok,reg)
            res.append((best[0]/len(bits),kA,kB,best[1]))
        res.sort(reverse=True)
        print(pk,'best (bit0 pos, bit1 pos, agreement):',[(round(a,3),b,c2) for a,b,c2,_ in res[:3]])
