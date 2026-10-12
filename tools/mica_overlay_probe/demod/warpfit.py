"""Warp function from the exactly known unwarped constellation: regress the mean received point of each of the
512 sites on its ideal position  x = (i-.5, j-.5).  z = A * x * phi(c |x|^2) + o,  A and o complex (equalizer
gain/rotation, residual offset); scan the shape parameter c."""
import math, pickle, cmath, sys
from collections import defaultdict
from sites import sites
from trellis import unwarp
from register3 import START
S='/private/tmp/claude-501/-Users-scottcryan-v90modem/8fb1ce38-0fb9-4688-a35e-483c585993ad/scratchpad/'
def site_means(pk,c0):
    z=pickle.load(open(S+pk,'rb')); ij=sites(S+pk,c0)
    s0=START[pk]; acc=defaultdict(lambda:[0j,0])
    for n in range(s0,len(z)):
        a=acc[ij[n]]; a[0]+=z[n]; a[1]+=1
    return {k:(v[0]/v[1],v[1]) for k,v in acc.items()}
def fit(means,C,phi,cvals):
    pts=[((i-C[0])+1j*(j-C[1]),m,w) for (i,j),(m,w) in means.items()]
    E=sum(abs(x)**2 for x,_,_ in pts)/len(pts)
    out=[]
    for c in cvals:
        # model basis u = x*phi(c|x|^2), z = A u + o : complex linear LS, weights w
        W=sum(w for _,_,w in pts)
        mu=sum(w*x*phi(c*abs(x)**2) for x,_,w in pts)/W; mz=sum(w*m for x,m,w in pts)/W
        num=sum(w*(x*phi(c*abs(x)**2)-mu).conjugate()*(m-mz) for x,m,w in pts)
        den=sum(w*abs(x*phi(c*abs(x)**2)-mu)**2 for x,m,w in pts)
        A=num/den; o=mz-A*mu
        res=sum(w*abs(m-A*x*phi(c*abs(x)**2)-o)**2 for x,m,w in pts)/W
        out.append((res/abs(A)**2,c,A,o))
    return E,out
ser=lambda z:1+z/6+z*z/120
def sinh_exact(z): return 1.0 if z<1e-12 else math.sinh(math.sqrt(z))/math.sqrt(z)
if __name__=='__main__':
    for pk,c0,C in (('rec2_z.pkl',0.48,(0.5,0.5)),):
        means=site_means(pk,c0)
        print(pk,'sites',len(means),' symbols per site (min/median)',min(v[1] for v in means.values()),sorted(v[1] for v in means.values())[256])
        for name,phi in (('series 1+z/6+z^2/120',ser),('exact sinh(sqrt z)/sqrt z',sinh_exact)):
            cv=[0.001*k for k in range(0,12000//5+1)]  # c per (cell^2)
            cv=[k*0.00002 for k in range(0,400)]
            E,out=fit(means,C,phi,cv)
            best=min(out)
            print(' ',name,': E(unit-cell^2)=%.2f  best c=%.5f /cell^2 -> kappa = c*E = %.4f ; normalised residual %.3e (no warp: %.3e)'%(E,best[1],best[1]*E,best[0],out[0][0]))
