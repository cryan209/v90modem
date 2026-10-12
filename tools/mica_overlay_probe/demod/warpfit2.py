import math, sys
from warpfit import site_means
def fit2(means,C,alpha,beta):
    pts=[((i-C[0])+1j*(j-C[1]),m,w) for (i,j),(m,w) in means.items()]
    W=sum(w for _,_,w in pts)
    ph=lambda x:1+alpha*abs(x)**2+beta*abs(x)**4
    mu=sum(w*x*ph(x) for x,_,w in pts)/W; mz=sum(w*m for x,m,w in pts)/W
    num=sum(w*(x*ph(x)-mu).conjugate()*(m-mz) for x,m,w in pts)
    den=sum(w*abs(x*ph(x)-mu)**2 for x,m,w in pts)
    A=num/den; o=mz-A*mu
    return sum(w*abs(m-A*x*ph(x)-o)**2 for x,m,w in pts)/W/abs(A)**2, A, o
means=site_means('rec2_z.pkl',0.48); C=(0.5,0.5)
E=81.59
res=[]
for ia in range(0,41):
    a=ia*0.00005            # alpha: V.34 at kappa .4 would be c/6 = 0.00082
    for ib in range(-10,31):
        b=ib*0.0000004
        res.append((fit2(means,C,a,b)[0],a,b))
res.sort()
r,a,b=res[0]
print('free fit: alpha=%.6f beta=%.3e  residual %.3e'%(a,b,r))
print('  V.34 series shape requires beta = 0.3 alpha^2 = %.3e'%(0.3*a*a))
# profile: best beta for each alpha near optimum, and the chi-square curvature to get uncertainty on beta/alpha^2
import collections
bya=collections.defaultdict(lambda:(9,0))
for r_,a_,b_ in res:
    if r_<bya[a_][0]: bya[a_]=(r_,b_)
for a_ in sorted(bya)[::4]: print('  alpha %.5f best beta %.3e  resid %.3e  (beta/alpha^2 = %.2f)'%(a_,bya[a_][1],bya[a_][0],bya[a_][1]/a_**2 if a_ else 0))
