import sys, math, pickle, random
sys.path.insert(0,'.')
from unwarp import best_d, score_at, z
from collections import Counter
random.seed(3)
def phi(zeta, exact=False):
    if exact:
        return 1.0 if zeta<1e-12 else math.sinh(math.sqrt(zeta))/math.sqrt(zeta)
    return 1+zeta/6+zeta*zeta/120
def inv(r_obs, c, exact=False):
    # solve r*phi(c*r^2)=r_obs for r (monotone)
    lo,hi=0.0,r_obs
    for _ in range(40):
        mid=(lo+hi)/2
        if mid*phi(c*mid*mid,exact) < r_obs: lo=mid
        else: hi=mid
    return (lo+hi)/2
def unwarp_c(pts,c,exact=False):
    out=[]
    for v in pts:
        r=abs(v)
        out.append(v if r<1e-9 else v*(inv(r,c,exact)/r))
    return out
if __name__=='__main__':
    sub=random.sample(z,6000)
    print('one-parameter V.34 family  zeta = c r^2,  phi = 1+zeta/6+zeta^2/120')
    res=[]
    for k in range(0,25):
        c=0.1+0.04*k
        u=unwarp_c(sub,c); sc,d,ox,oy=best_d(u,0.095,0.108,27)
        E=sum(abs(v)**2 for v in u)/len(u)
        res.append((sc,c,d,E))
    for sc,c,d,E in res: 
        mark='  <==' if sc==max(r[0] for r in res) else ''
        print('c=%.2f  score %.3f  d0=%.4f  E(unwarped)=%.3f  V.34 would need c = 0.3125/E = %.3f%s'%(c,sc,d,E,0.3125/E,mark))
