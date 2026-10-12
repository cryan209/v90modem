import sys, math, pickle, random
sys.path.insert(0,'.')
from collections import Counter
z=pickle.load(open('terbo_z.pkl','rb'))
random.seed(1)
def inv_g(r, a, b):
    # solve g(rl)=r with g(rl)=rl*(1+a*rl^2+b*rl^4), monotone for a,b>=0
    lo,hi=0.0,r
    for _ in range(40):
        mid=(lo+hi)/2
        if mid*(1+a*mid*mid+b*mid**4) < r: lo=mid
        else: hi=mid
    return (lo+hi)/2
def unwarp(pts,a,b):
    out=[]
    for v in pts:
        r=abs(v)
        if r<1e-9: out.append(v); continue
        rl=inv_g(r,a,b); out.append(v*(rl/r))
    return out
def score_at(pts,d):
    s=0; off=[]
    for vals in ([v.real for v in pts],[v.imag for v in pts]):
        re=sum(math.cos(2*math.pi*u/d) for u in vals); im=sum(math.sin(2*math.pi*u/d) for u in vals)
        s+=math.hypot(re,im)/len(vals); off.append(math.atan2(im,re)/(2*math.pi)*d)
    return s/2,off
def best_d(pts,lo=0.095,hi=0.108,n=27):
    best=(0,)
    for k in range(n):
        d=lo+(hi-lo)*k/(n-1); sc,off=score_at(pts,d)
        if sc>best[0]: best=(sc,d,off[0],off[1])
    return best
if __name__=='__main__':
    sub=random.sample(z,5000)
    res=[]
    for a in [0.0,0.04,0.08,0.12,0.16,0.20,0.25,0.30]:
        for b in [0.0,0.03,0.06,0.10,0.15]:
            u=unwarp(sub,a,b); sc,d,ox,oy=best_d(u); res.append((sc,a,b,d))
    res.sort(reverse=True)
    for sc,a,b,d in res[:8]: print('score %.3f  a=%.2f b=%.2f  d0=%.4f'%(sc,a,b,d))
    print('no warp (a=b=0): score %.3f'%[r[0] for r in res if r[1]==0 and r[2]==0][0])
