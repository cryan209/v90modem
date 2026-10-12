import sys, math, cmath
sys.path.insert(0,'.')
from dd import *
fn=sys.argv[1]; t=float(sys.argv[2]); d0=float(sys.argv[3]); rho=math.radians(float(sys.argv[4]))
x,w,phi,nu,n0,i0,L=setup(fn,t,6.0)
nsym=int(sys.argv[5]) if len(sys.argv)>5 else 14000
z,w2,rr,rms=run_dd(x,w,phi,nu,n0,i0,L,nsym,d0,rho,passes=3)
z,_,_,_=run_dd(x,w2,phi,nu,n0,i0,L,nsym,d0,rr,passes=1,adapt=False,track=True)
print(fn,'symbols',len(z),'rotation %.1f deg'%math.degrees(rr))
# refine d by joint scan on all points
best=(0,)
for k in range(-40,41):
    dd=d0*(1+k*0.0015)
    sc=0
    for vals in ([v.real for v in z],[v.imag for v in z]):
        re=sum(math.cos(2*math.pi*v/dd) for v in vals); im=sum(math.sin(2*math.pi*v/dd) for v in vals)
        sc+=math.hypot(re,im)/len(vals)
    if sc>best[0]: best=(sc,dd)
dd=best[1]; print('refined d %.4f (score sum %.3f)'%(dd,best[0]))
P=sum(abs(v)**2 for v in z)/len(z); print('mean power %.3f  => 6P/d^2+1 = %.0f'%(P,6*P/dd**2+1))
# row profile of the imaginary coordinate in units of d, offset 0.5: row index j=round(y/d-0.5)
from collections import Counter
rowc=Counter(round(v.imag/dd-0.5) for v in z); colc=Counter(round(v.real/dd-0.5) for v in z)
def prof(c): 
    ks=sorted(c); return [(k,c[k]) for k in ks if abs(k)<=16]
print('row counts (index: count):',' '.join(f'{k}:{n}' for k,n in prof(rowc)))
print('col counts (index: count):',' '.join(f'{k}:{n}' for k,n in prof(colc)))
