import sys, math, cmath
sys.path.insert(0,'.')
from data_eq import *
def f2d(pts, dlo=0.05, dhi=0.40, nd=70, nang=30):
    best=[]
    N=len(pts)
    for a in range(nang):
        ang=a*(math.pi/2)/nang
        ca,sa=math.cos(ang),math.sin(ang)
        # project onto axis at this angle (x') and (y') so lattice rows align when ang=rho
        xs=[v.real*ca+v.imag*sa for v in pts]; ys=[-v.real*sa+v.imag*ca for v in pts]
        for k in range(nd):
            d=dlo*(dhi/dlo)**(k/(nd-1))
            re=sum(math.cos(2*math.pi*v/d) for v in xs); im=sum(math.sin(2*math.pi*v/d) for v in xs)
            sx=math.hypot(re,im)/N
            re=sum(math.cos(2*math.pi*v/d) for v in ys); im=sum(math.sin(2*math.pi*v/d) for v in ys)
            sy=math.hypot(re,im)/N
            best.append((min(sx,sy),d,ang,sx,sy))
    best.sort(reverse=True); return best[:6]
if __name__=='__main__':
    fn=sys.argv[1]; t=float(sys.argv[2]); n=int(sys.argv[3]); skip=int(sys.argv[4]) if len(sys.argv)>4 else 0
    y,mse=equalize_data(fn,t,3.0)
    pts=y[skip:skip+n]
    print(fn,'block',skip,'..',skip+n)
    for sc,d,ang,sx,sy in f2d(pts): print('  score(min x,y) %.3f  d=%.4f  angle=%.1f deg  (x %.3f y %.3f)'%(sc,d,math.degrees(ang),sx,sy))
