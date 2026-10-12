import sys, math, cmath
sys.path.insert(0,'.')
from eq import *
def spacing_scan(vals, lo=0.05, hi=0.8, step=0.0005):
    best=(0,0,0)
    d=lo
    while d<hi:
        re=sum(math.cos(2*math.pi*v/d) for v in vals); im=sum(math.sin(2*math.pi*v/d) for v in vals)
        sc=math.hypot(re,im)/len(vals)
        if sc>best[0]: best=(sc,d,math.atan2(im,re))
        d+=step
    return best
def analyse(z, label, rot_extra=0.0):
    z=[v*cmath.exp(1j*rot_extra) for v in z]
    xs=[v.real for v in z]; ys=[v.imag for v in z]
    bx=spacing_scan(xs); by=spacing_scan(ys)
    d=(bx[1]+by[1])/2
    # offsets: phase -> position of lattice sites: sites at (phase/2pi + n)*d
    ox=bx[2]/(2*math.pi)*bx[1]; oy=by[2]/(2*math.pi)*by[1]
    return dict(label=label,sx=bx,sy=by,d=d,ox=ox,oy=oy)
def count_sites(z, d, ox, oy, frac=0.05):
    from collections import Counter
    c=Counter(); err=0.0
    for v in z:
        i=round((v.real-ox)/d); j=round((v.imag-oy)/d)
        c[(i,j)]+=1
        err+=((v.real-ox-i*d)**2+(v.imag-oy-j*d)**2)
    n=len(z); rms=math.sqrt(err/n)/d
    return c,rms
if __name__=='__main__':
    fn=sys.argv[1]; t0=float(sys.argv[2]); dur=float(sys.argv[3])
    buf=open(fn,'rb').read()
    zb=bb(buf,t0,dur); ph,_=timing_phase(zb); x=to_half_T(zb,ph)
    y,w=cma(x,L=32,mu=float(sys.argv[4]) if len(sys.argv)>4 else 2e-3,passes=int(sys.argv[5]) if len(sys.argv)>5 else 3)
    y=y[len(y)//3:]
    dz,_=derotate(y)
    best=None
    for name,extra in (('plain',0.0),('+45deg',math.pi/4)):
        r=analyse(dz,name,extra)
        zz=[v*cmath.exp(1j*extra) for v in dz]
        c,rms=count_sites(zz,r['d'],r['ox'],r['oy'])
        print(f"{name:7s} spacing x={r['sx'][1]:.4f} (peak {r['sx'][0]:.2f}) y={r['sy'][1]:.4f} (peak {r['sy'][0]:.2f})  rms dist to site {rms:.3f} d; sites hit {len(c)}")
