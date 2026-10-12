import sys, math, pickle
sys.path.insert(0,'.')
from dd import *
def equalized(fn,d0,nsym=14000):
    x,w,phi,nu,n0,i0,L=setup(fn,11.5,6.0)
    z,w2,rr,rms=run_dd(x,w,phi,nu,n0,i0,L,nsym,d0,math.radians(12),passes=3)
    z,_,_,_=run_dd(x,w2,phi,nu,n0,i0,L,nsym,d0,rr,passes=1,adapt=False,track=True)
    return z
def fit_global(z,d0):
    best=(0,)
    for k in range(-60,61):
        dd=d0*(1+k*0.001); sc=0; off=[]
        for vals in ([v.real for v in z],[v.imag for v in z]):
            re=sum(math.cos(2*math.pi*v/dd) for v in vals); im=sum(math.sin(2*math.pi*v/dd) for v in vals)
            sc+=math.hypot(re,im)/len(vals); off.append(math.atan2(im,re)/(2*math.pi)*dd)
        if sc>best[0]: best=(sc/2,dd,off[0],off[1])
    return best
def local_d(vals, centre, width, d0, span=0.12, nf=121):
    # Hann-weighted 1-D lattice spacing estimate for the values inside [centre-width/2, centre+width/2]
    sel=[(v,0.5+0.5*math.cos(2*math.pi*(v-centre)/width)) for v in vals if abs(v-centre)<width/2]
    W=sum(w for _,w in sel)
    best=(0,0)
    for k in range(nf):
        dd=d0*(1-span+2*span*k/(nf-1))
        re=sum(w*math.cos(2*math.pi*v/dd) for v,w in sel); im=sum(w*math.sin(2*math.pi*v/dd) for v,w in sel)
        sc=math.hypot(re,im)/W
        if sc>best[0]: best=(sc,dd)
    return best[1],best[0],len(sel)
def report(fn,d0):
    z=equalized(fn,d0)
    sc,dd,ox,oy=fit_global(z,d0)
    print(fn,'global d=%.4f score %.2f'%(dd,sc))
    # pick points close to a row centre on the other axis
    def near(coord_other,off): 
        f=((coord_other-off)/dd)%1.0; return min(f,1-f)<0.18      # within 0.18 d of a lattice line
    rowpts=[v.real for v in z if near(v.imag,oy)]    # x-values on rows
    colpts=[v.imag for v in z if near(v.real,ox)]    # y-values on columns
    for lab,vals in (('x along rows',rowpts),('y along columns',colpts)):
        out=[]
        for c in (-1.05,-0.7,-0.35,0.0,0.35,0.7,1.05):
            d,s,n=local_d(vals,c,0.75,dd)
            out.append(f'{c:+.2f}:{d:.4f}({s:.2f},{n})')
        print('  ',lab,' '.join(out))
if __name__=='__main__':
    report(sys.argv[1],float(sys.argv[2]))
