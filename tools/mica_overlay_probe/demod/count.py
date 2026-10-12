import sys; sys.path.insert(0,'.')
from cons import *
from collections import Counter
def fit_lattice(z, lo=0.15, hi=0.35):
    bx=spacing_scan([v.real for v in z],lo,hi,0.0005); by=spacing_scan([v.imag for v in z],lo,hi,0.0005)
    d=(bx[1]+by[1])/2
    # offsets from the phase at this common spacing
    def ph(vals):
        re=sum(math.cos(2*math.pi*v/d) for v in vals); im=sum(math.sin(2*math.pi*v/d) for v in vals)
        return math.atan2(im,re)/(2*math.pi)*d, math.hypot(re,im)/len(vals)
    ox,px=ph([v.real for v in z]); oy,py=ph([v.imag for v in z])
    return d,ox,oy,px,py
def refine(z, d, ox, oy, iters=4, blk=600):
    # DD phase refinement against nearest lattice site, per block
    for _ in range(iters):
        out=[]
        for i in range(0,len(z),blk):
            b=z[i:i+blk]; num=0j
            for v in b:
                s=complex(round((v.real-ox)/d)*d+ox, round((v.imag-oy)/d)*d+oy)
                num+=v*s.conjugate()
            r=cmath.exp(-1j*cmath.phase(num))
            out+=[v*r for v in b]
        z=out
    return z
def summarise(z, label):
    d,ox,oy,px,py=fit_lattice(z)
    zr=refine(z,d,ox,oy)
    d,ox,oy,px,py=fit_lattice(zr)
    c=Counter(); err=0
    for v in zr:
        i=round((v.real-ox)/d); j=round((v.imag-oy)/d); c[(i,j)]+=1
        err+=(v.real-ox-i*d)**2+(v.imag-oy-j*d)**2
    rms=math.sqrt(err/len(zr))/d
    n=len(zr); sites=sorted(c.values(),reverse=True)
    # occupancy histogram: sites well above the noise floor
    avg=n/ max(1,sum(1 for v in sites if v>n*0.002))
    thr=[0.15,0.3,0.5]
    res={t:sum(1 for v in sites if v>=t*avg) for t in thr}
    print(f'{label}: d={d:.4f} (peaks x{px:.2f} y{py:.2f}) offsets ({ox/d:+.2f},{oy/d:+.2f}) d; rms dist to site {rms:.3f} d; symbols {n}')
    print('   distinct sites hit',len(c),' sites with >= 0.15/0.3/0.5 x average count:',res, ' average count/site %.1f'%avg)
    return zr,c,d,ox,oy
if __name__=='__main__':
    fn=sys.argv[1]; t=float(sys.argv[2]); dur=float(sys.argv[3])
    y,mse=equalize_data(fn,t,dur)
    z,rhos=track(y,600)
    zr,c,d,ox,oy=summarise(z,fn)
    # row profile: number of occupied sites per row
    rows={}
    for (i,j),v in c.items():
        if v>=0.3*(len(zr)/128): rows.setdefault(j,[]).append(i)
    print('   rows (j: count):',{j:len(v) for j,v in sorted(rows.items())})
