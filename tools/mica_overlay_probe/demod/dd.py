import sys, math, cmath
sys.path.insert(0,'.')
from eq2 import *
from f2d import f2d
from collections import Counter

def setup(fn, t_data0, dur):
    buf=open(fn,'rb').read(); t0=8.4
    x=prep(buf,t0,t_data0+dur-t0)
    ex=trn_expected(4400); n_est=int(round(2*(476+20)+1.5)); L=32
    a,d,p0=align(x,ex,True,n_est)
    w,(phi,nu,mse)=train(x,n_est+d+L//2,ex,True,L=L,phi0=p0)
    n0=n_est+d+L//2
    i0=int(round((t_data0-8.607)*2400))
    return x,w,phi,nu,n0,i0,L

def run_dd(x,w,phi,nu,n0,i0,L,nsym,d,rho,passes=3,mu=0.003,kp=0.01,off=0.5,adapt=True,track=True):
    w=list(w)
    for ps in range(passes):
        out=[]; rr=rho; errs=0.0
        for k in range(nsym):
            i=i0+k; n=n0+2*i
            if n+1>=len(x): break
            win=x[n-L+1:n+1]
            raw=sum(w[j]*win[L-1-j] for j in range(L))
            yb=raw*cmath.exp(-1j*(phi+nu*(i-4400)))
            u=yb*cmath.exp(-1j*rr)
            si=math.floor(u.real/d)+off; sj=math.floor(u.imag/d)+off     # nearest odd-lattice site centre
            s=complex(si*d,sj*d)
            if adapt:
                e=(s*cmath.exp(1j*rr+1j*(phi+nu*(i-4400))))-raw          # error in raw domain
                nrm=sum(abs(v)**2 for v in win)+1e-9
                g=mu*e/nrm
                for j in range(L): w[j]+=g*win[L-1-j].conjugate()
            if track:
                rr+=kp*cmath.phase(u*s.conjugate())*0.2
            errs+=abs(u-s)**2
            out.append(u)
        rms=math.sqrt(errs/len(out))/d
    return out,w,rr,rms

def summarise_points(z,d,label):
    # refine d and offsets by scan around d
    best=(0,)
    for k in range(-30,31):
        dd=d*(1+k*0.002)
        xs=[v.real for v in z]; ys=[v.imag for v in z]
        re=sum(math.cos(2*math.pi*v/dd) for v in xs); im=sum(math.sin(2*math.pi*v/dd) for v in xs)
        sx=math.hypot(re,im)/len(z)
        re2=sum(math.cos(2*math.pi*v/dd) for v in ys); im2=sum(math.sin(2*math.pi*v/dd) for v in ys)
        sy=math.hypot(re2,im2)/len(z)
        if min(sx,sy)>best[0]: best=(min(sx,sy),dd,math.atan2(im,re)/(2*math.pi)*dd,math.atan2(im2,re2)/(2*math.pi)*dd)
    sc,dd,ox,oy=best
    c=Counter(); err=0
    for v in z:
        i=round((v.real-ox)/dd); j=round((v.imag-oy)/dd); c[(i,j)]+=1
        err+=(v.real-ox-i*dd)**2+(v.imag-oy-j*dd)**2
    rms=math.sqrt(err/len(z))/dd
    P=sum(abs(v)**2 for v in z)/len(z)
    n=len(z); counts=sorted(c.values(),reverse=True)
    # a site is "real" if it holds at least 25% of the mean occupancy of the top-N sites; sweep N
    print(f'{label}: d={dd:.4f} score {sc:.2f} offsets ({ox/dd:+.2f},{oy/dd:+.2f})d  rms dist {rms:.3f} d  mean power {P:.3f}  symbols {n}')
    print(f'   power-implied points 6P/d^2+1 = {6*P/dd**2+1:.0f}')
    for thr_div in (64,128,256,512,1024):
        pass
    sites=len(c)
    for frac in (0.1,0.2,0.3,0.5):
        # mean occupancy of sites above 10% of overall mean to avoid the tail
        tot=sum(counts); mean_all=tot/len(counts)
        core=[v for v in counts if v>=mean_all*0.5]
        m=sum(core)/len(core)
        print(f'   sites >= {frac:.1f} x core mean ({m:.1f}): {sum(1 for v in counts if v>=frac*m)}',end='')
    print()
    return c,dd,ox,oy

if __name__=='__main__':
    fn=sys.argv[1]; t=float(sys.argv[2]); d0=float(sys.argv[3]); rho=math.radians(float(sys.argv[4])); passes=int(sys.argv[5]) if len(sys.argv)>5 else 3
    x,w,phi,nu,n0,i0,L=setup(fn,t,3.0)
    nsym=7000
    z0,_,_,rms0=run_dd(x,w,phi,nu,n0,i0,L,nsym,d0,rho,passes=1,adapt=False)
    print('before DD refinement: rms dist to nearest site %.3f d'%rms0)
    z,w2,rr,rms=run_dd(x,w,phi,nu,n0,i0,L,nsym,d0,rho,passes=passes)
    print('after %d DD passes: rms dist %.3f d, final rotation %.2f deg'%(passes,rms,math.degrees(rr)))
    # final pass without adaptation using refined weights, the clean scatter to analyse
    z,_,_,rms=run_dd(x,w2,phi,nu,n0,i0,L,nsym,d0,rr,passes=1,adapt=False,track=True)
    summarise_points(z,d0,fn)
