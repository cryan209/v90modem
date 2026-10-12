import sys, math, cmath
sys.path.insert(0,'.')
from eq import *
from trn import trn_expected
PTS=[(-6-2j),(2-6j),(-2+6j),(6+2j)]   # A,B,D,C by state number 0..3
def train(x, n0, ex, mirror, L=32, mu=0.02, passes=3, kp=0.05, ki=0.0008, phi0=0.0):
    # x: T/2 samples; symbol i output at x index n0+2*i ; ex[i]: state
    ref=[ (PTS[s].conjugate() if mirror else PTS[s])/math.sqrt(40) for s in ex]
    w=[0j]*L; w[L//2]=1+0j
    best=None
    for ps in range(passes):
        phi=phi0; nu=0.0; mse=[]
        for i in range(len(ref)):
            n=n0+2*i
            if n-L<0: continue
            win=x[n-L+1:n+1]
            raw=sum(w[j]*win[L-1-j] for j in range(L))
            yp=raw*cmath.exp(-1j*phi)
            err_ph=cmath.phase(yp*ref[i].conjugate())
            e=ref[i]*cmath.exp(1j*phi)-raw
            nrm=sum(abs(v)**2 for v in win)+1e-9
            g=mu*e/nrm
            for j in range(L): w[j]+=g*win[L-1-j].conjugate()
            phi+=kp*err_ph+nu; nu+=ki*err_ph
            mse.append(abs(yp-ref[i])**2)
        if ps==passes-1: final=(phi,nu,mse)
    return w,final
def prep(buf,t0,dur):
    zb=bb(buf,t0,dur); ph,_=timing_phase(zb); x=to_half_T(zb,ph)
    pw=math.sqrt(sum(abs(v)**2 for v in x)/len(x)); return [v/pw for v in x]
def align(x, ex, mirror, n_est):
    ref=[(PTS[s].conjugate() if mirror else PTS[s])/math.sqrt(40) for s in ex]
    best=None
    for d in range(-30,31):
        n0=n_est+d
        c=sum(x[n0+2*i]*ref[i].conjugate() for i in range(1,200))
        if best is None or abs(c)>best[0]: best=(abs(c),d,cmath.phase(c))
    return best
if __name__=='__main__':
    fn=sys.argv[1]; t0=8.4; dur=float(sys.argv[2])
    x=prep(open(fn,'rb').read(),t0,dur)
    ex=trn_expected(4400)
    n_est=int(round(2*(476+20)+1.5))
    for mirror in (False,True):
        a,d,p0=align(x,ex,mirror,n_est); L=32
        w,(phi,nu,mse)=train(x,n_est+d+L//2,ex,mirror,L=L,phi0=p0)
        m=len(mse)
        print('mirror',mirror,'align',d,'phase0 %.2f'%p0,'mse first 200 %.3f mid %.3f last 500 %.4f'%(sum(mse[:200])/200,sum(mse[m//2:m//2+500])/500,sum(mse[-500:])/500),'nu',round(nu,5))
