import sys, math, cmath, pickle
sys.path.insert(0,'.')
from eq2 import *
from trn import quad_stream, states
from dd import run_dd
from f2d import f2d
from collections import Counter

def find_trn(buf, t_s, dur=2.6):
    q=quad_stream(buf,t_s,dur); ex=trn_expected(1500)
    best=(0,)
    for r in range(4):
        st=states(q,r)
        for p in range(100,min(1500,len(st)-1500)):
            m=sum(1 for i in range(300) if st[p+i]==ex[i])
            if m>best[0]: best=(m,r,p)
            if m==300: break
        if best[0]==300: break
    return best          # (match, rot, p)

def setup_gen(fn, t_s, data_delay, dur):
    buf=open(fn,'rb').read()
    m,r,p=find_trn(buf,t_s)
    x=prep(buf,t_s,data_delay+dur+0.3+ (p+20)/2400)
    ex=trn_expected(4400); n_est=int(round(2*(p+20)+1.5)); L=32
    a,d,p0=align(x,ex,True,n_est)
    w,(phi,nu,mse)=train(x,n_est+d+L//2,ex,True,L=L,phi0=p0)
    n0=n_est+d+L//2
    i0=int(round(data_delay*2400))
    t_trn=t_s+(p+20)/2400
    return x,w,phi,nu,n0,i0,L,(m,p,d,t_trn,sum(mse[-500:])/500)

def equalized_gen(fn,t_s,data_delay,dur=6.0,nsym=14000):
    x,w,phi,nu,n0,i0,L,info=setup_gen(fn,t_s,data_delay,dur)
    # rotation-invariant lattice search on the first 1200 symbols (TRN-trained equalizer, no DD)
    z0,_,_,_=run_dd(x,w,phi,nu,n0,i0,L,1200,0.108,0.0,passes=1,adapt=False,track=False)
    best=f2d(z0,0.08,0.30,60,30)[0]
    sc,d0,ang=best[0],best[1],best[2]
    z,w2,rr,rms=run_dd(x,w,phi,nu,n0,i0,L,nsym,d0,ang,passes=3)
    z,_,_,_=run_dd(x,w2,phi,nu,n0,i0,L,nsym,d0,rr,passes=1,adapt=False,track=True)
    return z,(info,sc,d0,math.degrees(ang))
if __name__=='__main__':
    fn=sys.argv[1]; t_s=float(sys.argv[2]); delay=float(sys.argv[3])
    z,info=equalized_gen(fn,t_s,delay)
    print(fn,'TRN match/p/align/t_trn/mse',info[0],' lattice score %.2f d0=%.4f rot=%.1f deg'%info[1:],' symbols',len(z))
    pickle.dump(z,open(fn.replace('.bin','_z.pkl'),'wb'))
