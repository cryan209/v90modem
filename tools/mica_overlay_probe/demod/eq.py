import sys, math, cmath
sys.path.insert(0,'.')
from dem import *

def lpf(fc, n=81):
    h=[]
    for k in range(n):
        m=k-(n-1)/2
        s=2*fc/FS if m==0 else math.sin(2*math.pi*fc*m/FS)/(math.pi*m)
        h.append(s*(0.54-0.46*math.cos(2*math.pi*k/(n-1))))
    g=sum(h); return [v/g for v in h]

def bb(buf,t0,dur,fc=1800.0,cut=1700.0):
    H=lpf(cut); i0=int(t0*FS); n=int(dur*FS); pad=len(H)
    x=[ul(v) for v in buf[i0-pad:i0+n]]
    z=[x[k]*cmath.exp(-2j*math.pi*fc*(i0-pad+k)/FS) for k in range(len(x))]
    return [sum(H[j]*z[k-j] for j in range(len(H)))*2 for k in range(pad,len(z))]

def to_half_T(zb, phase, taps=32):
    # 4800 Hz stream by windowed-sinc interpolation of the 8 kHz complex baseband
    out=[]; k=0; half=taps//2
    while True:
        t=(phase/(2*math.pi)+k/2.0)/BAUD*FS; i=int(math.floor(t))
        if i+half+1>=len(zb): break
        if i-half>=0:
            f=t-i; acc=0j
            for m in range(-half+1,half+1):
                u=m-f
                sinc=1.0 if abs(u)<1e-12 else math.sin(math.pi*u)/(math.pi*u)
                win=0.5+0.5*math.cos(math.pi*u/half)
                acc+=zb[i+m]*sinc*win
            out.append(acc)
        k+=1
    return out

def cma(x, L=32, mu=2e-3, passes=3, R2=1.38):
    # x: T/2 samples. normalise power
    p=sum(abs(v)**2 for v in x)/len(x); x=[v/math.sqrt(p) for v in x]
    w=[0j]*L; w[L//2]=1+0j
    y=[]
    for ps in range(passes):
        y=[]
        for n in range(L, len(x), 2):
            win=x[n-L:n]
            out=sum(w[j]*win[L-1-j] for j in range(L))
            e=out*(abs(out)**2-R2)
            for j in range(L):
                w[j]-=mu*e*win[L-1-j].conjugate()
            y.append(out)
    return y,w

def derotate(y, blk=300):
    out=[]; prev=None; phs=[]
    for i in range(0,len(y),blk):
        b=y[i:i+blk]; m4=sum(v**4 for v in b)
        ang=cmath.phase(m4)/4
        if prev is not None:
            while ang-prev>math.pi/4: ang-=math.pi/2
            while ang-prev<-math.pi/4: ang+=math.pi/2
        prev=ang; phs.append(ang); r=cmath.exp(-1j*ang)
        out+=[v*r for v in b]
    return out,phs
