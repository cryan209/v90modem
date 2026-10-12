import math, cmath
def ul(b):
    b=~b&0xff; sg=-1 if b&0x80 else 1; e=(b>>4)&7; m=b&15
    return sg*((((m<<3)+0x84)<<e)-0x84)
FS=8000.0; BAUD=2400.0
def lpf_taps(fc=1300.0, n=61):
    h=[]
    for k in range(n):
        m=k-(n-1)/2
        s=2*fc/FS if m==0 else math.sin(2*math.pi*fc*m/FS)/(math.pi*m)
        w=0.54-0.46*math.cos(2*math.pi*k/(n-1)); h.append(s*w)
    g=sum(h); return [v/g for v in h]
H=lpf_taps()
def baseband(buf,t0,dur,fc=1800.0):
    i0=int(t0*FS); n=int(dur*FS)
    x=[ul(v) for v in buf[i0-len(H):i0+n]]
    z=[x[k]*cmath.exp(-2j*math.pi*fc*(i0-len(H)+k)/FS) for k in range(len(x))]
    out=[]
    for k in range(len(H),len(z)):
        out.append(sum(H[j]*z[k-j] for j in range(len(H)))*2)
    return out  # sample k <-> time (i0+k)/FS
def timing_phase(zb):
    # phase of the baud line in |z|^2
    re=im=0
    for k,v in enumerate(zb):
        p=abs(v)**2; w=2*math.pi*BAUD*k/FS
        re+=p*math.cos(w); im-=p*math.sin(w)
    return math.atan2(im,re), math.hypot(re,im)/len(zb)
def symbols(zb,phase,offset_frac=0.0):
    # sample at t_k = (phase/(2pi) + k + offset)/BAUD  (in seconds relative to start)
    out=[]; k=0
    while True:
        t=((phase/(2*math.pi))+k+offset_frac)/BAUD*FS
        i=int(t); 
        if i+2>=len(zb): break
        if i>=1:
            f=t-i
            # cubic (Catmull-Rom) interpolation
            p0,p1,p2,p3=zb[i-1],zb[i],zb[i+1],zb[i+2]
            v=p1+0.5*f*(p2-p0+f*(2*p0-5*p1+4*p2-p3+f*(3*(p1-p2)+p3-p0)))
            out.append(v)
        k+=1
    return out
