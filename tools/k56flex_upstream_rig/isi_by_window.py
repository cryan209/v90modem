"""Per-window +/-24-symbol widely-linear ISI fit of MICA's trellis input
(47B9 capture) against the transmitted points: static vs drifting ISI."""
import sys,importlib.util,numpy as np,struct
spec=importlib.util.spec_from_file_location('l',__import__('os').path.join(__import__('os').path.dirname(__file__),'..','k56flex_upstream_lattice.py'));l=importlib.util.module_from_spec(spec);spec.loader.exec_module(l)
cap,tx,t0,t1=sys.argv[1],sys.argv[2],float(sys.argv[3]),float(sys.argv[4])
rx=l.load(cap,t0,t1); ref=np.array([complex(x,y) for x,y in struct.iter_unpack('<hh',open(tx,'rb').read())])/256
n=len(rx);best=None
for s in range(min(600,len(ref)-n)):
    r=ref[s:s+n];c=abs(np.vdot(r,rx))/np.sqrt(np.vdot(r,r).real*np.vdot(rx,rx).real)
    if best is None or c>best[0]:best=(c,s)
r=ref[best[1]:best[1]+n]; z=np.zeros(n,complex)
for w in range(0,n,64):
    sl=slice(w,min(n,w+64)); g=np.vdot(r[sl],rx[sl])/np.vdot(r[sl],r[sl]); z[sl]=rx[sl]/g
L=24; W=1024; prev=None
for w in range(L,n-L-W+1,W):
    idx=np.arange(w,w+W); e=z[idx]-r[idx]
    A=np.array([v for j in range(-L,L+1) for v in (r[idx+j],np.conj(r[idx+j]))]).T
    c=np.linalg.lstsq(A,e,rcond=None)[0]; res=e-A@c
    isi=np.sqrt(np.sum(abs(c)**2))
    drift='' if prev is None else ' change vs previous window %.3f'%np.sqrt(np.sum(abs(c-prev)**2))
    print('symbols %5d+%d: raw %.3f, ISI energy %.3f, after own +/-24 fit %.3f%s'%(w,W,np.sqrt(np.mean(abs(e)**2)),isi,np.sqrt(np.mean(abs(res)**2)),drift))
    prev=c
