"""Back-equalise our own upstream before data start (what did the client send?).

usage: startup_constellation.py UP.g711 TX.caller   (numpy)
UP.g711 is tee_peer.py's upstream recording, TX.caller the V34_DATA_TX_DUMP.
A linear receiver is fitted on the first 2000 data symbols, applied backwards
over the startup, and radius/point statistics are printed per 80 symbols.
"""
import sys,struct,numpy as np
up_path,tx_path=sys.argv[1],sys.argv[2]
from numpy.fft import fft,ifft
u=np.frombuffer(open(up_path,'rb').read(),np.uint8)
def ulaw(b):
    b=~b.astype(np.int32)&0xff; s=b&0x80; e=(b>>4)&7; m=b&15
    v=(((m<<3)+0x84)<<e)-0x84
    return np.where(s,-v,v).astype(float)
x=ulaw(u); n=np.arange(len(x)); fc=3200*4/7
b=x*np.exp(-2j*np.pi*fc*n/8000)
r=np.array([complex(p,q) for p,q in struct.iter_unpack('<hh',open(tx_path,'rb').read())])/256.0
b2=np.empty(2*len(b)-1,complex); b2[0::2]=b; b2[1::2]=(b[:-1]+b[1:])/2
K=1024; ker=np.zeros(5*K,complex); ker[::5]=np.conj(r[:K])
N=1<<int(np.ceil(np.log2(len(b2)+len(ker))))
kpad=np.zeros(N,complex); kpad[:len(ker)]=ker
c=ifft(fft(b2,N)*np.conj(fft(np.conj(kpad),N)))[:len(b2)-len(ker)]
t0=int(np.argmax(abs(c))); n0=t0/2.0
print('data start at %.4f s, peak/median %.1f'%(n0/8000,abs(c[t0])/np.median(abs(c))))
M=12
def row(k):
    c0=int(np.floor(n0+2.5*k)); seg=b[c0-M:c0+M+1]; return np.concatenate([seg,np.conj(seg)])
ks=np.arange(4,2004); W={}
for par in (0,1):
    sel=ks[(ks%2)==par]; A=np.array([row(k) for k in sel]); W[par]=np.linalg.lstsq(A,r[sel],rcond=None)[0]
    print('par',par,'fit err',np.sqrt(np.mean(abs(A@W[par]-r[sel])**2)))
kmin=-int((n0-2000)/2.5)
est=np.array([row(k)@W[k%2] for k in range(kmin,0)])
print('est symbols',len(est),'from t=%.3f'%((n0+2.5*kmin)/8000))
t=lambda k:(n0+2.5*k)/8000
B=80
for i in range(0,len(est),B):
    s=est[i:i+B]; rr=abs(s);
    if len(s)<B: break
    q=np.round(s.real/2)*2+1j*np.round(s.imag/2)*2   # coarse grid
    print('%.3f-%.3f rms %6.1f  min %6.1f max %6.1f  rad-cv %.2f  uniq-pts(2-grid) %d'%(t(kmin+i),t(kmin+i+B),np.sqrt(np.mean(rr**2)),rr.min(),rr.max(),rr.std()/rr.mean(),len(set(np.round(s/6).tolist()))))
