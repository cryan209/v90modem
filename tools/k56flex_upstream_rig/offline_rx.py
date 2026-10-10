"""Best linear receiver on the upstream G.711 MICA was fed (tee_peer.py output),
given the client's transmitted points; an upper bound for any equalizer."""
import sys,struct,numpy as np
up,txp=sys.argv[1],sys.argv[2]
u=np.frombuffer(open(up,'rb').read(),np.uint8)
def ulaw(b):
    b=~b.astype(np.int32)&0xff; s=b&0x80; e=(b>>4)&7; m=b&15
    v=(((m<<3)+0x84)<<e)-0x84
    return np.where(s,-v,v).astype(float)
x=ulaw(u); n=np.arange(len(x)); fc=3200*4/7
b=x*np.exp(-2j*np.pi*fc*n/8000)
r=np.array([complex(p,q) for p,q in struct.iter_unpack('<hh',open(txp,'rb').read())])/256.0
print('upstream samples',len(x),'(%.2f s)'%(len(x)/8000),'tx points',len(r))
# coarse search: 16 kHz grid (linear interp), symbols every 5 grid points
b2=np.empty(2*len(b)-1,complex); b2[0::2]=b; b2[1::2]=(b[:-1]+b[1:])/2
K=1024; ker=np.zeros(5*K,complex); ker[::5]=np.conj(r[:K])
from numpy.fft import fft,ifft
N=1<<int(np.ceil(np.log2(len(b2)+len(ker))))
corr=ifft(fft(b2,N)*np.conj(fft(ker[::-1].conj()[::-1],N)))  # placeholder
# direct sliding via FFT correlation: c[t]=sum_k conj(r_k) b2[t+5k]
kpad=np.zeros(N,complex); kpad[:len(ker)]=ker
c=ifft(fft(b2,N)*np.conj(fft(np.conj(kpad),N)))[:len(b2)-len(ker)]
t0=int(np.argmax(abs(c))); print('coarse start %.4f s (16 kHz index %d), peak/median %.1f'%(t0/16000,t0,abs(c[t0])/np.median(abs(c))))
n0=t0/2.0  # in 8 kHz samples
M=12
def design(ks):
    rows=[]
    for k in ks:
        c0=int(np.floor(n0+2.5*k)); seg=b[c0-M:c0+M+1]
        rows.append(np.concatenate([seg,np.conj(seg)]))
    return np.array(rows)
ks=np.arange(4,min(len(r)-4,int((len(x)-n0)/2.5)-8))
for lo,hi,lab in [(0,1024,'first 1024'),(0,len(ks),'all')]:
    kk=ks[lo:hi]; tot=0;sig=0
    out=[]
    for par in (0,1):
        sel=kk[(kk%2)==par]; A=design(sel); w=np.linalg.lstsq(A,r[sel],rcond=None)[0]
        e=A@w-r[sel]; tot+=np.sum(abs(e)**2); sig+=np.sum(abs(r[sel])**2)
    print('%s: %d symbols, LS linear receiver error RMS %.4f spacings, SNR %.1f dB'%(lab,len(kk),np.sqrt(tot/len(kk)),10*np.log10(sig/tot)))
# u-law quantisation alone: requantise ideal? report signal level
print('upstream RMS %.0f (dBFS %.1f), peak %.0f'%(x[int(n0):].std(),20*np.log10(x[int(n0):].std()/32124),abs(x[int(n0):]).max()))
