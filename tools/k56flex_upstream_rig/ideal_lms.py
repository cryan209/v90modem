import sys,struct,numpy as np
up,txp=sys.argv[1],sys.argv[2]
u=np.frombuffer(open(up,'rb').read(),np.uint8)
def ulaw(b):
    b=~b.astype(np.int32)&0xff; s=b&0x80; e=(b>>4)&7; m=b&15
    v=(((m<<3)+0x84)<<e)-0x84; return np.where(s,-v,v).astype(float)
x=ulaw(u); n=np.arange(len(x)); bb=x*np.exp(-2j*np.pi*(3200*4/7)*n/8000)
r=np.array([complex(p,q) for p,q in struct.iter_unpack('<hh',open(txp,'rb').read())])/256.0
# coarse align as offline_rx
b2=np.empty(2*len(bb)-1,complex); b2[0::2]=bb; b2[1::2]=(bb[:-1]+bb[1:])/2
K=1024; ker=np.zeros(5*K,complex); ker[::5]=np.conj(r[:K]); N=1<<int(np.ceil(np.log2(len(b2)+len(ker))))
c=np.fft.ifft(np.fft.fft(b2,N)*np.conj(np.fft.fft(np.conj(np.pad(ker,(0,N-len(ker)))),N)))[:len(b2)-len(ker)]
n0=np.argmax(abs(c))/2.0
M=12
def reg(k):
    c0=int(np.floor(n0+2.5*k)); return bb[c0-M:c0+M+1]
scale=np.sqrt(np.mean(abs(bb[int(n0):int(n0)+20000])**2))
ks=np.arange(8,5008)
for par in (0,1):
    kk=ks[ks%2==par]; A=np.array([reg(k)/scale for k in kk]); w=np.linalg.lstsq(A,r[kk],rcond=None)[0]
    print('LS parity',par,round(float(np.sqrt(np.mean(abs(A@w-r[kk])**2))),4))
def run(mu,train=509,start=8,test=2000):
    w=[np.zeros(2*M+1,complex),np.zeros(2*M+1,complex)]
    for k in range(start,start+train):
        xk=reg(k)/scale; p=k%2; e=r[k]-np.dot(w[p],xk); w[p]=w[p]+mu*e*np.conj(xk)
    errs=[r[k]-np.dot(w[k%2],reg(k)/scale) for k in range(start+train,start+train+test)]
    return float(np.sqrt(np.mean(abs(np.array(errs))**2)))
for mu in [0.005,0.01,0.02,0.03,0.05,0.08]:
    print('mu %.3f: after 509 symbols from zero, error %.3f spacings; after 2048: %.3f'%(mu,run(mu),run(mu,train=2048)))
