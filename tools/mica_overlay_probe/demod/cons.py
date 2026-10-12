import sys; sys.path.insert(0,'.')
from data_eq import *
def track(y, blk=600):
    out=[]; prev=None; rhos=[]
    for i in range(0,len(y),blk):
        b=y[i:i+blk]; m4=sum(v**4 for v in b)
        th=cmath.phase(m4)/4          # = pi/4 + rho  (mod pi/2) for an axis-aligned lattice
        rho=th-math.pi/4
        if prev is not None:
            while rho-prev>math.pi/4: rho-=math.pi/2
            while rho-prev<-math.pi/4: rho+=math.pi/2
        prev=rho; rhos.append(rho); r=cmath.exp(-1j*rho)
        out+=[v*r for v in b]
    return out,rhos
def scatter(z,m=2.0,N=57,n=7000):
    g=[[0]*N for _ in range(N)]
    for v in z[:n]:
        a=int((v.real/m+1)/2*(N-1)); b=int((1-(v.imag/m+1)/2)*(N-1))
        if 0<=a<N and 0<=b<N: g[b][a]+=1
    sym=' .:-=+*#%@'; mx=max(max(r) for r in g)
    print('\n'.join(''.join(sym[min(9,1+int(8*c/mx))] if c else ' ' for c in r) for r in g))
if __name__=='__main__':
    fn=sys.argv[1]; t=float(sys.argv[2]); dur=float(sys.argv[3]); blk=int(sys.argv[4]) if len(sys.argv)>4 else 600
    y,mse=equalize_data(fn,t,dur)
    z,rhos=track(y,blk)
    print('rho per block:',[round(r,3) for r in rhos])
    P=sum(abs(v)**2 for v in z)/len(z); print('mean power',round(P,3))
    for ax in ('x','y'):
        v=[ (q.real if ax=='x' else q.imag) for q in z]
        b=spacing_scan(v,0.1,0.6,0.001); print(ax,'spacing %.3f peak %.2f'%(b[1],b[0]))
    scatter(z)
