import sys; sys.path.insert(0,'.')
from eq2 import *
from lat import spacing_scan, count_sites
def equalize_data(fn, t_data0, dur_data, blk=500):
    buf=open(fn,'rb').read()
    t0=8.4; x=prep(buf,t0,t_data0+dur_data-t0)
    ex=trn_expected(4400); n_est=int(round(2*(476+20)+1.5)); L=32
    a,d,p0=align(x,ex,True,n_est)
    w,(phi,nu,mse)=train(x,n_est+d+L//2,ex,True,L=L,phi0=p0)
    n0=n_est+d+L//2
    i_start=int((t_data0-t0)*2400)-int((n0-0)/2)*0   # symbol index where data window begins (relative to TRN start index 0 at n0)
    # symbol i of the TRN grid sits at x index n0+2*i; TRN2 start (i=0) is at time t0+(496+...)/2400 ~ 8.607 s
    t_trn=8.607
    i0=int(round((t_data0-t_trn)*2400))
    y=[]
    for i in range(i0, i0+int(dur_data*2400)):
        n=n0+2*i
        if n+1>=len(x): break
        win=x[n-L+1:n+1]
        raw=sum(w[j]*win[L-1-j] for j in range(L))
        y.append(raw*cmath.exp(-1j*(phi+nu*(i-4400))))
    return y, mse
if __name__=='__main__':
    fn=sys.argv[1]; t=float(sys.argv[2]); dur=float(sys.argv[3])
    y,mse=equalize_data(fn,t,dur)
    print('TRN training mse (last 500): %.5f'%(sum(mse[-500:])/500),' data symbols',len(y))
    # block derotation + scatter
    dz,phs=derotate(y,blk=400)
    print('4th-power phase per block (rad):',[round(p,2) for p in phs[:10]])
    N=49; g=[[0]*N for _ in range(N)]; m=2.4
    for v in dz[:8000]:
        a=int((v.real/m+1)/2*(N-1)); b=int((1-(v.imag/m+1)/2)*(N-1))
        if 0<=a<N and 0<=b<N: g[b][a]+=1
    sym=' .:-=+*#%@'; mx=max(max(r) for r in g)
    for r in g: print(''.join(sym[min(9,1+int(8*c/mx))] if c else ' ' for c in r))
