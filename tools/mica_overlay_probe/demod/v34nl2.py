import sys, math, pickle
sys.path.insert(0,'.')
from v34nl import *
def metrics(c, pts, exact=False):
    u=unwarp_c(pts,c,exact)
    sc,d,ox,oy=best_d(u,d_lo,d_hi,41)
    cc=Counter(); err=0
    for v in u:
        i=round((v.real-ox)/d); j=round((v.imag-oy)/d); cc[(i,j)]+=1
        err+=(v.real-ox-i*d)**2+(v.imag-oy-j*d)**2
    rms=math.sqrt(err/len(u))/d
    n=len(u); mean=n/512
    amb=sum(1 for v in cc.values() if 0.2*mean<=v<=0.8*mean)      # cells neither clearly occupied nor clearly empty
    E=sum(abs(v)**2 for v in u)/len(u)
    return sc,d,rms,amb,E
d_lo,d_hi=0.094,0.108
print('c      V.34 needs   score   d0      rms/d   ambiguous cells   E')
for c in (0.30,0.34,0.36,0.38,0.42,0.46,0.50,0.54,0.58):
    sc,d,rms,amb,E=metrics(c,z)
    print('%.2f   %.3f      %.3f   %.4f  %.3f   %4d              %.3f'%(c,0.3125/E,sc,d,rms,amb,E))
