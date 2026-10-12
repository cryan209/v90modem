import pickle
from trellis import unwarp, best_d
def sites(pk,c):
    z=pickle.load(open(pk,'rb')); u=unwarp(z,c); sc,d,ox,oy=best_d(u,0.094,0.108,57)
    return [(round((v.real-ox)/d),round((v.imag-oy)/d)) for v in u]
def labels(pk,c):
    """(p, a, b): checkerboard parity, i mod 2, (j - 2 floor(i/2)) mod 4 -- the Z2 x Z4 coset of 2RZ^2"""
    return [((i+j)&1,i%2,(j-2*(i//2))%4) for i,j in sites(pk,c)]
def ung(pk,c):
    """Ungerboeck bits (z0,z1,z2) for Z2>RZ2>2Z2>2RZ2 chain, per symbol"""
    out=[]
    for i,j in sites(pk,c):
        p=(i+j)&1; ip=i-p; z1=ip&1
        i2=ip-z1; j2=j-z1
        out.append((p,z1,((i2+j2)//2)&1))
    return out
