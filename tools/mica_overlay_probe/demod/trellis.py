"""Is MICA's terbo data trellis-coded?  Slice the equalized, unwarped symbols to
integer lattice sites and test whether the coset label sequence has the full
entropy of independent symbols (no code) or less (a convolutional code).
Usage: trellis.py recN_z.pkl c"""
import sys, math, pickle, os
from collections import Counter

def phi(z): return 1+z/6+z*z/120
def inv(r,c):
    lo,hi=0.0,r
    for _ in range(40):
        m=(lo+hi)/2
        if m*phi(c*m*m)<r: lo=m
        else: hi=m
    return (lo+hi)/2
def unwarp(pts,c): return [v if abs(v)<1e-9 else v*(inv(abs(v),c)/abs(v)) for v in pts]
def score_at(pts,d):
    s=0; off=[]
    for vals in ([v.real for v in pts],[v.imag for v in pts]):
        re=sum(math.cos(2*math.pi*u/d) for u in vals); im=sum(math.sin(2*math.pi*u/d) for u in vals)
        s+=math.hypot(re,im)/len(vals); off.append(math.atan2(im,re)/(2*math.pi)*d)
    return s/2,off
def best_d(pts,lo,hi,n):
    best=(0,)
    for k in range(n):
        d=lo+(hi-lo)*k/(n-1); sc,off=score_at(pts,d)
        if sc>best[0]: best=(sc,d,off[0],off[1])
    return best

def entropy(seq,k):
    """block entropy (bits) of k-grams with Miller-Madow bias correction"""
    c=Counter(tuple(seq[i:i+k]) for i in range(len(seq)-k+1)); n=sum(c.values())
    h=-sum(v/n*math.log2(v/n) for v in c.values())
    return h+(len(c)-1)/(2*n*math.log(2))

def cosets(a,b,cc):
    """label of (i,j) in Z^2 / L, L=<(a,b),(0,cc)> Hermite form, index a*cc"""
    def f(i,j):
        q=i//a
        return (i%a,(j-q*b)%cc)
    return f

def main(pk,c):
    z=pickle.load(open(pk,'rb')); u=unwarp(z,c)
    sc,d,ox,oy=best_d(u,0.094,0.108,57)
    ij=[(round((v.real-ox)/d),round((v.imag-oy)/d)) for v in u]
    cnt=Counter(ij); mean=len(ij)/512
    good={k for k,v in cnt.items() if v>=0.35*mean}
    print(f'{os.path.basename(pk)}: c={c} d={d:.4f} score {sc:.2f} symbols {len(ij)}  sites kept {len(good)}')
    bad=sum(1 for p in ij if p not in good); print(f'  symbols off the kept set: {bad} ({100*bad/len(ij):.2f}%)')
    for N in (4,8,16):
        rows=[]
        for a in range(1,N+1):
            if N%a: continue
            cc=N//a
            for b in range(cc if a>1 else 1):
                f=cosets(a,b,cc); lab=[f(i,j) for i,j in ij]
                h1,h2,h3=entropy(lab,1),entropy(lab,2),entropy(lab,3)
                rows.append((h3-h2,h2-h1,h1,a,b,cc))
        rows.sort()
        print(f' index {N}: iid conditional entropy would be {math.log2(N):.2f} bit/symbol; lowest H(3|12):')
        for r in rows[:3]:
            print(f'   L=<({r[3]},{r[4]}),(0,{r[5]})>  H1 {r[2]:.3f}  H(2|1) {r[1]:.3f}  H(3|12) {r[0]:.3f}')
if __name__=='__main__': main(sys.argv[1],float(sys.argv[2]))
