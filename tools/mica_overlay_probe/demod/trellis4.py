"""Is the checkerboard parity p=(i+j)&1 of the next symbol a function of the previous labels?
Then search for a GF(2)-linear rule over the previous labels' bits."""
import sys, math, pickle
from collections import Counter, defaultdict
from trellis import unwarp, best_d
S=sys.argv[1]
def sites(pk,c):
    z=pickle.load(open(pk,'rb')); u=unwarp(z,c); sc,d,ox,oy=best_d(u,0.094,0.108,57)
    return [(round((v.real-ox)/d),round((v.imag-oy)/d)) for v in u]
def bits(i,j):                      # label in Z^2/2RZ^2: a=i%2, b=(j-2*(i//2))%4
    a=i%2; b=(j-2*(i//2))%4
    return (a,b&1,(b>>1)&1)
for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)):
    ij=sites(S+'/'+pk,c)
    p=[(i+j)&1 for i,j in ij]
    L=[bits(i,j) for i,j in ij]
    print(pk,'parity balance',sum(p)/len(p))
    for K in (1,2,3,4,5,6):
        ctx=defaultdict(Counter)
        for n in range(K,len(ij)): ctx[tuple(L[n-K:n])][p[n]]+=1
        err=sum(sum(c0.values())-max(c0.values()) for c0 in ctx.values()); tot=len(ij)-K
        print(f'  p given previous {K} labels: {len(ctx)} contexts, majority-rule error {err}/{tot} = {100*err/tot:.2f}%')
    # GF(2) search: previous 4 labels, 3 bits each = 12 bits
    K=4; H=Counter()
    for n in range(K,len(ij)):
        w=0
        for k in range(K):
            a,b0,b1=L[n-1-k]; w|=(a|(b0<<1)|(b1<<2))<<(3*k)
        H[(w,p[n])]+=1
    res=[]
    for m in range(1,1<<(3*K)):
        agree=0; tot=0
        for (w,pv),cn in H.items():
            q=bin(w&m).count('1')&1; tot+=cn
            if q==pv: agree+=cn
        res.append((max(agree,tot-agree)/tot,m,agree>=tot-agree))
    res.sort(reverse=True)
    names=[f'{n}{t}' for n in ('a1_','a2_','a3_','a4_') for t in ('x0','x1','x2')]  # label n back, bit
    for acc,m,pos in res[:4]:
        terms=[names[b] for b in range(12) if m>>b&1]
        print(f'   p == {"" if pos else "NOT "}XOR({", ".join(terms)}): agrees {100*acc:.2f}%')
