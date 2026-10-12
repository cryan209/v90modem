"""Follow-ups to trellis.py: deeper contexts, stationarity, and a synthetic
8-state rate-2/3 code through the same estimator (sensitivity control)."""
import sys, math, pickle, random
from collections import Counter
from trellis import unwarp, best_d, entropy, cosets

def labels(pk,c,coset):
    z=pickle.load(open(pk,'rb')); u=unwarp(z,c)
    sc,d,ox,oy=best_d(u,0.094,0.108,57)
    f=cosets(*coset)
    return [f(round((v.real-ox)/d),round((v.imag-oy)/d)) for v in u]

def report(name,lab):
    hs=[entropy(lab,k) for k in range(1,6)]
    cond=[hs[0]]+[hs[k]-hs[k-1] for k in range(1,5)]
    print(f'  {name}: H(k|k-1..) for k=1..5: '+' '.join(f'{x:.3f}' for x in cond))

if __name__=='__main__':
    S=sys.argv[1]
    # synthetic: 8-state systematic feedback code, Y1Y2 random, Y0 = state bit; state update uses Y1,Y2
    random.seed(5); s=0; syn=[]
    for _ in range(14000):
        y1,y2=random.getrandbits(1),random.getrandbits(1)
        y0=((s>>2)^(s&1))&1
        syn.append((y2,y1,y0))
        s=((s<<1)&7)|(y1^y2^(s>>2)) 
    print('synthetic 8-state code, index-8 labels (iid would be 3.000):'); report('synth',syn)
    for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)):
        print(pk)
        for cs in ((2,2,4),(2,0,4),(8,0,1),(1,0,8),(4,0,2)):
            lab=labels(S+'/'+pk,c,cs); report(f'index8 L=<({cs[0]},{cs[1]}),(0,{cs[2]})>',lab)
        for cs in ((2,0,2),(2,1,2)):
            lab=labels(S+'/'+pk,c,cs); report(f'index4 L=<({cs[0]},{cs[1]}),(0,{cs[2]})>',lab)
        lab=labels(S+'/'+pk,c,(2,2,4))
        print('  stationarity (index8, H(3|12)) over 4 quarters:',' '.join('%.3f'%(entropy(lab[i:i+3500],3)-entropy(lab[i:i+3500],2)) for i in range(0,14000,3500)))
        for par in (0,1):
            sub=lab[par::2]; print(f'  even/odd phase {par} (symbol-pair alignment) H(2|1) %.3f'%(entropy(sub,2)-entropy(sub,1)))
