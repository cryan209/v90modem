"""Fit V.32bis/V.17's 8-state coder (spandsp v32bis.c convolutional[8][4]) to the
terbo label sequence.  Label (Z2 x Z4 coset of 2RZ^2) -> (y0, d) with y0 tied to the
checkerboard parity p, d in 0..3 the differential-coded dibit; state' = conv[state][d];
y0 = bit2 of state' (spandsp B1 convention) or of state (old-state convention)."""
import sys, itertools
from sites import sites
CONV=[[0,2,3,1],[4,7,5,6],[1,3,2,0],[7,4,6,5],[2,0,1,3],[6,5,7,4],[3,1,0,2],[5,6,4,7]]
S=sys.argv[1]
def labels(pk,c):
    ij=sites(S+'/'+pk,c); out=[]
    for i,j in ij: out.append(((i+j)&1, i%2, (j-2*(i//2))%4))
    return out
def score(L,phi,flip,newstate,s0):
    s=s0; ok=0
    for p,a,b in L:
        d=phi[p][(a,b)]
        ns=CONV[s][d]
        pred=((ns if newstate else s)>>2)&1
        if pred==(p^flip): ok+=1
        s=ns
    return ok
def run(pk,c):
    L=labels(pk,c); head=L[:600]
    keys=[[(a,b) for a in (0,1) for b in range(4) if (a+b)%2==p] for p in (0,1)]
    # (a+b)&1 == p holds for p=(i+j)&1? check below
    best=[]
    for perm0 in itertools.permutations(range(4)):
        for perm1 in itertools.permutations(range(4)):
            phi=[dict(zip(keys[0],perm0)),dict(zip(keys[1],perm1))]
            for flip in (0,1):
                for ns in (0,1):
                    m=max(score(head,phi,flip,ns,s0) for s0 in range(8))
                    best.append((m,perm0,perm1,flip,ns))
    best.sort(reverse=True)
    print(pk,'top fits on first 600 symbols:',[(b[0],)+b[1:] for b in best[:3]],' chance ~300')
    m,perm0,perm1,flip,ns=best[0]
    phi=[dict(zip(keys[0],perm0)),dict(zip(keys[1],perm1))]
    full=max(score(L,phi,flip,ns,s0) for s0 in range(8))
    print('   whole record: %d / %d = %.2f%%'%(full,len(L),100*full/len(L)))
for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)): run(pk,c)
