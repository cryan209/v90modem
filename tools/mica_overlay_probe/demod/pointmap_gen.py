"""Generate MICA's terbo point map and compare with the measured table.
Rule (fitted, exact for all 8 cosets in both recordings):
  each of the 8 cosets of 2RZ^2 (the coder's subsets) contributes its 64 lowest points, ordered by
  (r^2, x+y, -x) about the cell centre C after a per-coset rotation of k*90 degrees; the 6-bit word
  of a point is its rank.  (The set is NOT simply the 512 points nearest C: the boundary shell is
  split.)  Rotation depends on (a, parity) = (i mod 2, (i+j) mod 2) as 2a+parity in recording 2's frame."""
import json
from collections import defaultdict
T=json.load(open('terbo_site_words.json'))
ROT=[lambda x,y:(x,y),lambda x,y:(-y,x),lambda x,y:(-x,-y),lambda x,y:(y,-x)]
def coset(i,j): return (i%2,(j-2*(i//2))%4)
REC={'rec2_z.pkl':((0.5,0.5),{(0,0):0,(0,1):1,(0,2):0,(0,3):1,(1,0):3,(1,1):2,(1,2):3,(1,3):2}),
     'rec1_z.pkl':((0.5,-0.5),{(0,0):1,(0,1):0,(0,2):1,(0,3):0,(1,0):2,(1,1):3,(1,2):2,(1,3):3})}
def gen(C,rm):
    """{(i,j): word}: per coset, the 64 lowest points of the infinite coset in the order above"""
    cs=defaultdict(list)
    for i in range(-30,31):
        for j in range(-30,31): cs[coset(i,j)].append((i,j))
    out={}
    for k,L in cs.items():
        g=ROT[rm[k]]
        key=lambda p:(lambda x,y:(x*x+y*y,x+y,-x))(*g(p[0]-C[0],p[1]-C[1]))
        for n,p in enumerate(sorted(L,key=key)[:64]): out[p]=n
    return out
if __name__=='__main__':
    for name,(C,rm) in REC.items():
        meas={(i,j):w for i,j,w in T[name]}; t=gen(C,rm)
        print(name,'generated set equals measured set:',set(t)==set(meas),'; words reproduced %d/512'%sum(1 for p,w in meas.items() if t.get(p)==w))
