"""Replay the recovered coder (isomorphic to spandsp's V.32bis CONV) on a second recording and on a shuffled control."""
import random
from sites import sites
from cmp_map import CONV_ISO, G
CONV=[[0,2,3,1],[4,7,5,6],[1,3,2,0],[7,4,6,5],[2,0,1,3],[6,5,7,4],[3,1,0,2],[5,6,4,7]]
S='/private/tmp/claude-501/-Users-scottcryan-v90modem/8fb1ce38-0fb9-4688-a35e-483c585993ad/scratchpad/'
def ung(i,j):
    p=(i+j)&1; ip=i-p; z1=ip&1; i2=ip-z1; j2=j-z1
    return (p,z1,((i2+j2)//2)&1)
def score(ij,P,flip,g):
    Z=[ung(*g(i,j)) for i,j in ij]; best=0
    for init in range(8):
        s=init; ok=0
        for z0,z1,z2 in Z:
            ok+=((s&1)^flip)==z0; s=CONV[s][P[z0][2*z2+z1]]
        best=max(best,ok)
    return best/len(Z)
for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)):
    ij=sites(S+pk,c); sh=ij[:]; random.seed(1); random.shuffle(sh)
    r=max((score(ij,P,fl,g),gi) for P,fl in CONV_ISO for gi,g in enumerate(G))
    ctl=max(score(sh,P,fl,g) for P,fl in CONV_ISO for g in G)
    print(pk,'best y0 prediction %.2f%% (lattice transform %d);  shuffled-order control %.2f%%'%(100*r[0],r[1],100*ctl))
