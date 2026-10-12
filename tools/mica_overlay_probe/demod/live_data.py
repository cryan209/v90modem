import sys, pickle
from sites import sites
from cmp_map import CONV_ISO, G
from descramble import ung
pk=sys.argv[1]; c=float(sys.argv[2])
ij=sites(pk,c)
P,flip=CONV_ISO[0]
for gi,g in enumerate(G[:4]):
    Z=[ung(*g(i,j)) for i,j in ij]; d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]
    Q=[(d[n]-d[n-1])%4 for n in range(1,len(d))]
    viol=[int((((Q[m]&1)^(Q[m-5]&1)^(Q[m-23]&1))!=1)) for m in range(23,len(Q))]
    print('transform',gi,'violation rate per 500 symbols:',[round(sum(viol[i:i+500])/500,2) for i in range(0,len(viol),500)])
