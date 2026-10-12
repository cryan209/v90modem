"""Descramble a live call that carried known DTE data.  usage: live_decode.py Z.pkl c [gi] [t0_sec_of_z0]"""
import sys, pickle
from collections import Counter
from sites import sites
from cmp_map import CONV_ISO, G
from descramble import ung
from pointmap_gen import gen, REC, coset
pk=sys.argv[1]; c=float(sys.argv[2]); gi=int(sys.argv[3]) if len(sys.argv)>3 else 0
t0=float(sys.argv[4]) if len(sys.argv)>4 else 15.0467
C,rm=REC['rec2_z.pkl']; TAB=gen(C,rm)
g=G[gi]; ij=sites(pk,c)
P,flip=CONV_ISO[0]
pts=[g(i,j) for i,j in ij]
inset=[p in TAB for p in pts]
print('symbols on the 512-point set:',sum(inset),'/',len(pts),' per 1000:',[sum(inset[i:i+1000]) for i in range(0,len(pts),1000)])
Z=[ung(*p) for p in pts]; d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]
bits=[]; ok=[]
for n in range(1,len(pts)):
    q=(d[n]-d[n-1])%4
    w=TAB.get(pts[n])
    if w is None: bits+= [q&1,q>>1]+[0]*6; ok.append(False)
    else: bits+= [q&1,q>>1]+[(w>>k)&1 for k in range(6)]; ok.append(True)
inb=[ (bits[n]^(bits[n-5] if n>=5 else 0)^(bits[n-23] if n>=23 else 0)) for n in range(len(bits))]
# ones fraction of descrambled stream per 500 symbols (4000 bits)
blk=[round(sum(inb[8*i:8*i+4000])/4000,2) for i in range(0,len(inb)//8,500)]
print('descrambled ones fraction per 500 symbols:',blk)
pickle.dump((inb,bits,ok,t0),open(pk.replace('.pkl','_bits.pkl'),'wb'))
