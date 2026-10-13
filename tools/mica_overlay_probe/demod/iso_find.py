"""Which coder isomorphism does MICA use?  Compare cells of the C53's data memory with the label series sliced from
the same call's audio.  Inputs (not kept in the repo): iso_zz.pkl = equalized symbols of the call (gen.py), and
iso_dmw.txt = mica_line's DM_TRACE=14,0.2,8000,9fff output ('DMW addr value pc' lines) from the same run.
Result: cell 8c0c is the diff-encoder state d and 8c0e the convolutional state s; against CONV_ISO[0] they match
exactly (d identity, y0 = bit 2 of s), and s[i] == CONV[s[i-1]][d[i]] for 478 of 478 symbols."""
import sys, pickle
from collections import defaultdict, Counter
from sites import sites
from cmp_map import CONV_ISO, G
from descramble import ung
CONV=[[0,2,3,1],[4,7,5,6],[1,3,2,0],[7,4,6,5],[2,0,1,3],[6,5,7,4],[3,1,0,2],[5,6,4,7]]
ij=sites('iso_zz.pkl',0.48); g=G[2]
Z=[ung(*g(i,j)) for i,j in ij]
t0=12.0467
ser=defaultdict(list)
for l in open('/private/tmp/claude-501/-Users-scottcryan-v90modem/b442a30f-0d70-488d-a0ac-3b8d09113582/scratchpad/rig/iso_dmw.txt'):
    _,a,v,pc=l.split(); ser[int(a,16)].append(int(v,16))
n0=int((14.0-t0)*2400)
feats={}
for k,(P,flip) in enumerate(CONV_ISO):
    d=[P[z0][2*z2+z1] for z0,z1,z2 in Z]; y0=[z0^flip for z0,_,_ in Z]
    feats['iso%d d'%k]=d; feats['iso%d y0'%k]=y0
    feats['iso%d Q1'%k]=[((d[n]-d[n-1])%4)&1 for n in range(len(d))]
    feats['iso%d Q2'%k]=[((d[n]-d[n-1])%4)>>1 for n in range(len(d))]
# lattice-level (iso independent) features
feats['z0']=[z0 for z0,_,_ in Z]; feats['z1']=[z1 for _,z1,_ in Z]; feats['z2']=[z2 for _,_,z2 in Z]
res=[]
W=160
for a,s in ser.items():
    if len(s)<250 or len(set(s))>8 or len(set(s))<2: continue
    seq=s[:W]
    for name,f in feats.items():
        best=(0,0,0)
        for lag in range(-700,700):
            st=n0+lag
            if st<0 or st+W>len(f): continue
            x=f[st:st+W]
            # allow a value permutation by counting the best consistent mapping cell value -> feature value
            m=defaultdict(Counter)
            for cv,fv in zip(seq,x): m[cv][fv]+=1
            agree=sum(max(c.values()) for c in m.values())
            if agree>best[0]: best=(agree,lag,len(m))
        res.append((best[0],a,name,best[1],best[2],len(set(s))))
res.sort(reverse=True)
for r in res[:14]: print('consistent %d/%d cell %04x vs %-9s lag %d (cell values %d, distinct %d)'%(r[0],W,r[1],r[2],r[3],r[4],r[5]))

print('--- exact relation of the DSP cells to each isomorphism (lag 33)')
lag=33; W=300
cd=ser[0x8c0c][:W]; cs=ser[0x8c0e][:W]
for k,(P,flip) in enumerate(CONV_ISO):
    d=feats['iso%d d'%k][n0+lag:n0+lag+W]; y0=feats['iso%d y0'%k][n0+lag:n0+lag+W]
    md=defaultdict(Counter); ms=defaultdict(Counter)
    for c,x in zip(cd,d): md[c][x]+=1
    for c,x in zip(cs,y0): ms[c][x]+=1
    print('iso%d (flip %d, class perms %s / %s): DSP d-cell -> iso d: %s | DSP state-cell -> y0: %s'%(
        k,flip,P[0],P[1],{c:dict(v) for c,v in sorted(md.items())},{c:dict(v) for c,v in sorted(ms.items())}))

print('--- does the DSP state cell follow spandsp CONV table with the DSP d cell?')
cd=ser[0x8c0c]; cs=ser[0x8c0e]
n=min(len(cd),len(cs))
for off in (0,1,-1):
    ok=tot=0
    for i in range(1,n-1):
        j=i+off
        if 0<=j<n:
            tot+=1; ok+= cs[i]==CONV[cs[i-1]][cd[j]]
    print('d cell offset %+d: state[i] == CONV[state[i-1]][d[i+offset]] for %d / %d'%(off,ok,tot))
