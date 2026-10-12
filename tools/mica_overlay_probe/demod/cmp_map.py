"""Compare MICA's recovered subset labelling with spandsp's V.32bis 14400 constellation map
(index = data<<3 | d<<1 | y0, coordinates in lattice units)."""
import re, itertools
H=open('/Users/scottcryan/v90modem/spandsp-master/src/v17_v32bis_tx_constellation_maps.h').read()
body=H[H.index('v17_v32bis_14400_constellation[128]'):]
body=body[:body.index('};')]
pts=[(float(a),float(b)) for a,b in re.findall(r'SCALE\(\s*(-?[\d.]+)f\),\s*FP_CONSTELLATION_SCALE\(\s*(-?[\d.]+)f',body)]
assert len(pts)==128
V32={(int(x),int(y)):((i>>1)&3,i&1) for i,(x,y) in enumerate(pts)}
def coset(i,j): return (i%2,(j-2*(i//2))%4)                       # (a,b): Z^2 / 2RZ^2
def ung(i,j):
    p=(i+j)&1; ip=i-p; z1=ip&1; i2=ip-z1; j2=j-z1
    return (p,z1,((i2+j2)//2)&1)
CONV_ISO=[(((0,2,1,3),(3,2,0,1)),0),(((1,3,2,0),(0,3,1,2)),1),(((2,0,3,1),(1,0,2,3)),0),(((3,1,0,2),(2,1,3,0)),1)]
G=[lambda x,y:(x,y),lambda x,y:(-y,x),lambda x,y:(-x,-y),lambda x,y:(y,-x),
   lambda x,y:(-x,y),lambda x,y:(y,x),lambda x,y:(x,-y),lambda x,y:(-y,-x)]
hits=[]
for P,flip in CONV_ISO:
    mica={}      # coset -> (d, y0) as the recovered coder sees it
    for i in range(8):
        for j in range(8):
            z0,z1,z2=ung(i,j); mica[coset(i,j)]=(P[z0][2*z2+z1],z0^flip)
    for gi,g in enumerate(G):
        for sx in range(4):
            for sy in range(4):
                # consistent, bijective map from V.32bis points to MICA labels?
                m={}; ok=True
                for (x,y),lab in V32.items():
                    gx,gy=g(x,y); c=coset(gx+sx,gy+sy)
                    if m.setdefault(c,lab)!=lab: ok=False;break
                if not ok: continue
                # labels: must also cover the 8 (d,y0) pairs bijectively
                if len(set(m.values()))!=8: continue
                # relation d_mica vs d_v32 per coset: is it a fixed permutation of d (same for y0 classes)?
                rel={}
                for c,lab in m.items():
                    rel.setdefault((mica[c][1],lab[1]),set()).add((mica[c][0],lab[0]))
                hits.append((P,flip,gi,sx,sy,{k:sorted(v) for k,v in rel.items()}))
print('geometric alignments for which V.32bis labels are a function of MICA cosets:',len(hits))
for h in hits[:6]: print(' ',h)

print('--- which index-8 sublattices make the V.32bis labels (y0,d) and y0 alone, d alone constant on cosets?')
def hnf_cosets(a,b,cc):
    return lambda i,j:(i%a,(j-(i//a)*b)%cc)
for a in (1,2,4,8):
    cc=8//a
    for b in range(cc if a>1 else 1):
        f=hnf_cosets(a,b,cc)
        for name,sel in (('(d,y0)',lambda l:l),('y0',lambda l:l[1]),('d',lambda l:l[0])):
            m={}; ok=True
            for (x,y),lab in V32.items():
                if m.setdefault(f(x,y),sel(lab))!=sel(lab): ok=False;break
            if ok: print('  L=<(%d,%d),(0,%d)>'%(a,b,cc),name,'is a function of the coset')
