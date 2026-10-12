import sys, math, pickle
sys.path.insert(0,'.')
from v34nl import phi, inv
z=pickle.load(open('terbo_z.pkl','rb'))
d_ref=0.1010
def lattice_off(vals_other, d):
    # offset of the perpendicular lattice (sites at (k+0.5)d => -0.5d) assumed odd lattice
    return -0.5*d
def near(v,d,tol):
    f=((v+0.5*d)/d)%1.0; return min(f,1-f)<tol
def local(vals, centre, width, lo=0.085, hi=0.140, n=111):
    sel=[(v,0.5+0.5*math.cos(2*math.pi*(v-centre)/width)) for v in vals if abs(v-centre)<width/2]
    W=sum(w for _,w in sel); best=(0,0)
    for k in range(n):
        d=lo+(hi-lo)*k/(n-1)
        re=sum(w*math.cos(2*math.pi*v/d) for v,w in sel); im=sum(w*math.sin(2*math.pi*v/d) for v,w in sel)
        sc=math.hypot(re,im)/W
        if sc>best[0]: best=(sc,d)
    return best[1],best[0],len(sel)
# rows with the perpendicular coordinate on a lattice line.  d for the row selection: use a mid value; tolerance wide enough for the warp
rad=[];tan=[]
for c in (0.0,0.3,0.6,0.9,1.15):
    # radial: x spacing on central rows, window centred at +-c (fold both sides)
    vals=[v.real for v in z if abs(v.imag)<0.12 and near(v.imag,d_ref,0.22)]
    dp=local(vals,c,0.5); dn=local(vals,-c,0.5)
    rad.append((c,dp,dn))
for c in (0.0,0.3,0.6,0.9,1.15):
    # tangential: x spacing near x=0 on rows at |y|~c  (both signs)
    out=[]
    for sg in (1,-1):
        vals=[v.real for v in z if abs(v.imag-sg*c)<0.12 and near(v.imag,d_ref,0.3) and abs(v.real)<0.4]
        out.append(local(vals,0.0,0.5) if len(vals)>200 else (0,0,len(vals)))
    tan.append((c,out[0],out[1]))
print('radial spacing (x along central rows):')
for c,dp,dn in rad: print('  |x|~%.2f  +: d=%.4f (score %.2f, n=%d)   -: d=%.4f (%.2f, %d)'%(c,dp[0],dp[1],dp[2],dn[0],dn[1],dn[2]))
print('tangential spacing (x near x=0 on rows at |y|~c):')
for c,a,b in tan: print('  |y|~%.2f  +: d=%.4f (%.2f, %d)   -: d=%.4f (%.2f, %d)'%(c,a[0],a[1],a[2],b[0],b[1],b[2]))
pickle.dump((rad,tan),open('curves.pkl','wb'))
