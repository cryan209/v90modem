import sys, math, pickle
sys.path.insert(0,'.')
from warp2 import *
fn='mono_2_0x6000_p1.bin'; d0=0.108
z=equalized(fn,d0)
pickle.dump(z,open('terbo_z.pkl','wb'))
d=0.1058; oy=-0.5*d; ox=-0.5*d
def near(v,off,dd,tol=0.18):
    f=((v-off)/dd)%1.0; return min(f,1-f)<tol
# radial: x along central rows (|y|<0.2) at |x| windows ; tangential: x along rows at larger |y| near x=0 window
print('radial   (x spacing on rows with |y|<0.25):')
for c in (0.0,0.5,0.9,1.15):
    vals=[v.real for v in z if abs(v.imag)<0.25 and near(v.imag,oy,d,0.2)]
    vals=[abs(v) for v in vals]       # fold sign (symmetric)
    # use signed windows on both sides
    both=[v.real for v in z if abs(v.imag)<0.25 and near(v.imag,oy,d,0.2)]
    s=[(x if c>=0 else x) for x in both]
    dp,sp,n=local_d([x for x in both],c,0.7,d); dn,sn,nn=local_d([x for x in both],-c,0.7,d)
    print('  |x|~%.2f: +side d=%.4f (%.2f,%d)  -side d=%.4f (%.2f,%d)'%(c,dp,sp,n,dn,sn,nn))
print('tangential (x spacing near x=0 on rows at various |y|):')
for yc in (0.0,0.4,0.8,1.1):
    for sgn in (+1,-1):
        vals=[v.real for v in z if abs(v.imag-sgn*yc)<0.15 and near(v.imag,oy,d,0.2)]
        if len(vals)<300: continue
        dd_,s_,n_=local_d(vals,0.0,0.7,d)
        print('  y~%+.2f: d=%.4f (%.2f,%d)'%(sgn*yc,dd_,s_,n_))
