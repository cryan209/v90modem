import sys, math, pickle
sys.path.insert(0,'.')
from v34nl import phi, inv
def near(v,d,tol):
    f=((v+0.5*d)/d)%1.0; return min(f,1-f)<tol
def local(vals, centre, width, lo=0.085, hi=0.145, n=121):
    sel=[(v,0.5+0.5*math.cos(2*math.pi*(v-centre)/width)) for v in vals if abs(v-centre)<width/2]
    W=sum(w for _,w in sel)
    if W<=0: return 0,0,0
    best=(0,0)
    for k in range(n):
        d=lo+(hi-lo)*k/(n-1)
        re=sum(w*math.cos(2*math.pi*v/d) for v,w in sel); im=sum(w*math.sin(2*math.pi*v/d) for v,w in sel)
        sc=math.hypot(re,im)/W
        if sc>best[0]: best=(sc,d)
    return best[1],best[0],len(sel)
def radial_curve(z, d_ref=0.1010, radii=(0.0,0.3,0.6,0.9,1.15)):
    out=[]
    # x along central rows and y along central columns, both signs: four independent estimates per radius
    vx=[v.real for v in z if abs(v.imag)<0.12 and near(v.imag,d_ref,0.22)]
    vy=[v.imag for v in z if abs(v.real)<0.12 and near(v.real,d_ref,0.22)]
    for c in radii:
        est=[]
        for vals in (vx,vy):
            for sg in ((1,-1) if c>0 else (1,)):
                d,s,n=local(vals,sg*c,0.5)
                if s>0.5: est.append(d)
        out.append((c,sum(est)/len(est) if est else 0,len(est)))
    return out
def model(r_obs,c):
    rl=inv(r_obs,c); zeta=c*rl*rl
    ph=phi(zeta); dph=1/6+zeta/60
    return ph, ph+2*zeta*dph
def fit_c(curve):
    d0=curve[0][1]
    meas=[(r,d/d0) for r,d,n in curve if r>0 and d>0]
    best=None; rows=[]
    for k in range(0,81):
        c=0.10+0.01*k
        chi=sum((model(r,c)[1]-m)**2 for r,m in meas)
        rows.append((c,chi))
        if best is None or chi<best[0]: best=(chi,c)
    # 1-sigma-ish interval: where chi2 <= 2*chi_min + noise floor (0.5% per point => 4*(0.005)^2)
    floor=4*0.005**2
    ok=[c for c,chi in rows if chi<=best[0]+floor*2]
    return best[1],min(ok),max(ok),meas,d0
if __name__=='__main__':
    for lab,pk in (('rec1 (orig analysis)','terbo_z.pkl'),('rec1 (gen pipeline)','rec1_z.pkl'),('rec2 (B14 only, start 4.5 s)','rec2_z.pkl')):
        z=pickle.load(open(pk,'rb'))
        cv=radial_curve(z)
        c,lo,hi,meas,d0=fit_c(cv)
        print(lab)
        print('   centre spacing d0=%.4f; ratios:'%d0,' '.join('%.2f:%.3f'%(r,m) for r,m in meas))
        print('   best V.34-family c=%.2f   (interval %.2f-%.2f)   predicted ratios:'%(c,lo,hi),' '.join('%.3f'%model(r,c)[1] for r,_ in meas))
