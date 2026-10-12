import sys, math, pickle
sys.path.insert(0,'.')
from v34nl import unwarp_c
from unwarp import best_d
from collections import Counter
for lab,pk,c in (('rec1','rec1_z.pkl',0.49),('rec2','rec2_z.pkl',0.48)):
    z=pickle.load(open(pk,'rb'))
    u=unwarp_c(z,c)
    sc,d,ox,oy=best_d(u,0.094,0.108,57)
    cc=Counter()
    for v in u: cc[(round((v.real-ox)/d),round((v.imag-oy)/d))]+=1
    n=len(u); mean=n/512
    E=sum(abs(v)**2 for v in u)/n
    counts={f:sum(1 for v in cc.values() if v>=f*mean) for f in (0.25,0.35,0.5,0.65)}
    keep=[k for k,v in cc.items() if v>=0.5*mean]
    cx=sum(i for i,j in keep)/len(keep); cy=sum(j for i,j in keep)/len(keep)
    R=[0]*12
    for i,j in keep:
        a=int(((math.atan2(j-cy,i-cx)+math.pi)/(2*math.pi))*12)%12; R[a]=max(R[a],math.hypot(i-cx,j-cy))
    rows=len({j for i,j in keep})
    # radial bands: mean count per kept site
    rs=sorted((math.hypot(i-cx,j-cy),cc[(i,j)]) for i,j in keep)
    bands=[]
    for lo,hi in ((0,4),(4,8),(8,10),(10,12),(12,14)):
        sel=[nn for r,nn in rs if lo<=r<hi]; bands.append(round(sum(sel)/len(sel),1) if sel else 0)
    Rdisc=12.75*d
    print(f'{lab}: c={c}  d0={d:.4f} score {sc:.2f}  E(unwarped)={E:.3f}  kappa=c*E={c*E:.3f}  (V.34 constant 0.3125)')
    print(f'   sites >= 0.25/0.35/0.5/0.65 x mean count {mean:.1f}: {counts}   rows kept {rows}   sector max radius {min(R):.1f}-{max(R):.1f} cells   band means {bands}')
