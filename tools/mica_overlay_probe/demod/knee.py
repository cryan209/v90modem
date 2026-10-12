import sys, math
sys.path.insert(0,'.')
from dd import *
fn=sys.argv[1]; t=float(sys.argv[2]); d0=float(sys.argv[3]); rho=math.radians(float(sys.argv[4])); dur=float(sys.argv[5]); nsym=int(sys.argv[6])
x,w,phi,nu,n0,i0,L=setup(fn,t,dur)
z,w2,rr,rms=run_dd(x,w,phi,nu,n0,i0,L,nsym,d0,rho,passes=4)
z,_,_,_=run_dd(x,w2,phi,nu,n0,i0,L,nsym,d0,rr,passes=1,adapt=False,track=True)
# refine d and offsets on the final points
best=(0,)
for k in range(-30,31):
    dd=d0*(1+k*0.001); sc=0; off=[]
    for vals in ([v.real for v in z],[v.imag for v in z]):
        re=sum(math.cos(2*math.pi*v/dd) for v in vals); im=sum(math.sin(2*math.pi*v/dd) for v in vals)
        sc+=math.hypot(re,im)/len(vals); off.append(math.atan2(im,re)/(2*math.pi)*dd)
    if sc>best[0]: best=(sc,dd,off[0],off[1])
sc,dd,ox,oy=best
c=Counter()
for v in z: c[(round((v.real-ox)/dd),round((v.imag-oy)/dd))]+=1
cnt=sorted(c.values(),reverse=True)
print(fn,'symbols',len(z),'d=%.4f score %.2f'%(dd,sc/2),' distinct cells hit',len(cnt),' mean power %.3f'%(sum(abs(v)**2 for v in z)/len(z)))
print('ranked occupancy (rank: count):',' '.join(f'{r}:{cnt[r]}' for r in (0,50,100,200,300,400,450,480,500,510,520,540,560,580,620,700,800) if r<len(cnt)))
# knee: largest drop in smoothed counts between rank r and r+20
sm=[sum(cnt[i:i+10])/10 for i in range(0,len(cnt)-10)]
drops=sorted(((sm[i]-sm[i+20],i) for i in range(len(sm)-20)),reverse=True)[:3]
print('biggest drops over 20 ranks (drop, at rank):',[(round(a,1),b) for a,b in drops])
