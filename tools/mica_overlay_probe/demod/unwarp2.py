import sys, math, pickle, random
sys.path.insert(0,'.')
from unwarp import *
random.seed(2)
sub=random.sample(z,6000)
res=[]
for a in [x*0.005 for x in range(10,23)]:
    for b in [0.0,0.005,0.01,0.015,0.02,0.03]:
        u=unwarp(sub,a,b); sc,d,ox,oy=best_d(u,0.097,0.106,37); res.append((sc,a,b,d))
res.sort(reverse=True)
for sc,a,b,d in res[:6]: print('score %.3f  a=%.3f b=%.3f  d0=%.4f'%(sc,a,b,d))
sc,a,b,d=res[0]
u=unwarp(z,a,b)
sc2,dd,ox,oy=best_d(u,d*0.98,d*1.02,41)
c=Counter(); err=0
for v in u:
    i=round((v.real-ox)/dd); j=round((v.imag-oy)/dd); c[(i,j)]+=1
    err+=(v.real-ox-i*dd)**2+(v.imag-oy-j*dd)**2
cnt=sorted(c.values(),reverse=True)
print('unwarped: a=%.3f b=%.3f d0=%.4f score %.2f rms to site %.3f d  offsets (%.2f,%.2f)d  cells hit %d'%(a,b,dd,sc2,math.sqrt(err/len(u))/dd,ox/dd,oy/dd,len(cnt)))
print('ranked occupancy:',' '.join(f'{r}:{cnt[r]}' for r in (0,50,100,200,300,400,450,480,500,510,520,540,560,600) if r<len(cnt)))
sm=[sum(cnt[i:i+10])/10 for i in range(0,len(cnt)-10)]
drops=sorted(((sm[i]-sm[i+20],i+20) for i in range(len(sm)-20)),reverse=True)[:4]
print('biggest drops (drop, rank):',[(round(x,1),y) for x,y in drops])
pickle.dump((a,b,dd,ox,oy,u),open('unwarped.pkl','wb'))
