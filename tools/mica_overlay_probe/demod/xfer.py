import sys, math
sys.path.insert(0,'.')
from dd import *
t=11.5; dur=6.0; nsym=14000
xs,ws,phs,nus,n0s,i0s,L=setup('mono_2_0.bin',t,dur)
zs,w2s,rrs,rms=run_dd(xs,ws,phs,nus,n0s,i0s,L,nsym,0.2256,math.radians(12),passes=4)
print('std DD-converged: rms %.3f d, rotation %.2f deg'%(rms,math.degrees(rrs)))
xp,wp,php,nup,n0p,i0p,Lp=setup('mono_2_0x6000_p1.bin',t,dur)
print('alignment identical:',n0s==n0p, i0s==i0p)
for lab,wt in (('TRN-trained',wp),('std-DD-converged',w2s)):
    for rot in (rrs,):
        z,_,rr,rms=run_dd(xp,wt,php,nup,n0p,i0p,L,nsym,0.1070,rot,passes=1,adapt=False,track=True,kp=0.02)
        best=(0,)
        for k in range(-30,31):
            dd=0.1070*(1+k*0.001); sc=0; off=[]
            for vals in ([v.real for v in z],[v.imag for v in z]):
                re=sum(math.cos(2*math.pi*v/dd) for v in vals); im=sum(math.sin(2*math.pi*v/dd) for v in vals)
                sc+=math.hypot(re,im)/len(vals); off.append(math.atan2(im,re)/(2*math.pi)*dd)
            if sc>best[0]: best=(sc,dd,off[0],off[1])
        sc,dd,ox,oy=best
        c=Counter(); err=0
        for v in z:
            i=round((v.real-ox)/dd); j=round((v.imag-oy)/dd); c[(i,j)]+=1
            err+=(v.real-ox-i*dd)**2+(v.imag-oy-j*dd)**2
        cnt=sorted(c.values(),reverse=True)
        print(f'{lab}: d={dd:.4f} lattice score {sc/2:.2f}  rms to nearest site {math.sqrt(err/len(z))/dd:.3f} d  cells hit {len(cnt)}  P={sum(abs(v)**2 for v in z)/len(z):.3f}')
        print('   ranked occupancy:',' '.join(f'{r}:{cnt[r]}' for r in (0,100,200,300,400,450,480,500,510,520,540,560,580,620) if r<len(cnt)))
        sm=[sum(cnt[i:i+10])/10 for i in range(0,len(cnt)-10)]
        drops=sorted(((sm[i]-sm[i+20],i+20) for i in range(len(sm)-20)),reverse=True)[:3]
        print('   biggest drops (drop, rank):',[(round(a,1),b) for a,b in drops])
