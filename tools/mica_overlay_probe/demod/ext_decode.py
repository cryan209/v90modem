import sys; sys.path.insert(0,'.')
from trn import *
def words_from(st,start,a,tap=4):
    reg=0; prev=st[start+a-1] if start+a>0 else 0; out=[]; k=start+a
    while k+8<=len(st):
        w=0
        for i in range(8):
            d=DEC[prev][st[k+i]]; prev=st[k+i]
            b0=d&1; o0=(b0^(reg>>tap)^(reg>>22))&1; reg=((reg<<1)|b0)&0x7FFFFF
            b1=(d>>1)&1; o1=(b1^(reg>>tap)^(reg>>22))&1; reg=((reg<<1)|b1)&0x7FFFFF
            w|=(o0<<(2*i))|(o1<<(2*i+1))
        out.append(w); k+=8
    return out
if __name__=='__main__':
    fn=sys.argv[1]; t0=8.4; dur=float(sys.argv[2])
    buf=open(fn,'rb').read()
    q=quad_stream(buf,t0,dur); ex=trn_expected(1500)
    best=(0,)
    for r in range(4):
        st=states(q,r)
        for p in range(300,700):
            m=sum(1 for i in range(300) if st[p+i]==ex[i])
            if m>best[0]: best=(m,r,p)
    m,r,p=best; st=states(q,r)
    print('TRN match',best, 'start t=',round(t0+(20+p)/2400,3))
    ex=trn_expected(len(st)-p)
    prof=[]
    for i in range(0,len(st)-p-100,100):
        f=sum(1 for j in range(100) if st[p+i+j]==ex[i+j])
        prof.append(f)
    print('TRN-match per 100 symbols (100=exact TRN):',prof)
    # first symbol where match collapses (two consecutive windows < 60)
    for i,f in enumerate(prof[:-1]):
        if f<60 and prof[i+1]<60: end=p+i*100; break
    else: end=len(st)-50
    print('TRN ends near symbol',end,'t=',round(t0+(20+end)/2400,3),'s')
    for a in range(8):
        ws=words_from(st,end-8,a); rl=rle(ws)
        print(f'align {a}:',[(hex(w),n) for w,n in rl if n>=3][:8],' words',len(ws))
