import sys; sys.path.insert(0,'.')
from words import *
def decode(quads,r,a,tap=4):
    st=[[0,2,3,1][(q+r)%4] for q in quads]
    reg=0; prev=0; words=[]; k=a
    while k+8<=len(st):
        w=0
        for i in range(8):
            d=DEC[prev][st[k+i]]; prev=st[k+i]
            b0=(d&1); o0=(b0^(reg>>tap)^(reg>>22))&1; reg=((reg<<1)|b0)&0x7FFFFF
            b1=(d>>1)&1; o1=(b1^(reg>>tap)^(reg>>22))&1; reg=((reg<<1)|b1)&0x7FFFFF
            w|=o0<<(2*i); w|=o1<<(2*i+1)
        words.append(w); k+=8
    return words
def rle(ws):
    out=[]
    for w in ws:
        if out and out[-1][0]==w: out[-1][1]+=1
        else: out.append([w,1])
    return out
def best_runs(q,tap=4):
    best=(0,)
    for r in range(4):
        for a in range(8):
            ws=decode(q,r,a,tap)
            runs=rle(ws[4:])
            top=max(runs,key=lambda x:x[1])
            if top[1]>best[0]: best=(top[1],r,a,hex(top[0]),[ (hex(w),n) for w,n in runs if n>=3][:6])
    return best
if __name__=='__main__':
    f=sys.argv[1]; t0=float(sys.argv[2]); t1=float(sys.argv[3]); tap=int(sys.argv[4]) if len(sys.argv)>4 else 4
    buf=open(f,'rb').read()
    t=t0
    while t<t1:
        q=quad_stream(buf,t,1.5); print(f'{t:5.1f}s',best_runs(q,tap),flush=True); t+=1.5
