import sys; sys.path.insert(0,'.')
from scan import *
def scr_bit(reg,tap,inp):
    out=(inp^(reg>>tap)^(reg>>22))&1
    return out,((reg<<1)|out)&0x7FFFFF
def trn_expected(n,tap=4):
    reg=0; ex=[]
    for pos in range(n):
        b0,reg=scr_bit(reg,tap,1); b1,reg=scr_bit(reg,tap,1)
        ex.append((2 if b0 else 0) if False else None)
        ex[-1]=(3 if b0 else 0) if pos<256 else (b0|(b1<<1))   # C=3 / A=0 (state numbering A0 B1 D2 C3)
    return ex
def states(q,r): return [[0,2,3,1][(x+r)%4] for x in q]
if __name__=='__main__':
    buf=open(sys.argv[1],'rb').read(); t0=float(sys.argv[2]); dur=float(sys.argv[3])
    q=quad_stream(buf,t0,dur); ex=trn_expected(1500)
    best=(0,)
    for r in range(4):
        st=states(q,r)
        for p in range(0,len(st)-600):
            m=sum(1 for i in range(300) if st[p+i]==ex[i])
            if m>best[0]: best=(m,r,p)
    print('best TRN match (of 300 first symbols), rot, start symbol:',best,' => t=',t0+ (20+best[2])/2400)
