import sys; sys.path.insert(0,'.')
from dem import *
DEC=[[2,3,0,1],[0,2,1,3],[3,1,2,0],[1,0,3,2]]
def quad_stream(buf,t0,dur,off=0.75):
    zb=baseband(buf,t0,dur); ph,_=timing_phase(zb); s=symbols(zb,ph,off)[20:-20]
    # derotate in blocks of 400 symbols to follow slow carrier drift
    out=[]; prev=None
    for i in range(0,len(s),200):
        blk=s[i:i+200]; m4=sum((v/abs(v))**4 for v in blk if abs(v)>1e-9)
        ang=cmath.phase(m4)/4
        if prev is not None:
            while ang-prev>math.pi/4: ang-=math.pi/2
            while ang-prev<-math.pi/4: ang+=math.pi/2
        prev=ang; rot=cmath.exp(-1j*ang)
        out+= [int(round(cmath.phase(v*rot)/(math.pi/2)))%4 for v in blk]
    return out
def decode(quads,r,a,tap=4):
    st=[[0,2,3,1][(q+r)%4] for q in quads]
    reg=0; prev=0; words=[]
    k=a
    while k+8<=len(st):
        w=0
        for i in range(8):
            d=DEC[prev][st[k+i]]; prev=st[k+i]
            for b in (d&1,(d>>1)&1):
                o=(b^(reg>>tap)^(reg>>22))&1; reg=((reg<<1)|b)&0x7FFFFF
                w|=o<<(2*i) if b is d&1 and False else 0
            # recompute properly below
        k+=8
    return words
