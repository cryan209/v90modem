"""Which parts of the 3 previous labels does the next checkerboard parity depend on?"""
import sys
from collections import Counter, defaultdict
from trellis4 import sites, bits
S=sys.argv[1]
FUN={'full':lambda l:l,'drop':lambda l:0,'p':lambda l:l[0]^l[1],'a':lambda l:l[0],'b0':lambda l:l[1],'b1':lambda l:l[2],
     'b':lambda l:(l[1],l[2]),'a+b1':lambda l:l[0]^l[2],'p,b1':lambda l:(l[0]^l[1],l[2]),'a,b1':lambda l:(l[0],l[2]),'a,b0':lambda l:(l[0],l[1])}
def err(L,p,fs):
    ctx=defaultdict(Counter)
    for n in range(3,len(L)): ctx[tuple(f(L[n-3+k]) for k,f in enumerate(fs))][p[n]]+=1
    return sum(sum(c.values())-max(c.values()) for c in ctx.values())/(len(L)-3)*100, len(ctx)
for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)):
    ij=sites(S+'/'+pk,c); p=[(i+j)&1 for i,j in ij]; L=[bits(i,j) for i,j in ij]
    print(pk)
    for lag in (0,1,2):       # index 0 = oldest (n-3), 2 = newest (n-1)
        row=[]
        for name,f in FUN.items():
            fs=[FUN['full']]*3; fs[lag]=f
            e,nc=err(L,p,fs); row.append(f'{name}:{e:.1f}')
        print(f'  lag n-{3-lag} reduced to ->',' '.join(row))
