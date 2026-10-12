"""Six-bit point map: within each coset the word is the rank of the point's distance from the constellation centre.
Find the exact centre and tie-break by search, on both recordings."""
import json, math, itertools
from collections import defaultdict
T=json.load(open('terbo_site_words.json'))
def cosets(S):
    c=defaultdict(list)
    for (i,j),w in S.items(): c[(i%2,(j-2*(i//2))%4)].append((i,j,w))
    return c
def check(S,center,tb):
    bad=0
    for k,L in cosets(S).items():
        key=lambda t:(round((t[0]-center[0])**2+(t[1]-center[1])**2,6),)+tb(t[0]-center[0],t[1]-center[1])
        order=sorted(L,key=key)
        bad+=sum(1 for n,t in enumerate(order) if t[2]!=n)
    return bad
for name in T:
    S={(i,j):w for i,j,w in T[name]}
    xs=[i for i,j in S]; ys=[j for i,j in S]
    print(name,'x range',min(xs),max(xs),' y range',min(ys),max(ys))
    for center in ((0.5,0.5),(-0.5,0.5),(0.5,-0.5),(-0.5,-0.5),(0,0),(1,0),(0,1),(1,1),(0.5,0),(0,0.5)):
        # radius-only: count words out of place if ties could be arbitrary (compare multiset per group)
        bad=0
        for k,L in cosets(S).items():
            order=sorted(L,key=lambda t:round((t[0]-center[0])**2+(t[1]-center[1])**2,6))
            # a word is "wrong" if its r2-rank block (positions with equal r2) does not contain it
            r2=[round((t[0]-center[0])**2+(t[1]-center[1])**2,6) for t in order]
            blocks=defaultdict(set)
            for n,t in enumerate(order): blocks[r2[n]].add(n)
            bad+=sum(1 for n,t in enumerate(order) if t[2] not in blocks[r2[n]])
        print('   centre',center,': words outside their equal-radius block:',bad,'of 512')

print('--- tie-break search (rec2, centre (0.5,0.5))')
S={(i,j):w for i,j,w in T['rec2_z.pkl']}; C=(0.5,0.5)
def exact(tb):
    bad=0
    for k,L in cosets(S).items():
        key=lambda t:(round((t[0]-C[0])**2+(t[1]-C[1])**2,6),)+tb(t[0]-C[0],t[1]-C[1])
        bad+=sum(1 for n,t in enumerate(sorted(L,key=key)) if t[2]!=n)
    return bad
res=[]
for sx,sy in itertools.product((1,-1),repeat=2):
    for swap in (0,1):
        for second in (0,1):
            def tb(x,y,sx=sx,sy=sy,swap=swap,second=second):
                a,b=(sx*x,sy*y) if not swap else (sy*y,sx*x)
                return (a,b)
            res.append((exact(tb),'lex sx=%d sy=%d swap=%d'%(sx,sy,swap)))
for d in (1,-1):
    for th in range(0,360,5):
        t0=math.radians(th)
        def tb(x,y,d=d,t0=t0):
            a=(d*(math.atan2(y,x)-t0))%(2*math.pi)
            return (round(a,6),)
        res.append((exact(tb),'angle dir=%d start=%d deg'%(d,th)))
res.sort()
for r in res[:6]: print(' ',r)
