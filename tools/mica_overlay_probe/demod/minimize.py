"""Moore-minimise the coder implied by the data.  History h=(p3, l2, l1) (l = full Ungerboeck label);
output = z0 of the next symbol (majority vote); input = next label.  Equivalent histories => states."""
import sys, itertools
from collections import Counter, defaultdict
from sites import ung
S='/private/tmp/claude-501/-Users-scottcryan-v90modem/8fb1ce38-0fb9-4688-a35e-483c585993ad/scratchpad/'
Z=ung(S+'rec2_z.pkl',0.48)
t=defaultdict(Counter)
for n in range(3,len(Z)):
    t[(Z[n-3][0],Z[n-2],Z[n-1])][Z[n][0]]+=1
labs=list(itertools.product((0,1),repeat=3))
H=[(p3,l2,l1) for p3 in (0,1) for l2 in labs for l1 in labs]
out={h:(0 if t[h][0]>=t[h][1] else 1) for h in H}
conf=sum(min(t[h].values()) if len(t[h])>1 else 0 for h in H)
print('histories',len(H),' minority (noise) count',conf,'of',sum(sum(t[h].values()) for h in H))
# next history after label l: (p3',l2',l1') = (l2[0], l1, l)
def nxt(h,l): return (h[1][0],h[2],l)
cls={h:out[h] for h in H}
while True:
    sig={h:(cls[h],)+tuple(cls[nxt(h,l)] for l in labs) for h in H}
    ids={}; new={h:ids.setdefault(sig[h],len(ids)) for h in H}
    if len(set(new.values()))==len(set(cls.values())): break
    cls=new
print('equivalence classes (states) after minimisation:',len(set(cls.values())))

# ---- extract the 8-state machine
rep={}
for h in H: rep.setdefault(cls[h],h)
order=sorted(rep)
names={c:i for i,c in enumerate(order)}
def st(h): return names[cls[h]]
# input to the machine is the label z1,z2 (z0 is forced by the state); transition on label l
print('state: output z0 | next state for (z1,z2) = 00 10 01 11 (z1 z2 order: z1 + 2*z2 -> columns)')
Tm={}
for c in order:
    h=rep[c]
    row=[]
    for z2 in (0,1):
        for z1 in (0,1):
            l=(out[h],z1,z2); row.append(st(nxt(h,l)))
    Tm[names[c]]=(out[h],row)
    print(' ',names[c],':',out[h],'|',row)
# consistency: every history in a class gives the same row
bad=0
for h in H:
    s=st(h); o,row=Tm[s]
    r=[st(nxt(h,(out[h],z1,z2))) for z2 in (0,1) for z1 in (0,1)]
    if r!=row or out[h]!=o: bad+=1
print('histories inconsistent with their class row:',bad)
# ---- isomorphism with spandsp CONV (Moore, y0=bit2 of current state), inputs d = bijection of (z1,z2)
CONV=[[0,2,3,1],[4,7,5,6],[1,3,2,0],[7,4,6,5],[2,0,1,3],[6,5,7,4],[3,1,0,2],[5,6,4,7]]
found=[]
for perm in itertools.permutations(range(4)):          # data column k -> CONV input perm[k]
    for s0 in range(8):                                  # image of our state 0
        for flip in (0,1):
            m={0:s0}; stack=[0]; ok=True
            while stack and ok:
                a=stack.pop(); b=m[a]
                if ((b>>2)&1)^flip!=Tm[a][0]: ok=False;break
                for k in range(4):
                    a2=Tm[a][1][k]; b2=CONV[b][perm[k]]
                    if a2 in m:
                        if m[a2]!=b2: ok=False;break
                    else: m[a2]=b2; stack.append(a2)
            if ok and len(m)==8 and len(set(m.values()))==8: found.append((perm,s0,flip))
print('isomorphisms to spandsp V.32bis CONV (old-state Moore):',len(found),found[:4])

# ---- is it GF(2)-linear?  next(s,k) = A s ^ B k, z0 = bit1(s)
Bk=[Tm[0][1][k] for k in range(4)]
A={e:Tm[e][1][0] for e in (1,2,4)}
def Ap(s): 
    r=0
    for e in (1,2,4):
        if s&e: r^=A[e]
    return r
lin=all(Tm[s][1][k]==Ap(s)^Bk[k] for s in range(8) for k in range(4))
print('next(s,k) = A s ^ B k exactly:',lin,' A(e1,e2,e4)=',[A[e] for e in (1,2,4)],' B(k)=',Bk,' z0 = bit1(s) for all:',all(Tm[s][0]==(s>>1)&1 for s in range(8)))
# replay the machine on the observed labels from every start state, count agreement of predicted z0
best=0
for s0 in range(8):
    s=s0; ok=0
    for z0,z1,z2 in Z:
        ok+=(Tm[s][0]==z0); s=Tm[s][1][2*z2+z1]
    best=max(best,ok)
print('machine replayed over the whole 14000-symbol record: z0 predicted %d / %d = %.2f%%'%(best,len(Z),100*best/len(Z)))

# ---- transition-graph isomorphism with CONV ignoring outputs; then what is the output function in CONV's states?
iso=[]
for perm in itertools.permutations(range(4)):
    for s0 in range(8):
        m={0:s0}; stack=[0]; ok=True
        while stack and ok:
            a=stack.pop(); b=m[a]
            for k in range(4):
                a2=Tm[a][1][k]; b2=CONV[b][perm[k]]
                if a2 in m:
                    if m[a2]!=b2: ok=False;break
                else: m[a2]=b2; stack.append(a2)
        if ok and len(m)==8 and len(set(m.values()))==8: iso.append((perm,s0,dict(m)))
print('transition-graph isomorphisms to CONV:',len(iso))
for perm,s0,m in iso[:3]:
    outs={m[a]:Tm[a][0] for a in m}
    print('  input map',perm,' state map',m,' output on CONV states',[outs[s] for s in range(8)])

# ---- allow the label->d bijection to differ per output class; CONV output = bit0 of the current state (y0 = bit2 of conv[s][d] == s&1)
def try_iso(outbit):
    res=[]
    for perm0 in itertools.permutations(range(4)):
        for perm1 in itertools.permutations(range(4)):
            P=(perm0,perm1)
            for s0 in range(8):
                for flip in (0,1):
                    m={0:s0}; stack=[0]; ok=True
                    while stack and ok:
                        a=stack.pop(); b=m[a]
                        if (((b>>outbit)&1)^flip)!=Tm[a][0]: ok=False;break
                        pm=P[Tm[a][0]]
                        for k in range(4):
                            a2=Tm[a][1][k]; b2=CONV[b][pm[k]]
                            if a2 in m:
                                if m[a2]!=b2: ok=False;break
                            else: m[a2]=b2; stack.append(a2)
                    if ok and len(m)==8 and len(set(m.values()))==8: res.append((P,s0,flip))
    return res
for ob in (0,1,2):
    r=try_iso(ob); print('CONV output = bit%d of state, per-class input bijections: %d isomorphisms'%(ob,len(r)), r[:1])

# ---- replay spandsp's coder directly on the lattice labels, for every isomorphism found
r=try_iso(0)
print('--- direct replay of CONV (state'' = CONV[s][d], y0 = bit2(state'') = bit0(s)) on the sliced labels')
for P,s0,flip in r:
    best=(0,)
    for init in range(8):
        s=init; ok=0
        for z0,z1,z2 in Z:
            ok+=((s&1)^flip)==z0
            d=P[z0][2*z2+z1]; s=CONV[s][d]
        best=max(best,(ok,init))
    print('  perm class0 (k=z1+2*z2 -> d):',P[0],' class1:',P[1],' flip',flip,' -> y0 predicted %d/%d = %.2f%%'%(best[0],len(Z),100*best[0]/len(Z)))
