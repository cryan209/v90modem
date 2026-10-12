"""With the scrambler register known, predict the six uncoded bits of every symbol and check they are a function
of the lattice site (i.e. the site is chosen by the scrambled-ones word).  Run on the clean region."""
import random, pickle
from collections import Counter, defaultdict
from register import masks, solve, evalreg
from register3 import getQ, START
def run(pk,c,gi):
    ij,Z,d,Q=getQ(pk,c,gi); s0=START[pk]; n=len(Q)-s0
    M=masks(8*n+8)
    bits=[]
    for m,q in enumerate(Q[s0:s0+300]): bits+=[(8*m,q&1),(8*m+1,q>>1)]
    random.seed(3)
    while True:
        reg=solve([(M[p],v) for p,v in random.sample(bits,28)])
        if reg is not None and all(evalreg(reg,M[p])==v for p,v in bits): break
    # symbol m of the clean region is ij[s0+1+m]  (Q[n] belongs to symbol n+1)
    word=lambda m:sum(evalreg(reg,M[8*m+k])<<(k-2) for k in range(2,8))
    ok=sum(1 for m in range(n) for k in (0,1) if evalreg(reg,M[8*m+k])==((Q[s0+m]>>k)&1))
    print(pk,'register %#07x; Q1,Q2 reproduced %d/%d over %d symbols'%(reg,ok,2*n,n))
    site_words=defaultdict(Counter)
    for m in range(n): site_words[ij[s0+1+m]][word(m)]+=1
    tot=sum(sum(c.values()) for c in site_words.values()); pure=sum(max(c.values()) for c in site_words.values())
    print('  sites seen %d; symbols whose 6-bit word equals the most common word for their site: %d/%d = %.1f%%'%(len(site_words),pure,tot,100*pure/tot))
    # contrast: shuffle words across symbols
    ws=[word(m) for m in range(n)]; random.shuffle(ws); sw=defaultdict(Counter)
    for m in range(n): sw[ij[s0+1+m]][ws[m]]+=1
    print('  control (words shuffled between symbols): %.1f%%'%(100*sum(max(c.values()) for c in sw.values())/tot))
    return site_words
if __name__=='__main__':
    for a in (('rec2_z.pkl',0.48,0),('rec1_z.pkl',0.49,1)): run(*a)
