"""GF(2)-linear parity-check fit on Ungerboeck bits: z0[n] = XOR over taps (lag 0..K, bit) of z_bit[n-lag]"""
import sys
from collections import Counter
from sites import ung
S='/private/tmp/claude-501/-Users-scottcryan-v90modem/8fb1ce38-0fb9-4688-a35e-483c585993ad/scratchpad/'
for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)):
    Z=ung(S+pk,c); K=4
    H=Counter()
    for n in range(K,len(Z)):
        w=0
        for k in range(1,K+1):
            for b in range(3): w|=Z[n-k][b]<<(3*(k-1)+b)
        H[(w,Z[n][0])]+=1
    tot=sum(H.values()); res=[]
    for m in range(1<<(3*K)):
        agree=sum(v for (w,p),v in H.items() if (bin(w&m).count('1')&1)==p)
        res.append((max(agree,tot-agree)/tot,m,agree*2>=tot))
    res.sort(reverse=True)
    print(pk)
    for acc,m,pos in res[:3]:
        print('   z0[n] = %sXOR('%('' if pos else '1^')+', '.join(f'z{b}[n-{k+1}]' for k in range(K) for b in range(3) if m>>(3*k+b)&1)+')  agrees %.2f%%'%(100*acc))
