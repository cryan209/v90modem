#!/usr/bin/env python3
"""Score a V.34 receiver's Phase 4 TRN decode against the transmitter's own symbols.

Every other measurement of this receiver's Phase 4 health in this tree has
turned out to be self-referential -- 4th-power coherence over the symbol dump
reads the same in a run that completes with zero errors and one that never
trains, and the [EQ] decision error targets an angle integrated from the
received dibit.  This one is not: v34_duplex_test runs both modems in one
process, so the transmitter's own TRN symbol indices can be dumped and the
receiver's decoded dibits scored directly against them.

    V34_P4TRN_TX_DUMP=tx.txt V34_P4TRN_RX_DUMP=rx.txt ./v34_duplex_test 3200 9600 ulaw
    tools/v34_trn_ground_truth.py "tx.txt|rx.txt|3200"

The receiver's absolute constellation rotation and its dibit labelling are both
free, so the scan maximises over all 24 symbol permutations, both the
differential and absolute dibit fields, and the alignment between the two
streams.  The alignment must be exact to the symbol: a lag step coarser than 1
destroys the match completely and reports chance for a receiver that is in fact
decoding perfectly.

Measured over the bare G.711 round trip at 9600 bit/s, this reports 0.99 at
2400 baud and about 0.58 at 3200 -- against 0.25 for chance.
"""
import sys, itertools
perms=list(itertools.permutations(range(4)))
def load(txf,rxf):
    tx={}; rx={}
    for l in open(txf):
        r,n,v=l.split(); tx.setdefault(r,[]).append(int(v))
    for l in open(rxf):
        r,n,d,a=l.split(); rx.setdefault(r,[]).append((int(d),int(a)))
    return tx['answer'], rx['caller']
def scan(txf,rxf,label,rostep=200,n=200):
    A,R=load(txf,rxf)
    best=None
    for ro in range(0,len(R)-n,rostep):
        for field in (0,1):
            seg=[R[ro+i][field] for i in range(n)]
            for lag in range(0,len(A)-n):
                cm=[0]*16
                for x,y in zip(seg,A[lag:lag+n]): cm[(x<<2)|y]+=1
                for p in perms:
                    m=cm[p[0]]+cm[4+p[1]]+cm[8+p[2]]+cm[12+p[3]]
                    if best is None or m>best[0]:
                        best=(m,ro,lag,field,p)
    m,ro,lag,field,p=best
    print(f"{label}: best={m/n:.2f} rx_off={ro} tx_lag={lag} field={'diff' if field==0 else 'abs'} perm={p}")
for a in sys.argv[1:]:
    txf,rxf,label=a.split('|')
    scan(txf,rxf,label)
