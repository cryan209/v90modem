"""Support of the next-label distribution given the previous 3 labels (standard Z2 -> 2RZ2 partition)."""
import sys
from collections import Counter, defaultdict
from trellis2 import labels
S=sys.argv[1]
for pk,c in (('rec2_z.pkl',0.48),('rec1_z.pkl',0.49)):
    lab=labels(S+'/'+pk,c,(2,2,4))
    ids={l:n for n,l in enumerate(sorted(set(lab)))}
    seq=[ids[l] for l in lab]
    for K in (3,4):
        ctx=defaultdict(Counter)
        for i in range(K,len(seq)): ctx[tuple(seq[i-K:i])][seq[i]]+=1
        sizes=Counter(); cover=[]
        for k,cn in ctx.items():
            n=sum(cn.values())
            # labels carrying >=10% of this context
            sup=sorted(l for l,v in cn.items() if v>=0.10*n)
            sizes[len(sup)]+=1
        print(pk,'context length',K,': contexts',len(ctx),' support size histogram (labels >=10% of context):',dict(sorted(sizes.items())))
    # which structure do the supports have?  group labels by a candidate binary function
    ctx=defaultdict(Counter)
    for i in range(3,len(seq)): ctx[tuple(seq[i-3:i])][seq[i]]+=1
    sups=Counter()
    for k,cn in ctx.items():
        n=sum(cn.values()); sups[tuple(sorted(l for l,v in cn.items() if v>=0.10*n))]+=1
    print('  most common supports:',sups.most_common(8))
    print('  label ids:',ids)
