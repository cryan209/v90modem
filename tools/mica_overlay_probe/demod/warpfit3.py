"""kappa with an error bar: chi-square from the within-site scatter, both recordings, V.34 series shape."""
import math, pickle
from collections import defaultdict
from warpfit import fit, ser, S
from sites import sites
from register3 import START
def run(pk,c0,C):
    z=pickle.load(open(S+pk,'rb')); ij=sites(S+pk,c0); s0=START[pk]
    acc=defaultdict(list)
    for n in range(s0,len(z)): acc[ij[n]].append(z[n])
    means={k:(sum(v)/len(v),len(v)) for k,v in acc.items()}
    # within-site scatter (complex, per symbol): variance of z about the site mean, in cell^2 after gain A
    var=sum(sum(abs(x-means[k][0])**2 for x in v) for k,v in acc.items())/sum(len(v) for v in acc.values())
    E,out=fit(means,C,ser,[k*0.00001 for k in range(200,800)])
    best=min(out); _,c,A,o=best
    # residual is in cell^2 after dividing |A|^2 ; sigma^2 per symbol in the same units:
    s2=var/abs(A)**2
    # chi2(c) = sum_sites w |resid|^2 / s2 ; fit() returned res/|A|^2 = (1/W) sum w |r|^2 /|A|^2 -> chi2 = W*res/s2
    W=sum(w for _,w in means.values())
    chi=[(W*r/s2,cc) for r,cc,_,_ in out]; c2=min(chi)
    band=[cc for x,cc in chi if x<=c2[0]+1]
    print(pk,': E=%.2f cell^2, within-site rms %.3f cell, best c=%.5f, kappa=c*E=%.4f  (1-sigma %.4f..%.4f);  reduced chi2 = %.2f for %d sites'%(E,math.sqrt(s2),c,c*E,min(band)*E,max(band)*E,c2[0]/(len(means)-5),len(means)))
    return c*E
run('rec2_z.pkl',0.48,(0.5,0.5)); run('rec1_z.pkl',0.49,(0.5,-0.5))
