/* EC-only cut of rig/slmodemd/v92_demapper_ec_wrap.c: DM_V92EC_BYPASS=1 makes
 * V92EchoCanceller::process a copy (the SIP rig has no echo, and the frozen
 * canceller adds noise proportional to slmodemd's own PCM upstream). */
#include <stdio.h>
#include <stdlib.h>
extern void orig_ecp(void *self, float *in, float *out, unsigned n) __asm__("_ZN16V92EchoCanceller7processEPfS0_j_original");
void wrap_ecp(void *self, float *in, float *out, unsigned n) __asm__("_ZN16V92EchoCanceller7processEPfS0_j");
void wrap_ecp(void *self, float *in, float *out, unsigned n){
    static int init=-1;
    if(init<0){ init = getenv("DM_V92EC_BYPASS")!=NULL; fprintf(stderr,"<ecwrap> V92EchoCanceller bypass=%d\n",init); }
    if(init){ for(unsigned i=0;i<n;i++) out[i]=in[i]; return; }
    orig_ecp(self,in,out,n);
}
