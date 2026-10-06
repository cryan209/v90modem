#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
extern void orig_reset(void *self, void *p) __asm__("_ZN11V90Demapper5resetEP16V90MappingParams_original");
extern void orig_reset_ns(void *self, void *p) __asm__("_ZN11V90Demapper14resetNoSpectralEP16V90MappingParams_original");
static void dump(const char *tag, void *self, void *p){
    FILE *f = fopen("/tmp/v90map.txt","a");
    if(!f) return;
    unsigned *w = (unsigned*)p;
    fprintf(f,"== %s self=%p t=%ld\n", tag, self, (long)time(NULL));
    for(int i=0;i<640;i++){ fprintf(f,"%u%c", w[i], (i%16==15)?'\n':' '); }
    fclose(f);
}
void wrap_reset(void *self, void *p) __asm__("_ZN11V90Demapper5resetEP16V90MappingParams");
void wrap_reset(void *self, void *p){ dump("reset", self, p); orig_reset(self,p); }
void wrap_reset_ns(void *self, void *p) __asm__("_ZN11V90Demapper14resetNoSpectralEP16V90MappingParams");
void wrap_reset_ns(void *self, void *p){ dump("resetNoSpectral", self, p); orig_reset_ns(self,p);
  FILE *f=fopen("/tmp/v90map.txt","a"); if(f){ unsigned *w=(unsigned*)self; fprintf(f,"== demapper-after self=%p\n",self); for(int i=0;i<24;i++) fprintf(f,"%u%c",w[i],(i%8==7)?'\n':' '); fclose(f);} }
extern void orig_sbe(void *self, unsigned a, unsigned b) __asm__("_ZN20V90SignBitsExtractor5resetEjj_original");
void wrap_sbe(void *self, unsigned a, unsigned b) __asm__("_ZN20V90SignBitsExtractor5resetEjj");
void wrap_sbe(void *self, unsigned a, unsigned b){
    FILE *f = fopen("/tmp/v90map.txt","a");
    if(f){ fprintf(f,"== SBEreset self=%p a=%u b=%u t=%ld\n", self, a, b, (long)time(NULL)); fclose(f);}
    orig_sbe(self,a,b);
}
extern void orig_ecp(void *self, float *in, float *out, unsigned n) __asm__("_ZN16V92EchoCanceller7processEPfS0_j_original");
void wrap_ecp(void *self, float *in, float *out, unsigned n) __asm__("_ZN16V92EchoCanceller7processEPfS0_j");
void wrap_ecp(void *self, float *in, float *out, unsigned n){
    static int init=-1;
    if(init<0){ init = getenv("DM_V92EC_BYPASS")!=NULL; FILE*f=fopen("/tmp/v90map.txt","a"); if(f){fprintf(f,"== ECbypass=%d\n",init); fclose(f);} }
    if(init){ for(unsigned i=0;i<n;i++) out[i]=in[i]; return; }
    orig_ecp(self,in,out,n);
}
