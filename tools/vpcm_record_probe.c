/* Offline analogue recording recovery helper. No live bearer processing. */

/* Offline strict Table-14 validation and known TRN2d reference. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <spandsp.h>
#include "vpcm_cp.h"
#include "v90.h"
#include "v91.h"
static int reference_main(int argc,char **argv) {
 uint8_t bits[VPCM_CP_MAX_BITS] = {0},ref[60000];vpcm_cp_diag_t d = {0};v90_shaped_rx_state_t state={0};
 if(argc<2)return 2;
 FILE *f=fopen(argv[1],"rb");if(!f)return 2;int n=(int)fread(bits,1,sizeof(bits),f);fclose(f);
 if(n<156 || !vpcm_cp_decode_diag(bits,n,&d)){fprintf(stderr,"invalid frame crc=%04x reserved=%d starts=%d fill=%d\n",d.crc_remainder,d.reserved_bits_ok,d.start_bits_ok,d.fill_ok);return 1;}
 printf("valid=1 crc=%04x D=%d drn=%u Sr=%u ld=%u compat=%d rate_mask=%04x a1=%d a2=%d b1=%d b2=%d\n",d.crc_field,d.frame.drn+8,d.frame.drn,d.frame.shaping_redundancy,d.frame.shaping_lookahead,d.frame.v90_compatibility,d.frame.upstream_rate_mask,(int8_t)d.frame.shaping_a1_q1_6,(int8_t)d.frame.shaping_a2_q1_6,(int8_t)d.frame.shaping_b1_q1_6,(int8_t)d.frame.shaping_b2_q1_6);
 if(argc<3){
 for(int slot=0;slot<6;slot++){int c=d.frame.dfi[slot];int rank=0;
 for(int u=0;u<128;u++)if(vpcm_cp_mask_get(d.frame.masks[c],u)){
 int dest=u;if(d.frame.codec_constellations_differ){int r=rank;for(int k=0;k<128;k++)if(vpcm_cp_mask_get(d.frame.codec_masks[c],k)){if(r--==0){dest=k;break;}}}
 v91_law_t law=d.frame.codec_alaw?V91_LAW_ALAW:V91_LAW_ULAW;
 uint8_t a=v91_ucode_to_codeword(law,u,true),b=v91_ucode_to_codeword(law,dest,true);
 printf("slot=%d ucode=%d level=%d codec_ucode=%d codec_level=%d\n",slot,u,d.frame.codec_alaw?alaw_to_linear(a):ulaw_to_linear(a),dest,d.frame.codec_alaw?alaw_to_linear(b):ulaw_to_linear(b));rank++;
 }}return 0;}
 if(argc>3) d.frame.drn += 12; /* B1d uses D=drn+20, V.90 8.6.1. */
 n=v90_generate_trn2d_codewords(d.frame.codec_alaw?V90_LAW_ALAW:V90_LAW_ULAW,&d.frame,&state,10000,ref,sizeof(ref));
 if(n<=0){fprintf(stderr,"reference generation failed\n");return 1;}
 f=fopen(argv[2],"wb");if(!f)return 2;
 for(int i=0;i<n;i++){int16_t a=d.frame.codec_alaw?alaw_to_linear(ref[i]):ulaw_to_linear(ref[i]);
 if(argc>4 && d.frame.codec_constellations_differ){
 int c=d.frame.dfi[i%6];v91_law_t law=d.frame.codec_alaw?V91_LAW_ALAW:V91_LAW_ULAW;int u=v91_codeword_to_ucode(law,ref[i]);int rank=0;
 for(int k=0;k<u;k++)if(vpcm_cp_mask_get(d.frame.masks[c],k))rank++;
 int dest=-1;for(int k=0;k<128;k++)if(vpcm_cp_mask_get(d.frame.codec_masks[c],k)){if(rank--==0){dest=k;break;}}
 if(dest<0)return 2;uint8_t cw=v91_ucode_to_codeword(law,dest,a>=0);a=d.frame.codec_alaw?alaw_to_linear(cw):ulaw_to_linear(cw);
 }fwrite(&a,2,1,f);}fclose(f);return 0;
}

/* Offline Table-14 shaped demapping. Output is unverified candidate bits. */
#include <stdio.h>
#include <stdlib.h>
#include <spandsp.h>
#include "vpcm_cp.h"
#include "v90.h"
static int demap_main(int argc,char **argv){
 uint8_t bits[VPCM_CP_MAX_BITS] = {0},cw[6],out[64];int16_t linear[6];vpcm_cp_frame_t cp;uint32_t reg=0;v90_shaped_rx_state_t shaper={0};
 if(argc<4||argc>5)return 2;if(argc==5)shaper.prev_odd=(uint8_t)(atoi(argv[4])!=0);FILE *f=fopen(argv[1],"rb");if(!f)return 2;int n=(int)fread(bits,1,sizeof(bits),f);fclose(f);if(n<156 || !vpcm_cp_decode_bits(bits,n,&cp))return 2;
 f=fopen(argv[2],"rb");FILE *o=fopen(argv[3],"wb");if(!f||!o)return 2;int frames=0,bad=0,ones=0,run=0,longest=0,total=0;
 while(fread(linear,2,6,f)==6){for(int i=0;i<6;i++)cw[i]=cp.codec_alaw?linear_to_alaw(linear[i]):linear_to_ulaw(linear[i]);
 n=v90_demap_shaped_frame(cp.codec_alaw?V90_LAW_ALAW:V90_LAW_ULAW,&cp,cp.drn+(cp.v90_compatibility?20:8),&reg,&shaper,cw,out);frames++;
 if(n<=0){bad++;n=cp.drn+(cp.v90_compatibility?20:8);for(int i=0;i<n;i++)out[i]=2;}
 for(int i=0;i<n;i++){total++;ones+=(out[i]==1);run=out[i]==1?run+1:0;if(run>longest)longest=run;}fwrite(out,1,n,o);}
 fclose(f);fclose(o);printf("frames=%d bad=%d ones=%d bits=%d longest_ones=%d\n",frames,bad,ones,total,longest);return 0;
}

/* Offline known-ones sign recovery through the native V.90 shaper inverse.
 * V.90 8.6.5 initializes GPC/shaping at TRN2d; 5.4.5 permits sign choices.
 * No protocol or DSP constants are modified. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <spandsp.h>
#include "v90.h"
#include "vpcm_cp.h"
#define BEAM 8
#define MAXFRAMES 3000
typedef struct {v90_shaped_rx_state_t sh;uint32_t reg;double cost;int prev,mask;} node;
static node history[MAXFRAMES+1][BEAM];
static int counts[MAXFRAMES+1];
static int same(node*a,node*b){return a->reg==b->reg&&a->sh.prev_odd==b->sh.prev_odd&&a->sh.trellis_state==b->sh.trellis_state&&!memcmp(a->sh.prev_t,b->sh.prev_t,6);}
static int training_main(int argc,char**argv){
 if(argc!=5)return 2;uint8_t bits[VPCM_CP_MAX_BITS] = {0},cw[6],out[64];vpcm_cp_frame_t cp;
 FILE*f=fopen(argv[1],"rb");if(!f)return 2;int n=(int)fread(bits,1,sizeof(bits),f);fclose(f);if(n<156 || !vpcm_cp_decode_bits(bits,n,&cp))return 2;
 FILE*ref=fopen(argv[2],"rb"),*eq=fopen(argv[3],"rb");if(!ref||!eq)return 2;int frames=0;int16_t mag[MAXFRAMES][6];double val[6];counts[0]=1;
 while(frames<MAXFRAMES&&fread(mag[frames],2,6,ref)==6&&fread(val,8,6,eq)==6){
  node next[BEAM];int nn=0;
  for(int p=0;p<counts[frames];p++)for(int mask=0;mask<64;mask++){
   node cur=history[frames][p];double cost=cur.cost;
   for(int i=0;i<6;i++){int16_t v=(mask&(1<<i))?-abs(mag[frames][i]):abs(mag[frames][i]);cw[i]=cp.codec_alaw?linear_to_alaw(v):linear_to_ulaw(v);double e=val[i]-v;cost+=e*e;}
   int got=v90_demap_shaped_frame(cp.codec_alaw?V90_LAW_ALAW:V90_LAW_ULAW,&cp,cp.drn+8,&cur.reg,&cur.sh,cw,out);
   if(got!=cp.drn+8)continue;/* First two frames condition the 23-bit GPC and sign memory. */int ok=1;for(int i=0;i<got;i++)if(frames>=2 && out[i]!=1)ok=0;if(!ok)continue;
   cur.cost=cost;cur.prev=p;cur.mask=mask;int duplicate=-1;
   for(int j=0;j<nn;j++)if(same(&cur,&next[j])){duplicate=j;break;}
   if(duplicate>=0){if(cur.cost>=next[duplicate].cost)continue;next[duplicate]=cur;}
   else if(nn<BEAM)next[nn++]=cur;
   else {int worst=0;for(int j=1;j<nn;j++)if(next[j].cost>next[worst].cost)worst=j;if(cur.cost>=next[worst].cost)continue;next[worst]=cur;}
  }
  if(!nn){fprintf(stderr,"known-ones path failed frame=%d\n",frames);return 1;}
  frames++;counts[frames]=nn;memcpy(history[frames],next,(size_t)nn*sizeof(node));
 }
 fclose(ref);fclose(eq);if(frames==0)return 1;int best=0;for(int i=1;i<counts[frames];i++)if(history[frames][i].cost<history[frames][best].cost)best=i;
 double cost=history[frames][best].cost;for(int frame=frames;frame>0;frame--){node*p=&history[frame][best];for(int i=0;i<6;i++)mag[frame-1][i]=(p->mask&(1<<i))?-abs(mag[frame-1][i]):abs(mag[frame-1][i]);best=p->prev;}
 f=fopen(argv[4],"wb");if(!f)return 2;fwrite(mag,12,(size_t)frames,f);fclose(f);printf("frames=%d known_bits=%d eq_fit_rmse=%.3f\n",frames,frames*(cp.drn+8),sqrt(cost/(frames*6)));return 0;
}

/* Feed recovered bits to the unmodified public V.42 detector. */
#include <stdio.h>
#include <stdlib.h>
#include <spandsp.h>
static unsigned long bitpos;static int detected,connected,payload_bytes;
static int get_msg(void*u,uint8_t*m,int n){(void)u;(void)m;(void)n;return 0;}
static void put_msg(void*u,const uint8_t*m,int n){(void)u;(void)m;payload_bytes+=n;}
static void status(void*u,int s){(void)u;if(s==V42_STATUS_DETECTION_SUCCEEDED){detected=1;printf("detection_succeeded bit=%lu\n",bitpos);}if(s==SIG_STATUS_LINK_CONNECTED)connected=1;}
static int v42_main(int argc,char**argv){if(argc<2 || argc>3)return 2;FILE*f=fopen(argv[1],"rb");if(!f)return 2;v42_state_t*s=v42_init(NULL,true,true,get_msg,put_msg,NULL);if(!s)return 2;v42_set_status_callback(s,status,NULL);v42_set_bit_rate(s,argc==3?atoi(argv[2]):44000);v42_restart(s);int b;while((b=fgetc(f))!=EOF){if(b>1)return 2;v42_rx_bit(s,b);bitpos++;}fclose(f);v42_free(s);printf("bits=%lu detection=%d connected=%d application_bytes=%d\n",bitpos,detected,connected,payload_bytes);return detected?0:1;}


int main(int argc, char **argv) {
 if(argc<2){fprintf(stderr,"usage: %s reference|training|demap|v42 <args>\n",argv[0]);return 2;}
 if(!strcmp(argv[1],"reference"))return reference_main(argc-1,argv+1);
 if(!strcmp(argv[1],"training"))return training_main(argc-1,argv+1);
 if(!strcmp(argv[1],"demap"))return demap_main(argc-1,argv+1);
 if(!strcmp(argv[1],"v42"))return v42_main(argc-1,argv+1);
 return 2;
}
