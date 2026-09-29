/* Build: cc -O1 -Ispandsp-master/src $(pkg-config --cflags libtiff-4) tools/v8_loop_test.c spandsp-master/src/.libs/libspandsp.a $(pkg-config --libs libtiff-4) -L/opt/homebrew/lib -ljpeg -lm -o v8_loop_test
   Run:   ./v8_loop_test <one-way delay ms> <near-end echo dB, 0 = none> <echo delay ms> <echo side: 1 caller, 2 answerer, 3 both>
   Env:   NOCI=1 caller sends no CI; LOG=1 caller flow log; ME_V8_TE_MS as in v8.c.  See docs/v8_conformance_audit.md. */
/* Closed-loop V.8: our caller vs our answerer through a one-way delay and
   optional near-end echo at each side.  argv: delay_ms echo_db (0 = none) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <math.h>
#include "spandsp.h"
static double now_s; static int done[2]; static double t_done[2]; static int st[2];
static void handler(void *u, v8_parms_t *r)
{ int w=(int)(intptr_t)u; if (r->status==V8_STATUS_V8_CALL||r->status==V8_STATUS_FAILED||r->status==V8_STATUS_NON_V8_CALL){ if(!done[w]){done[w]=1;t_done[w]=now_s;st[w]=r->status;} } }
#define N 160
#define MAXD 8000
int main(int argc,char**argv){
  int d=atoi(argv[1])*8; double eg = argc>2 && atof(argv[2])!=0 ? pow(10,atof(argv[2])/20):0; int ed=atoi(argc>3?argv[3]:"40")*8; int side=argc>4?atoi(argv[4]):3;
  v8_parms_t c,a; memset(&c,0,sizeof c); memset(&a,0,sizeof a);
  c.modem_connect_tone=MODEM_CONNECT_TONES_NONE; c.send_ci=getenv("NOCI")?false:true; c.v92=-1; c.jm_cm.call_function=V8_CALL_V_SERIES;
  c.jm_cm.modulations=V8_MOD_V90|V8_MOD_V34|V8_MOD_V22; c.jm_cm.protocols=V8_PROTOCOL_LAPM_V42; c.jm_cm.nsf=-1; c.jm_cm.t66=-1;
  a=c; a.modem_connect_tone=MODEM_CONNECT_TONES_ANSAM_PR;
  v8_state_t *vc=v8_init(NULL,true,&c,handler,(void*)0); if(getenv("LOG")){span_log_set_level(v8_get_logging_state(vc),SPAN_LOG_FLOW|SPAN_LOG_SHOW_TAG);} v8_state_t *va=v8_init(NULL,false,&a,handler,(void*)1);
  static int16_t c2a[MAXD*4], a2c[MAXD*4]; static int16_t chist[MAXD*4], ahist[MAXD*4];
  int16_t cb[N],ab[N],ci[N],ai[N]; long n=0; int L=MAXD*4;
  for(int blk=0; blk<30*50 && !(done[0]&&done[1]); blk++){
    now_s=n/8000.0;
    memset(cb,0,sizeof cb); memset(ab,0,sizeof ab);
    v8_tx(vc,cb,N); v8_tx(va,ab,N);
    for(int i=0;i<N;i++){ long k=n+i; c2a[k%L]=cb[i]; a2c[k%L]=ab[i]; chist[k%L]=cb[i]; ahist[k%L]=ab[i]; }
    for(int i=0;i<N;i++){ long k=n+i; int x=(k>=d)?a2c[(k-d)%L]:0, y=(k>=d)?c2a[(k-d)%L]:0;
      if(eg){ if(k>=ed){ if(side&1) x+= (int)(eg*chist[(k-ed)%L]); if(side&2) y+=(int)(eg*ahist[(k-ed)%L]); } }
      ci[i]=x>32767?32767:x<-32768?-32768:x; ai[i]=y>32767?32767:y<-32768?-32768:y; }
    v8_rx(vc,ci,N); v8_rx(va,ai,N); n+=N;
  }
  printf("caller %s %.2fs answerer %s %.2fs\n", done[0]?(st[0]==V8_STATUS_V8_CALL?"OK":"FAIL"):"none",t_done[0], done[1]?(st[1]==V8_STATUS_V8_CALL?"OK":"FAIL"):"none",t_done[1]);
  return 0; }
