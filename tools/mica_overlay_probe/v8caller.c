#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <spandsp.h>
static v8_state_t *s; static v8_parms_t p; static int status=-99; static uint32_t res_mod, res_peer;
static void handler(void *u, v8_parms_t *r){(void)u;status=r->status;res_mod=r->jm_cm.modulations;res_peer=r->jm_cm.peer_modulations;}
int v8c_init(uint32_t mods){
  memset(&p,0,sizeof p);
  p.modem_connect_tone=MODEM_CONNECT_TONES_NONE;
  p.send_ci=1; p.v92=-1;
  p.jm_cm.call_function=V8_CALL_V_SERIES;
  p.jm_cm.modulations=mods;
  p.jm_cm.protocols=V8_PROTOCOL_LAPM_V42;
  p.jm_cm.pstn_access=0; p.jm_cm.pcm_modem_availability=0;
  p.jm_cm.nsf=-1; p.jm_cm.t66=-1;
  s=v8_init(NULL,1,&p,handler,NULL); return s?0:-1;
}
void v8c_step(const uint8_t *in,uint8_t *out,int n){
  int16_t a[1024]; int i;
  for(i=0;i<n;i++) a[i]=ulaw_to_linear(in[i]);
  v8_rx(s,a,n);
  int k=v8_tx(s,a,n); for(i=k;i<n;i++) a[i]=0;
  for(i=0;i<n;i++) out[i]=linear_to_ulaw(a[i]);
}
int v8c_status(void){return status;} unsigned v8c_mod(void){return res_mod;} unsigned v8c_peer(void){return res_peer;}
