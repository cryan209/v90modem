#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES
#include <stdint.h>
#include <stdlib.h>
#include <spandsp.h>
static v32bis_state_t *s;
static int get_bit(void *u){(void)u;return 1;}
static void put_bit(void *u,int b){(void)u;(void)b;}
int caller_init(int rates){
  s=v32bis_init(NULL,14400,1,get_bit,NULL,put_bit,NULL);
  if(!s)return -1;
  v32bis_set_supported_bit_rates(s,rates);
  return v32bis_start_tones(s);
}
/* answerer's octets in, caller's octets out; law 0 ulaw 1 alaw */
void caller_step(const uint8_t *in,uint8_t *out,int n,int law){
  int16_t a[1024];
  int i;
  for(i=0;i<n;i++) a[i]=law?alaw_to_linear(in[i]):ulaw_to_linear(in[i]);
  v32bis_rx(s,a,n);
  v32bis_tx(s,a,n);
  for(i=0;i<n;i++) out[i]=law?linear_to_alaw(a[i]):linear_to_ulaw(a[i]);
}
int caller_rate(void){return v32bis_current_bit_rate(s);}
int caller_done(void){return v32bis_startup_complete(s);}
int caller_remote_rates(void){return v32bis_startup_remote_rates(s);}
int caller_nt_mt(int *nt,int *mt){return v32bis_round_trip_symbols(s,nt,mt);}
