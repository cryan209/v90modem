/* Synchronous raw-G.711 closed-loop peer. The driver supplies the clock.
 * Every frame returned here is generated after consuming peer RX, never replayed.
 *
 *   v90_engine_peer [pty-link] [--call] [--alaw]
 *
 * --call makes this end the calling modem (as an ATD would), otherwise it
 * answers.  Frames on stdin are a 2-byte little-endian length then that many
 * received codewords; the same number of transmitted codewords go to stdout. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "modem_engine.h"
#include "data_interface.h"
static void dial(const char *uri, void *p) {(void)uri;(void)p;}
static void control(void *p) {(void)p;}
int main(int argc,char **argv) {
 const char *pty="/tmp/x2-loop-pty"; int call=0; me_law_t law=ME_LAW_ULAW;
 for(int i=1;i<argc;i++){
  if(!strcmp(argv[i],"--call"))call=1;
  else if(!strcmp(argv[i],"--alaw"))law=ME_LAW_ALAW;
  else if(argv[i][0]!='-')pty=argv[i];
  else {fprintf(stderr,"usage: %s [pty-link] [--call] [--alaw]\n",argv[0]);return 2;}
 }
 int wire=dup(STDOUT_FILENO); if(wire<0)return 2;
 dup2(STDERR_FILENO,STDOUT_FILENO);
 FILE *output=fdopen(wire,"wb"); if(!output)return 2;
 me_set_verbose(1); me_init();
 di_set_callbacks(dial,control,control,NULL);
 if(di_open(pty)<0)return 2;
 me_set_law(law);
 uint8_t header[2],rx[4096],tx[4096]; unsigned long count=0;
 while(fread(header,1,2,stdin)==2) {
  /* The call starts with the first frame, so a driver can configure the
     modem over its PTY (AT+MS, ...) beforehand, as a DTE would. */
  if(count==0){ if(call)me_dial("closed-loop"); me_on_sip_connected(); }
  int n=header[0]|header[1]<<8; if(!n||n>4096)return 3;
  if(fread(rx,1,n,stdin)!=(size_t)n)return 3;
  /* DTE payload into the engine, through the same helper sip_modem.c and the
     couplers use, so engine_pair_test covers it. */
  (void)me_pump_dte();
  me_rx_g711(rx,n); (void)me_tx_g711(tx,n); me_flush_g711_taps();
  if(fwrite(tx,1,n,output)!=(size_t)n || fflush(output))return 3;
  count+=n;
 }
 me_diag_snapshot_t s; me_get_diag_snapshot(&s);
 fprintf(stderr,"closed loop: samples=%lu state=%s modulation=%s rx=%llu tx=%llu\n",count,me_state_to_str(s.state),me_modulation_to_str(s.modulation),(unsigned long long)s.g711_rx_octets,(unsigned long long)s.g711_tx_octets);
 di_close(); me_destroy(); fclose(output); return 0;
}
