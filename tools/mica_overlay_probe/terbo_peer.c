/* Block-protocol far end for MicaEmu's mica_line: a SpanDSP V.32bis caller offering MICA the V.32terbo
 * rate signal (call3.py's caller, without the Python).  stdin: 160-octet PCMU blocks of MICA's audio;
 * stdout: 160 octets of ours.  Silence until START seconds, then the caller.
 * usage: terbo_peer START_SECONDS EXT_HEX PROBE(0/1) [CALLER_BYTES_FILE]
 * env TERBO_TX=iso,quarter_turns,mirror makes the caller transmit the 512-point constellation after E.
 * The caller's own payload is a known byte pattern if a file is given (async 8N1 would need a UART
 * model, so it is raw bits, LSB first); otherwise marks. */
#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <spandsp.h>
static v32bis_state_t *s;
static uint8_t *pay; static long paylen, pos; static int bitn;
static int get_bit(void *u){(void)u; if(!pay||pos>=paylen) return 1; int b=(pay[pos]>>bitn)&1; if(++bitn==8){bitn=0;pos++;} return b;}
static long pb_ones,pb_total;
static void put_bit(void *u,int b){(void)u; if(b==0||b==1){pb_total++; pb_ones+=b;}}
static int rd(int fd,uint8_t*b,int n){int g=0;while(g<n){int r=read(fd,b+g,n-g);if(r<=0)return 0;g+=r;}return 1;}
int main(int argc,char**argv){
  double start=argc>1?atof(argv[1]):4.5; int ext=argc>2?(int)strtol(argv[2],0,0):0; int probe=argc>3?atoi(argv[3]):0;
  if(argc>4){FILE*f=fopen(argv[4],"rb"); if(f){fseek(f,0,SEEK_END);paylen=ftell(f);rewind(f);pay=malloc(paylen);fread(pay,1,paylen,f);fclose(f);}}
  int out_fd=dup(1); dup2(2,1);
  uint8_t in[160],out[160]; int16_t a[160]; long blk=0; int on=0;
  while(rd(0,in,160)){
    double t=blk*0.02; blk++;
    if(!on && t>=start){
      s=v32bis_init(NULL,14400,1,get_bit,NULL,put_bit,NULL);
      v32bis_set_supported_bit_rates(s,getenv("RATES")?(int)strtol(getenv("RATES"),0,0):0x1260);
      v32bis_start_tones(s);
      v32bis_set_rate_signal_extension(s,(uint16_t)ext); v32bis_set_terbo_probe(s,probe!=0);
      if(getenv("TERBO_TX")){int iso=0,k=0,m=0; sscanf(getenv("TERBO_TX"),"%d,%d,%d",&iso,&k,&m); fprintf(stderr,"terbo tx iso %d quarter-turns %d mirror %d: %d\n",iso,k,m,v32bis_set_terbo_tx(s,true,iso,k,m!=0));}
      on=1; fprintf(stderr,"[%5.2fs] V.32bis caller starts\n",t);
    }
    if(!on || (getenv("SILENT_AT") && t>=atof(getenv("SILENT_AT")))){memset(out,0xFF,160); if(on){v32bis_rx(s,a,0);} }
    else{
      for(int i=0;i<160;i++)a[i]=ulaw_to_linear(in[i]);
      v32bis_rx(s,a,160); v32bis_tx(s,a,160);
      if(getenv("GAIN")){double g=atof(getenv("GAIN")); for(int i=0;i<160;i++){double v=a[i]*g; a[i]=v>32767?32767:(v<-32768?-32768:(int16_t)v);}}
      for(int i=0;i<160;i++)out[i]=linear_to_ulaw(a[i]);
      if(blk%50==0) fprintf(stderr,"[%5.2fs] caller rate=%d done=%d rx bits %ld ones %.3f\n",t,v32bis_current_bit_rate(s),v32bis_startup_complete(s),pb_total,pb_total?(double)pb_ones/pb_total:0.0); pb_total=pb_ones=0;
    }
    if(write(out_fd,out,160)!=160)break;
  }
  return 0;
}
