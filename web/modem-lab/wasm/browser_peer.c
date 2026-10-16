/* Browser adapter for the real engine. ITU-T V.90 §5: preserve DS0 bytes.
 * Each JS module instance owns one engine. No SIP, PTY, fax, or OS audio.
 * V.250 §6.5 framing settings are supplied through the existing engine API.
 */
#include <emscripten.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "modem_engine.h"
#include "data_interface.h"
#include "line_monitor.h"

static uint8_t input[160], output[160], received[65536];
static int received_n, connected_rate;
static char monitor_json[65536];
static di_connect_info_cb_t connect_info;
static v250_connect_report_t report;
static v250_ctl_t browser_settings;
static int browser_line_mode;
EMSCRIPTEN_KEEPALIVE int browser_init(int mode, int calling, int startup, int ceiling) {
    browser_line_mode=(mode==32 || mode==34);
    received_n=connected_rate=0;memset(&report,0,sizeof report);
    v250_ctl_reset(&browser_settings);
    if(mode==92){browser_settings.pig=0;browser_settings.pqc=(startup & 4)?0:3;}
    setenv("ME_MODE", mode == 92 ? "v92" : mode == 90 ? "v90" : mode == 34 ? "v34" : "v32bis", 1);
    setenv("ME_V8", (startup & 1) ? "1" : "0", 1);
    setenv("ME_V8BIS", (startup & 2) ? "1" : "0", 1);
    setenv("ME_DATA_FRAMING", "lapm", 1);
    setenv("ME_GUI_PORT", "browser-passive-taps", 1);
    setenv("ME_V90_JA_HEURISTIC_FALLBACK_MS", "0", 1);
    setenv("ME_V90_P3_JA_SCAN_SYNC", "1", 1);
    if ((mode == 90 || mode == 92) && calling) setenv("ME_V90_ROLE", "analogue", 1);
    /* V.34 §11.4: exercise the existing 3429-symbol/s training path.
       Keep normal TRN/B1 rate selection and §11.5 retraining enabled. */
    if (mode == 34) setenv("ME_V34_BAUD", "3429", 1);
    if (mode == 34 && ceiling > 0) { char rate[16];snprintf(rate,sizeof rate,"%d",ceiling);setenv("ME_V34_BPS",rate,1); }
    if (mode == 32) setenv("ME_V32BIS_MAX_BPS", "9600", 1);
    setenv("ME_V34_SPAN_FLOW_LOG", "1", 1);
    setenv("DS_TX_BIT_DUMP", "/tx.bits", 1);
    setenv("DS_RX_BIT_DUMP", "/rx.bits", 1);
    me_set_verbose(1); me_init(); me_set_law(ME_LAW_ULAW);
    if (calling) me_dial("browser-peer");
    me_on_sip_connected();
    return 0;
}
EMSCRIPTEN_KEEPALIVE int browser_step(void) {
    me_pump_dte(); me_rx_g711(input, 160); me_tx_g711(output, 160);
    return me_get_state();
}
EMSCRIPTEN_KEEPALIVE uint8_t *browser_input(void) { return input; }
EMSCRIPTEN_KEEPALIVE uint8_t *browser_output(void) { return output; }
EMSCRIPTEN_KEEPALIVE int browser_send(const uint8_t *p, int n) { return me_put_data(p,n); }
EMSCRIPTEN_KEEPALIVE char *browser_monitor(void) {
    int n=lm_gui_json(monitor_json,sizeof monitor_json);
    if(n<0)monitor_json[0]=0;
    return monitor_json;
}
/* V.42 §7 wire bits, before HDLC interpretation. MEMFS only; no host files. */
EMSCRIPTEN_KEEPALIVE int browser_bit_count(int dir) {
    fflush(NULL);
    FILE *f=fopen(dir ? "/tx.bits" : "/rx.bits","r");
    if(!f)return 0; fseek(f,0,SEEK_END); int n=(int)ftell(f); fclose(f); return n;
}
EMSCRIPTEN_KEEPALIVE char *browser_read_bits(int dir) {
    int n=browser_bit_count(dir); char *p=malloc((size_t)n+1); if(!p)return NULL;
    FILE *f=fopen(dir ? "/tx.bits" : "/rx.bits","r");
    if(f){n=(int)fread(p,1,(size_t)n,f);fclose(f);}else n=0;
    p[n]=0;return p;
}
EMSCRIPTEN_KEEPALIVE const char *browser_modulation(void) { return report.carrier ? report.carrier : ""; }
EMSCRIPTEN_KEEPALIVE int browser_rate(void) { return connected_rate; }
EMSCRIPTEN_KEEPALIVE int browser_tx_rate(void) { return report.tx_rate; }
EMSCRIPTEN_KEEPALIVE int browser_rx_rate(void) { return report.rx_rate ? report.rx_rate : connected_rate; }
EMSCRIPTEN_KEEPALIVE int browser_received_n(void) { return received_n; }
EMSCRIPTEN_KEEPALIVE uint8_t *browser_received(void) { return received; }
EMSCRIPTEN_KEEPALIVE void browser_clear_received(void) { received_n=0; }
EMSCRIPTEN_KEEPALIVE void browser_destroy(void) { me_destroy(); }
void di_get_v250_settings(v250_ctl_t *out) { *out=browser_settings; }
void di_on_connected(int rate) {
    connected_rate=rate; memset(&report,0,sizeof report); report.tx_rate=rate;
    if(connect_info) connect_info(rate,&report);
}
void di_set_connect_info_cb(di_connect_info_cb_t cb) { connect_info=cb; }
void di_on_disconnected_cause(const char *s,int o) {(void)s;(void)o;connected_rate=0;}
void di_on_call_failed(int s) {(void)s;connected_rate=0;}
int di_read_data(uint8_t *p,int n) {(void)p;(void)n;return 0;}
int di_write_data(const uint8_t *p,int n) {
    /* Match the native DTE data-port gate: startup bytes are not host data. */
    if(!connected_rate)return n;
    int copy=n; if(copy>65536-received_n)copy=65536-received_n;
    memcpy(received+received_n,p,copy); received_n+=copy; return copy;
}
void di_set_modulation_ops(di_ms_set_cb_t a,di_ms_get_cb_t b,di_ms_reset_cb_t c){(void)a;(void)b;(void)c;}
void di_set_modulation_limits_op(di_ms_limits_cb_t a){(void)a;}
void di_set_link_detail_cb(di_link_detail_cb_t a){(void)a;}
void di_set_break_cb(di_break_cb_t a){(void)a;}
void di_set_loop3_cb(di_loop3_cb_t cb){(void)cb;}
void di_set_pmhr_cb(di_pmhr_cb_t a){(void)a;}
void di_report_pmhr(int a){(void)a;}
void di_update_link(const v250_connect_report_t *a,const char *b,const char *c){(void)a;(void)b;(void)c;}
int di_fax_active(void){return 0;}
int di_fax_rx(const int16_t *a,int n){(void)a;return n;}
int di_fax_tx(int16_t *a,int n){memset(a,0,n*sizeof(*a));return n;}
int di_fax_v34hdx_profile(int a,int *b,int *c){(void)a;(void)b;(void)c;return 0;}
void di_fax_v34hdx_set_channel(int a){(void)a;}
int di_fax_v34hdx_get_request(int *a){(void)a;return 0;}
int di_fax_v34hdx_start_control(int a){(void)a;return -1;}
void di_fax_v34hdx_advance(int a){(void)a;}
int di_fax_v34hdx_get_bit(void){return 1;}
void di_fax_v34hdx_put_bit(int a){(void)a;}
int di_fax_v34hdx_get_mode(void){return 0;}

/* Payload-only demonstrations: x2 Draft 0.33 §§10/11/15/16 and K56flex
 * Draft 0.23 §§7.1–7.12. Fixed shared profiles; no simulated CONNECT,
 * upstream, line training or negotiated parameters. RX gets codewords only. */
#include "x2.h"
#include "x2_mp_rx.h"
#include "k56flex.h"

typedef struct { const uint8_t *data; unsigned bits, position; } browser_pcm_source_t;
static int browser_pcm_bit(void *user) {
    browser_pcm_source_t *s=user;
    unsigned i=s->position++;
    return i<s->bits ? (s->data[i/8]>>(i%8))&1 : 0;
}
static int browser_pcm_word(void *user, uint16_t *word) {
    *word=0;
    for(unsigned i=0;i<16;i++) *word|=(uint16_t)(browser_pcm_bit(user)<<i);
    return 0;
}
static int browser_x2_config(x2_pcm_config_t *cfg) {
    /* Recovered short-record allocation 15: B=37, MD=5, 42 bits/6 DS0s.
       A fixed demonstration profile, not a received MP or CONNECT rate. */
    const x2_mp_t mp={{(15<<2)|(13<<6),0x03fe,0,0x0500},0};
    return x2_short_record_config(&mp,0x7fff,cfg)==15 ? 0 : -1;
}
EMSCRIPTEN_KEEPALIVE int browser_pcm_encode(int mode, const uint8_t *source, int n,
                                            uint8_t *out, int capacity) {
    if((mode!=2 && mode!=56) || n<0 || n>60002 || capacity<0) return -1;
    browser_pcm_source_t bits={source,(unsigned)n*8,0};
    unsigned width=mode==2?6:8, frame_bits=mode==2?42:56;
    unsigned samples=((bits.bits+frame_bits-1)/frame_bits)*width;
    if(samples>(unsigned)capacity) return -1;
    if(mode==2) {
        x2_pcm_config_t cfg; x2_pcm_tx_t tx;
        if(browser_x2_config(&cfg) || x2_pcm_tx_init(&tx,&cfg,18,browser_pcm_bit,&bits)) return -1;
        return x2_pcm_tx_g711(&tx,out,samples)==samples ? (int)samples : -1;
    }
    k56flex_pcm_config_t cfg={K56FLEX_LAW_MU,56000,0}; k56flex_pcm_tx_t tx;
    if(k56flex_pcm_tx_init(&tx,&cfg,browser_pcm_word,&bits)) return -1;
    return k56flex_pcm_tx_g711(&tx,out,samples)==samples ? (int)samples : -1;
}
EMSCRIPTEN_KEEPALIVE int browser_pcm_decode(int mode, const uint8_t *octets, int samples,
                                            uint8_t *out, int capacity) {
    if((mode!=2 && mode!=56) || samples<0 || capacity<0) return -1;
    unsigned width=mode==2?6:8, frame_bits=mode==2?42:56, position=0;
    if(samples%(int)width || ((unsigned)samples/width*frame_bits+7)/8>(unsigned)capacity) return -1;
    memset(out,0,(size_t)capacity);
    x2_pcm_config_t xcfg; x2_pcm_state_t xr={0}; x2_scrambler_t descrambler;
    k56flex_pcm_config_t kcfg={K56FLEX_LAW_MU,56000,0}; k56flex_pcm_rx_t kr;
    if(mode==2) {
        if(browser_x2_config(&xcfg) || x2_scrambler_init(&descrambler,18,0)) return -1;
    } else if(k56flex_pcm_rx_init(&kr,&kcfg)) return -1;
    for(int i=0;i<samples;i+=(int)width) {
        uint8_t recovered[64]; uint64_t scrambled;
        if(mode==2) {
            if(x2_pcm_decode(&xcfg,&xr,octets+i,&scrambled)) return -1;
            for(unsigned b=0;b<frame_bits;b++) recovered[b]=(uint8_t)x2_descramble_bit(&descrambler,(unsigned)(scrambled>>b)&1);
        } else if(k56flex_pcm_rx_frame(&kr,octets+i,recovered)!=(int)frame_bits) return -1;
        for(unsigned b=0;b<frame_bits;b++,position++) out[position/8]|=(uint8_t)(recovered[b]<<(position%8));
    }
    return (int)((position+7)/8);
}


/* Synthetic analogue receive channel, never a decode/re-encode of the DS0.
 * The fractional clock experiment keeps the caller's sample accounting: a
 * bounded, causal 65-tap sinc reads waveform history at an offset clock.
 * Its window, noise reference and 128-sample lookback are lab model choices,
 * not ITU modem constants. It runs only on browser V.32/V.34 receivers. */
static double line_gain=1, line_noise, line_ratio=1, line_position=-128;
static uint32_t line_random=1234;
static int16_t line_history[4096], line_last[320];
static int64_t line_written;
static unsigned line_clipped;
static float line_taps[1024][65];
static int line_taps_ready;
EMSCRIPTEN_KEEPALIVE int browser_line_config(double noise_db, double loss_db, double ppm, unsigned seed) {
    if(!isfinite(noise_db)||!isfinite(loss_db)||!isfinite(ppm)
       ||(noise_db!=0 && (noise_db<10 || noise_db>80))
       ||loss_db<0 || loss_db>30 || fabs(ppm)>100) return -1;
    if(!browser_line_mode && (noise_db || loss_db || ppm)) return -1;
    line_gain=pow(10,-loss_db/20);line_noise=noise_db?2500*pow(10,-noise_db/20):0;
    line_ratio=1+ppm*1e-6;line_position=-128;line_written=0;line_random=seed;
    line_clipped=0;memset(line_history,0,sizeof line_history);memset(line_last,0,sizeof line_last);
    if(ppm && !line_taps_ready){
        for(int p=0;p<1024;p++){
            double sum=0;
            for(int k=-32;k<=32;k++){
                double x=k-p/1024.0;
                double sinc=fabs(x)<1e-12?1:sin(3.141592653589793*x)/(3.141592653589793*x);
                double window=.42+.5*cos(3.141592653589793*x/32)+.08*cos(2*3.141592653589793*x/32);
                line_taps[p][k+32]=(float)(fabs(x)<=32?sinc*window:0);sum+=line_taps[p][k+32];
            }
            for(int k=0;k<65;k++)line_taps[p][k]/=(float)sum;
        }
        line_taps_ready=1;
    }
    return 0;
}
static double browser_line_gaussian(void){
    double sum=0;
    for(int i=0;i<12;i++){
        line_random=1664525u*line_random+1013904223u;
        sum+=(double)(line_random>>8)/16777216.0-.5;
    }
    return sum;
}
void browser_receive_line(int16_t *samples, int count){
    for(int i=0;i<count;i++){
        double value=samples[i];
        if(browser_line_mode){
            if(line_ratio!=1){
                line_history[line_written++ & 4095]=samples[i];
                int64_t centre=(int64_t)floor(line_position);
                int phase=(int)((line_position-centre)*1024);value=0;
                for(int k=-32;k<=32;k++){
                    int64_t at=centre+k;
                    if(at>=0 && at<line_written && line_written-at<=4096)
                        value+=line_history[at & 4095]*line_taps[phase][k+32];
                }
                line_position+=line_ratio;
            }
            value*=line_gain;
            if(line_noise)value+=line_noise*browser_line_gaussian();
            if(value>32767){value=32767;line_clipped++;}
            if(value<-32768){value=-32768;line_clipped++;}
            samples[i]=(int16_t)lrint(value);
        }
        if(i<320)line_last[i]=samples[i];
    }
}
EMSCRIPTEN_KEEPALIVE int16_t *browser_line_samples(void){return line_last;}
EMSCRIPTEN_KEEPALIVE unsigned browser_line_clipped(void){return line_clipped;}
