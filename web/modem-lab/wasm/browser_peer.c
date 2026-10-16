/* Browser adapter for the real engine. ITU-T V.90 §5: preserve DS0 bytes.
 * Each JS module instance owns one engine. No SIP, PTY, fax, or OS audio.
 * V.250 §6.5 framing settings are supplied through the existing engine API.
 */
#include <emscripten.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "modem_engine.h"
#include "data_interface.h"
#include "line_monitor.h"

static uint8_t input[160], output[160], received[65536];
static int received_n, connected_rate;
static char monitor_json[65536];
static di_connect_info_cb_t connect_info;
static v250_connect_report_t report;
EMSCRIPTEN_KEEPALIVE int browser_init(int mode, int calling) {
    setenv("ME_MODE", mode == 90 ? "v90" : "v32bis", 1);
    setenv("ME_DATA_FRAMING", "lapm", 1);
    setenv("ME_GUI_PORT", "browser-passive-taps", 1);
    setenv("ME_V90_JA_HEURISTIC_FALLBACK_MS", "0", 1);
    setenv("ME_V90_P3_JA_SCAN_SYNC", "1", 1);
    if (mode == 90 && calling) setenv("ME_V90_ROLE", "analogue", 1);
    if (mode == 32) setenv("ME_V32BIS_MAX_BPS", "9600", 1);
    me_set_verbose(0); me_init(); me_set_law(ME_LAW_ULAW);
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
EMSCRIPTEN_KEEPALIVE int browser_rate(void) { return connected_rate; }
EMSCRIPTEN_KEEPALIVE int browser_tx_rate(void) { return report.tx_rate; }
EMSCRIPTEN_KEEPALIVE int browser_rx_rate(void) { return report.rx_rate ? report.rx_rate : connected_rate; }
EMSCRIPTEN_KEEPALIVE int browser_received_n(void) { return received_n; }
EMSCRIPTEN_KEEPALIVE uint8_t *browser_received(void) { return received; }
EMSCRIPTEN_KEEPALIVE void browser_clear_received(void) { received_n=0; }
EMSCRIPTEN_KEEPALIVE void browser_destroy(void) { me_destroy(); }
void di_get_v250_settings(v250_ctl_t *out) { v250_ctl_reset(out); }
void di_on_connected(int rate) {
    connected_rate=rate; memset(&report,0,sizeof report); report.tx_rate=rate;
    if(connect_info) connect_info(rate,&report);
}
void di_set_connect_info_cb(di_connect_info_cb_t cb) { connect_info=cb; }
void di_on_disconnected_cause(const char *s,int o) {(void)s;(void)o;connected_rate=0;}
void di_on_call_failed(int s) {(void)s;connected_rate=0;}
int di_read_data(uint8_t *p,int n) {(void)p;(void)n;return 0;}
int di_write_data(const uint8_t *p,int n) {
    int copy=n; if(copy>65536-received_n)copy=65536-received_n;
    memcpy(received+received_n,p,copy); received_n+=copy; return copy;
}
void di_set_modulation_ops(di_ms_set_cb_t a,di_ms_get_cb_t b,di_ms_reset_cb_t c){(void)a;(void)b;(void)c;}
void di_set_modulation_limits_op(di_ms_limits_cb_t a){(void)a;}
void di_set_link_detail_cb(di_link_detail_cb_t a){(void)a;}
void di_set_break_cb(di_break_cb_t a){(void)a;}
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
