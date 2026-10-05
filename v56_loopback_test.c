/* Offline V.56ter (08/96) 6.3.1-inspired synchronous BER exercise.
 * Two independent modems exchange the 511-bit pattern through a
 * sample-preserving synthetic line. This is not the complete V.56bis
 * network model or a V.56ter certification test. See docs/v56_loopback.md.
 * Impairments apply to simulated analog samples, never live RTP codewords.
 */
#include <spandsp.h>
#include "v34_line_ec.h"
#include "v56bis_filters.h"
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BLOCK 160
#define PERIOD 511
#define RING 8192
#define TX_DBM0 (-12.0f)

typedef struct {
    int baud, rate, bits, seconds, delay, echo_delay, seed, ad, edd;
    double snr, loss, echo_db;
    int json, cancel_echo;
    int modulation; /* 0 V.34, 1 V.32bis, 2 V.22bis */
} options_t;

typedef struct {
    uint8_t pattern[PERIOD];
    uint64_t signatures[PERIOD], window;
    int tx_pos, rx_pos, hunt_bits, synced, failed;
    uint64_t bits, errors, skipped, samples, sync_sample;
    int target;
} endpoint_t;

typedef struct {
    int16_t delay[RING], echo[RING];
    unsigned pos;
    double gain, echo_gain;
    const double *filter;
    awgn_state_t *noise;
    uint64_t clips;
} line_t;

static void pattern_init(endpoint_t *e, int phase, int target)
{
    /* V.56ter 6.3.1.5 specifies the 511 pattern. SpanDSP names this
     * generator O.153_9: x^9 + x^5 + 1, not its 2047-bit default BERT. */
    bert_state_t *b = bert_init(NULL, 0, BERT_PATTERN_ITU_O153_9, 0, 0);
    if (!b) { fprintf(stderr, "BERT allocation failed\n"); exit(2); }
    memset(e, 0, sizeof(*e));
    for (int i = 0; i < PERIOD; i++) e->pattern[i] = (uint8_t)bert_get_bit(b);
    bert_free(b);
    for (int p = 0; p < PERIOD; p++)
        for (int i = 0; i < 64; i++)
            e->signatures[p] = (e->signatures[p] << 1) | e->pattern[(p+i)%PERIOD];
    e->tx_pos = phase;
    e->target = target;
}

static int get_bit(void *ctx)
{
    endpoint_t *e = ctx;
    int bit = e->pattern[e->tx_pos];
    e->tx_pos = (e->tx_pos + 1)%PERIOD;
    return bit;
}

static void put_bit(void *ctx, int bit)
{
    endpoint_t *e = ctx;
    if (bit < 0) {
        /* Startup tone transitions can intentionally drop carrier. A missing
         * acquisition is still bounded by the simulated deadline. */
        if (bit == SIG_STATUS_TRAINING_FAILED || (bit == SIG_STATUS_CARRIER_DOWN && e->synced))
            e->failed = 1;
        return;
    }
    if (!e->synced) {
        e->window = (e->window << 1) | (bit & 1);
        e->hunt_bits++;
        e->skipped++;
        if (e->hunt_bits >= 64)
            for (int p = 0; p < PERIOD; p++)
                if (e->window == e->signatures[p]) {
                    e->synced = 1;
                    e->rx_pos = (p+64)%PERIOD;
                    e->sync_sample = e->samples;
                    break;
                }
        return;
    }
    /* Never re-lock after acquisition: lost bits and long error bursts must
     * remain visible. Grade precisely --bits in each direction. */
    if (e->bits >= (uint64_t)e->target) return;
    e->errors += (bit != e->pattern[e->rx_pos]);
    e->bits++;
    e->rx_pos = (e->rx_pos+1)%PERIOD;
}

static int16_t saturate(double v, uint64_t *clips)
{
    if (v > 32767) { (*clips)++; return 32767; }
    if (v < -32768) { (*clips)++; return -32768; }
    return (int16_t)lrint(v);
}

static void line_init(line_t *l, const options_t *o, int direction)
{
    memset(l, 0, sizeof(*l));
    if (o->ad)
        for (int i=0;i<6;i++)
            if (v56bis_ad_numbers[i]==o->ad)
                l->filter=v56bis_filter_bank[i][o->edd-1];
    l->gain = pow(10.0, -o->loss/20.0);
    /* A synthetic three-tap near-end hybrid, normalized for return loss.
     * These are harness parameters, not an ITU local-loop coefficient set. */
    l->echo_gain = o->echo_db < 0 ? 0 : pow(10.0, -o->echo_db/20.0)/sqrt(.905);
    if (isfinite(o->snr)) {
        l->noise = awgn_init_dbm0(NULL, o->seed + direction*104729,
                                 (float)(TX_DBM0-o->loss-o->snr));
        if (!l->noise) { fprintf(stderr, "noise allocation failed\n"); exit(2); }
    }
}

static int16_t line_sample(line_t *l, const options_t *o,
                           int16_t remote, int16_t local, int alaw)
{
    l->delay[l->pos] = remote;
    l->echo[l->pos] = local;
    double remote_filtered = 0;
    unsigned delayed = l->pos-(unsigned)o->delay;
    if (l->filter)
        for (int k=0;k<V56BIS_FILTER_TAPS;k++)
            remote_filtered += l->filter[k]*l->delay[(delayed-(unsigned)k)&(RING-1)];
    else
        remote_filtered = l->delay[delayed&(RING-1)];
    double v = l->gain*remote_filtered;
    if (l->echo_gain) {
        unsigned p = l->pos-(unsigned)o->echo_delay;
        v += l->echo_gain*(.80*l->echo[p&(RING-1)]
                        - .45*l->echo[(p-1)&(RING-1)]
                        + .25*l->echo[(p-2)&(RING-1)]);
    }
    if (l->noise) v += awgn(l->noise);
    l->pos = (l->pos+1)&(RING-1);
    int16_t x = saturate(v, &l->clips);
    return alaw ? alaw_to_linear(linear_to_alaw(x))
                : ulaw_to_linear(linear_to_ulaw(x));
}

static int run(const options_t *o, int alaw)
{
    endpoint_t a, b;
    pattern_init(&a, 0, o->bits);
    pattern_init(&b, 137, o->bits);
    line_t *line = calloc(2, sizeof(*line));
    v34_line_ec_t *ec = calloc(2, sizeof(*ec));
    if (!line || !ec) { free(line); free(ec); return 2; }
    line_init(&line[0], o, 0);
    line_init(&line[1], o, 1);
    v34_state_t *ma = NULL, *mb = NULL;
    v32bis_state_t *xa = NULL, *xb = NULL;
    v22bis_state_t *ya = NULL, *yb = NULL;
    int initialized = 0;
    if (o->modulation == 0) {
        ma = v34_init(NULL, o->baud, o->rate, true, true, get_bit, &a, put_bit, &a);
        mb = v34_init(NULL, o->baud, o->rate, false, true, get_bit, &b, put_bit, &b);
        if (ma && mb) {
            v34_tx_power(ma, TX_DBM0); v34_tx_power(mb, TX_DBM0);
            /* V.56ter 6.3.1.3: request and verify a fixed line rate. */
            v34_set_mp_rate_policy(ma, o->rate/2400, o->rate/2400);
            v34_set_mp_rate_policy(mb, o->rate/2400, o->rate/2400);
            initialized = 1;
        }
    } else if (o->modulation == 1) {
        int mask = o->rate == 4800 ? V32BIS_RATE_4800 : o->rate == 7200 ? V32BIS_RATE_7200
                 : o->rate == 9600 ? V32BIS_RATE_9600 : o->rate == 12000 ? V32BIS_RATE_12000 : V32BIS_RATE_14400;
        xa = v32bis_init(NULL, o->rate, true, get_bit, &a, put_bit, &a);
        xb = v32bis_init(NULL, o->rate, false, get_bit, &b, put_bit, &b);
        if (xa && xb) {
            v32bis_tx_power(xa, TX_DBM0); v32bis_tx_power(xb, TX_DBM0);
            /* V.32bis clause 6: reactive tone/startup dialogue, one offered rate. */
            initialized = !v32bis_set_supported_bit_rates(xa, mask)
                       && !v32bis_set_supported_bit_rates(xb, mask)
                       && !v32bis_set_echo_canceller(xa, o->cancel_echo)
                       && !v32bis_set_echo_canceller(xb, o->cancel_echo)
                       && !v32bis_start_tones(xa) && !v32bis_start_tones(xb);
        }
    } else {
        ya = v22bis_init(NULL, o->rate, 0, true, get_bit, &a, put_bit, &a);
        yb = v22bis_init(NULL, o->rate, 0, false, get_bit, &b, put_bit, &b);
        if (ya && yb) {
            v22bis_tx_power(ya, TX_DBM0); v22bis_tx_power(yb, TX_DBM0);
            initialized = 1;
        }
    }
    if (!initialized) {
        fprintf(stderr, "modem initialization failed\n");
        if (ma) v34_free(ma); if (mb) v34_free(mb);
        if (xa) v32bis_free(xa); if (xb) v32bis_free(xb);
        if (ya) v22bis_free(ya); if (yb) v22bis_free(yb);
        for (int d=0; d<2; d++) if (line[d].noise) awgn_free(line[d].noise);
        free(line); free(ec); return 2;
    }
    int16_t ta[BLOCK], tb[BLOCK], ra[BLOCK], rb[BLOCK];
    int blocks = 0;
    for (; blocks < o->seconds*50; ) {
        int na = ma ? v34_tx(ma, ta, BLOCK) : xa ? v32bis_tx(xa, ta, BLOCK) : v22bis_tx(ya, ta, BLOCK);
        int nb = mb ? v34_tx(mb, tb, BLOCK) : xb ? v32bis_tx(xb, tb, BLOCK) : v22bis_tx(yb, tb, BLOCK);
        if (na < 0 || na > BLOCK || nb < 0 || nb > BLOCK) { a.failed = 1; break; }
        memset(ta+na, 0, (BLOCK-na)*sizeof(*ta));
        memset(tb+nb, 0, (BLOCK-nb)*sizeof(*tb));
        for (int i=0; i<BLOCK; i++) {
            rb[i] = line_sample(&line[0], o, ta[i], tb[i], alaw);
            ra[i] = line_sample(&line[1], o, tb[i], ta[i], alaw);
        }
        if (ma && o->echo_db >= 0 && o->cancel_echo) {
            char msg[256];
            v34_line_ec_tx(&ec[0], tb, BLOCK);
            v34_line_ec_tx(&ec[1], ta, BLOCK);
            if (v34_line_ec_rx(&ec[0], rb, BLOCK, v34_rx_line_ec_window(mb), msg, sizeof(msg)))
                fprintf(stderr, "[EC B] %s\n", msg);
            if (v34_line_ec_rx(&ec[1], ra, BLOCK, v34_rx_line_ec_window(ma), msg, sizeof(msg)))
                fprintf(stderr, "[EC A] %s\n", msg);
        }
        blocks++;
        a.samples = b.samples = (uint64_t)blocks*BLOCK;
        if (ma) { v34_rx(mb, rb, BLOCK); v34_rx(ma, ra, BLOCK); }
        else if (xa) { v32bis_rx(xb, rb, BLOCK); v32bis_rx(xa, ra, BLOCK); }
        else { v22bis_rx(yb, rb, BLOCK); v22bis_rx(ya, ra, BLOCK); }
        if (a.failed || b.failed || (a.bits >= (uint64_t)o->bits && b.bits >= (uint64_t)o->bits)) break;
    }
    int complete = a.bits == (uint64_t)o->bits && b.bits == (uint64_t)o->bits;
    int a_ba=0, a_ab=0, b_ba=0, b_ab=0;
    int rates_available;
    if (ma) {
        rates_available = v34_get_negotiated_mp_rates(ma,&a_ba,&a_ab)==0
                       && v34_get_negotiated_mp_rates(mb,&b_ba,&b_ab)==0;
        a_ba *= 2400; a_ab *= 2400; b_ba *= 2400; b_ab *= 2400;
    } else {
        a_ba = a_ab = xa ? v32bis_current_bit_rate(xa) : v22bis_get_current_bit_rate(ya);
        b_ba = b_ab = xb ? v32bis_current_bit_rate(xb) : v22bis_get_current_bit_rate(yb);
        rates_available = a.synced && b.synced;
    }
    int rates_ok = rates_available && a_ab==b_ab && a_ba==b_ba
                 && a_ab==o->rate && a_ba==o->rate;
    const char *status = (a.failed || b.failed) ? "carrier_lost" : !complete ? "timeout"
                       : (a.errors || b.errors) ? "errors" : !rates_ok ? "rate_mismatch" : "pass";
    /* A receives B->A; B receives A->B. Report this explicitly. */
    if (o->json) {
        printf("{\"modulation\":\"%s\",", o->modulation==0?"v34":o->modulation==1?"v32bis":"v22bis");
        printf("\"status\":\"%s\",\"baud\":%d,\"requested_bps\":%d,\"law\":\"%s\","
               "\"seed\":%d,\"snr_db\":", status,o->baud,o->rate,alaw?"alaw":"ulaw",o->seed);
        if (isfinite(o->snr)) printf("%.6g", o->snr); else printf("null");
        printf(",\"loss_db\":%.6g,\"delay_samples\":%d,\"ad\":%d,\"edd\":%d,\"filter_nominal_delay_samples\":%d,\"echo_db\":",
               o->loss,o->delay,o->ad,o->edd,o->ad?(V56BIS_FILTER_TAPS-1)/2:0);
        if (o->echo_db >= 0) printf("%.6g",o->echo_db); else printf("null");
        printf(",\"a_ab_bps\":%d,\"a_ba_bps\":%d,\"b_ab_bps\":%d,\"b_ba_bps\":%d", a_ab,a_ba,b_ab,b_ba);
        printf(",\"echo_delay_samples\":%d,\"echo_cancel\":%s,\"target_bits\":%d,\"elapsed_s\":%.2f,"
               "\"ab_bits\":%llu,\"ab_errors\":%llu,\"ba_bits\":%llu,\"ba_errors\":%llu,"
               "\"ab_synced\":%s,\"ba_synced\":%s,\"ab_sync_s\":%.2f,\"ba_sync_s\":%.2f,\"ab_clips\":%llu,\"ba_clips\":%llu,"
               "\"rates_available\":%s,\"a_mp_ab_bps\":%d,\"a_mp_ba_bps\":%d,\"b_mp_ab_bps\":%d,\"b_mp_ba_bps\":%d}\n",
               o->echo_delay,(o->modulation!=2 && o->cancel_echo)?"true":"false",o->bits,blocks*.02,
               (unsigned long long)b.bits,(unsigned long long)b.errors,
               (unsigned long long)a.bits,(unsigned long long)a.errors,
               b.synced?"true":"false",a.synced?"true":"false",
               b.sync_sample/8000.0,a.sync_sample/8000.0,
               (unsigned long long)line[0].clips,(unsigned long long)line[1].clips,
               rates_available?"true":"false",a_ab,a_ba,b_ab,b_ba);
    } else {
        printf("V.56 loopback %s: %d/%d/%s, %.2fs; A->B %llu bits/%llu errors, B->A %llu bits/%llu errors\n",
               status,o->baud,o->rate,alaw?"alaw":"ulaw",blocks*.02,
               (unsigned long long)b.bits,(unsigned long long)b.errors,
               (unsigned long long)a.bits,(unsigned long long)a.errors);
        printf("  pattern=511; sync A->B %.2fs B->A %.2fs; seed=%d; snr=%.1f dB; loss=%.1f dB; delay=%d samples; echo=%.1f dB\n",
               b.sync_sample/8000.0,a.sync_sample/8000.0,o->seed,o->snr,o->loss,o->delay,o->echo_db);
        printf("  settled rates: A sees A->B %d B->A %d; B sees A->B %d B->A %d\n",
               a_ab,a_ba,b_ab,b_ba);
        if (o->ad) printf("  V.56bis AD-%d / EDD-%d FIR; nominal reference delay %d samples per direction\n",
                          o->ad,o->edd,(V56BIS_FILTER_TAPS-1)/2);
    }
    if (ma) v34_free(ma); if (mb) v34_free(mb);
    if (xa) v32bis_free(xa); if (xb) v32bis_free(xb);
    if (ya) v22bis_free(ya); if (yb) v22bis_free(yb);
    for (int d=0; d<2; d++) if (line[d].noise) awgn_free(line[d].noise);
    free(line); free(ec);
    return strcmp(status,"pass") ? 1 : 0;
}

static void usage(void)
{
    puts("Usage: v56_loopback_test [--baud 2400] [--rate 9600] [--law ulaw|alaw]\n"
         "  --modulation v34|v32bis|v22bis (default v34)\n"
         "  --bits N          checked bits per direction (default 1000000)\n"
         "  --seconds N       simulated call deadline (default 240)\n"
         "  --snr-db DB       AWGN relative to nominal received -12 dBm0 TX (default off)\n"
         "  --loss-db DB      one-way attenuation (default 0)\n"
         "  --delay N         one-way delay in 8 kHz samples (default 0)\n"
         "  --ad N --edd N    Table A.10 AD {1,5,6,7,8,9} and A.11 EDD {1,2,3}\n"
         "                    both required; FIR adds 256 samples nominal delay\n"
         "  --echo-db DB      local echo return loss (default off)\n"
         "  --echo-delay N    local echo delay in samples (default 2136)\n"
         "  --seed N          reproducible noise seed (default 1)\n"
         "  --no-echo-cancel   bypass V.34 line echo canceller\n"
         "  --json            one JSON result on stdout\n"
         "  --self-test       test pattern/checker and synthetic channel\n"
         "Exit 0: both directions exact; 1: errors/timeout/carrier loss; 2: invalid setup.");
}

static int number(const char *s, double min, double max, int integer, double *out)
{
    char *end; errno=0;
    double v = strtod(s,&end);
    if (errno || end==s || *end || !isfinite(v) || v<min || v>max || (integer && v!=floor(v))) return 0;
    *out=v; return 1;
}

static int self_test(void)
{
    options_t o = {.delay=3,.echo_db=-1,.seed=1,.snr=INFINITY};
    endpoint_t e; pattern_init(&e,0,1024);
    /* Verify the exact period, distinct phase signatures, and a detector
     * that does not forgive injected errors or re-lock after acquisition. */
    bert_state_t *b=bert_init(NULL,0,BERT_PATTERN_ITU_O153_9,0,0);
    int ones=0, bad=0;
    for (int i=0;i<PERIOD*2;i++) {
        int bit=bert_get_bit(b);
        bad |= bit!=e.pattern[i%PERIOD];
        if (i<PERIOD) ones+=bit;
    }
    bert_free(b);
    bad |= ones!=256;
    for (int i=0;i<PERIOD;i++) for (int j=i+1;j<PERIOD;j++) bad |= e.signatures[i]==e.signatures[j];
    put_bit(&e, SIG_STATUS_CARRIER_DOWN);
    bad |= e.failed;
    for (int i=0;i<64+1024;i++) put_bit(&e,e.pattern[i%PERIOD] ^ (i==100 || i==700));
    bad |= !e.synced || e.bits!=1024 || e.errors!=2;
    put_bit(&e, SIG_STATUS_CARRIER_DOWN);
    bad |= !e.failed;
    pattern_init(&e,0,1024);
    for (int i=0;i<2048;i++) put_bit(&e,0);
    bad |= e.synced || e.bits!=0;
    line_t l; line_init(&l,&o,0);
    for (int i=0;i<8;i++) {
        int16_t got=line_sample(&l,&o,i==0?1000:0,0,0);
        int16_t want=ulaw_to_linear(linear_to_ulaw(i==3?1000:0));
        bad |= got!=want;
    }
    o.delay=0;
    line_init(&l,&o,0);
    for (int i=-32000;i<=32000;i+=97)
        bad |= line_sample(&l,&o,(int16_t)i,0,1)!=alaw_to_linear(linear_to_alaw((int16_t)i));
    o.loss=6;
    line_init(&l,&o,0);
    uint64_t clips=0;
    int16_t attenuated=saturate(1000*pow(10.0,-6.0/20.0),&clips);
    bad |= line_sample(&l,&o,1000,0,0)!=ulaw_to_linear(linear_to_ulaw(attenuated));
    o.loss=0; o.echo_db=20; o.echo_delay=3;
    line_init(&l,&o,0);
    for (int i=0;i<8;i++) {
        double tap=i==3?.80:i==4?-.45:i==5?.25:0;
        int16_t expected=saturate(1000*.1*tap/sqrt(.905),&clips);
        bad |= line_sample(&l,&o,0,i==0?1000:0,0)!=ulaw_to_linear(linear_to_ulaw(expected));
    }
    o.echo_db=-1;
    line_t *n=calloc(3,sizeof(*n));
    if (!n) return 2;
    o.snr=30; for (int i=0;i<3;i++) line_init(&n[i],&o,i==2?1:0);
    int differs=0;
    for (int i=0;i<1000;i++) {
        int a=line_sample(&n[0],&o,1000,0,0),b2=line_sample(&n[1],&o,1000,0,0);
        int c=line_sample(&n[2],&o,1000,0,0);
        bad |= a!=b2; differs |= a!=c;
    }
    bad |= !differs;
    for (int i=0;i<3;i++) awgn_free(n[i].noise);
    free(n);
    /* Exercise the actual streaming convolution, not only the generator's
     * frequency-domain checks: coefficient order, delay, and gain matter. */
    o.ad=1; o.edd=3; o.snr=INFINITY; o.delay=7;
    line_init(&l,&o,0);
    for (int i=0;i<V56BIS_FILTER_TAPS+8;i++) {
        double target=(i>=7 && i<7+V56BIS_FILTER_TAPS)?10000*v56bis_ad1_edd3[i-7]:0;
        int16_t expected=saturate(target,&clips);
        bad |= line_sample(&l,&o,i==0?10000:0,0,0)!=ulaw_to_linear(linear_to_ulaw(expected));
    }
    puts(bad?"V.56 harness self-test FAIL":"V.56 harness self-test PASS");
    return bad?1:0;
}

int main(int argc, char **argv)
{
    options_t o={.baud=2400,.rate=9600,.bits=1000000,.seconds=240,
                 .echo_delay=2136,.seed=1,.snr=INFINITY,.echo_db=-1,.cancel_echo=1};
    int alaw=0;
    for (int i=1;i<argc;i++) {
        const char *k=argv[i]; double v;
        if (!strcmp(k,"--help")) { usage(); return 0; }
        if (!strcmp(k,"--self-test")) return argc==2?self_test():2;
        if (!strcmp(k,"--json")) { o.json=1; continue; }
        if (!strcmp(k,"--no-echo-cancel")) { o.cancel_echo=0; continue; }
        if (++i==argc) { fprintf(stderr,"Missing value for %s\n",k); return 2; }
        if (!strcmp(k,"--modulation")) {
            if (!strcmp(argv[i],"v34")) o.modulation=0;
            else if (!strcmp(argv[i],"v32bis")) o.modulation=1;
            else if (!strcmp(argv[i],"v22bis")) o.modulation=2;
            else return 2;
            continue;
        }
        if (!strcmp(k,"--law")) {
            if (strcmp(argv[i],"ulaw") && strcmp(argv[i],"alaw")) return 2;
            alaw=!strcmp(argv[i],"alaw"); continue;
        }
        if (!number(argv[i],0,200000000,0,&v)) { fprintf(stderr,"Invalid value for %s\n",k); return 2; }
        if (!strcmp(k,"--snr-db") && v<=100) o.snr=v;
        else if (!strcmp(k,"--loss-db") && v<=60) o.loss=v;
        else if (!strcmp(k,"--echo-db") && v<=120) o.echo_db=v;
        else if (v!=floor(v)) return 2;
        else if (!strcmp(k,"--baud") && (v==600 || v==2400 || v==2743 || v==2800 || v==3000 || v==3200 || v==3429)) o.baud=(int)v;
        else if (!strcmp(k,"--rate") && v>=1200 && v<=33600 && (int)v%1200==0) o.rate=(int)v;
        else if (!strcmp(k,"--bits") && v>=1 && v<=100000000) o.bits=(int)v;
        else if (!strcmp(k,"--seconds") && v>=1 && v<=86400) o.seconds=(int)v;
        else if (!strcmp(k,"--delay") && v<RING) o.delay=(int)v;
        else if (!strcmp(k,"--ad") && (v==1 || v==5 || v==6 || v==7 || v==8 || v==9)) o.ad=(int)v;
        else if (!strcmp(k,"--edd") && v>=1 && v<=3) o.edd=(int)v;
        else if (!strcmp(k,"--echo-delay") && v<=6000) o.echo_delay=(int)v;
        else if (!strcmp(k,"--seed") && v>=1 && v<=1000000) o.seed=(int)v;
        else { fprintf(stderr,"Unknown option or invalid value: %s %s\n",k,argv[i]); return 2; }
    }
    if ((o.modulation==0 && (o.rate%2400 || o.baud==600))
        || (o.modulation==1 && (o.baud!=2400 || o.rate<4800 || o.rate>14400 || o.rate%2400))
        || (o.modulation==2 && (o.rate!=1200 && o.rate!=2400))) {
        fprintf(stderr,"Unsupported modulation/baud/rate combination\n"); return 2;
    }
    if (o.modulation==2) o.baud=600;
    if ((!o.ad)!=(!o.edd)) { fprintf(stderr,"--ad and --edd must be used together\n"); return 2; }
    if (o.ad && o.delay>RING-V56BIS_FILTER_TAPS) {
        fprintf(stderr,"Filtered delay must be <= %d samples\n",RING-V56BIS_FILTER_TAPS); return 2;
    }
    return run(&o,alaw);
}
