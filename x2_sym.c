#include "x2_sym.h"
#include <string.h>
/* Draft 0.33 §12.2; Ie030002 E33D/E348/E355 source and
 * E571/E594/E440/E46B receiver, independently reconstructed. */
int x2_sym_startup_init(x2_sym_startup_t *s, unsigned answering)
{
    if (!s || answering > 1) return -1;
    memset(s, 0, sizeof(*s));
    s->answering = answering; s->tx_enabled = answering; s->phase = 4;
    return 0;
}
size_t x2_sym_startup_tx(x2_sym_startup_t *s, uint8_t *out, size_t count)
{
    size_t n = 0;
    if (!s || (!out && count)) return 0;
    while (n < count) {
        unsigned p = s->tx_position;
        if (!s->tx_enabled) { out[n++] = 0xff; continue; }
        if (p == 2010) break;
        if (p < 1747) out[n++] = 0x7e;
        else if (p < 1754) out[n++] = 0;
        else { p -= 1754; out[n++] = (uint8_t)((p & 1) ? 255-p/2 : p/2); }
        ++s->tx_position;
    }
    return n;
}
static void restart(x2_sym_startup_t *s)
{
    s->rx_stage = X2_SYM_RUN; s->run = s->zeros = s->ramp_position = 0;
    s->phase = 4; s->error_map = 0; ++s->restarts;
}
size_t x2_sym_startup_rx(x2_sym_startup_t *s, const uint8_t *in, size_t count)
{
    size_t n = 0;
    if (!s || (!in && count)) return 0;
    while (n < count && s->rx_stage != X2_SYM_ACQUIRED) {
        unsigned word = in[n++], error = 0;
        s->phase = (s->phase + 1) % 6;
        switch (s->rx_stage) {
        case X2_SYM_RUN:
            if ((word | 1) == 0x7f) {
                error = word & 1;
                if (++s->run == 7) s->rx_stage = X2_SYM_ZEROS;
            } else s->run = 0;
            break;
        case X2_SYM_ZEROS:
            if (word <= 3) {
                error = word & 1;
                if (word & 2) s->error_map |= 0x40;
                if (++s->zeros == 7) s->rx_stage = X2_SYM_RAMP;
            } else if ((word | 1) == 0x7f) {
                s->zeros = 0; s->error_map &= ~0x40u; error = word & 1;
            }
            else restart(s);
            break;
        case X2_SYM_RAMP: {
            unsigned p = s->ramp_position;
            unsigned expected = (p & 1) ? 255-p/2 : p/2;
            if ((word | 1) != (expected | 1)) {
                if (p == 0 && word == 2) s->error_map |= 0x40;
                else { restart(s); break; }
            } else error = word != expected;
            if (++s->ramp_position == 256) {
                s->rx_stage = X2_SYM_ACQUIRED;
                if (!s->answering) s->tx_enabled = 1;
            }
            break;
        }
        case X2_SYM_ACQUIRED: break;
        }
        if (error) s->error_map |= 1u << s->phase;
    }
    return n;
}
static uint8_t reverse_octet(uint8_t x)
{
    x = (uint8_t)(((x & 0x55) << 1) | ((x >> 1) & 0x55));
    x = (uint8_t)(((x & 0x33) << 2) | ((x >> 2) & 0x33));
    return (uint8_t)((x << 4) | (x >> 4));
}
/* Draft 0.33 §13.2–.3: mask, LSB-first GPC, then octet reversal.
 * Receive reverses before masking; wire octets must remain byte-exact. */
int x2_sym_data_init(x2_sym_data_t *s, unsigned width, unsigned scramble, unsigned reverse)
{
    if (!s || (width != 7 && width != 8) || scramble > 1 || reverse > 1) return -1;
    memset(s, 0, sizeof(*s));
    s->width = width; s->scramble = scramble; s->reverse = reverse;
    x2_scrambler_init(&s->tx, 18, 0); x2_scrambler_init(&s->rx, 18, 0);
    return 0;
}
uint8_t x2_sym_data_tx(x2_sym_data_t *s, uint8_t source)
{
    uint8_t out = 0;
    for (unsigned b = 0; b < s->width; ++b) {
        unsigned bit = (source >> b) & 1;
        out |= (uint8_t)((s->scramble ? x2_scramble_bit(&s->tx, bit) : bit) << b);
    }
    return s->reverse ? reverse_octet(out) : out;
}
uint8_t x2_sym_data_rx(x2_sym_data_t *s, uint8_t octet)
{
    uint8_t out = 0;
    if (s->reverse) octet = reverse_octet(octet);
    for (unsigned b = 0; b < s->width; ++b) {
        unsigned bit = (octet >> b) & 1;
        out |= (uint8_t)((s->scramble ? x2_descramble_bit(&s->rx, bit) : bit) << b);
    }
    return out;
}
/* Draft 0.33 §12.2: Ie030002 E36E..E3D9 and E499..E568.
 * CRC consumes full body octets, then four LSB-first nibbles. */
static uint16_t crc_bits(uint16_t crc, unsigned value, unsigned count)
{
    for (unsigned i=0;i<count;++i)
        crc=(uint16_t)((crc>>1)^(((crc^(value>>i))&1)?0x8408:0));
    return crc;
}
int x2_sym_cap_build(x2_sym_cap_t *cap, unsigned errors, unsigned mask, unsigned scramble)
{
    if (!cap || errors>0x7f || mask>0x7f || scramble>1) return -1;
    cap->words[0]=cap->words[1]=1;
    cap->words[2]=(uint8_t)(((errors<<1)&0x7e)|1);
    cap->words[3]=(uint8_t)(1|(!errors?2|(scramble?8:0):0)|((errors&mask&0x40)?4:0));
    cap->words[4]=(uint8_t)(((mask<<1)&0x7e)|1);
    return 0;
}
int x2_sym_cap_encode(const x2_sym_cap_t *cap, uint8_t *out)
{
    uint16_t crc=0xffff;
    if (!cap || !out) return -1;
    for(unsigned i=0;i<5;++i)if(!(cap->words[i]&1)||cap->words[i]>0x7f)return -1;
    out[0]=out[1]=0x81;
    for(unsigned i=0;i<5;++i) { out[2+i]=cap->words[i];crc=crc_bits(crc,cap->words[i],8); }
    crc^=0xffff;
    for(unsigned i=0;i<4;++i)out[7+i]=(uint8_t)((((crc>>(i*4))&15)<<1)|1);
    return 0;
}
int x2_sym_cap_decode(const uint8_t *in, x2_sym_cap_t *cap)
{
    x2_sym_cap_t result;
    uint16_t crc=0xffff;
    if(!in||!cap||(in[0]|1)!=0x81||(in[1]|1)!=0x81)return -1;
    for(unsigned i=0;i<5;++i) {
        if(!(in[2+i]&1)||in[2+i]>0x7f)return -1;
        result.words[i]=in[2+i];crc=crc_bits(crc,in[2+i],8);
    }
    for(unsigned i=0;i<4;++i) {
        if(!(in[7+i]&1)||in[7+i]>0x1f)return -1;
        crc=crc_bits(crc,in[7+i]>>1,4);
    }
    if(crc!=0xf0b8)return -1;
    *cap=result;return 0;
}
unsigned x2_sym_cap_merge(const x2_sym_cap_t *local, const x2_sym_cap_t *peer,
                          unsigned *errors, unsigned *mask, unsigned *scramble)
{
    unsigned le,pe,lm,pm,merged,rate;
    uint8_t check[11];
    if(!local||!peer||!errors||!mask||!scramble)return 0;
    if(x2_sym_cap_encode(local,check)||x2_sym_cap_encode(peer,check))return 0;
    le=(local->words[2]>>1)|((local->words[3]&4)<<4);
    pe=(peer->words[2]>>1)|((peer->words[3]&4)<<4);
    lm=(local->words[4]>>1)|((local->words[3]&2)<<5);
    pm=(peer->words[4]>>1)|((peer->words[3]&2)<<5);
    merged=lm&pm;rate=(merged&0x40)?64000:(merged&1)?56000:0;
    if(!rate)return 0;
    *errors=le|pe;*mask=merged;*scramble=!!(local->words[3]&peer->words[3]&8);
    return rate;
}
/* Draft 0.33 §12.2. Ie030002 E51B..E528, E3F9..E41B, E548..E56D:
 * answer acknowledges first (FF plus four 81s); caller acknowledges only
 * after receiving those four. TX and RX data boundaries are independent. */
int x2_sym_link_init(x2_sym_link_t *s, unsigned answering,
                     x2_get_bit_func_t get_bit, x2_put_bit_func_t put_bit, void *context)
{
    if(!s || answering>1)return -1;
    memset(s,0,sizeof(*s));x2_sym_startup_init(&s->startup,answering);
    s->get_bit=get_bit;s->put_bit=put_bit;s->context=context;
    return 0;
}
void x2_sym_link_tx(x2_sym_link_t *s, uint8_t *out, size_t count)
{
    for(size_t n=0;n<count;++n) {
        if(s->failed){out[n]=0xff;continue;}
        if(s->tx_data) {
            unsigned word=0;
            for(unsigned b=0;b<s->data.width;++b) {
                int bit=s->get_bit?s->get_bit(s->context):1;
                word|=(unsigned)(bit<0?1:bit&1)<<b;
            }
            out[n]=x2_sym_data_tx(&s->data,(uint8_t)word);continue;
        }
        if(s->confirm_tx) {
            out[n]=(s->confirm_tx==1)?0xff:0x81;
            if(++s->confirm_tx==6){s->confirm_tx=0;s->tx_data=1;}
            continue;
        }
        /* E365..E368 repeats the source. E48D selects the answerer's
         * capability TX only after peer ramp; E527 selects the caller's
         * capability TX only after peer CRC. Sending it early suppresses
         * the native caller's source before its first octet. */
        if((s->startup.answering && s->startup.rx_stage!=X2_SYM_ACQUIRED)
           || (!s->startup.answering && !s->cap_valid)) {
            if(s->startup.tx_position==2010)s->startup.tx_position=0;
            x2_sym_startup_tx(&s->startup,out+n,1);continue;
        }
        if(!s->frame_position) {
            x2_sym_cap_build(&s->local,s->startup.error_map,0x41,1);
            s->frame[0]=0xff;x2_sym_cap_encode(&s->local,s->frame+1);
        }
        out[n]=s->frame[s->frame_position++];s->frame_position%=12;
    }
}
void x2_sym_link_rx(x2_sym_link_t *s, const uint8_t *in, size_t count)
{
    for(size_t n=0;n<count;++n) {
        if(s->failed)continue;
        if(s->rx_data) {
            unsigned word=x2_sym_data_rx(&s->data,in[n]);
            if(s->put_bit)for(unsigned b=0;b<s->data.width;++b)s->put_bit(s->context,(word>>b)&1);
            continue;
        }
        if(s->startup.rx_stage!=X2_SYM_ACQUIRED) {
            x2_sym_startup_rx(&s->startup,in+n,1);continue;
        }
        if(s->cap_valid) {
            if((in[n]|1)==0x81)++s->confirm_rx;else s->confirm_rx=0;
            if(s->confirm_rx==4) {
                s->rx_data=1;
                if(!s->startup.answering)s->confirm_tx=1;
            }
            continue;
        }
        if(s->receive_count<11)s->receive[s->receive_count++]=in[n];
        else {memmove(s->receive,s->receive+1,10);s->receive[10]=in[n];}
        if(s->receive_count==11 && !x2_sym_cap_decode(s->receive,&s->peer)) {
            x2_sym_cap_build(&s->local,s->startup.error_map,0x41,1);
            s->rate=x2_sym_cap_merge(&s->local,&s->peer,&s->errors,&s->mask,&s->scramble);
            if(!s->rate){s->failed=1;continue;}
            x2_sym_data_init(&s->data,s->rate==64000?8:7,s->scramble,1);
            s->cap_valid=1;
            if(s->startup.answering)s->confirm_tx=1;
        }
    }
}
