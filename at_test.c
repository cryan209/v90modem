/* ITU-T V.250 (07/2003) 6.7.2.10-.14. No live bearer processing. */
#include "at_test.h"
#include <spandsp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void at_test_reset(at_test_t *s) { memset(s, 0, sizeof(*s)); }
void at_test_disconnect(at_test_t *s)
{
    s->local_loop=false;
    s->type=0;
    s->blocks_left=0;
}

/* Strict decimal list: no empty fields, signs, wrapping or trailing text. */
static int numbers(const char *p, unsigned *out, int max)
{
    int n=0;
    while (*p) {
        if (n==max || *p<'0' || *p>'9') return -1;
        unsigned value=0;
        do {
            value=value*10+(unsigned)(*p++-'0');
            if (value>65535) return -1;
        } while (*p>='0' && *p<='9');
        out[n++]=value;
        if (!*p)break;
        if (*p++!=',' || !*p)return -1;
    }
    return n;
}

static int start(at_test_t *s, const unsigned *v)
{
    if (!s->local_loop || v[0]<1 || v[0]>3 || !v[1] || !v[2]
        || v[3]<1 || v[3]>4) return -1;
    /* V.250 6.7.2.11 pattern codes. The unsupported 63-bit pattern is not
     * advertised. SpanDSP calls the 2047 generator O.152_11, the 511 O.153_9. */
    static const int patterns[]={0,BERT_PATTERN_ITU_O153_9,
        BERT_PATTERN_ITU_O152_11,BERT_PATTERN_ONES,BERT_PATTERN_1_TO_1};
    int length=v[3]==1?511:v[3]==2?2047:v[3]==3?1:2;
    bert_state_t *bert=bert_init(NULL,0,patterns[v[3]],0,0);
    if (!bert)return -1;
    uint8_t pattern[2047];
    for (int i=0;i<length;i++)pattern[i]=(uint8_t)bert_get_bit(bert);
    bert_free(bert);
    s->type=s->last_type=(int)v[0];
    s->block_length=(int)v[1];s->blocks_left=v[2];
    s->pattern_id=(int)v[3];s->pattern_length=length;
    memcpy(s->pattern,pattern,(size_t)length);
    s->block_bits=0;s->block_bad=false;
    s->checked_bits=s->bit_errors=s->block_errors=0;
    s->tx_pos=s->rx_pos=0;
    return 0;
}
int at_test_get_bit(at_test_t *s)
{
    if (!s->type)return 1;
    int bit=s->pattern[s->tx_pos];
    s->tx_pos=(s->tx_pos+1)%s->pattern_length;
    return bit;
}
void at_test_put_bit(at_test_t *s, int bit)
{
    if (!s->type || bit<0)return;
    bool bad=(bit!=s->pattern[s->rx_pos]);
    s->rx_pos=(s->rx_pos+1)%s->pattern_length;
    s->bit_errors+=bad;
    s->block_bad|=bad;
    s->checked_bits++;
    if (++s->block_bits==(uint32_t)s->block_length) {
        s->block_errors+=s->block_bad;
        s->block_bad=false;s->block_bits=0;
        if (!--s->blocks_left)s->type=0;
    }
}
void at_test_clock_local(at_test_t *s, uint64_t bits)
{
    /* The established local interface loop returns the pattern at the same
     * bit position. This is not a measurement through a modulation datapump. */
    while (s->local_loop && s->type && bits--)
        at_test_put_bit(s,at_test_get_bit(s));
}
/* V.250 6.7.2.16 safe partial check for this software DCE: host arithmetic,
 * allocated working memory, and the local test controller. It is intentionally
 * not an analogue-interface test or an intrusive DSP/call reset. Volatile
 * memory accesses ensure the compiler actually performs the memory check. */
static bool safe_self_test(void)
{
    volatile uint8_t *memory=malloc(4096);
    if(!memory)return false;
    bool ok=true;
    for(int pass=0;pass<6;pass++) {
        for(int i=0;i<4096;i++) {
            uint8_t value=pass==0?0:pass==1?255:pass==2?0x55:pass==3?0xaa:
                pass==4?(uint8_t)i:(uint8_t)(i^0xaa);
            memory[i]=value;
        }
        for(int i=0;i<4096;i++) {
            uint8_t value=pass==0?0:pass==1?255:pass==2?0x55:pass==3?0xaa:
                pass==4?(uint8_t)i:(uint8_t)(i^0xaa);
            if(memory[i]!=value)ok=false;
        }
    }
    volatile uint32_t operands[2]={0x12345678U,0x10203040U};
    ok=ok && operands[0]+operands[1]==0x225486B8U;
    free((void *)memory);
    for(unsigned pattern=1;ok && pattern<=4;pattern++) {
        at_test_t scratch;
        unsigned params[4]={3,511,2,pattern};
        at_test_reset(&scratch);scratch.local_loop=true;
        if(start(&scratch,params)<0)return false;
        at_test_clock_local(&scratch,1022);
        ok=scratch.checked_bits==1022 && !scratch.type
            && !scratch.bit_errors && !scratch.block_errors;
        if(!ok || start(&scratch,params)<0)return false;
        for(int i=0;i<1022;i++)
            at_test_put_bit(&scratch,at_test_get_bit(&scratch)^(i==31));
        ok=scratch.checked_bits==1022 && !scratch.type
            && scratch.bit_errors==1 && scratch.block_errors==1;
    }
    return ok;
}

int at_test_command(at_test_t *s, const char *command, bool online,
                    char *response, size_t size)
{
    const char *arg=command;
    unsigned v[4];
    int n;
    if (!response || !size)return -1;
    response[0]='\0';
    if (!strncmp(arg,"+TLDL",5)) {
        arg+=5;
        if (!strcmp(arg,"?"))snprintf(response,size,"+TLDL: %d",s->local_loop);
        else if (!strcmp(arg,"=?"))snprintf(response,size,"+TLDL: (0,1)");
        else if (*arg=='=' && (n=numbers(arg+1,v,1))==1 && v[0]<=1 && online) {
            s->local_loop=v[0]!=0;
            if (!s->local_loop)at_test_disconnect(s);
        } else return -1;
        return 0;
    }
    if (!strncmp(arg,"+TTER",5)) {
        arg+=5;
        if (!strcmp(arg,"?"))snprintf(response,size,"+TTER: %d,%d,%u,%d",
            s->type,s->block_length,s->blocks_left,s->pattern_id);
        else if (!strcmp(arg,"=?"))snprintf(response,size,"+TTER: (0-3),(1-65535),(1-65535),(1-4)");
        else if (*arg=='=' && online) {
            n=numbers(arg+1,v,4);
            if (n==1 && !v[0]) {s->type=0;s->blocks_left=0;}
            else if (n!=4 || start(s,v)<0)return -1;
        } else return -1;
        return 0;
    }
    if (!strcmp(arg,"+TNUM?")) {
        /* 6.7.2.12: unavailable count is zero, public range ends at 65535.
         * Keep exact internal totals; saturate presentation rather than wrap.
         * The Recommendation's example prefixes this +TTER (an apparent
         * typo); use the queried parameter's +TNUM label consistently. */
        uint64_t bits=s->last_type==2?0:s->bit_errors;
        uint64_t blocks=s->last_type==1?0:s->block_errors;
        snprintf(response,size,"+TNUM: %llu,%llu",
            (unsigned long long)(bits>65535?65535:bits),
            (unsigned long long)(blocks>65535?65535:blocks));
        return 0;
    }
    if (!strcmp(arg,"+TNUM=?")) {snprintf(response,size,"+TNUM: (0-65535),(0-65535)");return 0;}
    /* Only point-to-point mode is implemented. No unsupported mode may OK. */
    if (!strcmp(arg,"+TMODE?")) {snprintf(response,size,"+TMODE: 0");return 0;}
    if (!strcmp(arg,"+TMODE=?")) {snprintf(response,size,"+TMODE: (0)");return 0;}
    if (!strcmp(arg,"+TMODE=0"))return 0;
    /* V.54 remote and analogue loop signalling is not wired to the engine.
     * 6.7.2.14 requires confirmation before OK; never acknowledge a stub. */
    if (!strcmp(arg,"+TRDL?") || !strcmp(arg,"+TRDLS?") || !strcmp(arg,"+TALS?")) {
        snprintf(response,size,"%.*s: 0",(int)strlen(arg)-1,arg);return 0;
    }
    if (!strcmp(arg,"+TRES?")) {snprintf(response,size,"+TRES: %d",s->self_result);return 0;}
    if (!strcmp(arg,"+TRES=?")) {snprintf(response,size,"+TRES: (0-2)");return 0;}
    if (!strcmp(arg,"+TSELF=?")) {snprintf(response,size,"+TSELF: (1)");return 0;}
    if (!strcmp(arg,"+TSELF=1")) {s->self_result=safe_self_test()?1:2;return 0;}
    /* Intrusive full self-test and external hardware diagnostics are absent. */
    return -1;
}
