/* Phase-3 decision hook for the SmartLink DSP blob (dsplibs.o, i386).
 *
 * Interposes on V90Phase3Demodulator::getV90Decision(float) and
 * ::getV92Decision(float), the per-symbol Phase-3 state machines (34-state
 * jump table on this+0x28, symbol counter this+0x2c, per-state value this+0x30,
 * framePosition this+0x04).  Both have exactly one call site (R_386_PC32 from
 * the dispatcher at .text 0x2590a / 0x2591b), so a hook linked ahead of the
 * blob with --allow-multiple-definition catches every call.
 *
 *   /tmp/p3hook.txt  one line per state transition
 *   /tmp/p3hook.bin  one p3rec per call (P3HOOK_MAX records, default 2000000)
 *
 * Build: see rig/slmodemd/README.md ("Phase-3 decision hook").  Offsets are for
 * dsplibs.o e64b4a7 (AonCyberLabs); check `objdump -t dsplibs.o | grep Decision`.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

extern int orig_v90(void *self, float x) __asm__("_ZN20V90Phase3Demodulator14getV90DecisionEf_original");
extern int orig_v92(void *self, float x) __asm__("_ZN20V90Phase3Demodulator14getV92DecisionEf_original");
int hook_v90(void *self, float x) __asm__("_ZN20V90Phase3Demodulator14getV90DecisionEf");
int hook_v92(void *self, float x) __asm__("_ZN20V90Phase3Demodulator14getV92DecisionEf");

struct p3rec {
    uint32_t seq;
    uint8_t fn;             /* 0 = getV90Decision, 1 = getV92Decision */
    uint8_t st_before;
    uint8_t st_after;
    uint8_t b18;            /* this+0x18 (byte) */
    int32_t cnt;            /* this+0x2c before the call */
    int32_t v30;            /* this+0x30 after the call */
    int32_t frame_pos;      /* this+0x04 after the call */
    float x;                /* input sample */
    int32_t ret;
};

static FILE *txt, *bin;
static uint32_t seq;
static long max_rec = -1;

#define I32(p, o) (*(int32_t *)((char *)(p) + (o)))

static int wrap(int fn, void *self, float x)
{
    int before = I32(self, 0x28);
    int cnt = I32(self, 0x2c);
    int ret = fn ? orig_v92(self, x) : orig_v90(self, x);
    int after = I32(self, 0x28);

    if (max_rec < 0) {
        const char *e = getenv("P3HOOK_MAX");
        max_rec = e ? atol(e) : 2000000;
        txt = fopen("/tmp/p3hook.txt", "w");
        bin = fopen("/tmp/p3hook.bin", "wb");
    }
    if (after != before && txt) {
        fprintf(txt, "%u %s %d -> %d cnt=%d v30=%d framePos=%d x=%.1f ret=%d\n",
                seq, fn ? "V92" : "V90", before, after, cnt,
                I32(self, 0x30), I32(self, 0x04), x, ret);
        fflush(txt);
    }
    if (bin && (long)seq < max_rec) {
        struct p3rec r = { seq, (uint8_t)fn, (uint8_t)before, (uint8_t)after,
                           *((uint8_t *)self + 0x18), cnt, I32(self, 0x30),
                           I32(self, 0x04), x, ret };
        fwrite(&r, sizeof r, 1, bin);
        if ((seq & 4095) == 0)
            fflush(bin);
    }
    seq++;
    return ret;
}

int hook_v90(void *self, float x) { return wrap(0, self, x); }
int hook_v92(void *self, float x) { return wrap(1, self, x); }
