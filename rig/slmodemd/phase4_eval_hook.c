/* Logs every V90ConnectionEvaluator::evaluatePhase4(float ratio) call of the
 * SmartLink DSP blob (dsplibs.o, i386; .text 0x3fad0, called from
 * V90Demodulator::progress with V90Equalizer's
 * ph4MeanErrorEnergyBeforeToAfterUpdateRatio) to /tmp/p4eval.txt: the ratio,
 * the evaluator's state and the V90Parameters thresholds it reads, and the
 * verdict (4 = retrain, 5 = fall back to V.34).  The parameters block is
 * *(this+0). */
#include <stdio.h>
#include <stdint.h>

extern int orig_eval(void *self, float ratio) __asm__("_ZN22V90ConnectionEvaluator14evaluatePhase4Ef_original");
int hook_eval(void *self, float ratio) __asm__("_ZN22V90ConnectionEvaluator14evaluatePhase4Ef");

#define F(p, o) (*(float *)((char *)(p) + (o)))
#define I(p, o) (*(int32_t *)((char *)(p) + (o)))
#define S(p, o) (*(int16_t *)((char *)(p) + (o)))

int hook_eval(void *self, float ratio)
{
    static FILE *f;
    char *p = *(char **)self;
    int before_n = I(self, 0x4), flag = S(self, 0xb0);
    float avepdsnr = F(self, 0x70), pdthr = F(self, 0xac);
    int ret = orig_eval(self, ratio);

    if (!f)
        f = fopen("/tmp/p4eval.txt", "a");
    if (f) {
        fprintf(f, "ratio %.3f flag(0xb0) %d retrains %d->%d count(0x74) %d avePdsnr %.3f pdsnrThr %.3f | "
                   "p410 %.3f p434 %.3f p438 %.3f p440 %.3f p45c %d p460 %d | ret %d\n",
                ratio, flag, before_n, I(self, 0x4), I(self, 0x74), avepdsnr, pdthr,
                F(p, 0x410), F(p, 0x434), F(p, 0x438), F(p, 0x440), I(p, 0x45c), I(p, 0x460), ret);
        fflush(f);
    }
    return ret;
}
