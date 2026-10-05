/* v70_g729a.c -- adapter over the ITU G.729A reference source; see v70_g729a.h. */
#include "v70_g729a.h"

#include "TYPEDEF.H"
#include "LD8A.H"

extern Word16 *new_speech;              /* the reference encoder's input buffer */
static Word16 synth_buf[L_FRAME + M];   /* decoder synthesis history, as DECODER.C's */
extern Flag Overflow, Carry;            /* the reference arithmetic's saturation flags */
extern const Word16 bitsno[];           /* bits per parameter (tab_ld8a.c) */
Word16 bad_lsf;                         /* defined by the reference DECODER.C's file; the
                                         * library functions read and set it */

void v70_g729a_init(void)
{
    int i;

    Overflow = 0;                       /* flags the programs never need to reset, a */
    Carry = 0;                          /* long-lived process does */
    bad_lsf = 0;                        /* as DECODER.C initialises it */
    for (i = 0; i < M; i++)
        synth_buf[i] = 0;
    Init_Pre_Process();
    Init_Coder_ld8a();
    Init_Decod_ld8a();
    Init_Post_Filter();
    Init_Post_Process();
}

void v70_g729a_encode(const int16_t pcm[G729A_FRAME_SAMPLES], uint8_t out[G729A_FRAME_OCTETS])
{
    Word16 prm[PRM_SIZE];
    int i, b, pos = 0;

    for (i = 0; i < L_FRAME; i++)
        new_speech[i] = pcm[i];
    Pre_Process(new_speech, L_FRAME);
    Coder_ld8a(prm);
    for (i = 0; i < G729A_FRAME_OCTETS; i++)
        out[i] = 0;
    for (i = 0; i < PRM_SIZE; i++)
        for (b = bitsno[i] - 1; b >= 0; b--, pos++)
            if ((prm[i] >> b) & 1)
                out[pos >> 3] |= (uint8_t)(0x80 >> (pos & 7));
}

void v70_g729a_decode(const uint8_t in[G729A_FRAME_OCTETS], int erased, int16_t pcm[G729A_FRAME_SAMPLES])
{
    Word16 parm[PRM_SIZE + 1], Az_dec[MP1 * 2], T2[2];
    Word16 *synth = synth_buf + M;
    int i, b, pos = 0;

    for (i = 0; i < PRM_SIZE; i++) {
        Word16 v = 0;

        for (b = 0; b < bitsno[i]; b++, pos++)
            v = (Word16)((v << 1) | ((in[pos >> 3] >> (7 - (pos & 7))) & 1));
        parm[i + 1] = v;
    }
    parm[0] = erased ? 1 : 0;
    parm[4] = Check_Parity_Pitch(parm[3], parm[4]);     /* as DECODER.C does */
    Decod_ld8a(parm, synth, Az_dec, T2);
    Post_Filter(synth, Az_dec, T2);
    Post_Process(synth, L_FRAME);
    for (i = 0; i < L_FRAME; i++)
        pcm[i] = synth[i];
}
