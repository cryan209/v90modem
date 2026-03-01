/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post_anasyn.h
 *  header: Header of analysis/synthesis for post-processing
 *------------------------------------------------------------------------
 */

#ifndef POST_ANASYN_H
#define POST_ANASYN_H

#include "post_const.h"
 
extern const Word16 Hann_sh16[L_WIN];
extern const Word16 Hann_sh16_p6[7];
extern const Word16 Hann_sh16_p6m1[7];


void   postProc_Init_anaSynth(VAR_ANASYNTH *var_anaSynth);
void   postProc_Analysis(const Word16 *bloc_in, VAR_ANASYNTH *var_anaSynth);
void   postProc_Synthesis(Word16 *bloc_out, const Word16 *filtre,
                          VAR_ANASYNTH *var_anaSynth);
Word16 postProc_Filter16(const Word16 *filter, const Word16 *dataIn,
                         Word16 filt_length);

#endif /* POST_ANASYN_H */

