/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post_rfft.h
 *  header: real fft and ifft prototypes
 *------------------------------------------------------------------------
 */

#ifndef POST_RFFT_H
#define POST_RFFT_H
 
extern const Word16 r[64];
extern const Word16 w[16];
 
void rfft16_64(Word16 *xw, Word16 *X);
void rsifft16_64(Word16 *data);

#endif /* POST_RFFT_H */
