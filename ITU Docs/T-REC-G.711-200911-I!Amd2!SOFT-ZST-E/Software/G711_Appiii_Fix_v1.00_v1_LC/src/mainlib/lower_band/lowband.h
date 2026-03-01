/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/

/*
 *------------------------------------------------------------------------
 *  File: lowband.h
 *  Function: Header of lower-band encoder and decoder
 *------------------------------------------------------------------------
 */

#ifndef LOWBAND_H
#define LOWBAND_H


#define  ALAW_DEADZONE   11  /* Quantizer dead zone around zero (to minimize crackling) */
#define  MULAW_DEADZONE  7   /* Quantizer dead zone around zero (to minimize crackling) */
#define  ALAW_OFFSET     8   /* A-law offset to enable a zero output (0 = disable) */

#define  G711ULAW        1
#define  G711ALAW        2

/* Noise shaping parameters */

#define L_WINDOW         80     /* length of the LP analysis */
#define ORD_M            4      /* LP order (and # of "lags" in autocorr.c)  */
#define GAMMA1           30147  /* 0.92f in Q15 */
#define MAX_NORM         16     /* when to begin noise shaping deactivation  */
                                /* use MAX_NORM = 32 to disable this feature */

void    *lowband_encode_const (int, int);
void    lowband_encode_dest (void*);
void    lowband_encode_reset (void*);
void    lowband_encode (const Word16*, unsigned char*, void*);
void    *lowband_decode_const (int, int, int);
void    lowband_decode_dest (void*);
void    lowband_decode_reset (void*);
void    lowband_decode (const unsigned char*, int, Word16*, Word16*, void*);

void    lbe_bitalloc( Word16* expi, Word16* bit_alloc );

Word16  AutocorrNS(Word16 x[], Word16 r[], Word16 r_l[]);


/* Tables used in AutocorrNS() */
extern Word16 NS_window[L_WINDOW];
extern const Word16 NS_lag_h[ORD_M];
extern const Word16 NS_lag_l[ORD_M];

#endif
