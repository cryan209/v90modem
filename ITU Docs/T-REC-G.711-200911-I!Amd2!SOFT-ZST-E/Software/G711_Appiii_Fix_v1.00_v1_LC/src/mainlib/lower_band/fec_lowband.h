/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: fec_lowband.h
 *  Function: Header of frame erasure concealment (FERC)
 *------------------------------------------------------------------------
 */

#ifndef  FEC_LOWBAND_H
#define  FEC_LOWBAND_H

#ifndef NO_FRAME_LOSS
#define NO_FRAME_LOSS    0
#endif
#ifndef FRAME_LOSS
#define FRAME_LOSS       1
#endif

void    *FEC_lowerband_const( Word16 ferc, Word16 pf );
void    FEC_lowerband_dest( void *ptr_work );
void    FEC_lowerband_reset( void *ptr_work );
void    FEC_lowerband( int flag, Word16 *sig, Word16 *sout, void *work);

Word16  get_lb_pit(void * pFECWork);


/**********************
 * Constants for FERC *
 **********************/

/* signal classes */
#define LBFEC_TRANSIENT          (Word16)3
#define LBFEC_UNVOICED           (Word16)1
#define LBFEC_WEAKLY_VOICED      (Word16)5
#define LBFEC_VOICED             (Word16)0

/* 4:1 decimation constants */
#define FACT                     4             /* decimation factor for pitch analysis */
#define FACTLOG2                 2             /* log2(FACT) */
#define FACT_M1                  (FACT-1)
#define FEC_L_FIR_FILTER_LTP     9             /* length of decimation filter */
#define FEC_L_FIR_FILTER_LTP_M1  (FEC_L_FIR_FILTER_LTP-1) /* length of decimation filter - 1 */
#define NOOFFSIG_LEN             (MAXPIT2+FEC_L_FIR_FILTER_LTP_M1)
#define MEMSPEECH_LEN            (MAXPIT2P1+ORD_LPC)
#define MEMSPEECH_LEN_MFRAME     (MEMSPEECH_LEN-L_FRAME_NB)

/* open-loop pitch parameters */
#define MAXPIT                   144           /* maximal pitch lag (18ms @8kHz) => 55 Hz */
#define MAXPIT2                  (2*MAXPIT)
#define MAXPIT2P1                (MAXPIT2+1)   /*length of mem_exc*/
#define MAXPIT_S2                (MAXPIT/2)
#define MAXPIT_DS                (MAXPIT/FACT) /* maximal pitch lag (18ms @2kHz) => 55 Hz */
#define MAXPIT_DSP1              (MAXPIT_DS+1)
#define MAXPIT2_DS               (MAXPIT2/FACT)
#define MAXPIT2_DSM1             (MAXPIT2_DS-1)
#define MAXPIT_S2_DS             (MAXPIT_S2/FACT)
#define MINPIT                   16            /* minimal pitch lag (2ms @8kHz) => 500 Hz */
#define MINPIT_DS                (MINPIT/FACT) /* minimal pitch lag (2ms @2kHz) => 500 Hz */
#define GAMMA                    30802         /* 0.94 in Q15 */
#define GAMMA2                   28954         /* 0.94^2 */
#define GAMMA_AZ1                32440         /* 0.99 in Q15 */
#define GAMMA_AZ2                32116         /* 0.99^2 in Q15 */
#define GAMMA_AZ3                31795         /* 0.99^3 in Q15 */
#define GAMMA_AZ4                31477         /* 0.99^4 in Q15 */
#define GAMMA_AZ5                31162         /* 0.99^5 in Q15 */
#define GAMMA_AZ6                30850         /* 0.99^6 in Q15 */
#define T0_SAVEPLUS              2             /*number of extra saved samplesfor a pitch period*/

/* LPC windowing */
#define ORD_LPC                  6             /* LPC order */
#define HAMWINDLEN               80            /* length of the assymetrical hamming window */

/* cross-fading parameters */
#define CROSSFADELEN             L_FRAME_NB    /* length of crossfade (5 ms @8kHz) */

/* adaptive muting parameters , curve of 3 slopes, during 60 ms*/
#define END_1ST_PART             80            /* end of first slope: 10ms @ 8kHz */
#define END_2ND_PART             160           /* end of middle slope: 20ms @ 8kHz */ 
#define END_3RD_PART             480           /* end of last slope: 60ms @ 8kHz */ 
/* for all classes except TRANSIENT calss */ 
#define FACT1_V                  10                 /*attenuation, first slope*/
#define FACT2_V                  20                 /*attenuation, middle slope*/
#define FACT3_V                  95  /*30367/320*/  /*attenuation, last slope, to get 0 at the end*/
#define FACT2P_V                 (FACT2_V-FACT1_V)  /*additional attenuation between the first two slopes*/
#define FACT3P_V                 (FACT3_V-FACT2_V)  /*additional attenuation between the last two slopes*/
/* for  TRANSIENT calss, same slope, 15 ms, as time increment is set to 4*/ 
#define FACT1_V_R                273 /*32768/120*/  /*attenuation, first slope*/
#define FACT2P_V_R               0              /*additional attenuation between the first two slopes*/
#define FACT3P_V_R               0              /*additional attenuation between the last two slopes*/


/**********************
 * Tables for FERC    *
 **********************/

extern const Word16    LBFEC_lag_h[16];
extern const Word16    LBFEC_lag_l[16];
extern const Word16    LBFEC_lpc_win_80[80];
extern const Word16    LBFEC_fir_lp[FEC_L_FIR_FILTER_LTP];

#endif  /* FEC_LOWBAND_H */
