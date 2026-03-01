/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post_const.h
 *  Header: post-processing constants
 *------------------------------------------------------------------------
 */

#ifndef POST_CONST_H
#define POST_CONST_H
 
 
/* Definition of constants */

#define L_WIN        64           /* Length of the analysis window */
#define L_FFT        L_WIN        /* Length of the FFT */
#define L_FFT_DIV_2  32           /* Length of the FFT /2 */
#define L_FFT_M1     (L_FFT-1)    /* Length of the FFT -1 */
#define L_FLT        (Word16)33   /* Length of the time filter */

#define FRAME_LGTH_POSTPROC (Word16)40
                        /* Frame shifting (in samples) */

#define NSAMP_INTERP    (Word16)8
                        /* Number of samples for interpolation (reconstruction) */
#define NSAMP_INTERP_M1 (NSAMP_INTERP-1)
                        /* Number of samples for interpolation (reconstruction)-1 */

#define NB_BINS     (Word16)(L_FFT/2+1)
                        /* Number of frequency bins : 1+L_FFT/2 */
#define L_FLT_DIV2  (Word16)((L_FLT-1)/2)
                        /* Half length of time filter : (L_FLT-1)/2 */
#define L_BUF_CIRC  (Word16)2*L_FLT
                        /* Length of circuling buffer : 2*L_FLT */

/* W16_MIN=-6 dB */
#define W16_MIN     (Word16)16423 /* Maximum gain for the filter (6 dB) (Q15) */

#define BETA16M1    655           /* Constant beta for the computation of the
                                     a priori SNR (Decision-Directed) */

/*-- Parameters for 8-bit A-law --*/
/* -38.16 dB => (REAL)1.527566058238073e-004 = 328042 [Q31] */
#define NOISE_A_LAW_64K    (Word32)328042  /* Noise quantization level for 
                                              high level signals
                                              logarithmic part of the curve */
#define NOISE_A_LAW_64K_HI (Word16)5       /* L_Extract(x_64K, x_HI, x_LOW) */
#define NOISE_A_LAW_64K_LO (Word16)181

/* 178753578 = [FRAME_LGTH_POSTPROC/(10^(26.8174/10))] [Q31] */
#define UNIF_LOG_THRESH    (Word32)178753578  /* Uniform/log A-law threshold */


/* = [FRAME_LGTH_POSTPROC/(50 dB)] in Q31 (with FRAME_LGTH_POSTPROC = 40) */
#define DEC_ON_50DB        (Word32)858993
/* = [FRAME_LGTH_POSTPROC/(60 dB)] in Q31 (with FRAME_LGTH_POSTPROC = 40) */
#define DEC_ON_60DB        (Word32)85899

/* -56.23dB = 10*log10( (8*8*40)/(32768*32768) ) = 5120 [Q31] */
#define SILENCE_THRESH     (Word32)5120    /* Threshold of the energy of 
                                              frame above which silence is
                                              detected (corresponds to 
                                              -8 /+8 quantization steps for
                                              8-bit A-law) */


/****************************************************************************/
/**** Name of the structure : VAR_ANASYNTH                                  */
/* Parameters for analysis and synthesis                                    */
/****************************************************************************/
typedef struct
{
  Word16  X[L_FFT],    /* FFT of the current frame */
          x[L_FFT],    /* Current frame */
          buf_circ[L_BUF_CIRC],
                       /* Circuling buffer */
          x_shift,     /* Shift for scaling the samples 'x' of the current frame */
          bottom,      /* Index of the current sample in the circuling buffer  */
          hOld[L_FLT]; /* Filter of the preceding frame
                          (interpolation for reconstruction) */
} VAR_ANASYNTH;


/****************************************************************************/
/**** Name of the structure : VAR_GAIN                                      */
/* Parameters for the processing of the post process filter                 */
/****************************************************************************/
typedef struct
{
  Word16  h[L_FLT];       /* Impulse Response of the post-processing filter */
  Word32  S2_32[NB_BINS]; /* Power spectrum of the useful signal */
} VAR_GAIN;


/****************************************************************************/
/**** Name of the structure : VAR_MEM                                       */
/* Global structure including the sub-stuctures of variables                */
/****************************************************************************/
typedef struct
{
  VAR_GAIN     var_gain;     /* State structure for the gain processing */
  VAR_ANASYNTH var_anaSynth; /* State structure for the Analysis/Synthesis */
} VAR_MEM;

#endif /* POST_CONST_H */
