/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post_gainfct.c
 *  Function: Subroutines for estimating of the post-processing filter
 *------------------------------------------------------------------------
 */

#include "stl.h"
#include "oper_32b.h"
#include "post_const.h"
#include "post_rfft.h"
#include "post_gainfct.h"

static void postProc_FFTtoPSD_F32(Word16 *fftIn, Word16 shiftIn, Word32 *specOut);
static void postProc_SpecFilter32(Word32 *specIn, Word16 *W, Word32 *specOut);


/**
* Initialisation of variables in the structure 
*/
void postProc_InitGainProcess(
  VAR_GAIN *var_gain  /* I/O: VAR_GAIN state structure */
)
{
  Word16  i;

  /* Initilisation of table */
  FOR (i = 0; i < NB_BINS; i++)
  {
    var_gain->S2_32[i] = (Word32) 0;
    move32();
  }
}


/**
* Computation of a post-processing time filter (casual) that reduces the 
* quantization noise. The time filter is store in var_gain->h
*/
void postProc_GainProcess(
  Word16       *X,            /* IN:  FFT of "noised" signal */
  Word16       X_shift,       /* IN:  Global shift of FFT data */
  VAR_GAIN     *var_gain,     /* I/O: VAR_GAIN state structure */
  VAR_ANASYNTH *var_anasynth  /* I/O: VAR_ANASYNTH state structure */
)
{
  Word16  W[L_FFT];
  Word16  p, q, r;

  Word32  energyFrame32,      /* Energy of the current frame */
          PSD_N32[NB_BINS],   /* Power Spectral Density of the noise */
          X2_32  [NB_BINS];   /* PSD of the "noised" signal */
  Word32  SNRpost32,          /* a posteriori Signal to Noise Ratio */
          SNRprio32,          /* a priori Signal to Noise Ratio */
          PSD_N_tmp32;        /* temporary PSD variable */

  /* Variables loaded in the structure */
  Word16  *h;
  Word32  *S2_32;

  h = var_gain->h;          move16();
  S2_32 = var_gain->S2_32;  move32();

  /*-- Power Spectral Density of quantized ("noised") signal --*/
  /* X2 : Q[30+2*(X_shift-4)]
          -> X_shift = global shift done per frame before FFT computation
          -> 4 = 3+1 (3 = shift of the FFT, 1 = adding shift) */
  postProc_FFTtoPSD_F32(X, X_shift, X2_32);

  /*-- Energy of the current frame --*/
  energyFrame32 = (Word32)0;  move32();

  /* The earliest samples are at the end of the window */
  FOR (p=L_WIN-FRAME_LGTH_POSTPROC; p<L_WIN; p++)
  {
    /* energyFrame32 += (var_anasynth->x[p])*(var_anasynth->x[p]); */
    /* saturation may occur for very high level signals
        --> where Signal to Quantization Noise Ratio (SQNR) is flat (38 dB)
        --> consequently, it aims to under estimate the quantization noise */
    /* L_mult (Q15,Q15) => Q31 */
    energyFrame32 = L_mac(energyFrame32,
                          var_anasynth->x[p],
                          var_anasynth->x[p]);
  }

  /* Load factor = sqrt(x_sat/sigma_x) = sqrt(1/sigma_x^2)
     an estimation of sigma_x = energyFrame32/FRAME_LGTH_POSTPROC */

  /* Load factor = sqrt(FRAME_LGTH_POSTPROC/energyFrame32)
     <=> loadFactor^2 = FRAME_LGTH_POSTPROC/energyFrame32 */

  /*-- Evaluation of the power of the quantization noise --*/
  
  IF ( L_sub(energyFrame32,UNIF_LOG_THRESH) >0 )
  {
    /* Logarithmic part of the curve: SQNR = contant */
    Word16  hi,lo;
    
    L_Extract(energyFrame32,&hi,&lo);
    /* PSD_N32[0] = energyFrame32 * NOISE_A_LAW_64K;  [Q31] */
    PSD_N32[0] = Mpy_32(hi,lo,NOISE_A_LAW_64K_HI,NOISE_A_LAW_64K_LO);
  }
  ELSE
  {
    /* Uniform part of the curve --> PSD_N32[] constant 
       (SQNR decreases with the level of the signal) */
    /* Load factor threshold = 50 dB */
    IF ( L_sub(energyFrame32,DEC_ON_50DB) <0 )
    {
      /* 67909396 = 0.03162277660168 = -15 dB [Q31] */
      /* L_Extract(67909396,&hi,&lo) --> hi = 1036 et lo = 7050 */
      Word16  hi,lo;
    
      L_Extract(energyFrame32,&hi,&lo);
      /* PSD_N32[0] = energyFrame32 * (-15dB);  [Q31] */
      PSD_N32[0] = Mpy_32(hi,lo,(Word16)1036,(Word16)7050);
    }
    ELSE
    {
      /* PSD_N32 = [ 1/((A/1+ln(A))*3*(2^(2*8))) ]
         --> (1/3145728) [Q31] = 683 */
      PSD_N32[0] = (Word32)683;
    }
  }
  /*-- End of Evaluation of the power of the quantization noise --*/
  move32();

  /*-- Evaluation of a priori SNR --*/

  /*---------------------------------------------------*/
  /* Filter computation in frequency domain : 1st step */
  /*---------------------------------------------------*/

  /* PSD_N_tmp32 same dynamic as X2_32 */
  PSD_N_tmp32 = L_shl(PSD_N32[0],sub(shl(X_shift,(Word16)1),(Word16)9));

  /* Loop over frequency channels */
  FOR (p=0;p<NB_BINS;p++)
  {
    /* PSD_N32 in Q31 and X2_32 in Q[30+2*(X_shift-4)] */
    
    Word32  difh32,tmp32;
    Word16  hi,lo;
    Word16  num,denom,decTmp;

    /*---------------------------------*/
    /* Computation of a posteriori SNR */
    /* SNRpost = X2[p] - PSD_N[0];   */
    /*---------------------------------*/
    /* SNRpost32 same dynamic as X2_32 */
    SNRpost32 = L_sub(X2_32[p],PSD_N_tmp32);

    /* Half wave rectification */
    IF (SNRpost32 < (Word32)0)
    {
      /* No division by 0 as PSD_N>0
         Furthermore, adding EPS --> problem of dynamic because
         SNRpost32 is in Q[30+2*(X_shift-4)] */
      SNRpost32 = (Word32)0;
      move32();
    }

    /*---------------------------------------------*/
    /* A priori SNR computation          */
    /* SNRprio = beta*P_s_est[p] + betam1*SNRpost; */
    /*---------------------------------------------*/
    difh32 = L_sub(S2_32[p],SNRpost32);
    L_Extract(difh32,&hi,&lo);
    /* SNRprio32 same dynamic as X2_32 (if S2_32 idem X2_32) */
    SNRprio32 = L_sub(S2_32[p],Mpy_32_16(hi,lo,BETA16M1));

    /*--------------------------------------------*/
    /* Computation of filterW in frequency domain */
    /* W[p] = SNRprio/(SNRprio + PSD_N[0]);     */
    /*--------------------------------------------*/
    tmp32   = L_add(SNRprio32,PSD_N_tmp32);
    decTmp  = norm_l(tmp32);
    /* apply same shift to numerator and denominator */
    denom   = extract_h(L_shl(tmp32,decTmp));
    num   = extract_h(L_shl(SNRprio32,decTmp));
    
    /* result in Q15 */
    W[p] = (Word16) 32767;  move16();
    if (denom > 0)
    {
      W[p] = div_s(num,denom);
      move16();
    }
  }

  /*------------------------------------------------------*/
  /* Computation of filter in frequency domain : 2nd step */
  /*------------------------------------------------------*/

  /* Loop over frequency channels */
  FOR (p=0;p<NB_BINS;p++)
  {
    Word16  hi1,hi2,lo1,lo2;
    Word16  decTmp,num,denom;
    Word32  tmp32;


    /* SNRprio32 in Q31 */
    SNRprio32 = L_mult(W[p],W[p]);
    L_Extract(SNRprio32,&hi1,&lo1);
    L_Extract(X2_32[p],&hi2,&lo2);
    SNRprio32 = Mpy_32(hi1,lo1,hi2,lo2);

    /*-----------------------------------------------------*/
    /* W[p] = Max( SNRprio/(SNRprio + PSD_N[0]) , W_MIN ); */
    /*-----------------------------------------------------*/
    tmp32  = L_add(SNRprio32,PSD_N_tmp32);
    decTmp = norm_l(tmp32);
    /* apply same shift to numerator and denominator */
    denom  = extract_h(L_shl(tmp32,decTmp));
    num    = extract_h(L_shl(SNRprio32,decTmp));

    if (denom > 0)
    {
      decTmp = div_s(num,denom);
    }
    W[p] = s_max(decTmp, W16_MIN);  move16();
  }

  /*- If silent frame, a 20 dB attenuation is applied -*/
  IF ( L_sub(energyFrame32,SILENCE_THRESH)<=0 )
  {
    FOR (p=0; p<NB_BINS; p++)
    {
      /* W[p] = W[p]*(REAL)0.25; */
      W[p] = shr(W[p],(Word16)2);
      move16();
    }
  }
  /*------------------------------------------------------------------------*/


  /*******************************************************************/
  /* Computation of an estimation of the PSD of the "cleaned" signal */
  /*******************************************************************/
  postProc_SpecFilter32(X2_32,W,S2_32);


  /************************************************/
  /* Computation of the filter in the time domain */
  /************************************************/

  /* iFFT */
  rsifft16_64(W);

  /* Windowing + constraint on the symmetry of the filter */
  q = 0;           move16();
  r = L_FLT_DIV2;  move16();

  FOR (p=L_FLT_DIV2; p>=0 ; p--)
  {
    h[p] = mult(W[q],WinFilt[p]);  move16();
    h[r] = h[p];                   move16();
    q++;
    r++;
  }
  return;
}


/**
* Squared module of a complex frequency table
* [Re(0) Re(1) ... Re(N/2) Re(N/2+1) Im(N/2) ... Im(1)]
*/
static void postProc_FFTtoPSD_F32(
  Word16  *fftIn,    /* IN:  Complex FFT of length L_FFT */
  Word16  shiftIn,   /* IN:  Global shift of FFT complex data */
  Word32  *specOut   /* OUT: Power spectrum (length NB_BINS=L_FFT/2+1) */
)
{
  Word16  i,j;
  Word32  w32tmp;
  
  /* bin 0 and Fe/2 (index 0 and L_FFT_2) are real */

  /* specOut in Q[30+2*(X_shift-4)] */
  specOut[0]           = L_mult0(fftIn[0],fftIn[0]);
  specOut[L_FFT_DIV_2] = L_mult0(fftIn[L_FFT_DIV_2],fftIn[L_FFT_DIV_2]);

  /* other bins */
  j = L_FFT_M1;  move16();
  FOR (i=1 ; i<L_FFT_DIV_2 ; i++)
  {
    /* specOut[i] = Re^2 + Im^2 = fftIn[i]*fftIn[i] + fftIn[j]*fftIn[j] */

    /* Re^2 in Q[30+2*(X_shift-4)] */
    w32tmp = L_mult0(fftIn[i],fftIn[i]);
    /* ||^2 = Re^2 + Im^2 in Q[30+2*(X_shift-4)] */
    specOut[i] = L_mac0(w32tmp,fftIn[j],fftIn[j]);  move32();

    j--;
  }
}


/**
* Filtering of the power spectrum by the transfert function W
*/
static void postProc_SpecFilter32(
  Word32  *specIn,   /* IN:  Power spectrum to filter */
  Word16  *W,        /* IN:  Transfert function (real) of the filter */
  Word32  *specOut   /* OUT: Power spectrum fitered */
)
{
  Word16  i;
  Word32  W2_32;

  /* bin 0 and Fe/2 (index 0 and L_FFT_2) are real */

  /* specOut = |W|^2 * specIn (specIn is already a power spectrum */
  FOR (i=0 ; i<NB_BINS ; i++)
  {
    Word16 hi1,lo1,hi2,lo2;
    
    /* W2_32 = W*W */
    W2_32 = L_mult(W[i],W[i]);    /* W2_32 in Q[31], W en Q[15] */

    /* specOut[i] = W*W*specIn[i] */
    /* specIn and specOut in Q[30+2*(X_shift-4)] */
    L_Extract(W2_32,&hi1,&lo1);
    L_Extract(specIn[i],&hi2,&lo2);
    specOut[i] = Mpy_32(hi1,lo1,hi2,lo2);  move32();
  }
}
