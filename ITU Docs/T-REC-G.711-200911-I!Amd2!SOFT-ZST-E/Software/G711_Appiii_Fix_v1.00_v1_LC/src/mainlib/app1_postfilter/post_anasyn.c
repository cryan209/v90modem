/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post_anasyn.c
 *  Function: Analysis/synthesis subroutines for post-processing
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "post_const.h"
#include "post_anasyn.h"
#include "post_rfft.h"


/**
* Windowing of a signal of length L_WIN with the analysis window
*/
static void postProc_WindowingAna(
  Word16  *DataIn16,  /* IN:  Input signal */
  Word16  *DataOut16, /* OUT: Output windowed signal */
  Word16  shiftSig    /* IN:  Shift for a full scale representation of signal */
)
{
  Word16  i;

  FOR (i=0 ; i<L_WIN ; i++)
  {
    DataOut16[i] = mult(shl(DataIn16[i],shiftSig),Hann_sh16[i]);
    move16();
  }

  return;
}


/**
* Initialization of the state structure VAR_ANASYNTH
*/
void postProc_Init_anaSynth(
  VAR_ANASYNTH *var_anaSynth  /* OUT: VAR_ANASYNTH state structure */
)
{
  zero16(L_FFT, var_anaSynth->X);
  zero16(L_WIN, var_anaSynth->x);
  zero16(L_BUF_CIRC, var_anaSynth->buf_circ);
  zero16(L_FLT, var_anaSynth->hOld);

  var_anaSynth->bottom  = 0;  move16();
  var_anaSynth->x_shift = 0;  move16();
}


/**
* Analysis of the current frame :
*  1. Store new samples bloc_in in var_anaSynth->x
*  2. Window the frame
*  3. Compute the FFT (of the windowed frame), saved in var_anaSynth->X
*/
void postProc_Analysis(
  const Word16 *bloc_in,      /* IN:  Block of new samples */
  VAR_ANASYNTH *var_anaSynth  /* I/O: State structure of analysis/synthesis */
)
{
  Word16  p,q;
  Word16  maxi;

  Word16  *x, *x_shift;
  Word16  *X;

  Word16  xw[L_WIN];
  
  /* Loading of the state structure */
  x       = var_anaSynth->x;           move16();
  x_shift = &(var_anaSynth->x_shift);  move16();
  X       = var_anaSynth->X;           move16();

  /* Beginning */
  p = L_WIN-FRAME_LGTH_POSTPROC;  move16();

  FOR (q=0; q<FRAME_LGTH_POSTPROC; q++)
  {
    x[p++] = bloc_in[q];
    move16();
  }

  /* Scaling of data of x */
  /* Shift value */
  maxi = 1;      move16();
  *x_shift = 0;  move16();

  FOR (p=0;p<L_WIN;p++)
  {
    maxi = s_or(maxi, abs_s(x[p]));
  }
  WHILE ( s_and(maxi, 0xC000u) == 0)
  {
    *x_shift = add(*x_shift,1);  move16();
    maxi = shl(maxi,1);
  }

  /* Scaling of data + windowing */
  postProc_WindowingAna(x,xw,*x_shift);

  /* FFT */

  rfft16_64(xw, X);

  return;
}


/**
* Reconstruction of the useful ("denoised") signal :
* 1. Filtering of the FRAME_LGTH_POSTPROC new samples with the filter h
* 2. Reconstruction of the useful signal with a time OLS (with interpolation)
*/
void postProc_Synthesis(
  Word16 *bloc_out,  /* OUT: Block of filtered samples */
  const Word16 *h,   /* IN:  Impulse response of the filter of the current frame */
  VAR_ANASYNTH *var_anaSynth
                     /* IN:  State structure including the samples to filter */
)
{
  Word16  top,temp;
  Word16  p,q;
  
  Word16  *buf_circ,*bottom, *x, *hOld;

  /* Loading of the state structure*/
  buf_circ = var_anaSynth->buf_circ;  move16();
  hOld = var_anaSynth->hOld;          move16();
  bottom = &(var_anaSynth->bottom);   move16();
  x = var_anaSynth->x;                move16();

  /*******************************************************/
  /* Filtering of the new samples with a circular buffer */
  /*******************************************************/
  q = L_WIN-FRAME_LGTH_POSTPROC;  move16();

  /* Loop over the samples */
  FOR (p=0; p<NSAMP_INTERP_M1; p++)
  {
    temp = x[q];  move16();
    if (sub(*bottom, L_FLT) == 0)
    {
      *bottom = 0;
      move16();
    }
    top = add(*bottom, L_FLT);
    buf_circ[top] = temp;      move16();
    buf_circ[*bottom] = temp;  move16();

    (*bottom) = add(*bottom, 1);
    q = add(q, 1);

    bloc_out[p] = postProc_Filter16(h, &(buf_circ[top]), L_FLT);
    move16();

    /* Interpolation on the first samples */
    temp = postProc_Filter16(hOld, &(buf_circ[top]), L_FLT);
    bloc_out[p] = add(mult(temp,Hann_sh16_p6m1[p]),
                      mult(bloc_out[p],Hann_sh16_p6[p])
                     );
    move16();

  }   /* end of loop over the samples */

  /* Loop over the samples */
  FOR (p=NSAMP_INTERP_M1; p<FRAME_LGTH_POSTPROC; p++)
  {
    temp = x[q];  move16();
    if (sub(*bottom, L_FLT) == 0)
    {
      *bottom = 0;
      move16();
    }
    top = add(*bottom, L_FLT);
    buf_circ[top] = temp;      move16();
    buf_circ[*bottom] = temp;  move16();

    (*bottom) = add(*bottom, 1);
    q = add(q, 1);

    bloc_out[p] = postProc_Filter16(h, &(buf_circ[top]), L_FLT);
    move16();

  }   /* end of loop over the samples */


  /* Shift the frame for the next block */
  FOR (p=0; p<L_WIN-FRAME_LGTH_POSTPROC; p++)
  {
    x[p] = x[p+FRAME_LGTH_POSTPROC]; 
    move16();
  }
  
  /* Refresh the coefficients of the filter */
  FOR (p=0 ; p<L_FLT ; p++)
  {
    hOld[p] = h[p]; 
    move16();
  }
  return;
}


/**
* Convolve the signal dataIn with the impulse response filter
*  out = sum_k { filter(k) * dataIn(-k) }
*/
Word16 postProc_Filter16(
  const Word16 *filter, /* IN: Impulse Response of the filter */
  const Word16 *dataIn, /* IN: Data to filter : points on the most recent data */
  Word16 filt_length    /* IN: Length of the filter (i) */
)
{
  Word32  accu;
  Word16  k;

  accu = (Word32)0;  move32();

  FOR (k=0; k<filt_length; k++)
  {
    accu = L_mac(accu,dataIn[-k],filter[k]); 
  }
  return round(accu);
}
