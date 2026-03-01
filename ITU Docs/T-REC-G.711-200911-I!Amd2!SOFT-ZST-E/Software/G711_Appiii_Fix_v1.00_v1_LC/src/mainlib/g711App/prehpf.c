/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: prehpf.c
 *  Function: Pre-processing 1-tap high-pass filtering
 *            Cut-off (-3dB) frequency is approximately 50 Hz,
 *            if the recommended filt_no value is used.
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "prehpf.h"

typedef struct {
  Word16  memx;  /*Q0*/
  Word32  memy;  /*Q14*/
} HPASSMEM;

/* Constructor */
void  *highpass_1tap_iir_const(void)  /* returns pointer to work space */
{
  HPASSMEM *hpmem;

  hpmem = (HPASSMEM *)malloc( sizeof(HPASSMEM) );
  if ( hpmem != NULL )
    highpass_1tap_iir_reset( (void *)hpmem );
  return (void *)hpmem;
}

/* Destructor */
void  highpass_1tap_iir_dest(void *ptr)
{
  HPASSMEM *hpmem = (HPASSMEM *)ptr;
  if (hpmem != NULL ) free( hpmem );
}

/* Reset */
void  highpass_1tap_iir_reset(void *ptr)
{
  HPASSMEM *hpmem = (HPASSMEM *)ptr;

  if (hpmem != NULL) {
    hpmem->memx = 0;
    hpmem->memy = 0L;
  }
}

/* Filering */
void  highpass_1tap_iir(
  Word16  filt_no,  /* (i):   Filter cutoff specification.                 */
                    /*        Use 5 for 8-kHz input                        */
  Word16  n,        /* (i):   Number of samples                            */
  Word16  sigin[],  /* (i):   Input signal (Q0)  */
  Word16  sigout[], /* (i):   Output signal (Q0) */
  void    *ptr      /* (i/o): Work space       */
) {
  int      k;
  Word16   sSigpre;  /*Q0*/
  Word32   lAcc;     /*Q14*/
  HPASSMEM *hpmem = (HPASSMEM *)ptr;

  lAcc = hpmem->memy;     move32();
  sSigpre = hpmem->memx;  move16();

  FOR ( k = 0; k < n; k++ )
  {
    /* y[k] = a * y[k-1] + x[k] - x[k-1] */
    lAcc = L_sub( lAcc, L_shr(lAcc, filt_no) );   /* a = 0.96875  for filt_no=5 */
    lAcc = L_mac( lAcc, 0x2000, *sigin );   /* Q14 */
    lAcc = L_msu( lAcc, 0x2000, sSigpre );  /* Q14 */
    sSigpre  = *sigin++;  move16();
    *sigout++ = round( L_shl(lAcc, 2) );
  }
  hpmem->memx = sSigpre; move16();
  hpmem->memy = lAcc;    move32();
}
