/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post_rfft.c
 *  Function: 64-point real fft and ifft
 *------------------------------------------------------------------------
 */

#include "stl.h"
#include "post_rfft.h"

/**
* 16-bit FFT of a real signal (Split-radix - time decimation).
* Output spectrum is saved among the following sequency :
* [ Re(0) Re(1) ... Re(N/2) Re(N/2+1) Im(N/2) ... Im(1) ]
*/
void  rfft16_64(
  Word16  *xw,  /* I/O: Signal to analyse with FFT */
  Word16  *X    /* OUT: Coefficients of the FFT */
)
{
  short  i, k;
  short  Xi[64];
  short  xwi[64];
  short  tmpr, tmpi;
  short  i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12;

  FOR (i = 0; i < 64; i++) 
  {
    X[i] = shr(xw[r[i]], 1);
    move16();
  }

  FOR (i = 0; i < 64; i+=2) 
  {
    i1 = add(i,1);

    xw[i]  = shr(add(X[i], X[i1]),1);    move16();
    xw[i1] = shr(sub(X[i], X[i1]),1);    move16();
  }

  FOR (i = 0; i < 64; i+=4) 
  {
    i1 = add(i,1);
    i2 = add(i,2);
    i3 = add(i,3);

    X[i]   = shr(add(xw[i], xw[i2]),1);  move16();
    X[i2]  = shr(sub(xw[i], xw[i2]),1);  move16();
    X[i1]  = shr(xw[i1], 1);             move16();
    Xi[i1] = shr(negate(xw[i3]),1);      move16();
  }

  FOR (i = 0; i < 64; i+=8) 
  {
    i1 = add(i,1);
    i2 = add(i,2);
    i3 = add(i,3);
    i4 = add(i,4);
    i5 = add(i,5);
    i6 = add(i,6);

    xw[i]   = shr(add(X[i], X[i4]), 1);  move16();
    xw[i4]  = shr(sub(X[i], X[i4]), 1);  move16();
    xw[i2]  = shr(X[i2], 1);             move16();
    xwi[i2] = shr(negate(X[i6]), 1);     move16();

    tmpr = mult_r(w[8], add(X[i5], Xi[i5]));
    tmpi = mult_r(w[8], sub(Xi[i5], X[i5]));
    xw[i1]  = shr(add(X[i1], tmpr), 1);  move16();
    xwi[i1] = shr(add(Xi[i1], tmpi), 1); move16();
    xw[i3]  = shr(sub(X[i1], tmpr), 1);  move16();
    xwi[i3] = shr(sub(tmpi, Xi[i1]), 1); move16();
  }

  FOR (i = 0; i < 64; i+=16) 
  {
    i1 = add(i,1);
    i2 = add(i,2);
    i3 = add(i,3);
    i4 = add(i,4);
    i5 = add(i,5);
    i6 = add(i,6);
    i7 = add(i,7);
    i8 = add(i,8);
    i9 = add(i,9);
    i10 = add(i,10);
    i11 = add(i,11);
    i12 = add(i,12);

    X[i]   = add(xw[i], xw[i8]);
    X[i8]  = sub(xw[i], xw[i8]);
    X[i4]  = xw[i4];
    Xi[i4] = negate(xw[i12]);

    tmpr = mult_r(w[8], add(xw[i10], xwi[i10]));
    tmpi = mult_r(w[8], sub(xwi[i10], xw[i10]));
    X[i2]  = add(xw[i2], tmpr);
    Xi[i2] = add(xwi[i2], tmpi);
    X[i6]  = sub(xw[i2], tmpr);
    Xi[i6] = sub(tmpi, xwi[i2]);

    tmpr = add(mult_r(w[12], xw[i9]), mult_r(w[4],xwi[i9]));
    tmpi = sub(mult_r(w[12], xwi[i9]), mult_r(w[4],xw[i9]));
    X[i1]  = add(xw[i1], tmpr);
    Xi[i1] = add(xwi[i1], tmpi);
    X[i7]  = sub(xw[i1], tmpr);
    Xi[i7] = sub(tmpi, xwi[i1]);

    tmpr = add(mult_r(w[4], xw[i11]), mult_r(w[12],xwi[i11]));
    tmpi = sub(mult_r(w[4], xwi[i11]), mult_r(w[12],xw[i11]));
    X[i3]  = add(xw[i3], tmpr);
    Xi[i3] = add(xwi[i3], tmpi);
    X[i5]  = sub(xw[i3], tmpr);
    Xi[i5] = sub(tmpi, xwi[i3]);

    move16();  move16();  move16();  move16();
    move16();  move16();  move16();  move16();
    move16();  move16();  move16();  move16();
    move16();  move16();  move16();  move16();
  }

  FOR (i = 0; i < 64; i+=32) 
  {
    i4 = add(i,16);
    i2 = add(i,8);
    i3 = add(i,24);

    xw[i]   = add(X[i], X[i4]);  move16();
    xw[i4]  = sub(X[i], X[i4]);  move16();
    xw[i2]  = X[i2];             move16();
    xwi[i2] = negate(X[i3]);     move16();

    FOR (k = 1; k < 8; k++)
    {
      i1 = add(i, k);
      i2 = add(i1, 16);
      i3 = sub(i4, k);
      i5 = add(k,k);
      i6 = sub(16, i5);

      tmpr = add(mult_r(w[i6], X[i2]), mult_r(w[i5],Xi[i2]));
      tmpi = sub(mult_r(w[i6], Xi[i2]), mult_r(w[i5],X[i2]));
      xw[i1]  = add(X[i1], tmpr);   move16();
      xwi[i1] = add(Xi[i1], tmpi);  move16();
      xw[i3]  = sub(X[i1], tmpr);   move16();
      xwi[i3] = sub(tmpi, Xi[i1]);  move16();
    }
  }

  X[0]   = add(xw[0], xw[32]);   move16();
  X[32]  = sub(xw[0], xw[32]);   move16();
  X[16]  = xw[16];               move16();
  Xi[16] = negate(xw[48]);       move16();
  
  FOR (k = 1; k < 16; k++)
  {
    i2 = add(k, 32);
    i3 = sub(32, k);
    i6 = sub(16, k);
    
    tmpr = add(mult_r(w[i6], xw[i2]), mult_r(w[k],xwi[i2]));
    tmpi = sub(mult_r(w[i6], xwi[i2]), mult_r(w[k],xw[i2]));
    X[k]   = add(xw[k], tmpr);   move16();
    Xi[k]  = add(xwi[k], tmpi);  move16();
    X[i3]  = sub(xw[k], tmpr);   move16();
    Xi[i3] = sub(tmpi, xwi[k]);  move16();
  }

  FOR (i = 33; i < 64; i++)
  {
    X[i] = Xi[64-i];
    move16(); 
  }
}


/**
* 16-bit IFFT of an hermitia, spectrum (Split-radix - time decimation).
* The input spectrum should be given among the following sequency :
* [ Re(0) Re(1) ... Re(N/2) Re(N/2+1) Im(N/2) ... Im(1) ] 
*/
/*spectrum real and symmetrical*/
void rsifft16_64(Word16 *W) /**< I/O: FFT [0..32] only real -> sig[0..31] */
{
  short  i, k;
  short  X[64];
  short  xw[64];
  short  Xi[64];
  short  xwi[64];
  short  tmpr, tmpi;
  short  i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12;

  FOR (i = 0; i < 33; i++) 
  {
    xw[i] = W[i];
  }
  FOR (i = 33; i < 64; i++) 
  {
    xw[i] = W[64-i];
  }

  FOR (i = 0; i < 64; i++) 
  {
    X[i] = shr(xw[r[i]],1);
    move16();
  }

  FOR (i = 0; i < 64; i+=2) 
  {
    i1 = add(i,1);

    xw[i] = shr(add(X[i], X[i1]),1);     move16();
    xw[i1] = shr(sub(X[i], X[i1]),1);    move16();
  }

  FOR (i = 0; i < 64; i+=4) 
  {
    i1 = add(i,1);
    i2 = add(i,2);
    i3 = add(i,3);

    X[i]   = shr(add(xw[i], xw[i2]),1);  move16();
    X[i2]  = shr(sub(xw[i], xw[i2]),1);  move16();
    X[i1]  = shr(xw[i1], 1);             move16();
    Xi[i1] = shr(xw[i3],1);              move16();
  }

  FOR (i = 0; i < 64; i+=8) 
  {
    i1 = add(i,1);
    i2 = add(i,2);
    i3 = add(i,3);
    i4 = add(i,4);
    i5 = add(i,5);
    i6 = add(i,6);

    xw[i]   = shr(add(X[i], X[i4]), 1);  move16();
    xw[i4]  = shr(sub(X[i], X[i4]), 1);  move16();
    xw[i2]  = shr(X[i2], 1);             move16();
    xwi[i2] = shr(X[i6], 1);             move16();

    tmpr = mult_r(w[8], sub(X[i5], Xi[i5]));
    tmpi = mult_r(w[8], add(Xi[i5], X[i5]));
    xw[i1]  = shr(add(X[i1], tmpr), 1);  move16();
    xwi[i1] = shr(add(Xi[i1], tmpi), 1); move16();
    xw[i3]  = shr(sub(X[i1], tmpr), 1);  move16();
    xwi[i3] = shr(sub(tmpi, Xi[i1]), 1); move16();
  }

  FOR (i = 0; i < 64; i+=16) 
  {
    i1 = add(i,1);
    i2 = add(i,2);
    i3 = add(i,3);
    i4 = add(i,4);
    i5 = add(i,5);
    i6 = add(i,6);
    i7 = add(i,7);
    i8 = add(i,8);
    i9 = add(i,9);
    i10 = add(i,10);
    i11 = add(i,11);
    i12 = add(i,12);

    X[i]   = shr(add(xw[i], xw[i8]), 1);
    X[i8]  = shr(sub(xw[i], xw[i8]), 1);
    X[i4]  = shr(xw[i4], 1);
    Xi[i4] = shr(xw[i12], 1);

    tmpr = mult_r(w[8], sub(xw[i10], xwi[i10]));
    tmpi = mult_r(w[8], add(xwi[i10], xw[i10]));
    X[i2]  = shr(add(xw[i2], tmpr), 1);
    Xi[i2] = shr(add(xwi[i2], tmpi), 1);
    X[i6]  = shr(sub(xw[i2], tmpr), 1);
    Xi[i6] = shr(sub(tmpi, xwi[i2]), 1);

    tmpr = sub(mult_r(w[12], xw[i9]), mult_r(w[4],xwi[i9]));
    tmpi = add(mult_r(w[12], xwi[i9]), mult_r(w[4],xw[i9]));
    X[i1]  = shr(add(xw[i1], tmpr), 1);
    Xi[i1] = shr(add(xwi[i1], tmpi), 1);
    X[i7]  = shr(sub(xw[i1], tmpr), 1);
    Xi[i7] = shr(sub(tmpi, xwi[i1]), 1);

    tmpr = sub(mult_r(w[4], xw[i11]), mult_r(w[12],xwi[i11]));
    tmpi = add(mult_r(w[4], xwi[i11]), mult_r(w[12],xw[i11]));
    X[i3]  = shr(add(xw[i3], tmpr), 1);
    Xi[i3] = shr(add(xwi[i3], tmpi), 1);
    X[i5]  = shr(sub(xw[i3], tmpr), 1);
    Xi[i5] = shr(sub(tmpi, xwi[i3]), 1);

    move16();  move16();  move16();  move16();
    move16();  move16();  move16();  move16();
    move16();  move16();  move16();  move16();
    move16();  move16();  move16();  move16();
  }

  xw[0]  = shr(add(X[0], X[16]), 1);   move16();
  xw[16] = shr(sub(X[0], X[16]), 1);   move16();
  xw[8]  = shr(X[8], 1);               move16();

  FOR (k = 1; k < 8; k++)
  {
    i2 = add(k, 16);
    i3 = sub(16, k);
    i5 = add(k,k);
    i6 = sub(16, i5);

    tmpr = sub(mult_r(w[i6], X[i2]), mult_r(w[i5],Xi[i2]));
    xw[k]  = shr(add(X[k], tmpr), 1);   move16();
    xw[i3] = shr(sub(X[k], tmpr), 1);   move16();
  }

  xw[32]  = shr(add(X[32], X[48]), 1);  move16();
  xw[48]  = shr(sub(X[32], X[48]), 1);  move16();
  xw[40]  = shr(X[40], 1);              move16();
  xwi[40] = shr(X[56], 1);              move16();

  FOR (k = 1; k < 8; k++)
  {
    i1 = add(32, k);
    i2 = add(i1, 16);
    i3 = sub(48, k);
    i5 = add(k,k);
    i6 = sub(16, i5);

    tmpr = sub(mult_r(w[i6], X[i2]), mult_r(w[i5],Xi[i2]));
    tmpi = add(mult_r(w[i6], Xi[i2]), mult_r(w[i5],X[i2]));
    xw[i1]  = shr(add(X[i1], tmpr), 1);   move16();
    xwi[i1] = shr(add(Xi[i1], tmpi), 1);  move16();
    xw[i3]  = shr(sub(X[i1], tmpr), 1);   move16();
    xwi[i3] = shr(sub(tmpi, Xi[i1]), 1);  move16();
  }


  W[0]  = add(xw[0], xw[32]);  move16();
  W[32] = sub(xw[0], xw[32]);  move16();
  W[16] = xw[16];              move16();

  FOR (k = 1; k < 16; k++)
  {
    i2 = add(k, 32);
    i3 = sub(32, k);
    i6 = sub(16, k);
    
    tmpr = sub(mult_r(w[i6], xw[i2]), mult_r(w[k],xwi[i2]));
    W[k]  = add(xw[k], tmpr);   move16();
    W[i3] = sub(xw[k], tmpr);   move16();
  }
}
