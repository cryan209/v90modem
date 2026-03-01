/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: lpctool.c
 *  Function: Linear prediction tools
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "oper_32b.h"
#include "lpctool.h"

/*-------------------------------------------------------------------------*
* Function Levinson                                                        *
*--------------------------------------------------------------------------*/

#define MAXORD  6

void Levinson(
  Word16 R_h[],     /* (i)     : Rh[M+1] Vector of autocorrelations (msb) */
  Word16 R_l[],     /* (i)     : Rl[M+1] Vector of autocorrelations (lsb) */
  Word16 rc[],      /* (o) Q15 : rc[M]   Reflection coefficients.         */
  Word16 * stable,  /* (o)     : Stability flag                           */
  Word16 ord,       /* (i)     : LPC order                                */
  Word16 * a        /* (o) Q12 : LPC coefficients                         */
)
{
  Word32  t0, t1, t2;                     /* temporary variable */
  Word16  A_h[MAXORD+1],  A_l[MAXORD+1];  /* LPC coef. in double prec.    */
  Word16  An_h[MAXORD+1], An_l[MAXORD+1]; /* LPC coef. for next iteration in double prec. */
  Word16  i, j;
  Word16  hi, lo;
  Word16  K_h, K_l;                       /* reflexion coefficient; hi and lo */
  Word16  alp_h, alp_l, alp_exp;          /* Prediction gain; hi lo and exponent */
  Word16  rctmp[MAXORD];

  *stable = 0; move16();

  /* K = A[1] = -R[1] / R[0] */

  t1 = L_Comp(R_h[1], R_l[1]);            /* R[1]             */
  t2 = L_abs(t1);                         /* abs R[1]         */
  t0 = Div_32(t2, R_h[0], R_l[0]);        /* R[1]/R[0] in Q31 */

  if (t1 > 0)
  {
    t0 = L_negate(t0);                    /* -R[1]/R[0] */
  }
  L_Extract(t0, &K_h, &K_l);              /* K in DPF */

  rctmp[0] = K_h; move16();

  t0 = L_shr(t0, 4);
  L_Extract(t0, &A_h[1], &A_l[1]);        /* A[1] in DPF */

  /* Alpha = R[0] * (1-K**2) */

  t0 = Mpy_32(K_h, K_l, K_h, K_l);        /* K*K */
  t0 = L_abs(t0);                         /* Some case <0 !! */
  t0 = L_sub((Word32) 0x7fffffffL, t0);   /* 1 - K*K */
  L_Extract(t0, &hi, &lo);
  t0 = Mpy_32(R_h[0], R_l[0], hi, lo);    /* Alpha in DPF format */

  /* Normalize Alpha */

  alp_exp = norm_l(t0);
  t0 = L_shl(t0, alp_exp);
  L_Extract(t0, &alp_h, &alp_l);          /* DPF format */

  /*-------------------------------------- */
  /* ITERATIONS  I=2 to lpc_order          */
  /*-------------------------------------- */

  FOR (i = 2; i <= ord; i++)
  {
    /* t0 = SUM ( R[j]*A[i-j] ,j=1,i-1 ) + R[i] */

    t0 = Mpy_32(R_h[1], R_l[1], A_h[i - 1], A_l[i - 1]);
    FOR (j = 2; j < i; j++)
    {
      t0 = L_add(t0, Mpy_32(R_h[j], R_l[j], A_h[i - j], A_l[i - j]));
    }
    t0 = L_shl(t0, 4);

    t1 = L_Comp(R_h[i], R_l[i]);
    t0 = L_add(t0, t1);                   /* add R[i] */

    /* K = -t0 / Alpha */

    t1 = L_abs(t0);
    t2 = Div_32(t1, alp_h, alp_l);        /* abs(t0)/Alpha */

    if (t0 > 0)
    {
      t2 = L_negate(t2);                  /* K =-t0/Alpha */
    }
    t2 = L_shl(t2, alp_exp);              /* denormalize; compare to Alpha */
    L_Extract(t2, &K_h, &K_l);            /* K in DPF */
    rctmp[i - 1] = K_h; move16();

    /* Test for unstable filter. If unstable keep old A(z) */

    IF (sub(abs_s(K_h), 32750) > 0)
    {
      *stable = 1; move16();
      return;
    }

    /*------------------------------------------ */
    /*    Compute new LPC coeff. -> An[i]        */
    /*    An[j]= A[j] + K*A[i-j]   , j=1 to i-1  */
    /*    An[i]= K                               */
    /*------------------------------------------ */

    FOR (j = 1; j < i; j++)
    {
      t0 = Mpy_32(K_h, K_l, A_h[i - j], A_l[i - j]);
      t0 = L_add(t0, L_Comp(A_h[j], A_l[j]));
      L_Extract(t0, &An_h[j], &An_l[j]);
    }
    t2 = L_shr(t2, 4);
    L_Extract(t2, &An_h[i], &An_l[i]);

    /* Alpha = Alpha * (1-K**2) */

    t0 = Mpy_32(K_h, K_l, K_h, K_l);      /* K*K */
    t0 = L_abs(t0);                       /* Some case <0 !! */
    t0 = L_sub((Word32) 0x7fffffffL, t0); /* 1 - K*K */
    L_Extract(t0, &hi, &lo);              /* DPF format */
    t0 = Mpy_32(alp_h, alp_l, hi, lo);

    /* Normalize Alpha */

    j = norm_l(t0);
    t0 = L_shl(t0, j);
    L_Extract(t0, &alp_h, &alp_l);        /* DPF format */
    alp_exp = add(alp_exp, j);            /* Add normalization to alp_exp */

    /* A[j] = An[j] */

    FOR (j = 1; j <= i; j++)
    {
      A_h[j] = An_h[j]; move16();
      A_l[j] = An_l[j]; move16();
    }
  }

  a[0] = 4096; move16();

  FOR (i = 1; i <= ord; i++)
  {
    t0 = L_Comp(A_h[i], A_l[i]);
    a[i] = round(L_shl(t0, 1));  move16();
  }
  mov16( ord, rctmp, rc );

  return;
}

/*----------------------------------------------------------*
 * Function Lag_window()                                    *
 *                                                          *
 * r[i] *= lag_wind[i]                                      *
 *                                                          *
 *    r[i] and lag_wind[i] are in special double precision. *
 *    See "oper_32b.c" for the format                       *
 *                                                          *
 *----------------------------------------------------------*/

void Lag_window(
  Word16 * R_h,
  Word16 * R_l,
  const Word16 * W_h,
  const Word16 * W_l,
  Word16 ord
)
{
  Word32  x;
  Word16  i;

  FOR (i = 1; i <= ord; i++)
  {
    x = Mpy_32(R_h[i], R_l[i], W_h[i - 1], W_l[i - 1]);
    L_Extract(x, &R_h[i], &R_l[i]);
  }
  return;
}

/*-------------------------------------------------------------------------*
* Function Autocorr                                                        *
*--------------------------------------------------------------------------*/

#define MAX_LEN    80

void Autocorr(
  Word16 x[],         /* (i)    : Input signal                      */
  const Word16 win[], /* (i)    : Analysis window                   */
  Word16 r_h[],       /* (o)    : Autocorrelations  (msb)           */
  Word16 r_l[],       /* (o)    : Autocorrelations  (lsb)           */
  Word16 ord,         /* (i)    : LPC order                         */
  Word16 len          /* (i)    : length of analysis                */
)
{
  Word32  sum;
  Word16  i, j, norm, tmp;
  Word16  y[MAX_LEN];

  /* Windowing of signal */

  FOR (i = 0; i < len; i++)
  {
    y[i] = mult_r(x[i], win[i]); move16();
  }

  /* Compute r[0] and test for overflow */

  DO
  {
    Overflow = 0; move16();

    sum = L_mac(1, y[0], y[0]); /*  1 for Avoid case of all zeros */
    FOR (i = 1; i < len; i++)
    {
      sum = L_mac(sum, y[i], y[i]);
    }

    /* If overflow divide y[] by 4 */

    IF (Overflow != 0)
    {
      FOR (i = 0; i < len; i++)
      {
        y[i] = shr(y[i], 2); move16();
      }
    }
  }
  WHILE(Overflow != 0);

  /* Normalization of r[0] */

  norm = norm_l(sum);
  sum = L_shl(sum, norm);
  L_Extract(sum, &r_h[0], &r_l[0]); /* Put in DPF format (see oper_32b) */

  /* r[1] to r[m] */

  FOR (i = 1; i <= ord; i++)
  {
    sum = L_mult(y[0], y[i]);
    tmp = sub(len, i);
    FOR (j = 1; j < tmp; j++)
    {
      sum = L_mac(sum, y[j], y[j + i]);
    }
    sum = L_shl(sum, norm);
    L_Extract(sum, &r_h[i], &r_l[i]);
  }
  return;
}

/*------------------------------------------------------------------------*
 *                         WEIGHT_A.C                                     *
 *------------------------------------------------------------------------*
 *   Weighting of LPC coefficients                                        *
 *   ap[i]  =  a[i] * (gamma ** i)                                        *
 *                                                                        *
 *------------------------------------------------------------------------*/

void Weight_a(
  Word16 a[],        /* (i) Q*  : a[m+1]  LPC coefficients             */
  Word16 ap[],       /* (o) Q*  : Spectral expanded LPC coefficients   */
  Word16 gamma,      /* (i) Q15 : Spectral expansion factor.           */
  Word16 m           /* (i)     : LPC order.                           */
)
{
  Word16 i, fac;

  ap[0] = a[0]; move16();
  fac = gamma;  move16();
  FOR (i = 1; i < m; i++)
  {
    ap[i] = mult_r (a[i], fac); move16();
    fac = mult_r (fac, gamma);
  }
  ap[m] = mult_r (a[m], fac); move16();

  return;
}
