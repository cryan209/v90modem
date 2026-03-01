/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: g711a.c
 *  Function: encoding/decoding (A-law)
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"

/*
 * Conversion of a linear sample to an enhanced Mu law coded sample
 *
 * G.711 quantizer encodes the input sample with 8 bits
 */

/* Core layer encoding */
short convertLin_ALaw(  /* Returns core layer code */
  short x,       /* (i): Input sample                                   */
  short *ind2,   /* (o): Refinement signal which has a 3-bit resolution */
  short *xq,     /* (o): Locally decoded core layer sample              */
  short *expo    /* (o): Exponent value of core layer sample            */
)
{
  short sign, exp, mant, ind_tmp, ind, val;

  sign = 0x80; move16();

  IF (x < 0)
  {
    x = negate(x);                  /* 2's complement is used. */
 /* x = s_xor(x, (short)0xFFFF); */ /* abs(x) - 1 */
    sign = 0; move16();
  }
  IF (sub(x, 255) > 0)              /* 1st bit 1 + 4 saved bits */
  {
    exp = sub(7, norm_s(x));        /* 7 >= exp >= 1 */
    x = shr(x, exp);
    *ind2 = s_and(x, 0x7);          /* save 3 LSB */
    move16();                       /* *ind2 is an array variable. */
    x = shr(x, 3);
    mant = sub(x, 16);              /* remove leading 1 */
  }
  ELSE
  {
    exp = 0; move16();
    x = shr(x, 1);
    *ind2 = s_and(x, 0x7);          /* vorono?index : save 3 LSB */
    move16();                       /* *ind2 is an array variable. */
    mant = shr(x, 3);
  }
  ind_tmp = add(sign, add(shl(exp,4), mant));
  ind = s_xor(ind_tmp, 0x0055);     /* toogle odd bits */

  *expo = exp;  move16();

  /* decode g711 for noise shaping */

  val = shl(mant, 4);               /* get mantissa to right position */
  val = add(val, 8);                /* rounding */

  IF (exp > 0)
  {
    exp = sub(exp, 1);
    val = shl(add(val, 256), exp);  /* adding leading 1 */
  }
  if (sign == 0)                    /* sign bit ==0 ' negative value */
  {
    val = negate(val);
  }
  *xq = val;  move16();

  return ind;
}

/* Core layer decoding */
short convertALaw_Lin(  /* Returns core layer signal */
  short ind,     /* (i): Input core layer (G.711) code       */
  short *expo,   /* (o): Exponent value of core layer sample */
  short *signo   /* (o): Sign of core layer sample           */
)
{
  short y, exp, val;
  short sign;

  sign = s_and(ind, 0x80);
  *signo = sign; move16();          /* *signo is an array variable. */

  y = s_and(s_xor(ind, 0x0055), 0x7F);  /* without sign */
  exp = shr(y, 4);
  *expo = exp; move16();            /* *expo is an array variable. */

  val = shl(s_and(y, 0xF), 4);      /* get mantissa to right position */
  val = add(val, 8);                /* rounding */

  IF (exp > 0)
  {
    exp = sub(exp, 1);
    val = shl(add(val, 256), exp);  /* adding leading 1 */
  }

  if (sign == 0)  /* sign bit ==0 ' negative value */
  {
    val = negate(val);
  }

  return val;
}

