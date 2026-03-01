/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/

#include "dsputil.h"

/*----------------------------------------------------------------
  Function:
    Fills zeros in an array.
  Return value
    None
  ----------------------------------------------------------------*/
void zero16(
  int     n,     /* I : */
  Word16  *sx    /* O : */
)
{
  int k;

  FOR ( k = 0; k < n; k++ )
  {
    sx[k] = 0; move16();
  }
}

/*----------------------------------------------------------------
  Function:
    Copies array data.
  Return value
    None
  ----------------------------------------------------------------*/
void mov16(
  int     n,     /* I : */
  Word16  *sx,   /* I : */
  Word16  *sy    /* O : */
)
{
  int k;

  FOR ( k = 0; k < n; k++ )
  {
    sy[k] = sx[k]; move16();
  }
}

/*----------------------------------------------------------------
  Function:
    Converts 32-bit value to left-justified 16-bit value
  Return value
    left-justified 16-bit value
  ----------------------------------------------------------------*/
Word16 Cnv32toNrm16(
  Word32  lX,
  Word16  *pnQ
)
{
  Word16  sX, nExp;

  nExp = norm_l( lX );
  sX = extract_h( L_shl( lX, nExp ) );

  *pnQ = nExp; move16();
  return sX;
}

/*----------------------------------------------------------------
  Function:
    Finds number of shifts to normalize a 16-bit array variable.
  Return value
    Number of shifts
  ----------------------------------------------------------------*/
int    Exp16Array(
  int     n,     /* (i): Array size   */
  Word16  *sx    /* (i): Data array   */
)
{
  int     k;
  int     exp;
  Word16  sMax;
  Word16  sAbs;

  sMax = 0; move16();

  FOR ( k = 0; k < n; k++ )
  {
    sAbs = abs_s( sx[k] );
    if ( sub( sMax, sAbs ) < 0 )
    {
      sMax = sAbs; move16();
    }
  }

  exp = norm_s( sMax );
  return exp;
}
