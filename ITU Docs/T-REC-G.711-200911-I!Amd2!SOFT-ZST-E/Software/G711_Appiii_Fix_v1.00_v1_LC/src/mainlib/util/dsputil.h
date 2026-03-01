/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/

#ifndef DSPUTIL_H
#define DSPUTIL_H

#ifndef _STDLIB_H_
#include <stdlib.h>
#endif
#ifndef _STL_H
#include "stl.h"
#endif

#define a_mac() mac_r(0L,0,0)       /* Addressing MAC operator */

void   zero16( int n, Word16 *xx_16 );
void   mov16( int n, Word16 *xx_16, Word16 *yy_16 );
Word16 Cnv32toNrm16( Word32 lX, Word16 *pnQ );
int    Exp16Array( int n, Word16 *x_16 );

#include "mathtool.h"

#endif
