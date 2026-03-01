/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/

#ifndef MATHTOOL_H
#define MATHTOOL_H

int    SqrtI31( Word32 input, Word32 *output );
void   Isqrt_n( Word32 *frac, Word16 *exp );
Word32 Inv_sqrt( Word32 L_x );
Word16 sqrt_q15( Word16 in );


/* Tables */
extern const Word16 table_sqrt_w[49];
extern Word16 table_isqrt[49];

#endif
