/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: g711.h
 *  Function: Header of PCM coders
 *------------------------------------------------------------------------
 */

#ifndef G711_H
#define G711_H


short convertLin_ALaw(short x, short *ind2, short *xq, short* expo);
short convertALaw_Lin(short ind, short *expo, short* signo);

short convertLin_MuLaw(short x, short *ind2, short *xq, short* expo);
short convertMuLaw_Lin(short ind, short *expo, short* signo);

#endif
