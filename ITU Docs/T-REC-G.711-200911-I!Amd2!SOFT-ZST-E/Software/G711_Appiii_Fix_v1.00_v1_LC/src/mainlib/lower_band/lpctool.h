/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: lpctool.h
 *  Function: Header of linear prediction tools
 *------------------------------------------------------------------------
 */

#ifndef LPCTOOL_H
#define LPCTOOL_H


void  Levinson(Word16 R_h[], Word16 R_l[], Word16 rc[], Word16 * stable, Word16 ord, Word16 * a);

void  Lag_window(Word16 * R_h, Word16 * R_l, const Word16 * W_h, const Word16 * W_l, Word16 ord);

void  Autocorr(Word16 x[], const Word16 win[], Word16 r_h[], Word16 r_l[], Word16 ord, Word16 len);

void  Weight_a(Word16 a[], Word16 ap[], Word16 gamma, Word16 m);

#endif
