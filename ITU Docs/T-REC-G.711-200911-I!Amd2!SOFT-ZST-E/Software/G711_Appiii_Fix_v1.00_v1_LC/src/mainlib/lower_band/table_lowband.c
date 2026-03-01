/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: table_lowband.c
 *  Function: Tables for lower-band modules
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "lowband.h"
#include "fec_lowband.h"

/***********************************
   Tables used in autocorr_ns.c
 ***********************************/

/* Window for LPC analysis in noise shaping, Q15 */
/* Average of cos/ham (60+20) */
Word16 NS_window[L_WINDOW] = {
      0,    668,   1142,   1636,   2152,   2688,   3245,   3820,   4414,   5026,
   5654,   6299,   6959,   7634,   8321,   9020,   9731,  10451,  11181,  11917,
  12660,  13408,  14160,  14913,  15668,  16423,  17176,  17925,  18671,  19410,
  20142,  20866,  21579,  22282,  22971,  23646,  24306,  24950,  25575,  26181,
  26767,  27332,  27873,  28391,  28884,  29352,  29793,  30206,  30590,  30946,
  31271,  31566,  31830,  32061,  32261,  32428,  32562,  32663,  32730,  32764,
  32730,  32428,  31830,  30946,  29793,  28391,  26767,  24950,  22971,  20866,
  18671,  16423,  14160,  11917,   9731,   7634,   5654,   3820,   2152,    668};

/* Lag window for noise shaping */
/* Bandwidth expansion = 120Hz (fs = 8kHz) */
/* noise floor = 1.0001 (1/1.0001 on r[1]..r[M], r[0] not stored) */
/* for i=1:M-1, wdw(i)=32768/wnc*exp(-.5*(2*pi*bwe/fs*i)^2); end; */

/*floor(exp(-0.5*(2*pi*120*k/8000)^2)*.9999*32768) */
const Word16 NS_lag_h[ORD_M] = {
  32619,
  32187,
  31480,
  30517
};

/*floor(rem(floor(exp(-0.5*(2*pi*120*k/8000)^2)*.9999*32768*65536+0.5),65536)/2)*/
const Word16 NS_lag_l[ORD_M] = {
  17275,
  25832,
  28989,
  7367,
};


/***********************************
   Tables used in fec_lowband.c
 ***********************************/

/*-----------------------------------------------------*
| Lag window for low-band FERC.                       |
| noise floor = 1.0001   = (0.9999  on r[1] ..r[10])  |
| Bandwidth expansion = 60 Hz                         |
|                                                     |
| Special double precision format. See "oper_32b.c"   |
|                                                     |
| lag_wind[0] =  1.00000000    (not stored)           |
| lag_wind[1] =  0.99879038                           |
| lag_wind[2] =  0.99546897                           |
| lag_wind[3] =  0.98995781                           |
| lag_wind[4] =  0.98229337                           |
| lag_wind[5] =  0.97252619                           |
| lag_wind[6] =  0.96072036                           |
| lag_wind[7] =  0.94695264                           |
| lag_wind[8] =  0.93131179                           |
|                                                     |
-----------------------------------------------------*/

const Word16    LBFEC_lag_h[16] = {
  32728,
  32619,
  32438,
  32187,
  31867,
  31480,
  31029,
  30517,
  29946,
  29321,
  28645,
  27923,
  27158,
  26356,
  25521,
  24658
};

const Word16    LBFEC_lag_l[16] = {
  11918,
  17274,
  30692,
  25832,
  24195,
  28989,
  24381,
  7367,
  19522,
  14788,
  22083,
  12911,
  31066,
  27372,
  22094,
  5192
};

/* Window for LPC analysis in low-band FEC, Q15 */
const Word16    LBFEC_lpc_win_80[80] = {
  (Word16)  2621, (Word16)  2637, (Word16)  2684, (Word16)  2762, (Word16)  2871, 
  (Word16)  3010, (Word16)  3180, (Word16)  3380, (Word16)  3610, (Word16)  3869, 
  (Word16)  4157, (Word16)  4473, (Word16)  4816, (Word16)  5185, (Word16)  5581, 
  (Word16)  6002, (Word16)  6447, (Word16)  6915, (Word16)  7406, (Word16)  7918, 
  (Word16)  8451, (Word16)  9002, (Word16)  9571, (Word16) 10158, (Word16) 10760, 
  (Word16) 11376, (Word16) 12005, (Word16) 12647, (Word16) 13298, (Word16) 13959, 
  (Word16) 14628, (Word16) 15302, (Word16) 15982, (Word16) 16666, (Word16) 17351, 
  (Word16) 18037, (Word16) 18723, (Word16) 19406, (Word16) 20086, (Word16) 20761, 
  (Word16) 21429, (Word16) 22090, (Word16) 22742, (Word16) 23383, (Word16) 24012, 
  (Word16) 24629, (Word16) 25231, (Word16) 25817, (Word16) 26386, (Word16) 26938, 
  (Word16) 27470, (Word16) 27982, (Word16) 28473, (Word16) 28941, (Word16) 29386, 
  (Word16) 29807, (Word16) 30203, (Word16) 30573, (Word16) 30916, (Word16) 31231, 
  (Word16) 31519, (Word16) 31778, (Word16) 32008, (Word16) 32208, (Word16) 32378, 
  (Word16) 32518, (Word16) 32627, (Word16) 32705, (Word16) 32751, (Word16) 32767, 
  (Word16) 32029, (Word16) 29888, (Word16) 26554, (Word16) 22352, (Word16) 17694, 
  (Word16) 13036, (Word16)  8835, (Word16)  5500, (Word16)  3359, (Word16)  2621
};

/* FIR decimation filter coefficients in low-band FERC, Q16 */
/* 8th order FIRLS 8000 400 900 3 19 */ 
const Word16    LBFEC_fir_lp[FEC_L_FIR_FILTER_LTP] = {
  (Word16)  3692, (Word16)  6190, (Word16)  8525, (Word16) 10186, 
  (Word16) 10787, (Word16) 10186, (Word16)  8525, (Word16)  6190, (Word16)  3692
};
