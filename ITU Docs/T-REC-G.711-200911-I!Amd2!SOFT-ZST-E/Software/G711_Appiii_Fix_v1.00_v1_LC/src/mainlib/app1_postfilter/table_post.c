/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: table_post.c
 *  Function: Tables for postfilter
 *------------------------------------------------------------------------
 */

#include "stl.h"
#include "post_const.h"

/* A-law quantization stepsize */
const  Word16 max_err_quant[16] =  {512,256,128,64,32,16,8,8,8,8,8,8,8,8,8,8};

/* Asymmetric Hanning window (for 64-point FFT processing) */
const Word16 Hann_sh16[L_WIN] = {
        34      ,   137     ,   308     ,   547     ,
        852     ,   1222    ,   1656    ,   2151    ,
        2706    ,   3319    ,   3986    ,   4705    ,
        5474    ,   6288    ,   7144    ,   8039    ,
        8969    ,   9931    ,   10919   ,   11930   ,
        12960   ,   14005   ,   15059   ,   16119   ,
        17180   ,   18237   ,   19287   ,   20325   ,
        21346   ,   22346   ,   23322   ,   24268   ,
        25181   ,   26057   ,   26893   ,   27684   ,
        28429   ,   29122   ,   29763   ,   30347   ,
        30872   ,   31337   ,   31739   ,   32077   ,
        32349   ,   32554   ,   32691   ,   32759   ,
        32694   ,   32104   ,   30947   ,   29263   ,
        27113   ,   24576   ,   21743   ,   18716   ,
        15604   ,   12521   ,   9578    ,   6880    ,
        4526    ,   2601    ,   1174    ,   296
    };

/* Half of 16-point Hanning window for filter interpolation */
const Word16 Hann_sh16_p6[7] = {
        1656    ,   5474    ,   10919   ,   17180   ,
        23322   ,   28429   ,   31739
    };

/* Complementary half of Hanning window for filter interpolation */
const Word16 Hann_sh16_p6m1[7] = {
        31111   ,   27293   ,   21848   ,   15587   ,
        9445    ,   4338    ,   1028
    };

/* Index mapping table for FFT */
const Word16 r[64] = {
         0, 32, 16, 48,  8, 40, 24, 56,
         4, 36, 20, 52, 12, 44, 28, 60,
         2, 34, 18, 50, 10, 42, 26, 58,
         6, 38, 22, 54, 14, 46, 30, 62,
         1, 33, 17, 49,  9, 41, 25, 57,
         5, 37, 21, 53, 13, 45, 29, 61,
         3, 35, 19, 51, 11, 43, 27, 59,
         7, 39, 23, 55, 15, 47, 31, 63
};

/* Twiddle factors for FFT */
const Word16 w[16] = {
       -1,  3212,  6393,  9512, 12540, 15447, 18205, 20788,
    23170, 25330, 27246, 28899, 30274, 31357, 32138, 32610
};

/* Truncating window for impulse response of noise reduction filter */
const Word16 WinFilt[L_FLT_DIV2+1] = {
        0,   278,  1106,  2454,  4276,  6510,  9081, 11900, 14872,
    17895, 20867, 23686, 26257, 28491, 30313, 31661, 32489,
};
