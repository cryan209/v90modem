/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post.c
 *  Function: Main routine that calls all post-processing subroutines
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "post_const.h"
#include "post.h"
#include "post_anasyn.h"
#include "post_gainfct.h"


/**
* Process of post processing (done per frame)
* Call sequently the different functions :
* 1. Analysis of the signal : postProc_Analysis()
* 2. Compute the Impulse Response of the post process filter : postProc_GainProcess() 
* 3. Synthesis of the output signal with a time OLS : postProc_Synthesis()
*/

void postProc_Processing(
  Word16  *bloc_in,      /* IN:  Table of the new samples */
  Word16  *bloc_out,     /* OUT: Table of the filtered samples */
  VAR_MEM *var_mem,      /* I/O: State structure including state variables */
  Word16  postfilter_sw, /* IN:  Post processing ON/OFF switch */
  Word16  anaflag,       /* IN:  Analysis flag */
  Word16  *xl_nq         /* IN:  Speech buffer */
)
{
  Word16  max_eq;
  Word16  i;

  IF (postfilter_sw != 0)  /* Postfilter option is ON */
  {
    IF (anaflag != 0)
    {
      postProc_Analysis(bloc_in, &(var_mem->var_anaSynth));
  
      postProc_GainProcess(var_mem->var_anaSynth.X,
                           var_mem->var_anaSynth.x_shift,
                           &(var_mem->var_gain),
                           &(var_mem->var_anaSynth)
                          );
    }
    ELSE
    {
      mov16(FRAME_LGTH_POSTPROC,
            bloc_in,
            &(var_mem->var_anaSynth.x[L_WIN-FRAME_LGTH_POSTPROC])
           );
    }
    postProc_Synthesis(bloc_out,
                       var_mem->var_gain.h,
                       &(var_mem->var_anaSynth)
                      );

    FOR (i= 0; i < FRAME_LGTH_POSTPROC; i++)
    {
      IF (sub(abs_s(bloc_out[i]),24) > 0)
      {
        max_eq = max_err_quant[norm_s(xl_nq[i])];
        bloc_out[i] = s_max(bloc_out[i], sub(xl_nq[i], max_eq));
        bloc_out[i] = s_min(bloc_out[i], add(xl_nq[i], max_eq));

        move16();
        move16();
        move16();
      }
    }
  }
}
