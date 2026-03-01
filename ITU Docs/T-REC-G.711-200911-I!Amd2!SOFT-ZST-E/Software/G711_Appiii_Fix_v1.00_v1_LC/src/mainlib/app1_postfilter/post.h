/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: post.h
 *  Header: Header of main routine of the post-processing
 *------------------------------------------------------------------------
 */

#ifndef POST_H
#define POST_H
 
extern const  Word16 max_err_quant[16];

 
void postProc_Processing(Word16 *bloc_in,
                         Word16 *bloc_out,
                         VAR_MEM *var_mem,
                         Word16  postfilter_sw,
                         Word16  anaflag,
                         Word16 *xl_nq);

#endif /* POST_H */
