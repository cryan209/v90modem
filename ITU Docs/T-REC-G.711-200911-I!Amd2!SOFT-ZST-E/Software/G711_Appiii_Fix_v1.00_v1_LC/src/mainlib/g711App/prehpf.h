/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: prehpf.h
 *  Function: Header of pre-processing 1-tap high-pass filtering
 *------------------------------------------------------------------------
 */

#ifndef FPREHPF_H
#define FPREHPF_H

void  *highpass_1tap_iir_const (void);
void  highpass_1tap_iir_dest (void*);
void  highpass_1tap_iir_reset (void*);
void  highpass_1tap_iir (Word16, Word16, Word16*, Word16*, void*);

#endif  /* FPREHPF_H */
