/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: g711App.h
 *  Function: Header of the G.711 encoder and decoder Toolbox
 *------------------------------------------------------------------------
 */

#ifndef G711APP_H
#define G711APP_H

#define  MODE_ULAW           1
#define  MODE_ALAW           2

#define  NSamplesPerFrame8k  40   /* Number of samples a frame in 8kHz  */

#define  NBytesPerFrame0     40   /* Sub codec 0 */

#define  NBitsPerFrame0      (NBytesPerFrame0*8)

#define  NBYTEPERFRAME_MAX   NBytesPerFrame0 /* Max value of NBytesPerFrameX */

#define  TotalBytesPerFrame  (NBytesPerFrame0)
#define  TotalBitsPerFrame   (NBitsPerFrame0)

/* Function prototypes */

void* G711AppEncode_const(int law, int ns);
void  G711AppEncode_dest(void* p_work);
int   G711AppEncode_reset(void* p_work);
int   G711AppEncode( const short* inwave, unsigned char* bitstream, void* p_work );

void* G711AppDecode_const(int law, int ng, int ferc, int pf);
void  G711AppDecode_dest(void* p_work);
int   G711AppDecode_reset(void* p_work);
int   G711AppDecode( const unsigned char* bitstream, short* outwave, void* p_work, int ploss_status );

#endif  /* G711APP_H */
