/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/

#include <stdio.h>
#include <stdlib.h>
#include "errexit.h"

void  error_exit( char *str )
{
  if ( str != NULL )
    fprintf( stderr, "%s\n", str );
  exit(1);
}
