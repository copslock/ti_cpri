/*==========================================================================
* Copyright (c) Texas Instruments Incorporated 2006, 2007, 2008, 2009, 2010
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_
#include <sys/types.h>

#if (CC_VERSION == 42)
     #ifdef bool
         #undef bool
     #endif
     #define bool uint8_t
     #define true 1
     #define false 0
#endif

#if !defined(_SYS_INT_TYPES_H) && !defined(__BIT_TYPES_DEFINED__)
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned int uint32_t;
typedef int int32_t;

#ifdef _WIN32
#ifndef GCC
typedef unsigned __int64 uint64_t;
typedef __int64 int64_t;
#else
typedef unsigned long long uint64_t;
typedef long long int64_t;
#endif
#else
typedef unsigned long long uint64_t;
typedef long long int64_t;
#endif/*WIN32*/

#elif defined(__BIT_TYPES_DEFINED__)
typedef u_int8_t  uint8_t;
typedef u_int16_t uint16_t;
typedef u_int32_t uint32_t;
typedef u_int64_t uint64_t;
#endif /* !defined(_SYS_INT_TYPES_H) && !defined(__bit_types_defined__) */
#if defined(__hpux)
#define tid_t tid_t_t
#endif

#ifndef WIN32
#define stricmp strcasecmp
#endif
#endif /*_TYPEDEFS_H_*/
