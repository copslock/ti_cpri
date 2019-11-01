/**
 *   @file  sample_osal.c
 *
 *   @brief   
 *      This is the OS abstraction layer and is used by the CPPI and QMSS
 *      low level drivers for the CPPI sample example.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
/* XDC includes */ 
#include <xdc/std.h>
#include <xdc/cfg/global.h>

/**
 *  @b Description
 *  @n  
 *      The function is the TCP3D OSAL Logging API which logs 
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_tcp3dLog ( char *fmt, ... )
{
}

/**
 *  @b Description
 *  @n  
 *      The function is the TCP3D OSAL memory copy API which is directly 
 *      mapped to memcpy funciton.
 *
 *  @param[in]  _s1
 *      destination address
 *
 *  @param[in]  _s2
 *      source address
 *
 *  @param[in]  _n
 *      number of bytes
 *
 *  @retval
 *      destination address
 */
void* Osal_memcpy(void *_s1, const void *_s2, size_t _n)
{
    return memcpy(_s1, _s2, _n);
}
