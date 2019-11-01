/**
 *   @file  tcp3d_osal.h
 *
 *   @brief   
 *      This is the OS adaptation layer for the TCP3D Driver which has 
 *      been ported for XDC Runtime and BIOS.
 *
 *      This is an example of <b> Approach 2 </b> in which the 
 *      application rebuilds the TCP3D driver with the new definitions.
 *
 *      Please refer to the 'tcp3d_osal.h' in the API documentation.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
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
#ifndef __TCP3D_OSAL_H__
#define __TCP3D_OSAL_H__

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>

/* TCP3D OSAL Logging API is mapped directly to an XDC Runtime API */
#define Tcp3d_osalLog           System_printf

/* TCP3D OSAL memory operation APIs are mapped directly with standard runtime APIs */
#define Tcp3d_memcpy            memcpy
#define Tcp3d_memset            memset

#endif /* __TCP3D_OSAL_H__ */

