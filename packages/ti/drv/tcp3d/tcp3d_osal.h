/**
 *   @file  tcp3d_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the TCP3D
 *      driver. The OSAL layer can be ported in either of the following 
 *      manners to a native OS:
 *
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt Libraries
 *           - Ensure that the provide an implementation of all 
 *             Osal_XXX API for their native OS.
 *           - Link the prebuilt libraries with their application.
 *           - Refer to the "example" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Customers can reuse prebuilt TI provided libraries
 *       @n <b> Cons: </b>
 *           - Level of indirection in the API to get to the actual OS call
 *              
 *      <b> Approach 2: </b>
 *      @n  Rebuilt Library 
 *           - Create a copy of this file and modify it to directly 
 *             inline the native OS calls
 *           - Rebuild the TCP3D Driver library; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this 
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - TCP3D Libraries need to be rebuilt by the customer.
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

/** @addtogroup TCP3D_OSAL_API
 @{ */

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

extern void* Osal_memset(void *_mem, int32_t _ch, size_t _n);
extern void* Osal_memcpy(void *_s1, const void *_s2, size_t _n);

extern void Osal_tcp3dLog( String fmt, ... );
extern void Osal_tcp3dBeginMemAccess (void *ptr, uint32_t size);
extern void Osal_tcp3dEndMemAccess (void *ptr, uint32_t size);

/**
 * @brief   The macro is used by the TCP3D driver to log various messages.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_tcp3dLog( String fmt, ... ) 
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string 
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Tcp3d_osalLog            Osal_tcp3dLog

/**
 * @brief   The macro is used by the TCP3D driver to do memory copy.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_memcpy(void *_s1, const void *_s2, size_t _n)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  _s1 - destination address
 *  @n  _s2 - source address
 *  @n  _n  - number of bytes
 *
 *  <b> Return Value </b>
 *  @n  destination address
 */
#define Tcp3d_memcpy            Osal_memcpy

/**
 * @brief   The macro is used by the TCP3D driver to initialize memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_memset(void *_mem, int32_t _ch, size_t _n)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  _mem - destination address
 *  @n  _ch  - value to be set with
 *  @n  _n   - number of bytes
 *
 *  <b> Return Value </b>
 *  @n  address value
 */
#define Tcp3d_memset            Osal_memset

/**
@}
*/

#endif /* __TCP3D_OSAL_H__ */

