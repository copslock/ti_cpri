/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
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

/**
 *   @file  tsip_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the TSIP
 *      driver. The OSAL layer can be ported in either of the following 
 *      manners to a native OS:
 *
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt Libraries
 *           - Ensure that an implementation is provided for all 
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
 *           - Rebuild the TSIP Driver library; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - TSIP Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2010 Texas Instruments, Inc.
 *  \par
 */
#ifndef __TSIP_OSAL_H__
#define __TSIP_OSAL_H__

/** @addtogroup TSIP_OSAL_API
 @{ */
/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

extern void Osal_tsipEnterCriticalSection(uint32_t *intState);
extern void Osal_tsipExitCriticalSection(uint32_t intState);

/**
 * @brief   The macro is used by the TSIP driver to protect its internal shared
 * "resources". The macro needs to be defined accounting for the system 
 *  architecture in which the TSIP driver is being used. 
 *
 *  This is called by the TSIP driver when it enters the shared resource 
 *  modification
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_tsipEnterCriticalSection(unsigned int *intState)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  intState is memory location where current state of protection is 
 *      saved for future use while restoring it via Osal_tsipExitCriticalSection().
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Tsip_osalEnterCriticalSection      Osal_tsipEnterCriticalSection

/**
 * @brief   The macro is used by the TSIP driver to protect its internal shared
 * "resources". The macro is called by the TSIP driver when its exits the shared
 * resource modification.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_tsipExitCriticalSection(void* critSectHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  intState is original state of protection at time when the corresponding
 *      Osal_tsipEnterCriticalSection() was called
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Tsip_osalExitCriticalSection      Osal_tsipExitCriticalSection

/**
@}
*/

#endif /* __TSIP_OSAL_H__ */

