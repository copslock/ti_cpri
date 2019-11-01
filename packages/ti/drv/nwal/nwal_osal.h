/******************************************************************************
 * FILE PURPOSE:  NWAL header file with OSAL APIs
 ******************************************************************************
 * FILE NAME:   nwal_osal.h 
 *
 * DESCRIPTION: API functions implementation from external application
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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
 *   @file  nwal_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the NWAL 
 *      driver. The OSAL layer can be ported in either of the following 
 *      manners to a native OS:
 *
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt Libraries
 *           - Ensure that the provide an implementation of all 
 *             Osal_XXX API for their native OS.
 *           - Link the prebuilt libraries with their application.
 *           - Refer to the "test" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Customers can reuse prebuilt TI provided libraries
 *       @n <b> Cons: </b>
 *           - Level of indirection in the API to get to the actual OS call
 *              
 *      <b> Approach 2: </b>
 *      @n  Rebuilt Library 
 *           - Create a copy of this file and modify it to directly 
 *             inline the native OS calls
 *           - Rebuild the NWAL low level driver library; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this 
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - NWAL LLD Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      Copyright (c) Texas Instruments Incorporated 2010-2011
 *  \par
 */
#ifndef __NWAL_OSAL_H__
#define __NWAL_OSAL_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <ti/drv/nwal/nwal.h> 


/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

extern void  Osal_nwalCsEnter (uint32_t *key);
extern void  Osal_nwalCsExit (uint32_t key);

extern void Osal_nwalInvalidateCache (void *ptr, uint32_t size);
extern void Osal_nwalWriteBackCache (void *ptr, uint32_t size);

extern uint16_t Osal_nwalGetProcId (void );
extern unsigned int Osal_nwalLocToGlobAddr (unsigned int x);
extern uint64_t Osal_nwalGetTimeStamp(void);

/**
 * @brief   The macro is used by the NWAL LLD to provide critical sections to 
 *          protect global and shared variables access from multiple threads 
 *          on single core and from multiple cores 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_nwalCsEnter (uint32_t *key)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Key used to lock the critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define NWAL_osalCsEnter          Osal_nwalCsEnter

/**
 * @brief   The macro is used by the NWAL LLD to exit a critical section 
 *      protected using Osal_nwalCsEnter() API.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_nwalCsExit (uint32_t key)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Key used to lock the critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define NWAL_osalCsExit           Osal_nwalCsExit

/**
 * @brief   The macro is used by the NWAL LLD to indicate that a block
 * of memory is about to be accessed. If the memory block is cached then
 * this indicates that the application would need to ensure that the cache
 * is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_nwalInvalidateCache (void *ptr, uint32_t size) 
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */ 
#define NWAL_osalInvalidateCache             Osal_nwalInvalidateCache


/**
 * @brief   The macro is used by the NWAL LLD to indicate that the block of 
 * memory has finished being accessed. If the memory block is cached then the 
 * application would need to ensure that the contents of the cache are updated
 * immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_nwalWriteBackCache (void *ptr, uint32_t size) 
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define NWAL_osalWriteBackCache               Osal_nwalWriteBackCache

/**
 * @brief   The macro is used by the NWAL LLD to get the processor ID(zero based)
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      uint32_t Osal_nwalGetProcId () 
 *   @endverbatim
 *
 *  <b> Return Value Processor ID zero based </b>
 *  @n  Not applicable.
 */ 
#define NWAL_osalGetProcId             Osal_nwalGetProcId

/**
 * @brief   The macro is used by the NWAL LLD to convert local to Global Address
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      unsigned int Osal_nwalLocToGlobAddr () 
 *   @endverbatim
 *
 *  <b> Convert local to Global address </b>
 *  @n  Not applicable.
 */ 
#define NWAL_locToGlobAddr            Osal_nwalLocToGlobAddr

/**
 * @brief   The macro is used by the NWAL LLD to get system timestamp
 *          Timestamp will be passed to application while sending packet back.
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      unsigned int Osal_nwalGetTimeStamp () 
 *   @endverbatim
 *
 *  <b> Get Timestamp </b>
 *  @n  Not applicable.
 */ 
#define NWAL_getTimeStamp            Osal_nwalGetTimeStamp

/**
@}
*/
#ifdef __cplusplus
}
#endif
#endif /* __NPU2_OSAL_H__ */

