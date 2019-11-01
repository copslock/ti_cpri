/**
 *   @file  bmet_eth_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the BareMetal Transport low level
 *      driver. 
 *    
 *   @details The OSAL layer can be ported in either of the following manners to a native OS:
 *
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt Libraries
 *           - Ensure that the LLD users provide an implementation of all 
 *             Osal_XXX API for their native OS.
 *           - Link the prebuilt libraries with their application.
 *           - Refer to the "test" directory for an example of this
 *       @n <b> Pros: </b>
 *              - Customers can reuse prebuilt TI provided libraries
 *       @n <b> Cons: </b>
 *              - Level of indirection in the API to get to the actual OS call
 *              
 *      <b> Approach 2: </b>
 *      @n  Rebuilt Library 
 *           - Create a copy of this file and modify it to directly 
 *             inline the native OS calls
 *           - Rebuild the BMET_ETH  low level driver library; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this 
 *       @n <b> Pros: </b>
 *              - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *              - BMET_ETH  LLD Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 - 2018 Texas Instruments, Inc.
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
#ifndef _BMET_ETH_OSAL_H
#define _BMET_ETH_OSAL_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BMET_ETH_LLD_OSAL
 @{ */
 
/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/
extern void  Osal_bmet_eth_EndPktAccess (void* pkt, uint32_t size);
extern void  Osal_bmet_eth_EndMemAccess (void* addr, uint32_t size);
extern void  Osal_bmet_eth_BeginPktAccess(void* addr, uint32_t size);
extern void  Osal_bmet_eth_MemFree(void* ptr, uint32_t size);
extern void* Osal_bmet_eth_MemAlloc(uint32_t num_bytes, uint32_t alignment);
extern void Osal_bmet_eth_Exception (uint32_t moduleID, int32_t exception_num);


/**
 * @brief   The macro is used by the bmet Library to allocate the memory
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *       void* Osal_bmet_eth_MemAlloc (uint32_t num_bytes, uint32_t alignment)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n num_bytes      - number of bytes to be allocated
 *  @n alignment      - memory alignment
 *
 *  <b> Return Value </b>
 *  @n  memory address allocated
 */

#define BMET_ETH_osalMemAlloc   Osal_bmet_eth_MemAlloc


/**
 * @brief   The macro is used by the bmet Library to free the memory
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *       void  Osal_bmet_eth_MemFree (void* ptr, uint32_t size)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n ptr      - memory base address to be freed
 *  @n size     - size of the mem block
 *
 *  <b> Return Value </b>
 *  @n  None
 */

#define BMET_ETH_osalMemFree   Osal_bmet_eth_MemFree



/**
 * @brief   The macro is used by the bmet Library to indicate that packet
 * access has been accessed & updated . If the packet is in cached memory the 
 * implementation should writeback the contents of the packet
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *       void Osal_bmet_eth_EndMemAccess (void* addr, uint32_t sizeWords)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n addr      - The address of the table to be accessed
 *  @n sizeWords - The number of bytes in the table
 *
 *  <b> Return Value </b>
 *  @n  None
 */

#define BMET_ETH_osalEndPktAccess   Osal_bmet_eth_EndPktAccess


/**
 * @brief   The macro is used by the bmet Library to indicate that packet
 * access has been accessed & updated . If the packet is in cached memory the 
 * implementation should writeback the contents of the packet
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *       void Osal_bmet_eth_BeginMemAccess (void* addr, uint32_t sizeWords)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n addr      - The address of the table to be accessed
 *  @n sizeWords - The number of bytes in the table
 *
 *  <b> Return Value </b>
 *  @n  None
 */

#define BMET_ETH_osalBeginPktAccess   Osal_bmet_eth_BeginPktAccess



/**
 * @brief  This macro is used to alert the application that the BMET_ETH 
 *         has completed access to table memory. This call will always
 *         be made following a call to Osal_BMET_ETH BeginMemAccess and have
 *         the same parameters
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      void Osal_bmet_eth_EndMemAccess (void* addr, uint32_t sizeWords)
 *  @endverbatim
 *
 *  <b> Parameters </b>
 *  @n addr      - The address of the table to be accessed
 *  @n sizeWords - The number of bytes in the table
 *
 *  @note BMET_ETH  will make nested calls to this function for memory access
 *        protection of different memory tables. The multicore semaphore
 *        should be freed when all previous memory access has completed,
 *        in other words, when the nested call level reaches 0.
 */
 
#define BMET_ETH_osalEndMemAccess   Osal_bmet_eth_EndMemAccess

/**
 * @brief  This macro is used to alert the application that the BMET_ETH 
 *         has encountered an exception
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      void Osal_bmet_eth_Exception (uint32_t moduleID, int32_t exception_num)
 *  @endverbatim
 *
 *  <b> Parameters </b>
 *  @n addr      - The address of the table to be accessed
 *  @n sizeWords - The number of bytes in the table
 *
 *  @note BMET_ETH  will make nested calls to this function for memory access
 *        protection of different memory tables. The multicore semaphore
 *        should be freed when all previous memory access has completed,
 *        in other words, when the nested call level reaches 0.
 */
 
#define BMET_ETH_osalException   Osal_bmet_eth_Exception

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _BMET_ETH_OSAL_H */

