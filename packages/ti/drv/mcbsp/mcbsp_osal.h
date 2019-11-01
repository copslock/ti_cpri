/**
 *   @file  mcbsp_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the MCBSP
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
 *           - Rebuild the MCBSP Driver library; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this 
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - MCBSP Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
 *  \par
*/
#ifndef __MCBSP_OSAL_H__
#define __MCBSP_OSAL_H__

#include <string.h>
#include <ti/drv/mcbsp/mcbsp_drv.h>

/** @addtogroup MCBSP_OSAL_API
 @{ */

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

extern void* Osal_mcbspMalloc(uint32_t numBytes);
extern void  Osal_mcbspFree (void* ptr, uint32_t size);
extern void* Osal_mcbspDataBufferMalloc(uint32_t numBytes);
extern void  Osal_mcbspDataBufferFree(void* ptr, uint32_t numBytes);
extern void  Osal_mcbspLog(char* fmt, ... );
extern void* Osal_mcbspCreateSem(void);
extern void  Osal_mcbspDeleteSem(void* semHandle);
extern void  Osal_mcbspPendSem(void* semHandle);
extern void  Osal_mcbspPostSem(void* semHandle);
extern void* Osal_mcbspEnterMultipleCoreCriticalSection(void);
extern void  Osal_mcbspExitMultipleCoreCriticalSection(void* critSectHandle);
extern void* Osal_mcbspEnterSingleCoreCriticalSection();
extern void  Osal_mcbspExitSingleCoreCriticalSection(void* critSectHandle);
extern void  Osal_mcbspBeginMemAccess(void* ptr, uint32_t size);
extern void  Osal_mcbspEndMemAccess(void* ptr, uint32_t size);

extern Bool  Osal_mcbspQueueEmpty(void* handle);
extern void* Osal_mcbspQueueGet(void* handle);
extern void  Osal_mcbspQueuePut(void* handle, Mcbsp_QueueElem* elem);
extern void  Osal_mcbspWaitNBitClocks(uint32_t nticks);
extern void *Osal_local2Global (void *addr);
/**
 * @brief   The macro is used by the MCBSP driver to create a semaphore for 
 * each MCBSP socket opened in blocking mode. Semaphores created should 
 * *initially* be created with a count of 0 i.e. unavailable. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_mcbspCreateSem(void);
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n Not Applicable
 *
 *  <b> Return Value </b>
 *  @n Semaphore Handle
 */
#define Mcbsp_osalCreateSem()        Osal_mcbspCreateSem()

/**
 * @brief   The macro is used by the MCBSP driver to delete a previously 
 * created semaphore. This is called when a MCBSP socket opened in blocking mode
 * is being closed.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspDeleteSem(void* semHandle)
    @endverbatim

 *  <b> Parameter </b>
 *  @n  Semaphore Handle returned by the create semaphore
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Mcbsp_osalDeleteSem(X)       Osal_mcbspDeleteSem(X)

/**
 * @brief   The macro is used by the MCBSP driver to pend on a semaphore
 * This is called when an application tries to receive data on a blocking
 * socket when there is no data available. Since all semaphores are initially
 * created to be unavailable; this will cause the application to block.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspPendSem(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Mcbsp_osalPendSem(X)         Osal_mcbspPendSem(X)

/**
 * @brief   The macro is used by the MCBSP driver to post the semaphore
 * The driver posts the semaphore once data is received on a specific 
 * socket.  
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspPostSem(void* semHandle)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Mcbsp_osalPostSem(X)     Osal_mcbspPostSem(X)

/**
 * @brief   The macro is used by the MCBSP driver to allocate memory
 * The MCBSP driver uses this macro to allocate memory for its internal 
 * driver structures. This is invoked during the driver initialization
 * and startup process.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_mcbspMalloc(uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Number of bytes to be allocated
 *
 *  <b> Return Value </b>
 *  @n  Pointer to the allocated block size
 *
 *  @sa
 *      Mcbsp_osalDataBufferMalloc
 */
#define Mcbsp_osalMalloc(X)      Osal_mcbspMalloc(X)

/**
 * @brief   The macro is used by the MCBSP driver to free a previously
 * allocated block of memory
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspFree(void* ptr, uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Pointer to the block of memory to be cleaned up.
 *  @n  Size of the allocated memory which is being freed.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Mcbsp_osalFree(X, Y)     Osal_mcbspFree(X, Y)

/**
 * @brief   The macro is used by the MCBSP driver to allocate memory
 * for the data buffers in Driver Managed Configuration. All data
 * buffers should allocated should be in the global address space. 
 * This macro is invoked during the data path.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_mcbspDataBufferMalloc(uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Number of bytes to be allocated
 *
 *  <b> Return Value </b>
 *  @n  Pointer to the allocated block size
 */
#define Mcbsp_osalDataBufferMalloc(X)      Osal_mcbspDataBufferMalloc(X)

/**
 * @brief   The macro is used by the MCBSP driver to convert local address
 * to Global address space
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void *Osal_local2Global (void *addr)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Local address pointer
 *
 *  <b> Return Value </b>
 *  @n  Global address pointer
 */

#define Mcbsp_osalLocal2Global(X) Osal_local2Global(X)
/**
 * @brief   The macro is used by the MCBSP driver to free a previously
 * allocated block data buffer. This macro is used to clean up previously
 * allocated data buffers and is invoked during the data path.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspDataBufferFree(void* ptr, uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Pointer to the block of memory to be cleaned up.
 *  @n  Size of the allocated memory which is being freed.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Mcbsp_osalDataBufferFree(X, Y)     Osal_mcbspDataBufferFree(X, Y)

/**
 * @brief   The macro is used by the MCBSP driver to log various 
 * messages. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspLog( char* fmt, ... ) 
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string 
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Mcbsp_osalLog            Osal_mcbspLog

/**
 * @brief   The macro is used by the MCBSP Driver to protect its shared resources
 * access from MULTIPLE CORES. This is required if the MCBSP Driver API's are being
 * invoked from multiple cores. If this is not the case then these macros can be
 * defined to be NOP.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_mcbspEnterMultipleCoreCriticalSection(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  None
 *
 *  <b> Return Value </b>
 *  @n  Opaque Handle used for holding critical section locking information
 */
#define Mcbsp_osalEnterMultipleCoreCriticalSection      Osal_mcbspEnterMultipleCoreCriticalSection

/**
 * @brief   The macro is used by the MCBSP driver to end the protection of its 
 * internal shared "resources" from MULTIPLE CORE access.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_mcbspExitMultipleCoreCriticalSection(void* critSectHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Opaque Handle used for holding critical section locking information
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Mcbsp_osalExitMultipleCoreCriticalSection      Osal_mcbspExitMultipleCoreCriticalSection

/**
 * @brief   The macro is used by the MCBSP driver to protect its internal shared
 * resources from SINGLE CORE MULTIPLE CONTEXT (thread or ISR) access. If all 
 * the MCBSP Driver APIs are being called from threads then this API could 
 * use semaphores. However if the MCBSP driver API's are being called from 
 * both ISR & Thread context then the critical section here would need to 
 * disable/enable interrupts. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_mcbspEnterSingleCoreCriticalSection()
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  None
 *
 *  <b> Return Value </b>
 *  @n  Opaque Handle used for holding critical section locking information
 */
#define Mcbsp_osalEnterSingleCoreCriticalSection     Osal_mcbspEnterSingleCoreCriticalSection

/**
 * @brief   The macro is used to denote the end of the protection of the internal
 * shared resource from SINGLE CORE MULTIPLE CONTEXT access.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_mcbspExitSingleCoreCriticalSection(void* critSectHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  critSectHandle - Opaque Handle used for holding critical section locking information
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Mcbsp_osalExitSingleCoreCriticalSection      Osal_mcbspExitSingleCoreCriticalSection

/**
 * @brief   The macro is used by the MCBSP driver to indicate that a block
 * of memory is about to be accessed. If the memory block is cached then
 * this indicates that the application would need to ensure that the cache
 * is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_mcbspBeginMemAccess(void* ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  ptr  - Pointer to the memory
 *  @n  size - Size of the memory being accessed.
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Mcbsp_osalBeginMemAccess      Osal_mcbspBeginMemAccess

/**
 * @brief   The macro is used by the MCBSP driver to indicate that  the block of 
 * memory has finished being accessed. If the memory block is cached then the 
 * application would need to ensure that the contents of the cache are updated
 * immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_mcbspEndMemAccess(void* ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  ptr  - Pointer to the memory 
 *  @n  size - Size of the memory
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Mcbsp_osalEndMemAccess       Osal_mcbspEndMemAccess

/**
 * @brief   The macro is used by the MCBSP driver to test for an 
 *          empty queue.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_mcbspQueueEmpty(void* handle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  handle - Handle of a previously created Queue instance object 
 *
 *  <b> Return Value </b>
 *  @n  TRUE - If the queue is empty
 */
#define Mcbsp_osalQueueEmpty         Osal_mcbspQueueEmpty

/**
 * @brief   The macro is used by the MCBSP driver to get an 
 *          element from the front of queue. The function 
 *          removes the element from the front of queue and
 *          returns a pointer to it.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void*  Osal_mcbspQueueGet(void* handle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  handle - Handle of a previously created Queue instance object 
 *
 *  <b> Return Value </b>
 *  @n  Handle (pointer) to former first element
 */
#define Mcbsp_osalQueueGet           Osal_mcbspQueueGet

/**
 * @brief   The macro is used by the MCBSP driver to put an 
 *          element at the end of queue. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspQueuePut(void* handle, Mcbsp_QueueElem* elem)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  handle - Handle of a previously created Queue instance object 
 *  @n  elem - Pointer to new queue element 
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Mcbsp_osalQueuePut           Osal_mcbspQueuePut

/**
 * @brief   The macro is used by the MCBSP driver to wait 'n' 
 *          bit clocks to ensure proper synchronization internally. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_mcbspWaitNBitClocks(uint32_t nticks)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  nticks - Number of bit clocks to wait 
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Mcbsp_osalWaitNBitClocks     Osal_mcbspWaitNBitClocks
/**
@}
*/

#endif /* __MCBSP_OSAL_H__ */

