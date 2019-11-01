/**
 *   @file  srio_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the SRIO
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
 *           - Rebuild the SRIO Driver library; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this 
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - SRIO Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009-2012 Texas Instruments, Inc.
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
#ifndef __SRIO_OSAL_H__
#define __SRIO_OSAL_H__

#include <ti/drv/srio/srio_drv.h>

/** @addtogroup SRIO_OSAL_API
 @{ */

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* #include <string.h> is here because there used to be 
 * memcpy/memset prototypes here.  This #include prevents warnings in 
 * other code that unintentionally worked because of these prototypes
 */
#include <string.h>

extern void* Osal_srioMalloc(uint32_t numBytes);
extern void  Osal_srioFree (void* ptr, uint32_t size);
extern void* Osal_srioDataBufferMalloc(uint32_t numBytes);
extern void  Osal_srioDataBufferFree(void* ptr, uint32_t numBytes);
extern void  Osal_srioLog(char* fmt, ... );
extern void* Osal_srioCreateSem(void);
extern void  Osal_srioDeleteSem(void* semHandle);
extern void  Osal_srioPendSem(void* semHandle);
extern void  Osal_srioPostSem(void* semHandle);
extern void* Osal_srioEnterMultipleCoreCriticalSection(void);
extern void  Osal_srioExitMultipleCoreCriticalSection(void* critSectHandle);
extern void* Osal_srioEnterSingleCoreCriticalSection(Srio_DrvHandle drvHandle);
extern void  Osal_srioExitSingleCoreCriticalSection(Srio_DrvHandle drvHandle, void* critSectHandle);
extern void  Osal_srioBeginMemAccess(void* ptr, uint32_t size);
extern void  Osal_srioEndMemAccess(void* ptr, uint32_t size);
extern void  Osal_srioBeginDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);
extern void  Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);

/**
 * @brief   The macro is used by the SRIO driver to create a semaphore for 
 * each SRIO socket opened in blocking mode. Semaphores created should 
 * *initially* be created with a count of 0 i.e. unavailable. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_srioCreateSem(void);
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n Not Applicable
 *
 *  <b> Return Value </b>
 *  @n Semaphore Handle
 */
#define Srio_osalCreateSem()        Osal_srioCreateSem()

/**
 * @brief   The macro is used by the SRIO driver to delete a previously 
 * created semaphore. This is called when a SRIO socket opened in blocking mode
 * is being closed.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_srioDeleteSem(void* semHandle)
    @endverbatim

 *  <b> Parameter </b>
 *  @n  Semaphore Handle returned by the create semaphore
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Srio_osalDeleteSem(X)       Osal_srioDeleteSem(X)

/**
 * @brief   The macro is used by the SRIO driver to pend on a semaphore
 * This is called when an application tries to receive data on a blocking
 * socket when there is no data available. Since all semaphores are initially
 * created to be unavailable; this will cause the application to block.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_srioPendSem(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Srio_osalPendSem(X)         Osal_srioPendSem(X)

/**
 * @brief   The macro is used by the SRIO driver to post the semaphore
 * The driver posts the semaphore once data is received on a specific 
 * socket.  
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_srioPostSem(void* semHandle)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Srio_osalPostSem(X)     Osal_srioPostSem(X)

/**
 * @brief   The macro is used by the SRIO driver to allocate memory
 * The SRIO driver uses this macro to allocate memory for its internal 
 * driver structures. This is invoked during the driver initialization
 * and startup process.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_srioMalloc(uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Number of bytes to be allocated
 *
 *  <b> Return Value </b>
 *  @n  Pointer to the allocated block size
 *
 *  @sa
 *      Srio_osalDataBufferMalloc
 */
#define Srio_osalMalloc(X)      Osal_srioMalloc(X)

/**
 * @brief   The macro is used by the SRIO driver to free a previously
 * allocated block of memory
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_srioFree(void* ptr, uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Pointer to the block of memory to be cleaned up.
 *  @n  Size of the allocated memory which is being freed.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Srio_osalFree(X, Y)     Osal_srioFree(X, Y)

/**
 * @brief   The macro is used by the SRIO driver to allocate memory
 * for the data buffers in Driver Managed Configuration. All data
 * buffers should allocated should be in the global address space. 
 * This macro is invoked during the data path.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_srioDataBufferMalloc(uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Number of bytes to be allocated
 *
 *  <b> Return Value </b>
 *  @n  Pointer to the allocated block size
 */
#define Srio_osalDataBufferMalloc(X)      Osal_srioDataBufferMalloc(X)

/**
 * @brief   The macro is used by the SRIO driver to free a previously
 * allocated block data buffer. This macro is used to clean up previously
 * allocated data buffers and is invoked during the data path.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_srioDataBufferFree(void* ptr, uint32_t numBytes)
    @endverbatim
 *      
 *  <b> Parameter </b>
 *  @n  Pointer to the block of memory to be cleaned up.
 *  @n  Size of the allocated memory which is being freed.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Srio_osalDataBufferFree(X, Y)     Osal_srioDataBufferFree(X, Y)

/**
 * @brief   The macro is used by the SRIO driver to log various 
 * messages. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_srioLog( char* fmt, ... ) 
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string 
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Srio_osalLog            Osal_srioLog

/**
 * @brief   The macro is used by the SRIO Driver to protect its shared resources
 * access from MULTIPLE CORES. This is required if the SRIO Driver API's are being
 * invoked from multiple cores. If this is not the case then these macros can be
 * defined to be NOP.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_srioEnterMultipleCoreCriticalSection(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  None
 *
 *  <b> Return Value </b>
 *  @n  Opaque Handle used for holding critical section locking information
 */
#define Srio_osalEnterMultipleCoreCriticalSection      Osal_srioEnterMultipleCoreCriticalSection

/**
 * @brief   The macro is used by the SRIO driver to end the protection of its 
 * internal shared "resources" from MULTIPLE CORE access.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_srioExitMultipleCoreCriticalSection(void* critSectHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Opaque Handle used for holding critical section locking information
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Srio_osalExitMultipleCoreCriticalSection      Osal_srioExitMultipleCoreCriticalSection

/**
 * @brief   The macro is used by the SRIO driver to protect its internal shared
 * resources from SINGLE CORE MULTIPLE CONTEXT (thread or ISR) access. If all 
 * the SRIO Driver APIs are being called from threads then this API could 
 * use semaphores. However if the SRIO driver API's are being called from 
 * both ISR & Thread context then the critical section here would need to 
 * disable/enable interrupts. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_srioEnterSingleCoreCriticalSection(Srio_DrvHandle drvHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  drvHandle      - Driver Handle for which the critical section is being entered.
 *
 *  <b> Return Value </b>
 *  @n  Opaque Handle used for holding critical section locking information
 */
#define Srio_osalEnterSingleCoreCriticalSection     Osal_srioEnterSingleCoreCriticalSection

/**
 * @brief   The macro is used to denote the end of the protection of the internal
 * shared resource from SINGLE CORE MULTIPLE CONTEXT access.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_srioExitSingleCoreCriticalSection(Srio_DrvHandle drvHandle, void* critSectHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  drvHandle      - Driver Handle for which the critical section is being exited.
 *  @n  critSectHandle - Opaque Handle used for holding critical section locking information
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Srio_osalExitSingleCoreCriticalSection      Osal_srioExitSingleCoreCriticalSection

/**
 * @brief   The macro is used by the SRIO driver to indicate that a block
 * of memory is about to be accessed. If the memory block is cached then
 * this indicates that the application would need to ensure that the cache
 * is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_srioBeginMemAccess(void* ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  ptr  - Pointer to the memory
 *  @n  size - Size of the memory being accessed.
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Srio_osalBeginMemAccess      Osal_srioBeginMemAccess

/**
 * @brief   The macro is used by the SRIO driver to indicate that  the block of 
 * memory has finished being accessed. If the memory block is cached then the 
 * application would need to ensure that the contents of the cache are updated
 * immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_srioEndMemAccess(void* ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  ptr  - Pointer to the memory 
 *  @n  size - Size of the memory
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Srio_osalEndMemAccess       Osal_srioEndMemAccess

/**
 * @brief   The macro is used by the SRIO driver to indicate that the driver
 * is about to start accessing a descriptor. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_srioBeginDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  drvHandle   - Driver Handle for which the descriptor is being accessed.
 *  @n  ptr         - Address of the descriptor which is being accessed.
 *  @n  size        - Size of the descriptor 
 *  @n                (Only Applicable for Driver Managed is 0 for App Managed)
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Srio_osalBeginDescriptorAccess      Osal_srioBeginDescriptorAccess

/**
 * @brief   The macro is used by the SRIO driver to indicate that the driver
 * is finished populating the descriptor. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  drvHandle   - Driver Handle for which the descriptor is being accessed.
 *  @n  ptr         - Address of the descriptor which is being accessed.
 *  @n  size        - Size of the descriptor 
 *  @n                (Only Applicable for Driver Managed is 0 for App Managed)
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Srio_osalEndDescriptorAccess      Osal_srioEndDescriptorAccess

/**
@}
*/

#endif /* __SRIO_OSAL_H__ */

