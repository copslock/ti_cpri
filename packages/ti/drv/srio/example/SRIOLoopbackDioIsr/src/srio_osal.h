/**
 *   @file  srio_osal.h
 *
 *   @brief   
 *      This is the OS adaptation layer for the SRIO Driver which has 
 *      been ported for XDC Runtime and BIOS.
 *
 *      This is an example of <b> Approach 2 </b> in which the 
 *      application rebuilds the SRIO driver with the new definitions.
 *
 *      Please refer to the 'srio_osal.h' in the API documentation.
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
 *  \par
*/
#ifndef __SRIO_OSAL_H__
#define __SRIO_OSAL_H__

#include <string.h>
#include <xdc/runtime/System.h>
// #include <ti/sysbios/ipc/Semaphore.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drv/srio/srio_drv.h>

/* Memory Allocation OSAL Extern Definitions. */
extern Void* biosMalloc (UInt32 num_bytes);
extern Void  biosFree (Void* ptr, UInt32 numBytes);
extern void* Osal_DataBufferMalloc(uint32_t numBytes);
extern void  Osal_DataBufferFree(void* ptr, uint32_t numBytes);

/* Multicore Protection OSAL Extern Definitions. */
extern Void* Osal_MultiCoreEnter(Void);
extern Void  Osal_MultiCoreExit(Void* critSectHandle);

/* Single Core Protection OSAL Extern Definitions. */
extern Void* Osal_SingleCoreEnter(Srio_DrvHandle drvHandle);
extern Void  Osal_SingleCoreExit(Srio_DrvHandle drvHandle, Void* critSectHandle);

/* Cache OSAL Extern Definitions. */
extern void  Osal_CacheInvalidate(void* ptr, uint32_t size);
extern void  Osal_CacheWriteback(void* ptr, uint32_t size);

/* Descriptor OSAL Extern Definitions. */
extern void  Osal_srioBeginDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);
extern void  Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);

/* SRIO OSAL Semaphore API are mapped directly to the BIOS API */
#define Srio_osalCreateSem()                        (Void*)Semaphore_create(0, NULL, NULL)
#define Srio_osalDeleteSem(X)                       Semaphore_delete(X)
#define Srio_osalPendSem(X)                         Semaphore_pend(X, BIOS_WAIT_FOREVER);
#define Srio_osalPostSem(X)                         Semaphore_post(X);

/* SRIO OSAL Memory Allocation API are redefined to TEST application API */
#define Srio_osalMalloc                             biosMalloc
#define Srio_osalFree                               biosFree

/* SRIO OSAL Memory Allocation API for the Data Buffer */
#define Srio_osalDataBufferMalloc                   Osal_DataBufferMalloc
#define Srio_osalDataBufferFree                     Osal_DataBufferFree

/* SRIO OSAL Logging API is mapped directly to an XDC Runtime API */
#define Srio_osalLog                                System_printf

/* Multicore Protection: */
#define Srio_osalEnterMultipleCoreCriticalSection   Osal_MultiCoreEnter
#define Srio_osalExitMultipleCoreCriticalSection    Osal_MultiCoreExit

/* Singlecore Protection: */
#define Srio_osalEnterSingleCoreCriticalSection     Osal_SingleCoreEnter
#define Srio_osalExitSingleCoreCriticalSection      Osal_SingleCoreExit

/* CACHE API: */
#define Srio_osalBeginMemAccess                     Osal_CacheInvalidate
#define Srio_osalEndMemAccess                       Osal_CacheWriteback

/* CACHE Descriptor API: */
#define Srio_osalBeginDescriptorAccess              Osal_srioBeginDescriptorAccess
#define Srio_osalEndDescriptorAccess                Osal_srioEndDescriptorAccess

#endif /* __SRIO_OSAL_H__ */

