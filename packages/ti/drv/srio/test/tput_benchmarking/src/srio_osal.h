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

/* Descriptor OSAL Extern Definitions. */
extern void  Osal_srioBeginDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);
extern void  Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);

/* Cache OSAL Extern Definitions. */
extern void  Osal_CacheInvalidate(void* ptr, uint32_t size);
extern void  Osal_CacheWriteback(void* ptr, uint32_t size);

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

