/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *   @file  IOLINK_osal.h
 *
 *   @brief
 *      This is the sample OS Adaptation layer which is used by the IOLINK
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
 *           - Rebuild the IOLINK Driver library; ensure that the Include
 *             path points to the directory where the copy of this file
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - IOLINK Libraries need to be rebuilt by the customer.
 *
 */

#ifndef IOLINK_OSAL_H
#define IOLINK_OSAL_H

/* include TI OSAL library interface header files */
#include <ti/osal/osal.h>
#include <ti/osal/SwiP.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   The macro is used by the IOLINK driver to create a semaphore for
 * each IOLINK  opened in blocking mode. Semaphores created should
 * *initially* be created with a count of 0 i.e. unavailable.
 *
*  @param  count  Initial count of the semaphore. For binary semaphores,
 *                 only values of 0 or 1 are valid.
 *
 *  @param  params  Pointer to the instance configuration parameters. NULL
 *                  denotes to use the default parameters (SemOSAL default
 *                  parameters as noted in ::SemOSAL_Params.
 *
 *  @return A void* on success or a NULL on an error
 */

#define IOLINK_osalCreateBlockingLock(count,params) (SemaphoreP_create((count),(params)))

/**
 * @brief   The macro is used by the IOLINK driver to delete a previously
 * created semaphore. This is called when a IOLINK  opened in blocking mode
 * is being closed.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void SemOSAL_delete(void* semHandle)
    @endverbatim

 *  <b> Parameter </b>
 *  @n  Semaphore Handle returned by the create semaphore
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define IOLINK_osalDeleteBlockingLock(X) (SemaphoreP_delete(X))


/*!
 *  @brief  Initialize params structure to default values.
 *
 *  The default parameters are:
 *   - mode: SemOSAL_Mode_COUNTING
 *   - name: NULL
 *
 *  @param params  Pointer to the instance configuration parameters.
 */
#define IOLINK_osalSemParamsInit(params) (SemaphoreP_Params_init(params))


/**
 * @brief   The macro is used by the IOLINK driver to pend on a semaphore
 * This is called when an application tries to receive data on a blocking
 *  when there is no data available. Since all semaphores are initially
 * created to be unavailable; this will cause the application to block.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void SemOSAL_pend(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define IOLINK_osalPendLock(X,Y) (SemaphoreP_pend((X),(Y)))

/**
 * @brief   The macro is used by the IOLINK driver to post the semaphore
 * The driver posts the semaphore once data is received on a specific
 *
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void SemOSAL_post(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define IOLINK_osalPostLock(X) (SemaphoreP_post(X))

/**
 * @brief   Function to disable interrupts to enter a critical region
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void HwiOSAL_enterCritical()
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not Applicable
 *
 *  <b> Return Value </b>
 *  @n  Interrupt key
 */
#define IOLINK_osalHardwareIntDisable() (HwiP_disable())

/**
 * @brief   Function to restore interrupts to exit a critical region
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void HwiOSAL_exitCritical(int key)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Interrupt key
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define IOLINK_osalHardwareIntRestore(X) (HwiP_restore(X))

/**
 * @brief  Function to deregister a interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       int HwiOSAL_deregisterInterrupt(void* hwiPtr)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Interrupt handler
 *
 *  <b> Return Value </b>
 *  @n  status of operation
 */
#define IOLINK_osalHardwareIntDestruct(X,Y) (Osal_DeleteInterrupt((X),(Y)))

/**
 * @brief  Function to register an interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       HwiP_Handle HwiP_create(X, Y, Z);
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Interrupt number
 *
 *  <b> Parameter </b>
 *  @n  hwiFxn entry function of the hardware interrupt
 *
 *  <b> Parameter </b>
 *  @n  params  argument passed into the entry function
 *
 *  <b> Return Value </b>
 *  @n  Interrupt handle
 */
#define IOLINK_osalRegisterInterrupt(X,Y) (Osal_RegisterInterrupt((X),(Y)))

/**
 * @brief  Function to initialize hardware interrupt parameters.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void HwiP_Params_init(HwiP_Params *params)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Parameters   Pointer to the hardware interrupt's parameter structure.
 *
 */
#define IOLINK_osalHwiParamsInit(X) (HwiP_Params_init(X))

/**
 * @brief  Function to disable specific hardware interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void HwiOSAL_disableInterrupt(unsigned int intrNum)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Interrupt number
 *
 */
#define IOLINK_osalHardwareIntrDisable(X,Y) (Osal_DisableInterrupt((X),(Y)))

/**
 * @brief  Function to enable specific hardware interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void HwiOSAL_enableInterrupt(unsigned int intrNum)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Interrupt number
 *
 */
#define IOLINK_osalHardwareIntrEnable(X,Y) (Osal_EnableInterrupt((X),(Y)))

/**
 * @brief  Function to clear specific hardware interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void IOLINK_osalHardwareIntrClear(unsigned int intrNum)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Interrupt number
 *
 */
#define IOLINK_osalHardwareIntrClear(X,Y) (Osal_ClearInterrupt((X),(Y)))

/**
 * @brief  Function to initialize software interrupt parameters.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void SwiP_Params_init(SwiP_Params *params)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Parameters   Pointer to the software interrupt's parameter structure.
 *
 */
#define IOLINK_osalSwiParamsInit(X) (HwiP_Params_init(X))

/**
 * @brief  Function to create a software interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       SwiP_Handle SwiP_create(X, Y);
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  swiFxn entry function of the software interrupt
 *
 *  <b> Parameter </b>
 *  @n  params  argument passed into the entry function
 *
 *  <b> Return Value </b>
 *  @n  Software interrupt handle
 */
#define IOLINK_osalSwiCreate(X,Y) (SwiP_create((X),(Y)))

/**
 * @brief  Function to destruct a software interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       int SwiP_delete(void* hwiPtr)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Software interrupt handler
 *
 *  <b> Return Value </b>
 *  @n  status of operation
 */
#define IOLINK_osalSoftwareIntDestruct(X) (SwiP_delete(X))

/**
 * @brief  Function to post a software interrupt
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       int SwiP_delete(void* hwiPtr)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Software interrupt handler
 *
 *  <b> Return Value </b>
 *  @n  status of operation
 */
#define IOLINK_osalSoftwareIntPost(X) (SwiP_post(X))

#ifdef __cplusplus
}
#endif

#endif /* __IOLINK_OSAL_H__ */

