/**
 *   @file  rm_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the Resource
 *      Manager. The OSAL layer can be ported in either of the following
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
 *           - Rebuild the RM low level drivver library; ensure that the Include
 *             path points to the directory where the copy of this file
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - RM LLD Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012-2015 Texas Instruments, Inc.
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
#ifndef RM_OSAL_H_
#define RM_OSAL_H_

/** @addtogroup RM_OSAL_API
 @{
 */

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

extern void *Osal_rmMalloc (uint32_t num_bytes);
extern void  Osal_rmFree (void *ptr, uint32_t size);
extern void *Osal_rmCsEnter (void);
extern void  Osal_rmCsExit (void *CsHandle);
extern void *Osal_rmMtCsEnter (void *mtSemObj);
extern void  Osal_rmMtCsExit (void *mtSemObj, void *CsHandle);
extern void  Osal_rmBeginMemAccess (void *ptr, uint32_t size);
extern void  Osal_rmEndMemAccess (void *ptr, uint32_t size);
extern void *Osal_rmTaskBlockCreate (void);
extern void  Osal_rmTaskBlock (void *handle);
extern void  Osal_rmTaskUnblock (void *handle);
extern void  Osal_rmTaskBlockDelete (void *handle);
extern void  Osal_rmLog (char *fmt, ... );

/**
 * @brief   The macro is used by RM to allocate memory of specified
 *          size
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void* Osal_rmMalloc (uint32_t numBytes)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Number of bytes to be allocated
 *
 *  <b> Return Value </b>
 *  @n  Pointer to the allocated block size
 */
#define Rm_osalMalloc             Osal_rmMalloc

/**
 * @brief   The macro is used by RM to free a allocated block of 
 *          memory 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void Osal_rmFree (void *ptr, uint32_t size)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Pointer to the block of memory to be cleaned up.
 *  @n  Size of the allocated memory which is being freed.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalFree               Osal_rmFree

/**
 * @brief   The function is used to enter a critical section.
 *          Function protects against 
 *
 *          access from multiple cores 
 *          and 
 *          access from multiple threads on single core
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void *Osal_rmCsEnter (void)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not applicable.
 *
 *  <b> Return Value </b>
 *  @n  Handle used to lock critical section
 */
#define Rm_osalCsEnter            Osal_rmCsEnter

/**
 * @brief   The function is used to exit a critical section
 *          protected using Osal_rmCsEnter() API.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void Osal_rmCsExit (void *CsHandle)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Handle for unlocking critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalCsExit             Osal_rmCsExit

/**
 * @brief   The function is used to enter a multi-threaded critical
 *          section.  Function protects against
 *
 *          access from multiple threads on single core
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void *Osal_rmMtCsEnter (void *mtSemObj)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Multi-threaded critical section object handle.
 *
 *  <b> Return Value </b>
 *  @n  Handle used to lock the multi-threaded critical section
 */
#define Rm_osalMtCsEnter          Osal_rmMtCsEnter

/**
 * @brief   The function is used to exit a multi-threaded critical
 *          section protected using Osal_rmMtCsEnter() API.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void Osal_rmMtCsExit (void *mtSemObj, void *CsHandle)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Multi-threaded critical section object handle.
 *
 *  <b> Parameter </b>
 *  @n  Handle for unlocking multi-threaded critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalMtCsExit           Osal_rmMtCsExit

/**
 * @brief   The function is used to indicate that a block of memory is 
 *          about to be accessed. If the memory block is cached then this 
 *          indicates that the application would need to ensure that the 
 *          cache is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void Osal_rmBeginMemAccess (void *ptr, uint32_t size)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block
 *  @n  Size of memory block
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalBeginMemAccess     Osal_rmBeginMemAccess

/**
 * @brief   The function is used to indicate that the block of memory has
 *          finished being accessed. If the memory block is cached then the
 *          application would need to ensure that the contents of the cache
 *          are updated immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void Osal_rmEndMemAccess (void *ptr, uint32_t size)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block
 *  @n  Size of memory block
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalEndMemAccess       Osal_rmEndMemAccess

/**
 * @brief   The macro is used by RM to create a task blocking
 *          mechanism allowing a RM instance to block when it
 *          has not been provided a service callback function
 *          and it must send a packet to remote RM instance
 *          to complete a service.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void *Osal_rmTaskBlockCreate (void)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not applicable.
 *
 *  <b> Return Value </b>
 *  @n  Task blocking mechanism handle cast as a void pointer
 */
#define Rm_osalTaskBlockCreate    Osal_rmTaskBlockCreate

/**
 * @brief   The macro is used by a RM instance to block when it
 *          has not been provided a service callback function
 *          and it must send a packet to remote RM instance
 *          to complete a service.  The blocking operation
 *          should block the current RM instance's task/thread
 *          from executing until a task/thread from which the
 *          RM receive code runs unblocks the RM instance
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void  Osal_rmTaskBlock (void *handle)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Task blocking mechanism handle cast as a void pointer
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalTaskBlock          Osal_rmTaskBlock

/**
 * @brief   The macro is used by a RM instance to unblock from
 *          a previous block operation.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void  Osal_rmTaskUnblock (void *handle)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Task blocking mechanism handle cast as a void pointer
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalTaskUnblock        Osal_rmTaskUnblock

/**
 * @brief   The macro is used by a RM instance to delete its
 *          task blocking mechanism.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void  Osal_rmTaskBlockDelete (void *handle)
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Task blocking mechanism handle cast as a void pointer
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalTaskBlockDelete    Osal_rmTaskBlockDelete

/**
 * @brief   The macro is used by RM to log various 
 *          messages.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *     void Osal_rmLog( char *fmt, ... )
 *  @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string 
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Rm_osalLog                Osal_rmLog

/**
@}
*/

#endif /* RM_OSAL_H_ */

