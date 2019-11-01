/**
 *   @file  bcp_osal.h
 *
 *   @brief   
 *      OS Adaptation Layer (OSAL) header file for BCP Test application. 
 *      Contains the BCP OSAL API prototype definitions.
 *
 *      This is a sample OSAL file ported to XDC/BIOS for demonstration
 *      purposes only. This would need to be ported by System integrator
 *      to a suitable OS and implemented accordingly.
 *
 *      Uses <b> Approach 2 </b> for OSAL implementation, i.e, the 
 *      BCP library sources would have to be re-compiled with the 
 *      new OSAL implementations.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
#ifndef _BCP_OSAL_H_
#define _BCP_OSAL_H_

/* BCP Test application types include */
#include <bcp_types.h>

/* XDC/BIOS include */
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

extern Void*    Osal_biosMalloc (UInt32 num_bytes, Bool bGlobalAddress);
extern Void     Osal_biosFree (Void* dataPtr, UInt32 num_bytes, Bool bGlobalAddress);
extern Void     Osal_biosMultiCoreCsEnter ();
extern Void     Osal_biosMultiCoreCsExit ();
extern Void     Osal_biosInterruptCsEnter ();
extern Void     Osal_biosInterruptCsExit ();
extern Void     Osal_bcpBeginMemAccess (Void* pBlockPtr,UInt32 byteCnt);
extern Void     Osal_bcpEndMemAccess (Void* pBlockPtr,UInt32 byteCnt);
extern Void     Osal_bcpBeginDescMemAccess (Void* hRx, Void* pBlockPtr);

/* Map out all BCP OSAL APIs to BIOS implementations */
#define     Bcp_osalMalloc             Osal_biosMalloc
#define     Bcp_osalFree               Osal_biosFree
#define     Bcp_osalMultiCoreCsEnter   Osal_biosMultiCoreCsEnter
#define     Bcp_osalMultiCoreCsExit    Osal_biosMultiCoreCsExit
#define     Bcp_osalInterruptCsEnter   Osal_biosInterruptCsEnter
#define     Bcp_osalInterruptCsExit    Osal_biosInterruptCsExit
#define     Bcp_osalLog                System_printf
#define     Bcp_osalBeginMemAccess     Osal_bcpBeginMemAccess
#define     Bcp_osalEndMemAccess       Osal_bcpEndMemAccess
#define     Bcp_osalBeginDescMemAccess Osal_bcpBeginDescMemAccess

/* Map out all BCP Semaphore APIs to corresponding BIOS implementations */
#define     Bcp_osalCreateSem()        (Void *) Semaphore_create (0, NULL, NULL)
#define     Bcp_osalDeleteSem(X)       Semaphore_delete ((Semaphore_Handle *)&X)
#define     Bcp_osalPendSem(X)         Semaphore_pend (X, BIOS_WAIT_FOREVER)
#define     Bcp_osalPostSem(X)         Semaphore_post (X)

#endif /* _BCP_OSAL_H_ */
