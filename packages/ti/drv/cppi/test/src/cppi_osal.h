/**
 *   @file  cppi_osal.h
 *
 *   @brief
 *      This is the OS adaptation layer for the CPPI low level Driver. This file should
 *      be modified by the system integrator to their system/OS implementation
 *      The file is provided as an 'example' template ported for XDC/BIOS.
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
#ifndef __CPPI_OSAL_H__
#define __CPPI_OSAL_H__

extern void* Osal_biosMalloc (uint32_t num_bytes);
extern void Osal_biosFree (void *ptr, uint32_t size);
extern void* Osal_cppiCsEnter (void);
extern void Osal_cppiCsExit (void *CsHandle);
extern void Osal_cppiBeginMemAccess (void *ptr, uint32_t size);
extern void Osal_cppiEndMemAccess (void *ptr, uint32_t size);

/* CPPI LLD OSAL Memory Allocation API are redefined to TEST application API */
#define Cppi_osalMalloc             Osal_biosMalloc
#define Cppi_osalFree               Osal_biosFree

/* CPPI LLD OSAL Critical section and cache coherency APIs are used without redefinition in TEST application API */
#define Cppi_osalCsEnter            Osal_cppiCsEnter
#define Cppi_osalCsExit             Osal_cppiCsExit
#define Cppi_osalBeginMemAccess     Osal_cppiBeginMemAccess
#define Cppi_osalEndMemAccess       Osal_cppiEndMemAccess

/* CPPI LLD OSAL Logging API is mapped directly to an XDC Runtime API */
#define Cppi_osalLog                System_printf

#endif /* __CPPI_OSAL_H__ */

