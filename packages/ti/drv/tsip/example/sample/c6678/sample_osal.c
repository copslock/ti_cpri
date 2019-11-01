/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
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
 *   @file  sample_osal.c
 *
 *   @brief   
 *      This is the OS abstraction layer and is used by the TSIP
 *      low level drivers for the sample example.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ============================================================================ 
 *  \par
 */

#include <ti/sysbios/hal/Hwi.h>


/**
 * \brief   TSIP OS Protect Entry
 *
 *      This function saves the current state of protection in 'intState'
 *      variable passed by caller, and also applies the interrupt protection
 *
 * \param   intState is memory location where current state of protection is
 *      saved for future use while restoring it via Osal_tsipExitCriticalSection().
 * \return  None
 */
void Osal_tsipEnterCriticalSection(unsigned int *intState)
{
    if (intState == NULL) 
        return;

    *intState = Hwi_disable();
}

/**
 * \brief   TSIP OS Protect Exit
 *
 *      This function undoes the protection enforced to original state
 *      as is specified by the variable 'intState' passed.
 *
 * \param   intState is original state of protection at time when the
 *      corresponding Osal_tsipEnterCriticalSection() was called.
 * \return  None
 */

void Osal_tsipExitCriticalSection(unsigned int intState)
{
    Hwi_restore(intState);
}

