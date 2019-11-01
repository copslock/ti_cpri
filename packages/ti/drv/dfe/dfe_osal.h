/********************************************************************
 * Copyright (C) 2013 Texas Instruments Incorporated.
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
#ifndef __DFE_OSAL_H__
#define __DFE_OSAL_H__

#ifdef __cplusplus
extern "C" {
#endif

extern void Osal_dfeLog (char *fmt, ... );
extern void Osal_dfeSleep (unsigned int ms);

/**
 * @brief   The macro is used by the DFE LLD to log various
 * messages.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_dfeLog( char *fmt, ... )
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Dfe_osalLog                Osal_dfeLog

/**
 * @brief   The macro is used by the DFE LLD to implement
 * a time delay during the initialization of DFE TX and RX HW
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_iqn2Sleep()
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  number of milliseconds to sleep
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Dfe_osalSleep                Osal_dfeSleep

#ifdef __cplusplus
}
#endif

#endif /* __DFE_OSAL_H__ */
