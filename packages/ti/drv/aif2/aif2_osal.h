/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
/**
 *   @file  aif2_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the AIF2 low level
 *      driver. The OSAL layer can be ported in either of the following 
 *      manners to a native OS:
 */

#ifndef __AIF2_OSAL_H__
#define __AIF2_OSAL_H__


/** @addtogroup AIF2_LLD_OSAL
 @{ */
extern void* Osal_aif2Malloc (uint32_t num_bytes);

extern void Osal_aif2Free (void *ptr, uint32_t num_bytes);

extern void Osal_aif2Log ( char *fmt, ... );

extern void Osal_aif2MulticoreSyncBarrier ();

extern void Osal_aif2ResetMulticoreSyncBarrier ();



/**
 * @brief   The macro is used by the AIF2 library to log various
 * messages.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_aif2Malloc( uint32_t num_byte )
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  dynamically allocate aifconfighandle ptr.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Aif2_osalMalloc                Osal_aif2Malloc

/**
 * @brief   The macro is used by the AIF2 library to log various
 * messages.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_aif2Log( char *fmt, ... )
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Aif2_osalFree                Osal_aif2Free

/**
 * @brief   The macro is used by the AIF2 library to log various
 * messages. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_aif2Log( char *fmt, ... )
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string 
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Aif2_osalLog                Osal_aif2Log

/**
 * @brief   This API is called from the AIF2 driver to acquire 
 * a barrier lock that will synchronize the configuration of 
 * the different core.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_aif2MulticoreSynchro(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Aif2_osalMulticoreSyncBarrier   Osal_aif2MulticoreSyncBarrier

/**
 * @brief   This API is called from the AIF2 driver to reset
 * the barrier lock that will synchronize the configuration of
 * the different core.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_aif2MulticoreSynchro(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Aif2_osalResetMulticoreSyncBarrier   Osal_aif2ResetMulticoreSyncBarrier

/**
@}
*/

#endif /* __AIF2_OSAL_H__ */

