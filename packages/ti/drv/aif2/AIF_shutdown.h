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
 * @file AIF_shutdown.h
 *
 * @brief Header file for AIF H/W cleanup
 * 
*/


/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */


#ifndef __AIF_SHUTDOWN_H
#define __AIF_SHUTDOWN_H


#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b AIF_resetFsync
 *
 *   @b Description
 *   @n This function resets the AIF2 AT event module
 *
 *   @b Arguments
 *   @verbatim
        hAif   Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  The AIF2 H/W timers are halted and disabled.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_resetFsync(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_SHUTDOWN_C
extern
#endif
void 
AIF_resetFsync(
    AIF_ConfigHandle    hAif
);

/** 
 *   @n@b AIF_resetAif
 *
 *   @b Description
 *   @n This function is used to reset AIF to its default state.
 *
 *   @b Arguments
 *   @verbatim
        hAif   Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  The AIF H/W is now in reset state.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_resetAif(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_SHUTDOWN_C
extern
#endif
void AIF_resetAif(
    AIF_ConfigHandle    hAif
);

/**
 *   @n@b AIF_disablePeCh
 *
 *   @b Description
 *   @n This function is used to disable a given PE channel.
 *
 *   @b Arguments
 *   @verbatim
        hAif   		Pointer to a AIF2_ConfigObj instance.
        peChanNum	Channel index number
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  Transmission of this Antenna Container is stopped.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            uint32_t              peChanNum;
            AIF_disablePeCh(hAif, peChanNum);
     @endverbatim
 *
 */
#ifndef __AIF_SHUTDOWN_C
extern
#endif
void AIF_disablePeCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          peChanNum
);

/**
 *   @n@b AIF_disablePdCh
 *
 *   @b Description
 *   @n This function is used to disable a given PD channel.
 *
 *   @b Arguments
 *   @verbatim
        hAif   		Pointer to a AIF2_ConfigObj instance.
        peChanNum	Channel index number
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  Reception of this Antenna Container is stopped.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            uint32_t              pdChanNum;
            AIF_disablePdCh(hAif, pdChanNum);
     @endverbatim
 *
 */
#ifndef __AIF_SHUTDOWN_C
extern
#endif
void AIF_disablePdCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          pdChanNum
);

#ifdef __cplusplus
}
#endif

/**
 *   @n@b AIF_enablePeCh
 *
 *   @b Description
 *   @n This function is used to enable a given PE channel.
 *
 *   @b Arguments
 *   @verbatim
        hAif   		Pointer to a AIF2_ConfigObj instance.
        peChanNum	Channel index number
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  Transmission of this Antenna Container is stopped.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            uint32_t              peChanNum;
            AIF_enablePeCh(hAif, peChanNum);
     @endverbatim
 *
 */
#ifndef __AIF_SHUTDOWN_C
extern
#endif
void AIF_enablePeCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          peChanNum
);

/**
 *   @n@b AIF_enablePdCh
 *
 *   @b Description
 *   @n This function is used to enable a given PD channel.
 *
 *   @b Arguments
 *   @verbatim
        hAif   		Pointer to a AIF2_ConfigObj instance.
        pdChanNum	Channel index number
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  Transmission of this Antenna Container is stopped.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            uint32_t              pdChanNum;
            AIF_enablePdCh(hAif, pdChanNum);
     @endverbatim
 *
 */
#ifndef __AIF_SHUTDOWN_C
extern
#endif
void AIF_enablePdCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          pdChanNum
);


#endif //__AIF_SHUTDOWN_H

/** @} */ // end of module additions
