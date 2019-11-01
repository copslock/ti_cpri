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
 * @file AIF_debug.h
 *
 * @brief Header file for AIF H/W debug setup
 * 
*/


/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */


#ifndef __AIF_DEBUG_H
#define __AIF_DEBUG_H


#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b AIF_enableException
 *
 *   @b Description
 *   @n This function configures AIF2 HW registers to enable
 *   errors at alarms at AIF2 level.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *   
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_enableException(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_enableException(
		AIF_ConfigHandle  hAif
);

/**
 *   @n@b AIF_getException
 *
 *   @b Description
 *   @n This function checks for AIF2 HW errors at alarms at AIF2 level.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_getException(hAif);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_getException(
		AIF_ConfigHandle  hAif
);


/**
 *   @n@b AIF_resetException
 *
 *   @b Description
 *   @n This function resets AIF2LLD exception counters.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  SW counters
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_resetException(hAif);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_resetException(
		AIF_ConfigHandle  hAif
);

/**
 *   @n@b AIF_printException
 *
 *   @b Description
 *   @n This function prints out AIF2 HW errors at alarms at AIF2 level.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_printException(hAif);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_printException(
		AIF_ConfigHandle  hAif
);

/**
 *   @n@b AIF_printStatus
 *
 *   @b Description
 *   @n This function calls AIF_getException and AIF_printException.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_printStatus(hAif);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_printStatus(
		AIF_ConfigHandle  hAif
);

/**
 *   @n@b AIF_captureException
 *
 *   @b Description
 *   @n This function captures the exception counts into a supplied destination storage
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
	   capturePtr  Pointer to the AIF_EeCountObj structure to capture the exception counters
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
			AIF_EeCountObj      capture;
            AIF_captureException(hAif, &capture);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_captureException (
		AIF_ConfigHandle  hAif,
		AIF_EeCountObj *snapPtr
);

/**
 *   @n@b AIF_enableDataTrace
 *
 *   @b Description
 *   @n This function enables AIF2 data trace for OBSAI debug mode.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
       hDataTrace  Pointer to a AIF2_DataTracObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The AIF is started and Obsai antenna traffic is up.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_DataTraceHandle hDataTrace;
            AIF_enableDataTrace(hAif, hDataTrace);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_enableDataTrace(
		AIF_ConfigHandle  	hAif,
		AIF_DataTraceHandle	hDataTrace
);

/**
 *   @n@b AIF_disableDataTrace
 *
 *   @b Description
 *   @n This function disables AIF2 data trace for OBSAI debug mode.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_disableDataTrace(hAif, hDataTrace);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void AIF_disableDataTrace(
		AIF_ConfigHandle  	hAif
);


#ifdef __cplusplus
}
#endif


#endif //__AIF_DEBUG_H

/** @} */ // end of module additions
