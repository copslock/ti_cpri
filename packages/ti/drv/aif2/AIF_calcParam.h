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
 * @file AIF_calcParam.h
 *
 * @brief Header file for AIF parameter calculations based on user inputs
 * 
*/
 

/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */


#ifndef __AIF_CALCPARAM_H
#define __AIF_CALCPARAM_H

#include <ti/csl/csl.h>
#include <ti/drv/aif2/AIF_defs.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b AIF_calcParameters
 *
 *   @b Description
 *   @n This function calculates some other AIF Parameters according to the pre-set
 *      parameters in AIF2_ConfigObj. This function should called before AIF_initHw().
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
 *   @n  None.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
        AIF2_ConfigObj aifObj;
            AIF_calcParameters(&aifObj);
     @endverbatim
 * 
 */
#ifndef __AIF_CALCPARAM_C
extern
#endif
void
AIF_calcParameters(
    AIF_ConfigHandle    hAif
);


/**
 *   @n@b AIF_calcAifTimingForTxNode
 *
 *   @b Description
 *   @n This function calculates the AIF timing parameters (pe2Offset, deltaOffset) for a given link
 *    based on the nodeTx and txWait parameters.
 *
 *   @b Arguments
 *   @verbatim
        hAif   		Pointer to a AIF2_ConfigObj instance.
        linkIndex	Link number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  None.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
        AIF2_ConfigObj aifObj;
        Aif2Fl_LinkIndex   linkIndex;
            AIF_calcAifTimingForTxNode(&aifObj, linkIndex);
     @endverbatim
 *
 */
#ifndef __AIF_CALCPARAM_C
extern
#endif
uint32_t
AIF_calcAifTimingForTxNode(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex
);

#ifdef __cplusplus
}
#endif


#endif //__AIF_CALCPARAM_H

/** @} */ // end of module additions
