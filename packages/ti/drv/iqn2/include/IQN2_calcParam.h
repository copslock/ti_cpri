/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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
 * @file IQN2_calcParam.h
 *
 * @brief Header file for IQN2 parameter calculations based on user inputs
 * 
*/
 

/** @addtogroup IQN2_FUNCTION  IQN2 Functions
 *  @{
 */


#ifndef __IQN2_CALCPARAM_H
#define __IQN2_CALCPARAM_H

#include <ti/csl/csl.h>
#include <ti/drv/iqn2/include/IQN2_defs.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b IQN2_calcParameters
 *
 *   @b Description
 *   @n This function calculates IQN2 timing and delay parameters according to the pre-set
 *      parameters in IQN2_ConfigObj. This function should be called before IQN2 configuration.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2   Pointer to a IQN2_ConfigObj instance.
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
        IQN2_ConfigObj iqn2Obj;
            IQN2_calcParameters(&iqn2Obj);
     @endverbatim
 * 
 */
#ifndef __IQN2_CALCPARAM_C
extern
#endif
void
IQN2_calcParameters(
    IQN2_ConfigHandle    hIqn2
);

/**
 *   @n@b IQN2_calcAilTimingForTxNode
 *
 *   @b Description
 *   @n This function calculates the AIF timing parameters (pe2Offset, deltaOffset) for a given AIL link
 *    based on the nodeTx and txWait parameters.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2       Pointer to a IQN2_ConfigObj instance.
        ailIndex    AIL link number.
     @endverbatim
 *
 *   <b> Return Value </b>  deltaOffset
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  IQN2 configuration structure.
 *
 *   @b Modifies
 *   @n  SW configuration
 *
 *   @b Example
 *   @verbatim
        IQN2_ConfigObj iqn2Obj;
        Iqn2Fl_AilInstance   linkIndex;
            IQN2_calcAilTimingForTxNode(&iqn2Obj, linkIndex);
     @endverbatim
 *
 */
#ifndef __IQN2_CALCPARAM_C
extern
#endif
uint32_t
IQN2_calcAilTimingForTxNode(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_AilInstance   ailIndex
);


#ifdef __cplusplus
}
#endif


#endif //__IQN2_CALCPARAM_H

/** @} */ // end of module additions
