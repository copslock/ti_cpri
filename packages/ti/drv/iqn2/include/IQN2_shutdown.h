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
 * @file IQN2_shutdown.h
 *
 * @brief Header file for IQN2 H/W cleanup
 * 
*/


/** @addtogroup IQN2_FUNCTION  IQN2 Functions
 *  @{
 */


#ifndef __IQN2_SHUTDOWN_H
#define __IQN2_SHUTDOWN_H


#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b IQN2_resetAt2
 *
 *   @b Description
 *   @n This function is used to disable events and stop timers.
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
 *   @n  The IQN2 AT2 H/W is now stopped.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_resetAt2(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_SHUTDOWN_C
extern
#endif
void IQN2_resetAt2(
    IQN2_ConfigHandle    hIqn2,
    Iqn2Fl_InitCfg*      hIqn2BaseAddr
);

/**
 *   @n@b IQN2_resetIqn2
 *
 *   @b Description
 *   @n This function is used to reset IQN2 to its default state.
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
 *   @n  The IQN2 H/W is now in reset state.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_resetIqn2(hIqn2);
     @endverbatim
 * 
 */
#ifndef __IQN2_SHUTDOWN_C
extern
#endif
void IQN2_resetIqn2(
    IQN2_ConfigHandle    hIqn2,
    Iqn2Fl_InitCfg*      hIqn2BaseAddr
);

#ifdef __cplusplus
}
#endif


#endif //__IQN2_SHUTDOWN_H

/** @} */ // end of module additions
