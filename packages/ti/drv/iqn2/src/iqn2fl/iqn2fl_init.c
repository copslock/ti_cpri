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

/** ===========================================================================
 *   @file  iqn2fl_init.c
 *
 *   @brief  IQN2 Intialization functional layer funtion
 *
 */

#include <ti/drv/iqn2/iqn2fl.h>

/** global variable definitions **/

/**************************************************************************\
* iqn2 global function declarations
\**************************************************************************/

/** ============================================================================
 *   @n@b Iqn2Fl_init
 *   @brief Peripheral specific initialization function.
 *
 *   @b Description
 *   @n This is the peripheral specific intialization function. This function is
 *   idempotent in that calling it many times is same as calling it once.
 *   This function initializes the CSL data structures, and doesn't touches
 *   the hardware.
 *
 *   @b Arguments
 *   @verbatim
        pContext    Pointer to module-context. As iqn2 doesn't
                    have any context based information user is expected to pass
                    NULL.
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *   @li  IQN2FL_SOK - Always returns
 *
 *   <b> Pre Condition </b>
 *   @n  This function should be called before using any of the CSL APIs in the iqn2
 *   module.
 *
 *   <b> Post Condition </b>
 *   @n  The CSL for iqn2 is initialized
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
          Iqn2Fl_init(NULL); // Init functional layer for iqn2 module
     @endverbatim
 * =============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Iqn2Fl_init, ".text:iqn2fl");
#endif
Iqn2Fl_Status  Iqn2Fl_init(
    Iqn2Fl_Context    *pContext
)
{
   // nothing to do, so return IQN2FL_SOK if pointer is not NULL
   if (pContext != NULL) {
	   return IQN2FL_SOK;
   } else {
	   return IQN2FL_BADHANDLE;
   }
}
