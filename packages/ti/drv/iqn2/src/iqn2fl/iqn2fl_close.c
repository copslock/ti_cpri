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
 *   @file  iqn2fl_close.c
 *
 *   @brief  IQN2FL close function
 *
 */                         

#include <ti/drv/iqn2/iqn2fl.h>

/** ============================================================================
 *   @n@b Iqn2FL_Close
 *
 *   @b Description
 *   @n The Close call releases the resources of the peripheral.
 *
 *   @b Arguments
 *   @verbatim
            hIqn2        Handle to the iqn2 instance
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *   @li                    IQN2FL_SOK             - Close successful
 *   @li                    IQN2FL_BADHANDLE  - Invalid handle
 *
 *   <b> Usage Constraints: </b>
 *   Both @a Iqn2FL_Init() and @a Iqn2FL_Open() must be called successfully
 *   in that order before Iqn2FL_Close() can be called.
 *   @b Example
 *   @verbatim
       Iqn2FL_Handle hIqn2;
       // Properly initialize and open desired AIL for use
       hIqn2 = Iqn2FL_Open(&iqn2Obj0, CSL_IQN, &iqn2Param, &status);
       Iqn2FL_Close(hIqn2);
     @endverbatim
 * =============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Iqn2Fl_close, ".text:iqn2fl");
#endif
Iqn2Fl_Status  Iqn2Fl_close(
    /** Pointer to the object that holds reference to the
     *  instance of iqn2 requested after the Iqn2Fl_open(...) call
    */
    Iqn2Fl_Handle        hIqn2
)
{
    Iqn2Fl_Status  st;
    
    if (hIqn2 != NULL) {
        hIqn2->regs = (Iqn2Fl_RegsOvly)NULL;
        hIqn2->perNum   = (CSL_InstNum)-1;
        
        st = IQN2FL_SOK;
    }
    else {
        st = IQN2FL_BADHANDLE;
    }
    return st;
}

