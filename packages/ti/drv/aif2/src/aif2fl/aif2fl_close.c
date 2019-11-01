/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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
 *   @file  aif2fl_close.c
 *
 *
 *   @brief  Antenna Interface 2 close function
 *
 */                         

/* =============================================================================
 * Revision History
 * ===============
 *  02-June-2009  Albert   File Created.
 *  06-June-2015  Seb      File imported in the driver
 *  
 *
 * =============================================================================
 */

#include <ti/drv/aif2/aif2fl.h>
/** ============================================================================
 *   @n@b Aif2Fl_close
 *
 *   @b Description
 *   @n The Close call releases the resources of the peripheral.
 *
 *   @b Arguments
 *   @verbatim
            haif2Link        Handle to the aif2 instance
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *   @li                    AIF2FL_SOK             - Close successful
 *   @li                    AIF2FL_BADHANDLE  - Invalid handle
 *
 *   <b> Usage Constraints: </b>
 *   Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before Aif2Fl_close() can be called.
 *   @b Example
 *   @verbatim
       Aif2Fl_handle haif2Link;
       // Properly initialize and open desired link for use
       haif2Link = Aif2Fl_open(&aif2LinkObj0, 0, &aif2Param, &status);
       Aif2Fl_close(haif2Link);
     @endverbatim
 * =============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_close, ".text:aif2fl");
#endif
Aif2Fl_Status  Aif2Fl_close(
    /** Pointer to the object that holds reference to the
     *  instance of aif2  link requested after the Aif2Fl_open(...) call
    */
    Aif2Fl_Handle        haif2
)
{
    Aif2Fl_Status  st;
    
    if (haif2 != NULL) {
        haif2->regs = (Aif2Fl_RegsOvly)NULL;
        haif2->perNum   = (CSL_InstNum)-1;
        
        st = AIF2FL_SOK;
    }
    else {
        st = AIF2FL_BADHANDLE;
    }
    return st;
}

