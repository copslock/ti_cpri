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
 *   @file  aif2fl_getBaseAddress.c
 *
 *   @brief File for functional layer
 *
 *
 *   @desc  Function definition.
 */
/* =============================================================================
 * Revision History
 * ===============
 *  06-June-2015  Seb      File imported in the driver
 *  
 *
 * =============================================================================
 */


#ifdef K2
#ifndef DEVICE_K2K
#define DEVICE_K2K
#endif
#endif
#include <ti/drv/aif2/aif2fl.h>

/** ============================================================================
 *   @n@b Aif2Fl_getBaseAddress
 *
 *   @b Description
 *   @n  Function to get the base address of the peripheral instance.
 *       This function is used for getting the base address of the peripheral
 *       instance. This function will be called inside the Aif2Fl_open()
 *       function call. This function is open for re-implementing if the user
 *       wants to modify the base address of the peripheral object to point to
 *       a different location and there by allow CSL initiated write/reads into
 *       peripheral. MMR's go to an alternate location.
 *
 *   @b Arguments
 *   @verbatim
            aif2Num        Specifies the instance of the bwmngmt to be opened.

            paif2Param     Module specific parameters.

            pBaseAddress      Pointer to baseaddress structure containing base
                              address details.

     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *   @li                    AIF2FL_SOK            Successful on getting the base 
 *                                             address of bwmngmt
 *   @li                    AIF2FL_FAIL      The instance number is invalid.
 *   @li                    AIF2FL_INVPARAMS Inavlid parameters 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  Base Address structure is populated
 *
 *   @b Affects
 *   @n    1. The status variable
 *
 *         2. Base address structure is modified.
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_Status              status;
        Aif2Fl_baseAddress  baseAddress;

       ...
      status = Aif2Fl_getBaseAddress(0, NULL, &baseAddress);

     @endverbatim
 * ===========================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_getBaseAddress, ".text:aif2fl");
#endif
Aif2Fl_Status
Aif2Fl_getBaseAddress (
        CSL_InstNum 	        aif2Num,
        Aif2Fl_Param *          paif2Param,
        Aif2Fl_BaseAddress *    pBaseAddress 
)
{
    Aif2Fl_Status st = AIF2FL_SOK;

    if (pBaseAddress == NULL) {
        return AIF2FL_INVPARAMS;
    }
#ifdef K2
    pBaseAddress->regs = (Aif2Fl_RegsOvly)CSL_AIF_CFG_REGS;
#else
    pBaseAddress->regs = (Aif2Fl_RegsOvly)CSL_AIF2_CONTROL_REGS;
#endif

    return st;
}


