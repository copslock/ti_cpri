/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008,2009
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
 *   @file  aif2fl_reset.c
 *
 *
 *   @brief  Antenna Interface 2 reset function
 */

/* =============================================================================
 * Revision History
 * ===============
 *  08-May-2009   Albert   File Created.
 *  06-June-2015  Seb      File imported in the driver
 *  
 *
 * =============================================================================
 */

#include <ti/drv/aif2/aif2fl.h>

/** ============================================================================
 *   @n@b Aif2Fl_reset
 *   @brief Reset whole AIF2 module.
 *
 *   @b Description
 *   @n Reset whole AIF2 devices and MMRs
 *        The handle returned by this call is input as an
 *        essential argument for rest of the APIs described for this module.
 *
 *   @b Arguments
 *   @verbatim            
           pAif2Handle    Pointer to the object that holds reference to the
                          instance of AIF2 requested after the call

     @endverbatim
 *
 *   <b> Return Value </b>  
 *      Aif2Fl_Status
 *        Valid AIF2 status will be returned if status value is
 *        equal to AIF2FL_SOK.
 *
 *   <b> Pre Condition </b>
 *   @n  AIF2 must be successfully initialized via @a Aif2Fl_init(), Aif2Fl_open() before 
 *   calling this function. Memory for the @a Aif2Fl_Obj must be allocated 
 *   outside this call. This object must be retained while usage of this peripheral.
 *
 *
 *   <b> Post Condition </b>
 *   @n   1.    The status is returned in the status variable. If status
 *              returned is
 *   @li            AIF2FL_SOK             Valid AIF2 handle is returned
 *   @li            AIF2FL_FAIL       The AIF2 instance is invalid
 *
 *        
 * 
 *   @b  Writes
 *   @n  AIF2_AIF2_RESET_SW_RST
 *
 *   @b Example:
 *   @verbatim
          //aif2  handle
          Aif2Fl_Handle hAif2;
         
          // CSL status
          Aif2Fl_Status status;
   
          //Reset handle  - for use 
          status = Aif2Fl_reset(hAif2);

          if (status != AIF2FL_SOK) 
          {
             printf ("\nError resetting AIF2FL");
             exit(1);
          }
       @endverbatim
 *
 * ===========================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_reset, ".text:aif2fl");
#endif
Aif2Fl_Status Aif2Fl_reset (
   /** Pointer to the object that holds reference to the
    *  instance of AIF2 requested after the call
    */
   Aif2Fl_Handle         pAif2Handle)
{
    Aif2Fl_Status           st;
    
    if (pAif2Handle == NULL) {
        st = AIF2FL_INVPARAMS;
        return st;
    }
    
    CSL_FINST(pAif2Handle->regs->AIF2_RESET, AIF2_AIF2_RESET_SW_RST, PULSE_RESET);
	
    st = AIF2FL_SOK;

    return st;
}

