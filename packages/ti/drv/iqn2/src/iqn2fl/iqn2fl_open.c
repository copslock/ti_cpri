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
 *   @file  iqn2fl_open.c
 *
 *   @brief  IQN2 open functional layer function
 *
 */

#include <ti/drv/iqn2/iqn2fl.h>


/** ============================================================================
 *   @n@b Iqn2Fl_open
 *   @brief Opens the instance of iqn2 requested.
 *
 *   @b Description
 *   @n The open call sets up the data structures for the particular instance of
 *   iqn2 device. The device can be re-opened anytime after it has been normally
 *   closed if so required. The handle returned by this call is input as an
 *   essential argument for rest of the APIs described for this module.
 *
 *   @b Arguments
 *   @verbatim            
           pIqn2LinkObj    Pointer to the object that holds reference to the
                           instance of iqn2 requested after the call
                           
           iqn2Num         Instance of iqn2 to which a handle is requested
           
           pIqn2Param      Module specific parameters
 
           pStatus         pointer for returning status of the function call

     @endverbatim
 *
 *   <b> Return Value </b>  
 *      Iqn2Fl_Handle
 *        Valid iqn2 instance handle will be returned if status value is
 *        equal to IQN2FL_SOK.
 *
 *   <b> Pre Condition </b>
 *   @n  iqn2 must be successfully initialized via @a Iqn2Fl_init() before
 *   calling this function. Memory for the @a Iqn2Fl_Obj must be allocated
 *   outside this call. This object must be retained while usage of this peripheral.
 *
 *
 *   <b> Post Condition </b>
 *   @n   1.    The status is returned in the status variable. If status
 *              returned is
 *   @li            IQN2FL_SOK             Valid iqn2 handle is returned
 *   @li            IQN2FL_FAIL       The iqn2 instance is invalid
 *
 *        2.    iqn2 object structure is populated
 * 
 *   @b  Writes
 *   @n    1. The status variable
 *         2. iqn2 object structure
 *
 *   @b Example:
 *   @verbatim
          // handle for IQN2 instance
          Iqn2Fl_Handle handleIqn2;
          // IQN2 object
          Iqn2Fl_Obj iqn2Obj;
          // iqn2 module specific parameters
          Iqn2Fl_Param  iqn2Param;
          // IQN2 Instance number
          CSL_InstNum iqn2InstNum;
          // CSL status
          Iqn2Fl_Status status;

          iqn2InstNum = CSL_IQN;

          // Open handle for IQN2 instance - for use 
          handleIqn2 = Iqn2Fl_open(&iqn2Obj0, iqn2InstNum, &iqn2Param, &status);

          if ((handleIqn2 == NULL) || (status != IQN2FL_SOK))
          {
             printf ("\nError opening CSL_IQN");
             exit(1);
          }
       @endverbatim
 *
 * ===========================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Iqn2Fl_open, ".text:iqn2fl");
#endif

Iqn2Fl_Handle Iqn2Fl_open (
   /** Pointer to the object that holds reference to the
    *  instance of iqn2 requested after the call
    */
   Iqn2Fl_Obj         *pIqn2Obj,
   /** Instance of iqn2 to which a handle is requested
    */
   CSL_InstNum         iqn2Num,
   /** Device specific configuration used to pass configuration register base address
    */
   Iqn2Fl_InitCfg     *pIqn2Cfg,
   /** This returns the status (success/errors) of the call.
    * Could be 'NULL' if the user does not want status information.
    */
   Iqn2Fl_Status      *pStatus
   )
{
    Iqn2Fl_Status           st;
    Iqn2Fl_Handle       hIqn2;

    if (pIqn2Obj == NULL) {
        *pStatus = IQN2FL_INVPARAMS;
        return NULL;
    }

    if (iqn2Num <= CSL_IQN) {
        pIqn2Obj->regs = pIqn2Cfg->dev.bases[iqn2Num].cfgBase;
        pIqn2Obj->perNum = (CSL_InstNum)iqn2Num;
        hIqn2 = (Iqn2Fl_Handle)pIqn2Obj;
        st = IQN2FL_SOK;
    } else {
        pIqn2Obj->regs = (Iqn2Fl_RegsOvly)NULL;
        pIqn2Obj->perNum = (CSL_InstNum)-1;
        hIqn2 = (Iqn2Fl_Handle)NULL;
        st = IQN2FL_FAIL;
    }
  
    if (pStatus != NULL) {
        *pStatus = st;
    }

    return hIqn2;
}
