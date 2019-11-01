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
 *   @file  aif2fl_open.c
 *
 *   @brief  Antenna Interface 2 open function
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
#include <ti/drv/aif2/AIF_defs.h>

/*extern Aif2Fl_Status Aif2Fl_getBaseAddress (
        CSL_InstNum 	        aif2Num,
        Aif2Fl_Param *          paif2Param,
        Aif2Fl_BaseAddress *    pBaseAddress 
);*/

/** ============================================================================
 *   @n@b Aif2Fl_open 
 *   @brief Opens the instance of aif2 requested.
 *
 *   @b Description
 *   @n The open call sets up the data structures for the particular instance of
 *   aif2 device. The device can be re-opened anytime after it has been normally
 *   closed if so required. The handle returned by this call is input as an
 *   essential argument for rest of the APIs described for this module.
 *
 *   @b Arguments
 *   @verbatim            
           paif2LinkObj    Pointer to the object that holds reference to the
                          instance of aif2 requested after the call
                           
           aif2Num         Instance of aif2 to which a handle is requested
           
           paif2Param      Module specific parameters
 
           pStatus        pointer for returning status of the function call

     @endverbatim
 *
 *   <b> Return Value </b>  
 *      Aif2Fl_handle
 *        Valid aif2 instance handle will be returned if status value is
 *        equal to AIF2FL_SOK.
 *
 *   <b> Pre Condition </b>
 *   @n  aif2 must be successfully initialized via @a Aif2Fl_init() before 
 *   calling this function. Memory for the @a Aif2Fl_Obj must be allocated 
 *   outside this call. This object must be retained while usage of this peripheral.
 *
 *
 *   <b> Post Condition </b>
 *   @n   1.    The status is returned in the status variable. If status
 *              returned is
 *   @li            AIF2FL_SOK             Valid aif2 handle is returned
 *   @li            AIF2FL_FAIL       The aif2 instance is invalid
 *
 *        2.    aif2 object structure is populated
 * 
 *   @b  Writes
 *   @n    1. The status variable
 *           2. aif2 object structure
 *
 *   @b Example:
 *   @verbatim
          // handle for link 0
          Aif2Fl_handle handleaif2Link0;
          //  link object for link 0
          Aif2Fl_linkObj aif2LinkObj0;
          //aif2 module specific parameters
          Aif2Fl_param  aif2Param;
          // CSL status
          Aif2Fl_Status status;

          aif2Param.LinkIndex = AIF2FL_LINK_0;
   
          // Open handle for link 0 - for use 
          handleaif2Link0 = Aif2Fl_open(&aif2LinkObj0, 0, &aif2Param, &status);

          if ((handleaif2Link0 == NULL) || (status != AIF2FL_SOK)) 
          {
             printf ("\nError opening AIF2FL");
             exit(1);
          }
       @endverbatim
 *
 * ===========================================================================
 */
 /** Device specific configuration used to pass configuration register base address
   */
AIF_InitCfg         *pGlobalAif2Cfg=NULL;
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_open, ".text:aif2fl");
#endif

Aif2Fl_Handle Aif2Fl_open (
   /** Pointer to the object that holds reference to the
    *  instance of aif2 requested after the call
    */
   Aif2Fl_Obj         *paif2Obj,
   /** Instance of aif2 to which a handle is requested
    */
   CSL_InstNum          aif2Num,
   /** Module specific parameters;
    */
   Aif2Fl_Param       *paif2Param,
   /** This returns the status (success/errors) of the call.
    * Could be 'NULL' if the user does not want status information.
    */
   Aif2Fl_Status          *pStatus
   )
{
    Aif2Fl_Status           st;
    Aif2Fl_Handle       haif2;
    Aif2Fl_BaseAddress  baseAddress;

    if (pStatus == NULL) {
        return NULL;
    }
    if (paif2Obj == NULL) {
        *pStatus = AIF2FL_INVPARAMS;
        return NULL;
    }

#ifdef _TMS320C6X
    if (Aif2Fl_getBaseAddress(aif2Num, paif2Param, &baseAddress) == AIF2FL_SOK) {
    	paif2Obj->regs = baseAddress.regs;
#else
    if ((pGlobalAif2Cfg !=NULL) && (pGlobalAif2Cfg->dev.bases[aif2Num].cfgBase != NULL)){
    	paif2Obj->regs = pGlobalAif2Cfg->dev.bases[aif2Num].cfgBase;
#endif
        paif2Obj->arg_link = (Aif2Fl_LinkIndex)0;
        paif2Obj->arg_dioEngine = (Aif2Fl_DioEngineIndex)0;
        paif2Obj->perNum = (CSL_InstNum)aif2Num;
        haif2 = (Aif2Fl_Handle)paif2Obj;
        st = AIF2FL_SOK;
    } else {
        paif2Obj->regs = (Aif2Fl_RegsOvly)NULL;
        paif2Obj->arg_link = (Aif2Fl_LinkIndex)0;
        paif2Obj->arg_dioEngine = (Aif2Fl_DioEngineIndex)0;
        paif2Obj->perNum = (CSL_InstNum)-1;
        haif2 = (Aif2Fl_Handle)NULL;
        st = AIF2FL_FAIL;
    }
  
    *pStatus = st;

    return haif2;
}



