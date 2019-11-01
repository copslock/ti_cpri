/*  ===========================================================================
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
 *   @file  iqn2fl_hwSetup.c
 *
 *   @brief  IQN2 HW setup functional layer function
 *
 */
 
#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwSetupAux.h>

/** ============================================================================
 *   @n@b Iqn2Fl_hwSetup
 *
 *   @b Description       
 *   @n It configures the IQN2 instance registers as per the values passed
 *      in the hardware setup structure.
 *
 *   @b Arguments
 *   @verbatim
            hIqn2           Handle to the IQN2 instance

            iqn2Setup       Pointer to hardware setup structure
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *   @li                    IQN2FL_SOK             - Hardware setup successful.
 *   @li                    IQN2FL_BADHANDLE  - Invalid handle
 *   @li                    IQN2FL_INVPARAMS  - Hardware structure is not
 *                                                properly initialized
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a Iqn2Fl_init() and @a Iqn2Fl_open() must be called successfully
 *   in that order before Iqn2Fl_hwSetup() can be called. The user has to
 *   allocate space for & fill in the main setup structure appropriately before
 *   calling this function.
 *
 *   <b> Post Condition </b>
 *   @n  The specified instance will be setup according to value passed.
 *
 *   @b Writes
 *   @n The hardware registers of IQN2.
 *
 *   @b Example                  
 *   @verbatim
          Iqn2Fl_Handle handleIqn2;
          
          Iqn2Fl_Obj Iqn2Obj;
          //IQN module specific parameters
          Iqn2Fl_Param  iqn2Param;
          // CSL status
          Iqn2Fl_Status status;
          // AIL setup
          Iqn2Fl_ailSetup linkCfg0 = {...};
          Iqn2Fl_ailSetup linkCfg1 = {...};

          // Setup for link 
          Iqn2Fl_Setup Config = {…};
   
          // Open handle - for use 
          handleIqn2 = Iqn2Fl_open(&Iqn2Obj, IQN2FL, &iqn2Param, &status);

          if ((handleIqn2 == NULL) || (status != IQN2FL_SOK))
          {
             printf ("\nError opening IQN2FL");
             exit(1);
          }

          // Do config 
          linkCfg0.ailInstNum = IQN2FL_AIL_0;
          linkCfg1.ailInstNum = IQN2FL_AIL_1;

          Config.ailSetup[0] = &linkCfg0;
          Config.ailSetup[1] = &linkCfg1;
          .
          .
          .

          //Do setup 
          Iqn2Fl_hwSetup(handleIqn2, &Config);
     @endverbatim
 * =============================================================================
 */ 
#ifdef _TMS320C6X
#pragma CODE_SECTION (Iqn2Fl_hwSetup, ".text:iqn2fl");
#endif
Iqn2Fl_Status  Iqn2Fl_hwSetup(
    /** Pointer to the object that holds reference to the
     *  instance of IQN2 link requested after the call
    */
    Iqn2Fl_Handle             hIqn2,
    /** Pointer to setup structure which contains the
     *  information to program IQN2 to a useful state
    */
    Iqn2Fl_Setup          *iqn2Setup
)

{
    Iqn2Fl_Status status = IQN2FL_INVPARAMS;
    uint32_t i;

    if (hIqn2 == NULL)
        return IQN2FL_BADHANDLE;

    if (iqn2Setup == NULL)
        return IQN2FL_INVPARAMS;

    /*  AIL (Antenna Interface Link) registers setup */
    for(i = 0; i < IQN2FL_AIL_MAX; i++) // loop to setup each link
    {
        if(iqn2Setup->ailSetup[i] != NULL)
        {
            hIqn2->arg_ail = (Iqn2Fl_AilInstance)i;

            /* AIL Egress setup */
            if (iqn2Setup->ailSetup[i]->pAilEgrSetup != NULL) {
                Iqn2Fl_setupAilEgressRegs(hIqn2, iqn2Setup->ailSetup[i]);
            }
            /* AIL Ingress setup */
            if (iqn2Setup->ailSetup[i]->pAilIgrSetup != NULL) {
                Iqn2Fl_setupAilIngressRegs(hIqn2, iqn2Setup->ailSetup[i]);
            }
            /* AIL Protocol Encoder (PE) setup */
            if (iqn2Setup->ailSetup[i]->pAilPeSetup != NULL) {
                Iqn2Fl_setupAilPeRegs(hIqn2, iqn2Setup->ailSetup[i]);
            }
            /* AIL Protocol Decoder (PD) setup */
            if (iqn2Setup->ailSetup[i]->pAilPdSetup != NULL) {
                Iqn2Fl_setupAilPdRegs(hIqn2, iqn2Setup->ailSetup[i]);
            }
            /* AIL Micro Antenna Timer (UAT) setup */
            if (iqn2Setup->ailSetup[i]->pAilUatSetup != NULL) {
                Iqn2Fl_setupAilUatRegs(hIqn2, iqn2Setup->ailSetup[i]);
            }
            /* AIL PHY (Global, RT, CI/CO LUT, TM, RM) setup */
            if (iqn2Setup->ailSetup[i]->pAilPhySetup != NULL) {
                Iqn2Fl_setupAilPhyRegs(hIqn2, iqn2Setup->ailSetup[i]);
            }
            status = IQN2FL_SOK;
        }
    }

    /*  Top registers setup */
    if(iqn2Setup->topSetup != NULL)
    {
        Iqn2Fl_setupTopRegs(hIqn2, iqn2Setup->topSetup);
        status = IQN2FL_SOK;
    }

    /*  AID2 (Antenna Interface to DFE) registers setup */
    if(iqn2Setup->aid2Setup != NULL)
    {
        Iqn2Fl_setupAid2Regs(hIqn2, iqn2Setup->aid2Setup);
        status = IQN2FL_SOK;
    }

    /*  IQS2 (IQN Switching Infrastructure) registers setup */
    if(iqn2Setup->iqs2Setup != NULL)
    {
        Iqn2Fl_setupIqs2Regs(hIqn2, iqn2Setup->iqs2Setup);
        status = IQN2FL_SOK;
    }

    /*  DIO2 (Direct I/O) registers setup */
    if(iqn2Setup->dio2Setup != NULL)
    {
        Iqn2Fl_setupDio2Regs(hIqn2, iqn2Setup->dio2Setup);
        status = IQN2FL_SOK;
    }

    /*  AT2 (Antenna Interface Timer) registers setup */
    if(iqn2Setup->at2Setup != NULL)
    {
        Iqn2Fl_setupAt2Regs(hIqn2, iqn2Setup->at2Setup);
        status = IQN2FL_SOK;
    }

    return status;
}
