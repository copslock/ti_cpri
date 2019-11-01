/*  ===========================================================================
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
 *   @file  aif2fl_hwSetup.c
 *
 *   @brief  AIF2 HW setup function
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
#include <ti/drv/aif2/aif2fl_hwSetupAux.h>

/** ============================================================================
 *   @n@b Aif2Fl_hwSetup
 *
 *   @b Description       
 *   @n It configures the AIF2 instance registers as per the values passed
 *      in the hardware setup structure.
 *
 *   @b Arguments
 *   @verbatim
            hAif2        Handle to the AIF2 instance

            aif2Setup       Pointer to hardware setup structure
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *   @li                    AIF2FL_SOK             - Hardware setup successful.
 *   @li                    AIF2FL_BADHANDLE  - Invalid handle
 *   @li                    AIF2FL_INVPARAMS  - Hardware structure is not
 *                                                properly initialized
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before Aif2Fl_hwSetup() can be called. The user has to
 *   allocate space for & fill in the main setup structure appropriately before
 *   calling this function.
 *
 *   <b> Post Condition </b>
 *   @n  The specified instance will be setup according to value passed.
 *
 *   @b Writes
 *   @n The hardware registers of AIF2.
 *
 *   @b Example                  
 *   @verbatim
          Aif2Fl_Handle handleAif2;
          
          Aif2Fl_Obj Aif2Obj;
          //AIF module specific parameters
          Aif2Fl_Param  aif2Param;
          // CSL status
          Aif2Fl_Status status;
          // global config for AIF2 
          Aif2Fl_GlobalSetup gblCfg = {�};

          // Setup objects for global configuring  
          Aif2Fl_GlobalSetup GlobalConfig;
          // Setup for common params  
          Aif2Fl_CommonSetup commoncfg= {AIF2FL_LINK_0};
          // Setup for link 
          Aif2Fl_Setup Config = {�};
   
          // Open handle - for use 
          handleAif2 = Aif2Fl_open(&Aif2Obj, 0, &aif2Param, &status);

          if ((handleAif2 == NULL) || (status != AIF2FL_SOK)) 
          {
             printf ("\nError opening AIF2FL");
             exit(1);
          }

          // Do config 
          Config.globalSetup = &gblCfg;
          Config.commonSetup = &commoncfg;
          Config.linkSetup = &linkCfg;
       
          //Do setup 
          Aif2Fl_hwSetup(handleAif2, &Config);
     @endverbatim
 * =============================================================================
 */ 
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_hwSetup, ".text:aif2fl");
#endif
Aif2Fl_Status  Aif2Fl_hwSetup(
    /** Pointer to the object that holds reference to the
     *  instance of AIF2 link requested after the call
    */
    Aif2Fl_Handle             hAif2,
    /** Pointer to setup structure which contains the
     *  information to program AIF2 to a useful state
    */
    Aif2Fl_Setup          *aif2Setup
)

{
    int i;
    Aif2Fl_Status status = AIF2FL_SOK;

    if (hAif2 == NULL)
        return AIF2FL_BADHANDLE;
    
    if (aif2Setup == NULL)
        return AIF2FL_INVPARAMS;

    /* SERDES Common setup to activate byte clock */
    if (aif2Setup->commonSetup->pSdCommonSetup != NULL) {
        Aif2Fl_setupSdCommonRegs(hAif2, aif2Setup->commonSetup);
    }
    
    /*  link setup*/	
    for(i = 0; i < 6; i++) // while  loop to repeatively setup each link
    {
	 if(aif2Setup->globalSetup->ActiveLink[i] == true)
	 {	 	
	    /* link common setup */
	    if (aif2Setup->linkSetup[i]->pComLinkSetup != NULL) {
	        Aif2Fl_setupLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }
	    
	    /* SERDES link setup */
	    if (aif2Setup->linkSetup[i]->pSdLinkSetup != NULL) {
	        Aif2Fl_setupSdLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }
		
	    /* TM setup*/
	    if (aif2Setup->linkSetup[i]->pTmLinkSetup != NULL) {
	        Aif2Fl_setupTmLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }

	     /* RM setup*/
	    if (aif2Setup->linkSetup[i]->pRmLinkSetup != NULL) {
	        Aif2Fl_setupRmLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }

            /* Retransmitter Configuration */
	    if (aif2Setup->linkSetup[i]->pRtLinkSetup != NULL) {
	        Aif2Fl_setupRtLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }
	    
	    /* Protocol Decoder Setup */
	    if (aif2Setup->linkSetup[i]->pPdLinkSetup != NULL) {
	        Aif2Fl_setupPdLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }
		
	    /* Protocol Encoder Setup */
	    if (aif2Setup->linkSetup[i]->pPeLinkSetup != NULL) {
	        Aif2Fl_setupPeLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }

	    /* AT link Configuration */
	    if (aif2Setup->linkSetup[i]->pAtLinkSetup != NULL) {
	        Aif2Fl_setupAtLinkRegs(hAif2, aif2Setup->linkSetup[i]);
	    }	    
	 }
    }
	
    /* Common setup */
	
    /* Protocol Decoder Setup */
    if (aif2Setup->commonSetup->pPdCommonSetup != NULL) {
        Aif2Fl_setupPdCommonRegs(hAif2, aif2Setup->commonSetup);
    }

    /* Protocol Encoder Setup */
    if (aif2Setup->commonSetup->pPeCommonSetup != NULL) {
        Aif2Fl_setupPeCommonRegs(hAif2, aif2Setup->commonSetup);
    }

     /* Ingress DB setup */
    if (aif2Setup->commonSetup->pIngrDbSetup != NULL)
    {
        Aif2Fl_setupIngrDbRegs(hAif2, aif2Setup->commonSetup);
    }

    /* Egress DB  setup */
    if (aif2Setup->commonSetup->pEgrDbSetup != NULL)
    {
        Aif2Fl_setupEgrDbRegs(hAif2, aif2Setup->commonSetup);
    }
	
     /* AD(Aif Dma) common setup */
    if (aif2Setup->commonSetup->pAdCommonSetup != NULL)
    {
        Aif2Fl_setupAdCommonRegs(hAif2, aif2Setup->commonSetup);
    }

    /* AD DIO  setup */
    if (aif2Setup->commonSetup->pAdDioSetup != NULL)
    {
        Aif2Fl_setupAdInDioRegs(hAif2, aif2Setup->commonSetup);

	 Aif2Fl_setupAdEDioRegs(hAif2, aif2Setup->commonSetup);
    }
  
     /* Aif2 Timer common setup */
    if (aif2Setup->commonSetup->pAtCommonSetup!= NULL)
    {
        Aif2Fl_setupAtCommonRegs(hAif2, aif2Setup->commonSetup);
    }

    /* AT Event  setup */
    if (aif2Setup->commonSetup->pAtEventSetup!= NULL)
    {
        Aif2Fl_setupAtEventRegs(hAif2, aif2Setup->commonSetup);
    }
 
    /* Global setup */
    if(aif2Setup->globalSetup !=NULL) {
        Aif2Fl_setupGlobalRegs(hAif2, aif2Setup->globalSetup);                                                       
    }
    
    return status;
}

