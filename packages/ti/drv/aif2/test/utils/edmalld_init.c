/*
 * edmalld_init.c
 *
 * Sample Initialization for the EDMA3 Driver for BIOS 6 based applications.
 * It should be MANDATORILY done once before EDMA3 usage.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

#include <ti/sdo/edma3/drv/sample/bios6_edma3_drv_sample.h>

#include <ti/csl/cslr.h>
//#include <ti/csl/csl_edma3Aux.h>
//#include <ti/csl/csl_edma3.h>

#include "edmalld.h"



/** @brief EDMA3 Driver Instance specific Semaphore handle */
extern EDMA3_OS_Sem_Handle semHandle[];

/**  To Register the ISRs with the underlying OS, if required. */
extern void registerEdma3Interrupts (unsigned int edma3Id);
/**  To Unregister the ISRs with the underlying OS, if previously registered. */
extern void unregisterEdma3Interrupts (unsigned int edma3Id);

/* To find out the DSP# */
extern unsigned short determineProcId();

/**
 * To check whether the global EDMA3 configuration is required or not.
 * It should be done ONCE by any of the masters present in the system.
 * This function checks whether the global configuration is required by the
 * current master or not. In case of many masters, it should be done only
 * by one of the masters. Hence this function will return true only once
 * and false for all other masters.
 */
extern unsigned short isGblConfigRequired(unsigned int dspNum);

/**
 * DSP instance number on which the executable is running. Its value is
 * determined by reading the processor specific register DNUM.
 */
unsigned int dsp_num;

/**
 * Shadow Region on which the executable is running. Its value is populated
 * with the DSP Instance Number here in this case.
 */
unsigned int region_id;

/* Number of EDMA3 controllers present in the system */
extern const unsigned int numEdma3Instances;

/* External Global Configuration Structure */
extern EDMA3_DRV_GblConfigParams sampleEdma3GblCfgParams[];

/* External Instance Specific Configuration Structure */
extern EDMA3_DRV_InstanceInitConfig sampleInstInitConfig[][EDMA3_MAX_REGIONS];

// text or not
#define verbose 0

/*
//EDMA3 interruption setting registers
#define ER_reg		0x02701000
#define ESR_reg		0x02701010
#define ESRH_reg	0x02701014
#define EER_reg		0x02701020
#define EERH_reg	0x02701024
#define EESR_reg	0x02701030
#define EESRH_reg	0x02701034
#define IER_reg		0x02701050
#define IERH_reg	0x02701054
#define IESR_reg	0x02701060
#define IESRH_reg	0x02701064
*/
//EDMA3 interruption setting registers
#define ER_reg		0x1000
#define ESR_reg		0x1010
#define ESRH_reg	0x1014
#define EER_reg		0x1020
#define EERH_reg	0x1024
#define EESR_reg	0x1030
#define EESRH_reg	0x1034
#define IER_reg		0x1050
#define IERH_reg	0x1054
#define IESR_reg	0x1060
#define IESRH_reg	0x1064

void printError(EDMA3_DRV_Result *isError, char currentFunction[64]);


/*****************************************************
 * EDMA LLD
 * 			INITIALIZATION
 *****************************************************/
/**
 * \brief   EDMA3 Initialization
 *
 * This function initializes the EDMA3 Driver and registers the
 * interrupt handlers.
 *
  * \return  EDMA3_DRV_SOK if success, else error code
 */
EDMA3_DRV_Handle edmalldinit (unsigned int edma3Id, EDMA3_DRV_Result *errorCode, unsigned int region)
    {
    EDMA3_DRV_Result edma3Result = EDMA3_DRV_E_INVALID_PARAM;
    Semaphore_Params semParams;
    EDMA3_DRV_GblConfigParams *globalConfig = NULL;
	EDMA3_DRV_InitConfig initCfg;
//	EDMA3_RM_MiscParam miscParam;
	EDMA3_DRV_Handle hEdma = NULL;
	// used in DRV_create() function to specify master/slave
	EDMA3_DRV_MiscParam miscParam;

	if ((edma3Id >= numEdma3Instances) || (errorCode == NULL))
		return hEdma;

    /* DSP instance number */
    dsp_num = determineProcId();

	globalConfig = &sampleEdma3GblCfgParams[edma3Id];

	/* Configure it as master, if required */
	miscParam.isSlave = isGblConfigRequired(dsp_num);
	edma3Result = EDMA3_DRV_create (edma3Id, globalConfig ,
									(void *)&miscParam);
	printError(&edma3Result, "EDMA3 DRV creation");

	if (edma3Result == EDMA3_DRV_SOK)
		{
		/**
		* Driver Object created successfully.
		* Create a semaphore now for driver instance.
		*/
		Semaphore_Params_init(&semParams);

		initCfg.drvSemHandle = NULL;
		edma3Result = edma3OsSemCreate(1, &semParams, &initCfg.drvSemHandle);
		printError(&edma3Result, "EDMA3 semaphore");
		}

	if (edma3Result == EDMA3_DRV_SOK)
		{
		/* Save the semaphore handle for future use */
		semHandle[edma3Id] = initCfg.drvSemHandle;

        /* Driver instance specific config NULL */
		initCfg.drvInstInitConfig = NULL;

#ifndef EDMA3_DRV_USE_DEF_RM_CFG
        /* Hook for running examples with default RM config */
		/* configuration structure for the Driver */
		initCfg.drvInstInitConfig = &sampleInstInitConfig[edma3Id][region];
#endif

		initCfg.isMaster = 1;
		/* Choose shadow region according to the DSP# */
		initCfg.regionId = (EDMA3_RM_RegionId)region;
		/*Saving the regionId for using it in the sample_cs.c file */
		region_id = (EDMA3_RM_RegionId)region;
		/* Driver instance specific config NULL */

		initCfg.gblerrCb = NULL;
		initCfg.gblerrData = NULL;

		/* Open the Driver Instance */
		hEdma = EDMA3_DRV_open (edma3Id, (void *) &initCfg, &edma3Result);
		}

#if defined (CHIP_TI814X)
    	{
    	if(hEdma && (edma3Result == EDMA3_DRV_SOK))
    		{
        	edma3Result = sampleInitXbarEvt(hEdma, edma3Id);
    		}
    	}
#endif
	if(hEdma && (edma3Result == EDMA3_DRV_SOK))
		{
		/**
		* Register Interrupt Handlers for various interrupts
		* like transfer completion interrupt, CC error
		* interrupt, TC error interrupts etc, if required.
		*/
		registerEdma3Interrupts(edma3Id);
		}

	*errorCode = edma3Result;
	return hEdma;
    }

/*****************************************************
 * EDMA LLD
 * 			DESINITIALIZATION
 *****************************************************/
/**
 * \brief   EDMA3 De-initialization
 *
 * This function removes the EDMA3 Driver instance and unregisters the
 * interrupt handlers.
 *
  * \return  EDMA3_DRV_SOK if success, else error code
 */
EDMA3_DRV_Result edma3deinit (unsigned int edma3Id, EDMA3_DRV_Handle hEdma)
    {
    EDMA3_DRV_Result edma3Result = EDMA3_DRV_E_INVALID_PARAM;

    /* Unregister Interrupt Handlers first */
    unregisterEdma3Interrupts(edma3Id);

    /* Delete the semaphore */
    edma3Result = edma3OsSemDelete(semHandle[edma3Id]);

    if (EDMA3_DRV_SOK == edma3Result )
        {
        /* Make the semaphore handle as NULL. */
        semHandle[edma3Id] = NULL;

        /* Now, close the EDMA3 Driver Instance */
        edma3Result = EDMA3_DRV_close (hEdma, NULL);
    	}

	if (EDMA3_DRV_SOK == edma3Result )
        {
        /* Now, delete the EDMA3 Driver Object */
        edma3Result = EDMA3_DRV_delete (edma3Id, NULL);
        }

    return edma3Result;
    }

/*****************************************************
 * EDMA LLD
 * 			OPEN CHANNEL
 *****************************************************/

void edmaOpenChannels(EDMA3_DRV_Handle* hEdma, channel_object* chanObj)
{
	EDMA3_DRV_Result currRes = EDMA3_DRV_SOK;

	// get channel for transfer
	currRes = EDMA3_DRV_requestChannel(*hEdma, &chanObj->chanNum, &chanObj->tccCh, chanObj->queue, NULL, NULL);
	printf("EDMA3\tChannel %d OPENING\n",chanObj->chanNum);
	printf("EDMA3\tTCC %d used\n",chanObj->tccCh);
	printError(&currRes, "\tEDMA3 Channel opening");

//	chanObj->isError = currRes;

//	EDMA3_DRV_mapChToEvtQ(hEdma,chanObj->chanNum, 0 );

	// set source parameters - also assigns defaults to Options register
	currRes = EDMA3_DRV_setSrcParams (*hEdma, chanObj->chanNum, (chanObj->srcAddr), EDMA3_DRV_ADDR_MODE_INCR, EDMA3_DRV_W8BIT); // set source parameters - also assigns defaults to Options register
	printError(&currRes, "\tEDMA3 src param set");
	// set dest parameters - also assigns defaults to Options register
	currRes = EDMA3_DRV_setDestParams (*hEdma, chanObj->chanNum, (chanObj->dstAddr), EDMA3_DRV_ADDR_MODE_INCR, EDMA3_DRV_W8BIT); // set dest parameters - also assigns defaults to Options register
	printError(&currRes, "\tEDMA3 dst param set");

	currRes = EDMA3_DRV_setSrcIndex(*hEdma, chanObj->chanNum, chanObj->srcBoffset, chanObj->srcCoffset); // set SRCBIDX to 2, SRCCIDX to 0 (single AB transfer)
	printError(&currRes, "\tEDMA3 src index set");
	currRes = 	EDMA3_DRV_setDestIndex(*hEdma, chanObj->chanNum, chanObj->dstBoffset, chanObj->dstCoffset);// set DSTBIDX to 2, DSTCIDX to 0
	printError(&currRes, "\tEDMA3 dst index set");

	currRes = EDMA3_DRV_setTransferParams(*hEdma, chanObj->chanNum, chanObj->aCnt,chanObj->bCnt,chanObj->cCnt,0,chanObj->syncType); //ACNT=2, BCNT=BUFFSIZE, CCNT=1, BCNTRLD=0, SYNC
	printError(&currRes, "\tEDMA3 transfer paramSet");

	//Interrupt enable ENABLED - required to generate interrupt when xfr is complete
//	currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_TCINTEN, EDMA3_DRV_TCINTEN_EN);   //Interrupt enable DISABLED - default is DIS if not called
//	currRes = EDMA3_DRV_setOptField (*hEdma,*chanNum, EDMA3_DRV_OPT_FIELD_SAM, EDMA3_DRV_ADDR_MODE_INCR);
//	printError(&currRes, "\tEDMA3 opt field set");

//	currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_ITCINTEN,EDMA3_DRV_ITCINTEN_EN);	//intermediate interrupt enabled
//	printError(&currRes, "EDMA3 opt field set");

	currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_ITCCHEN,chanObj->opt.itcchEn);	//Intermediate Transfer Completion CHannel
	printError(&currRes, "\tEDMA3 opt field set");
	currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_TCCHEN,chanObj->opt.tcchEn);	//Transfer Completion CHannel
	printError(&currRes, "\tEDMA3 opt field set");
	currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_ITCINTEN,chanObj->opt.itcintEn);	//Intermediate Transfer Completion INTerrupt
	printError(&currRes, "\tEDMA3 opt field set");
	currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_TCINTEN,chanObj->opt.tcintEn);	//Transfer Completion INTerrupt
	printError(&currRes, "\tEDMA3 opt field set");

    //si besoin
    currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_STATIC,EDMA3_DRV_STATIC_EN);      //Static mode
    printError(&currRes, "\tEDMA3 opt field set");


	if (chanObj->chainChan > -1)
		{
		currRes = EDMA3_DRV_chainChannel(*hEdma, chanObj->chanNum, (uint32_t)chanObj->chainChan, &chanObj->opt);
		printError(&currRes, "\tEDMA3 chain set\t");

		}
	else
		{
/*			currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_ITCCHEN,chanObj->opt.itcchEn);	//intermediate interrupt enabled
			printError(&currRes, "\tEDMA3 opt field set");
			currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_TCCHEN,chanObj->opt.tcchEn);	//intermediate interrupt enabled
			printError(&currRes, "\tEDMA3 opt field set");
			currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_ITCINTEN,chanObj->opt.itcintEn);	//intermediate interrupt enabled
			printError(&currRes, "\tEDMA3 opt field set");
			currRes = EDMA3_DRV_setOptField (*hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_TCINTEN,chanObj->opt.tcintEn);	//intermediate interrupt enabled
			printError(&currRes, "\tEDMA3 opt field set");
*/
		}

//	currRes = EDMA3_DRV_setOptField (hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_TCCMODE, EDMA3_DRV_TCCMODE_EARLY);	//interrupt can be triggered before the end of the transfer
//	currRes = EDMA3_DRV_setOptField (hEdma,chanObj->chanNum, EDMA3_DRV_OPT_FIELD_SYNCDIM,EDMA3_DRV_SYNC_A);
//	printError(&currRes, "EDMA3 opt field set");



}

// ============================    EDMA START   =======================================
//
// This function will start the EDMA3 transfer manually - i.e. an ASYNC transfer. This
// function would not be necessary if the transfer was sync'd to external event, for
// example, a peripheral like the McBSP.
//
// This function could be improved by returning the result.
// ===================================================================================

void edma_start(EDMA3_DRV_Handle* hEdma,uint32_t chanID, EDMA3_DRV_TrigMode triggerType)
{
	EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;
	//edma3Result = EDMA3_DRV_enableTransfer(*hEdma, *chanID, EDMA3_DRV_TRIG_MODE_MANUAL);
	edma3Result = EDMA3_DRV_enableTransfer(*hEdma, chanID, triggerType);
	printError(&edma3Result, "EDMA3 transfer enabling");
}


// ============================    EDMA WAIT   =======================================
//
// Check to see if the EDMA3 interrupt pending register (IPR) is set to one indicating
// that the transfer has completed. This is a blocking function - it will continue to
// poll the IPR bit and won't return until it becomes a 1.
//
// This function could be improved by returning the result. Also, adding a timeout
// feature would be an improvement.
// ===================================================================================

void edma_wait(EDMA3_DRV_Handle* hEdma, uint32_t* tccCh)
{
	EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;
    edma3Result = EDMA3_DRV_waitAndClearTcc(*hEdma, *tccCh);
    printError(&edma3Result, "EDMA3 wait");
}

// ============================    EDMA CHECK   ======================================
//
// Check to see if the EDMA3 interrupt pending register (IPR) is set to one indicating
// that the transfer has completed. This is a one-time check - not a blocking
// function. The calling code can check the status and respond accordingly.
//
// tccStatus = true or false
// ===================================================================================

unsigned short edma_check(EDMA3_DRV_Handle* hEdma, uint32_t* tccCh)
{
	unsigned short tccStatus = 0;
//	EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;

//	edma3Result =
	EDMA3_DRV_checkAndClearTcc(*hEdma, *tccCh, &tccStatus);

	return tccStatus;

}

// ============================    EDMA RELEASE   ======================================
//
// EDMA3_DRV_close is used to close an already opened EDMA3 Driver instance.
//
// EDMA3_DRV_delete deletes the driver instance and driver object.
// =====================================================================================

void edma_release(EDMA3_DRV_Handle* hEdma, unsigned int edma3Id)
{
	EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;

	edma3Result = EDMA3_DRV_close(*hEdma, NULL);
	printError(&edma3Result, "EDMA3 DRV closing");

	edma3Result = EDMA3_DRV_delete(edma3Id, NULL);
	printError(&edma3Result, "EDMA3 DRV deleting");

}


/*
void EDMA_TO_LSU()
{
	EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;

	CSL_edma3ChannelOpen();
    // Use TC 0
	EDMA3_DRV_chainChannel();
	edma3Result = CSL_edma3HwChannelSetupQue(hChannel1,CSL_EDMA3_QUE_3);
	printError(&edma3Result, "EDMA3 Open Channel");
}

*/

void printError(EDMA3_DRV_Result *isError, char currentFunction[64])
{
	if (verbose)
	{
		if (*isError != EDMA3_DRV_SOK)
		{
			printf("%s\tFAILED!\n\t\t|_ Error code: %d\r\n", currentFunction, *isError);
		}
		else
		{
			printf("%s\tSUCCEED!\r\n",currentFunction);
		}
	}
}


void EDMA_intr_init(EDMA3_DRV_Handle* hEdma, uint32_t eventChannel)
{

//	EDMA3_DRV_setCCRegister(hEdma, offsetReg, value);

//	EDMA3_DRV_registerTccCb (hEdma, channel, &test_isr_handler, NULL);

	if(eventChannel > 31)
		{
		eventChannel = 0x00000001 << (eventChannel - 32);
		//manually modify EDMA3CC registers
//		EDMA3_DRV_setCCRegister(*hEdma, IESRH_reg, eventChannel);	//IESR
		EDMA3_DRV_setCCRegister(*hEdma, EESRH_reg, eventChannel);	//EESR
		}
	else
		{
		eventChannel = 0x00000001 << eventChannel;
		//manually modify EDMA3CC registers
//		EDMA3_DRV_setCCRegister(*hEdma, IESR_reg, eventChannel);	//IESR
		EDMA3_DRV_setCCRegister(*hEdma, EESR_reg, eventChannel);	//EESR
		}




//	EDMA3_DRV_registerTccCb (hEdma, channel, &test_isr_handler, NULL);	//set the IER value of the channel
	/*
	unsigned int reg;

	//event number = bit number
	#define bitNum 0

	//Set IESR to auto-set IER to permit EESR to set an event
	reg = IESR_reg;
	CSL_FINSR(reg, bitNum, 0, 1);

	//Set EESR to auto-set EER to permit the event on ER register to occur
	reg = EESR_reg;
	CSL_FINSR(reg, bitNum, 0, 1);
*/
}


/* End of File */



