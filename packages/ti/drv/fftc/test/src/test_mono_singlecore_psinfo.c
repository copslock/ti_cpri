/** 
 *   @file  test_singlecore.c
 *
 *   @brief  
 *          Tests FFTC functionality on any single core using FFTC, CPPI, QMSS
 *          libraries.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
/* FFTC test application include */
#include "fftc_test.h"

/**************************************************************
************************* GLOBAL VARIABLES ********************
***************************************************************/
extern UInt32               totalNumTestsPass, totalNumTestsFail;

/**************************************************************
************************* TEST FUNCTIONS **********************
***************************************************************/

/* ============================================================================
 *  @n@b register_rx_interrupts
 *
 *  @b  brief
 *  @n  Utility function to register interrupts using BIOS APIs.
 *
 *  @param[in]
 *  @n fftcInstNum          FFTC instance corresponding to this handle
 *
 *  @param[in]
 *  @n hRxObj               Rx object handle 
 *
 *  @return     Int32
 *              -1      -   FFTC Rx test failed.
 *              0       -   FFTC Rx test successful.
 * ============================================================================
 */
static void register_rx_interrupts 
(
    UInt8               fftcInstNum,
    Fftc_RxHandle		hRxObj
)
{
    Int16               eventId;

    /* Map the FFTC system event number corresponding to the FFTC instance
     * we are using here to a interrupt vector to hook up interrupts on.
     */
    if (fftcInstNum == CSL_FFTC_0)
    {
        eventId     =   49;
    }
    else 
    {
        eventId     =   51;
    }

    /* Register the FFTC driver's high priority ISR handle for this event */
    EventCombiner_dispatchPlug (eventId, (EventCombiner_FuncPtr)Fftc_rxHiPriorityRxISR, (UArg)hRxObj, TRUE);
	EventCombiner_enableEvent(eventId);

    return;
}

/**
 * ============================================================================
 *  @n@b fftc_test_core
 *
 *  @b  brief
 *  @n  This API sets up the FFTC transmit, receive objects required for
 *      running the test on a given core and runs the following test:
 *
 *  @param[in]  
 *  @n  hFFTC               FFTC Driver handle
 *
 *  @param[in]  
 *  @n  txQNum              Tx queue number to run test on
 *
 *  @param[in]  
 *  @n  testCaseId          Test case to run
 *
 *  @return     Int32
 *              -1      -   FFTC test failed.
 *              0       -   FFTC test successful.
 * ============================================================================
 */
static Int32 fftc_test_core (Fftc_DrvHandle hFFTC, UInt32 txQNum, UInt32 testCaseId)
{
    Fftc_RequestHandle          hRequestInfo;
    Fftc_ResultHandle           hResultInfo;
    Fftc_TxHandle               hTxObj 	= 	NULL;
    Fftc_RxHandle               hRxObj	=	NULL;
    Fftc_TxCfg                  txCfg;    
    Fftc_RxCfg                  rxCfg;    
    UInt32                      blockExpVal, clipping;
    Fftc_BlockInfo              blockInfo;
    UInt8                       *pReqBuffer, *pResultBuffer, *pResultPSInfo, rxFlowId, rxSrcId;
    UInt32                      maxReqBufferLen, reqBufferLen, resultLen, rxPSInfoLen, txPSInfoLen = 0;
    Fftc_Result*                pFFTResult = NULL;
    Cplx16                      *xout, *xoutread;
    UInt16                      i, j, pktCounter = 0, rxDestnTagInfo;
    Int32                       retVal;
    UInt32                      bIsTestFailed = 0;
    FFT_TestCfg*                pFFTTestCfg = NULL;
    UInt32                      blockDataOffset = 0;
    UInt8                       coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
    UInt32                      ReqPSInfo [FFTC_MAX_NUM_PS_WORDS] = {0xDEAD1234, 0xBEEF5678, 0xABCD1234, 0x1234ABCD};

    /* Allocate memory for the test configuration */
    if (!(pFFTTestCfg = (FFT_TestCfg *) Fftc_osalMalloc (sizeof (FFT_TestCfg), FALSE)))
    {
        Fftc_osalLog ("[Core %d]: Error allocating memory for test configuration \n", coreNum);
        return -1;
    }

    /* Read the FFT input vector and configuration for this test 
     * from the corresponding input configuration file.
     */
    if (fftc_parse_testCfg (testCaseId, pFFTTestCfg, &blockInfo) < 0)
    {
        return -1;         
    }

    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC-CPPI SINGLECORE MONOLITHIC PS Test START on Core %d \n", coreNum);
    Fftc_osalLog ("Sample Size:             %d \n", pFFTTestCfg->numInputSamples[0]);
    Fftc_osalLog ("Number of Blocks:        %d \n", pFFTTestCfg->numBlocks);
    Fftc_osalLog ("Tx Queue:                %d \n", txQNum);
    Fftc_osalLog ("Descriptor Type:         Monolithic \n");
    Fftc_osalLog ("--------------------------------------------\n");         

    /* Setup a Rx flow. This flow will be used to retrieve FFT results
     * from a destination queue.
     *
     * Rx flow Configuration Params:
     * -----------------------------
     *      -   Use Monolithic descriptors
     *      -   No PS Info (bPSInfoPresent = 0)
     *      -   Setup Rx object for interrupts + blocking mode
     *      -   Let FFTC driver pick the destination queue by 
     *          calling Fftc_getDeviceAccumulatorConfig()
     *      -   Pick a buffer size big enough to hold the error information and
     *          FFT result itself.
     */
    memset (&rxCfg, 0, sizeof (rxCfg));
    rxCfg.useFlowId				            =	-1;
    rxCfg.bManageRxFlowCfg                  =   1;
    rxCfg.rxFlowCfg.drvCfg.descType         =   Cppi_DescType_MONOLITHIC; 
    rxCfg.rxFlowCfg.drvCfg.cppiNumDesc      =   2; 

    /* Allocate result buffers big enough to hold:
     *
     * Per block result =   Per block error info (block exp val + clipping detect + srcId + flowId) (16 bytes) +
     *                      block output data (number of samples * sample size)
     */
    rxCfg.rxFlowCfg.drvCfg.bufferSize       =   (pFFTTestCfg->numOutputSamples[0] * FFTC_TEST_SAMPLE_SIZE + 16) * pFFTTestCfg->numBlocks;  
    rxCfg.rxFlowCfg.drvCfg.bPSInfoPresent   =   1; 
    rxCfg.rxFlowCfg.drvCfg.psLocation       =   Cppi_PSLoc_PS_IN_SOP;
    rxCfg.cppiRxQNum                        =   -1;
    rxCfg.bBlockOnResult                    =   1;
    rxCfg.bUseInterrupts                    =   1;
    rxCfg.bManageAccumList                  =   1;      /* Let driver do the Accum Management */          
	rxCfg.accumCfg.drvCfg.bEnablePacing	    =	0;		/* Disable pacing */
	rxCfg.accumCfg.drvCfg.intThreshold		=	1;		/* Set interrupt threshold to 1 */
    Fftc_getDeviceAccumulatorConfig (CSL_FFTC_0, &rxCfg.accumCfg.drvCfg.accChannelNum, (UInt32*)&rxCfg.cppiRxQNum);

    /* Get a Rx Object Handle */
    if ((hRxObj = Fftc_rxOpen (hFFTC, &rxCfg)) == NULL)
    {
        Fftc_osalLog ("[Core %d]: Rx flow open failed \n", coreNum);
        goto error;
    }
    else
    {
        Fftc_osalLog ("[Core %d]: Rx Flow %d opened successfully using Rx queue %d \n", coreNum, 
                        Fftc_rxGetFlowId (hRxObj), Fftc_rxGetRxQueueNumber (hRxObj));
    }  
   
    register_rx_interrupts (CSL_FFTC_0, hRxObj);

    /* Setup a Tx Object. This object will be used to submit FFT requests.
     *
     * Tx object Configuration Params:
     * -----------------------------
     *      -   Pre-alloc Tx Free Desc (bManageReqBuffers = 1)
     *      -   Use Monolithic descriptors
     *      -   Pick a buffer size big enough to hold all the
     *          FFT request blocks.
     *      -   No PS Info (bPSInfoPresent = 0)
     *      -   No mixed size DFTs (bEnableDftSizeListCfg = 0)
     *      -   Open queue in shared mode (bSharedMode = 1). 
     *          Enables re-programming queue using CPPI.
     */
    memset (&txCfg, 0, sizeof (txCfg));
    txCfg.txQNum                =   (Fftc_QueueId) txQNum; 
    txCfg.bManageReqBuffers     =   1; 
    txCfg.bEnableDftSizeListCfg =   0; 
    txCfg.fftQCfg               =   pFFTTestCfg->fftcQCfg;
    txCfg.bSharedMode           =   1;
    txCfg.descType              =   Cppi_DescType_MONOLITHIC; 
    txCfg.cppiNumDesc           =   2; 
    txCfg.bufferSize            =   pFFTTestCfg->numInputSamples[0] * FFTC_TEST_SAMPLE_SIZE * pFFTTestCfg->numBlocks;  
    txCfg.bPSInfoPresent        =   1; 

    /* Get a Tx Object Handle */
    if ((hTxObj = Fftc_txOpen (hFFTC, &txCfg)) == NULL)
    {
        Fftc_osalLog ("[Core %d]: Tx open failed \n", coreNum);
        goto error;
    }

    /* -------------------------------------
     * Submit FFT request 
     * -------------------------------------
     */
    /* Run the test in a loop */
	while (pktCounter != NUM_TEST_PACKETS)
	{
        /* Lets send 4 words of PS data to the receiver */            
        txPSInfoLen =  FFTC_MAX_NUM_PS_WORDS * sizeof(UInt32);

        /* Initialize the request buffer offset */
        blockDataOffset	=	0;

        /* Get a request buffer */
        if (Fftc_txGetRequestBuffer (hTxObj, 
                                    &blockInfo, 
                                    &pFFTTestCfg->fftcQCfg, 
                                    txPSInfoLen,   
                                    Fftc_rxGetFlowId (hRxObj),                            
                                    pktCounter,
                                    &hRequestInfo, 
                                    &pReqBuffer, 
                                    &maxReqBufferLen) < 0)
        {
            Fftc_osalLog ("[Core %d]: Unable to get request buffer \n", coreNum);
            goto error;
        }

        /* Initialize the whole FFT request buffer */
        memset (pReqBuffer, 0, maxReqBufferLen);

        /* The request buffer MUST be populated in the following order:
         *
         * <PS Info to be passed to receiver> then followed by <FFT request data>
         */
        memcpy ((Ptr) pReqBuffer, (Ptr) ReqPSInfo, txPSInfoLen);
        blockDataOffset +=  txPSInfoLen;

        reqBufferLen                =   pFFTTestCfg->numBlocks * pFFTTestCfg->numInputSamples[0] * FFTC_TEST_SAMPLE_SIZE;   
        for (i = 0; i < pFFTTestCfg->numBlocks; i ++)
        {
            /* Copy the FFT request block 
             * Each block size = number of samples * size of each sample
             *                 = pFFTTestCfg->numInputSamples * FFTC_TEST_SAMPLE_SIZE
             */                
            memcpy ((Ptr) (pReqBuffer + blockDataOffset), 
                    (Ptr) pFFTTestCfg->pFftInputData [i], 
                    pFFTTestCfg->numInputSamples[0] * FFTC_TEST_SAMPLE_SIZE);
        
            /* Increment the data offset so that the next block data can 
             * be copied to the correct location in request buffer.
             */
            blockDataOffset +=  (pFFTTestCfg->numInputSamples[0] * FFTC_TEST_SAMPLE_SIZE);
        }

#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("\n[Core %d]: Submitting FFT Request ... \n", coreNum);
#endif

        /* Submit the FFT request for processing */
        if (Fftc_txSubmitRequest (hTxObj, 
                                  hRequestInfo, 
                                  reqBufferLen) < 0)
        {
            Fftc_osalLog ("[Core %d]: Unable to submit request \n", coreNum);
            goto error;
        }
        else
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("Submitted request %d successfully \n", pktCounter);       
#endif
        }

        /* -------------------------------------
         * Wait on FFT result and verify Result
         * -------------------------------------
         */
#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("\n[Core %d]: Waiting for Result ... \n", coreNum);
#endif

        /* Get the raw result from the engine. */   
        if ((retVal = Fftc_rxGetResult (hRxObj, 
                                        &hResultInfo,
                                        &pResultBuffer, 
                                        &resultLen, 
                                        &pResultPSInfo, 
                                        &rxPSInfoLen,
                                        &rxFlowId,
                                        &rxSrcId,
                                        &rxDestnTagInfo
                                        )) != FFTC_RETVAL_SUCCESS)
        {
            Fftc_osalLog ("[Core %d]: Invalid FFT result : %d \n", coreNum, retVal);
            goto error;
        }

        /* Allocate memory to hold the formatted result */
        pFFTResult = (Fftc_Result *) Fftc_osalMalloc (sizeof (Fftc_Result), FALSE);

        /* If we had sent PS info in request, verify that we received it 
         * back correctly too.
         */
        if (txPSInfoLen)
        {
            /* Verify PS info received */                
            if ((rxPSInfoLen != txPSInfoLen) ||
                (memcmp (pResultPSInfo, ReqPSInfo, txPSInfoLen)))
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog ("[Core %d]: PS Info Mismatch! PS Info len sent: %d received: %d \n",
                             coreNum, txPSInfoLen, rxPSInfoLen);
#endif
				bIsTestFailed ++;
            }
            else
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog ("[Core %d]: PS Info Correct!! \n", coreNum);
#endif
            }
        }
    
        /* Get the formatted result. */
        if ((retVal = Fftc_rxParseResult (hRxObj, 
                                          hResultInfo, 
                                            pResultBuffer, 
                                            resultLen, 
                                            &blockInfo,
                                            pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo,
                                            pFFTTestCfg->fftcQCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum,
                                          pFFTResult)) != FFTC_RETVAL_SUCCESS)
        {
            Fftc_osalLog ("[Core %d]: Error parsing result, error: %d \n", coreNum, retVal);
            goto error;
        }

        /* verify result */
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog("[Core %d]: Result Info:: DestnTag: %d Error detected: %d Number of FFTC Result Blocks: %d \n", coreNum, 
                        pFFTResult->destnTagInfo, pFFTResult->bErrorDetected, pFFTResult->numFFTCBlocks);
#endif

        /* Get the per block result and compare against the expected values
         * to see if the FFT operation succeeded.
         */
        for (j = 0; j < pFFTResult->numFFTCBlocks; j++)
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("\n[Core %d]: ********* Block %d Result *********\n", coreNum, j);
#endif

            /* Results got */                
            xout            =   (Cplx16 *) pFFTResult->fftBlockResult [j].pBuffer;
            blockExpVal     =   pFFTResult->fftBlockResult [j].blockExponentVal;
            clipping        =   pFFTResult->bClippingDetected;

            /* Expected result */
            xoutread        =   (Cplx16 *) pFFTTestCfg->pFftOutputData [j];

#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("\n[Core %d]: FFT Result details: blockExp: %d Clipping: %d Result buffer len: %d \n", coreNum, 
                            blockExpVal, clipping, pFFTResult->fftBlockResult [j].bufferLen);

            Fftc_osalLog ("\n[Core %d]: Comparing FFT Result from engine against expected ... \n", coreNum);
#endif

            /* Compare the FFT result data block against expected */
	        for (i = 0; i < pFFTTestCfg->numOutputSamples[0]; i++)
	        {
                /* Ignore LSB comparison. 
                 *
                 * Since the reference data is generated using
                 * MATLAB, there could be an error margin of +/-1 between the
                 * reference and actual received FFTC output.
                 *
                 * For now, mask off the LSB and compare for
                 * correctness of the FFT processing.
                 */
                if (((xout[i].real != xoutread[i].real) && 
                    ((xout[i].real - xoutread[i].real != -1) && (xout[i].real - xoutread[i].real != 1))) ||
                    ((xout[i].imag != xoutread[i].imag) && 
                    ((xout[i].imag - xoutread[i].imag != -1) && (xout[i].imag - xoutread[i].imag != 1))))
                {
#ifdef FFTC_TEST_DEBUG
                    Fftc_osalLog ("[Core %d]: Real data sample %d: test failed, read: %d actual: %d \n", coreNum
                                    , i, xout[i].real, xoutread[i].real);
                    Fftc_osalLog ("[Core %d]: Imag data sample %d: test failed, read: %d actual: %d \n", coreNum
                                    , i, xout[i].imag, xoutread[i].imag);
#endif
                    
                    bIsTestFailed ++;
                }
            }

#ifdef FFTC_TEST_DEBUG
            if (!bIsTestFailed)
                Fftc_osalLog ("[Core %d]: FFT Result Correct !! \n", coreNum);
            else
                Fftc_osalLog ("[Core %d]: FFT Result Wrong !! \n", coreNum);			                
#endif

            /* Compare the block exponent value got against expected value */
            if (!pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && blockExpVal != pFFTTestCfg->blockExpVal [j])
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog("[Core %d]: Block Exponent mismatch: test failed, read: %d actual: %d \n", coreNum
                            , blockExpVal, pFFTTestCfg->blockExpVal [j]);
#endif
                bIsTestFailed ++;
            }
            else
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog ("[Core %d]: Block Exponent Value Correct !! \n", coreNum);
#endif
            }

            /* Compare the clipping detected value got against expected value */
            if (!pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && clipping != pFFTTestCfg->bClippingDetected [j])
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog("[Core %d]: Clipping Detect mismatch: test failed, read: %d actual: %d \n", coreNum
                                ,clipping, pFFTTestCfg->bClippingDetected [j]);
#endif
                bIsTestFailed ++;
            }
            else
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog ("[Core %d]: Clipping Detect Value Correct !! \n", coreNum);
#endif
            }
        }

        /* Done using the result buffer, return it to the FFTC driver for recycling */
        if (Fftc_rxFreeResult (hRxObj, 
                               hResultInfo
                               ) < 0)
        {
            Fftc_osalLog ("[Core %d]: Error freeing result : %d \n", coreNum, i);
            goto error;
        }

		/* Free the formatted FFT result info memory */
		if (pFFTResult)
        {
			Fftc_osalFree (pFFTResult, sizeof (Fftc_Result), FALSE);
            pFFTResult = NULL;
        }
			
	    pktCounter ++;
	}
    	
	if (bIsTestFailed == 0)
	{
		Fftc_osalLog ("\n[Core %d]: FFTC Test PASS !! Processed FFT Requests = %d \n", coreNum, pktCounter);                
	}
	else
	{
		Fftc_osalLog ("\n[Core %d]: FFTC Test FAILED !! Num Tests Failed = %d \n", coreNum, bIsTestFailed);                
	}

    /* Free the FFT input, output data and test configuration memory allocated */
    fftc_clean_testCfg (pFFTTestCfg, &blockInfo);

    /* Close all FFTC handles */
    if (hRxObj)
    	Fftc_rxClose (hRxObj);

	if (hTxObj)    	
    	Fftc_txClose (hTxObj);

	/* Return success. */
	return 0;        

error:
    /* Free the FFT result info memory */
    if (pFFTResult)
        Fftc_osalFree (pFFTResult, sizeof (Fftc_Result), FALSE);

    /* Free the FFT input, output data and test configuration memory allocated */
    fftc_clean_testCfg (pFFTTestCfg, &blockInfo);

    /* Close all FFTC handles */
    if (hRxObj)
    	Fftc_rxClose (hRxObj);

	if (hTxObj)    	
    	Fftc_txClose (hTxObj);

    /* Return error */
    return -1;
}


/**
 * ============================================================================
 *  @n@b test_mono_singlecore_psinfo
 *
 *  @b  brief
 *  @n  This API sets up the test environment, test parameters, calls the test 
 *      core API to actually run the test and finally de-initializes the system.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void test_mono_singlecore_psinfo
(
    Fftc_DrvHandle          hFFTC
)
{
    UInt32                  testNum, queueNum;

    /* Run the following tests on all the 4 Tx queues:
     *
     *  Test Num					Description
     *  ---------                   ----------
     *  1 							(1 packet, 16 sample size, 5 blocks) 
     *  2 							(1 packet, 48 sample size, 5 blocks) 
     *  3 							(1 packet, 540 sample size, 3 blocks) 
     *  4 							(1 packet, 2048 sample size, 1 block) 
     */
    for (queueNum = 0; queueNum < FFTC_MAX_NUM_TXQUEUES; queueNum ++)
    {
    	for (testNum = 1; testNum <= 2; testNum ++)
    	{
    		if (fftc_test_core (hFFTC, queueNum, testNum) < 0)
            {
                totalNumTestsFail ++;                    
            }
            else
            {
                totalNumTestsPass ++;
            }
    	}
    }
    
    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC-CPPI SINGLECORE MONOLITHIC PS Test END\n");
    Fftc_osalLog ("--------------------------------------------\n");        

    return;
}
