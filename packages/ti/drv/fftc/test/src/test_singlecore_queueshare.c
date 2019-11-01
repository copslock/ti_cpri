/**
 *   @file  test_singlecore_queueshare.c
 *
 *   @brief  
 *          Tests FFTC functionality on any single core using FFTC, CPPI, QMSS
 *          libraries. This test sets up 2 Tx, Rx task pairs one each for FFTC_A,
 *          and FFTC_B instances. The FFT requests submitted from both FFTC_A and
 *          FFTC_B are to be received on one single Rx queue.
 *
 *          For this the test sets up:-
 *          2 Rx tasks, one each for each of the FFTC instances, both using the same
 *          Rx queue, say 704.
 *
 *          2 Tx tasks, one each for each of the FFTC instances. Each Tx task sends
 *          data for processing by the flow set up for the FFTC instance it uses.
 *
 *          This test case is meant to emulate LTE downlink use case wherein 
 *          FFTC_A and FFTC_B could potentially be used to send data directly
 *          to AIF Tx queue.
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

/* Global variable to track the Rx, Tx task completion. 
 *
 * Used by main task to wait for completion of Rx, Tx tasks
 * before all system resources are de-initialized.
 */
static UInt8          	    isTestTaskDone  =   0;
static UInt8          	    isTestInitDone  =   0;
static UInt8                numTasksCreated =   0;

/* Rx queue number on which results from FFTC_A, FFTC_B results are to be received. */
#define FFTC_RX_QNUM                704

/* Accumulator channel number to use. */
#define FFTC_RX_ACC_CHANNEL_NUM     0

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
    Fftc_RxHandle       hRxObj
)
{
    Int16               eventId;

    /* Event ID corresponds to the accumulator channel */
    if (fftcInstNum == CSL_FFTC_0)
    {
        eventId     =   48;
    }
    else 
    {
        eventId     =   52;
    }

    /* Register the FFTC driver's high priority ISR handle for this event */
    EventCombiner_dispatchPlug (eventId, (EventCombiner_FuncPtr)Fftc_rxHiPriorityRxISR, (UArg)hRxObj, TRUE);
	EventCombiner_enableEvent(eventId);

    return;
}

 /* ============================================================================
 *  @n@b fftc_rx
 *
 *  @b  brief
 *  @n  This API sets up a FFTC receive object required for
 *      receiving FFT results from the engine. It validates all the results
 *      received and prints the result on completion.
 *
 *  @param[in]
 *  @n hFFTC                FFTC driver handle
 *
 *  @param[in]
 *  @n fftcInstNum          FFTC instance corresponding to this handle
 *
 *  @return     Int32
 *              -1      -   FFTC Rx test failed.
 *              0       -   FFTC Rx test successful.
 * ============================================================================
 */
static Int32 fftc_rx (Fftc_DrvHandle hFFTC, UInt8 fftcInstNum)
{
    Fftc_ResultHandle           hResultInfo;
    Fftc_RxHandle               hRxObj	=	NULL;
    Fftc_RxCfg                  rxCfg;    
    UInt32                      blockExpVal, clipping;
    UInt8                       *pResultBuffer, *pResultPSInfo;
    UInt32                      resultLen, rxPSInfoLen;
    Fftc_Result*                pFFTResult = NULL;
    Cplx16                      *xout, *xoutread;
    UInt16                      i, j, pktCounter = 0, rxDestnTagInfo;
    Int32                       retVal;
    UInt32                      bIsTestFailed = 0;
    UInt8                       coreNum = CSL_chipReadReg (CSL_CHIP_DNUM), rxFlowId, rxSrcId, bUseRxObjForGetResult = 0;
	Fftc_BlockInfo              blockInfo;
	FFT_TestCfg*            	pFFTTestCfg = NULL;
    static UInt8                bIsRxAccSetup;
	
    /* Allocate memory for the test configuration */
    if (!(pFFTTestCfg = (FFT_TestCfg *) Fftc_osalMalloc (sizeof (FFT_TestCfg), FALSE)))
    {
        Fftc_osalLog ("Error allocating memory for test configuration \n");
        return -1;
    }

    /* Read the FFT input vector and configuration for this test 
     * from the corresponding input configuration file.
     */
    if (fftc_parse_testCfg (1, pFFTTestCfg, &blockInfo) < 0)
    {
        Fftc_osalLog ("Error parsing test configuration \n");
        return -1;      
    }
    
    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC Rx QUEUE SHARING Test Task START on Core %d Instance: %d\n", coreNum, fftcInstNum);
    Fftc_osalLog ("Sample Size:             %d \n", pFFTTestCfg->numInputSamples[0]);
    Fftc_osalLog ("Number of Blocks:        %d \n", pFFTTestCfg->numBlocks);
    Fftc_osalLog ("Descriptor Type:         Host \n");
    Fftc_osalLog ("--------------------------------------------\n");             

    /* Setup a Rx flow. This flow will be used to retrieve FFT results
     * from a destination queue.
     *
     * Rx flow Configuration Params:
     * -----------------------------
     *      -   Use Host descriptors
     *      -   No PS Info (bPSInfoPresent = 0)
     *      -   Setup Rx object for interrupts + blocking mode
     *      -   Specify the Rx queue on which results to be received
     *      -   Create a new flow
     *      -   Pick a buffer size big enough to hold the error information and
     *          FFT result itself.
     *      -   Enable result parsing for this object          
     */
    memset (&rxCfg, 0, sizeof (rxCfg));
    rxCfg.useFlowId                         =   -1;     
    rxCfg.bManageRxFlowCfg                  =   1;  
    rxCfg.rxFlowCfg.drvCfg.descType         =   Cppi_DescType_HOST; 
    rxCfg.rxFlowCfg.drvCfg.cppiNumDesc      =   2; 

    /* Allocate result buffers big enough to hold:
     *
     * Per block result =   Per block error info (block exp val + clipping detect + srcId + flowId) (16 bytes) +
     *                      block output data (number of samples * sample size)
     */
    rxCfg.rxFlowCfg.drvCfg.bufferSize       =   (pFFTTestCfg->numOutputSamples[0] * FFTC_TEST_SAMPLE_SIZE + 16) * pFFTTestCfg->numBlocks;  
    rxCfg.rxFlowCfg.drvCfg.bPSInfoPresent   =   0; 
    rxCfg.rxFlowCfg.drvCfg.psLocation       =   Cppi_PSLoc_PS_IN_DESC;
    rxCfg.cppiRxQNum                        =   FFTC_RX_QNUM;  
    /* Accumulator must be programmed only once for any given 
     * channel/Rx queue. There can be only one Rx object 
     * retrieving the results at any point of time from 
     * any given Accumulator list. Hence, setup only one Rx
     * object as the primary one that retrieves the result.
     */
    if (!bIsRxAccSetup)
    {
        rxCfg.bUseInterrupts                =   0;
        rxCfg.bBlockOnResult                =   1;
        rxCfg.bManageAccumList              =   1;      /* Let driver manage accumulator list */
	    rxCfg.accumCfg.drvCfg.bEnablePacing	=	0;		/* Disable pacing */
	    rxCfg.accumCfg.drvCfg.intThreshold	=	1;		/* Set interrupt threshold to 1 */
        rxCfg.accumCfg.drvCfg.accChannelNum =   FFTC_RX_ACC_CHANNEL_NUM;
    
        /* Mark accumulator setup done so that we don't set it up again */
        bIsRxAccSetup ++;
        bUseRxObjForGetResult               =   1;
    }
    else
    {
        /* This Rx object wont be used to retrieve the results.
         * This will be used only to setup the required flow
         * to send results to Rx queue chosen.
         */
        rxCfg.bUseInterrupts                =   0;
        bUseRxObjForGetResult               =   0;
    }

    /* Get a Rx Object Handle */
    if ((hRxObj = Fftc_rxOpen (hFFTC, &rxCfg)) == NULL)
    {
        Fftc_osalLog ("[Rx Task %d]: Rx flow open failed \n", fftcInstNum);
        goto error;
    }
    else
    {
        Fftc_osalLog ("[Rx Task %d]: Rx Flow %d opened successfully using Rx queue %d \n", 
                       fftcInstNum, Fftc_rxGetFlowId (hRxObj), Fftc_rxGetRxQueueNumber (hRxObj));
    }  

	/* Register ISR for Rx */
    if (rxCfg.bUseInterrupts)
	    register_rx_interrupts (fftcInstNum, hRxObj);
    isTestInitDone ++;

    /* Wait until all Rx, Tx task init is not done before starting the 
     * pcket transfer.
     */
    while (isTestInitDone != numTasksCreated)
    {
        Task_yield ();            
    }

    /* Use Rx object created for retrieving results to wait on results */
    if (bUseRxObjForGetResult)
    {
        /* Run the test multiple times */
	    while (pktCounter != NUM_TEST_PACKETS * 2)
	    {
            /* -------------------------------------
             * Wait on FFT result and verify Result
             * -------------------------------------
             */
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("\n[Rx Task %d]: Waiting for Result ... \n", fftcInstNum);
#endif
        
        	/* Poll for the FFT result. 
         	 *	
         	 * Lets just wait for few iterations and declare an error if we dont see 
         	 * a result otherwise.
         	 */
        	if (!rxCfg.bUseInterrupts)
        	{
        		while (!Fftc_rxGetNumPendingResults (hRxObj))
        		{
            		Task_yield ();        
        		}
        	}
        
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
                Fftc_osalLog ("[Rx Task %d]: Invalid FFT result : %d \n", fftcInstNum, retVal);
                goto error;
            }
//#ifdef FFTC_TEST_DEBUG            
            Fftc_osalLog ("\n[Rx Task %d]: Got FFT Result %d !!\n", fftcInstNum, rxDestnTagInfo);
//#endif

            /* Allocate memory to hold the formatted result */
            pFFTResult = (Fftc_Result *) Fftc_osalMalloc (sizeof (Fftc_Result), FALSE);
    
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
                Fftc_osalLog ("[Rx Task %d]: Error parsing result : %d  error: %d \n", fftcInstNum, pktCounter, retVal);
                goto error;
            }

            /* Verify result */
#ifdef  FFTC_TEST_DEBUG
            Fftc_osalLog("[Rx Task %d]: Result Info:: DestnTagInfo: %d Error detected: %d Number of FFTC Result Blocks: %d \n", 
                        fftcInstNum, pFFTResult->destnTagInfo, pFFTResult->bErrorDetected, pFFTResult->numFFTCBlocks);
#endif

            /* Get the per block result and compare against the expected values
             * to see if the FFT operation succeeded.
             */
            for (j = 0; j < pFFTResult->numFFTCBlocks; j++)
            {
#ifdef  FFTC_TEST_DEBUG
                Fftc_osalLog ("\n[Rx Task %d]: ********* Block %d Result *********\n", fftcInstNum, j);
#endif

                /* Results got */                
                xout            =   (Cplx16 *) pFFTResult->fftBlockResult [j].pBuffer;
                blockExpVal     =   pFFTResult->fftBlockResult [j].blockExponentVal;
                clipping        =   pFFTResult->bClippingDetected;

                /* Expected result */
                xoutread        =   (Cplx16 *) pFFTTestCfg->pFftOutputData [j];

#ifdef  FFTC_TEST_DEBUG
                Fftc_osalLog ("\n[Rx Task %d]: FFT Result details: blockExp: %d Clipping: %d Result buffer len: %d \n",  
                                fftcInstNum, blockExpVal, clipping, pFFTResult->fftBlockResult [j].bufferLen);

                Fftc_osalLog ("\n[Rx Task %d]: Comparing FFT Result from engine against expected ... \n", fftcInstNum);
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
                        Fftc_osalLog ("[Rx Task %d]: Real data sample %d: test failed, read: %d actual: %d \n", 
                                        fftcInstNum, i, xout[i].real, xoutread[i].real);
                        Fftc_osalLog ("[Rx Task %d]: Imag data sample %d: test failed, read: %d actual: %d \n",
                                        fftcInstNum, i, xout[i].imag, xoutread[i].imag);
#endif
                    
                        bIsTestFailed ++;
                    }
                }

#ifdef  FFTC_TEST_DEBUG
                if (!bIsTestFailed)
                    Fftc_osalLog ("[Rx Task %d]: FFT Result Correct !! \n", fftcInstNum);
                else
                    Fftc_osalLog ("[Rx Task %d]: FFT Result Wrong !! \n", fftcInstNum);			                
#endif

                /* Compare the block exponent value got against expected value */
                if (!pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && blockExpVal != pFFTTestCfg->blockExpVal [j])
                {
#ifdef  FFTC_TEST_DEBUG
                    Fftc_osalLog("[Rx Task %d]: Block Exponent mismatch: test failed, read: %d actual: %d \n", 
                                fftcInstNum, blockExpVal, pFFTTestCfg->blockExpVal [j]);
#endif
                    bIsTestFailed ++;
                }
                else
                {
#ifdef  FFTC_TEST_DEBUG
                    Fftc_osalLog ("[Rx Task %d]: Block Exponent Value Correct !! \n", fftcInstNum);
#endif
                }

                /* Compare the clipping detected value got against expected value */
                if (!pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && clipping != pFFTTestCfg->bClippingDetected [j])
                {
#ifdef  FFTC_TEST_DEBUG
                    Fftc_osalLog("[Rx Task %d]: Clipping Detect mismatch: test failed, read: %d actual: %d \n", 
                                fftcInstNum, clipping, pFFTTestCfg->bClippingDetected [j]);
#endif
                    bIsTestFailed ++;
                }
                else
                {
#ifdef  FFTC_TEST_DEBUG
                    Fftc_osalLog ("[Rx Task %d]: Clipping Detect Value Correct !! \n", fftcInstNum);
#endif
                }
            }

            /* Done using the result buffer, return it to the FFTC driver for recycling */
            if (Fftc_rxFreeResult (hRxObj, 
                                hResultInfo
                                ) < 0)
            {
                Fftc_osalLog ("[Rx Task %d]: Error freeing result : %d \n", fftcInstNum, i);
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
    }
    else
    {
        /* This Rx object is not used for result retrieval. Was just
         * created to create the necessary Rx flows required. Keep this
         * around until all Tx, Rx tasks are not done.
         */
        while (isTestTaskDone != (numTasksCreated - 1))
        	Task_yield ();
    }

    if (bIsTestFailed == 0)
    {
        Fftc_osalLog ("\n[Rx Task %d]: FFTC Test PASS !! FFT Requests processed = %d\n", fftcInstNum, pktCounter);   
        totalNumTestsPass ++;             
    }
    else
    {
        Fftc_osalLog ("\n[Rx Task %d]: FFTC Test FAILED !! Num Tests Failed = %d \n", fftcInstNum, bIsTestFailed);     
        totalNumTestsFail ++;           
    }

    /* Free the FFT input, output data and test configuration memory allocated */
    fftc_clean_testCfg (pFFTTestCfg, &blockInfo);
    
    /* Close all FFTC handles */
    if (hRxObj)
    	Fftc_rxClose (hRxObj);

    /* Increment the test task complete flag. */
    isTestTaskDone ++;

    Task_exit ();

	/* Return success. */
	return 0;        

error:
    /* Free the FFT input, output data and test configuration memory allocated */
    fftc_clean_testCfg (pFFTTestCfg, &blockInfo);
    
    /* Free the FFT result info memory */
    if (pFFTResult)
        Fftc_osalFree (pFFTResult, sizeof (Fftc_Result), FALSE);

    /* Close all FFTC handles */
    if (hRxObj)
    	Fftc_rxClose (hRxObj);

    /* Increment the test task complete flag. */
    isTestTaskDone ++;

    Task_exit ();
    
    totalNumTestsFail ++;

    /* Return error */
    return -1;
}


/**
 * ============================================================================
 *  @n@b fftc_tx
 *
 *  @b  brief
 *  @n  This API sets up the FFTC transmit object required for
 *      submitting FFT requests to engine using the driver and sends data 
 *      to the engine for processing.
 *
 *  @param[in]
 *  @n hFFTC                FFTC driver handle
 *
 *  @param[in]
 *  @n fftcInstNum          FFTC instance corresponding to this handle
 *
 *  @return     Int32
 *              -1      -   FFTC test failed.
 *              0       -   FFTC test successful.
 * ============================================================================
 */
static Int32 fftc_tx (Fftc_DrvHandle hFFTC,UInt8 fftcInstNum)
{
    Fftc_RequestHandle          hRequestInfo;
    Fftc_TxHandle               hTxObj 	= 	NULL;
    Fftc_TxCfg                  txCfg;    
    UInt8                       *pReqBuffer;
    UInt32                      maxReqBufferLen, reqBufferLen, txPSInfoLen = 0, qNum;
    Int16                      	i, pktCounter = 0;
    UInt32                      blockDataOffset = 0;
    UInt8                       coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
    Int8               			rxFlowId;
	Fftc_BlockInfo              blockInfo;
	FFT_TestCfg*            	pFFTTestCfg = NULL;
    UInt8	                    txQNum = 0;
	
    /* Allocate memory for the test configuration */
    if (!(pFFTTestCfg = (FFT_TestCfg *) Fftc_osalMalloc (sizeof (FFT_TestCfg), FALSE)))
    {
        Fftc_osalLog ("Error allocating memory for test configuration \n");
        return -1;
    }

    /* Read the FFT input vector and configuration for this test 
     * from the corresponding input configuration file.
     */
    if (fftc_parse_testCfg (1, pFFTTestCfg, &blockInfo) < 0)
    {
        Fftc_osalLog ("Error parsing test configuration \n");
        return -1;         
    }
    
    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC Tx QUEUE SHARING Test Task START on Core %d using Instance: %d\n", coreNum, fftcInstNum);
    Fftc_osalLog ("Sample Size:             %d \n", pFFTTestCfg->numInputSamples[0]);
    Fftc_osalLog ("Number of Blocks:        %d \n", pFFTTestCfg->numBlocks);
    Fftc_osalLog ("Tx Queue:                %d \n", txQNum);
    Fftc_osalLog ("Descriptor Type:         Host \n");
    Fftc_osalLog ("--------------------------------------------\n");         

    /* Setup a Tx Object. This object will be used to submit FFT requests.
     *
     * Tx object Configuration Params:
     * -----------------------------
     *      -   Let driver manage Tx Free Desc and buffers (bManageReqBuffers = 1)
     *      -   Use Host descriptors
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
    txCfg.descType              =   Cppi_DescType_HOST; 
    txCfg.cppiNumDesc           =   2; 
    txCfg.bufferSize            =   pFFTTestCfg->numInputSamples[0] * FFTC_TEST_SAMPLE_SIZE * pFFTTestCfg->numBlocks;  
    txCfg.bPSInfoPresent        =   0; 

    /* Get a Tx Object Handle */
    if ((hTxObj = Fftc_txOpen (hFFTC, &txCfg)) == NULL)
    {
        Fftc_osalLog ("[Tx Task %d]: Tx open failed \n", fftcInstNum);
        goto error;
    }
    
    isTestInitDone ++;

    /* Wait until all Rx, Tx task init is not done before starting the 
     * pcket transfer.
     */
    while (isTestInitDone != numTasksCreated)
    {
        Task_yield ();            
    }
    
    /* -------------------------------------
     * Submit FFT request 
     * -------------------------------------
     */
	while (pktCounter != NUM_TEST_PACKETS)
	{
        /* Initialize PS Info length, request buffer data offset */            
        txPSInfoLen 	=   0;
        blockDataOffset	=	0;

    	/* Setup the result receiver. 
     	 *
     	 * We need to find a flow that can receive FFT
     	 * results for the requests we submit on the
     	 * Rx queue specified in 'FFTC_RX_QNUM'.
     	 */
        qNum =     FFTC_RX_QNUM;
			 	 		
    	if ((rxFlowId = Fftc_findFlowIdByQueueNumber (hFFTC, qNum)) < 0)
    	{
        	Fftc_osalLog ("[Tx Task %d]: No flow found configured on Rx queue %d \n", qNum, fftcInstNum);
        	pktCounter ++;
        	continue;
    	}
    	
        /* Get a request buffer */
        if (Fftc_txGetRequestBuffer (hTxObj, 
                                    &blockInfo, 
                                    &pFFTTestCfg->fftcQCfg, 
                                    txPSInfoLen,                                 
                                    rxFlowId,
                                    pktCounter,
                                    &hRequestInfo, 
                                    &pReqBuffer, 
                                    &maxReqBufferLen) < 0)
        {
            Fftc_osalLog ("[Tx Task %d]: Unable to get request buffer \n", fftcInstNum);
            goto error;
        }

        /* Initialize the whole FFT request buffer */
        memset (pReqBuffer, 0, maxReqBufferLen);

        /* The request buffer MUST be populated in the following order:
         *
         * <PS Info to be passed to receiver> then followed by <FFT request data>
         */
        /* We have no Protocol Specific Pass through data to pass to receiver in this test.
         * So just copy the FFT request data itself into the request buffer, block by block 
         */
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
        Fftc_osalLog ("\n[Tx Task %d]: Submitting FFT Request ... \n", fftcInstNum);
#endif

        /* Submit the FFT request for processing */
        if (Fftc_txSubmitRequest (hTxObj, hRequestInfo, reqBufferLen) < 0)
        {
            Fftc_osalLog ("[Tx Task %d]: Unable to submit request \n", coreNum, fftcInstNum);
            goto error;
        }
        else
        {
//#ifdef FFTC_TEST_DEBUG        	
            Fftc_osalLog ("[Tx Task %d]: Submitted FFT request, Id: %d for processing by Flow %d RxQNum: %d \n", fftcInstNum, 
            			   pktCounter, rxFlowId, qNum);
//#endif               
        }

	    pktCounter ++;

        /* Pause before another request submission */
        Task_sleep (1);
	}

    /* Free the FFT input, output data and test configuration memory allocated */
    fftc_clean_testCfg (pFFTTestCfg, &blockInfo);
    	
    /* Close all FFTC handles */
	if (hTxObj)    	
    	Fftc_txClose (hTxObj);

    /* Increment the test task complete flag. */
    isTestTaskDone ++;

	totalNumTestsPass ++;
	
    Task_exit ();

	/* Return success. */
	return 0;        

error:
    /* Free the FFT input, output data and test configuration memory allocated */
    fftc_clean_testCfg (pFFTTestCfg, &blockInfo);
    
    /* Close all FFTC handles */
	if (hTxObj)    	
    	Fftc_txClose (hTxObj);

    /* Increment the test task complete flag. */
    isTestTaskDone ++;

	totalNumTestsFail ++;
	
    Task_exit ();

    /* Return error */
    return -1;
}


/**
 * ============================================================================
 *  @n@b test_host_singlecore_queueshare
 *
 *  @b  brief
 *  @n  This API sets up the test parameters, calls the test core API to 
 *      actually run the test.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void test_host_singlecore_queueshare
(
    UInt8                   fftcInstNum,
    Fftc_DrvHandle			hFFTC
)
{
    Task_Params             testTaskParams;

    /* Create the FFTC Rx test task */
    Task_Params_init(&testTaskParams);
    testTaskParams.arg0     =   (UInt32) hFFTC;
    testTaskParams.arg1     =   (UInt32) fftcInstNum;
    
    /* Create the Rx task */
    Task_create ((Task_FuncPtr)&fftc_rx, &testTaskParams, NULL);
    numTasksCreated ++;

    /* Create the FFTC Tx test task */
    Task_Params_init(&testTaskParams);
    testTaskParams.arg0     =   (UInt32) hFFTC;
    testTaskParams.arg1     =   (UInt32) fftcInstNum;
    
    /* Create the Tx task */
    Task_create ((Task_FuncPtr)&fftc_tx, &testTaskParams, NULL);
    numTasksCreated ++;

	/* Wait till all the Rx, Tx tasks are not done. */
    while (isTestTaskDone != numTasksCreated)
    {
        Task_yield ();            
    }

    return;
}
