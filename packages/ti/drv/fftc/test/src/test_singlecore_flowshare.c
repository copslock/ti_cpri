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

/* Global variable to track the Rx, Tx task completion. 
 *
 * Used by main task to wait for completion of Rx, Tx tasks
 * before all system resources are de-initialized.
 */
static UInt8                isTestTaskDone  =   0;
static UInt8                isTestInitDone  =   0;

/* Rx queue number to use for test. */
#ifdef DEVICE_K2L
#define FFTC_RX_QNUM                708
#define FFTC_RX_NEXT_QNUM_INC       4
#else
#define FFTC_RX_QNUM                720
#define FFTC_RX_NEXT_QNUM_INC       8
#endif

/* Accumulator channel number to use. */
#ifdef DEVICE_K2L
#define FFTC_RX_ACC_CHANNEL_NUM     8
#else
#define FFTC_RX_ACC_CHANNEL_NUM     16
#endif
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
 *  @n rxId                 Rx object id 
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
    UInt8               rxId,
    Fftc_RxHandle       hRx
)
{
    Int16               eventId;

    /* Map the FFTC system event number corresponding to the FFTC instance
     * we are using here to a interrupt vector to hook up interrupts on.
     */
    if (rxId == 0)
    {
#ifdef DEVICE_K2L
        eventId     =   49;
#else
        eventId     =   50;
#endif
    }
    else 
    {
#ifdef DEVICE_K2L
        eventId     =   50;
#else
        eventId     =   51;
#endif
    }

    /* Register the FFTC driver's high priority ISR handle for this event */
    EventCombiner_dispatchPlug (eventId, (EventCombiner_FuncPtr)Fftc_rxHiPriorityRxISR, (UArg)hRx, TRUE);
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
 *  @n rxId                 Rx object Id
 *
 *  @return     Int32
 *              -1      -   FFTC Rx test failed.
 *              0       -   FFTC Rx test successful.
 * ============================================================================
 */
static Int32 fftc_rx (Fftc_DrvHandle hFFTC, UInt8 rxId)
{
    Fftc_ResultHandle           hResultInfo;
    Fftc_RxHandle               hRxObj  =   NULL;
    Fftc_RxCfg                  rxCfg;    
    UInt32                      blockExpVal, clipping;
    UInt8                       *pResultBuffer, *pResultPSInfo;
    UInt32                      resultLen, rxPSInfoLen;
    Fftc_Result*                pFFTResult = NULL;
    Cplx16                      *xout, *xoutread;
    UInt16                      i, j, pktCounter = 0, rxDestnTagInfo;
    Int32                       retVal, useFlowId = -1;
    UInt32                      bIsTestFailed = 0;
    UInt8                       coreNum = CSL_chipReadReg (CSL_CHIP_DNUM), rxFlowId, rxSrcId;
    Fftc_BlockInfo              blockInfo;
    FFT_TestCfg*                pFFTTestCfg = NULL;
    
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
    Fftc_osalLog ("FFTC Rx FLOW SHARING Test Task START on Core %d \n", coreNum);
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
     *      -   Let FFTC driver pick the destination queue (cppiRxQNum = -1)
     *      -   Pick a buffer size big enough to hold the error information and
     *          FFT result itself.
     */
    memset (&rxCfg, 0, sizeof (rxCfg));
    if (rxId == 0)
    {
        /* Create a new Rx flow */            
        useFlowId    =   -1;            
    }
    else
    {
        /* Reuse the flow created earlier */            
        useFlowId    =   Fftc_findFlowIdByQueueNumber (hFFTC, FFTC_RX_QNUM);            
    }
    rxCfg.useFlowId                         =   useFlowId;     /* create a new flow/reuse flow */
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
    if (rxId == 0)
    {
        rxCfg.cppiRxQNum                    =   FFTC_RX_QNUM; 
        rxCfg.bUseInterrupts                =   0;
        rxCfg.bManageAccumList              =   1;      /* Let driver manage accumulator list */
        rxCfg.accumCfg.drvCfg.bEnablePacing =   0;      /* Disable pacing */
        rxCfg.accumCfg.drvCfg.intThreshold  =   1;      /* Set interrupt threshold to 1 */
        rxCfg.accumCfg.drvCfg.accChannelNum =   FFTC_RX_ACC_CHANNEL_NUM;
    }
    else
    {
        rxCfg.cppiRxQNum                    =   FFTC_RX_QNUM + FFTC_RX_NEXT_QNUM_INC;
        rxCfg.bUseInterrupts                =   0;
        rxCfg.bManageAccumList              =   1;      /* Let driver manage accumulator list */
        rxCfg.accumCfg.drvCfg.bEnablePacing =   0;      /* Disable pacing */
        rxCfg.accumCfg.drvCfg.intThreshold  =   1;      /* Set interrupt threshold to 1 */
        rxCfg.accumCfg.drvCfg.accChannelNum =   FFTC_RX_ACC_CHANNEL_NUM + 8;
    }   
    rxCfg.bBlockOnResult                    =   1;

    /* Get a Rx Object Handle */
    if ((hRxObj = Fftc_rxOpen (hFFTC, &rxCfg)) == NULL)
    {
        Fftc_osalLog ("[Rx Task %d]: Rx flow open failed \n", rxId);
        goto error;
    }
    else
    {
        Fftc_osalLog ("[Rx Task %d]: Rx Flow %d opened successfully using Rx queue %d \n", 
                       rxId, Fftc_rxGetFlowId (hRxObj), Fftc_rxGetRxQueueNumber (hRxObj));
    }  

    /* Register ISR for Rx */
    if (rxCfg.bUseInterrupts)
        register_rx_interrupts (CSL_FFTC_0, rxId, hRxObj);
    isTestInitDone ++;
    
    while (isTestInitDone != 4)
        Task_yield ();
    
    /* Run the test multiple times */
    while (pktCounter != NUM_TEST_PACKETS)
    {
        /* -------------------------------------
         * Wait on FFT result and verify Result
         * -------------------------------------
         */
#ifdef  FFTC_TEST_DEBUG
        Fftc_osalLog ("\n[Rx Task %d]: Waiting for Result ... \n", rxId);
#endif
        
        /* Poll for the FFT result. 
         *  
         * Lets just wait for few iterations and declare an error if we dont see 
         * a result otherwise.
         */
        if (!rxCfg.bUseInterrupts)
        {
            while (!Fftc_rxGetNumPendingResults (hRxObj)) 
                Task_yield ();
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
            Fftc_osalLog ("[Rx Task %d]: Invalid FFT result : %d \n", rxId, retVal);
            goto error;
        }
#ifdef  FFTC_TEST_DEBUG
        Fftc_osalLog ("\n[Rx Task %d]: Got FFT Result %d !!\n", rxId, rxDestnTagInfo);
#endif

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
            Fftc_osalLog ("[Rx Task %d]: Error parsing result : %d  error: %d \n", rxId, pktCounter, retVal);
            goto error;
        }

        /* verify result */
#ifdef  FFTC_TEST_DEBUG
        Fftc_osalLog("[Rx Task %d]: Result Info:: DestnTagInfo: %d Error detected: %d Number of FFTC Result Blocks: %d \n", 
                     rxId, pFFTResult->destnTagInfo, pFFTResult->bErrorDetected, pFFTResult->numFFTCBlocks);
#endif

        /* Get the per block result and compare against the expected values
         * to see if the FFT operation succeeded.
         */
        for (j = 0; j < pFFTResult->numFFTCBlocks; j++)
        {
#ifdef  FFTC_TEST_DEBUG
            Fftc_osalLog ("\n[Rx Task %d]: ********* Block %d Result *********\n", rxId, j);
#endif

            /* Results got */                
            xout            =   (Cplx16 *) pFFTResult->fftBlockResult [j].pBuffer;
            blockExpVal     =   pFFTResult->fftBlockResult [j].blockExponentVal;
            clipping        =   pFFTResult->bClippingDetected;

            /* Expected result */
            xoutread        =   (Cplx16 *) pFFTTestCfg->pFftOutputData [j];

#ifdef  FFTC_TEST_DEBUG
            Fftc_osalLog ("\n[Rx Task %d]: FFT Result details: blockExp: %d Clipping: %d Result buffer len: %d \n",  
                            rxId, blockExpVal, clipping, pFFTResult->fftBlockResult [j].bufferLen);

            Fftc_osalLog ("\n[Rx Task %d]: Comparing FFT Result from engine against expected ... \n", rxId);
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
                                    rxId, i, xout[i].real, xoutread[i].real);
                    Fftc_osalLog ("[Rx Task %d]: Imag data sample %d: test failed, read: %d actual: %d \n",
                                    rxId, i, xout[i].imag, xoutread[i].imag);
#endif
                    
                    bIsTestFailed ++;
                }
            }

#ifdef  FFTC_TEST_DEBUG
            if (!bIsTestFailed)
                Fftc_osalLog ("[Rx Task %d]: FFT Result Correct !! \n", rxId);
            else
                Fftc_osalLog ("[Rx Task %d]: FFT Result Wrong !! \n", rxId);                            
#endif

            /* Compare the block exponent value got against expected value */
            if (!pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && blockExpVal != pFFTTestCfg->blockExpVal [j])
            {
#ifdef  FFTC_TEST_DEBUG
                Fftc_osalLog("[Rx Task %d]: Block Exponent mismatch: test failed, read: %d actual: %d \n", 
                            rxId, blockExpVal, pFFTTestCfg->blockExpVal [j]);
#endif
                bIsTestFailed ++;
            }
            else
            {
#ifdef  FFTC_TEST_DEBUG
                Fftc_osalLog ("[Rx Task %d]: Block Exponent Value Correct !! \n", rxId);
#endif
            }

            /* Compare the clipping detected value got against expected value */
            if (!pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && clipping != pFFTTestCfg->bClippingDetected [j])
            {
#ifdef  FFTC_TEST_DEBUG
                Fftc_osalLog("[Rx Task %d]: Clipping Detect mismatch: test failed, read: %d actual: %d \n", 
                            rxId, clipping, pFFTTestCfg->bClippingDetected [j]);
#endif
                bIsTestFailed ++;
            }
            else
            {
#ifdef  FFTC_TEST_DEBUG
                Fftc_osalLog ("[Rx Task %d]: Clipping Detect Value Correct !! \n", rxId);
#endif
            }
        }

        /* Done using the result buffer, return it to the FFTC driver for recycling */
        if (Fftc_rxFreeResult (hRxObj, 
                               hResultInfo
                               ) < 0)
        {
            Fftc_osalLog ("[Rx Task %d]: Error freeing result : %d \n", rxId, i);
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
        Fftc_osalLog ("\n[Rx Task %d]: FFTC Test PASS !! Num Requests Processed = %d \n", rxId, pktCounter);
        totalNumTestsPass ++;                
    }
    else
    {
        Fftc_osalLog ("\n[Rx Task %d]: FFTC Test FAILED !! Num Tests Failed = %d \n", rxId, bIsTestFailed);     
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
    Fftc_osalLog ("\n[Rx Task %d]: FFTC Test FAILED !! Num Tests Failed = %d \n", rxId, bIsTestFailed);                

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
 *  @n rxId                 Rx object Id
 *
 *  @return     Int32
 *              -1      -   FFTC test failed.
 *              0       -   FFTC test successful.
 * ============================================================================
 */
static Int32 fftc_tx (Fftc_DrvHandle hFFTC,UInt8 rxId)
{
    Fftc_RequestHandle          hRequestInfo;
    Fftc_TxHandle               hTxObj  =   NULL;
    Fftc_TxCfg                  txCfg;    
    UInt8                       *pReqBuffer;
    UInt32                      maxReqBufferLen, reqBufferLen, txPSInfoLen = 0, qNum;
    Int16                       i, pktCounter = 0;
    UInt32                      blockDataOffset = 0;
    UInt8                       coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
    Int8                        rxFlowId;
    Fftc_BlockInfo              blockInfo;
    FFT_TestCfg*                pFFTTestCfg = NULL;
    UInt8                       txQNum = 0;
    
    /* Allocate memory for the test configuration */
    if (!(pFFTTestCfg = (FFT_TestCfg *) Fftc_osalMalloc (sizeof (FFT_TestCfg), FALSE)))
    {
        Fftc_osalLog ("Error allocating memory for test configuration \n");
        return -1;
    }

    /* Read the FFT input vector and configuration for this test 
     * from the corresponding input configuration file.
     *
     * Pick a different test to run on both the flows
     */
    if (fftc_parse_testCfg (1, pFFTTestCfg, &blockInfo) < 0)
    {
        Fftc_osalLog ("Error parsing test configuration \n");
        return -1;         
    }
    
    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC Tx FLOW SHARING Test Task START on Core %d \n", coreNum);
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
    txCfg.cppiNumDesc           =   1; 
    txCfg.bufferSize            =   pFFTTestCfg->numInputSamples[0] * FFTC_TEST_SAMPLE_SIZE * pFFTTestCfg->numBlocks;  
    txCfg.bPSInfoPresent        =   0; 

    /* Get a Tx Object Handle */
    if ((hTxObj = Fftc_txOpen (hFFTC, &txCfg)) == NULL)
    {
        Fftc_osalLog ("[Tx Task]: Tx open failed \n");
        goto error;
    }
    
    isTestInitDone ++;

    while (isTestInitDone != 4)
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
        txPSInfoLen     =   0;
        blockDataOffset =   0;

        /* Setup the result receiver. 
         *
         * We need to find a flow that can receive FFT
         * results for the requests we submit on the
         * Rx queue specified in 'FFTC_RX_QNUM'.
         */
        if (rxId == 0)
        {
            qNum =     FFTC_RX_QNUM;
            pFFTTestCfg->fftcQCfg.destQRegConfig.cppiDestQNum = FFTC_DEF_CPPI_QUEUE_NUM;
        }
        else
        {
            qNum =     FFTC_RX_QNUM + FFTC_RX_NEXT_QNUM_INC;
            pFFTTestCfg->fftcQCfg.destQRegConfig.cppiDestQNum = qNum;
        }
                        
        if ((rxFlowId = Fftc_findFlowIdByQueueNumber (hFFTC, qNum)) < 0)
        {
            Fftc_osalLog ("[Tx Task]: No flow found configured on Rx queue %d \n", qNum);
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
            Fftc_osalLog ("[Tx Task]: Unable to get request buffer \n");
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
        Fftc_osalLog ("\n[Tx Task]: Submitting FFT Request ... \n");
#endif

        /* Submit the FFT request for processing */
        if ((i = Fftc_txSubmitRequest (hTxObj, 
                                  hRequestInfo, 
                                  reqBufferLen)) < 0)
        {
            Fftc_osalLog ("[Tx Task]: Unable to submit request \n", coreNum);
            goto error;
        }
        else
        {
#ifdef FFTC_TEST_DEBUG              
            Fftc_osalLog ("[Tx Task]: Submitted FFT request, Id: %d for processing by Flow %d RxQNum: %d \n", 
                           pktCounter, rxFlowId, qNum);   
#endif            
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
 *  @n@b test_host_singlecore_flowshare
 *
 *  @b  brief
 *  @n  This API sets up the test parameters, calls the test core API to 
 *      actually run the test.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void test_host_singlecore_flowshare
(
    UInt8                   fftcInstNum,
    Fftc_DrvHandle          hFFTC
)
{
    UInt32                  rxId = 0, numTasksCreated = 0;
    Task_Params             testTaskParams;

    /* Create the FFTC Rx1 test task */
    Task_Params_init(&testTaskParams);
    testTaskParams.arg0     =   (UInt32) hFFTC;
    testTaskParams.arg1     =   (UInt32) rxId;
    
    /* Create the Rx task */
    Task_create ((Task_FuncPtr)&fftc_rx, &testTaskParams, NULL);
    numTasksCreated ++;

    /* Create the FFTC Tx1 test task */
    Task_Params_init(&testTaskParams);
    testTaskParams.arg0     =   (UInt32) hFFTC;
    testTaskParams.arg1     =   (UInt32) rxId;
    
    /* Create the Tx task */
    Task_create ((Task_FuncPtr)&fftc_tx, &testTaskParams, NULL);
    numTasksCreated ++;
    rxId ++;

    /* Create the FFTC Rx2 test task */
    Task_Params_init(&testTaskParams);
    testTaskParams.arg0     =   (UInt32) hFFTC;
    testTaskParams.arg1     =   (UInt32) rxId;
    
    /* Create the Rx task */
    Task_create ((Task_FuncPtr)&fftc_rx, &testTaskParams, NULL);
    numTasksCreated ++;

    /* Create the FFTC Tx2 test task */
    Task_Params_init(&testTaskParams);
    testTaskParams.arg0     =   (UInt32) hFFTC;
    testTaskParams.arg1     =   (UInt32) rxId;
    
    /* Create the Tx task */
    Task_create ((Task_FuncPtr)&fftc_tx, &testTaskParams, NULL);
    numTasksCreated ++;
    rxId ++;

    /* Wait till the Rx, Tx tasks are not done. */
    while (isTestTaskDone != numTasksCreated)
    {
        Task_yield ();            
    }

    return;
}
