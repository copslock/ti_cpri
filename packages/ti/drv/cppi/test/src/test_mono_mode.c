/**
 *   @file  test_mono_mode.c
 *
 *   @brief   
 *      Tests data transfer using monolithic descriptor
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
 *  \par
*/

#ifndef __LINUX_USER_SPACE
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#else
#include "fw_test.h"
#endif

#include <string.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* Test specific includes */
#include <cppi_test.h>

#ifndef __LINUX_USER_SPACE
/************************ EXTERN VARIABLES ********************/
extern UInt8                    dataBuff[SIZE_DATA_BUFFER * NUM_DATA_BUFFER];
extern UInt32                   errorCount, coreNum;
extern Qmss_QueueHnd            freeMonoQueHnd;

#ifdef CONFIG_ACC
/* List address for accumulator - twice the number of entries for Ping and Pong page */
extern Uint32                   hiPrioList[(NUM_PACKETS + 1) * 2];
#endif
#endif

/************************ EXTERN FUNCTIONS ********************/
extern uint32_t l2_global_address (uint32_t addr);

/*************************** FUNCTIONS ************************/

/**
 *  @b Description
 *  @n  
 *      Tests data transfer using monolithic descriptor 
 *  @retval
 *      Not Applicable.
 */
void testMonoDataTransfer (void)
{
    Qmss_Result             result;
    uint32_t                i, length, destLen, psLen; 
    uint8_t                 isAllocated;
    uint8_t                 *dataBuffPtr, *psBuffPtr;
    Cppi_Handle             cppiHnd;
    Cppi_ChHnd              rxChHnd, txChHnd;
    Qmss_QueueHnd           txQueHnd, rxQueHnd, freeQueHnd, txCmplQueHnd, txFreeQueHnd;
    Cppi_Desc               *monoDescPtr, *rxPkt;
    Cppi_FlowHnd            rxFlowHnd;
    Qmss_Queue              queInfo;
    Qmss_Queue              cmplQueInfo;
    uint8_t                 psData[4];
    Cppi_TxChInitCfg        txChCfg;
    Cppi_RxChInitCfg        rxChCfg;
    Cppi_RxFlowCfg          rxFlowCfg;
    Cppi_CpDmaInitCfg       cpdmaCfg;
    uint32_t                freeMonoQueGroup = Qmss_getQueueGroup (freeMonoQueHnd);
    Cppi_DescTag            tag;
#ifdef CONFIG_ACC
    Qmss_AccCmdCfg          cfg;
    uint32_t                index;
    volatile uint32_t       temp; 
#endif

    System_printf ("\n~~~~~~~~~~Core %d Testing data transfer using Monolithic descriptor~~~~~~~~\n", coreNum);

    /* Set up QMSS CPDMA configuration */
    memset ((void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
#ifndef NSS_LITE
    cpdmaCfg.dmaNum = Cppi_CpDma_QMSS_CPDMA;
#else
    cpdmaCfg.dmaNum = Cppi_CpDma_NETCP_CPDMA;
#endif    

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        System_printf ("Error Core %d : Initializing QMSS (or NetCP) CPPI CPDMA %d\n", coreNum, cpdmaCfg.dmaNum);
        return;
    }

    /* Open transmit free descriptor queue */
    #ifndef NSS_LITE
    if ((txFreeQueHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #else
    if ((txFreeQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #endif
    {
        System_printf ("Error Core %d : Opening Transmit free descriptor queue\n", coreNum);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Transmit Free Queue Number : %d opened\n", coreNum, txFreeQueHnd);

    for (i = 0; i < NUM_PACKETS; i++)
    {
        if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (freeMonoQueHnd)) == NULL)
        {
            System_printf ("Error Core %d : Getting descriptor from Queue Number %d\n", coreNum, freeMonoQueHnd);
            errorCount++;
            return;
        }

        /* Push descriptor to Tx free queue */
        Qmss_queuePushDesc (txFreeQueHnd, (uint32_t *) monoDescPtr);
    }

    /* Open receive free descriptor queue in same group as freeMonoQueHnd for divert */
    #ifndef NSS_LITE
    if ((freeQueHnd = Qmss_queueOpenInGroup (freeMonoQueGroup, Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #else
    if ((freeQueHnd = Qmss_queueOpenInGroup (freeMonoQueGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #endif
    {
        System_printf ("Error Core %d : Opening Receive free descriptor queue\n", coreNum);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Receive Free Queue Number : %d opened\n", coreNum, freeQueHnd);

    queInfo = Qmss_getQueueNumber (freeQueHnd);
    for (i = 0; i < NUM_PACKETS; i++)
    {
        if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (freeMonoQueHnd)) == NULL)
        {
            System_printf ("Error Core %d : Getting descriptor from Queue Number %d\n", coreNum, freeMonoQueHnd);
            errorCount++;
            return;
        }

        /* Set return queue */
        Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, monoDescPtr, queInfo);

        /* Push descriptor to Rx free queue */
        Qmss_queuePushDesc (freeQueHnd, (uint32_t *) monoDescPtr);
    }

    /* Set up Rx Channel parameters */
    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;
    
    /* Open Rx Channel */
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
        errorCount++;
        return;
    }
    else 
        System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));


    /* Set up Tx Channel parameters */
    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    txChCfg.priority = 0;
    txChCfg.filterEPIB = 0;
    txChCfg.filterPS = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_DISABLE;
    
    /* Open Tx Channel */
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
        errorCount++;
        return;
    }
    else 
        System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));

    /* Opens transmit queue. This is the infrastructure queue */
    if ((txQueHnd = Qmss_queueOpen (Qmss_QueueType_INFRASTRUCTURE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error Core %d : Opening Transmit Queue Number\n", coreNum);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Transmit Queue Number : %d opened\n", coreNum, txQueHnd);

    /* Opens receive queue */
    #ifndef NSS_LITE
    if ((rxQueHnd = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #else
    if ((rxQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #endif
    {
        System_printf ("Error Core %d : Opening Receive Queue Number\n", coreNum);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Receive Queue Number : %d opened\n", coreNum, rxQueHnd);

    /* Opens transmit completion queue in same group as freeMonoQueHnd for divert. */
    if ((txCmplQueHnd = Qmss_queueOpenInGroup (freeMonoQueGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error Core %d : Opening Tx Completion Queue Number\n", coreNum);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Tx Completion Queue Number : %d opened\n", coreNum, txCmplQueHnd);

    cmplQueInfo = Qmss_getQueueNumber (txCmplQueHnd);
    System_printf ("Core %d : Free Queue Number : %d opened\n", coreNum, freeQueHnd);
    System_printf ("Core %d : Transmit Free Queue Number : %d opened\n", coreNum, txFreeQueHnd);

#ifdef CONFIG_ACC
    /* program the high priority accumulator */
    memset ((void *) &hiPrioList, 0, sizeof (hiPrioList));
    cfg.channel = 0;
    cfg.command = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.queueEnMask = 0;
    cfg.listAddress = l2_global_address ((uint32_t) hiPrioList); /* Should be global if reading on another core */
    /* Get queue manager and queue number from handle */
    cfg.queMgrIndex = Qmss_getQIDFromHandle (rxQueHnd);
    cfg.maxPageEntries = 9; /* Sending 8 packets */
    cfg.timerLoadCount = 1;
    cfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
    cfg.listEntrySize = Qmss_AccEntrySize_REG_D;
    cfg.listCountMode = Qmss_AccCountMode_NULL_TERMINATE;
    cfg.multiQueueMode = Qmss_AccQueueMode_SINGLE_QUEUE;
    
    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : high priority accumulator programmed for channel : %d queue : %d\n", 
                        coreNum, cfg.channel, cfg.queMgrIndex);
#endif

    /* Set transmit queue threshold to high and when there is atleast one packet */
    /* Setting threshold on transmit queue is not required anymore. tx pending queue is not hooked to threshold. 
     * Qmss_setQueueThreshold (txQueHnd, 1, 1);
     */ 
            
    System_printf ("\n-----------------------PS in Desc-----------------------------\n");

    /* Setup Rx flow parameters */
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Don't specify flow number and let CPPI allocate the next available one */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;

    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (rxQueHnd);
    rxFlowCfg.rx_dest_qnum = queInfo.qNum;
    rxFlowCfg.rx_dest_qmgr = queInfo.qMgr;
    rxFlowCfg.rx_sop_offset = MONOLITHIC_DESC_DATA_OFFSET;
    rxFlowCfg.rx_desc_type = Cppi_DescType_MONOLITHIC; 

    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (freeQueHnd);
    rxFlowCfg.rx_fdq0_sz0_qnum = queInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr = queInfo.qMgr;
    rxFlowCfg.rx_psinfo_present = 1;
    rxFlowCfg.rx_ps_location = 0;

    /* Configure Rx flow */
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
        errorCount++;
        return;
    }
    else 
        System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId(rxFlowHnd));
    
    System_printf ("\n-----------------------Free Queue-----------------------------\n");
    for (i = 0; i < NUM_PACKETS; i++)
    {
        rxPkt = (Cppi_Desc *) Qmss_queuePop (freeQueHnd);
        {
            System_printf ("Core %d : Descriptor in free queue 0x%p\n", coreNum, rxPkt);
            /* Push descriptor back to free queue */
            Qmss_queuePushDesc (freeQueHnd, (uint32_t *) rxPkt);
        }
    }

    /* Enable transmit channel */
    if (Cppi_channelEnable (txChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
        errorCount++;
    }
    else 
        System_printf ("Core %d : Tx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (txChHnd));

    /* Enable receive channel */
    if (Cppi_channelEnable (rxChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
        errorCount++;
    }
    else 
        System_printf ("Core %d : Rx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (rxChHnd));


    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++) 
        dataBuff[i] = i;

    /* Fill in PS data */
    psData[0] = 0xAA;
    psData[1] = 0xBB;
    psData[2] = 0xCC;
    psData[3] = 0xDD;

    /* Set flow in tag */
    tag.destTagLo = 0;
    tag.destTagHi = 0;
    tag.srcTagLo = Cppi_getFlowId(rxFlowHnd);
    tag.srcTagHi = 0;
    System_printf ("\n--------------------Transmitting packets----------------------\n");
    /* Send out 8 packets */
    for (i = 0; i < NUM_PACKETS; i++)
    {
        /* Get a free descriptor */
        if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
        {
            System_printf ("Error Core %d : Getting descriptor from Queue Number: %d\n", coreNum, txFreeQueHnd);
            errorCount++;
            return;
        }

        /* Set return queue */
        Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, monoDescPtr, cmplQueInfo);
        
        /* Set Data Offset to include PS words */
        Cppi_setDataOffset (Cppi_DescType_MONOLITHIC, monoDescPtr, MONOLITHIC_DESC_DATA_OFFSET);

        /* Set PS data */
        Cppi_setPSLocation (Cppi_DescType_MONOLITHIC, monoDescPtr, Cppi_PSLoc_PS_IN_DESC);

        Cppi_setPSData (Cppi_DescType_MONOLITHIC, monoDescPtr, (uint8_t *) &psData, SIZE_PS_DATA);

        /* Add data buffer */
        Cppi_setData (Cppi_DescType_MONOLITHIC, monoDescPtr, (uint8_t *) dataBuff, SIZE_DATA_BUFFER);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_MONOLITHIC, monoDescPtr, SIZE_DATA_BUFFER);

        /* Set flow */
        Cppi_setTag (Cppi_DescType_MONOLITHIC, monoDescPtr, &tag);

        System_printf ("Core %d : Transmitting descriptor 0x%p\n", coreNum, monoDescPtr);

        /* Push descriptor to Tx queue */
        Qmss_queuePushDescSize (txQueHnd, (uint32_t *) monoDescPtr, MONOLITHIC_DESC_DATA_OFFSET);
    }

    System_printf ("\n-------------------------Queue status-------------------------\n");
    result = Qmss_getQueueEntryCount (txQueHnd);
    System_printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    System_printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

    System_printf ("\n--------------------Receiving packets-------------------------\n");

#ifndef CONFIG_ACC
    while (Qmss_getQueueEntryCount  (rxQueHnd) == 0);
    
    /* Get the rx packet */
    while ((rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (Qmss_queuePop (rxQueHnd))) != NULL)
    {
        length = Cppi_getPacketLen (Cppi_DescType_MONOLITHIC, rxPkt);

        System_printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

        /* Get PS Info */
        Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *) rxPkt, &psBuffPtr, &psLen);

        /* Check if length is correct */
        if (psLen != SIZE_PS_DATA)
        {
            System_printf ("Error Core %d : PS data length mismatch Tx: %d  - Rx: %d\n", coreNum, SIZE_PS_DATA, psLen);
            errorCount++;
        }

        /* Compare PS data */
        for (i = 0; i < psLen; i++)
        {
            if (psData[i] != psBuffPtr[i])
            {
                System_printf ("Error Core %d : In PS data Tx: %02x - Rx: %02x \n", coreNum, psData[i], psBuffPtr[i]);
                errorCount++;
            }
        }

        /* Get data buffer */
        Cppi_getData (Cppi_DescType_MONOLITHIC, rxPkt, &dataBuffPtr, &destLen);

        /* Compare */
        for (i = 0; i < destLen; i++)
        {
            if (dataBuff[i] != dataBuffPtr[i])
            {
                System_printf ("Error Core %d : In data buffer Tx: %02x - Rx: %02x \n", coreNum, dataBuff[i], dataBuffPtr[i]);
                errorCount++;
            }
        }
        
        /* Recycle the descriptors */
        queInfo = Cppi_getReturnQueue (Cppi_DescType_MONOLITHIC, rxPkt);
        
        /* Push descriptor back to free queue */
        Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (uint32_t *) rxPkt);
    }
#else
    {
        /* Burn some time for the accumulators to run. */
        temp = 0;
        for (i = 0; i < 50000; i++)
        {
            temp = i;
        }
        temp = 1;
    }

    /* Get the Rx packet */
    /* 8 packets were sent out */
    for (index = 0; index < NUM_PACKETS; index++)
    {
        rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (hiPrioList[index]);

        /* Get packet length */
        length = Cppi_getPacketLen (Cppi_DescType_MONOLITHIC, rxPkt);

        System_printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

        /* Get PS Info */
        Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, rxPkt, &psBuffPtr, &psLen);

        /* Check if length is correct */
        if (psLen != SIZE_PS_DATA)
        {
            System_printf ("Error Core %d : PS data length mismatch Tx: %d  - Rx: %d\n", coreNum, SIZE_PS_DATA, psLen);
            errorCount++;
        }

        /* Compare PS data */
        for (i = 0; i < psLen; i++)
        {
            if (psData[i] != psBuffPtr[i])
            {
                System_printf ("Error Core %d : In PS data Tx: %02x - Rx: %02x \n", coreNum, psData[i], psBuffPtr[i]);
                errorCount++;
            }
        }

        /* Get data buffer */
        Cppi_getData (Cppi_DescType_MONOLITHIC, rxPkt, &dataBuffPtr, &destLen);

        /* Compare data */
        for (i = 0; i < destLen; i++)
        {
            if (dataBuff[i] != dataBuffPtr[i])
            {
                System_printf ("Error Core %d : In data buffer Tx: %02X - Rx: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
                errorCount++;
            }
        }
        
        /* Recycle the descriptors */
        queInfo = Cppi_getReturnQueue (Cppi_DescType_MONOLITHIC, rxPkt);
        
        /* Push descriptor back to free queue */
        Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (uint32_t *) rxPkt);
    }
    Qmss_ackInterrupt (cfg.channel, 1);
    Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, cfg.channel);
#endif

    System_printf ("\n--------------------Deinitializing----------------------------\n");

    /* Recycle Tx and Rx descriptor to free queue */ 
    queue_divert_and_check (txCmplQueHnd, freeMonoQueHnd);
    queue_divert_and_check (freeQueHnd, freeMonoQueHnd);
 
    /* Close Rx flow */
    if ((result = Cppi_closeRxFlow (rxFlowHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Rx flow error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx flow closed successfully. Ref count : %d\n", coreNum, result);

#ifdef CONFIG_ACC
    /* Disable accumulator */
    if ((result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, cfg.channel)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Disabling high priority accumulator for channel : %d queue : %d error code: %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : high priority accumulator disabled for channel : %d queue : %d\n", 
                        coreNum, cfg.channel, cfg.queMgrIndex);
#endif

    result = Qmss_getQueueEntryCount (txQueHnd);
    System_printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    System_printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

    /* Close Tx channel */
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Tx channel error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Tx Channel closed successfully. Ref count : %d\n", coreNum, result);

    /* Close Rx channel */
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Rx channel error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx Channel closed successfully. Ref count : %d\n", coreNum, result);

    Qmss_queueEmpty (rxQueHnd);
    Qmss_queueEmpty (txQueHnd);
    Qmss_queueEmpty (freeQueHnd);
    Qmss_queueEmpty (txCmplQueHnd);
    Qmss_queueEmpty (txFreeQueHnd);

    /* Close the queues */
    if ((result = Qmss_queueClose (rxQueHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Rx queue error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx queue closed successfully. Ref count : %d\n", coreNum, result);
    
    if ((result = Qmss_queueClose (txQueHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing tx queue error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Tx queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (freeQueHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing free queue error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Free queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (txCmplQueHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing transmit completion queue error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Transmit completion queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (txFreeQueHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing transmit freequeue error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Transmit free queue closed successfully. Ref count : %d\n", coreNum, result);

    /* Close CPPI CPDMA instance */
    if ((result = Cppi_close (cppiHnd)) < CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing CPPI CPDMA error code : %d\n", coreNum, result);
        errorCount++;
    }
    else 
        System_printf ("Core %d : CPPI CPDMA closed successfully\n", coreNum);
}


