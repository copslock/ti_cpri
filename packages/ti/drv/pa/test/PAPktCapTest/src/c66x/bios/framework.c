/**
 * @file framework.c
 *
 * @brief
 *  This file holds all the platform specific framework
 *  initialization and setup code.
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
*/
#include "pcap_singlecore.h"
#include "ti/drv/pa/pa.h"
#include "ti/drv/pa/pasahost.h"
/* Firmware images */
#include <ti/drv/pa/fw/pafw.h>

#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

#define PASS_TEST_TX_CMD
/* High Priority Accumulation Interrupt Service Handler for this application */
void Cpsw_RxISR (void);

/* Constructed data packet to send. */
#pragma DATA_ALIGN(pktMatch, 16)
uint8_t pktMatch[] = {
							0x10, 0x11, 0x12, 0x13, 0x14, 0x15,                      /* Dest MAC */
                            0x00, 0x01, 0x02, 0x03, 0x04, 0x05,                      /* Src MAC  */
                            0x08, 0x00,                                              /* Ethertype = IPv4 */
                            0x45, 0x00, 0x00, 0x6c,                                  /* IP version, services, total length */
                            0x00, 0x00, 0x00, 0x00,                                  /* IP ID, flags, fragment offset */
                            0x05, 0x11, 0x32, 0x26,                                  /* IP ttl, protocol (UDP), header checksum */
                            0xc0, 0xa8, 0x01, 0x01,                                  /* Source IP address */
                            0xc0, 0xa8, 0x01, 0x0a,                                  /* Destination IP address */
                            0x12, 0x34, 0x56, 0x78,                                  /* UDP source port, dest port */
                            0x00, 0x58, 0x1d, 0x18,                                  /* UDP len, UDP checksum */
                            0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,          /* 80 bytes of payload data */
                            0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
                            0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
                            0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
                            0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                            0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
                            0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
                            0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
                            0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
                            0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81  };

/* Constructed data packet to send. */
#pragma DATA_ALIGN(pktDrop, 16)
uint8_t pktDrop[] = {
							0x10, 0x11, 0x12, 0x13, 0x14, 0x16,                      /* Dest MAC */
                            0x00, 0x01, 0x02, 0x03, 0x04, 0x06,                      /* Src MAC  */
                            0x08, 0x00,                                              /* Ethertype = IPv4 */
                            0x45, 0x00, 0x00, 0x6c,                                  /* IP version, services, total length */
                            0x00, 0x00, 0x00, 0x00,                                  /* IP ID, flags, fragment offset */
                            0x05, 0x11, 0x32, 0x26,                                  /* IP ttl, protocol (UDP), header checksum */
                            0xc0, 0xa8, 0x01, 0x01,                                  /* Source IP address */
                            0xc0, 0xa8, 0x01, 0x0a,                                  /* Destination IP address */
                            0x12, 0x34, 0x56, 0x78,                                  /* UDP source port, dest port */
                            0x00, 0x58, 0x1d, 0x18,                                  /* UDP len, UDP checksum */
                            0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,          /* 80 bytes of payload data */
                            0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
                            0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
                            0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
                            0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                            0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
                            0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
                            0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
                            0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
                            0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81  };


extern int    expectedTxCounter[NUM_MAC_PORTS], expectedRxCounter[NUM_MAC_PORTS];
extern int    actialTxCounter[NUM_MAC_PORTS], actualRxCounter[NUM_MAC_PORTS];

uTestEmacport_t ports[8]={
		/* Port 1 or pa_EMAC_PORT_0 */
		{
			pa_EMAC_PORT_0,
			AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 2 or pa_EMAC_PORT_1 */
		{
			pa_EMAC_PORT_1,
			NOT_AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 3 or pa_EMAC_PORT_2 */
		{
			pa_EMAC_PORT_2,
			NOT_AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 4 or pa_EMAC_PORT_3 */
		{
			pa_EMAC_PORT_3,
			AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 5 or pa_EMAC_PORT_4 */
		{
			pa_EMAC_PORT_4,
			AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 6 or pa_EMAC_PORT_5 */
		{
			pa_EMAC_PORT_5,
			AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 7 or pa_EMAC_PORT_6 */
		{
			pa_EMAC_PORT_6,
			AVAILABLE_FOR_PCAP_TEST
		},
		/* Port 8 or pa_EMAC_PORT_7 */
		{
			pa_EMAC_PORT_7,
			AVAILABLE_FOR_PCAP_TEST
		},

};


static	paQueueBounceConfig_t pcapQueueBounceCfg =
	        {
	            1,      /* Enable */
                PA_PKT_CAP_QUEUE_BOUNCE_QUEUE_DDR,              /* ddrQueueId */
                PA_PKT_CAP_QUEUE_BOUNCE_QUEUE_MSMC,             /* msmcQueueId */
                QMSS_PASS_QUEUE_BASE,                           /* hwQueueBegin */
                QMSS_PASS_QUEUE_BASE + NSS_NUM_TX_QUEUES - 1,   /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* QoS mode */
                    pa_QUEUE_BOUNCE_OP_DDR,     /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_NONE     /* All traffics */
                }
	        };


uint8_t * DataBufAlloc(void)
{
    uint8_t* pDataBuffer = NULL;
    if ((pDataBuffer = (Ptr) Memory_alloc(NULL, PA_EMAC_EX_RXBUF_SIZE, 0, NULL)) == NULL)
    {
        System_printf ("Error allocating memory for Rx data buffer \n");
    }
    return (pDataBuffer);
}

/* Free Attached Buffers */
void DataBufFree(void* pDataBuffer, uint32_t size)
{
	Memory_free(NULL, pDataBuffer, size);
}

/** ============================================================================
 *   @n@b Convert_CoreLocal2GlobalAddr
 *
 *   @b Description
 *   @n This API converts a core local L2 address to a global L2 address.
 *
 *   @param[in]
 *   @n addr            L2 address to be converted to global.
 *
 *   @return    uint32_t
 *   @n >0              Global L2 address
 * =============================================================================
 */
uint32_t Convert_CoreLocal2GlobalAddr (uint32_t  addr)
{
#ifdef _TMS320C6X
	uint32_t coreNum;

    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Compute the global address. */
    return ((1 << 28) | (coreNum << 24) | (addr & 0x00ffffff));
#else
    return (addr);
#endif
}

/** ============================================================================
 *   @n@b Convert_CoreGlobal2L2Addr
 *
 *   @b Description
 *   @n This API converts a core local L2 address to a global L2 address.
 *
 *   @param[in]
 *   @n addr            L2 address to be converted to global.
 *
 *   @return    uint32_t
 *   @n >0              Global L2 address
 * =============================================================================
 */
uint32_t Convert_CoreGlobal2L2Addr (uint32_t  addr)
{
#ifdef _TMS320C6X
    /* Compute the local l2 address. */
    return (addr & 0x00ffffff);
#else
    return (addr);
#endif
}

/** ============================================================================
 *   @n@b get_qmssGblCfgParamsRegsPhy2Virt
 *
 *   @b Description
 *   @n This API updates the QMSS global configuration registers to global
 *      addressable space for that platform.
 *
 *   @param[in]
 *   @n addr            L2 address to be converted to global.
 *
 *   @return    uint32_t
 *   @n >0              Global L2 address
 * =============================================================================
 */
void get_qmssGblCfgParamsRegsPhy2Virt(Qmss_GlobalConfigParams     *fw_qmssGblCfgParams)
{
	/* Since all physical memory is accessible in DSP, nothing to be done */
	return;
}

/** ============================================================================
 *   @n@b get_cppiGblCfgParamsRegsPhy2Virt
 *
 *   @b Description
 *   @n This API updates the QMSS global configuration registers to global
 *      addressable space for that platform.
 *
 *   @param[in]
 *   @n addr            L2 address to be converted to global.
 *
 *   @return    uint32_t
 *   @n >0              Global L2 address
 * =============================================================================
 */
void get_cppiGblCfgParamsRegsPhy2Virt(Cppi_GlobalConfigParams     *fw_cppiGblCfgParams)
{
	/* Since all physical memory is accessible in DSP, nothing to be done */
	return;
}

Bool                                    gIsPingListUsed = 0;
uint8_t                                 accChannelNum;
/* High Priority Accumulator List - [((Interrupt Threshold + 1) * 2)]
 *
 * MUST be 16 byte aligned.
 *
 * The High priority accumulator list consists of 2 buffers Ping and
 * Pong each consisting of the following entries:
 *
 * (1)  Entry count -   specifies number of packets accumulated in
 *                      the list.
 * (2)  Descriptors -   an array of Rx packet descriptors accumulated
 *                      in this list.
 *
 * Hence the size of high priority accumulator list is calculated as
 * follows:
 *
 * (1)  Get the interrupt threshold, i.e., maximum number of Rx
 *      packets to accumulate before an interrupt is generated.
 * (2)  Add an extra entry to the threshold to track
 *      entry count of the list.
 * (3)  Double this to accomodate space for Ping/Pong lists.
 * (4)  Each accumulator entry is 4 bytes wide.
 *
 * size =   ((interrupt threshold + 1) * 2) * 4 bytes
 *
 * Lets allocate here assuming that interrupt threshold is 1, i.e.,
 * interrupt on every Rxed packet.
 */
#pragma DATA_ALIGN (gHiPriAccumList, 16)
uint32_t                                  gHiPriAccumList[(RX_INT_THRESHOLD + 1) * 2];

int32_t setup_rx_queue(Qmss_Queue *rxQInfo)
{
    Qmss_AccCmdCfg                accCfg;
    uint16_t                      numAccEntries, intThreshold;
    uint8_t                       isAllocated;
    Qmss_Result                   result;
    int32_t                       eventId, vectId;
  	uint8_t                       coreNum = (uint8_t) CSL_chipReadReg(CSL_CHIP_DNUM);
	extern Qmss_QueueHnd gRxQHnd;

	//if (linuxBoot == FALSE)
	if (1)
	{

	   /* Open a Receive (Rx) queue.
		*
		* This queue will be used to hold all the packets received by PASS/CPSW
		*
		* Open the next available High Priority Accumulation queue for Rx.
		*/
	   if ((gRxQHnd = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	   {
		   System_printf ("Error opening a High Priority Accumulation Rx queue \n");
		   return -1;
	   }
	   *rxQInfo = Qmss_getQueueNumber (gRxQHnd);

       /* Setup high priority accumulation interrupts on the Rx queue.
        *
        * Let's configure the accumulator with the following settings:
        *      (1) Interrupt pacing disabled.
        *      (2) Interrupt on every received packet
        */
       intThreshold    =   RX_INT_THRESHOLD;
       numAccEntries   =   (intThreshold + 1) * 2;
       accChannelNum   =   PA_ACC_CHANNEL_NUM + coreNum;

       /* Initialize the accumulator list memory */
       memset ((void *) gHiPriAccumList, 0, numAccEntries * 4);

       /* Setup the accumulator settings */
       accCfg.channel             =   accChannelNum;
       accCfg.command             =   Qmss_AccCmd_ENABLE_CHANNEL;
       accCfg.queueEnMask         =   0;
       accCfg.listAddress         =   Convert_CoreLocal2GlobalAddr((uint32_t) gHiPriAccumList);
       accCfg.queMgrIndex         =   Qmss_getQIDFromHandle(gRxQHnd);
       accCfg.maxPageEntries      =   (intThreshold + 1); /* Add an extra entry for holding the entry count */
       accCfg.timerLoadCount      =   0;
       accCfg.interruptPacingMode =   Qmss_AccPacingMode_LAST_INTERRUPT;
       accCfg.listEntrySize       =   Qmss_AccEntrySize_REG_D;
       accCfg.listCountMode       =   Qmss_AccCountMode_ENTRY_COUNT;
       accCfg.multiQueueMode      =   Qmss_AccQueueMode_SINGLE_QUEUE;

       /* Program the accumulator */
       if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &accCfg)) != QMSS_ACC_SOK)
       {
           System_printf ("Error Programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                        accCfg.channel, accCfg.queMgrIndex, result);
           return -1;
       }

       /* Register interrupts for the system event corresponding to the
        * accumulator channel we are using.
        */
       /* System event 48 - Accumulator Channel 0 */
       eventId     	=   48;

       /* Pick a interrupt vector id to use */
       vectId       =   7;

       /* Register our ISR handle for this event */
       EventCombiner_dispatchPlug (eventId, (EventCombiner_FuncPtr)Cpsw_RxISR, (UArg)NULL, TRUE);

   	   /* Map the combiner's output event id (evevtId/32) to hardware interrupt 8. */
       /* The HW int 8 is slected via CM.eventGroupHwiNum[] specified at cpsw_example.cfg */
       Hwi_eventMap(vectId, 1);

       /* Enable interrupt 8. */
       Hwi_enableInterrupt(vectId);
	}
	else {
    	/* Open a Receive (Rx) queue.
         *
         * This queue will be used to hold all the packets received by PASS/CPSW
         *
         * Open the next available High Priority Accumulation queue for Rx.
         */
        if ((gRxQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, RX_QUEUE_NUM_INIT, &isAllocated)) < 0)
        {
            System_printf ("Error opening a High Priority Accumulation Rx queue \n");
            return -1;
        }
        *rxQInfo = Qmss_getQueueNumber (gRxQHnd);
	}

    return (0);

}

/** ============================================================================
 *   @n@b Cpsw_RxISR
 *
 *   @b Description
 *   @n This API is the example application's High Priority Accumulation interrupt
 *      Service Handler (ISR). This API is called in interrupt context. This API
 *      fetches the Received packet (descriptor) from the accumulator list and
 *      verifies the data received to ensure that it is correct. On success,
 *      this API recycles the Rx descriptor back to Rx free queue for use again.
 *      This API processes the Ping and Pong accumulator lists alternatively.
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
void Cpsw_RxISR (void)
{
    Cppi_Desc*                  pCppiDesc;
    uint32_t                  	count, i;

    /* Process ISR.
     *
     * Get the number of entries in accumulator list.
	 * The hardware enqueues data alternatively to Ping/Pong buffer lists in
     * the accumulator. Hence, we need to track which list (Ping/Pong)
     * we serviced the last time and accordingly process the other one
     * this time around.
     */
     if (!gIsPingListUsed)
     {
		/* Serviced Pong list last time. So read off the Ping list now */
    	count   =   gHiPriAccumList[0];
     }
     else
     {
		/* Serviced Ping list last time. So read off the Pong list now */
    	count   =   gHiPriAccumList[RX_INT_THRESHOLD + 1];
     }

    /* Process all the Results received
     *
     * Skip the first entry in the list that contains the
     * entry count and proceed processing results.
     */
    for (i = 1; i <= count; i ++)
    {
        /* Get the result descriptor.
         *
         * The hardware enqueues data alternatively to Ping/Pong buffer lists in
         * the accumulator. Hence, we need to track which list (Ping/Pong)
         * we serviced the last time and accordingly process the other one
         * this time around.
         */
        if (!gIsPingListUsed)
        {
            /* Serviced Pong list last time. So read off the Ping list now */
            pCppiDesc   =   (Cppi_Desc *) gHiPriAccumList [i];
        }
        else
        {
            /* Serviced Ping list last time. So read off the Pong list now
             *
             * Skip over Ping list length to arrive at Pong list start.
             */
            pCppiDesc   =   (Cppi_Desc *) gHiPriAccumList [i + RX_INT_THRESHOLD + 1];
        }

        /* Descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last
         * 4 bits of the address.
         */
	    pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);

	    VerifyPacket_port (pCppiDesc, dest_emac_port_id, 0xaaaaaaaa);
    }

    /* Clear the accumulator list and save whether we used Ping/Pong
     * list information for next time around.
     */
    if (!gIsPingListUsed)
    {
        /* Just processed Ping list */
        gIsPingListUsed  =   1;

        /* Clear the accumulator list after processing */
        memset ((void *) &gHiPriAccumList [0], 0, sizeof (uint32_t) * (RX_INT_THRESHOLD + 1));
    }
    else
    {
        /* Just processed Pong list */
        gIsPingListUsed  =   0;

        /* Clear the accumulator list after processing */
        memset ((void *) &gHiPriAccumList[RX_INT_THRESHOLD + 1], 0, sizeof (uint32_t) * (RX_INT_THRESHOLD + 1));
    }

	/* Clear INTD */
	Qmss_ackInterrupt(accChannelNum, 1);
	Qmss_setEoiVector(Qmss_IntdInterruptType_HIGH, accChannelNum);

    /* Done processing interrupt. Return */
    return;
}

/***************************************************************************************
 * FUNCTION PURPOSE: Power up PA subsystem
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void passPowerUp (void)
{

    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any
     * PASS device register access. This not required for the simulator. */

    /* Set PASS Power domain to ON */
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_NETCP);

    /* Enable the clocks for PASS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_PA, PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_CPGMAC,  PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_SA,  PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_NETCP);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_NETCP));

}

/** ============================================================================
 *   @n@b TrigPacketToPdsp5
 *
 *   @b Description
 *   @n This API is called to trigger the packet tobe send to an interface via PDSP5.
 *      On success, this API increments a global Tx counter to indicate the same.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t TrigPacketToPdsp5(int emac_dest_port)
{
    Cppi_Desc*      pCppiDesc;
    uint32_t        dataBufferSize;
    char            psFlags = (cpswSimTest)?pa_EMAC_PORT_NOT_SPECIFIED:(char)(emac_dest_port + pa_EMAC_PORT_0);
    uint8_t         *pkt = pktMatch;

    if (emac_dest_port == -1) {
    	psFlags = pa_EMAC_PORT_NOT_SPECIFIED;
        pkt = pktDrop;
    }

    paCmdInfo_t     cmdInfo;
    uint32_t        cmdBuf[4];
    uint16_t        cmdSize = sizeof(cmdBuf);

    paCmdNextRoute_t routeCmdEth = {
                                    0,              /*  ctrlBitfield */
                                    pa_DEST_EMAC,   /* Route - host       */
                                    0,              /* pktType don't care */
                                    0,              /* flow Id              */
                                    0,  			/* Queue                */
                                    0,              /* SWInfo 0             */
                                    0,              /* SWInfo 1 */
                                    0               /* multiRouteIndex (not used) */
                                 };

    routeCmdEth.pktType_emacCtrl = psFlags;

    /* Command : Next route */
    cmdInfo.cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo.params.route = routeCmdEth;

    /* Get a free descriptor from the global free queue we setup
     * during initialization.
     */
    if ((pCppiDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("No Tx free descriptor. Cant run send/rcv test \n");
        return -1;
    }

    /* The descriptor address returned from the hardware has the
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor size, always mask off the last
     * 4 bits of the address.
     */
    pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);

    dataBufferSize  = sizeof (pktMatch);
    Cppi_setData (  Cppi_DescType_HOST,
                    (Cppi_Desc *) pCppiDesc,
                    (uint8_t *) Convert_CoreLocal2GlobalAddr((uint32_t)pkt),
                    dataBufferSize
                 );
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, dataBufferSize);

    Pa_formatTxCmd (  1,        /* nCmd */
                      &cmdInfo,  /* command info */
                      0,        /* offset */
                      (Ptr)&cmdBuf[0],          /* Command buffer       */
                      &cmdSize);    /* Command size         */

    /* Attach the command in PS data */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, (uint8_t *)cmdBuf, cmdSize);

    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, 0);

    if (pdsp_halt)
    	mdebugHaltPdsp(nssGblCfgParams.layout.qPaTxCmdIndex);

    Qmss_queuePush (gPaTxQHnd[nssGblCfgParams.layout.qPaTxCmdIndex], pCppiDesc, dataBufferSize, SIZE_HOST_DESC, Qmss_Location_TAIL);

    /* Increment the application transmit counter */
    actual.txCount ++;

    /* Give some time for the PA to process the packet */
    CycleDelay (10000);

    return 0;
}
/** ============================================================================
 *   @n@b TrigPacketToPdsp0
 *
 *   @b Description
 *   @n This API is called to trigger the packet tobe send to pdsp0 from an interface.
 *      On success, this API increments a global Tx counter to indicate the same.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t TrigPacketToPdsp0(int emac_dest_port)
{
    Cppi_Desc*      pCppiDesc;
    uint32_t        dataBufferSize;
    char            psFlags = (cpswSimTest)?pa_EMAC_PORT_NOT_SPECIFIED:(char)(emac_dest_port + pa_EMAC_PORT_0);
    Cppi_DescTag    tag;

    /* Get a free descriptor from the global free queue we setup
     * during initialization.
     */
    if ((pCppiDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("No Tx free descriptor. Cant run send/rcv test \n");
        return -1;
    }

    /* The descriptor address returned from the hardware has the
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor size, always mask off the last
     * 4 bits of the address.
     */
    pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);

    dataBufferSize  =   sizeof (pktMatch);
    //dataBufferSize  =   1000;
    Cppi_setData (  Cppi_DescType_HOST,
                    (Cppi_Desc *) pCppiDesc,
                    (uint8_t *) Convert_CoreLocal2GlobalAddr((uint32_t)pktMatch),
                    dataBufferSize
                 );
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, dataBufferSize);

    /* Force the packet to the specific EMAC port */
    if (!nssGblCfgParams.layout.fNssGen2)
    {
        Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, psFlags);
    }
    else
    {
        tag.srcTagHi  = 0;
        tag.srcTagLo  = 0;
        tag.destTagHi = 0;
        tag.destTagLo = psFlags;
        Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, (Cppi_DescTag *)&tag);
    }

    /* Clear PS Data */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, 0);

    if (pdsp_halt)
    	mdebugHaltPdsp(0);
	/* Send the packet out the mac. It will loop back to PA if the mac/switch
     * have been configured properly
     */
    if (no_bootMode == TRUE)
    	Qmss_queuePush (gPaTxQHnd[nssGblCfgParams.layout.qCpswEthIndex], pCppiDesc, dataBufferSize, SIZE_HOST_DESC, Qmss_Location_TAIL);
    else {
        Qmss_queuePush (gPaTxQHnd[nssGblCfgParams.layout.qPaInputIndex], pCppiDesc, dataBufferSize, SIZE_HOST_DESC, Qmss_Location_TAIL);
    }

    /* Increment the application transmit counter */
    actual.txCount ++;

    /* Give some time for the PA to process the packet */
    //CycleDelay (10000);

    return 0;
}

/** ============================================================================
 *   @n@b VerifyPacket
 *
 *   @b Description
 *   @n This API verifies a packet received against the expected data and
 *      returns 0 to inidcate success and -1 to indicate a mismatch.
 *
 *   @param[in]
 *   @n pCppiDesc           Packet descriptor received.
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t VerifyPacket_queue (int direction, int emac_dest_port, uint32_t swInfoMatch)
{
    Cppi_Desc                   *pCppiDesc;
	Cppi_HostDesc               *pHostDesc;
	uint8_t                     *pDataBuffer;
	int32_t                      i;
	Qmss_QueueHnd                queue;

    /* Pop the descriptor from captured queue */
    /* Get a free descriptor from the global free queue we setup
     * during initialization.
     */
	if (direction)
	{
		queue = dest_emac_port_id + PA_PKT_CAP_INGRESS_CAP_BASE_QUEUE;
	}
	else
	{
		queue = dest_emac_port_id + PA_PKT_CAP_EGRESS_CAP_BASE_QUEUE;
	}
    while ((pCppiDesc = Qmss_queuePop (queue)) != NULL)
    {
		/* The descriptor address returned from the hardware has the
		 * descriptor size appended to the address in the last 4 bits.
		 *
		 * To get the true descriptor size, always mask off the last
		 * 4 bits of the address.
		 */
		pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);

		pHostDesc = (Cppi_HostDesc *)pCppiDesc;

		/* Verify the application software info we received is same
		 * as what we had sent earlier.
		 */
		if (pHostDesc->softwareInfo0 != swInfoMatch)
		{
			System_printf ("VerifyPacket: Found an entry in receive queue with swinfo0 = 0x%08x, expected 0x%08x\n",
							pHostDesc->softwareInfo0, 0xaaaaaaaa);

			pHostDesc->buffLen = pHostDesc->origBufferLen;
			Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

			return -1;
		}

		/* Verify the packet matches what we had sent */
		pDataBuffer = (uint8_t *) pHostDesc->buffPtr;
		for (i = 42; i < sizeof (pktMatch); i++)
		{
			if (pktMatch[i] != pDataBuffer[i])
			{
				System_printf ("VerifyPacket: Byte %d expected 0x%02x, found 0x%02x\n", i, pktMatch[i], pDataBuffer[i]);
				System_flush();

				/* Free the packet back to the Rx FDQ */
				pHostDesc->buffLen = pHostDesc->origBufferLen;
				Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
				return -1;
			}
		}

		//System_printf ("Packet Received Verified Successfully!\n");

		/* Increment Rx counter to indicate the number of successfully
		 * received packets by the example app.
		 */
		actual.cloneCaptureCount ++;

		/* Reset the buffer lenght and put the descriptor back on the free queue */
		pHostDesc->buffLen = pHostDesc->origBufferLen;
		Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
    }
    /* Verify packet done. Return success. */
	return 0;
}

/** ============================================================================
 *   @n@b VerifyPacket_portMirror
 *
 *   @b Description
 *   @n This API verifies a packet received against the expected data and
 *      returns 0 to inidcate success and -1 to indicate a mismatch.
 *
 *   @param[in]
 *   @n pCppiDesc           Packet descriptor received.
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t VerifyPacket_port (Cppi_Desc* pCppiDesc, int emac_dest_port, uint32_t swInfoMatch)
{
	Cppi_HostDesc               *pHostDesc;
	uint8_t                       *pDataBuffer;
	int32_t                       i;
	uint32_t		      	        infoLen;
	pasahoLongInfo_t 	        *pinfo;
    uint8_t                       portNum;

    pHostDesc = (Cppi_HostDesc *)pCppiDesc;

    /* Verify the application software info we received is same
     * as what we had sent earlier.
     */
    if (pHostDesc->softwareInfo0 != swInfoMatch)
    {
        System_printf ("VerifyPacket: Found an entry in receive queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                        pHostDesc->softwareInfo0, 0xaaaaaaaa);

        pHostDesc->buffLen = pHostDesc->origBufferLen;
        Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

        return -1;
    }

	/* Get the parse information, make sure there is an L4 offset */
	if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)pHostDesc, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
		System_printf ("VerifyPacket: Error getting control info from received data packet\n");
		return (-1);
	}
    else if(!cpswSimTest)
    {
    	/* do not check the port number if linux boot is true */
        if ( (no_bootMode == TRUE) && (emac_dest_port != -1) )
        {

           /* Verify the input port number */
           portNum = PASAHO_LINFO_READ_INPORT(pinfo);

           if ( (portNum == (pa_EMAC_PORT_0 + emac_dest_port)) )
           {
                /* actual port number */
        	   actual.emacRxCount ++;
           }
           else if ( (portNum == INGRESS_MIRROR_PORT) ||
        		     (portNum == EGRESS_MIRROR_PORT) )
           {
                /* expected port number */
        	   actual.cloneCaptureCount ++;
           }
           else
           {
        	   /* Un expected port for NON CPPI Port egress test */
	           System_printf ("VerifyPacket: receive packet from unexpected EMAC PORT %d (expected %d)\n", portNum - 1, emac_dest_port);
               System_flush();
           }
        }
    }

    /* Verify the packet matches what we had sent */
    pDataBuffer = (uint8_t *) pHostDesc->buffPtr;
    for (i = 42; i < sizeof (pktMatch); i++)
    {
        if (pktMatch[i] != pDataBuffer[i])
        {
            System_printf ("VerifyPacket: Byte %d expected 0x%02x, found 0x%02x\n", i, pktMatch[i], pDataBuffer[i]);
            System_flush();

            /* Free the packet back to the Rx FDQ */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
            return -1;
        }
    }

    /* Reset the buffer lenght and put the descriptor back on the free queue */
    pHostDesc->buffLen = pHostDesc->origBufferLen;
    Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

    /* Verify packet done. Return success. */
	return 0;
}

/** ============================================================================
 *   @n@b Download_PAFirmware
 *
 *   @b Description
 *   @n This API downloads the PA firmware required for PDSP operation.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Download_PAFirmware (void)
{
    extern Pa_Handle                               gPAInstHnd;

    int i;

    /* Hold the PA in reset state during download */
    Pa_resetControl (gPAInstHnd, pa_STATE_RESET);


	for ( i = 0; i < nssGblCfgParams.layout.numPaPdsps; i++)
    {

        Pa_downloadImage (gPAInstHnd, i,
                          (Ptr)nssGblCfgParams.layout.paPdspImage[i],
                          nssGblCfgParams.layout.paPdspImageSize[i]);
    }

    /* Enable the PA back */
    Pa_resetControl (gPAInstHnd, pa_STATE_ENABLE);

    return 0;
}

void CycleDelay (int32_t count)
{
    uint32_t                  TSCLin;

    if (count <= 0)
        return;

    /* Get the current TSCL  */
    TSCLin = TSCL ;

    while ((TSCL - TSCLin) < (uint32_t)count);
}

void APP_exit (int32_t code)
{
	BIOS_exit(code);
}


int  setupFramework(void)
{
#if RM
    if (setupRm ())
    {
      System_printf ("Function setupRm failed\n");
      System_flush();
      return (-1);
    }
#endif
    /* Initialize the components required to run the example:
     *  (1) QMSS
     *  (2) CPPI
     *  (3) Ethernet switch subsystem + MDIO + SGMII
     */

    /* Initialize QMSS */
    if (Init_Qmss () != 0)
    {
        System_printf ("QMSS init failed \n");
        System_flush();
        return (-1);
    }
    else
    {
        System_printf ("QMSS successfully initialized \n");
        System_flush();
    }

    /* Initialize CPPI */
    if (Init_Cppi () != 0)
    {
        System_printf ("CPPI init failed \n");
        System_flush();
        return (-1);
    }
    else
    {
        System_printf ("CPPI successfully initialized \n");
        System_flush();
    }

    /* Init PA LLD */
    if (Init_PASS () != 0)
    {
        System_printf ("PASS init failed \n");
        System_flush();
        return  (-1);
    }
    else
    {
        System_printf ("PASS successfully initialized \n");
        System_flush();
    }
#ifndef __LINUX_USER_SPACE
    if (no_bootMode == TRUE)
    {
        /* Initialize the CPSW switch */
        if (Init_Cpsw () != 0)
        {
            System_printf ("Ethernet subsystem init failed \n");
            System_flush();
            return (-1);
        }
        else
        {
            System_printf ("Ethernet subsystem successfully initialized \n");
            System_flush();
        }
    }
#endif

    /* Setup Tx */
    if (Setup_Tx () != 0)
    {
        System_printf ("Tx setup failed \n");
        System_flush();
        return (-1);
    }
    else
    {
        System_printf ("Tx setup successfully done \n");
        System_flush();
    }

    /* Setup Rx */
    if (Setup_Rx () != 0)
    {
        System_printf ("Rx setup failed \n");
        System_flush();
        return (-1);
    }
    else
    {
        System_printf ("Rx setup successfully done \n");
        System_flush();
    }

    /* Setup PA */
    if (Setup_PASS () != 0)
    {
        System_printf ("PASS setup failed \n");
        System_flush();
        return (-1);
    }
    else
    {
        System_printf ("PASS setup successfully done \n");
        System_flush();
    }

    return 0;
}
int32_t pa_global_config (paCtrlInfo_t* cfgInfo, uint32_t swInfoId )
{
    int32_t                       j;
    uint16_t                      cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */
                                                    0,                                          /* User chosen ID to go to swinfo0 */
                                                    0,                                          /* Destination queue */
                                                    0                                           /* Flow ID */
                                                };
    paReturn_t        retVal;
    paEntryHandle_t   retHandle;
    int32_t           handleType, cmdDest;
    uint32_t          psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t          myswinfo[]  =   {0x11112222, 0x33334444};
    Cppi_HostDesc*    pHostDesc;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");
        return -1;
    }

    /* The descriptor address returned from the hardware has the
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);

    /* Populate the Rx free descriptor with the fixed command buffer. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   swInfoId;  /* unique for each pa control command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_control  (gPAInstHnd,
                             cfgInfo,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)
    {
        System_printf ("Pa_control returned error %d\n", retVal);
        return -1;
    }

    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot
     */
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);

    if (pdsp_halt)
    	mdebugHaltPdsp(cmdDest);

    /* Send the command to the PA and wait for the return */
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    pHostDesc,
                    pHostDesc->buffLen,
                    SIZE_HOST_DESC,
                    Qmss_Location_TAIL
                   );

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)
        {
            /* We have a response from PA PDSP for the command we submitted earlier for
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)
            {
                System_printf ("PA sub-system rejected Pa_control command\n");
                return -1;
            }

            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)
    {
        System_printf ("pa_global_config(): Timeout waiting for reply from PA to Pa_control command\n");
        return -1;
    }

    return 0;
}

int32_t pcap_enable_global_ingress ( )
{
	paCtrlInfo_t cfgInfo;
	paPacketControl2Config_t pktCtrl2Cfg;
	uint32_t swInfoId = 0xFFFF0001;

	memset (&cfgInfo, 0, sizeof (cfgInfo));
	memset (&pktCtrl2Cfg, 0, sizeof (pktCtrl2Cfg));

	pktCtrl2Cfg.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_IGRESS_CLONE;
	pktCtrl2Cfg.ctrlBitMap  = pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE;

	cfgInfo.code = pa_CONTROL_SYS_CONFIG;
	cfgInfo.params.sysCfg.pPktControl2 = &pktCtrl2Cfg;

	pa_global_config (&cfgInfo, swInfoId );

	return (0);
}

int32_t pcap_enable_global_egress ( )
{
	paCtrlInfo_t cfgInfo;
	paPacketControl2Config_t pktCtrl2Cfg;
	uint32_t swInfoId = 0xFFFF0002;

	memset (&cfgInfo, 0, sizeof (cfgInfo));
	memset (&pktCtrl2Cfg, 0, sizeof (pktCtrl2Cfg));

	pktCtrl2Cfg.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_CLONE;
	pktCtrl2Cfg.ctrlBitMap  = pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE;

	cfgInfo.code = pa_CONTROL_SYS_CONFIG;
	cfgInfo.params.sysCfg.pPktControl2 = &pktCtrl2Cfg;

	pa_global_config (&cfgInfo, swInfoId );

	return 0;
}

int32_t pcap_disable_global_ingress ( )
{
	paCtrlInfo_t cfgInfo;
	paPacketControl2Config_t pktCtrl2Cfg;
	uint32_t swInfoId = 0xFFFF0003;

	memset (&cfgInfo, 0, sizeof (cfgInfo));
	memset (&pktCtrl2Cfg, 0, sizeof (pktCtrl2Cfg));

	pktCtrl2Cfg.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_IGRESS_CLONE;
	pktCtrl2Cfg.ctrlBitMap  = ~pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE;

	cfgInfo.code = pa_CONTROL_SYS_CONFIG;
	cfgInfo.params.sysCfg.pPktControl2 = &pktCtrl2Cfg;

	pa_global_config (&cfgInfo, swInfoId );

	return 0;
}

int32_t pcap_disable_global_egress ( )
{
	paCtrlInfo_t cfgInfo;
	paPacketControl2Config_t pktCtrl2Cfg;
	uint32_t swInfoId = 0xFFFF0004;

	memset (&cfgInfo, 0, sizeof (cfgInfo));
	memset (&pktCtrl2Cfg, 0, sizeof (pktCtrl2Cfg));

	pktCtrl2Cfg.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_CLONE;
	pktCtrl2Cfg.ctrlBitMap  = ~pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE;

	cfgInfo.code = pa_CONTROL_SYS_CONFIG;
	cfgInfo.params.sysCfg.pPktControl2 = &pktCtrl2Cfg;

	pa_global_config (&cfgInfo, swInfoId );

	return 0;
}

int32_t pcap_global_queue_bounce_config (int enable)
{
	paCtrlInfo_t cfgInfo;
    paQueueBounceConfig_t queueBounceDisable;
    uint32_t swInfoId = 0xFFFF0005;

	memset (&cfgInfo, 0, sizeof (cfgInfo));
    memset (&queueBounceDisable, 0, sizeof(paQueueBounceConfig_t));

	cfgInfo.code = pa_CONTROL_SYS_CONFIG;
    cfgInfo.params.sysCfg.pQueueBounceConfig = (enable)?&pcapQueueBounceCfg:
                                                        &queueBounceDisable;

	pa_global_config (&cfgInfo, swInfoId );

	return 0;
}

static void pcap_relay_queue_bounce_pkts (uint32_t ddrQ, uint32_t msmcQ, int* ddrCnt, int* msmcCnt)
{
	Cppi_HostDesc    *hd;
	uint32_t		 *swInfo;
    int              ddrCount = 0, msmcCount = 0;
    uint32_t         queueId;
    Qmss_QueueHnd    ddrQHnd  = Qmss_getHandleFromQID(ddrQ);
    Qmss_QueueHnd    msmcQHnd = Qmss_getHandleFromQID(msmcQ);
    /*
     * Look for entries in the Queue bounce DDR queue
     */
	while (Qmss_getQueueEntryCount(ddrQHnd) > 0)  {

	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (ddrQHnd)) & ~15);
	    if (hd == NULL)  {
		    System_printf ("pcap_relay_queue_bounce_pkts: Failed to pop a Queue Bounce DDR queue packet\n");
		    break;
	    }

		Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
        queueId = swInfo[1] &0x3FFF;
		Qmss_queuePushDescSize (Qmss_getHandleFromQID(queueId), (Ptr)hd, SIZE_HOST_DESC);

        ddrCount++;

	}

    /*
     * Look for entries in the Queue bounce DDR queue
     */
	while (Qmss_getQueueEntryCount(msmcQHnd) > 0)  {

	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (msmcQHnd)) & ~15);
	    if (hd == NULL)  {
		    System_printf ("pcap_relay_queue_bounce_pkts: Failed to pop a Queue Bounce MSMC queue packet\n");
		    break;
	    }

		Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
        queueId = swInfo[1] &0x3FFF;
		Qmss_queuePushDescSize (Qmss_getHandleFromQID(queueId), (Ptr)hd, SIZE_HOST_DESC);

        msmcCount++;

	}

    if(ddrCnt)*ddrCnt += ddrCount;
    if(msmcCnt)*msmcCnt += msmcCount;
}


int32_t  pkt_capture_test(int ingress, int dest_emac_port_id)
{
	paCtrlInfo_t    paCtrl;
	uint32_t        cmdId;
	int             i, retVal = 0;
    int             maxRetry = MAX_RETRIES;
    paReturn_t        paRetVal;
    int             bounceCount = 0;

    /* Enable Queue Bounce */
    pcap_global_queue_bounce_config(1);

	/* Can be at most pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES */
	paPktCaptureConfig_t pktCapCfg[1];

	/* Clear the paCtrl before configuration */
    memset (&paCtrl, 0, sizeof (paCtrl));

    /* Clear the expected counts */
    memset (&expected, 0, sizeof (expected));

    /* Clear the packet capture config structure */
    memset (pktCapCfg, 0, sizeof (pktCapCfg));

    /* 1. Ingress Packet Capture test */
    cmdId =  0xbbbbcccc;

    if (ingress)
    {
    	pktCapCfg[0].ctrlBitMap = pa_PKT_CLONE_ENABLE | pa_PKT_CLONE_INGRESS;
    	pktCapCfg[0].portToBeCaptured = (dest_emac_port_id + pa_EMAC_PORT_0);
    	pktCapCfg[0].flowId           = PA_PKT_CAP_FLOW;
    	pktCapCfg[0].queue            = (dest_emac_port_id + PA_PKT_CAP_INGRESS_CAP_BASE_QUEUE);
    	pktCapCfg[0].swInfo0          = PA_PKT_CAP_SWINFO_HIGH_WORD | dest_emac_port_id;

    	paCtrl.code = pa_CONTROL_EMAC_PORT_CONFIG;
        paCtrl.params.emacPortCfg.numEntries = 1;
        paCtrl.params.emacPortCfg.cfgType   = pa_EMAC_PORT_CFG_PKT_CAPTURE;
        paCtrl.params.emacPortCfg.u.pktCapCfg = &pktCapCfg[0];

        paRetVal = pa_global_config(&paCtrl, cmdId);
        if (paRetVal != pa_OK)
        	return -1;
        paRetVal = pcap_enable_global_ingress();
        if (paRetVal != pa_OK)
        	return -1;

    }
    else
    {
    	pktCapCfg[0].ctrlBitMap = pa_PKT_CLONE_ENABLE;
    	pktCapCfg[0].portToBeCaptured = (dest_emac_port_id + pa_EMAC_PORT_0);
    	pktCapCfg[0].flowId           = PA_PKT_CAP_FLOW;
    	pktCapCfg[0].queue            = (dest_emac_port_id + PA_PKT_CAP_EGRESS_CAP_BASE_QUEUE);
    	pktCapCfg[0].swInfo0          = PA_PKT_CAP_SWINFO_HIGH_WORD | dest_emac_port_id;

    	paCtrl.code = pa_CONTROL_EMAC_PORT_CONFIG;
        paCtrl.params.emacPortCfg.numEntries = 1;
        paCtrl.params.emacPortCfg.cfgType   = pa_EMAC_PORT_CFG_PKT_CAPTURE;
        paCtrl.params.emacPortCfg.u.pktCapCfg = &pktCapCfg[0];

        paRetVal = pa_global_config(&paCtrl, cmdId);
        if (paRetVal != pa_OK)
        	return -1;
        paRetVal = pcap_enable_global_egress();
        if (paRetVal != pa_OK)
        	return -1;
    }

	/* Set the expectations */
    if (dest_emac_port_id != -1)
    {
    	expected.emacRxCount       =   MAX_NUM_PACKETS;
        expected.cloneCaptureCount =   MAX_NUM_PACKETS;
    }
    else
    {
    	expected.emacRxCount       =   0; /* CPPI egress test */
        expected.cloneCaptureCount =   MAX_NUM_PACKETS;
    }
	expected.txCount               =   MAX_NUM_PACKETS;

	/* Clear the acutal counts */
	memset (&actual, 0, sizeof (actual));

    /* This would do a packet capture at the queue configured from ingress traffic */
	for (i = 0; i < MAX_NUM_PACKETS; i ++)
    {
		if (ingress) {
			if (TrigPacketToPdsp0 (dest_emac_port_id) != 0)
			{
				System_printf ("Packet %d send to pdsp0 failed \n", i);
			   System_flush();
			   return (-1);
			}
		}
		else
		{
			if (TrigPacketToPdsp5 (dest_emac_port_id) != 0)
			{
			   System_printf ("Packet %d send to pdsp5 failed \n", i);
			   System_flush();
			   return (-1);
			}
		}
    }

    /* Check if expected values match with actual values */
    maxRetry = MAX_NUM_PACKETS*2;
	do {
		 /* Wait for some cycles to receive all packets */
		 CycleDelay(200000);

         pcap_relay_queue_bounce_pkts(PA_PKT_CAP_QUEUE_BOUNCE_QUEUE_DDR,
                                      PA_PKT_CAP_QUEUE_BOUNCE_QUEUE_MSMC,
                                      &bounceCount, &bounceCount);

	    /* This would validate all the captured packets in the host */
        if (VerifyPacket_queue(ingress, dest_emac_port_id, PA_PKT_CAP_SWINFO_HIGH_WORD | dest_emac_port_id) != 0)
        {
    	    System_printf (" verify packet for host error \n");
    	    System_flush();
            retVal = -1;
            break;
	    }

		if (memcmp(&expected,&actual, sizeof(pktStats_t)))
		{
			if (--maxRetry == 0) {
				System_printf (" Packet capture test Failed \n");
				System_flush();
				retVal = -1;
			}
		}
		else {
			break;
		}
	}while (maxRetry);

    if (ingress)
    {
    	if (retVal == 0)
    		System_printf ("|Ingress Packet Capture Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|PASS\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    	else
    		System_printf ("|Ingress Packet Capture Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|FAIL\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    }
    else
    {
    	if (retVal == 0)
    		System_printf ("|Egress Packet Capture Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|PASS\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    	else
    		System_printf ("|Egress Packet Capture Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|FAIL\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    }

    //System_printf("pkt_capture_test: receives %d queue bounce packets\n", bounceCount);

    System_flush();

    /* Disable Queue Bounce */
    pcap_global_queue_bounce_config(0);

    paRetVal = pcap_disable_global_ingress();
    if (paRetVal != pa_OK)
    	return (-1);
    paRetVal = pcap_disable_global_egress();
    if (paRetVal != pa_OK)
    	return (-1);
    return (retVal);
}

int32_t  port_mirror_test(int direction, int dest_emac_port_id)
{
	paCtrlInfo_t    paCtrl;
	uint32_t        cmdId;
    int             i, maxRetry = MAX_RETRIES;
    int32_t         retVal = 0;
    paReturn_t      paRetVal;

    /* can be at most pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES */
    paPortMirrorConfig_t mirrorCfg[1];

	/* Clear the paCtrl before configuration */
    memset (&paCtrl, 0, sizeof (paCtrl));

    /* Clear the expected counts */
    memset (&expected, 0, sizeof (expected));

    /* Clear the mirror configuration */
    memset (&mirrorCfg[0], 0, sizeof (paPortMirrorConfig_t));

    /* 1. Ingress Packet Capture test */
    cmdId =  0xbbbbcccc;

    if (direction)
    {
    	mirrorCfg[0].ctrlBitMap = pa_PKT_CLONE_ENABLE | pa_PKT_CLONE_INGRESS;
    	mirrorCfg[0].portToBeMirrored = (dest_emac_port_id + pa_EMAC_PORT_0);
    	mirrorCfg[0].mirrorPort       = INGRESS_MIRROR_PORT;

    	paCtrl.code = pa_CONTROL_EMAC_PORT_CONFIG;
        paCtrl.params.emacPortCfg.numEntries = 1;
        paCtrl.params.emacPortCfg.cfgType   = pa_EMAC_PORT_CFG_MIRROR;
        paCtrl.params.emacPortCfg.u.mirrorCfg  = &mirrorCfg[0];


        paRetVal = pa_global_config(&paCtrl, cmdId);
        if (paRetVal != pa_OK)
        	return -1;
        paRetVal = pcap_enable_global_ingress();
        if (paRetVal != pa_OK)
        	return -1;

    }
    else
    {
    	mirrorCfg[0].ctrlBitMap = pa_PKT_CLONE_ENABLE;
    	mirrorCfg[0].portToBeMirrored = (dest_emac_port_id + pa_EMAC_PORT_0);
    	mirrorCfg[0].mirrorPort       = EGRESS_MIRROR_PORT;

    	paCtrl.code = pa_CONTROL_EMAC_PORT_CONFIG;
        paCtrl.params.emacPortCfg.numEntries = 1;
        paCtrl.params.emacPortCfg.cfgType   = pa_EMAC_PORT_CFG_MIRROR;
        paCtrl.params.emacPortCfg.u.mirrorCfg  = &mirrorCfg[0];

        pa_global_config(&paCtrl, cmdId);
        pcap_enable_global_egress();
    }

	/* Set the expectations */
    expected.cloneCaptureCount =   MAX_NUM_PACKETS;
	expected.emacRxCount       =   MAX_NUM_PACKETS;
	expected.txCount           =   MAX_NUM_PACKETS;

	/* Clear the acutal counts */
	memset (&actual, 0, sizeof (actual));

    /* This would do a packet capture at the queue configured from ingress traffic */
	for (i = 0; i < MAX_NUM_PACKETS; i ++)
    {
		if (direction) {
			if (TrigPacketToPdsp0 (dest_emac_port_id) != 0)
			{
				System_printf ("Packet %d send to pdsp0 failed \n", i);
			   System_flush();
			   return (-1);
			}
		}
		else
		{
			if (TrigPacketToPdsp5 (dest_emac_port_id) != 0)
			{
			   System_printf ("Packet %d send to pdsp5 failed \n", i);
			   System_flush();
			   return (-1);
			}
		}

    }

    /* Check if expected values match with actual values */
    maxRetry = MAX_NUM_PACKETS*2;
	do {
	        /* Wait for some cycles to receive all packets */
			CycleDelay(200000);

		    if (memcmp(&expected,&actual, sizeof(pktStats_t)))
		    {
			    if (--maxRetry == 0) {
				    System_printf (" Port Mirror test Failed \n");
				    System_flush();
				    retVal = -1;
			    }
		    }
		    else {
			    break;
		    }

	}while (maxRetry);

    if (direction)
    {
    	if (retVal == 0)
    		System_printf ("|Ingress Port Mirror Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|PASS\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    	else
    		System_printf ("|Ingress Port Mirror Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|FAIL\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    }
    else
    {
    	if (retVal == 0)
    		System_printf ("|Egress Port Mirror Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|PASS\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    	else
    		System_printf ("|Egress Port Mirror Test \t|%d\t\t|%d\t\t|%d\t\t|%d\t\t\t|FAIL\n", dest_emac_port_id + pa_EMAC_PORT_0,actual.txCount, actual.emacRxCount, actual.cloneCaptureCount);
    }

    if (direction)
    {
    	paRetVal = pcap_disable_global_ingress();
    }
    else
    {
    paRetVal = pcap_disable_global_egress();
    }

    if (paRetVal != pa_OK)
    	return -1;

    System_flush();

    return (retVal);
}
/* Nothing past this point */
