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
#include "cpsw_singlecore.h"
#include "ti/drv/pa/pa.h"
#include "fw_test.h"
#include "fw_mem_allocator.h"
#include "ti/drv/pa/pasahost.h"
extern Qmss_QueueHnd gRxQHnd;

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

/******************************************************************************
* Macro to convert to IP Register Virtual Address from a mapped base Virtual Address
* Input: virtBaseAddr: Virtual base address mapped using mmap for IP
*        phyBaseAddr: Physical base address for the IP
*        phyRegAddr:  Physical register address
******************************************************************************/
static inline void* FW_GET_REG_VADDR (void * virtBaseAddr, uint32_t phyBaseAddr, uint32_t phyRegAddr)
{
    return((void *)((uint8_t *)virtBaseAddr + (phyRegAddr - phyBaseAddr)));
}

/** ============================================================================
 *   @n@b CycleDelay
 *
 *   @b Description
 *   @n This API implements a clock delay logic using the Time Stamp Counter (TSC)
 *      of the DSP.
 *
 *   @param[in]
 *   @n count               Number of delay cycles to wait.
 *
 *   @return
 *   @n None
 * =============================================================================
 */

void CycleDelay (int32_t count)
{
    volatile int32_t                  i;

    if (count <= 0)
        return;

	for (i=0;i<count;i++);
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
    return (addr);
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
   return (addr);
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
    uint32_t                    count;
    /* Convert address to Virtual address */
    for(count=0;count < fw_qmssGblCfgParams->maxQueMgrGroups;count++)
    {

        fw_qmssGblCfgParams->groupRegs[count].qmConfigReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmConfigReg);

        fw_qmssGblCfgParams->groupRegs[count].qmDescReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmDescReg);

        fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtReg);

        fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtProxyReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtProxyReg);

        fw_qmssGblCfgParams->groupRegs[count].qmQueStatReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmQueStatReg);

        fw_qmssGblCfgParams->groupRegs[count].qmStatusRAM =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmStatusRAM);

        fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtDataReg =
            FW_GET_REG_VADDR(fw_qmssDataVaddr,QMSS_DATA_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtDataReg);

        fw_qmssGblCfgParams->groupRegs[count].qmQueMgmtProxyDataReg = NULL; /* not supported on k2 hardware, and not used by lld */
    }

    for(count=0;count < QMSS_MAX_INTD;count++)
    {
        fw_qmssGblCfgParams->regs.qmQueIntdReg[count] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->regs.qmQueIntdReg[count]);
    }

    for(count=0;count < QMSS_MAX_PDSP;count++)
    {
        fw_qmssGblCfgParams->regs.qmPdspCmdReg[count] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->regs.qmPdspCmdReg[count]);

        fw_qmssGblCfgParams->regs.qmPdspCtrlReg[count] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->regs.qmPdspCtrlReg[count]);

        fw_qmssGblCfgParams->regs.qmPdspIRamReg[count] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->regs.qmPdspIRamReg[count]);
    }

    fw_qmssGblCfgParams->regs.qmLinkingRAMReg =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->regs.qmLinkingRAMReg);

    fw_qmssGblCfgParams->regs.qmBaseAddr =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,(uint32_t)fw_qmssGblCfgParams->regs.qmBaseAddr);

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
    /* Convert Physical address to Virtual address for LLD access */
#if defined(SOC_K2K) || defined(SOC_K2H)
    /* SRIO CPDMA regs */
    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_SRIO_CPDMA].gblCfgRegs =
        FW_GET_REG_VADDR(fw_srioCfgVaddr,CSL_SRIO_CFG_REGS,CSL_SRIO_CFG_PKTDMA_GLOBAL_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_SRIO_CPDMA].txChRegs =
        FW_GET_REG_VADDR(fw_srioCfgVaddr,CSL_SRIO_CFG_REGS,CSL_SRIO_CFG_PKTDMA_TX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_SRIO_CPDMA].rxChRegs =
        FW_GET_REG_VADDR(fw_srioCfgVaddr,CSL_SRIO_CFG_REGS,CSL_SRIO_CFG_PKTDMA_RX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_SRIO_CPDMA].txSchedRegs =
        FW_GET_REG_VADDR(fw_srioCfgVaddr,CSL_SRIO_CFG_REGS,CSL_SRIO_CFG_PKTDMA_TX_SCHEDULER_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_SRIO_CPDMA].rxFlowRegs =
        FW_GET_REG_VADDR(fw_srioCfgVaddr,CSL_SRIO_CFG_REGS,CSL_SRIO_CFG_PKTDMA_RX_FLOW_CFG_REGS);

#endif

    /* PASS CPDMA regs */
    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].gblCfgRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_GLOBAL_CFG_REGS);

#if defined(SOC_K2L) || defined(SOC_K2E)
    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].txChRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_GLOBAL_TX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].rxChRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_GLOBAL_RX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].txSchedRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_GLOBAL_TX_SCHEDULER_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].rxFlowRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_GLOBAL_RX_FLOW_CFG_REGS);
#else
    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].txChRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_TX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].rxChRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_RX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].txSchedRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_TX_SCHEDULER_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_PASS_CPDMA].rxFlowRegs =
        FW_GET_REG_VADDR(fw_passCfgVaddr,CSL_NETCP_CFG_REGS,CSL_NETCP_CFG_PKTDMA_RX_FLOW_CFG_REGS);

#endif

    /* QMSS CPDMA regs */
    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_QMSS_CPDMA].gblCfgRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,CSL_QMSS_CFG_PKTDMA_1_GLOBAL_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_QMSS_CPDMA].txChRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,CSL_QMSS_CFG_PKTDMA_1_TX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_QMSS_CPDMA].rxChRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,CSL_QMSS_CFG_PKTDMA_1_RX_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_QMSS_CPDMA].txSchedRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,CSL_QMSS_CFG_PKTDMA_1_TX_SCHEDULER_CFG_REGS);

    fw_cppiGblCfgParams->cpDmaCfgs[Cppi_CpDma_QMSS_CPDMA].rxFlowRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,QMSS_CFG_BASE_ADDR,CSL_QMSS_CFG_PKTDMA_1_RX_FLOW_CFG_REGS);

	return;
}

int32_t setup_rx_queue(Qmss_Queue *rxQInfo)
{
    uint8_t                       isAllocated;

	/* Open a Receive (Rx) queue */
    if ((gRxQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error opening a High Priority Accumulation Rx queue \n");
        return -1;
    }
    *rxQInfo = Qmss_getQueueNumber (gRxQHnd);


    return (0);

}

/** ============================================================================
 *   @n@b SendPacket
 *
 *   @b Description
 *   @n This API is called to actually send out data onto wire using ethernet.
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
int32_t SendPacket (void)
{
    Cppi_Desc*      pCppiDesc;
    uint32_t        dataBufferSize;
   // char            psFlags = (cpswSimTest)?pa_EMAC_PORT_0:pa_EMAC_PORT_1;
    uint8_t        *pCppiMemTX = 0;	/* Buffers to be used for TX */
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
    pCppiMemTX = (void *)fw_memAlloc(dataBufferSize,CACHE_LINESZ);
    if(pCppiMemTX == NULL)
        return -1;
    /* Populate the Tx free descriptor with the buffer. */
    memcpy(pCppiMemTX,pktMatch,dataBufferSize);
    Cppi_setData (Cppi_DescType_HOST,
	              pCppiDesc,
				  pCppiMemTX,
				  dataBufferSize);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, dataBufferSize);

    Qmss_queuePush (gPaTxQHnd[nssGblCfgParams.layout.qPaInputIndex], pCppiDesc, dataBufferSize, SIZE_HOST_DESC, Qmss_Location_TAIL);

    /* Increment the application transmit counter */
    gTxCounter ++;

    /* Give some time for the PA to process the packet */
    CycleDelay (10000);

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
int32_t VerifyPacket (Cppi_Desc* pCppiDesc)
{
	Cppi_HostDesc               *pHostDesc;
	uint8_t                       *pDataBuffer;
	uint32_t                    i;
	uint32_t		      	        infoLen;
	pasahoLongInfo_t 	        *pinfo;
  //uint8_t                       portNum;

    pHostDesc = (Cppi_HostDesc *)pCppiDesc;

    /* Verify the application software info we received is same
     * as what we had sent earlier.
     */
    if (pHostDesc->softwareInfo0 != 0xaaaaaaaa)
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
    gRxCounter ++;

    /* Reset the buffer lenght and put the descriptor back on the free queue */
    pHostDesc->buffLen = pHostDesc->origBufferLen;
    Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

    /* Verify packet done. Return success. */
	return 0;
}

/** ============================================================================
 *   @n@b ReceivePacket
 *
 *   @b Description
 *   @n This API is called to Receive packets.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t ReceivePacket (void)
{
	Cppi_Desc           *hd;
	int32_t             j;
    int32_t             status=0;

	/* Wait for a data packet from PA */
    for (j = 0; j < 100; j++)
    {
      CycleDelay (1000);
      if (Qmss_getQueueEntryCount (gRxQHnd) > 0)
      {
        hd = (Cppi_Desc *)(((uint32_t)Qmss_queuePop (gRxQHnd)) & ~0xf);
        if(VerifyPacket(hd) != 0)
            status=-1;
      }
    }

    return (status);
}

uint8_t * DataBufAlloc(void)
{
    uint8_t* pDataBuffer = NULL;
#ifdef EXT_DEBUG
	uint32_t  cl = CACHE_LINESZ;
#endif
    pDataBuffer = (uint8_t *)fw_memAlloc(PA_EMAC_EX_RXBUF_SIZE,CACHE_LINESZ);
#ifdef EXT_DEBUG
	printf ("CACHE_LINESZ:%d\n", cl);
#endif
    return (pDataBuffer);
}

void DataBufFree(void* pDataBuffer, uint32_t size)
{
   /* Since all the buffers are allocated using mmap, no need to free up since the process would clean up
    * after the process is completed */
	return;
}


void APP_exit (int32_t code)
{

}

/* Nothing past this point */
