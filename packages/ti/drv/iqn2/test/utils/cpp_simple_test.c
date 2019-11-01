/****************************************************************************\
 *           (C) Copyright 2013, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <constants.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_chip.h>

#include <ti/drv/dfe/dfe_drv.h>

#include <ti/drv/iqn2/test/utils/mnavUtils.h>
#include <ti/drv/iqn2/test/utils/cslUtils.h>


//#include "IQN2_config.h"
//#include <ti/drv/iqn2/test/utils/dfe_inc.h>
#include "tpr_17113.h"

#define LINK_LEN        2
#if LINK_LEN > NUM_PKTS_PER_CHL
#error "LINK_LEN greater than NUM_PKTS_PER_CHL!"
#endif

#define UL_CPP_64BITS	0

#define UL_CTL_CHL            0
#define PROG_DL_CTL_CHL       0
#define EMBED_DL_CTL_CHL      1
#define MANUAL_DL_CTL_CHL     2

extern Uint32 *g_useHostDescrip;


DfeFl_CppDmaObj    g_cppDmaObj[DFE_FL_CPP_NUM_DMA];
DfeFl_CppDescriptorObj g_cppDescripObj[DFE_FL_CPP_NUM_DESCRIPTORS];

DfeFl_CppDmaHandle hDmaDl0 = NULL, hDmaDl1 = NULL, hDma2 = NULL;
DfeFl_CppDmaHandle hDmaUl0 = NULL;
DfeFl_CppDescriptorHandle  hDescripDl[NUM_PKTS_PER_CHL], hDescripUl[NUM_PKTS_PER_CHL];
DfeFl_CppDescripConfig cfgDescripDl, cfgDescripUl;


DfeFl_CppResMgr g_cppResMgr;

void cppInit()
{
	int i;

	// none resource reserved
	memset(&g_cppResMgr, 0, sizeof(g_cppResMgr));

	for(i = 0; i < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; i++)
	{
		g_cppResMgr.discreteTrig[i] = DFE_FL_CPP_OPEN_ANY;
	}
}

//
// simple loopback config
//  write DPD LUT b0r0c0 with pattern using route DDR3 => CTL Dl0 => CPP DMA0
//  read DPD LUT b0r0c0 using CPP DMA16 => CTL => Ul0 => DDR3
//  then do comparation
int testOpenProgMode(DFE_Handle  hDfe)
{
    int i;
    DfeFl_Status status = DFE_FL_SOK;
    
    // open CPP DMA for DL, PROG mode
    hDmaDl0 = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        0, // dmaId
        DFE_FL_CPP_DMA_MODE_PROG, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &g_cppResMgr, // resMgr
        &status);
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DMA);
        return FAIL;
    }
    
    for(i = 0; i < LINK_LEN; i++)
    {
        // alloc CPP descriptors for Dl
        hDescripDl[i] = dfeFl_CppDecripOpen(
            hDfe->hDfeMisc[0],
            DFE_FL_CPP_OPEN_ANY, // descriptor id
            &g_cppResMgr, // resMgr
            &status);
        if(status != DFE_FL_SOK)
        {
//            SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DECSRIP);
            return FAIL;
        }
            
        // config Dl descriptor
        //  
        cfgDescripDl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfe->hDfeDpd[0]->regs->dpd0_dpdlut_b0_r0_c0) + 0x800 * i;
//        cfgDescripDl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfeCb[0]->regs->capture_buffer_a_16msb) + 0x800 * i;
        cfgDescripDl.chanNum = PROG_DL_CTL_CHL; // CTL Dl ch#
        cfgDescripDl.rw = DFE_FL_CPP_DMA_DL;
        cfgDescripDl.numBytes = 1024;
        cfgDescripDl.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
        cfgDescripDl.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
        cfgDescripDl.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
        cfgDescripDl.midImm = 0;
        cfgDescripUl.linkNext = 0; // change later
        dfeFl_CppDescripWrite(hDescripDl[i], &cfgDescripDl);        
    }    
    
    // build linklist 
    for(i = 0; i < LINK_LEN - 1; i++)
    {
        dfeFl_CppDescripLink(hDescripDl[i], hDescripDl[i+1]);
    }    
    // end of linklist
    dfeFl_CppDescripLink(hDescripDl[LINK_LEN - 1], hDescripDl[LINK_LEN - 1]);
        
    return PASS;    
}

int testOpenUl(DFE_Handle  hDfe)
{
    int i;
    DfeFl_Status status = DFE_FL_SOK;
    
    // open CPP DMA for Ul
    hDmaUl0 = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        16, // dmaId
        DFE_FL_CPP_DMA_MODE_PROG, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &g_cppResMgr, // resMgr
        &status);
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DMA);
        return FAIL;
    }
    
    for(i = 0; i < LINK_LEN; i++)
    {
        // alloc CPP descriptor for Ul
        hDescripUl[i] = dfeFl_CppDecripOpen(
            hDfe->hDfeMisc[0],
            DFE_FL_CPP_OPEN_ANY, // descriptor id
            &g_cppResMgr, // resMgr
            &status);
        if(status != DFE_FL_SOK)
        {
//            SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DECSRIP);
            return FAIL;
        }
        
        // config Ul descriptor
        //
#if UL_CPP_64BITS > 0
        cfgDescripUl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfe->hDfeDpd[0]->regs->dpd0_dpdlut_b0_r0_c0) + 0x800 * i + 4;
//        cfgDescripUl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfeCb[0]->regs->capture_buffer_a_16msb) + 0x800 * i + 4;
        cfgDescripUl.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_8;
        cfgDescripUl.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_8;
#else
        cfgDescripUl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfe->hDfeDpd[0]->regs->dpd0_dpdlut_b0_r0_c0) + 0x800 * i;
//        cfgDescripUl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfeCb[0]->regs->capture_buffer_a_16msb) + 0x800 * i;
        cfgDescripUl.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
        cfgDescripUl.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
#endif
        cfgDescripUl.chanNum = UL_CTL_CHL; // CTL Ul ch#
        cfgDescripUl.rw = DFE_FL_CPP_DMA_UL;
        cfgDescripUl.numBytes = 1024;
        cfgDescripUl.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
        cfgDescripUl.midImm = 0;
        cfgDescripUl.linkNext = 0; // change later
        dfeFl_CppDescripWrite(hDescripUl[i], &cfgDescripUl);
    }    
    
    // build linklist 
    for(i = 0; i < LINK_LEN - 1; i++)
    {
        dfeFl_CppDescripLink(hDescripUl[i], hDescripUl[i+1]);
    }    
    // end of linklist
    dfeFl_CppDescripLink(hDescripUl[LINK_LEN - 1], hDescripUl[LINK_LEN - 1]);
        
    return PASS;    
    
}

void testCloseUl()
{
    Uint32 i;
    
    dfeFl_CppDmaClose(hDmaUl0);
    
    for(i = 0; i < LINK_LEN; i++)
    {
        dfeFl_CppDescripClose(hDescripUl[i]);
    }
}

// close DMAs and Descriptors
void testCloseProgMode()
{
    Uint32 i;
    
    dfeFl_CppDmaClose(hDmaDl0);
    
    for(i = 0; i < LINK_LEN; i++)
    {
        dfeFl_CppDescripClose(hDescripDl[i]);
    }
}

int testOpenEmbedMode(DFE_Handle  hDfe)
{
    DfeFl_Status status = DFE_FL_SOK;
    
    // open CPP DMA for DL, EMBED mode
    hDmaDl1 = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        DFE_FL_CPP_OPEN_ANY, // dmaId
        DFE_FL_CPP_DMA_MODE_EMBED, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &g_cppResMgr, // resMgr
        &status);
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DMA);
        return FAIL;
    }

    return PASS;
}

void testCloseEmbedMode()
{
    dfeFl_CppDmaClose(hDmaDl1);
}

int testOpenManualMode(DFE_Handle  hDfe)
{
    int i;
    DfeFl_Status status = DFE_FL_SOK;
    
    // open CPP DMA for DL, EMBED mode
    hDma2 = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        DFE_FL_CPP_OPEN_ANY, // dmaId
        DFE_FL_CPP_DMA_MODE_PROG, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &g_cppResMgr, // resMgr
        &status);
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DMA);
        return FAIL;
    }    

    for(i = 0; i < LINK_LEN; i++)
    {
        // alloc CPP descriptors for Dl
        hDescripDl[i] = dfeFl_CppDecripOpen(
            hDfe->hDfeMisc[0],
            DFE_FL_CPP_OPEN_ANY, // descriptor id
            &g_cppResMgr, // resMgr
            &status);
        if(status != DFE_FL_SOK)
        {
//            SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CPP_DECSRIP);
            return FAIL;
        }
            
        // config Dl descriptor
        //  
        cfgDescripDl.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfe->hDfeDpd[0]->regs->dpd0_dpdlut_b0_r0_c0) + 0x800 * i;
        cfgDescripDl.chanNum = MANUAL_DL_CTL_CHL; // CTL Dl ch#
        cfgDescripDl.rw = DFE_FL_CPP_DMA_DL;
        cfgDescripDl.numBytes = 1024;
        cfgDescripDl.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
        cfgDescripDl.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
        cfgDescripDl.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
        cfgDescripDl.midImm = 0;
        cfgDescripUl.linkNext = 0; // change later
        dfeFl_CppDescripWrite(hDescripDl[i], &cfgDescripDl);        
    }    
    
    // build linklist 
    for(i = 0; i < LINK_LEN - 1; i++)
    {
        dfeFl_CppDescripLink(hDescripDl[i], hDescripDl[i+1]);
    }    
    // end of linklist
    dfeFl_CppDescripLink(hDescripDl[LINK_LEN - 1], hDescripDl[LINK_LEN - 1]);
    
    return PASS;
}

void testCloseManualMode()
{
    Uint32 i;
    
    dfeFl_CppDmaClose(hDma2);
    
    for(i = 0; i < LINK_LEN; i++)
    {
        dfeFl_CppDescripClose(hDescripDl[i]);
    }
}

// start DL DMA
int testStartDlProgMode()
{
    DfeFl_Status status = DFE_FL_SOK;
    status = dfeFl_CppDmaArm(hDmaDl0, dfeFl_CppDescripGetId(hDescripDl[0]), DFE_FL_CPP_DMA_SSEL_DL_CTL_DATA_AVAIL(PROG_DL_CTL_CHL));
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_ARM_CPP_DMA);
        return FAIL;        
    }
    
    return PASS;
}

int testStartDlEmbedMode()
{
    DfeFl_Status status = DFE_FL_SOK;
    status = dfeFl_CppDmaArm(hDmaDl1, EMBED_DL_CTL_CHL, DFE_FL_CPP_DMA_SSEL_DL_CTL_DATA_AVAIL(EMBED_DL_CTL_CHL));
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_ARM_CPP_DMA);
        return FAIL;        
    }
    
    return PASS;
}

int testStartDlManualMode()
{
    DfeFl_Status status = DFE_FL_SOK;
    status = dfeFl_CppDmaArm(hDma2, dfeFl_CppDescripGetId(hDescripDl[0]), DFE_FL_CPP_DMA_SSEL_MANUAL);
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_ARM_CPP_DMA);
        return FAIL;        
    }
    
    return PASS;
}

// start UL DMA
int testStartUl(DFE_Handle  hDfe)
{
    DfeFl_Status status = DFE_FL_SOK;
    status = dfeFl_CppDmaArm(hDmaUl0, dfeFl_CppDescripGetId(hDescripUl[0]), DFE_FL_CPP_DMA_SSEL_GSYNC(DFE_FL_SYNC_GEN_SIG_MPU_SYNC));
    if(status != DFE_FL_SOK)
    {
//        SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_ARM_CPP_DMA);
        return FAIL;        
    }
    Dfe_issueSync(hDfe,DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_WAITFOREVER);
//    issueSync(DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_WAITFOREVER);
    
    return PASS;
}

// the simple loogpback test
int testSimpleProgMode(DFE_Handle  hDfe, PktDmaConfigHandle hPktDma)
{
    int rc = PASS;
    Uint32 pkt, idx;
	volatile Uint32  queCount;
	Cppi_MonolithicDesc *mono_pkt;
	Cppi_HostDesc *host_pkt;
    Qmss_Queue           descQueue;
    Cppi_DescTag         descTag;
    uint32_t             *payloadPtr;
    uint32_t			 payloadLen;
    uint32_t 			 *txBuffer;
    uint32_t 			 *rxBuffer;
    Qmss_Queue           queueInfo;

    rc = testOpenProgMode(hDfe);
    if(rc == PASS)
    {
        // start Cpp DL
        testStartDlProgMode();

        // ============================= PROG mode ========================
        // push packets into Tx queue
        for(pkt = 0; pkt < LINK_LEN; pkt++)
        {
        	if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
        	{
				mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[PROG_DL_CTL_CHL]));

				Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
				Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
				Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,12);
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,1024);
				Cppi_setPSFlags (Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
				Cppi_setPSLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);

				queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[PROG_DL_CTL_CHL]);
				descQueue.qMgr = queueInfo.qMgr;
				descQueue.qNum = queueInfo.qNum;
				Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);

				descTag.destTagHi = 0;                  descTag.destTagLo = 0;
				descTag.srcTagHi  = 0;                  descTag.srcTagLo  = 0;
				Cppi_setTag(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,&descTag);

				// Get payload address
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++) payloadPtr[idx] = ((pkt+1) << 16) + idx;

				UTILS_cacheWriteBackInvalidate((void*)mono_pkt, 1024 + 12);
				Qmss_queuePushDesc((hPktDma->txQCtrl[PROG_DL_CTL_CHL]), (uint32_t*)mono_pkt);
        	} else {
        		txBuffer = hPktDma->txDataBuff[PROG_DL_CTL_CHL];
        		host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[PROG_DL_CTL_CHL]));
        		UTILS_cacheInvalidate((void *)host_pkt, 64);
//        		Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)host_pkt,(Uint8*) &txBuffer[pkt*hPktDma->txDescSizeCtrl[PROG_DL_CTL_CHL]/4], 1024);
//        		Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*) host_pkt, (Uint8 *) (Uint8*) &txBuffer[pkt*hPktDma->txDescSizeCtrl[PROG_DL_CTL_CHL]/4], 1024);
//        		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*) host_pkt, 1024);
//
//				descTag.destTagHi = 0;                  descTag.destTagLo = 0;
//				descTag.srcTagHi  = 0;                  descTag.srcTagLo  = 0;
//				Cppi_setTag(Cppi_DescType_HOST,(Cppi_Desc *)host_pkt,&descTag);

        		Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t**)&payloadPtr, &payloadLen);
        		for (idx = 0; idx < 1024 / 4; idx++) payloadPtr[idx] = ((pkt+1) << 16) + idx;
        		UTILS_cacheWriteBackInvalidate((void*)host_pkt, 64);
//        		Qmss_queuePushDesc((hPktDma->txQCtrl[PROG_DL_CTL_CHL]), (uint32_t*)host_pkt);
        		Qmss_queuePushDescSize((hPktDma->txQCtrl[PROG_DL_CTL_CHL]), (uint32_t*)host_pkt, 64);
        	}
        }
        
        // wait till data arrives DPD cells
        UTILS_waitForHw(10000);
        do
        {
            // Get current descriptor count for monolithic RX queue
            //queCount = qm_get_descriptor_count(CTL_TX_Q + PROG_DL_CTL_CHL);
            queCount = Qmss_getQueueEntryCount(hPktDma->txQCtrl[PROG_DL_CTL_CHL]);
        } while (queCount > 0);
    
        // start Cpp UL
        testStartUl(hDfe);
        UTILS_waitForHw(10000);
        
        // wait till data arrives Rx queue    
        do
        {
            // Get current descriptor count for monolithic RX queue
            //queCount = qm_get_descriptor_count(CTL_RX_Q + UL_CTL_CHL);
            queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[UL_CTL_CHL]);
        }while (queCount < LINK_LEN);

       /*****************************************************************
        * Compare the data in the destination buffers.
        */
    
        /* Compare the Monolithic packet data */
        //queCount = qm_get_descriptor_count((CTL_RX_Q + UL_CTL_CHL));
        queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[UL_CTL_CHL]);
    
        for (pkt = 0; pkt < queCount; pkt ++)
        {
            //tmp = qm_pop_queue((CTL_RX_Q + UL_CTL_CHL));
            //tmp &= 0xFFFFFFF0;// clean DESC_SIZE field
        	if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
        	{
				mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[UL_CTL_CHL]));
				if (mono_pkt == NULL)
					printf("Rx Ctrl descriptor NULL: descriptor %d, chan %d\n", pkt, UL_CTL_CHL);
				UTILS_cacheInvalidate((void*)mono_pkt, 1024 + 12);

				// Get payload address
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++)
				{
					if(payloadPtr[idx] != ((pkt+1) << 16) + idx)
					{
						//SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_VERIFY1);
						rc = FAIL;
						break;
					}
				}

				//qm_push_queue(CTL_RX_FDQ, 1, 0, tmp);
				Qmss_queuePushDesc((hPktDma->rxFqCtrl[UL_CTL_CHL]), (uint32_t*)mono_pkt);

        	} else {
        		host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[UL_CTL_CHL]));

				if (host_pkt == NULL)
					printf("Rx Ctrl descriptor NULL: descriptor %d, chan %d\n", pkt, UL_CTL_CHL);

				UTILS_cacheInvalidate((void*)host_pkt, 64);

				// Get payload address
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++)
				{
					if(payloadPtr[idx] != ((pkt+1) << 16) + idx)
					{
						//SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_VERIFY1);
						rc = FAIL;
						break;
					}
				}
				Qmss_queuePushDescSize((hPktDma->rxFqCtrl[UL_CTL_CHL]), (uint32_t*)host_pkt, 64);
        	}
        }
    
        testCloseProgMode();
    }
    
    printf(" Simple Loopback (PROG mode) test: %s\n", (rc==PASS) ? "PASS" : "FAIL");
        
    return rc;
}
        
int testSimpleEmbedMode(DFE_Handle  hDfe, PktDmaConfigHandle hPktDma)
{
    int rc = PASS;
    Uint32 pkt, idx; //, tmp, *temp;
    volatile Uint32  queCount;
    Cppi_MonolithicDesc *mono_pkt;
    Cppi_HostDesc *host_pkt;
    Qmss_Queue           descQueue;
    Cppi_DescTag         descTag;
    uint32_t            *payloadPtr;
    uint32_t            payloadLen;
    Qmss_Queue           queueInfo;
    DfeFl_CppEmbedHeader *embedHdr;
    DfeFl_CppDescripConfig *descripCfg;
    
    rc = testOpenEmbedMode(hDfe);
    if(rc == PASS)
    {
        // start Cpp DL
        testStartDlEmbedMode();

        // ============================= EMBED mode ========================
        // push packets into Tx queue
        for(pkt = 0; pkt < LINK_LEN; pkt++)
        {
        	if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
        	{
				mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[EMBED_DL_CTL_CHL]));

				Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
				Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
				Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,12);
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,1024 + sizeof(DfeFl_CppEmbedHeader));
				Cppi_setPSFlags (Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
				Cppi_setPSLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);

				queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[EMBED_DL_CTL_CHL]);
				descQueue.qMgr = queueInfo.qMgr;
				descQueue.qNum = queueInfo.qNum;
				Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);

				descTag.destTagHi = 0;                  descTag.destTagLo = 0;
				descTag.srcTagHi  = 0;                  descTag.srcTagLo  = 0;
				Cppi_setTag(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,&descTag);

				// Get payload address
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

        	} else {
        		host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[EMBED_DL_CTL_CHL]));
        		UTILS_cacheInvalidate((void *)host_pkt, 64);
        		Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*) host_pkt, (Uint8 *) (Uint8*) hPktDma->txDataBuff[EMBED_DL_CTL_CHL], hPktDma->txDescSizeCtrl[EMBED_DL_CTL_CHL]);
        		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*) host_pkt, hPktDma->txDescSizeCtrl[EMBED_DL_CTL_CHL]);

        		Cppi_getOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc*)host_pkt,(Uint8**) payloadPtr, &payloadLen);

        	}
            //tmp = qm_pop_queue(CTL_TX_COMPLETE_Q);
            //tmp &= 0xFFFFFFF0;//set DESC_SIZE field to zero

/*            if(*g_useHostDescrip > 0)
            {
            	host_pkt = (MNAV_HostPacketDescriptor *)tmp;

            	host_pkt->buffer_ptr = host_pkt->orig_buff0_ptr;
            	host_pkt->buffer_len = 1024 + sizeof(DfeFl_CppEmbedHeader);
            	host_pkt->packet_length = 1024 + sizeof(DfeFl_CppEmbedHeader);

            	temp = (Uint32 *)host_pkt->orig_buff0_ptr;
            }
            else
            {
				mono_pkt = (MNAV_MonolithicPacketDescriptor *)tmp;

				//Create Mono packet (initialize non-zero fields)
				mono_pkt->type_id = MNAV_DESC_TYPE_MONO;
				mono_pkt->data_offset = MNAV_MONO_PACKET_SIZE;
				mono_pkt->packet_length = 1024 + sizeof(DfeFl_CppEmbedHeader);

				mono_pkt->ps_flags = 0;
				mono_pkt->epib = 0;
				mono_pkt->psv_word_count = 0; // 4 byte PS field length
				mono_pkt->pkt_return_qnum = CTL_TX_COMPLETE_Q;
				mono_pkt->src_tag_lo = 0; //copied to .flo_idx of streaming i/f

				temp = (Uint32 *)(tmp + MNAV_MONO_PACKET_SIZE);
            }
*/
            embedHdr = (DfeFl_CppEmbedHeader *)payloadPtr;
            descripCfg = &embedHdr->descripCfg;
            descripCfg->mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((Uint32)&hDfe->hDfeDpd[0]->regs->dpd0_dpdlut_b0_r0_c0) + 0x800 * pkt;
            descripCfg->chanNum = EMBED_DL_CTL_CHL; // CTL Dl ch#
            descripCfg->rw = DFE_FL_CPP_DMA_DL;
            descripCfg->numBytes = 1024;
            descripCfg->ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
            descripCfg->mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
            descripCfg->pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
            descripCfg->midImm = 0;
            descripCfg->linkNext = 0; // not used in embedded mode
            // first two words of data
            embedHdr->data0 = 0; //((pkt+1) << 17) + 0;
            embedHdr->data1 = 0; //((pkt+1) << 17) + 1;
            // remainning words od data
            payloadPtr += sizeof(DfeFl_CppEmbedHeader) / 4;
            for (idx = 0; idx < 1024 / 4; idx++)
            {
                payloadPtr[idx] = ((pkt+1) << 17) + idx;
            }

            //qm_push_queue((CTL_TX_Q + EMBED_DL_CTL_CHL), 1, 0, tmp+3);


            if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
            {
            	UTILS_cacheWriteBackInvalidate((void*)mono_pkt, 1024 + 12 + + sizeof(DfeFl_CppEmbedHeader));
            	Qmss_queuePushDesc((hPktDma->txQCtrl[EMBED_DL_CTL_CHL]), (uint32_t*)mono_pkt);
            } else {
            	UTILS_cacheWriteBackInvalidate((void*)host_pkt, 64);
            	Qmss_queuePushDescSize((hPktDma->txQCtrl[EMBED_DL_CTL_CHL]), (uint32_t*)host_pkt, 64);
            }
        }
        
        // wait till data arrives DPD cells
        UTILS_waitForHw(10000);
        do
        {
            // Get current descriptor count for monolithic RX queue
            //queCount = qm_get_descriptor_count(CTL_TX_Q + EMBED_DL_CTL_CHL);
            queCount = Qmss_getQueueEntryCount(hPktDma->txQCtrl[EMBED_DL_CTL_CHL]);
        } while (queCount > 0);
    
        // start Cpp UL
        testStartUl(hDfe);
        UTILS_waitForHw(10000);
        
        // wait till data arrives Rx queue    
        do
        {
            // Get current descriptor count for monolithic RX queue
            //queCount = qm_get_descriptor_count(CTL_RX_Q + UL_CTL_CHL);
            queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[UL_CTL_CHL]);
        }while (queCount < LINK_LEN);
    
       /*****************************************************************
        * Compare the data in the destination buffers.
        */
    
        /* Compare the Monolithic packet data */
        //queCount = qm_get_descriptor_count((CTL_RX_Q + UL_CTL_CHL));
        queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[UL_CTL_CHL]);
    
        for (pkt = 0; pkt < queCount; pkt ++)
        {
        	if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
        	{
				//tmp = qm_pop_queue((CTL_RX_Q + UL_CTL_CHL));
				//tmp &= 0xFFFFFFF0;// clean DESC_SIZE field
				mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[UL_CTL_CHL]));
				if (mono_pkt == NULL)
					printf("Rx Ctrl descriptor NULL: descriptor %d, chan %d\n", pkt, UL_CTL_CHL);
				UTILS_cacheInvalidate((void*)mono_pkt, 1024 + 12);

				// Get payload address
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++)
				{
					if(payloadPtr[idx] != ((pkt+1) << 17) + idx)
					{
						//SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_VERIFY2);
						rc = FAIL;
						break;
					}
				}

				//qm_push_queue(CTL_RX_FDQ, 1, 0, tmp);
				Qmss_queuePushDesc((hPktDma->rxFqCtrl[UL_CTL_CHL]), (uint32_t*)mono_pkt);
        	} else {
        		host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[UL_CTL_CHL]));
        		if (host_pkt == NULL)
					printf("Rx Ctrl descriptor NULL: descriptor %d, chan %d\n", pkt, UL_CTL_CHL);

        		UTILS_cacheInvalidate((void*)host_pkt, 64);

				// Get payload address
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++)
				{
					if(payloadPtr[idx] != ((pkt+1) << 17) + idx)
					{
						//SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_VERIFY2);
						rc = FAIL;
						break;
					}
				}

				//qm_push_queue(CTL_RX_FDQ, 1, 0, tmp);
				Qmss_queuePushDescSize((hPktDma->rxFqCtrl[UL_CTL_CHL]), (uint32_t*)host_pkt, 64);

        	}
        }
        
        testCloseEmbedMode();
    }
    
    printf(" Simple Loopback (EMBED mode) test: %s\n", (rc==PASS) ? "PASS" : "FAIL");
        
    return rc;                
}

int testSimpleManualMode(DFE_Handle  hDfe, PktDmaConfigHandle hPktDma)
{
    int rc = PASS;
    Uint32 pkt, idx; //, tmp, *temp;
	volatile Uint32  queCount;
    Cppi_MonolithicDesc *mono_pkt;
    Cppi_HostDesc *host_pkt;
    Qmss_Queue           descQueue;
    Cppi_DescTag         descTag;
    uint32_t            *payloadPtr;
    uint32_t            payloadLen;
    Qmss_Queue           queueInfo;
    
    rc = testOpenManualMode(hDfe);
    if(rc == PASS)
    {
        // start Cpp DL
        testStartDlManualMode();

        // ============================= PROG mode ========================
        // push packets into Tx queue
        for(pkt = 0; pkt < LINK_LEN; pkt++)
        {
        	if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
        	{
				mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[MANUAL_DL_CTL_CHL]));

				Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
				Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
				Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,12);
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,1024);
				Cppi_setPSFlags (Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
				Cppi_setPSLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);

				queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[MANUAL_DL_CTL_CHL]);
				descQueue.qMgr = queueInfo.qMgr;
				descQueue.qNum = queueInfo.qNum;
				Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);

				descTag.destTagHi = 0;                  descTag.destTagLo = 0;
				descTag.srcTagHi  = 0;                  descTag.srcTagLo  = 0;
				Cppi_setTag(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,&descTag);

				// Get payload address
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++) payloadPtr[idx] = ((pkt+1) << 18) + idx;

				UTILS_cacheWriteBackInvalidate((void*)mono_pkt, 1024 + 12);
				Qmss_queuePushDesc((hPktDma->txQCtrl[MANUAL_DL_CTL_CHL]), (uint32_t*)mono_pkt);
        	} else {
        		host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[MANUAL_DL_CTL_CHL]));
        		UTILS_cacheInvalidate((void *)host_pkt, 64);
//        		Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*) host_pkt, (Uint8 *) (Uint8*) hPktDma->txDataBuff[MANUAL_DL_CTL_CHL], 1024);
//        		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*) host_pkt, 1024);

        		Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++) payloadPtr[idx] = ((pkt+1) << 18) + idx;

				UTILS_cacheWriteBackInvalidate((void*)host_pkt, 64);
				Qmss_queuePushDescSize((hPktDma->txQCtrl[MANUAL_DL_CTL_CHL]), (uint32_t*)host_pkt, 64);
        	}
        }
        
        // wait till data arrives DPD cells
        UTILS_waitForHw(10000);
        do
        {
            // Get current descriptor count for monolithic RX queue
            //queCount = qm_get_descriptor_count(CTL_TX_Q + MANUAL_DL_CTL_CHL);
            queCount = Qmss_getQueueEntryCount(hPktDma->txQCtrl[MANUAL_DL_CTL_CHL]);
        } while (queCount > 0);
    
        // start Cpp UL
        testStartUl(hDfe);
        UTILS_waitForHw(10000);
        
        // wait till data arrives Rx queue    
        do
        {
            // Get current descriptor count for monolithic RX queue
            //queCount = qm_get_descriptor_count(CTL_RX_Q + UL_CTL_CHL);
            queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[UL_CTL_CHL]);
        }while (queCount < LINK_LEN);
    
       /*****************************************************************
        * Compare the data in the destination buffers.
        */
    
        /* Compare the Monolithic packet data */
        //queCount = qm_get_descriptor_count((CTL_RX_Q + UL_CTL_CHL));
        queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[UL_CTL_CHL]);
    
        for (pkt = 0; pkt < queCount; pkt ++)
        {
        	if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
        	{
				/*tmp = qm_pop_queue((CTL_RX_Q + UL_CTL_CHL));
				tmp &= 0xFFFFFFF0;// clean DESC_SIZE field*/
				mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[UL_CTL_CHL]));
				if (mono_pkt == NULL)
					printf("Rx Ctrl descriptor NULL: descriptor %d, chan %d\n", pkt, UL_CTL_CHL);
				UTILS_cacheInvalidate((void*)mono_pkt, 1024 + 12);

				// Get payload address
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++)
				{
					if(payloadPtr[idx] != ((pkt+1) << 18) + idx)
					{
						//SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_VERIFY2);
						rc = FAIL;
						break;
					}
				}

				//qm_push_queue(CTL_RX_FDQ, 1, 0, tmp);
				Qmss_queuePushDesc((hPktDma->rxFqCtrl[UL_CTL_CHL]), (uint32_t*)mono_pkt);
        	} else {
        		host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[UL_CTL_CHL]));
				if (host_pkt == NULL)
					printf("Rx Ctrl descriptor NULL: descriptor %d, chan %d\n", pkt, UL_CTL_CHL);
				UTILS_cacheInvalidate((void*)host_pkt, 64);

				// Get payload address
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t**)&payloadPtr, &payloadLen);

				for (idx = 0; idx < 1024 / 4; idx++)
				{
					if(payloadPtr[idx] != ((pkt+1) << 18) + idx)
					{
						//SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_VERIFY2);
						rc = FAIL;
						break;
					}
				}

				//qm_push_queue(CTL_RX_FDQ, 1, 0, tmp);
				Qmss_queuePushDescSize((hPktDma->rxFqCtrl[UL_CTL_CHL]), (uint32_t*)host_pkt, 64);
        	}
        }
    
        testCloseManualMode();
    }
    
    printf(" Simple Loopback (MANUAL mode) test: %s\n", (rc==PASS) ? "PASS" : "FAIL");
        
    return rc;
}

int testDfeCtlSimple(DFE_Handle  hDfe, PktDmaConfigHandle hPktDma)
{
    int data, rc = PASS;
    //DfeFl_Status status = DFE_FL_SOK;
    
    // enable MPU access to DPD LUTs
    data = DFE_FL_MEM_MPU_ACCESS_DPD;
    dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_MEM_MPU_ACCESS, &data);
    // enable MPU access to CB Buffers
    data = DFE_FL_MEM_MPU_ACCESS_CB;
    dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_MEM_MPU_ACCESS, &data);

    // open uplink
    if(rc == PASS)
        rc = testOpenUl(hDfe);
    else
        printf("testOpenUl fail \n");

    // prog mode loopback
    if(rc == PASS)
        rc = testSimpleProgMode(hDfe, hPktDma);
    else
        printf("testSimpleProgMode fail \n");
            
    // embed mode loopback
    //if(rc == PASS) rc = testSimpleEmbedMode(hPktDma);
//    rc = PASS;

    // manual mode loopback
    if(rc == PASS)
        rc = testSimpleManualMode(hDfe, hPktDma);
    else
        printf("testSimpleManualMode fail \n");

    return rc;
}
