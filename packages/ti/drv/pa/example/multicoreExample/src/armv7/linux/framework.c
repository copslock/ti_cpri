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
#include "multicore_example.h"
#include "ti/drv/pa/pa.h"
#include "fw_test.h"
#include "ti/drv/pa/pasahost.h"
extern Qmss_QueueHnd gRxQHnd;

/* Socket Includes */
#include "sockutils.h"
#include <ti/drv/rm/rm_server_if.h>

#include <errno.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <sys/ipc.h>

/* RM Includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

/* Semaphore Includes */
#include <semaphore.h>

/* Socket timeout */
#define CLIENT_SOCK_TIMEOUT_USEC     (500)

/* Application's registered RM transport indices */
#define SERVER_TO_CLIENT_MAP_ENTRY   0
/* Maximum number of registered RM transports */
#define MAX_MAPPING_ENTRIES          1

/* Error checking macro */
#define RM_ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)                 \
    if (resultVal != checkVal) {                                                  \
        char errorMsgToPrint[] = printMsg;                                        \
        printf("RM Inst : %s : ", rmInstName);                                    \
        printf("%s with error code : %d, exiting\n", errorMsgToPrint, resultVal); \
        return(-1);                                                               \
    }

/* RM registered transport mapping structure */
typedef struct trans_map_entry_s {
    /* Registered RM transport handle */
    Rm_TransportHandle        transportHandle;
    /* Remote socket tied to the transport handle */
    sock_name_t              *remote_sock;
} Transport_MapEntry;


/* RM initialization sync point */
#pragma DATA_SECTION (isRmInitialized, ".rm");
/* RM Client Vars */
extern Rm_Handle           rmClientHandle;
extern Rm_ServiceHandle   *rmClientServiceHandle;
volatile uint32_t           isRmInitialized;

/* Linux Specific global variables per process */
sock_h                      rmClientSocket;
sem_t                       mutex;


/* Client socket name */
#define MAX_CLIENT_SOCK_NAME 32
char                rmClientSockName[MAX_CLIENT_SOCK_NAME];

/* Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmClientName[RM_NAME_MAX_CHARS] = "RM_Client0";

/* Client socket name */
char                rmClientSockName[] = "/tmp/var/run/rm/rm_client";

/* Transport map stores the RM transport handle to IPC MessageQ queue mapping */
Transport_MapEntry  rmTransportMap[MAX_MAPPING_ENTRIES];

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

void APP_exit (int32_t code)
{
}

void APP_publishGlobalCfgDone(void)
{
	/* Store all global handles in the shared memory and Publish the config is done */
	fw_shmSetEntry();

}

void APP_waitGlobalCfgDone(void)
{
	uint32_t globalCfgDone;
	pa_Example_shmStr_e shm;

	shm = globalCfgDoneAddr;
    do {
    	globalCfgDone = (uint32_t) fw_shmGetEntry(shm);
		yield();
    } while(!globalCfgDone);
}

void APP_waitAllLocalCfgDone(void)
{
	uint32_t localCfgDone;

    do {
    	localCfgDone = (uint32_t) fw_shmGetEntry(localCfgDoneAddr);
		yield();
    } while (localCfgDone != pa_MC_EXAMPLE_NUM_CORES);
}

void APP_publishLocalCfgDone(void)
{
	uint32_t* addr;

    /* wait for the semaphore */
	sem_wait(&mutex);

    addr = (uint32_t *) &shObj->localCfgDone;
	*addr += 1;
	SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

    /* Release the semaphore. */
	sem_post(&mutex);

}

void APP_waitAllLocalTestDone(void)
{
	uint32_t localTestDone;

    do {
    	localTestDone = (uint32_t) fw_shmGetEntry(localTestDoneAddr);
		yield();
    } while (localTestDone != pa_MC_EXAMPLE_NUM_CORES);
}


void APP_publishLocalTestDone(void)
{
	uint32_t* addr;

	sem_wait(&mutex);

    addr = (uint32_t *) &shObj->localTestDone;
	*addr += 1;
	SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

    /* Release the hardware semaphore. */
	sem_post(&mutex);

}

int32_t APP_checkTestStatus(void)
{
	uint32_t testRxCnt;

    testRxCnt = (uint32_t) fw_shmGetEntry(allPktRxCntAddr);

    return (testRxCnt == pa_MC_EXAMPLE_NUM_CORES);
}



void APP_publishTestStatus(void)
{
	uint32_t* addr;

    /* wait for the semaphore */
	sem_wait(&mutex);

    addr = (uint32_t *) &shObj->allPktRxCnt;
	*addr += 1;
	SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

    /* Release the semaphore. */
	sem_post(&mutex);

}


uint32_t fw_shmGetEntry(pa_Example_shmStr_e shmstr)
{
	uint32_t addr;

	switch (shmstr)
	{
    case gTxFreeQHndAddr:
        SYS_CACHE_INV ((void *) &shObj->gTxFreeQHnd, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->gTxFreeQHnd;
        break;
    case gRxFreeQHndAddr:
        SYS_CACHE_INV ((void *) &shObj->gRxFreeQHnd, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->gRxFreeQHnd;
        break;
    case gCpdmaHndAddr :
        SYS_CACHE_INV ((void *) &shObj->gCpdmaHnd, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->gCpdmaHnd;
        break;
    case 	gPAInstHndAddr:
        SYS_CACHE_INV ((void *) &shObj->gPAInstHnd, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->gPAInstHnd;
    	break;
    case gPaL2HandlesAddr:
        SYS_CACHE_INV ((void *) &shObj->gPaL2Handles, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->gPaL2Handles;
    	break;
    case gPaL3HandlesAddr:
        SYS_CACHE_INV ((void *) &shObj->gPaL3Handles, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->gPaL3Handles;
    	break;
    case globalCfgDoneAddr:
        SYS_CACHE_INV ((void *) &shObj->globalCfgDone, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->globalCfgDone;
    	break;
    case localCfgDoneAddr:
        SYS_CACHE_INV ((void *) &shObj->localCfgDone, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->localCfgDone;
    	break;
    case localTestDoneAddr:
        SYS_CACHE_INV ((void *) &shObj->localTestDone, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->localTestDone;
    	break;
    case allPktRxCntAddr:
        SYS_CACHE_INV ((void *) &shObj->allPktRxCnt, CACHE_LINESZ, CACHE_WAIT);
    	addr = (uint32_t) shObj->allPktRxCnt;
    	break;
    case gPaInstBufAddr:
       	addr = (uint32_t) shObj->gPAInst;
    	break;
    case gMemL2RamBufAddr:
       	addr = (uint32_t) shObj->gMemL2Ram;
    	break;
    case gMemL3RamBufAddr:
    	addr = (uint32_t) shObj->gMemL3Ram;
    	break;
    case gMemUsrStatsBufAddr:
    	addr = (uint32_t) shObj->gMemUsrStats;
    	break;
    case gPaInstBufSize:
    	addr = (uint32_t) sizeof (shObj->gPAInst);
    	break;
    case gMemL2RamBufSize:
    	addr = (uint32_t) sizeof (shObj->gMemL2Ram);
    	break;
    case gMemL3RamBufSize:
    	addr = (uint32_t) sizeof (shObj->gMemL3Ram);
    	break;
    case gMemUsrStatsBufSize:
    	addr = (uint32_t) sizeof (shObj->gMemUsrStats);
    	break;
    default:
    	addr = (uint32_t) NULL;
    	break;
	}

	return (addr);
}

/* Note that this function needs to be called by Master Core only */
void fw_shmSetEntry(void)
{
	    uint32_t* addr;

    	addr = (uint32_t*) &shObj->gTxFreeQHnd;
    	*addr = (uint32_t) gTxFreeQHnd;
        SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gRxFreeQHnd;
    	*addr = (uint32_t) gRxFreeQHnd;
        SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gCpdmaHnd;
    	*addr = (uint32_t) gCpdmaHnd;
        SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gPAInstHnd;
    	*addr = (uint32_t) gPAInstHnd;
        SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gPaL2Handles;
    	*addr = (uint32_t) gPaL2Handles;
        SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gPaL3Handles;
    	*addr = (uint32_t) gPaL3Handles;
        SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

        addr = (uint32_t *) &shObj->globalCfgDone;
    	*addr = TRUE;
    	SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);
}

#define SHM_KEY  100
static int shm_id = 0;
extern void    perror();

/* shared memory create at Master process */
void fw_shmCreate(void)
{
    struct shmid_ds info;
	char * shm;
    shm_id = shmget(SHM_KEY, pa_Example_MC_SHM_SIZE, IPC_CREAT | 0666 );
    if (shm_id < 0) {
        perror("fw_shmCreate: shmget\n");
        APP_exit(-1);
    }
	else {
		shm = shmat(shm_id, 0, 0);
		if (shm == (char *) -1) {
			perror ("fw_shmCreate: shmat\n");
		}
		shObj = (pa_Example_MC_ShObj_t *) shm;
		shmctl(shm_id, IPC_STAT, &info);
	    memset(shObj, 0, pa_Example_MC_SHM_SIZE);
	}
}

int32_t fw_shmDelete(void)
{
    shm_id = shmget(SHM_KEY, 0, 0666 );
    if (shm_id < 0)
    {
        perror("fw_shmDelete: shmget error\n");
        APP_exit(-1);
    }
    else
    {
        int err=shmctl(shm_id, IPC_RMID, 0);
        if(err<0)
        {
            perror("fw_shmDelete: shmctl failed\n");
        }

    }
    return (0);
}


/* Slave Process would open the previously created Shared memory */
void fw_shmOpen(void)
{
    struct shmid_ds info;
    shm_id = shmget(SHM_KEY, 0, 0666 );

	if (shm_id < 0)	{
	   perror ("fw_shmOpen");
	   APP_exit(-1);
	}
	else {
		shmctl(shm_id, IPC_STAT, &info);
	    shObj = (pa_Example_MC_ShObj_t *) shmat(shm_id, 0, 0);
	}
}

void fw_shmClose(void)
{
    /* Remove the references */
	shObj = (pa_Example_MC_ShObj_t *) NULL;
}

/* Linux Semaphore Initialization */
void fw_SemInit(void)
{
	/* create, initialize semaphore */
	if( sem_init(&mutex,1,1) < 0)
	  {
		perror("sem_init:");
		APP_exit(0);
	  }
}

void fw_SemDestroy(void)
{
	if( sem_destroy(&mutex) < 0)
	  {
		perror("sem_destroy");
		APP_exit(0);
	  }

}

Rm_Packet *transportAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rm_pkt = NULL;

    rm_pkt = calloc(1, sizeof(*rm_pkt));
    if (!rm_pkt) {
        printf("can't malloc for RM send message (err: %s)\n", strerror(errno));
        return (NULL);
    }
    rm_pkt->pktLenBytes = pktSize;
    *pktHandle = rm_pkt;

    return(rm_pkt);
}

void transportFree (Rm_Packet *rm_pkt)
{
    if (rm_pkt) {
        free (rm_pkt);
    }
}

void transportReceive (void)
{
    int32_t             rm_result;
    int                 retval;
    int                 length = 0, sz;
    sock_name_t         server_sock_addr;
    Rm_Packet          *rm_pkt = NULL;
    struct sockaddr_un  server_addr;

    retval = sock_wait(rmClientSocket, &length, NULL, -1);
    if (retval == -2) {
        /* Timeout */
        printf("core %d: Error socket timeout\n", coreNum);
        return;
    }
    else if (retval < 0) {
        printf("core %d: Error in reading from socket, error %d\n", coreNum, retval);
        return;
    }
    sz = sizeof(*rm_pkt);
    if (length < sz) {
        printf("core %d: invalid RM message length %d\n", coreNum, length);
        return;
    }
    rm_pkt = calloc(1, length);
    if (!rm_pkt) {
        printf("core %d: can't malloc for recv'd RM message (err: %s)\n",
               coreNum, strerror(errno));
        return;
    }

    server_sock_addr.type = sock_addr_e;
    server_sock_addr.s.addr = &server_addr;
    retval = sock_recv(rmClientSocket, (char *)rm_pkt, length, &server_sock_addr);
    if (retval != length) {
        printf("core %d: recv RM pkt failed from socket, received = %d, expected = %d\n",
               coreNum, retval, length);
        return;
    }

    //printf("core %d: received RM pkt of size %d bytes from %s\n", coreNum, length, server_sock_addr.s.addr->sun_path);

    /* Provide packet to RM Client for processing */
    if ((rm_result = Rm_receivePacket(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle, rm_pkt))) {
        printf("core %d: RM failed to process received packet: %d\n", coreNum, rm_result);
    }

    transportFree(rm_pkt);
}

int32_t transportSendRcv (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    sock_name_t *server_sock_name = (sock_name_t *)appTransport;
    Rm_Packet   *rm_pkt = (Rm_Packet *)pktHandle;

    if (sock_send(rmClientSocket, (char *)rm_pkt, (int) rm_pkt->pktLenBytes, server_sock_name)) {
        printf("core %d: send data failed\n", coreNum);
    }

    /* Wait for response from Server */
    transportReceive();

    return (0);
}

int connection_setup(void)
{
    Rm_TransportCfg rmTransCfg;
    int32_t         rm_result;
    int             i;
    sock_name_t     sock_name;
    char            server_sock_name[] = RM_SERVER_SOCKET_NAME;

    /* Initialize the transport map */
    for (i = 0; i < MAX_MAPPING_ENTRIES; i++) {
        rmTransportMap[i].transportHandle = NULL;
    }

    if (snprintf (rmClientSockName, MAX_CLIENT_SOCK_NAME, "/tmp/var/run/rm/rm_client%d", coreNum) >= MAX_CLIENT_SOCK_NAME)
    {
        printf("error: client socket name truncated\n");
        return -1;
    }
    sock_name.type = sock_name_e;
    sock_name.s.name = rmClientSockName;

    rmClientSocket = sock_open(&sock_name);
    if (!rmClientSocket) {
        printf("connection_setup: Client socket open failed\n");
        return (-1);
    }

    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock = calloc(1, sizeof(sock_name_t));
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->type = sock_name_e;
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->s.name = calloc(1, strlen(server_sock_name)+1);
    strncpy(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->s.name, server_sock_name, strlen(server_sock_name)+1);

    /* Register the Server with the Client instance */
    rmTransCfg.rmHandle = rmClientHandle;
    rmTransCfg.appTransportHandle = (Rm_AppTransportHandle) rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock;
    rmTransCfg.remoteInstType = Rm_instType_SERVER;
    rmTransCfg.transportCallouts.rmAllocPkt = transportAlloc;
    rmTransCfg.transportCallouts.rmSendPkt = transportSendRcv;
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle = Rm_transportRegister(&rmTransCfg, &rm_result);

    return(0);
}

/** ============================================================================
 *   @n@b initRm
 *
 *   @b Description
 *   @n This API initializes the RM Client for the QMSS test establishing
 *      a socket connection with the RM Server
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int initRm (void)
{
    Rm_InitCfg         rmInitCfg;
    int32_t            result;

    /* Initialize the RM Client - RM must be initialized before anything else in the system */
    memset(&rmInitCfg, 0, sizeof(rmInitCfg));
    if (snprintf (rmClientName, RM_NAME_MAX_CHARS, "RM_Client%d", coreNum) >= RM_NAME_MAX_CHARS)
    {
        printf("client name truncated\n");
        return -1;
    }
    rmInitCfg.instName = rmClientName;
    rmInitCfg.instType = Rm_instType_CLIENT;
    rmClientHandle = Rm_init(&rmInitCfg, &result);
    RM_ERROR_CHECK(RM_OK, result, rmClientName, "Initialization failed");

    printf("\n\nInitialized %s\n\n", rmClientName);

    /* Open Client service handle */
    rmClientServiceHandle = Rm_serviceOpenHandle(rmClientHandle, &result);
    RM_ERROR_CHECK(RM_OK, result, rmClientName, "Service handle open failed");

    return(connection_setup());
}


/* Nothing past this point */
