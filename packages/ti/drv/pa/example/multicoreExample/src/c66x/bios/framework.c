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

#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* PASS RL file */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_pa_ss.h>

/* Firmware images */
#include <ti/drv/pa/fw/pafw.h>

/** ============================================================================
 *   @n@b Convert_CoreLocal2GlobalAddr
 *
 *   @b Description
 *   @n This API converts a core local L2 address to a global L2 address.
 *
 *   @param[in]
 *   @n addr            L2 address to be converted to global.
 *
 *   @return    UInt32
 *   @n >0              Global L2 address
 * =============================================================================
 */
UInt32 Convert_CoreLocal2GlobalAddr (UInt32  addr)
{
	uint32_t coreNum;

    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Compute the global address. */
    return ((1 << 28) | (coreNum << 24) | (addr & 0x00ffffff));
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
    /* Compute the local l2 address. */
    return (addr & 0x00ffffff);
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
    } while(!globalCfgDone);
}

/* Huge memory block reserved for shared memory use */
#pragma DATA_ALIGN   (mem_blk_shm, CACHE_LINESZ)
#pragma DATA_SECTION (mem_blk_shm, ".sharedDDR")
uint8_t      mem_blk_shm[pa_Example_MC_SHM_SIZE];

int32_t fw_shmDelete(void)
{
	return 0;
}

void APP_readAllHndles(void)
{
	gTxFreeQHnd  = (Qmss_QueueHnd) fw_shmGetEntry(gTxFreeQHndAddr);
	gRxFreeQHnd  = (Qmss_QueueHnd) fw_shmGetEntry(gRxFreeQHndAddr);
	//gCpdmaHnd    = (Cppi_Handle)   fw_shmGetEntry(gCpdmaHndAddr);
	gPAInstHnd   = (Pa_Handle)     fw_shmGetEntry(gPAInstHndAddr);
    gPaL2Handles = (paHandleL2L3_t)fw_shmGetEntry(gPaL2HandlesAddr);
    gPaL3Handles = (paHandleL2L3_t)fw_shmGetEntry(gPaL3HandlesAddr);
}

uint32_t fw_shmGetEntry(pa_Example_shmStr_e shmstr)
{
	uint32_t addr;

	switch (shmstr)
	{
    case gTxFreeQHndAddr:
        SYS_CACHE_INV ((void *) &shObj->gTxFreeQHnd, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->gTxFreeQHnd;
        break;
    case gRxFreeQHndAddr:
        SYS_CACHE_INV ((void *) &shObj->gRxFreeQHnd, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->gRxFreeQHnd;
        break;
    case gCpdmaHndAddr :
        SYS_CACHE_INV ((void *) &shObj->gCpdmaHnd, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->gCpdmaHnd;
        break;
    case 	gPAInstHndAddr:
        SYS_CACHE_INV ((void *) &shObj->gPAInstHnd, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->gPAInstHnd;
    	break;
    case gPaL2HandlesAddr:
        SYS_CACHE_INV ((void *) &shObj->gPaL2Handles, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->gPaL2Handles;
    	break;
    case gPaL3HandlesAddr:
        SYS_CACHE_INV ((void *) &shObj->gPaL3Handles, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->gPaL3Handles;
    	break;
    case globalCfgDoneAddr:
        SYS_CACHE_INV ((void *) &shObj->globalCfgDone, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->globalCfgDone;
    	break;
    case localCfgDoneAddr:
        SYS_CACHE_INV ((void *) &shObj->localCfgDone, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->localCfgDone;
    	break;
    case localTestDoneAddr:
        SYS_CACHE_INV ((void *) &shObj->localTestDone, 128, CACHE_WAIT);
    	addr = (uint32_t) shObj->localTestDone;
    	break;
    case allPktRxCntAddr:
        SYS_CACHE_INV ((void *) &shObj->allPktRxCnt, 128, CACHE_WAIT);
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
        SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gRxFreeQHnd;
    	*addr = (uint32_t) gRxFreeQHnd;
        SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gCpdmaHnd;
    	*addr = (uint32_t) gCpdmaHnd;
        SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gPAInstHnd;
    	*addr = (uint32_t) gPAInstHnd;
        SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gPaL2Handles;
    	*addr = (uint32_t) gPaL2Handles;
        SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

        addr = (uint32_t *) &shObj->gPaL3Handles;
    	*addr = (uint32_t) gPaL3Handles;
        SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

        addr = (uint32_t *) &shObj->globalCfgDone;
    	*addr = TRUE;
    	SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

}

void fw_shmCreate(void)
{
	uint32_t coreNum;
    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

	if (coreNum == SYSINIT)
	{
	    memset(mem_blk_shm, 0, sizeof (mem_blk_shm));
        SYS_CACHE_WB ((void *) mem_blk_shm, sizeof (mem_blk_shm), CACHE_WAIT);
        shObj = (pa_Example_MC_ShObj_t*) mem_blk_shm;
	}

}

void fw_shmOpen(void)
{
    shObj = (pa_Example_MC_ShObj_t*) mem_blk_shm;
}

void fw_shmClose(void)
{
	shObj = (pa_Example_MC_ShObj_t*) NULL;
}

void APP_publishLocalCfgDone(void)
{
	uint32_t* addr;

    while ((CSL_semAcquireDirect (PA_APP_HW_SEM_SYS)) == 0);

    addr = (uint32_t *) &shObj->localCfgDone;
	*addr += 1;
	SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

    /* Release the hardware semaphore. */
    CSL_semReleaseSemaphore (PA_APP_HW_SEM_SYS);
}

void APP_waitAllLocalCfgDone(void)
{
	uint32_t localCfgDone;

    do {
    	localCfgDone = (uint32_t) fw_shmGetEntry(localCfgDoneAddr);
    } while (localCfgDone != pa_MC_EXAMPLE_NUM_CORES);
}


void APP_publishLocalTestDone(void)
{
	uint32_t* addr;

    while ((CSL_semAcquireDirect (PA_APP_HW_SEM_SYS)) == 0);

    addr = (uint32_t *) &shObj->localTestDone;
	*addr += 1;
	SYS_CACHE_WB ((void *) addr, 128, CACHE_WAIT);

    /* Release the hardware semaphore. */
    CSL_semReleaseSemaphore (PA_APP_HW_SEM_SYS);
}

void APP_waitAllLocalTestDone(void)
{
	uint32_t localCfgDone;

    do {
    	localCfgDone = (uint32_t) fw_shmGetEntry(localTestDoneAddr);
    } while (localCfgDone != pa_MC_EXAMPLE_NUM_CORES);
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
	while ((CSL_semAcquireDirect (PA_APP_HW_SEM_SYS)) == 0);

    addr = (uint32_t *) &shObj->allPktRxCnt;
	*addr += 1;
	SYS_CACHE_WB ((void *) addr, CACHE_LINESZ, CACHE_WAIT);

    /* Release the hardware semaphore. */
    CSL_semReleaseSemaphore (PA_APP_HW_SEM_SYS);

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
#if RM && !defined(__LINUX_USER_SPACE)
    int32_t                 rmResult;
#endif /* RM */

#if RM
		rmClientServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
		if (rmResult != RM_OK)
		{
			System_printf ("Error Core %d : Creating RM service handle error code : %d\n", coreNum, rmResult);
			return -1;
		}
#endif /* RM */
    return 0;

}

/* Nothing past this point */
