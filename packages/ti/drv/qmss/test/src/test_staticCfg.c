/** @file  test_staticCfg.c
 *
 *   @brief   This is the QMSS unit test code.
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
#include "fw_mem_allocator.h"
#endif

#include <string.h>

#ifdef NSS_LITE
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#endif

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

#ifndef __LINUX_USER_SPACE
/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>
#endif

#ifdef __LINUX_USER_SPACE
void initRm();
#endif

#ifdef NSS_LITE
/* There are only 39 general purpose queues at the NSS Lite devices and therefore we need
 * to reduce the number of static memory regions because each memory region requires one
 * gereral purpose queue
 */
#define QMSS_SC_TEST_MAX_MEM_REGIONS    32
#else
#define QMSS_SC_TEST_MAX_MEM_REGIONS QMSS_MAX_MEM_REGIONS
#endif

/************************ USER DEFINES ************************/
#define NUM_MONOLITHIC_DESC         32
#define SIZE_MONOLITHIC_DESC        16
#define LINKING_RAM_SIZE            NUM_MONOLITHIC_DESC * QMSS_SC_TEST_MAX_MEM_REGIONS

/************************ GLOBAL VARIABLES ********************/

/* Descriptor pool [Size of descriptor * Number of descriptors] */
#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
#pragma DATA_ALIGN (monolithicDesc, 16)
uint8_t  monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC * QMSS_SC_TEST_MAX_MEM_REGIONS];
#else
uint8_t  monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC * QMSS_SC_TEST_MAX_MEM_REGIONS] __attribute__ ((aligned (16)));
#endif
#else
uint8_t *monolithicDesc;
#endif

/* Global variable common to all test cases */

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* Memory region configuration status */
Qmss_MemRegCfg          memRegStatus;
/* QM descriptor configuration */
Qmss_DescCfg            descCfg;
/* Store the queue handle for destination queues on which allocated descriptors are stored */
Qmss_QueueHnd           QueHnd[QMSS_SC_TEST_MAX_MEM_REGIONS];

/************************ EXTERN VARIABLES ********************/

/* Error counter */
extern uint32_t errorCount;
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/**
 *  @b Description
 *  @n
 *      Utility function which converts a local GEM L2 memory address
 *      to global memory address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Computed L2 global Address
 */
#ifndef __LINUX_USER_SPACE
static uint32_t l2_global_address (uint32_t addr)
{
#ifdef _TMS320C6X
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
}
#else
static uint32_t l2_global_address (uint32_t addr)
{
    return addr;
}
#endif


/***************************************************************************************
 * FUNCTION PURPOSE: Power up PA subsystem
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void passPowerUp (void)
{

#ifdef NSS_LITE
    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any 
     * PASS device register access. This not required for the simulator. */

    /* Set NSS Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_NSS);

    /* Enable the clocks for NSS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_NSS, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_NSS);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_NSS));
#endif  
  
}

void testMemoryRegionStaticConfig (void)
{
    Qmss_Result             result, region[QMSS_SC_TEST_MAX_MEM_REGIONS];
    uint32_t                numAllocated, i, corenum, nQueHnds = 0, region_idx = 0;

    /* Get the core number. */
#ifndef __LINUX_USER_SPACE
    passPowerUp();
#ifdef _TMS320C6X
    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING STATIC MEMORY REGION CONFIGURATION ************\n", corenum);

    memset ((void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use the internal Linking RAM.
     * LLD will configure the internal linking RAM address and default size if a value of zero is specified.
     * Linking RAM1 is not used */
#ifndef __LINUX_USER_SPACE
    qmssInitConfig.linkingRAM0Base = 0;
    #ifndef NSS_LITE
    qmssInitConfig.linkingRAM0Size = 0;
    #else
    qmssInitConfig.linkingRAM0Size = LINKING_RAM_SIZE;
    #endif
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = LINKING_RAM_SIZE;

    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", corenum, result);
        errorCount++;
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", corenum, result);
    }
#else
    initRm();
    initQmss((NUM_MONOLITHIC_DESC * QMSS_SC_TEST_MAX_MEM_REGIONS), false);
    /* Allocate memory for descriptor region */
    monolithicDesc = (uint8_t *)fw_memAlloc((SIZE_MONOLITHIC_DESC *
                                             NUM_MONOLITHIC_DESC *
                                             QMSS_SC_TEST_MAX_MEM_REGIONS),
                                            CACHE_LINESZ);
    if (monolithicDesc == NULL) {
        System_printf ("Core %d : Error allocating monolithic descriptor memory\n", corenum);
        return;
    }
#endif

    memset ((void *) monolithicDesc, 0, SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC * QMSS_SC_TEST_MAX_MEM_REGIONS);

    for (i = 0; i < QMSS_SC_TEST_MAX_MEM_REGIONS; i++)
    {
        memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) (monolithicDesc + (i * SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC)));
        memInfo.descSize = SIZE_MONOLITHIC_DESC;
        memInfo.descNum = NUM_MONOLITHIC_DESC;
        memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
        memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
        memInfo.startIndex = 0;

        region[region_idx] = Qmss_insertMemoryRegion (&memInfo);
        if (region[region_idx] < QMSS_SOK)
        {
            if (region[region_idx] == QMSS_RESOURCE_MEM_REGION_INIT_DENIED)
            {
                System_printf ("Core %d : RM denied insert of region %d, presuming done\n", corenum, i);
                break;
            }
            System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", corenum, i, region[region_idx]);
            errorCount++;
        } else
        {
            region_idx ++;
        }
    }

    System_printf ("\n~~~~~~~~~~Core %d Memory region Configuration~~~~~~~~~~~~\n", corenum);
    result = Qmss_getMemoryRegionCfg (&memRegStatus);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : query memory region error code : %d\n", corenum, result);
        errorCount++;
    }

    System_printf ("Current Desc count  : %d\n", memRegStatus.currDescCnt);
    for (i = 0; i < QMSS_SC_TEST_MAX_MEM_REGIONS; i++)
    {
        if (memRegStatus.memRegInfo[i].descBase != 0)
        {
            System_printf ("\nMemory Region Index : %d\n", memRegStatus.memRegInfo[i].memRegion);
            System_printf ("Start Index         : %d\n", memRegStatus.memRegInfo[i].startIndex);
            System_printf ("Descriptor Size     : %d\n", memRegStatus.memRegInfo[i].descSize);
            System_printf ("Descriptor Num      : %d\n", memRegStatus.memRegInfo[i].descNum);
            System_printf ("Descriptor Base     : 0x%p\n", memRegStatus.memRegInfo[i].descBase);
            System_printf ("Managed Descriptor  : %d\n", memRegStatus.memRegInfo[i].manageDescFlag);
        }
    }

    for (i = 0; i < region_idx; i++)
    {
        descCfg.memRegion = (Qmss_MemRegion) region[i];
        descCfg.descNum = NUM_MONOLITHIC_DESC;
        descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
        descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
        
        #ifdef NSS_LITE
        if ( i != 0)
        {
            descCfg.destQueueNum = QMSS_QUEUE_NUMBER(QueHnd[0]);
        }
        #endif
    
        if ((QueHnd[nQueHnds] = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
        {
            System_printf ("Error Core %d : Getting descriptors from memory region %d error code: %d \n", corenum, region[i], QueHnd[nQueHnds]);
            errorCount++;
        }
        else
        {
            nQueHnds++;
            if (descCfg.descNum != numAllocated)
            {
                errorCount++;
            }
            System_printf ("Core %d : Memory region %d Number of descriptors requested : %d. Number of descriptors allocated : %d \n",
                        corenum, region[i], descCfg.descNum, numAllocated);
        }
    }

    /* Clean the queues */
    for (i = 0; i < nQueHnds; i++)
    {
        /* Discard descriptors so region can be safely removed */
        Qmss_queueEmpty (QueHnd[i]);
        /* Close the queue */
#ifdef NSS_LITE
        if ( (result = Qmss_queueClose (QueHnd[i])) < 0)
#else
        if ( (result = Qmss_queueClose (QueHnd[i])) != QMSS_SOK)
#endif
        {
            System_printf ("Error closing queue %d: %d\n", QueHnd[i], result);
            errorCount++;
        }
    }

    for (i = 0; i < region_idx; i++)
    {
        if ( (result = Qmss_removeMemoryRegion (region[i], 0)) != QMSS_SOK)
        {
            System_printf ("Error Core %d : Error freeing region %d error code: %d \n", corenum, region[i], result);
            errorCount++;
        }
    }

    System_printf ("Core %d : exit QM\n", corenum);
    if ( (result = Qmss_exit()) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Core %d : Error exiting QM: %d \n", corenum, result);
    }

    if (errorCount == 0)
    {
        System_printf ("Core %d : covered %d of %d regions (RM blocked %d)\n",
                       corenum, region_idx, QMSS_SC_TEST_MAX_MEM_REGIONS, QMSS_SC_TEST_MAX_MEM_REGIONS - region_idx);
        System_printf ("\nCore %d : Static Memory region configuration tests Passed\n", corenum);
    }
    else
    {
        System_printf ("Core %d test failed with %d errors\n", corenum, errorCount);
    }
}

void run_test (void)
{
    /* Test memory region simple static configuration */
    testMemoryRegionStaticConfig ();
}


