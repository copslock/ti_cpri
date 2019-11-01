/**
 *   @file  queueAlloc.c
 *
 *   @brief
 *      This is the Queue Allocation test file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2014, Texas Instruments, Inc.
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

/* XDC includes */
#ifndef __LINUX_USER_SPACE
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <xdc/cfg/global.h>
#else
#include "fw_test.h"
#endif

#include <string.h>

#ifdef NSS_LITE
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#endif

/* sysbios includes */
#ifndef __LINUX_USER_SPACE
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#endif

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

#ifndef __LINUX_USER_SPACE
/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#ifdef _TMS320C6X
#include <ti/csl/csl_cacheAux.h>
#endif
#endif

#ifdef __LINUX_USER_SPACE
void initRm();
#endif

/************************ USER DEFINES ********************/

/* Define to test AIF, FFTC A and FFTC B */
#undef  TEST_ALL

#define SYSINIT                     0

#define NUM_HOST_DESC               32
#define SIZE_HOST_DESC              48
#define NUM_MONOLITHIC_DESC         32
#define SIZE_MONOLITHIC_DESC        160
#define NUM_DATA_BUFFER             32
#define SIZE_DATA_BUFFER            64

#define MAPPED_VIRTUAL_ADDRESS      0x81000000

/* MPAX segment 2 registers */
#define XMPAXL2                     0x08000010
#define XMPAXH2                     0x08000014

/************************ GLOBAL VARIABLES ********************/
#ifndef __LINUX_USER_SPACE
/* linking RAM */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (linkingRAM0, 16)
uint64_t linkingRAM0[NUM_HOST_DESC + NUM_MONOLITHIC_DESC];
#else
uint64_t linkingRAM0[NUM_HOST_DESC + NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));
#endif
#endif

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
#pragma DATA_SECTION (isSysInitialized, ".qmss");
#endif
#endif
volatile uint32_t       isSysInitialized;

/* Queue handles for close */
Qmss_QueueHnd           handles[QMSS_MAX_QUEUES_SS_0];
uint32_t                handle_idx = 0;

const char* qmssQTypeStr[] =
{
    "Low Priority queues",        /* Qmss_QueueType_LOW_PRIORITY_QUEUE */
    "AIF queues",                 /* Qmss_QueueType_AIF_QUEUE */
    "NSS queues",                 /* Qmss_QueueType_PASS_QUEUE */
    "INTC Set 1 queues",          /* Qmss_QueueType_INTC_QUEUE */
    "INTC Set 2 queues",          /* Qmss_QueueType_INTC_SET2_QUEUE */
    "INTC Set 3 queues",          /* Qmss_QueueType_INTC_SET3_QUEUE */
    "INTC Set 4 queues",          /* Qmss_QueueType_INTC_SET4_QUEUE */
    "INTC Set 5 queues",          /* Qmss_QueueType_INTC_SET4_QUEUE */
    "INTC EDMA Set 0 queues",     /* Qmss_QueueType_INTC_EDMA_SET0_QUEUE */
    "INTC EDMA Set 1 queues",     /* Qmss_QueueType_INTC_EDMA_SET1_QUEUE */
    "INTC EDMA Set 2 queues",     /* Qmss_QueueType_INTC_EDMA_SET2_QUEUE */
    "SOC Set 0 queues",           /* Qmss_QueueType_SOC_SET0_QUEUE */
    "SOC Set 1 queues",           /* Qmss_QueueType_SOC_SET1_QUEUE */
    "SRIO queues",                /* Qmss_QueueType_SRIO_QUEUE */
    "FFTC A queues",              /* Qmss_QueueType_FFTC_A_QUEUE */
    "FFTC B queues",              /* Qmss_QueueType_FFTC_B_QUEUE */
    "FFTC C queues",              /* Qmss_QueueType_FFTC_C_QUEUE */
    "FFTC D queues",              /* Qmss_QueueType_FFTC_D_QUEUE */
    "FFTC E queues",              /* Qmss_QueueType_FFTC_E_QUEUE */
    "FFTC F queues",              /* Qmss_QueueType_FFTC_F_QUEUE */
    "BCP queues",                 /* Qmss_QueueType_BCP_QUEUE */
    "High Priority queues",       /* Qmss_QueueType_HIGH_PRIORITY_QUEUE */
    "Starvation Counter queues",  /* Qmss_QueueType_STARVATION_COUNTER_QUEUE */
    "Infrastructure queues",      /* Qmss_QueueType_INFRASTRUCTURE_QUEUE */
    "QM2 Infrastructure queues",  /* Qmss_QueueType_QM2_INFRASTRUCTURE_QUEUE */
    "Traffic Shaping queues",     /* Qmss_QueueType_TRAFFIC_SHAPING_QUEUE */
    "GIC400 queues",              /* Qmss_QueueType_GIC400_QUEUE */
    "EDMA 4 queues",              /* Qmss_QueueType_EDMA_4_QUEUE */
    "Hyperlink Broadcast queues", /* Qmss_QueueType_HLINK_BROADCAST_QUEUE */
    "Hyperlink 0 queues",         /* Qmss_QueueType_HLINK_0_QUEUE */
    "Hyperlink 1 queues",         /* Qmss_QueueType_HLINK_1_QUEUE */
    "XGE queues",                 /* Qmss_QueueType_XGE_QUEUE */
    "DXB queues",                 /* Qmss_QueueType_DXB_QUEUE */
    "IQNET queues",               /* Qmss_QueueType_IQNET_QUEUE */
    "EDMA 0 queues",              /* Qmss_QueueType_EDMA_0_QUEUE */
    "EDMA 1 queues",              /* Qmss_QueueType_EDMA_1_QUEUE */
    "EDMA 2 queues",              /* Qmss_QueueType_EDMA_2_QUEUE */
    "EDMA 3 queues",              /* Qmss_QueueType_EDMA_3_QUEUE */
    "Receive queues",             /* Qmss_QueueType_RECEIVE_QUEUE */
    "General purpose queues",     /* Qmss_QueueType_GENERAL_PURPOSE_QUEUE */
    "reserved 1",
    "reserved 2",
    "reserved 3",
    "reserved 4"
};

/************************ EXTERN VARIABLES ********************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/*************************** FUNCTIONS ************************/

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
#ifndef INTERNAL_LINKING_RAM
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
#endif
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

/**
 *  @b Description
 *  @n
 *      Entry point for the test code.
 *      This is an example code that shows CPPI LLD API usage.
 *
 *      It performs the following
 *          - Initializes the Queue Manager low level driver.
 *          - Initializes the CPPI low level driver.
 *          - Opens the CPPI CPDMA in queue manager
 *          - Initializes descriptors and pushes to free queue
 *          - Opens Rx and Tx channel
 *          - Pushes packet on Tx channel. Diverts the packet to Tx channel.
 *          - Process the Rx packet
 *          - Closes Rx and Tx channel
 *          - Closes all open queues
 *          - Closes CPDMA instance
 *          - Deinitializes CPPI LLD
 *
 *  @retval
 *      Not Applicable.
 */
void queueAllocation (void)
{
    int32_t                 result;
    uint32_t                i;
    uint8_t                 isAllocated;
    Qmss_QueueHnd           txQueHnd;
    uint32_t                corenum ;
    Qmss_QueuePushHnd       DRegPtrData, DRegPtrCtrl;
    extern uint32_t         errorCount;

#ifdef L2_CACHE
    uint32_t                *xmpaxPtr;
#endif

    System_printf ("**************************************************\n");
    System_printf ("************ QMSS Queue Allocate testing**********\n");
    System_printf ("**************************************************\n");

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
    /* Get the core number. */
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);

#else
    corenum = 0;
#endif
#else
    corenum = 0;
#endif

    System_printf ("*******Test running on Core %d *******************\n", corenum);

#ifndef __LINUX_USER_SPACE
    if (corenum == SYSINIT)
    {
        /* Reset the variable to indicate to other cores system init is not yet done */
        isSysInitialized = 0;
#ifdef _TMS320C6X
#ifdef L2_CACHE
        /* Set L2 cache to 512KB */
        CACHE_setL2Size (CACHE_512KCACHE);
#endif
        System_printf ("Core %d : L1D cache size %d. L2 cache size %d.\n", corenum, CACHE_getL1DSize(), CACHE_getL2Size());

#ifdef L2_CACHE
        /* Define an MPAX segment in the virtual (CGEM) address space.
         * Map MSMC physical address to virtual address.
         * Configure all +rwx permissions.
         */

        /* Phy address base: 0x0C00 0000
         * Size: 1MB (0x13 according to encoding)
         * Virtual address: 0x8100 0000
         * Permission: 0xFF
         * MAR used: (0x8100 0000 >> 24) = 129
         */

        /* map using MPAX segment 2 registers */
        xmpaxPtr  = (uint32_t *)(XMPAXH2);
        *xmpaxPtr = ((MAPPED_VIRTUAL_ADDRESS >> 12) << 12) | (0x13);

        xmpaxPtr  = (uint32_t *)(XMPAXL2);
        *xmpaxPtr = ((0x0c000000 >> 12) << 8) | (0xFF);

        /* Enable caching for MAR 129. CSL does not define these MARS. Define a macro */
        CACHE_enableCaching ((MAPPED_VIRTUAL_ADDRESS) >> 24);
#endif
#endif
        memset ((void *) &linkingRAM0, 0, sizeof (linkingRAM0));
        memset ((void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

        /* Set up the linking RAM. Use the internal Linking RAM.
         * LLD will configure the internal linking RAM address and default size if a value of zero is specified.
         * Linking RAM1 is not used */
#ifdef INTERNAL_LINKING_RAM
        qmssInitConfig.linkingRAM0Base = 0;
        #ifndef NSS_LITE
        qmssInitConfig.linkingRAM0Size = 0;
        #else
        qmssInitConfig.linkingRAM0Size = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
        #endif
        qmssInitConfig.linkingRAM1Base = 0;
        qmssInitConfig.maxDescNum      = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
#else
        qmssInitConfig.linkingRAM0Base = l2_global_address ((uint32_t) linkingRAM0);
        qmssInitConfig.linkingRAM0Size = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
        qmssInitConfig.linkingRAM1Base = 0;
        qmssInitConfig.maxDescNum      = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
#endif
    /* Run in split mode since this test case depends on groups explicitly */
    qmssInitConfig.mode            = Qmss_Mode_SPLIT;

        /* Initialize Queue Manager SubSystem */
        result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
        if (result != QMSS_SOK)
        {
            System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", corenum, result);
            return;
        }

        /* Start Queue Manager SubSystem */
        result = Qmss_start ();
        if (result != QMSS_SOK)
        {
            System_printf ("Core %d : Error starting Queue Manager error code : %d\n", corenum, result);
        }
        else
            System_printf ("\nCore %d : QMSS initialization done\n", corenum);

        /* Indicate to other cores system init is done */
        isSysInitialized = 1;

#ifdef _TMS320C6X
        /* Writeback L1D */
        CACHE_wbL1d ((void *) &isSysInitialized, 4, CACHE_WAIT);
#endif
    }
    else
    {
        /* Start Queue Manager SubSystem */
        System_printf ("Core %d : Waiting for QMSS to be initialized...\n\n", corenum);

        /* Synchronize all consumer cores. They must wait for the producer core to finish initialization. */

#ifdef _TMS320C6X
        do{
            CACHE_invL1d ((void *) &isSysInitialized, 4, CACHE_WAIT);
        } while (isSysInitialized == 0);
#endif

        /* Start Queue Manager SubSystem */
        result = Qmss_start ();
        if (result != QMSS_SOK)
        {
            System_printf ("Core %d : Error starting Queue Manager error code : %d\n", corenum, result);
        }
        else
            System_printf ("\nCore %d : QMSS initialization done\n", corenum);
    }
#else
    initRm();
#ifdef INTERNAL_LINKING_RAM
    initQmss((NUM_HOST_DESC + NUM_MONOLITHIC_DESC), true);
#else
    initQmss((NUM_HOST_DESC + NUM_MONOLITHIC_DESC), false);
#endif
#endif

#ifdef TEST_ALL
    /* Only for QM2 */
    if ( (txQueHnd = Qmss_queueOpenInRange (QMSS_QM2_HIGH_PRIORITY_QUEUE_BASE,
                                            QMSS_QM2_HIGH_PRIORITY_QUEUE_BASE +
                                            QMSS_MAX_QM2_HIGH_PRIORITY_QUEUE,
                                            &isAllocated)) < 0)
    {
        System_printf ("Core %d : Error opening QM2_HIGH_PRIORITY : %d \n", corenum, txQueHnd);
        errorCount++;
    }
    if (Qmss_queueClose (txQueHnd) < 0 )
    {
        errorCount++;
        System_printf ("Core %d : Error closing Queue Number : %d \n", corenum, txQueHnd);
    }
    if ( (txQueHnd = Qmss_queueOpenInRange (QMSS_QM2_STARVATION_COUNTER_QUEUE_BASE,
                                            QMSS_QM2_STARVATION_COUNTER_QUEUE_BASE +
                                            QMSS_MAX_QM2_STARVATION_COUNTER_QUEUE,
                                            &isAllocated)) < 0)
    {
        System_printf ("Core %d : Error opening QM2_HIGH_PRIORITY : %d \n", corenum, txQueHnd);
        errorCount++;
    }
    if (Qmss_queueClose (txQueHnd) < 0 )
    {
        errorCount++;
        System_printf ("Core %d : Error closing Queue Number : %d \n", corenum, txQueHnd);
    }
#endif
    /* Get queue to check getQueuePushHandle APIs */
    if ((txQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Core %d : Failed to open queue to test getQueuePushHandle APIs (%d)\n", corenum, txQueHnd);
        errorCount++;
    }

    /* Check the Qmss_getQueuePushHandle APIs */
    DRegPtrData = Qmss_getQueuePushHandle (txQueHnd);
    DRegPtrCtrl = Qmss_getQueuePushHandleCfg (txQueHnd);
    /* Low 20 bits - 16 bits for queue offset, and 4 bits since there are 16 bytes per queue
     * should be same
     */
    System_printf ("Core %d: Queue %d's data reg address: %08x; control reg address: %08x\n",
                   corenum, txQueHnd, (uint32_t)DRegPtrData, (uint32_t)DRegPtrCtrl);
    if ( (((uint32_t) DRegPtrData) & 0xfffff) != (((uint32_t) DRegPtrCtrl) & 0xfffff))
    {
        System_printf ("Core %d : ERROR: data reg addresses mismatch\n", corenum);
        errorCount++;
    }
    /* Close getQueuePushHandle queue */
    if ((result = Qmss_queueClose (txQueHnd) < QMSS_SOK))
    {
        errorCount++;
        System_printf ("Core %d : Error closing Queue Number : %d (%d)\n", corenum, txQueHnd, result);
    }

    System_printf ("Core %d : Comprehensively opening all queues:\n", corenum);
    {
        int group, typeIdx;
        int totQueuesOpened = 0, totQueuesDef = 0;
        Qmss_QueueType qType;
        for (group = 0; group < QMSS_MAX_QMGR_GROUPS; group++)
        {
            for (typeIdx = 0; typeIdx < qmssGblCfgParams.numQueueNum[group]; typeIdx++)
            {
                qType = qmssGblCfgParams.maxQueueNum[group][typeIdx].queueType;
                totQueuesDef += qmssGblCfgParams.maxQueueNum[group][typeIdx].maxNum;

                handle_idx = 0;
                while ( (handles [handle_idx++] = Qmss_queueOpenInGroup (group, qType, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) >= QMSS_SOK);
                handle_idx--;
                totQueuesOpened += handle_idx;
                System_printf ("Core %d: Group: %d: type %d (%s) opened %d queues; closing\n",
                    corenum, group, typeIdx, qmssQTypeStr[(int)qType], handle_idx);
                for (i = 0; i < handle_idx; i++)
                {
                    if ((result = Qmss_queueClose (handles[i])) < QMSS_SOK)
                    {
                        errorCount++;
                        System_printf ("Core %d : Error closing Queue Number : %d (%d)\n", corenum, handles[i], result);
                    }
                }
                /* Try opening one queue (if available) with unspecified type since this has different code path */
                if (handle_idx > 0)
                {
                    Qmss_QueueHnd hnd;
                    int32_t       QID = QMSS_QUEUE_QID(handles[0]);
                    if ((hnd = Qmss_queueOpen ( (Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                               QID, &isAllocated)) < QMSS_SOK)
                    {
                        errorCount++;
                        System_printf ("Core %d : Error opening queue %d with NOT_SPECIFIED type: %d\n",
                                       corenum, QID, hnd);
                    }
                    else if ((result = Qmss_queueClose (hnd)) < QMSS_SOK)
                    {
                        errorCount++;
                        System_printf ("Core %d : Error closing queue %d with NOT_SPECIFIED type: %d\n",
                                        corenum, QID, result);
                    }
                }
            }
        }
        System_printf ("Core %d: opened %d total queues of %d.  RM limits this to less than 100%%\n",
                        corenum, totQueuesOpened, totQueuesDef);
    }

    System_printf ("Core %d : testing Qmss_queueOpenuse\n", corenum);
    /* Get a trial queue */
    if ((txQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Core %d : Error opening trial queue: %d \n", corenum, txQueHnd);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d: trial queue %d\n", corenum, txQueHnd);
    }
    /* Try opening it for use (should pass) */
    if (((handles[0] = Qmss_queueOpenUse (Qmss_getQIDFromHandle(txQueHnd), &isAllocated)) < 0) || (isAllocated != 2))
    {
        System_printf ("Core %d : Error Qmss_queueOpenUse on trial queue: %d use %d\n", corenum, handles[0], isAllocated);
        errorCount++;
    }
    else
    {
        /* Close it twice */
        if (((result = Qmss_queueClose (txQueHnd)) < 0) || ((result = Qmss_queueClose (handles[0])) < 0))
        {
            System_printf ("Core %d : close of trial queue failed (%d)\n", corenum, result);
            errorCount++;
        }

        /* Try opening it again (should fail) */
        if (((handles[0] = Qmss_queueOpenUse (Qmss_getQIDFromHandle(txQueHnd), &isAllocated)) != QMSS_QUEUE_NOT_ALREADY_OPEN))
        {
            System_printf ("Core %d : Error Qmss_queueOpenUse on trial queue worked: %d\n", corenum, handles[0]);
            errorCount++;
            Qmss_queueClose (handles[0]);
        }
    }


    System_printf ("Core %d : exit QM\n", corenum);
    if ( (result = Qmss_exit()) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Core %d : Error exiting QM: %d \n", corenum, result);
    }


    if (errorCount)
    {
        System_printf ("Core %d test failed with %d errors\n", corenum, errorCount);
    }
    else
    {
        System_printf ("*******************************************************\n");
        System_printf ("******* QMSS Queue Allocate testing Done (PASS) *******\n");
        System_printf ("*******************************************************\n");
    }
}

void run_test (void)
{
    queueAllocation ();
}

